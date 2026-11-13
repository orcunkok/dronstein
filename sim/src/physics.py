"""6-DOF multirotor physics in NED (world) / FRD (body): rigid body, ground contact, and two force models:
"sih" (PX4's SIH quadcopter equations) and "dsb-sim" (ours: a rotor model per motor, after Gazebo's
MulticopterMotorModel with PX4's x500 constants, plus body drag and the battery).

The state is one vector y = [p(3) v(3) q(4) w(3) a(4) soc]: position and velocity NED (m, m/s), attitude quaternion
body->NED (w, x, y, z), body rates FRD (rad/s), actuator state (sih: lagged command 0..1, dsb-sim: rotor speed rad/s) and
battery state of charge. The math is compiled with numba (plain loops: numpy is slow on 3-vectors); the parameters
are namedtuples, one per params.toml section."""
from collections import namedtuple

import numpy as np
from numba import njit

P, V, Q, W, A, SOC = slice(0, 3), slice(3, 6), slice(6, 10), slice(10, 13), slice(13, 17), 17
# The params.toml sections the compiled code reads, as module-level types (numba's cache finds them by name).
Sih = namedtuple("Sih", "mass ixx iyy izz l_roll l_pitch t_max q_max kdv kdw t_tau")
Dsb = namedtuple("Dsb", "mass ixx iyy izz rotor_pos spin k_t k_m tau_up tau_down k_drag k_roll w_min w_max v_ref "
                        "eta rho cda")
Battery = namedtuple("Battery", "cells capacity_ah soc0 r_int p_avionics soc ocv")
Ground = namedtuple("Ground", "feet k c mu v_slip")
SECTIONS = {"sih": Sih, "dsb-sim": Dsb, "battery": Battery, "ground": Ground}


@njit(cache=True)
def gravity(lat_deg):
    """Somigliana normal gravity (m/s^2), as SIH uses."""
    s2 = np.sin(np.radians(lat_deg)) ** 2
    return 9.7803253359 * (1 + 0.00193185265241 * s2) / np.sqrt(1 - 0.00669437999013 * s2)


@njit(cache=True)
def dcm(q):
    """Body->NED rotation matrix of a quaternion (w, x, y, z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@njit(cache=True)
def cross(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


@njit(cache=True)
def rot(R, v):
    """R v."""
    return np.array([R[0, 0] * v[0] + R[0, 1] * v[1] + R[0, 2] * v[2],
                     R[1, 0] * v[0] + R[1, 1] * v[1] + R[1, 2] * v[2],
                     R[2, 0] * v[0] + R[2, 1] * v[1] + R[2, 2] * v[2]])


@njit(cache=True)
def rot_t(R, v):
    """R^T v."""
    return np.array([R[0, 0] * v[0] + R[1, 0] * v[1] + R[2, 0] * v[2],
                     R[0, 1] * v[0] + R[1, 1] * v[1] + R[2, 1] * v[2],
                     R[0, 2] * v[0] + R[1, 2] * v[1] + R[2, 2] * v[2]])


@njit(cache=True)
def sih_wrench(v, w, R, u, s):
    """PX4 SIH quadcopter (sih.cpp, generate_force_and_torques): force NED without gravity (N), moment FRD (Nm)."""
    T = s.t_max * (u[0] + u[1] + u[2] + u[3])
    F = rot(R, np.array([0.0, 0.0, -T])) - s.kdv * v
    M = np.array([s.l_roll * s.t_max * (-u[0] + u[1] + u[2] - u[3]),
                  s.l_pitch * s.t_max * (u[0] - u[1] + u[2] - u[3]),
                  s.q_max * (u[0] + u[1] - u[2] - u[3])]) - s.kdw * w
    return F, M


@njit(cache=True)
def rotor_wrench(v, w, R, omega, g):
    """dsb-sim: each rotor's thrust kT w^2, drag torque kM T, rotor drag and rolling moment against the air velocity
    across the rotor (Gazebo's MulticopterMotorModel), plus quadratic body drag. Force NED without gravity, moment FRD."""
    v_b = rot_t(R, v)
    F, M = np.zeros(3), np.zeros(3)
    for i in range(4):
        r = g.rotor_pos[i]                       # hub from the CoM, FRD
        v_r = v_b + cross(w, r)                  # the hub's air velocity (no wind); its x, y are across the rotor axis
        T, a = g.k_t * omega[i] ** 2, abs(omega[i])
        F_r = np.array([-a * g.k_drag * v_r[0], -a * g.k_drag * v_r[1], -T])
        F += F_r
        M += cross(r, F_r)
        M[0] -= a * g.k_roll * v_r[0]
        M[1] -= a * g.k_roll * v_r[1]
        M[2] += g.spin[i] * g.k_m * T
    F -= 0.5 * g.rho * g.cda * np.abs(v_b) * v_b
    return rot(R, F), M


@njit(cache=True)
def rotor_power(omega, g):
    """Electrical power the rotors draw (W): shaft power kM*T*w over the motor+ESC efficiency."""
    return g.k_m * g.k_t * np.sum(np.abs(omega) ** 3) / g.eta


@njit(cache=True)
def battery(soc, power, b):
    """Loaded pack voltage (V) and current (A) at a power draw (W): OCV(SoC) behind an internal resistance."""
    E = b.cells * np.interp(soc, b.soc, b.ocv)
    P = power + b.p_avionics
    V = 0.5 * (E + np.sqrt(max(E * E - 4 * P * b.r_int, 0.0)))
    return V, P / V


@njit(cache=True)
def ground(p, v, R, w, c):
    """Spring-damper feet on the z = 0 plane with smoothed Coulomb friction: force NED, moment FRD."""
    F, M = np.zeros(3), np.zeros(3)
    for i in range(c.feet.shape[0]):
        r = c.feet[i]
        p_f, v_f = p + rot(R, r), v + rot(R, cross(w, r))
        if p_f[2] <= 0:
            continue
        Fn = max(c.k * p_f[2] + c.c * v_f[2], 0.0)
        k = -c.mu * Fn / np.sqrt(v_f[0] ** 2 + v_f[1] ** 2 + c.v_slip ** 2)
        F_f = np.array([k * v_f[0], k * v_f[1], -Fn])
        F += F_f
        M += cross(r, rot_t(R, F_f))
    return F, M


@njit(cache=True)
def deriv(y, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g):
    """dy/dt, the specific force NED (non-gravity force / mass) and the battery (V, A). With world, ext (force NED,
    moment FRD) is everything the world does to the drone and replaces the sim's own ground."""
    p, v, q, w, a = y[0:3], y[3:6], y[6:10], y[10:13], y[13:17]
    R = dcm(q)
    dy = np.empty(18)
    if dsb:
        s = dsb_p
        F, M = rotor_wrench(v, w, R, a, s)
        volts, amps = battery(y[17], rotor_power(a, s), bat_p)
        for i in range(4):  # ESC: 0..1 maps to w_min..w_max rad/s (as PX4's gz bridge), scaled by the pack voltage
            ref = (s.w_min + min(max(u[i], 0.0), 1.0) * (s.w_max - s.w_min)) * volts / s.v_ref if armed else 0.0
            dy[13 + i] = (ref - a[i]) / (s.tau_up if ref > a[i] else s.tau_down)
        dy[17] = -amps / (3600 * bat_p.capacity_ah)
        mass, J = s.mass, np.array([s.ixx, s.iyy, s.izz])
    else:
        F, M = sih_wrench(v, w, R, a, sih_p)
        volts, amps = 0.0, 0.0
        for i in range(4):
            dy[13 + i] = ((u[i] if armed else 0.0) - a[i]) / sih_p.t_tau
        dy[17] = 0.0
        mass, J = sih_p.mass, np.array([sih_p.ixx, sih_p.iyy, sih_p.izz])
    Fg, Mg = (ext[:3], ext[3:]) if world else ground(p, v, R, w, gnd_p)
    f = (F + Fg) / mass
    dy[0:3] = v
    dy[3:6] = f
    dy[5] += g
    dy[6] = 0.5 * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2])
    dy[7] = 0.5 * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1])
    dy[8] = 0.5 * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0])
    dy[9] = 0.5 * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0])
    dy[10:13] = (M + Mg - cross(w, J * w)) / J
    return dy, f, volts, amps


@njit(cache=True)
def rk4(y, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g, dt, n):
    """Advance dt with n RK4 substeps (u held); the specific force and battery at the end."""
    h = dt / n
    for _ in range(n):
        k1 = deriv(y, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g)[0]
        k2 = deriv(y + 0.5 * h * k1, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g)[0]
        k3 = deriv(y + 0.5 * h * k2, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g)[0]
        k4 = deriv(y + h * k3, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g)[0]
        y = y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        y[6:10] /= np.sqrt(np.sum(y[6:10] ** 2))
    _, f, volts, amps = deriv(y, u, armed, ext, world, dsb, sih_p, dsb_p, bat_p, gnd_p, g)
    return y, f, volts, amps


def step(y, u, armed, prm, model, dt=0.004, n=4, ext=None):
    """Advance dt with n RK4 substeps (u held). Returns y, the specific force NED and the battery (V, A) at the end.
    ext: the world's force NED and moment FRD on the drone (Lucky); None uses the sim's own ground."""
    y, f, volts, amps = rk4(y, np.asarray(u, float), bool(armed), *_args(prm, model, ext), dt, n)
    return y, f, (volts, amps)


def specific_force(y, u, armed, prm, model, ext=None):
    """The specific force NED (m/s^2) the accelerometer feels in state y."""
    return deriv(y, np.asarray(u, float), bool(armed), *_args(prm, model, ext))[1]


def wrench(y, prm, model):
    """The drone's own force NED and moment FRD in state y (rotors and drag; no gravity, no world)."""
    R = dcm(y[Q])
    if model == "dsb-sim":
        return rotor_wrench(y[V], y[W], R, y[A], prm["dsb-sim"])
    return sih_wrench(y[V], y[W], R, y[A], prm["sih"])


def _args(prm, model, ext):
    """What the compiled code takes after (y, u, armed): the world wrench, the model flag and the parameters."""
    return (np.zeros(6) if ext is None else np.asarray(ext, float), ext is not None, model == "dsb-sim", prm["sih"],
            prm["dsb-sim"], prm["battery"], prm["ground"], prm["g"])


def initial_state(prm, model):
    """At rest on the ground, level, facing north, motors stopped, battery full."""
    c = prm["ground"]
    y = np.zeros(18)
    y[2] = -c.feet[0, 2] + prm[model].mass * prm["g"] / (len(c.feet) * c.k)
    y[6] = 1.0
    y[SOC] = prm["battery"].soc0
    return y


def section(name, sec):
    """A params.toml section as its namedtuple of floats and float arrays (one type each, so numba compiles once)."""
    sec = {k: np.asarray(v, float) if isinstance(v, list) else float(v) for k, v in sec.items()}
    if name == "dsb-sim":
        sec["rotor_pos"] = np.ascontiguousarray(np.stack([sec.pop("rotor_px"), sec.pop("rotor_py"),
                                                          sec.pop("rotor_pz")], -1))
    elif name == "ground":
        sec["feet"] = np.ascontiguousarray(np.stack([sec.pop("feet_x"), sec.pop("feet_y"), sec.pop("feet_z")], -1))
    return SECTIONS[name](**sec)


def load(path):
    """params.toml -> dict: the physics sections as namedtuples, the others (home, sensors) as dicts, and gravity."""
    import tomllib
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    prm = {k: section(k, v) if k in SECTIONS else v for k, v in raw.items()}
    prm["g"] = float(gravity(prm["home"]["lat"]))
    return prm
