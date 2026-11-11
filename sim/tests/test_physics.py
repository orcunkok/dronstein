"""Physics checks: sim/.venv/bin/python -m pytest sim/tests"""
import os

import numpy as np

import physics as ph

PRM = ph.load(os.path.join(os.path.dirname(__file__), "..", "params.toml"))


def airborne(prm, model, a, alt=10.0):
    y = ph.initial_state(prm, model)
    y[2], y[ph.A] = -alt, a
    return y


def rotor_hover(prm, soc=1.0):
    """Analytic hover: rotor speed, and the command that holds it at this charge."""
    g = prm["dsb-sim"]
    omega = np.sqrt(g.mass * prm["g"] / (4 * g.k_t))
    volts, _ = ph.battery(soc, ph.rotor_power(np.full(4, omega), g), prm["battery"])
    return omega, (omega * g.v_ref / volts - g.w_min) / (g.w_max - g.w_min)


def test_hover_sih():
    s = PRM["sih"]
    u = s.mass * PRM["g"] / (4 * s.t_max)
    y = airborne(PRM, "sih", u)
    for _ in range(500):  # 2 s
        y, f, _ = ph.step(y, np.full(4, u), True, PRM, "sih")
    assert np.abs(y[ph.V]).max() < 1e-9 and np.abs(y[ph.W]).max() < 1e-9
    assert np.allclose(f, [0, 0, -PRM["g"]])


def test_hover_dsb():
    omega, u = rotor_hover(PRM)
    assert 0.55 < u < 0.75
    y = airborne(PRM, "dsb-sim", omega)
    for _ in range(500):
        y, f, (volts, amps) = ph.step(y, np.full(4, u), True, PRM, "dsb-sim")
    assert abs(y[5]) < 0.03 and np.abs(y[ph.W]).max() < 1e-9  # only the 2 s of battery drain moves it
    assert 10 < amps < 25 and 15.5 < volts < 16.8


def test_free_fall_then_ground():
    prm = {**PRM, "sih": PRM["sih"]._replace(kdv=0.0)}
    y = airborne(prm, "sih", 0.0, alt=3.0)
    for _ in range(125):  # 0.5 s, still in the air
        y, f, _ = ph.step(y, np.zeros(4), False, prm, "sih")
    assert abs(y[5] - prm["g"] * 0.5) < 1e-9 and np.allclose(f, 0)
    for _ in range(750):  # lands and settles
        y, f, _ = ph.step(y, np.zeros(4), False, prm, "sih")
    rest = ph.initial_state(prm, "sih")
    assert abs(y[2] - rest[2]) < 1e-4 and np.abs(y[ph.V]).max() < 1e-3
    assert np.allclose(f, [0, 0, -prm["g"]], atol=1e-2) and abs(y[6] - 1) < 1e-6


def test_sih_matches_hand_step():
    s = PRM["sih"]
    q = np.array([0.9, 0.1, -0.2, 0.3])
    q /= np.linalg.norm(q)
    R = ph.dcm(q)
    u, v, w = np.array([0.6, 0.5, 0.4, 0.7]), np.array([1.0, 2.0, -0.5]), np.array([0.1, -0.2, 0.3])
    F, M = ph.sih_wrench(v, w, R, u, s)
    # sih.cpp: T_B = (0, 0, -T_MAX * sum u); Mt_B with L_ROLL, L_PITCH, Q_MAX; Fa_E = -KDV v; Ma_B = -KDW w
    assert np.allclose(F, R @ [0, 0, -9.0 * 2.2] - 1.0 * v)
    assert np.allclose(M, [0.177 * 9 * (-0.4) - 0.025 * 0.1, 0.177 * 9 * (-0.2) + 0.025 * 0.2, 0.15 * 0 - 0.025 * 0.3])
    # a whole step from rest in the air: the motor lag (tau 0.05 s) and the resulting climb acceleration
    y = airborne(PRM, "sih", 0.0)
    y, f, _ = ph.step(y, np.ones(4), True, PRM, "sih")
    lag = 1 - np.exp(-0.004 / 0.05)
    assert np.allclose(y[ph.A], lag, rtol=1e-6)
    assert np.allclose(f, [0, 0, (-4 * 9.0 * lag - 1.0 * y[5]) / 2.0], rtol=1e-6)  # thrust + KDV drag


def test_rotor_drag_forward_flight():
    prm = {**PRM, "dsb-sim": PRM["dsb-sim"]._replace(cda=np.zeros(3))}
    g = prm["dsb-sim"]
    omega = np.full(4, rotor_hover(prm)[0])
    F, M = ph.rotor_wrench(np.array([10.0, 0, 0]), np.zeros(3), np.eye(3), omega, g)
    drag = 4 * omega[0] * g.k_drag * 10
    assert abs(F[0] + drag) < 1e-9 and 2.0 < drag < 3.0 and abs(F[1]) < 1e-12  # ~2.4 N backwards at 10 m/s
    # drag at the hubs, 6 cm above the CoM, pitches the nose up; the rolling moment is along -v (gz convention)
    assert abs(M[1] - 0.06 * drag) < 1e-9 and abs(M[0] + 4 * omega[0] * g.k_roll * 10) < 1e-12
    # body drag adds 1/2 rho CdA v^2
    F2, _ = ph.rotor_wrench(np.array([10.0, 0, 0]), np.zeros(3), np.eye(3), omega, PRM["dsb-sim"])
    assert abs(F2[0] - F[0] + 0.5 * 1.225 * 0.03 * 100) < 1e-9


def test_battery_sag_raises_hover_command():
    assert rotor_hover(PRM, 0.3)[1] > rotor_hover(PRM, 1.0)[1] + 0.03
    prm = {**PRM, "battery": PRM["battery"]._replace(capacity_ah=0.05)}  # drains in ~10 s at hover
    omega, u0 = rotor_hover(prm)
    y, i_err, us, volts = airborne(prm, "dsb-sim", omega), 0.0, [], []
    for _ in range(2000):  # 8 s holding 10 m with a PI loop on climb rate
        i_err += (y[2] + 10) * 0.004
        u = u0 + 0.3 * (y[2] + 10) + 0.2 * y[5] + 0.3 * i_err
        y, _, (v, _) = ph.step(y, np.full(4, u), True, prm, "dsb-sim")
        us.append(u)
        volts.append(v)
    assert abs(y[2] + 10) < 0.1
    assert np.mean(us[-250:]) > np.mean(us[:250]) + 0.03 and volts[-1] < volts[0] - 0.8
