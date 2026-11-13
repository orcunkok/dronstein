"""The Lucky loop without Lucky or PX4: a fake Lucky (the same rigid body, moved only by the tick's average force, as
Lucky's motors hold it) against the sim's prediction + world wrench (lucky.anchor)."""
import os

import numpy as np

import physics as ph
from lucky import anchor

PRM = ph.load(os.path.join(os.path.dirname(__file__), "..", "params.toml"))
MODEL, TICK, N = "dsb-sim", 0.02, 5  # Lucky's 50 Hz tick = 5 sim steps
S = PRM[MODEL]
M, J, G = S.mass, np.array([S.ixx, S.iyy, S.izz]), PRM["g"]


def lucky_tick(y, held, floor):
    """Lucky: its body moved by the held force/torque (or resting on its floor, motors off)."""
    for _ in range(N):
        y[ph.A] = 0.0  # no rotors of its own: only the held force moves it
        y, _, _ = ph.step(y, np.zeros(4), False, PRM, MODEL, TICK / N, ext=None if floor else held)
    return y


def sim_tick(y, ext, u, armed):
    """The sim: 5 steps with the world's wrench; the average of its own wrench (what Lucky applies) and the IMU."""
    own = np.zeros(6)
    for _ in range(N):
        own += np.concatenate(ph.wrench(y, PRM, MODEL)) / N
        y, f, _ = ph.step(y, u, armed, PRM, MODEL, TICK / N, ext=ext)
    return y, own, f


def fly(y_lucky, ext, u, armed, floor, ticks):
    y = y_lucky.copy()
    for _ in range(ticks):
        y, ext = anchor(y, ext, TICK, y_lucky[ph.P], y_lucky[ph.V], y_lucky[ph.Q], y_lucky[ph.W], M, J)
        y, own, f = sim_tick(y, ext, u, armed)
        y_lucky = lucky_tick(y_lucky, own, floor)
    return y, ext, f, y_lucky


def test_resting_on_the_floor():
    rest = ph.initial_state(PRM, MODEL)
    y, ext, f, _ = fly(rest, np.array([0, 0, -M * G, 0, 0, 0]), np.zeros(4), False, True, 50)
    assert np.allclose(ext[:3], [0, 0, -M * G], atol=0.05) and np.abs(ext[3:]).max() < 0.01  # the floor holds it
    assert np.allclose(f, [0, 0, -G], atol=0.05) and np.abs(y[ph.V]).max() < 1e-3  # the IMU reads the floor's push


def test_free_air_hover_feels_no_world():
    omega = np.sqrt(M * G / (4 * S.k_t))
    volts, _ = ph.battery(1.0, ph.rotor_power(np.full(4, omega), S), PRM["battery"])
    u = np.full(4, (omega * S.v_ref / volts - S.w_min) / (S.w_max - S.w_min))
    air = ph.initial_state(PRM, MODEL)
    air[2], air[ph.A] = -10.0, omega
    _, ext, _, lucky = fly(air, np.zeros(6), u, True, False, 10)  # 0.2 s: open loop, the aft CoM soon tips it over
    assert np.abs(ext[:3]).max() < 0.02 * M * G and np.abs(ext[3:]).max() < 0.005  # nothing out there
    assert abs(lucky[2] + 10) < 0.05  # Lucky's drone hovers on the sim's forces


def test_drop_lands_on_lucky_floor():
    air = ph.initial_state(PRM, MODEL)
    air[2] -= 1.0
    y, ext, f, lucky = fly(air, np.zeros(6), np.zeros(4), False, True, 150)
    assert abs(lucky[2] - ph.initial_state(PRM, MODEL)[2]) < 0.01  # Lucky's floor stopped it
    assert abs(ext[2] + M * G) < 0.05 * M * G and np.allclose(f, [0, 0, -G], atol=0.5)  # and the sim feels it
