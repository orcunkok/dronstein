#!/bin/sh
"exec" "${0%/*}/../.venv/bin/python" "$0" "$@"  # sh runs autonomy/.venv's Python on this file; to Python it's only a string
__doc__ = """Calibration flight through link/mavlink.py: take off, fly PX4 to each blue dot of the test square in Lucky's
scene, one after the other, and print how long each took and how far from the dot it settled; then RTL.
Run: autonomy/tests/calibrate.py [URL] [ALT] (default udpout:127.0.0.1:14580, 2.5 m)"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from link.mavlink import URL, MavLink
from state import LOITER, RTL, State

START = (-5.0, 0.0)  # the X500's start in Lucky's MuJoCo frame (x north, y west): PX4's local origin
DOTS = [(14.3, 1.3), (14.3, -6.3), (6.7, -6.3), (6.7, 1.3)]  # the test square's corners, MuJoCo frame
NEAR, STILL = 0.2, 0.2  # m, m/s: arrived

link, state = MavLink(sys.argv[1] if len(sys.argv) > 1 else URL), State()
alt = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5


def run_until(done, timeout):
    """Fly the link until done(); False on timeout."""
    end = time.monotonic() + timeout
    while not done():
        if time.monotonic() > end:
            return False
        link.step(state)
        time.sleep(0.02)
    return True


def command(cmd, value):
    state.request(cmd, value)
    return run_until(lambda: state.ack == state.seq, 2)


run_until(lambda: state.connected and state.landed, 30) or sys.exit("no PX4")
ground = state.pos[2]
command("mode", LOITER)
run_until(lambda: state.armed or command("arm", 1), 15) or sys.exit("PX4 won't arm")
command("takeoff", alt)
run_until(lambda: abs(state.pos[2] - ground + alt) < 0.2, 30) or sys.exit("no takeoff")
for i, (x, y) in enumerate(DOTS):
    n, e = x - START[0], -(y - START[1])
    state.setpoint = [n, e, ground - alt, state.att[2]]
    t0 = time.monotonic()
    there = run_until(lambda: math.hypot(state.pos[0] - n, state.pos[1] - e) < NEAR
                      and math.hypot(*state.vel[:2]) < STILL, 30)
    run_until(lambda: False, 2)  # settle, then measure
    err = math.hypot(state.pos[0] - n, state.pos[1] - e)
    print(f"dot {i + 1} N {n:5.1f} E {e:5.1f}: {'reached' if there else 'NOT reached'} in "
          f"{time.monotonic() - t0 - 2:4.1f} s, settled {err * 100:4.1f} cm off, alt {ground - state.pos[2]:.2f} m")
state.setpoint = None
command("mode", RTL)
run_until(lambda: state.landed and not state.armed, 90)
print("landed" if not state.armed else "still armed")
