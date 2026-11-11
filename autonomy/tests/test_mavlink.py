#!/bin/sh
"exec" "${0%/*}/../.venv/bin/python" "$0" "$@"  # sh runs autonomy/.venv's Python on this file; to Python it's only a string
__doc__ = """Flies PX4 SITL through link/mavlink.py, no gamepad: arm, take off, LOITER forward, ALT_HOLD, a GUIDED setpoint,
RTL. Each step must happen within its timeout, or it stops there.
Run: autonomy/tests/test_mavlink.py [URL] (default udpout:127.0.0.1:14580)"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from link.mavlink import URL, MavLink
from state import ALT_HOLD, LOITER, MODE_NAMES, RTL, State

link, state = MavLink(sys.argv[1] if len(sys.argv) > 1 else URL), State()
start = time.monotonic()


def log(text):
    n, e, d = state.pos
    print(f"{time.monotonic() - start:6.1f} s  {text:<34} {MODE_NAMES.get(state.mode, state.mode)}"
          f"{' armed' if state.armed else ''}{' landed' if state.landed else ''}  N {n:.2f} E {e:.2f} z {d:.2f}")


def run(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.step(state)
        time.sleep(0.02)


def until(what, done, timeout):
    end = time.monotonic() + timeout
    while not done():
        if time.monotonic() > end:
            log(f"FAIL: {what}")
            sys.exit(1)
        run(0.02)
    log(what)


def command(cmd, value, timeout=5, quiet=False):
    """Request cmd and wait for PX4 to take it; quiet: give up after timeout and return False instead of failing."""
    state.request(cmd, value)
    if not quiet:
        return until(f"{cmd} {value} acked", lambda: state.ack == state.seq, timeout)
    end = time.monotonic() + timeout
    while state.ack != state.seq and time.monotonic() < end:
        run(0.02)
    return False


until("connected", lambda: state.connected and state.landed, 30)
run(1)
ground = state.pos[2]  # PX4's local z on the ground (its origin isn't at ground level)
command("mode", LOITER)
until("armed", lambda: state.armed or command("arm", 1, 1, quiet=True), 15)  # PX4 refuses until it has a position
command("takeoff", 3)
until("airborne at 3 m", lambda: not state.landed and abs(state.pos[2] - ground + 3) < 0.3, 30)
run(2)
north = state.pos[0]
command("mode", LOITER)
state.rc = [1500, 1000, 1500, 1500]  # pitch forward
run(2)
state.rc = [1500] * 4
until("moved north", lambda: state.pos[0] - north > 2 and math.hypot(*state.vel[:2]) < 0.3, 15)
command("mode", ALT_HOLD)
until("ALT_HOLD", lambda: state.mode == ALT_HOLD, 3)
n, e, d = state.pos
state.setpoint = [n, e + 5, d, state.att[2]]
until("GUIDED, 5 m east", lambda: MODE_NAMES.get(state.mode) == "GUIDED" and
      math.hypot(state.pos[0] - n, state.pos[1] - e - 5) < 0.3, 20)
state.setpoint = None
command("mode", RTL)
until("landed and disarmed", lambda: state.landed and not state.armed, 90)
command("gripper", 1)
until("gripper closed", lambda: state.gripper, 1)
print("PASS")
