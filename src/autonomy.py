"""Autonomy: picks up a random tennis ball in Lucky. It sets the position setpoints (navigation.py): onto the ball. Once it
has been landed for LAND_HOLD the claw closes. dronstein.py runs it as a mode (F key, armed only). Each step is a
generator: it ticks once per frame, so the steps read straight through."""
import math
import random
import time

from navigation import navigate
from state import LOITER

TAKEOFF_ALT = 2.0   # m, before it flies to the ball
CLAW_BACK = 0.1015  # m: the claw is this far behind the drone's centre (CLAW_POS in tools/x500v2/build_x500v2.py)
LAND_HOLD = 0.2     # s: landed this long, in a row, before the claw closes


def until(state, done):
    """Ticks each frame until done()."""
    while not done():
        yield


def command(state, cmd, value):
    """Sends a command, then waits until the FC has taken it: the link holds only the latest command."""
    state.request(cmd, value)
    yield from until(state, lambda: state.ack == state.seq)


def held(landed, seconds):
    """Ticks until landed() has been true for `seconds` in a row."""
    since = None
    while since is None or time.monotonic() - since < seconds:
        if not landed():
            since = None
        elif since is None:
            since = time.monotonic()
        yield


def pick_up(state):
    """The autonomy: ticks every frame, and ends once the claw has closed on a ball. It starts armed (dronstein.py only
    starts it then): on the ground it takes off, then flies over a random ball and lands on it."""
    yield from until(state, lambda: state.connected and state.balls)
    yield from command(state, "mode", LOITER)
    if state.landed:
        yield from command(state, "takeoff", TAKEOFF_ALT)
        yield from until(state, lambda: -state.pos[2] > TAKEOFF_ALT - 0.2)
    ball = random.choice(state.balls)
    yaw = state.att[2]  # keep the heading it has now
    # The drone's centre goes this far behind the claw's spot, so the claw is over the ball.
    n, e = ball[0] - CLAW_BACK * math.cos(yaw), ball[1] - CLAW_BACK * math.sin(yaw)
    navigate(state, n, e, 0.0, yaw)  # the FC flies there and lands on the ball
    yield from until(state, lambda: state.landed)
    yield from held(lambda: state.landed, LAND_HOLD)
    yield from command(state, "gripper", 1)
    yield from until(state, lambda: state.gripper)
    state.setpoint = None
