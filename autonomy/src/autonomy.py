"""Autonomy: picks up a random tennis ball in Lucky (pick_up), flies it to the bin and lets it go (to_bin), then returns
home (RTL). It sets the position setpoints (navigation.py): onto the ball, over the bin. Once it has been landed for
LAND_HOLD the claw closes. dronstein.py runs it as a mode (F key). Each step is a
generator: it ticks once per frame, so the steps read straight through."""
import math
import random
import time

from navigation import navigate
from state import LOITER, RTL

TAKEOFF_ALT = 1.0   # m, before it flies to the ball, and to the bin with it
CLAW_BACK = 0.1015  # m: the claw is this far behind the drone's centre (CLAW_POS in tools/x500v2/build_x500v2.py)
CLOSE_TIME = 1.0   # s: the claw has been closed this long before it takes off, so it holds the ball
SETTLE_TIME = 1.0  # s: over the bin and still this long before the claw opens, so the ball drops and isn't thrown
DROP_TIME = 1.0    # s: the claw has been open this long before it returns home, so the ball has fallen
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
    """The autonomy: ticks every frame, and ends once the claw has closed on a ball. On the ground it arms (if it isn't)
    and takes off, then flies over a random ball and lands on it."""
    yield from until(state, lambda: state.connected and state.balls)
    if not state.armed:
        yield from command(state, "arm", 1)
    yield from command(state, "mode", LOITER)
    if state.landed:
        yield from command(state, "takeoff", TAKEOFF_ALT)
        yield from until(state, lambda: -state.pos[2] > TAKEOFF_ALT - 0.2)
    ball = random.choice(state.balls)
    yaw = state.att[2]  # keep the heading it has now
    # The claw is this far behind the drone's centre, so the centre goes this far in front of the ball: the claw is over it.
    n, e = ball[0] + CLAW_BACK * math.cos(yaw), ball[1] + CLAW_BACK * math.sin(yaw)
    navigate(state, n, e, state.pos[2], yaw)  # over the ball, at this height
    yield from until(state, lambda: math.hypot(state.pos[0] - n, state.pos[1] - e) < 0.02)
    navigate(state, n, e, 0.0, yaw)  # then down onto it
    yield from until(state, lambda: state.landed)
    yield from held(lambda: state.landed, LAND_HOLD)
    yield from command(state, "gripper", 1)
    yield from until(state, lambda: state.gripper)
    state.setpoint = None


def to_bin(state):
    """After pick_up: takes off with the ball, flies over the bin, opens the claw, then returns home (RTL) and lands."""
    yield from held(lambda: state.gripper, CLOSE_TIME)
    yield from command(state, "mode", LOITER)
    if state.landed:
        yield from command(state, "takeoff", TAKEOFF_ALT)
        yield from until(state, lambda: -state.pos[2] > TAKEOFF_ALT - 0.2)
    yaw = state.att[2]
    bin = state.bin  # the bin can be pushed: read where it is now
    n, e = bin[0] + CLAW_BACK * math.cos(yaw), bin[1] + CLAW_BACK * math.sin(yaw)
    navigate(state, n, e, state.pos[2], yaw)
    yield from held(lambda: math.hypot(state.pos[0] - n, state.pos[1] - e) < 0.1 and math.hypot(*state.vel) < 0.1, SETTLE_TIME)
    yield from command(state, "gripper", 0)
    yield from held(lambda: not state.gripper, DROP_TIME)
    state.setpoint = None
    yield from command(state, "mode", RTL)
    yield from until(state, lambda: not state.armed)  # RTL lands and disarms


def mission(state):
    yield from pick_up(state)
    yield from to_bin(state)
