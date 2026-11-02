"""Autonomy: picks up a random tennis ball in Lucky. dronstein.py runs it as a mode (F key) and takes the sticks while it
is on. Each step is a generator: it yields the RC channels for one frame, so the steps read straight through."""
import math
import random

from state import LOITER

TAKEOFF_ALT = 2.0   # m, before it flies to the ball
CLAW_BACK = 0.1015  # m: the claw is this far behind the drone's centre (CLAW_POS in tools/x500v2/build_x500v2.py)
GAIN = 0.2          # stick per metre of error to the ball
MAX_STICK = 0.4     # the most an approach stick goes: ~2 m/s
GATE = 0.08         # m: descend only while the claw is this close to the ball
DESCEND = -0.3      # climb stick while the claw is over the ball: a slow descent


def pwm(v):
    return round(1500 + 500 * v)


def clamp(v):
    return max(-MAX_STICK, min(MAX_STICK, v))


def until(state, done, sticks=lambda: (0.0, 0.0, 0.0)):
    """Yields the RC channels each frame until done(); sticks() gives (roll, pitch, climb) for each frame."""
    while not done():
        roll, pitch, climb = sticks()
        yield [pwm(roll), pwm(pitch), pwm(climb), 1500]


def command(state, cmd, value):
    """Sends a command, then holds the sticks centred until the FC has taken it: the link holds only the latest command."""
    state.request(cmd, value)
    yield from until(state, lambda: state.ack == state.seq)


def approach(state, ball):
    """Sticks that fly the claw over the ball, and descend once it is over it."""
    def sticks():
        yaw = state.att[2]
        claw_n = state.pos[0] - CLAW_BACK * math.cos(yaw)
        claw_e = state.pos[1] - CLAW_BACK * math.sin(yaw)
        dn, de = ball[0] - claw_n, ball[1] - claw_e
        forward = dn * math.cos(yaw) + de * math.sin(yaw)
        right = -dn * math.sin(yaw) + de * math.cos(yaw)
        over = math.hypot(dn, de) < GATE
        return clamp(GAIN * right), clamp(-GAIN * forward), DESCEND if over else 0.0
    return sticks


def pick_up(state):
    """The autonomy: yields the RC channels every frame, and ends once the claw has closed on a ball."""
    yield from until(state, lambda: state.connected and state.balls)
    yield from command(state, "mode", LOITER)
    if state.landed:
        yield from command(state, "arm", 1)
        yield from until(state, lambda: state.armed)
        yield from command(state, "takeoff", TAKEOFF_ALT)
        yield from until(state, lambda: -state.pos[2] > TAKEOFF_ALT - 0.2)
    ball = random.choice(state.balls)
    yield from until(state, lambda: state.landed, approach(state, ball))
    yield from command(state, "gripper", 1)
    yield from until(state, lambda: state.gripper)
