"""The one state the ground station shares with the flight controller, in ArduPilot's terms, whichever link carries it
(lucky/link.py to Lucky's sim, ardu_link.py to ArduPilot)."""
import time
from dataclasses import dataclass, field

# ArduCopter's mode numbers
ALT_HOLD, GUIDED, LOITER, RTL = 2, 4, 5, 6
MODE_NAMES = {ALT_HOLD: "ALT_HOLD", GUIDED: "GUIDED", LOITER: "LOITER", RTL: "RTL"}


@dataclass
class State:
    # To the FC. RC1-4 in PWM, 1500 centred: roll (right high), pitch (forward low), throttle (climb high, 1500 holds
    # the altitude), yaw (right high).
    rc: list = field(default_factory=lambda: [1500] * 4)
    # The latest command: "arm" (1 / 0), "mode" (a number above), "takeoff" (height, m), "gripper" (1 grab, 0 release).
    # The FC runs it once, when seq is new; seq starts new each run, so it never takes an old run's command for this one's.
    cmd: str = ""
    value: float = 0.0
    seq: int = field(default_factory=lambda: int(time.time()))

    # From the FC
    connected: bool = False
    armed: bool = False
    mode: int = LOITER
    landed: bool = True
    pos: tuple = (0.0, 0.0, 0.0)  # m, NED
    vel: tuple = (0.0, 0.0, 0.0)  # m/s, NED
    att: tuple = (0.0, 0.0, 0.0)  # rad: roll, pitch, yaw
    gripper: bool = False         # grabbed (closed)
    ack: int = 0                  # the seq of the last command the FC has taken

    def request(self, cmd, value):
        self.cmd, self.value, self.seq = cmd, float(value), self.seq + 1
