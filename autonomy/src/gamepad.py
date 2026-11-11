"""The Xbox controller as the ground station's sticks and buttons (the bindings below): it sets the state's RC
channels and asks the FC for arm, mode, takeoff and gripper, like a GCS. No controller: the sticks rest centred."""
import os
import time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window here: read the pad anyway
os.environ.setdefault("SDL_NO_SIGNAL_HANDLERS", "1")  # leave Ctrl+C to Python
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from pygame._sdl2 import controller

from state import ALT_HOLD, LOITER, RTL

TAKEOFF_ALT = 0.5  # m, X's takeoff

# The flying bindings: action -> (sign, SDL name). Names are SDL's, the same for any Xbox pad; a sign of -1 flips an axis.
# Where an action is on/off (climb, descend, turn, the buttons), an axis counts when pushed past halfway.
BIND = {
    "roll": (1, "leftx"),            # left stick left/right
    "pitch": (1, "lefty"),           # left stick up/down: up flies forward (low PWM, as ArduPilot's RC2)
    "climb": (1, "righttrigger"),    # RT
    "descend": (1, "lefttrigger"),   # LT
    "yaw_left": (1, "leftshoulder"),   # LB
    "yaw_right": (1, "rightshoulder"),  # RB
    "arm": (1, "start"),             # Menu: arm / disarm (the FC allows it on the ground only)
    "mode": (1, "b"),                # B: LOITER <-> ALT_HOLD; from RTL or GUIDED: LOITER
    "takeoff_rtl": (1, "x"),         # X: take off when landed, RTL in the air, LOITER again during RTL
    "gripper": (1, "a"),             # A: grab / release
}
DEADZONE = 0.15  # stick values below this count as 0

AXES = {n: getattr(pygame, f"CONTROLLER_AXIS_{n.upper()}") for n in
        ("leftx", "lefty", "rightx", "righty")} | {
        "lefttrigger": pygame.CONTROLLER_AXIS_TRIGGERLEFT, "righttrigger": pygame.CONTROLLER_AXIS_TRIGGERRIGHT}
BUTTONS = {n: getattr(pygame, f"CONTROLLER_BUTTON_{n.upper().replace('DP', 'DPAD_')}") for n in
           ("a", "b", "x", "y", "back", "start", "guide", "leftshoulder", "rightshoulder", "leftstick", "rightstick",
            "dpup", "dpdown", "dpleft", "dpright")}


def pwm(v):
    return round(1500 + 500 * v)


class Gamepad:
    def __init__(self):
        self.bind, self.deadzone = BIND, DEADZONE
        pygame.init()
        controller.init()
        self.pad, self.scan, self.down = None, 0.0, set()

    def axis(self, name):
        sign, key = self.bind.get(name, (1, None))
        if key not in AXES:
            return 0.0
        v = sign * self.pad.get_axis(AXES[key]) / 32767
        return 0.0 if abs(v) < self.deadzone else max(-1.0, min(1.0, v))

    def held(self, name):
        key = self.bind.get(name, (1, None))[1]
        return self.axis(name) > 0.5 if key in AXES else key in BUTTONS and self.pad.get_button(BUTTONS[key])

    def pressed(self, name):
        down = self.held(name)
        was = name in self.down
        (self.down.add if down else self.down.discard)(name)
        return down and not was

    def read(self, state):
        pygame.event.pump()
        if self.pad and not self.pad.attached():
            self.pad = None
        if self.pad is None and time.monotonic() > self.scan:
            self.scan = time.monotonic() + 1.0
            if controller.get_count():
                self.pad = controller.Controller(0)
                print(f"\ngamepad: {self.pad.name}")
        if self.pad is None:
            state.rc = [1500] * 4
            return
        state.rc = [pwm(self.axis("roll")), pwm(self.axis("pitch")),
                    pwm(self.held("climb") - self.held("descend")), pwm(self.held("yaw_right") - self.held("yaw_left"))]
        if self.pressed("arm"):
            state.request("arm", 0 if state.armed else 1)
        if self.pressed("mode"):
            state.request("mode", ALT_HOLD if state.mode == LOITER else LOITER)
        if self.pressed("takeoff_rtl"):
            if state.landed:
                state.request("takeoff", TAKEOFF_ALT)
            else:
                state.request("mode", LOITER if state.mode == RTL else RTL)
        if self.pressed("gripper"):
            state.request("gripper", 0 if state.gripper else 1)
