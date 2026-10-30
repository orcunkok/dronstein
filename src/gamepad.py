"""The Xbox controller as the ground station's sticks and buttons (ControllerBindings.txt): it sets the state's RC
channels and asks the FC for arm, mode, takeoff and gripper, like a GCS. No controller: the sticks rest centred."""
import os
import time
from pathlib import Path

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # no window here: read the pad anyway
os.environ.setdefault("SDL_NO_SIGNAL_HANDLERS", "1")  # leave Ctrl+C to Python
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from pygame._sdl2 import controller

from state import ALT_HOLD, LOITER, RTL

TAKEOFF_ALT = 0.5  # m, Menu's takeoff

AXES = {n: getattr(pygame, f"CONTROLLER_AXIS_{n.upper()}") for n in
        ("leftx", "lefty", "rightx", "righty")} | {
        "lefttrigger": pygame.CONTROLLER_AXIS_TRIGGERLEFT, "righttrigger": pygame.CONTROLLER_AXIS_TRIGGERRIGHT}
BUTTONS = {n: getattr(pygame, f"CONTROLLER_BUTTON_{n.upper().replace('DP', 'DPAD_')}") for n in
           ("a", "b", "x", "y", "back", "start", "guide", "leftshoulder", "rightshoulder", "leftstick", "rightstick",
            "dpup", "dpdown", "dpleft", "dpright")}


def pwm(v):
    return round(1500 + 500 * v)


class Gamepad:
    def __init__(self, path=Path(__file__).with_name("ControllerBindings.txt")):
        self.bind, self.deadzone = {}, 0.15  # action -> (sign, SDL name)
        for raw in path.read_text().splitlines():
            f = raw.split("#")[0].split("=")
            if len(f) != 2:
                continue
            name, value = f[0].strip(), f[1].strip()
            if name == "deadzone":
                self.deadzone = float(value)
            elif value.lstrip("-") in AXES or value in BUTTONS:
                self.bind[name] = (-1 if value.startswith("-") else 1, value.lstrip("-"))
            else:
                print(f"gamepad: '{value}' for {name} is not an axis or a button, ignored")
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
