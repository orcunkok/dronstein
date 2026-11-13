#!/bin/sh
"exec" "${0%/*}/../.venv/bin/python" "$0" "$@"  # sh runs autonomy/.venv's Python on this file; to Python it's only a string
__doc__ = """The ground station: flies the X500 with the Xbox controller through one link: PX4 over MAVLink (--mav; the one
that flies on px4-dsbSim-lucky) or Lucky's own controller (default, holybro-mvp). F (typed here, or pressed in Lucky's window) picks up a random tennis ball (autonomy.py), arming first if needed; F again
gives the sticks back to the controller.
Run: ./dronstein.py [--mav [URL]] (it uses autonomy/.venv)"""
import argparse
import os
import re
import select
import shutil
import sys
import termios
import time
import tty

from autonomy import mission
from gamepad import Gamepad
from state import MODE_NAMES, State

# ANSI colours for the status line's words, as the HUD shows them: armed green, disarmed red.
GREEN, RED, ORANGE, RESET = "\033[32m", "\033[31m", "\033[38;5;208m", "\033[0m"
COLORS = {"ARMED": GREEN, "DISARMED": RED, "CLOSED": RED, "OPEN": GREEN, "AUTO": GREEN,
          **{m: ORANGE for m in MODE_NAMES.values()}}
WORDS = re.compile(r"\b(" + "|".join(COLORS) + r")\b")


def key():
    """The key typed in this terminal since the last call, or '' (the terminal is in cbreak mode: see main)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return os.read(sys.stdin.fileno(), 1).decode()
    return ""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mav", nargs="?", const="udpout:127.0.0.1:14580", metavar="URL",
                   help="talk MAVLink to PX4 (default URL: %(const)s, SITL's API link)")
    args = p.parse_args()
    if args.mav:
        from link.mavlink import MavLink
        link = MavLink(args.mav)
    else:
        from link.luckyLink import LuckyLink
        link = LuckyLink()
    pad, state = Gamepad(), State()
    auto = None  # mission's steps while F has the autonomy on, else None
    seen = 0     # Lucky's F count last time; it starts again with each Play, so only a rise is a press
    saved = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)  # one key at a time, no echo; Ctrl+C still stops
    try:
        while True:
            typed = key() in ("f", "F")
            pressed = state.pickups > seen
            seen = state.pickups
            if typed or pressed:
                auto = None if auto else mission(state)
            if auto is not None:
                state.rc = [1500] * 4  # autonomy flies by setpoints: the sticks rest
                try:
                    next(auto)
                except StopIteration:
                    auto = None  # the steps are done, so the controller takes over again
            if auto is None:
                state.setpoint = None  # no autonomy, no setpoint: the FC holds where it is
                pad.read(state)
            link.step(state)
            if state.connected:
                n, e, d = state.pos
                line = (f"{'AUTO  ' if auto else ''}{'ARMED' if state.armed else 'DISARMED'} "
                        f"{MODE_NAMES.get(state.mode, state.mode)}  alt {-d:.2f} m  N {n:.2f} m  E {e:.2f} m  "
                        f"gripper {'CLOSED' if state.gripper else 'OPEN'}")
            else:
                line = "waiting for the flight controller..."
            width = shutil.get_terminal_size().columns - 1  # wider would wrap, and \r only goes back to the last row
            line = WORDS.sub(lambda m: COLORS[m[1]] + m[1] + RESET, f"{line[:width]:<{width}}")  # colour after fitting it
            print(f"\r{line}", end="", flush=True)
            time.sleep(0.02)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
