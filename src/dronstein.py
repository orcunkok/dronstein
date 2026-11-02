#!/usr/bin/env -S sh -c 'exec "$(dirname "$0")/.venv/bin/python" "$0" "$@"'
"""The ground station: flies the X500 with the Xbox controller through one link, to Lucky's sim (default) or to
ArduPilot (--ardu). F picks up a random tennis ball (autonomy.py); F again gives the sticks back to the controller.
Run: ./dronstein.py [--ardu] (it uses src/.venv)"""
import argparse
import os
import re
import select
import shutil
import sys
import termios
import time
import tty

from autonomy import pick_up
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
    p.add_argument("--ardu", action="store_true", help="talk MAVLink to ArduPilot (not implemented yet)")
    args = p.parse_args()
    if args.ardu:
        from ardu_link import ArduLink as Link
    else:
        from lucky.link import LuckyLink as Link
    link, pad, state = Link(), Gamepad(), State()
    auto = None  # pick_up's steps while F has the autonomy on, else None
    saved = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)  # one key at a time, no echo; Ctrl+C still stops
    try:
        while True:
            if key() in ("f", "F"):
                auto = None if auto else pick_up(state)
            if auto is not None:
                state.rc = next(auto, None)  # None: the steps are done, so the controller takes over again
                if state.rc is None:
                    auto = None
            if auto is None:
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
