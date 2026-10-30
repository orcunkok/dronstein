#!/usr/bin/env -S sh -c 'exec "$(dirname "$0")/.venv/bin/python" "$0" "$@"'
"""The ground station: flies the X500 with the Xbox controller through one link, to Lucky's sim (default) or to
ArduPilot (--ardu). Run: ./dronstein.py [--ardu] (it uses src/.venv)"""
import argparse
import time

from gamepad import Gamepad
from state import MODE_NAMES, State


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ardu", action="store_true", help="talk MAVLink to ArduPilot (not implemented yet)")
    args = p.parse_args()
    if args.ardu:
        from ardu_link import ArduLink as Link
    else:
        from lucky.link import LuckyLink as Link
    link, pad, state = Link(), Gamepad(), State()
    while True:
        pad.read(state)
        link.step(state)
        if state.connected:
            n, e, d = state.pos
            line = (f"{MODE_NAMES.get(state.mode, state.mode)} {'armed' if state.armed else 'disarmed'} "
                    f"{'landed' if state.landed else 'flying'} alt {-d:.2f} m  N {n:.2f} E {e:.2f}  "
                    f"gripper {'closed' if state.gripper else 'open'}")
        else:
            line = "waiting for the flight controller..."
        print(f"\r{line:<90}", end="", flush=True)
        time.sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
