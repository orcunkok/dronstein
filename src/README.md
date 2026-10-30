# Ground station (Python)

This is what you fly the drone with. Later the autonomy goes here too. It's written for ArduPilot, so the same code can
fly the real X500 later.

## Set up (once)
Needs [uv](https://docs.astral.sh/uv/). From the repo root:
```
uv venv src/.venv
uv pip install --python src/.venv/bin/python -r src/requirements.txt
```

## Run
```
src/dronstein.py          # fly the drone in Lucky (press Play in Lucky, before or after)
src/dronstein.py --ardu   # fly through ArduPilot (not ready yet)
```
It shows one status line: the mode, armed or not, height and position. Ctrl+C stops it.
Plug the Xbox controller in at any time.

## What's in here
| File | What it does |
|---|---|
| `dronstein.py` | Starts everything. |
| `gamepad.py` | Reads the Xbox controller. |
| `ControllerBindings.txt` | Which button does what. Edit it and start again. |
| `state.py` | Everything the ground station and the drone tell each other: sticks, commands (arm, mode, takeoff, gripper), and where the drone is. |
| `lucky/link.py` | Talks to Lucky. |
| `ardu_link.py` | Will talk to ArduPilot (the real drone). Not built yet. |

## How it talks to the drone
The drone's own controller decides what to do (take off, return home, land). This program only sends the sticks and
asks for things, like you would from a real ground station. In Lucky, that controller is the `X500Flight` script. On the
real drone, it will be ArduPilot.
