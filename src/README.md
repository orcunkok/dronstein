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

**F** (in the terminal running `dronstein.py`) switches the autonomy on: it picks up a random tennis ball. The status
line shows AUTO while it runs. Press F again to take the sticks back. The autonomy ends by itself once the claw has closed
on the ball, and the controller takes over again.

## What's in here
| File | What it does |
|---|---|
| `dronstein.py` | Starts everything. F runs the autonomy. |
| `autonomy.py` | The autonomy (F): picks up a random tennis ball in Lucky, flies over it, lands with the claw on it and closes the claw. The debug ball at the helipad (`TennisBall_01`) is not in the list. |
| `gamepad.py` | Reads the Xbox controller. |
| `state.py` | Everything the ground station and the drone tell each other: sticks, commands (arm, mode, takeoff, gripper), where the drone is, and the balls. |
| `link/luckyLink.py` | Talks to Lucky. |
| `link/arduLink.py` | Will talk to ArduPilot (the real drone). Not built yet. |

## How it talks to the drone
The drone's own controller decides what to do (take off, return home, land). This program only sends the sticks and
asks for things, like you would from a real ground station. In Lucky, that controller is the `X500Flight` script. On the
real drone, it will be ArduPilot.
