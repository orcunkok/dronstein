# autonomy: the ground station (Python)

This is what you fly the drone with. Later the autonomy goes here too. It talks MAVLink to PX4, so the same code can
fly the real X500 later.

## Set up (once)
Needs [uv](https://docs.astral.sh/uv/). From the repo root:
```
uv venv autonomy/.venv
uv pip install --python autonomy/.venv/bin/python -r autonomy/requirements.txt
```

## Run
```
autonomy/src/dronstein.py --mav    # fly PX4 over MAVLink (SITL's udpout:127.0.0.1:14580; --mav URL for another)
autonomy/src/dronstein.py          # Lucky's own controller: holybro-mvp only (on this branch PX4 flies)
```
With PX4 in Docker, start the container first: PX4 answers only the first sender it hears, and Docker drops what
arrives before PX4 listens. Or let PX4 send first (start in any order): run the container with `-e API_HOST=host.docker.internal`
and `autonomy/src/dronstein.py --mav udpin:0.0.0.0:14540`.

It shows one status line: the mode, armed or not, height and position. Ctrl+C stops it.
Plug the Xbox controller in at any time.

**F** (in the terminal running `dronstein.py`) switches the autonomy on. It needs the balls' and the bin's positions,
which only Lucky's own link sends (holybro-mvp); over MAVLink it has none yet. With them, it picks up a random tennis ball, flies it to the bin, lets it go and returns home (RTL). The status
line shows AUTO while it runs. Press F again to take the sticks back. The autonomy ends by itself once it has landed back home, and the controller takes over again.

## What's in here
| File (under `src/`) | What it does |
|---|---|
| `dronstein.py` | Starts everything. F runs the autonomy. |
| `autonomy.py` | The autonomy (F): picks up a random tennis ball in Lucky, flies over it, lands with the claw on it and closes the claw; `to_bin` then takes off, flies over the bin (the sim sends its position), waits until it is still, opens the claw and returns home. The debug ball at the helipad (`TennisBall_01`) is not in the list. |
| `gamepad.py` | Reads the Xbox controller. |
| `state.py` | Everything the ground station and the drone tell each other: sticks, commands (arm, mode, takeoff, gripper), where the drone is, and the balls. |
| `link/luckyLink.py` | Talks to Lucky's own controller (`GcsLink.cs`, holybro-mvp only). |
| `link/mavlink.py` | Talks MAVLink to PX4 (SITL now, the real X500 later). It maps `state.py`'s ArduPilot modes to PX4's: LOITER is POSCTL, ALT_HOLD ALTCTL, GUIDED OFFBOARD, RTL AUTO.RTL. |

Tests are under `tests/`: `tests/test_mavlink.py [URL]` flies PX4 SITL through `link/mavlink.py` (arm, take off,
LOITER, ALT_HOLD, a GUIDED setpoint, RTL); `tests/calibrate.py [URL] [ALT]` flies PX4 to the four blue dots of the
calibration square in Lucky and prints how close it settled.

## How it talks to the drone
The drone's own controller decides what to do (take off, return home, land). This program only sends the sticks and
asks for things, like you would from a real ground station. That controller is PX4 (`--mav`), in SITL here and on the
real drone; on holybro-mvp it is Lucky's `X500Flight` script.
