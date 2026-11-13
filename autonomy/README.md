# autonomy: the ground station

What you fly the drone with, in Python. It talks MAVLink to PX4 like a ground station, so the same code flies PX4 in
simulation now and the real X500 later. The autonomy goes here too.

## Quick start
From the dronstein folder, with PX4 running (see the [main README](../README.md)):
```sh
uv venv autonomy/.venv && uv pip install --python autonomy/.venv/bin/python -r autonomy/requirements.txt   # once

autonomy/src/dronstein.py --mav           # fly with the Xbox controller (plug it in any time)
autonomy/tests/calibrate.py               # or: fly the four calibration dots by itself
```
It shows one status line: the mode, armed or not, height and position. Ctrl+C stops it.

**Start PX4's container first**: PX4 answers only the first program that talks to it, and Docker drops what arrives
before PX4 listens. To start in any order, let PX4 talk first: run the container with `-e API_HOST=host.docker.internal`
and use the URL `udpin:0.0.0.0:14540`.

## Commands and options
| Command | Default | What it does |
|---|---|---|
| `src/dronstein.py --mav [URL]` | `udpout:127.0.0.1:14580` (PX4 SITL's API link) | Fly PX4 with the Xbox controller; F in the terminal runs the autonomy |
| `src/dronstein.py` | | Lucky's own controller instead of PX4: `lucky-sim` branch only |
| `tests/calibrate.py [URL] [ALT]` | `udpout:127.0.0.1:14580`, `2.5` m | Take off to ALT, fly to each blue dot of the calibration square, print the time and how far it settled, RTL |
| `tests/test_mavlink.py [URL]` | `udpout:127.0.0.1:14580` | Scripted check: arm, take off, LOITER forward, ALT_HOLD, a 5 m setpoint, RTL; stops at the first step that times out |

URLs are [pymavlink](https://github.com/ArduPilot/pymavlink) connection strings: `udpout:HOST:PORT` (we talk first),
`udpin:0.0.0.0:PORT` (PX4 talks first), or a serial port like `/dev/ttyAMA0` (the real drone's Pixhawk).

## Controller
| Button | What it does |
|---|---|
| Left stick | Move forward/back and left/right |
| RT / LT | Climb / descend |
| LB / RB | Turn left / right |
| Menu | Arm / disarm, on the ground only |
| B | POSCTL / ALTCTL; from OFFBOARD or RTL, back to POSCTL |
| X | Take off (0.5 m) on the ground; in the air, return home (RTL); again during RTL, back to POSCTL |
| A | Gripper grab / release |

The bindings are `BIND` in `src/gamepad.py`.

## The autonomy (F)
F in the terminal running `dronstein.py` switches it on: it picks up a random tennis ball, flies it to the bin, lets it
go and returns home (RTL); the status line shows AUTO. F again takes the sticks back. It needs the balls' and the bin's
positions, which only Lucky's own link sends (`lucky-sim`); over MAVLink it has none yet.

## How it talks to the drone
The drone's own controller decides what to do (take off, return home, land): PX4, in SITL here and on the real drone.
This program only sends the sticks and asks for things, like you would from a real ground station. `state.py` keeps
ArduPilot's mode numbers; `link/mavlink.py` maps them to PX4's: LOITER is POSCTL, ALT_HOLD ALTCTL, GUIDED OFFBOARD,
RTL AUTO.RTL.

## Code
| File | What it does |
|---|---|
| `src/dronstein.py` | Starts everything: the link, the controller, the status line, F for the autonomy |
| `src/link/mavlink.py` | MAVLink to PX4: sticks as MANUAL_CONTROL, commands, OFFBOARD setpoints; PX4's state back |
| `src/link/luckyLink.py` | Lucky's own controller (`GcsLink.cs`, `lucky-sim` only) |
| `src/state.py` | Everything the ground station and the drone tell each other: sticks, commands, where it is, the balls |
| `src/gamepad.py` | Reads the Xbox controller |
| `src/autonomy.py` | The autonomy (F): pick up a ball, take it to the bin, come home |
| `src/navigation.py` | Position setpoints for the controller |
| `tests/calibrate.py` | The calibration flight |
| `tests/test_mavlink.py` | The scripted MAVLink flight check |
