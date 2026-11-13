# dronstein
Lucky Robots: a Holybro X500 V2 drone in Lucky Engine.

A Holybro X500 V2 with a 4S battery (1.36 kg) flies in a field of tennis balls with a trash can. Its physics and autopilot gains are parameters, so they can be matched to the real drone (sim2real).

> This is the `lucky-sim` branch: PX4-free, everything runs inside Lucky with its own stand-in flight controller, for
> quick tests, training and debugging. `px4-dsbSim-lucky` (merged into `main`) flies the drone with PX4 instead.

## How to run
First time only, set up Python (needs [uv](https://docs.astral.sh/uv/)):
```
uv venv src/.venv
uv pip install --python src/.venv/bin/python -r src/requirements.txt
```
Then, in any order:
1. Open the `LuckyEngine` project in Lucky Engine and press **Play**.
2. Run `src/dronstein.py`.
3. Fly with the Xbox controller.

## What's what
- **Lucky Engine** (`LuckyEngine/`) is the simulator. It also plays the drone's flight controller: a stand-in
  autopilot with ArduPilot-style modes and gains (`X500Flight.cs`); on the real drone and on `px4-dsbSim-lucky`, PX4.
- **The ground station** (`src/`) is Python. You fly with it, and later the autonomy goes there. See
  [src/README.md](src/README.md).

## Controller
| Button | What it does |
|---|---|
| Left stick | Move forward/back and left/right |
| RT / LT | Climb / descend (2.5 m/s) |
| LB / RB | Turn left / right |
| Menu | Arm / disarm, on the ground only |
| B | Switch LOITER / ALT_HOLD; from GUIDED or RTL, back to LOITER |
| X | Take off to 0.5 m on the ground; in the air, return home (RTL). Press again to take RTL back. |
| A | Gripper grab / release |
| Right stick | Orbit / tilt the camera |
| D-pad up / down | Zoom in / out |
| View | Next camera: orbit the X500, follow it (behind its tail), orbit the gripper |

The buttons are hard-coded: the flying ones in `src/gamepad.py` (`BIND`), the camera ones in `X500Flight.cs` (`m_Bindings`).

## Keyboard
| Key | What it does |
|---|---|
| Q / E | Zoom in / out |
| C | Next camera, same as View on the controller |
| O | Gripper fully open / fully closed |
| Right mouse drag | Rotate the camera |
| R | Reset the scene: the X500, the balls and the trash can go back to where they started |
| Y | In the air: fly to the next corner of the test square (calibration) |

## Flying
B switches between two modes, shown on the screen as MODE:

| Mode | Left stick | Let go |
|---|---|---|
| **LOITER** (default) | Sets a speed, up to 5 m/s | It brakes and holds its spot |
| **ALT_HOLD** | Sets the lean, up to 30° | It levels and coasts until drag stops it, like the real one |

Both hold the altitude.

It starts **disarmed**. Menu arms it (props idle). **RTL** climbs to 3 m, flies back over the spot where it was armed, comes down at 0.5 m/s and disarms when it lands; B or any stick or trigger takes it back. The props spin with their motors' thrust. If the ground station stops, the drone acts as if you let go of the sticks.

**Calibration:** in the air, each press of keyboard Y flies it to the next blue dot of the 7.6 m test square on the grey test mat in the north-east corner ((14.3, 1.3), (14.3, -6.3), (6.7, -6.3), (6.7, 1.3), MuJoCo frame) at its altitude, where it holds. MODE shows **GUIDED** in yellow. B, X or any stick or trigger takes it back.

## On screen
Top right: **CAMERA** (X500 V2, Follow or Gripper), and **MODE** (LOITER, ALT_HOLD, GUIDED for takeoff and calibration, or RTL; ArduPilot's names, as `src/dronstein.py` shows them; yellow while calibrating) with **Armed** (green) or **Disarmed** (red) left of it.

## On the ground
- Checkered ground: each tile is 1 m.
- Tennis balls and a trash can: props you can push around with the drone or drag with the mouse (Force Tool).
- Blue dots: a test square of waypoints, flown with keyboard Y (calibration).
- A tennis ball waits under the claw (the claw starts open): land, close the claw (O, or A), take off with X, and the ball comes with you.

## Tuning it to the real drone
Everything about it is at the top of `LuckyEngine/Assets/Scripts/Client/Source/X500Flight.cs`: the drone (thrust, motor lag, yaw, drag), the motors (`MOT_*`) and the autopilot's gains (ArduPilot's X500v2 frame file) and limits, under ArduPilot's parameter names. Change the drone's values to match logs of the real drone. `tools/x500v2/build_x500v2.py` builds its look and its physics.

## Latest changes
- **Python ground station:** the controller is read by `src/dronstein.py`, which flies the drone in Lucky the way it will fly the real one through ArduPilot.
- **The claw grips:** a physical claw (fingers, servos, collisions) under the payload board, lowered so a landed X500 can close it on a tennis ball and lift it. The X500's own physics file is now turned half a turn, to match its model (payload board, GPS and claw at the tail).
- **Y calibration is back:** in the air, keyboard Y flies the X500 around the blue test square, one corner per press (Calib n/4, yellow).
- **X500 only:** the Skydio X2 with its SO-100 arm, its autonomous picking and the Tuning panel are gone. The controller always flies the X500; LT climbs and RT descends.
- **X500 arming, takeoff and RTL:** Y arms / disarms, Menu takes off (0.5 m) or returns home (RTL), B switches Loiter / Manual; the screen shows Armed / Disarmed next to MODE, and the props spin.
- **X500 Loiter mode:** let go and it stops. Yawing doesn't bob it up and down, it idles on the ground, and its drag is set only by the drag constants in `X500Flight.cs`.
- **Holybro X500 V2 flies:** physics and gains are at the top of `LuckyEngine/Assets/Scripts/Client/Source/X500Flight.cs`.
- **Trash can can be moved:** a free, light (~1.1 kg) aluminium can.
- **New look:** checkered 1 m ground tiles and higher sunlight.
- **Controller:** reconnects on its own if unplugged.
