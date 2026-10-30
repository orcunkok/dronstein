# dronstein
Lucky Robots: a Holybro X500 V2 drone in Lucky Engine.

A Holybro X500 V2 with a 4S battery (1.36 kg) flies like a stock ArduPilot setup, in a field of golf balls with a trash can. Its physics and autopilot gains are parameters, so they can be matched to the real drone (sim2real).

## How to run
1. Open the `LuckyEngine` project in Lucky Engine.
2. Press **Play**.
3. Fly with an Xbox controller. A few extra keys are on the keyboard.

## Controller
| Button | What it does |
|---|---|
| Left stick | Move forward/back and left/right |
| LT / RT | Climb / descend (2.5 m/s) |
| LB / RB | Turn left / right |
| Y | Arm / disarm, on the ground only |
| B | Switch Loiter / Manual (also takes RTL back) |
| Share | Take off to 0.5 m on the ground; in the air, return home (RTL). Press again to take RTL back. (A is the same button as Share on this controller.) |
| Right stick | Orbit the camera |
| D-pad up / down | Zoom in / out |
| X | Next camera: orbit the X500, follow it (behind its tail), orbit the gripper |

To change buttons, edit `LuckyEngine/ControllerBindings.txt`, save it, and press Play again.

## Keyboard
| Key | What it does |
|---|---|
| Q / E | Zoom in / out |
| C | Next camera, same as X on the controller |
| O | Gripper fully open / fully closed |
| Right mouse drag | Rotate the camera |
| R | Reset the scene: the X500, the balls and the trash can go back to where they started |
| Y | In the air: fly to the next corner of the test square (calibration) |

## Flying
B switches between two modes, shown on the screen as MODE:

| Mode | Left stick | Let go |
|---|---|---|
| **Loiter** (default) | Sets a speed, up to 5 m/s | It brakes and holds its spot |
| **Manual** | Sets the lean, up to 30° | It levels and coasts until drag stops it, like the real one |

Both hold the altitude.

It starts **disarmed**. Y arms it (props idle). **RTL** climbs to 3 m, flies back over the spot where it was armed, comes down at 0.5 m/s and disarms when it lands; B or any stick or trigger takes it back. The props spin with their motors' thrust.

**Calibration:** in the air, each press of keyboard Y flies it to the next blue dot of the 20 m test square ((10, 10), (10, -10), (-10, -10), (-10, 10)) at its altitude, where it holds. MODE shows **Calib 1/4** in yellow. B, Share or any stick or trigger takes it back.

## On screen
Top right: **CAMERA** (X500 V2, Follow or Gripper), and **MODE** (Loiter, Manual, Takeoff, RTL or Calib n/4 in yellow) with **Armed** (green) or **Disarmed** (red) left of it.

## On the ground
- Checkered ground: each tile is 1 m.
- Golf balls and a trash can: props you can push around with the drone or drag with the mouse (Force Tool).
- Blue dots: a test square of waypoints, flown with keyboard Y (calibration).

## Tuning it to the real drone
Everything about it is in `LuckyEngine/X500.param`: the drone (`SIM_*`: thrust, motor lag, yaw, drag), the motors (`MOT_*`) and the autopilot's gains (ArduPilot's X500v2 frame file) and limits. Change `SIM_*` to match logs of the real drone. `tools/x500v2/build_x500v2.py` builds its look and its physics.

## Latest changes
- **Y calibration is back:** in the air, keyboard Y flies the X500 around the blue test square, one corner per press (Calib n/4, yellow).
- **X500 only:** the Skydio X2 with its SO-100 arm, its autonomous picking and the Tuning panel are gone. The controller always flies the X500; LT climbs and RT descends.
- **X500 arming, takeoff and RTL:** Y arms / disarms, Share takes off (0.5 m) or returns home (RTL), B switches Loiter / Manual; the screen shows Armed / Disarmed next to MODE, and the props spin.
- **X500 Loiter mode:** let go and it stops. Yawing doesn't bob it up and down, it idles on the ground, and its drag is set only by `X500.param`.
- **Holybro X500 V2 flies:** physics and gains are in `LuckyEngine/X500.param`.
- **Trash can can be moved:** a free, light (~1.1 kg) aluminium can.
- **New look:** checkered 1 m ground tiles and higher sunlight.
- **Controller:** reconnects on its own if unplugged.
