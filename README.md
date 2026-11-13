# dronstein
Lucky Robots: a Holybro X500 V2 drone in Lucky Engine, flown by PX4.

This is the `px4-dsbSim-lucky` branch: PX4 SITL flies the drone, our headless flight physics (dsb-sim) computes its
forces at 250 Hz, and Lucky's MuJoCo moves it through Lucky's world, so every contact (floor, walls, ceiling, balls) is
Lucky's own. `holybro-mvp` is the PX4-free branch, where a controller inside Lucky flies it.

## What's what
- **Lucky Engine** (`LuckyEngine/`): the world, the drone's MuJoCo body, the camera and the HUD. Its `X500Flight`
  script is the link to the sim: each tick it sends the drone's state and applies the force and torque the sim sends back.
- **The flight physics** (`sim/`): dsb-sim, headless Python, the drone's rotors, drag, battery and sensors, flown by PX4
  over MAVLink HIL. It also runs on its own, without Lucky. See [sim/README.md](sim/README.md).
- **PX4**: the autopilot, in a small Docker image (`px4-x500/px4`, from the `px4` branch of `tiny-sitl`).
- **The ground station** (`autonomy/`): Python, talks MAVLink to PX4 like it will to the real drone. See
  [autonomy/README.md](autonomy/README.md).

## How to run
First time only, set up Python (needs [uv](https://docs.astral.sh/uv/)) and build PX4's image:
```
uv venv sim/.venv && uv pip install --python sim/.venv/bin/python -r sim/requirements.txt
uv venv autonomy/.venv && uv pip install --python autonomy/.venv/bin/python -r autonomy/requirements.txt
cd ../tiny-sitl && git checkout px4 && DOCKER_BUILDKIT=0 docker build -t px4-x500/px4 . && cd -
```
Then:
1. `sim/.venv/bin/python sim/src --lucky`
2. Open the `LuckyEngine` project in Lucky Engine and press **Play** (top left turns green: Connected).
3. `docker run --rm -d -p 18570:18570/udp -p 14580:14580/udp -v $PWD/sim/tests/indoor.params:/params/extra/indoor.params:ro px4-x500/px4`
   (`indoor.params`: RTL returns at 3 m, under the office ceiling).
4. Fly with the Xbox controller: `autonomy/src/dronstein.py --mav`, or run the calibration flight:
   `autonomy/tests/calibrate.py`.

PX4 answers only the first program that talks to it: start a fresh container for another one.

## Controller (through `autonomy/src/dronstein.py --mav`, to PX4)
| Button | What it does |
|---|---|
| Left stick | Move forward/back and left/right |
| RT / LT | Climb / descend |
| LB / RB | Turn left / right |
| Menu | Arm / disarm, on the ground only |
| B | POSCTL / ALTCTL; from OFFBOARD or RTL, back to POSCTL |
| X | Take off on the ground; in the air, return home (RTL) |
| Right stick | Orbit / tilt the camera |
| D-pad up / down | Zoom in / out |
| View | Next camera: orbit the X500, follow it (behind its tail), orbit the gripper |

The flying buttons are in `autonomy/src/gamepad.py` (`BIND`), the camera ones in `X500Flight.cs` (`m_Bindings`).

## Keyboard (in Lucky)
| Key | What it does |
|---|---|
| Q / E | Zoom in / out |
| C | Next camera, same as View on the controller |
| Right mouse drag | Rotate the camera |
| R | Reset Lucky's scene: the X500, the balls and the trash can go back to where they started |

## On screen
- Top left: the sim's link, **Connected** (green) or **Disconnected** (red).
- Top right: **CAMERA** (X500 V2, Follow or Gripper) and PX4's **MODE** (POSCTL, ALTCTL, OFFBOARD, TAKEOFF, HOLD, RTL,
  LAND, ...; "No PX4" in red when nothing is connected), with **Armed** (green) or **Disarmed** (red) left of it.

## Calibration
`autonomy/tests/calibrate.py` takes off and sends PX4 to each blue dot of the 7.6 m test square on the grey mat in the
north-east corner ((14.3, 1.3), (14.3, -6.3), (6.7, -6.3), (6.7, 1.3), MuJoCo frame), then RTL. It prints how long each
took and how far from the dot it settled: the numbers to tune the sim (`sim/params.toml`) and PX4's gains against the
real drone.

## On the ground
- Checkered ground: each tile is 1 m.
- Tennis balls and a trash can: props the drone pushes around, or drag them with the mouse (Force Tool).
- Blue dots: the calibration test square.
- The claw and the tennis ball under it are there, but nothing drives the claw on this branch yet.

## The drone
Its mass, inertia, centre of mass and rotor positions are Lucky's model (`Assets/Robots/X500V2/x500v2.mjcf`, built by
`tools/x500v2/build_x500v2.py`), copied into `sim/params.toml`; the rotors, drag, battery and sensors are dsb-sim's
(`sim/params.toml`); the gains are PX4's X500 V2 airframe (in `tiny-sitl`).
