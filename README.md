# dronstein

A Holybro X500 V2 drone in [Lucky Engine](https://luckyrobots.com), flown by [PX4](https://px4.io).

PX4 SITL flies the drone; **dsb-sim**, our headless flight physics, computes its forces at 250 Hz; Lucky's MuJoCo moves
it through Lucky's world, so every contact (floor, walls, ceiling, props) is Lucky's own. The same ground-station code
talks MAVLink to PX4 here and to the real drone later.

> This is the `px4-dsbSim-lucky` branch. `holybro-mvp` is the PX4-free branch, where a controller inside Lucky flies it.

## Quick run
You need [Lucky Engine](https://luckyrobots.com), [Docker](https://www.docker.com) and
[uv](https://docs.astral.sh/uv/) (Python ≥ 3.11); an Xbox controller is optional.

Once:
```sh
git clone -b px4 https://github.com/orcunkok/tiny-sitl ../tiny-sitl                          # PX4's image (~10 min)
(cd ../tiny-sitl && DOCKER_BUILDKIT=0 docker build -t px4-x500/px4 .)
uv venv sim/.venv      && uv pip install --python sim/.venv/bin/python      -r sim/requirements.txt
uv venv autonomy/.venv && uv pip install --python autonomy/.venv/bin/python -r autonomy/requirements.txt
```
Each time:
```sh
sim/.venv/bin/python sim/src --lucky        # 1. the flight physics
#                                             2. open LuckyEngine/ in Lucky Engine, press Play ("Connected", top left)
docker run --rm -d --name px4 -p 18570:18570/udp -p 14580:14580/udp \
  -v $PWD/sim/tests/indoor.params:/params/extra/indoor.params:ro px4-x500/px4                  # 3. PX4
autonomy/src/dronstein.py --mav             # 4. fly with the Xbox controller, or: autonomy/tests/calibrate.py
```
Stop: Ctrl+C, `docker rm -f px4`, Stop in Lucky. Start a fresh PX4 container before connecting another program.
All options: [sim/README.md](sim/README.md) (the sim, PX4's container, ports) and
[autonomy/README.md](autonomy/README.md) (the ground station, the test scripts).

## In Lucky

### Xbox controller
| Button | What it does |
|---|---|
| Left stick | Move forward/back and left/right |
| RT / LT | Climb / descend |
| LB / RB | Turn left / right |
| Menu | Arm / disarm, on the ground only |
| B | POSCTL / ALTCTL; from OFFBOARD or RTL, back to POSCTL |
| X | Take off on the ground; in the air, return home (RTL) |
| A | Gripper grab / release (not driven in Lucky on this branch) |
| Right stick | Orbit / tilt the camera |
| D-pad up / down | Zoom in / out |
| View | Next camera: orbit the X500, follow it (behind its tail), orbit the gripper |

The flying buttons go to PX4 through `autonomy/src/dronstein.py` (`BIND` in `autonomy/src/gamepad.py`); the camera ones
are read in Lucky (`m_Bindings` in `X500Flight.cs`).

### Keyboard
| Key | What it does |
|---|---|
| Q / E | Zoom in / out |
| C | Next camera, same as View on the controller |
| Right mouse drag | Rotate the camera |
| R | Reset the scene: the X500, the balls and the trash can go back to where they started |

### On screen
- Top left: the sim's link, **Connected** (green) or **Disconnected** (red).
- Top right: **CAMERA** (X500 V2, Follow or Gripper) and PX4's **MODE** (POSCTL, ALTCTL, OFFBOARD, TAKEOFF, HOLD, RTL,
  LAND, ...; "No PX4" in red when nothing is connected), with **Armed** (green) or **Disarmed** (red) left of it.
- Frame rate: **Window → Scene Renderer → Statistics**. Lucky's tick is the sim's clock: at ~15 fps the loop runs at
  about 0.66x real time; a smaller viewport, or Ambient Occlusion / Shadows off (same panel), speeds it up.

### The scene
- An office; checkered ground, each tile 1 m. The ceiling is ~8.8 m above the start (`indoor.params` keeps RTL at 3 m).
- Tennis balls and a trash can: props the drone pushes around, or drag them with the mouse (Force Tool).
- Blue dots: the 7.6 m calibration square on the grey mat in the north-east corner ((14.3, 1.3), (14.3, -6.3),
  (6.7, -6.3), (6.7, 1.3), MuJoCo frame). `autonomy/tests/calibrate.py` flies PX4 to each and prints how close it
  settled: the numbers to tune the sim and PX4's gains against the real drone.
- The claw and the tennis ball under it are there, but nothing drives the claw on this branch yet.

## The repo
| Folder | What it is |
|---|---|
| `LuckyEngine/` | The Lucky project: the office world, the drone's MuJoCo body, cameras and HUD. `X500Flight.cs` is the link to the sim (state out, force and torque in); `X500View.cs` the cameras and HUD |
| `sim/` | dsb-sim, the headless flight physics PX4 flies over MAVLink HIL. [sim/README.md](sim/README.md) |
| `autonomy/` | The ground station (MAVLink to PX4) and the test scripts. [autonomy/README.md](autonomy/README.md) |
| `tools/` | Builders for the drone's model (`x500v2`), the claw and the office |
| `hardware/` | The real drone's bill of materials |

The drone's mass, inertia, centre of mass and rotor positions are Lucky's model
(`LuckyEngine/Assets/Robots/X500V2/x500v2.mjcf`, built by `tools/x500v2/build_x500v2.py`), copied into
`sim/params.toml`; rotors, drag, battery and sensors are dsb-sim's; the flight-control gains are PX4's X500 V2 airframe
(in tiny-sitl).
