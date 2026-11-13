# dronstein

A Holybro X500 V2 drone in [Lucky Engine](https://luckyrobots.com), flown by [PX4](https://px4.io).

PX4 SITL flies the drone; **dsb-sim**, our headless flight physics, computes its forces at 250 Hz (rotors, drag,
battery, sensors); Lucky's MuJoCo moves it through Lucky's world, so every contact (floor, walls, ceiling, props) is
Lucky's own. The same ground-station code talks MAVLink to PX4 here and to the real drone later.

> This is the `px4-dsbSim-lucky` branch. `holybro-mvp` is the PX4-free branch, where a controller inside Lucky flies it.

## Quick start

### What you need
- macOS or Linux, [Lucky Engine](https://luckyrobots.com), [Docker](https://www.docker.com),
  [uv](https://docs.astral.sh/uv/) (Python ≥ 3.11), an Xbox controller (optional).
- PX4's image, built once from [tiny-sitl](https://github.com/orcunkok/tiny-sitl)'s `px4` branch (~10 min):
  ```sh
  git clone -b px4 https://github.com/orcunkok/tiny-sitl && cd tiny-sitl
  DOCKER_BUILDKIT=0 docker build -t px4-x500/px4 .
  ```

### Set up (once, from this folder)
```sh
uv venv sim/.venv      && uv pip install --python sim/.venv/bin/python      -r sim/requirements.txt
uv venv autonomy/.venv && uv pip install --python autonomy/.venv/bin/python -r autonomy/requirements.txt
```

### Fly
```sh
sim/.venv/bin/python sim/src --lucky                       # 1. the flight physics, waiting for Lucky and PX4
#                                                            2. open LuckyEngine/ in Lucky Engine, press Play:
#                                                               top left turns green, "Connected"
docker run --rm -d --name px4 -p 18570:18570/udp -p 14580:14580/udp \
  -v $PWD/sim/tests/indoor.params:/params/extra/indoor.params:ro px4-x500/px4   # 3. PX4
autonomy/src/dronstein.py --mav                            # 4. fly with the Xbox controller
```
Or, instead of step 4, fly the calibration square by itself: `autonomy/tests/calibrate.py`.

Stop: Ctrl+C the sim, `docker rm -f px4`, Stop in Lucky. **PX4 answers only the first program that talks to it**:
start a fresh container (`docker rm -f px4`, then step 3) before connecting another one.

### Without Lucky (headless)
```sh
sim/.venv/bin/python sim/src          # the sim's own flat ground instead of Lucky's world; then steps 3 and 4
```

## Options

### `sim/src` (the flight physics)
| Option | Default | What it does |
|---|---|---|
| `--lucky [HOST]` | off; `127.0.0.1` when given | Fly in Lucky's MuJoCo world, listening for Lucky's `X500Flight` script on `HOST:14600`. Lucky's tick is then the clock |
| `--model sih\|dsb-sim` | `dsb-sim` | Force model: ours (rotors, drag, battery), or PX4's SIH equations (for comparing with PX4's built-in sim) |
| `--speed X` | `1` | Headless only: `1` real time, `0` as fast as PX4's lockstep allows (~5x) |
| `--home LAT,LON,ALT` | `47.397742,8.545594,488` | Where on Earth it starts (deg, deg, m above sea level), for GPS and the magnetometer |
| `--instance N` | `0` | Listen for PX4 on TCP `4560 + N` (run several drones; start PX4 with `INSTANCE=N`) |
| `--log FILE` | off | Write the true state every 4 ms to a CSV |
| `--params FILE` | `sim/params.toml` | All physical parameters (see [sim/README.md](sim/README.md)) |

### PX4 container (`px4-x500/px4`, environment variables with `-e`)
| Variable | Default | What it does |
|---|---|---|
| `SIM_HOST` | `host.docker.internal` | Where the sim runs (name or IP); PX4 connects to `SIM_HOST:4560+INSTANCE` |
| `INSTANCE` | `0` | Shifts every port by `+INSTANCE` (sim TCP, GCS and API UDP) |
| `SYSID` | `INSTANCE+1` | MAVLink system id |
| `GCS_HOST` / `API_HOST` | unset | Make PX4 send first, to `host:14550` (QGroundControl) / `host:14540` (your software) |
| `/params/extra/*.params` (mount) | none | Extra PX4 parameters, `NAME VALUE` per line or a QGroundControl export |

### `autonomy/src/dronstein.py` and the test scripts
| Command | Default | What it does |
|---|---|---|
| `dronstein.py --mav [URL]` | `udpout:127.0.0.1:14580` | Fly PX4 with the Xbox controller |
| `tests/calibrate.py [URL] [ALT]` | `udpout:127.0.0.1:14580`, `2.5` m | Fly the four calibration dots, print the errors, RTL |
| `tests/test_mavlink.py [URL]` | `udpout:127.0.0.1:14580` | Scripted flight check: arm, take off, LOITER, ALT_HOLD, a setpoint, RTL |

### Ports
| Port | Who listens | For |
|---|---|---|
| TCP 4560 (+N) | sim | PX4 ↔ sim (MAVLink HIL, lockstep) |
| UDP 14600 | sim (`--lucky`) | Lucky's `X500Flight` ↔ sim |
| UDP 14580 (+N) | PX4 | API link: `dronstein.py`, the test scripts, MAVSDK |
| UDP 18570 (+N) | PX4 | GCS link: the sim reads PX4's mode here for Lucky's HUD |

## Controls

### Xbox controller (through `dronstein.py --mav`, to PX4)
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

The flying buttons are in `autonomy/src/gamepad.py` (`BIND`), the camera ones in `X500Flight.cs` (`m_Bindings`).

### Keyboard (in Lucky)
| Key | What it does |
|---|---|
| Q / E | Zoom in / out |
| C | Next camera, same as View on the controller |
| Right mouse drag | Rotate the camera |
| R | Reset Lucky's scene: the X500, the balls and the trash can go back to where they started |

### On screen
- Top left: the sim's link, **Connected** (green) or **Disconnected** (red).
- Top right: **CAMERA** (X500 V2, Follow or Gripper) and PX4's **MODE** (POSCTL, ALTCTL, OFFBOARD, TAKEOFF, HOLD, RTL,
  LAND, ...; "No PX4" in red when nothing is connected), with **Armed** (green) or **Disarmed** (red) left of it.
- Frame rate: Lucky's **Window → Scene Renderer → Statistics**.

## Calibration
`autonomy/tests/calibrate.py` takes off and sends PX4 to each blue dot of the 7.6 m test square on the grey mat in the
north-east corner ((14.3, 1.3), (14.3, -6.3), (6.7, -6.3), (6.7, 1.3), MuJoCo frame), then RTL. It prints how long
each took and how far from the dot it settled: the numbers to tune the sim (`sim/params.toml`) and PX4's gains against
the real drone.

## Good to know
- **Real time:** Lucky's tick is the clock. With a big viewport Lucky renders ~15 fps and the whole loop runs at about
  0.66x real time; a smaller viewport, or Ambient Occlusion / Shadows off (Window → Scene Renderer), speeds it up.
- **Indoors:** `sim/tests/indoor.params` makes RTL return at 3 m; the office ceiling is ~8.8 m above the start.
- **The claw** and the tennis ball under it are in the scene, but nothing drives the claw on this branch yet.

## What's in here
| Folder | What it is |
|---|---|
| `LuckyEngine/` | The Lucky project: the office world, the drone's MuJoCo body, cameras and HUD. `X500Flight.cs` is the link to the sim (state out, force and torque in); `X500View.cs` the cameras and HUD |
| `sim/` | dsb-sim, the headless flight physics PX4 flies over MAVLink HIL. See [sim/README.md](sim/README.md) |
| `autonomy/` | The ground station (MAVLink to PX4) and the test scripts. See [autonomy/README.md](autonomy/README.md) |
| `tools/` | Builders for the drone's model (`x500v2`), the claw and the office |
| `hardware/` | The real drone's bill of materials |

## The drone's numbers
Mass, inertia, centre of mass and rotor positions are Lucky's model (`LuckyEngine/Assets/Robots/X500V2/x500v2.mjcf`,
built by `tools/x500v2/build_x500v2.py`), copied into `sim/params.toml`; rotors, drag, battery and sensors are
dsb-sim's (`sim/params.toml`); the flight-control gains are PX4's X500 V2 airframe (in tiny-sitl).
