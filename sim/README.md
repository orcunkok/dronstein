# sim: dsb-sim, headless X500 flight physics for PX4

A Python flight-physics simulator of the Holybro X500 V2 that PX4 SITL flies in lockstep over MAVLink HIL
(`simulator_mavlink`), with no Gazebo and no display. It runs on its own (headless), or flies the drone in Lucky's
MuJoCo world (`--lucky`). The math is compiled with numba: a 4 ms step takes ~30 µs.

## Quick start
From the dronstein folder:
```sh
uv venv sim/.venv && uv pip install --python sim/.venv/bin/python -r sim/requirements.txt   # once (Python >= 3.11)

sim/.venv/bin/python sim/src                  # headless: the sim's own flat ground
sim/.venv/bin/python sim/src --lucky          # in Lucky's world: then press Play in Lucky

docker run --rm -d --name px4 -p 18570:18570/udp -p 14580:14580/udp px4-x500/px4   # PX4 (tiny-sitl, px4 branch)
```
Then fly it over MAVLink (`autonomy/src/dronstein.py --mav`, QGroundControl, MAVSDK, ...). The first start compiles
the physics (a few seconds); later starts use numba's cache.

## Options
`sim/.venv/bin/python sim/src [options]`

| Option | Default | What it does |
|---|---|---|
| `--lucky [HOST]` | off; `127.0.0.1` when given | Fly in Lucky's MuJoCo world, listening for Lucky's `X500Flight` script on `HOST:14600` (`[lucky] port`) |
| `--model sih\|dsb-sim` | `dsb-sim` | Force model: `dsb-sim` (per-rotor model, drag, battery) or `sih` (PX4's SIH equations, for parity checks) |
| `--speed X` | `1` | Headless only: `1` real time, `0` as fast as PX4's lockstep allows (~5x). With `--lucky`, Lucky's tick is the clock |
| `--home LAT,LON,ALT` | `[home]` in params.toml | Start point on Earth (deg, deg, m above sea level) for GPS, baro and the magnetometer |
| `--instance N` | `0` | Listen for PX4 on TCP `4560 + N`, for PX4 started with `INSTANCE=N` |
| `--log FILE` | off | Write the true state every step (4 ms) to a CSV |
| `--params FILE` | `sim/params.toml` | The parameter file |

### PX4 container
`px4-x500/px4` is built from [tiny-sitl](https://github.com/orcunkok/tiny-sitl)'s `px4` branch
(`DOCKER_BUILDKIT=0 docker build -t px4-x500/px4 .`). Its environment variables (`-e NAME=value`):

| Variable | Default | What it does |
|---|---|---|
| `SIM_HOST` | `host.docker.internal` | Where the sim runs (name or IP); PX4 connects to `SIM_HOST:4560+INSTANCE` |
| `INSTANCE` | `0` | Shifts every port by `+INSTANCE` (sim TCP, GCS and API UDP) |
| `SYSID` | `INSTANCE+1` | MAVLink system id |
| `GCS_HOST` / `API_HOST` | unset | Make PX4 send first, to `host:14550` (QGroundControl) / `host:14540` (your software) |
| `/params/extra/*.params` (mount) | none | Extra PX4 parameters, `NAME VALUE` per line or a QGroundControl export; `tests/indoor.params` makes RTL return at 3 m in Lucky's office |

PX4 answers only the first program that talks to it: start a fresh container before connecting another one.

### Ports
| Port | Who listens | For |
|---|---|---|
| TCP 4560 (+N) | sim | PX4 ↔ sim (MAVLink HIL, lockstep) |
| UDP 14600 | sim (`--lucky`) | Lucky's `X500Flight` ↔ sim |
| UDP 14580 (+N) | PX4 | API link: `autonomy/src/dronstein.py`, the test scripts, MAVSDK |
| UDP 18570 (+N) | PX4 | GCS link: QGroundControl; the sim reads PX4's mode here for Lucky's HUD |

## How it works
- **Headless:** every 4 ms of sim time PX4 gets HIL_SENSOR (250 Hz IMU, 100 Hz mag, 50 Hz baro), HIL_GPS (10 Hz) and
  HIL_STATE_QUATERNION (50 Hz); the sim waits for HIL_ACTUATOR_CONTROLS and steps 4 ms (RK4, 4 substeps). The drone
  stands on four landing-gear feet on a flat ground.
- **With Lucky (`--lucky`):** Lucky's MuJoCo moves the one drone body; every contact is Lucky's. Each Lucky tick
  (50 Hz) its `X500Flight` script sends the body's state; the sim continues from it, flies the tick with PX4 (5 × 4 ms)
  and answers with the tick's average force and torque, which Lucky applies through the free-joint motors. What
  Lucky's velocities show beyond that is the world's push (contacts), added as a force so PX4's IMU feels it. The
  answer also carries PX4's armed state and mode for Lucky's HUD, read from PX4's GCS link (`[lucky] px4_gcs`).
- PX4's battery reading is its own (`simulator_mavlink` takes no battery message); the sim's battery drives the rotors.

## Parameters (`params.toml`, SI units)
| Section | What it holds |
|---|---|
| `[home]` | Start point: latitude, longitude, altitude |
| `[sih]` | PX4 SIH's quadcopter (mass, inertia, max thrust, drag), for `--model sih` |
| `[dsb-sim]` | The airframe (mass, inertia, rotor hubs: Lucky's x500v2 model) and our rotor model (thrust and moment constants, motor time constants, rotor drag, ESC range, body drag) |
| `[lucky]` | Lucky link: port, the x500v2's centre of mass from its body origin, PX4's GCS address for the HUD |
| `[battery]` | 4S LiPo: cells, capacity, internal resistance, open-circuit voltage curve |
| `[ground]` | Headless ground: landing-gear feet, stiffness, damping, friction |
| `[sensors]` | IMU / baro / mag / GPS noise and rates |

## Tests
```sh
sim/.venv/bin/python -m pytest sim/tests                 # physics checks and the Lucky loop without Lucky (seconds)
sim/.venv/bin/python sim/tests/compare.py [a b c]        # flight comparison, needs Docker and both PX4 images
```
`compare.py` flies one mission on (a) PX4's SIH image `px4-x500/sitl`, (b) PX4 + this sim with `--model sih`, (c) PX4 +
this sim with `dsb-sim`, and writes `sim/tests/out/compare_report.md` (default: all three). `tests/indoor.params`: PX4
parameters for flying in Lucky's office (RTL at 3 m).

## Code
| File | What it is |
|---|---|
| `src/__main__.py` | The command line |
| `src/physics.py` | 6-DOF rigid body (NED/FRD, quaternion, RK4), the ground, and the force models: `sih` (PX4 SIH, as in `sih.cpp`) and `dsb-sim` (each rotor's thrust kT w², kM yaw moment, rotor drag and rolling moment after Gazebo's MulticopterMotorModel, PX4's 150..1000 rad/s ESC range scaled by the pack voltage, quadratic body drag, 4S battery). An optional world wrench replaces the ground (Lucky) |
| `src/sensors.py` | IMU from the true specific force (noise + bias random walk), ISA baro, WMM magnetometer, GPS |
| `src/px4.py` | The HIL server: PX4 connects to TCP `4560 + instance`; the headless loop and the Lucky-clocked loop |
| `src/lucky.py` | The Lucky link: frames (MuJoCo world and FLU body ↔ NED / FRD, body origin ↔ CoM), the world wrench, PX4's mode for the HUD |
| `tests/test_physics.py` | Hover, free fall and landing, SIH's equations, rotor drag, battery sag |
| `tests/test_lucky.py` | The Lucky loop against a fake Lucky: resting, free-air hover, a drop onto the floor |
| `tests/compare.py` | The three-way flight comparison |
