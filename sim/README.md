# sim: headless X500 physics for PX4 SITL

A Python flight-physics simulator of the Holybro X500 V2 that PX4 SITL flies in lockstep over MAVLink HIL
(`simulator_mavlink`), with no Gazebo and no display. The math is compiled with numba: a 4 ms step takes ~30 µs.

| Path | What it is |
|---|---|
| `src/physics.py` | 6-DOF rigid body (NED/FRD, quaternion, RK4 at 1 kHz), four landing-gear feet on a spring-damper ground with friction, and two force models: **`sih`**, PX4's SIH quadcopter equations exactly as in `sih.cpp` (parity with the SIH image); **`dsb-sim`**, ours: a rotor model per motor after Gazebo's `MulticopterMotorModel`, starting from PX4's x500 constants (up/down time constants, kT w², kM yaw moment, rotor drag and rolling moment), PX4's 150..1000 rad/s ESC mapping scaled by battery voltage, quadratic body drag and a 4S LiPo (OCV(SoC) + internal resistance, current from rotor shaft power). |
| `src/sensors.py` | IMU from the true specific force (noise + bias random walk), ISA baro, WMM magnetometer, GPS. |
| `src/px4.py` | The HIL server. PX4 connects to tcp `4560 + instance`; every 4 ms of sim time it gets HIL_SENSOR (250 Hz IMU, 100 Hz mag, 50 Hz baro), HIL_GPS at 10 Hz and HIL_STATE_QUATERNION at 50 Hz, then the sim waits for HIL_ACTUATOR_CONTROLS and steps 4 ms. |
| `src/__main__.py` | The command line. |
| `params.toml` | Every parameter, in SI units. |
| `tests/test_physics.py` | Physics checks (pytest). |
| `tests/compare.py` | Flies one mission on PX4's SIH image and on PX4 + this sim (`sih` and `dsb-sim`), and writes `tests/out/compare_report.md`. |

## Run

```sh
uv venv sim/.venv && uv pip install --python sim/.venv/bin/python -r sim/requirements.txt  # once (Python >= 3.11)
sim/.venv/bin/python sim/src                       # from the dronstein folder; --model sih (default dsb-sim), --speed 0, --log truth.csv
docker run --rm -d --name px4 -p 18570:18570/udp -p 14580:14580/udp px4-x500/px4
```

`--speed 1` keeps real time, `--speed 0` runs as fast as PX4's lockstep allows (about 5x on a laptop).
`--instance N` listens on 4560 + N, for PX4 started with `INSTANCE=N`. `--home lat,lon,alt` moves the start point.
The first start compiles the physics (a few seconds); later starts use numba's cache.
PX4's battery stays its own (`simulator_mavlink` takes no battery message), so the sim's battery only affects the motors.

## Test

```sh
sim/.venv/bin/python -m pytest sim/tests          # physics checks
sim/.venv/bin/python sim/tests/compare.py         # three-way flight comparison, needs Docker and both PX4 images
```
