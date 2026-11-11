"""Flies one scripted mission on three setups and writes a side-by-side report (sim/tests/out/compare_report.md):
  a. px4-x500/sitl          PX4 with its built-in SIH physics (the reference)
  b. px4-x500/px4 + sim sih our sim, PX4's SIH equations (parity check: should match a)
  c. px4-x500/px4 + dsb-sim  our sim, our rotor model + battery
Mission: arm, take off 5 m, hover 20 s, 5 m step north, 5 m step east, 8 m/s north for 6 s, yaw 90 deg, RTL.
Times are PX4's clock (time_boot_ms), so a sim running slower than real time doesn't change them.
Run: sim/.venv/bin/python sim/tests/compare.py [a b c]"""
import math
import os
import subprocess
import sys
import time

import numpy as np

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

m = mavutil.mavlink
HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..")
PY, SRC, OUT = os.path.join(SIM, ".venv", "bin", "python"), os.path.join(SIM, "src"), os.path.join(HERE, "out")
SETUPS = {"a": ("px4-x500/sitl:latest", None), "b": ("px4-x500/px4:latest", "sih"),
          "c": ("px4-x500/px4:latest", "dsb-sim")}
NAMES = {"a": "PX4 SIH (px4-x500)", "b": "our sim, sih", "c": "our sim, dsb-sim"}
API = 14591  # host port for the container's API link
OFFBOARD = 6 << 16
POS, VEL = 0x9F8, 0x9C7  # SET_POSITION_TARGET type_mask: position + yaw / velocity + yaw (0x400 would ignore yaw)


class Flight:
    def __init__(self):
        self.mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{API}", source_system=255)
        self.rows = []  # (px4 s, msg type, msg)
        self.latest = {}  # msg type -> the last one
        self.setpoint = None  # (type_mask, n, e, d, yaw)
        self.beat = 0.0
        self.status = []
        self.t = 0.0  # PX4's clock (s), from the last message that carries it

    def pump(self, seconds=0.05):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now - self.beat > 0.05:
                self.mav.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                if self.setpoint:
                    mask, a, b, c, yaw = self.setpoint
                    p = (a, b, c, 0, 0, 0) if mask == POS else (0, 0, 0, a, b, c)
                    self.mav.mav.set_position_target_local_ned_send(0, 1, 1, m.MAV_FRAME_LOCAL_NED, mask, *p,
                                                                    0, 0, 0, yaw, 0)
                self.beat = now
            msg = self.mav.recv_match(blocking=True, timeout=0.01)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "STATUSTEXT" and msg.chunk_seq and self.status:  # PX4 splits long texts: the rest of the last one
                self.status[-1] = (*self.status[-1][:2], self.status[-1][2] + msg.text.strip())
            elif t == "STATUSTEXT":
                self.status.append((self.t, msg.severity, msg.text.strip()))
            if hasattr(msg, "time_boot_ms"):
                self.t = msg.time_boot_ms / 1000
            self.rows.append((self.t, t, msg))
            self.latest[t] = msg

    def last(self, t):
        return self.latest.get(t)

    def until(self, what, done, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            if done():
                return self.t
        raise TimeoutError(what)

    def command(self, cmd, *p):
        p = (list(p) + [0] * 7)[:7]
        self.mav.mav.command_long_send(1, 1, cmd, 0, *p)

    def armed(self):
        hb = self.last("HEARTBEAT")
        return bool(hb.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)

    def pos(self):
        p = self.last("LOCAL_POSITION_NED")
        return np.array([p.x, p.y, p.z])

    def fly(self):
        self.until("heartbeat", lambda: self.last("HEARTBEAT") is not None, 60)
        for msg_id, hz in ((m.MAVLINK_MSG_ID_HIGHRES_IMU, 50), (m.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 10),
                           (m.MAVLINK_MSG_ID_VFR_HUD, 20), (m.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
                           (m.MAVLINK_MSG_ID_ATTITUDE, 50)):
            self.command(m.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        self.until("home", lambda: self.last("HOME_POSITION") and self.last("LOCAL_POSITION_NED"), 60)
        ev = {}

        def arm():
            self.command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.pump(0.5)
            return self.armed()
        ev["armed"] = self.until("armed", arm, 60)
        g = self.last("GLOBAL_POSITION_INT")
        home_amsl, ground = (g.alt - g.relative_alt) / 1000, self.pos()[2]
        self.command(m.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, home_amsl + 5)
        ev["takeoff"] = self.until("5 m", lambda: self.pos()[2] - ground < -4.8, 60)
        self.pump(3)
        p0, yaw0 = self.pos(), self.last("ATTITUDE").yaw
        self.setpoint = (POS, *p0, yaw0)
        self.pump(1)
        self.command(m.MAV_CMD_DO_SET_MODE, m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
        self.until("offboard", lambda: self.last("HEARTBEAT").custom_mode == OFFBOARD, 10)
        ev["hover"] = self.t
        self.pump(20)
        ev["step_n"] = self.t
        self.setpoint = (POS, p0[0] + 5, p0[1], p0[2], yaw0)
        self.pump(12)
        ev["step_e"] = self.t
        self.setpoint = (POS, p0[0] + 5, p0[1] + 5, p0[2], yaw0)
        self.pump(12)
        ev["fast"] = self.t
        self.setpoint = (VEL, 8, 0, 0, yaw0)
        self.pump(6)
        ev["brake"] = self.t
        self.setpoint = (POS, *self.pos(), yaw0)
        self.pump(10)
        ev["yaw"] = self.t
        self.setpoint = (POS, *self.setpoint[1:4], yaw0 + math.pi / 2)
        self.pump(8)
        ev["rtl"] = self.t
        self.setpoint = None
        self.command(m.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        ev["disarmed"] = self.until("landed and disarmed", lambda: not self.armed(), 180)
        return ev


def series(rows, t0, t1, typ, *fields):
    """[(t, f1, f2, ...)] of one message type between PX4 times t0 and t1."""
    return np.array([(t, *(getattr(msg, f) for f in fields)) for t, ty, msg in rows if ty == typ and t0 <= t < t1])


def step_metrics(rows, t0, axis, target):
    """Rise 10-90 %, overshoot, settle into +-5 % (0.25 m) for a 5 m position step."""
    s = series(rows, t0, t0 + 12, "LOCAL_POSITION_NED", "x", "y")
    x, t = s[:, 1 + axis], s[:, 0] - t0
    start, span = x[0], target - x[0]
    frac = (x - start) / span
    rise = t[np.argmax(frac >= 0.9)] - t[np.argmax(frac >= 0.1)]
    out = np.abs(frac - 1) > 0.05
    settle = t[np.nonzero(out)[0][-1] + 1] if out.any() and not out[-1] else (math.nan if out[-1] else 0.0)
    return rise, max(0.0, frac.max() - 1) * 100, settle


def analyse(f, ev):
    r, out = f.rows, {}
    out["time to armable (s, PX4 clock)"] = ev["armed"]
    out["takeoff to 5 m (s)"] = ev["takeoff"] - ev["armed"]
    hud = series(r, ev["hover"] + 5, ev["step_n"], "VFR_HUD", "throttle")
    out["hover throttle (%)"] = hud[:, 1].mean()
    p = series(r, ev["hover"] + 5, ev["step_n"], "LOCAL_POSITION_NED", "x", "y", "z")
    out["hover hold, horizontal std (cm)"] = 100 * np.hypot(p[:, 1].std(), p[:, 2].std())
    out["hover hold, vertical std (cm)"] = 100 * p[:, 3].std()
    for name, t0, axis in (("north", ev["step_n"], 0), ("east", ev["step_e"], 1)):
        s0 = series(r, t0 - 0.5, t0, "LOCAL_POSITION_NED", "x", "y")[-1]
        rise, over, settle = step_metrics(r, t0, axis, s0[1 + axis] + 5)
        out[f"5 m step {name}: rise / overshoot / settle"] = f"{rise:.2f} s / {over:.0f} % / {settle:.2f} s"
    v = series(r, ev["fast"] + 3, ev["brake"], "LOCAL_POSITION_NED", "vx")
    out["8 m/s leg: speed reached (m/s)"] = v[:, 1].mean()
    a = series(r, ev["fast"] + 3, ev["brake"], "ATTITUDE", "pitch")
    out["8 m/s leg: pitch (deg)"] = math.degrees(a[:, 1].mean())
    imu = series(r, ev["fast"] + 3, ev["brake"], "HIGHRES_IMU", "xacc", "zacc")
    out["8 m/s leg: IMU x / z accel (m/s^2)"] = f"{imu[:, 1].mean():.2f} / {imu[:, 2].mean():.2f}"
    hud = series(r, ev["fast"] + 3, ev["brake"], "VFR_HUD", "throttle")
    out["8 m/s leg: throttle (%)"] = hud[:, 1].mean()
    y = series(r, ev["yaw"], ev["rtl"], "ATTITUDE", "yaw")
    y0 = series(r, ev["yaw"] - 0.5, ev["yaw"], "ATTITUDE", "yaw")[-1, 1]
    err = np.degrees(np.abs((y[:, 1] - y0 - math.pi / 2 + math.pi) % (2 * math.pi) - math.pi))
    out["yaw 90 deg: time to within 5 deg (s)"] = y[np.argmax(err < 5), 0] - ev["yaw"] if (err < 5).any() else math.nan
    out["RTL to disarmed (s)"] = ev["disarmed"] - ev["rtl"]
    est = series(r, ev["armed"], ev["disarmed"], "ESTIMATOR_STATUS", "vel_ratio", "pos_horiz_ratio",
                 "pos_vert_ratio", "mag_ratio")
    out["EKF2 max test ratio vel / pos h / pos v / mag"] = " / ".join(f"{x:.2f}" for x in est[:, 1:].max(0))
    warn = sorted({text for t, sev, text in f.status if sev <= m.MAV_SEVERITY_WARNING and "Arming denied" not in text})
    out["PX4 warnings and errors (after arming became possible)"] = "; ".join(warn) or "none"
    out["arming refusals before ready"] = sum("Arming denied" in s[2] for s in f.status)
    return out


def run(key):
    image, model = SETUPS[key]
    sim, sim_log = None, None
    if model:
        sim_log = open(os.path.join(OUT, f"compare_{key}_sim.log"), "w")
        sim = subprocess.Popen([PY, "-u", SRC, "--model", model], stdout=sim_log, stderr=subprocess.STDOUT)
        time.sleep(2)
    subprocess.run(["docker", "rm", "-f", "cmp-px4"], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", "cmp-px4", "-p", f"{API}:14580/udp", image], check=True,
                   capture_output=True)
    time.sleep(3)  # PX4 answers the first sender only: let it listen first
    try:
        f = Flight()
        ev = f.fly()
        res = analyse(f, ev)
    finally:
        logs = subprocess.run(["docker", "logs", "cmp-px4"], capture_output=True, text=True)
        with open(os.path.join(OUT, f"compare_{key}_px4.log"), "w") as fh:
            fh.write(logs.stdout + logs.stderr)
        subprocess.run(["docker", "rm", "-f", "cmp-px4"], capture_output=True)
        if sim:
            sim.terminate()
            sim.wait()
            sim_log.close()
    if model:
        rates = [float(line.split("s ")[1].split("x")[0]) for line in open(sim_log.name) if "x real time" in line]
        res["sim speed (x real time, mean)"] = np.mean(rates[2:]) if len(rates) > 2 else math.nan
    return res


def main():
    os.makedirs(OUT, exist_ok=True)
    keys = sys.argv[1:] or list(SETUPS)
    results = {}
    for k in keys:
        print(f"== {k}: {NAMES[k]}", flush=True)
        results[k] = run(k)
        for name, v in results[k].items():
            print(f"   {name}: {v:.2f}" if isinstance(v, float) else f"   {name}: {v}", flush=True)
    rows = list(dict.fromkeys(n for r in results.values() for n in r))
    lines = ["# Sim comparison", "", "| | " + " | ".join(NAMES[k] for k in keys) + " |",
             "|---|" + "---|" * len(keys)]
    for n in rows:
        cells = [results[k].get(n, "") for k in keys]
        lines.append(f"| {n} | " + " | ".join(f"{c:.2f}" if isinstance(c, (float, np.floating)) else str(c)
                                              for c in cells) + " |")
    with open(os.path.join(OUT, "compare_report.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
