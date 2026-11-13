"""The link to Lucky, where the drone is a MuJoCo body in the real world (X500Flight.cs). Each Lucky tick Lucky sends
the body's state; the sim flies that tick with PX4 and answers with its average force and torque, which Lucky applies.
UDP, one JSON line each way. Frames are mapped here and nowhere else: MuJoCo's world is x north, y west, z up (NED:
n = x, e = -y, d = -z); Lucky's body frame is FLU (FRD = x, -y, -z) and its origin isn't the CoM ([lucky] com)."""
import json
import socket

import numpy as np
from pymavlink import mavutil

from physics import P, Q, V, W, dcm

F = np.array([1.0, -1.0, -1.0])
FLIP = np.array([1.0, 1.0, -1.0, -1.0])  # quaternion FLU->world <-> FRD->NED
# PX4 custom_mode (main << 16 | sub << 24) -> the name the HUD shows
MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO", 6: "OFFBOARD", 7: "STAB", 4 | 2 << 8: "TAKEOFF",
         4 | 3 << 8: "HOLD", 4 | 4 << 8: "MISSION", 4 | 5 << 8: "RTL", 4 | 6 << 8: "LAND"}


def anchor(y, ext, dt, p, v, q, w, m, J):
    """Continue from Lucky's state. Lucky applied the same forces as the prediction; what its velocities show beyond
    that is the world (contacts), added to the world's wrench (force NED, moment FRD) the sim flies with."""
    ext = ext + np.concatenate([m * (v - y[V]), J * (w - y[W])]) / dt
    y = y.copy()
    y[P], y[V], y[Q], y[W] = p, v, q, w
    return y, ext


class Lucky:
    def __init__(self, host, prm):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, prm["lucky"]["port"]))
        self.com = F * np.asarray(prm["lucky"]["com"], float)  # FLU, from the body origin
        self.lucky = None  # where Lucky's script answers from
        self.home = None  # the drone's first (x, y): the NED origin, where PX4 starts

    def wait(self, timeout):
        """Lucky's next tick: (dt, its state as position NED, velocity NED, quaternion FRD->NED, rates FRD at the
        CoM), or None when nothing came."""
        data = None
        self.sock.settimeout(timeout)
        try:
            data, self.lucky = self.sock.recvfrom(65536)
            self.sock.settimeout(0)
            while True:  # the newest tick only: older ones queue up while nobody answers them
                data, self.lucky = self.sock.recvfrom(65536)
        except (TimeoutError, BlockingIOError):
            if data is None:
                return None
        m = json.loads(data)
        q, v = np.asarray(m["qpos"], float), np.asarray(m["qvel"], float)
        R = self.R = dcm(q[3:7])  # body FLU -> world, kept for the reply
        p = q[:3] + R @ self.com
        if self.home is None:
            self.home = p[:2].copy()
        p[:2] -= self.home
        return m["dt"], F * p, F * (v[:3] + R @ np.cross(v[3:], self.com)), q[3:7] * FLIP, F * v[3:]

    def reply(self, force, moment, armed, mode):
        """The sim's force NED and moment FRD about the CoM -> Lucky's motors: force in the world, torque in the body
        frame about the body origin (where the free joint applies the force)."""
        f_world = F * force
        torque = F * moment + np.cross(self.com, self.R.T @ f_world)
        line = json.dumps({"force": f_world.tolist(), "torque": torque.tolist(), "armed": armed, "mode": mode})
        self.sock.sendto(line.encode(), self.lucky)


class Hud:
    """PX4's armed state and mode, from its MAVLink GCS link (HEARTBEAT), for Lucky's HUD."""
    def __init__(self, url):
        self.mav = mavutil.mavlink_connection(url, source_system=254)
        self.armed, self.mode, self.beat = False, "", 0

    def read(self, k):
        if k % 250 == 0:  # once a second (4 ms steps): PX4 answers whoever talks to it
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        while (msg := self.mav.recv_match(type="HEARTBEAT", blocking=False)) is not None:
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                main, sub = msg.custom_mode >> 16 & 0xFF, msg.custom_mode >> 24 & 0xFF
                self.mode = MODES.get(main | sub << 8 if main == 4 else main, f"MODE {main}.{sub}")
        return self.armed, self.mode
