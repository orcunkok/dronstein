"""MAVLink 2 HIL server: PX4 SITL (simulator_mavlink) connects as a TCP client and flies the physics in lockstep.
Every 4 ms of sim time: HIL_SENSOR out (its time_usec is PX4's clock), HIL_ACTUATOR_CONTROLS back, physics steps 4 ms."""
import select
import socket
import time

import numpy as np
from pymavlink.dialects.v20 import common as mavlink

import physics
from lucky import Hud, anchor
from sensors import Sensors, ned_to_lla

DT = 0.004
ACCEL, GYRO, MAG, BARO = 0b111, 0b111000, 0b111000000, 0b1101000000000  # HIL_SENSOR fields_updated, as PX4 reads them
ARMED = mavlink.MAV_MODE_FLAG_SAFETY_ARMED
WAIT = 0.1  # s wall, longest wait for PX4's controls before stepping without them


class Link:
    """What pymavlink writes to: the TCP socket."""
    def __init__(self, sock):
        self.sock = sock

    def write(self, buf):
        self.sock.sendall(buf)


def serve(prm, model, port, speed, log=None, lucky=None):
    """Accept PX4 connections one at a time, a fresh vehicle on the ground for each."""
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"sim: {model} model, waiting for PX4 on tcp 0.0.0.0:{port}", flush=True)
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"sim: PX4 connected from {addr[0]}:{addr[1]}", flush=True)
        session = Session(conn, prm, model, speed, log, lucky)
        try:
            session.run()
        except (ConnectionError, OSError) as e:
            print(f"sim: PX4 disconnected ({e})", flush=True)
        finally:
            conn.close()
            if session.log:
                session.log.close()


class Session:
    def __init__(self, conn, prm, model, speed, log, lucky):
        self.conn, self.prm, self.model, self.speed, self.lucky = conn, prm, model, speed, lucky
        self.mav = mavlink.MAVLink(Link(conn), srcSystem=1, srcComponent=mavlink.MAV_COMP_ID_USER1)
        self.sens = Sensors(prm["sensors"], prm["home"])
        self.y = physics.initial_state(prm, model)
        self.f = physics.specific_force(self.y, np.zeros(4), False, prm, model)
        self.bat = (0.0, 0.0)
        self.u, self.armed, self.controls = np.zeros(4), False, 0
        s = prm["sensors"]  # steps between samples of the slower sensors
        self.mag_n, self.baro_n, self.gps_n, self.truth_n = (max(1, round(1 / (s[f"{x}_hz"] * DT)))
                                                             for x in ("mag", "baro", "gps", "truth"))
        self.log = open(log, "w") if log else None
        if self.log:
            self.log.write("t,n,e,d,vn,ve,vd,qw,qx,qy,qz,p,q,r,a0,a1,a2,a3,u0,u1,u2,u3,armed,volts,amps,soc\n")

    def receive(self, timeout):
        """Read what PX4 sent within timeout; True once a new HIL_ACTUATOR_CONTROLS came in."""
        deadline, got = time.monotonic() + timeout, False
        while True:
            left = max(0.0, deadline - time.monotonic())
            if not select.select([self.conn], [], [], 0 if got else left)[0]:
                return got
            data = self.conn.recv(65536)
            if not data:
                raise ConnectionError("closed")
            for msg in self.mav.parse_buffer(data) or []:
                if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                    self.u = np.array(msg.controls[:4], float)  # PX4 motor i = CA_ROTOR i = our rotor i
                    armed = bool(msg.mode & ARMED)
                    if armed != self.armed:
                        print(f"sim: {'armed' if armed else 'disarmed'} at t={self.t:.2f} s", flush=True)
                    self.armed, self.controls, got = armed, self.controls + 1, True

    def send(self, k, t_us):
        s, prm, y = self.prm["sensors"], self.prm, self.y
        R = physics.dcm(y[physics.Q])
        f_b = R.T @ self.f  # specific force, body
        accel, gyro = self.sens.imu(f_b, y[physics.W], DT)
        fields, mag, (p, p_alt, temp) = ACCEL | GYRO, np.zeros(3), (0.0, 0.0, 0.0)
        if k % self.mag_n == 0:
            fields, mag = fields | MAG, self.sens.mag(R)
        if k % self.baro_n == 0:
            fields, (p, p_alt, temp) = fields | BARO, self.sens.baro(prm["home"]["alt"] - y[2])
        self.mav.hil_sensor_send(t_us, *accel, *gyro, *mag, p, 0.0, p_alt, temp, fields, 0)
        if k % self.gps_n == 0:
            lat, lon, galt, v = self.sens.gps(y[physics.P], y[physics.V])
            vel = np.hypot(v[0], v[1])
            cog = round(np.degrees(np.arctan2(v[1], v[0])) % 360 * 100) % 36000 if vel > 0.2 else 65535
            self.mav.hil_gps_send(t_us, 3, round(lat * 1e7), round(lon * 1e7), round(galt * 1000),
                                  round(s["gps_eph"] * 100), round(s["gps_epv"] * 100), round(vel * 100),
                                  *(round(x * 100) for x in v), cog, s["gps_sats"], 0, 0)
        if k % self.truth_n == 0:
            lat, lon, talt = ned_to_lla(*y[physics.P], prm["home"])
            v = y[physics.V]
            self.mav.hil_state_quaternion_send(
                t_us, y[physics.Q].tolist(), *y[physics.W], round(lat * 1e7), round(lon * 1e7), round(talt * 1000),
                *(int(np.clip(round(x * 100), -32767, 32767)) for x in v),
                round(np.linalg.norm(v) * 100), round(np.linalg.norm(v) * 100),
                *(int(np.clip(round(x / prm["g"] * 1000), -32767, 32767)) for x in f_b))
        if k % 250 == 0:
            self.mav.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def write_log(self, t):
        y, (v, a) = self.y, self.bat
        row = [t, *y[:17], *self.u, int(self.armed), v, a, y[physics.SOC]]
        self.log.write(",".join(f"{x:.6g}" for x in row) + "\n")

    def tick(self, ext=None):
        """One 4 ms lockstep step: sensors to PX4, its motor commands back, physics (ext: the world's wrench)."""
        self.k, self.t = self.k + 1, (self.k + 1) * DT
        self.send(self.k, round(self.t * 1e6))
        # PX4 only answers once its controllers run; until its first controls, stream sensors without waiting.
        self.receive(WAIT if self.controls else 0.0)
        self.y, self.f, self.bat = physics.step(self.y, self.u, self.armed, self.prm, self.model, DT, ext=ext)
        if self.log:
            self.write_log(self.t)

    def status(self):
        now = time.monotonic()
        if now - self.last[0] < 5:
            return
        rate = (self.t - self.last[1]) / (now - self.last[0])
        late = (self.k - self.last[2]) - (self.controls - self.last[3])  # steps PX4's controls didn't arrive in time for
        print(f"sim: t={self.t:7.1f} s  {rate:5.2f}x real time  alt={-self.y[2]:6.2f} m  "
              f"u={np.round(self.u, 2).tolist()}  {'armed' if self.armed else 'disarmed'}"
              + (f"  {self.bat[0]:.2f} V {self.bat[1]:.1f} A" if self.model == "dsb-sim" else "")
              + (f"  late {late}" if self.controls and late > 0 else ""), flush=True)
        self.last = (now, self.t, self.k, self.controls)

    def run(self):
        self.k, self.t = 0, 0.0
        self.last = (time.monotonic(), 0.0, 0, 0)
        if self.lucky:
            return self.run_lucky()
        next_wall = time.monotonic()
        while True:
            self.tick()
            speed = self.speed if self.controls else 1.0
            if speed > 0:
                next_wall = max(next_wall + DT / speed, time.monotonic() - 0.1)
                time.sleep(max(0.0, next_wall - time.monotonic()))
            self.status()

    def run_lucky(self):
        """Lucky's MuJoCo moves the drone and its tick is the clock. Each tick: take Lucky's state (what the world did
        shows as the difference to our prediction, kept as the world's wrench), fly the tick with PX4, answer with the
        average of our own wrench for Lucky to apply."""
        s, m = self.prm[self.model], self.prm[self.model].mass
        J = np.array([s.ixx, s.iyy, s.izz])
        ext = np.array([0.0, 0.0, -m * self.prm["g"], 0.0, 0.0, 0.0])  # it starts resting on Lucky's floor
        hud, armed, mode = None, False, ""
        while True:
            tick = self.lucky.wait(1.0)
            if tick is None:
                print("sim: waiting for Lucky (X500Flight in Play)", flush=True)
                continue
            dt = tick[0]
            self.y, ext = anchor(self.y, ext, *tick, m, J)
            own = np.zeros(6)
            n = max(1, round(dt / DT))
            for _ in range(n):
                own += np.concatenate(physics.wrench(self.y, self.prm, self.model)) / n
                self.tick(ext)
            if hud is None and self.controls and self.t > 3:  # PX4's MAVLink is up (Docker drops what comes earlier)
                hud = Hud(self.prm["lucky"]["px4_gcs"])
            if hud:
                armed, mode = hud.read(self.k)
            self.lucky.reply(own[:3], own[3:], armed, mode)
            self.status()
