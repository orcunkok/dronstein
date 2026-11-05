"""Offline MuJoCo port of the original DroneTeleop.cs flight controller + helpers.
Frames: MuJoCo z up. Drone nose = body -x. m_Yaw = heading of the nose in world: nose dir = (cos yaw, sin yaw).
Identity quaternion == yaw = pi (nose along world -x), exactly as in the C#.
"""
import os, math
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
MAX_THRUST = 13.0
ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
STOWED = np.array([0, -3.32, 3.11, 1.18, 0.0])
DEPLOYED = np.array([0, -2.01, 1.85, -0.35, 1.5708])
ROTORS = np.array([[-0.14, -0.18, -0.0201], [-0.14, 0.18, 0.0201], [0.14, 0.18, -0.0201], [0.14, -0.18, 0.0201]])

_MODEL = None
def load_model():
    global _MODEL
    if _MODEL is None:
        _MODEL = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'scene.xml'))
    return _MODEL


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def solve4(a):
    a = a.copy()
    for c in range(4):
        p = c + int(np.argmax(np.abs(a[c:, c])))
        a[[c, p]] = a[[p, c]]
        for r in range(4):
            if r == c: continue
            f = a[r, c] / a[c, c]
            a[r, c:] -= f * a[c, c:]
    return np.array([a[i, 4] / a[i, i] for i in range(4)])


class Quintic:
    """Min-jerk-style quintic from (p,v,a) to (p1, 0, 0) in T seconds, per axis (vector valued)."""
    def __init__(self, p0, v0, a0, p1, vmax, amax, min_T=0.2):
        p0, v0, a0, p1 = map(lambda x: np.array(x, float), (p0, v0, a0, p1))
        D = np.linalg.norm(p1 - p0)
        T = max(1.875 * D / vmax, math.sqrt(5.7735 * D / amax), min_T)
        # extra margin if starting with velocity
        T = max(T, 2.0 * np.linalg.norm(v0) / amax)
        self.T = T
        # coefficients of p(t)=c0+c1 t+c2 t^2+c3 t^3+c4 t^4+c5 t^5
        self.c0, self.c1, self.c2 = p0, v0, a0 / 2
        d = p1 - (p0 + v0 * T + 0.5 * a0 * T * T)
        dv = -(v0 + a0 * T)
        da = -a0
        A = np.array([[T**3, T**4, T**5], [3*T**2, 4*T**3, 5*T**4], [6*T, 12*T**2, 20*T**3]])
        sol = np.linalg.solve(A, np.stack([d, dv, da]))
        self.c3, self.c4, self.c5 = sol
        self.p1 = p1

    def at(self, t):
        t = min(max(t, 0.0), self.T)
        p = self.c0 + self.c1*t + self.c2*t**2 + self.c3*t**3 + self.c4*t**4 + self.c5*t**5
        v = self.c1 + 2*self.c2*t + 3*self.c3*t**2 + 4*self.c4*t**3 + 5*self.c5*t**4
        a = 2*self.c2 + 6*self.c3*t + 12*self.c4*t**2 + 20*self.c5*t**3
        return p, v, a


class Params:
    KpPos = 2.0; BrakeAcc = 8.0; KpVel = 6.0; KiPos = 8.0; IntegralRange = 0.3
    MaxTiltAcc = 4.0; KpAtt = 21.258; KdAtt = 1.5
    MaxSpeed = 0.7          # Tuning.MaxSpeed, slow mode
    ArmSpeed = 1.0          # rad/s
    TurnSpeed = 1.0         # rad/s
    VMax = 0.7; AMax = 0.7  # slow-mode reference trajectory
    pos_noise = 0.002; vel_noise = 0.01   # measurement noise (m, m/s)


class Sim:
    """Drone + arm + ball + bin, controlled like Lucky: controller at 60 Hz (zero-order hold), physics at model dt."""
    FRAME_DT = 1.0 / 60.0

    def __init__(self, seed=0, P=None, ball_pos=(-1.0, 0.0), bin_pos=(-3.0, 0.0), start_yaw=math.pi,
                 start_xy=(1.0, 0.0), ball_fric=1.0, ball_mass=0.0459, noise=True, dist_force=0.0):
        import copy
        self.m = copy.copy(load_model())
        self.d = mujoco.MjData(self.m)
        self.P = P or Params()
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        m, d = self.m, self.d
        m.geom_friction[m.geom('ball_geom').id][0] = ball_fric
        bb = m.body('ball').id
        sc = ball_mass / m.body_mass[bb]
        m.body_mass[bb] *= sc; m.body_inertia[bb] *= sc
        # reset
        mujoco.mj_resetData(m, d)
        self.drone = m.body('x2').id
        self.ball_b = m.body('ball').id
        self.bin_b = m.body('bin').id
        qa = m.joint('ball_free').qposadr[0]
        d.qpos[qa:qa+3] = [ball_pos[0], ball_pos[1], 0.02135]
        qb = m.joint('bin_free').qposadr[0]
        d.qpos[qb:qb+3] = [bin_pos[0], bin_pos[1], 0.0]
        d.qpos[0:2] = start_xy
        d.qpos[2] = 0.5
        h = (start_yaw - math.pi) / 2
        d.qpos[3:7] = [math.cos(h), 0, 0, math.sin(h)]
        self.arm_q = {n: m.joint(n).qposadr[0] for n in ARM}
        for n, v in zip(ARM[:5], STOWED): d.qpos[self.arm_q[n]] = v
        d.qpos[self.arm_q['gripper']] = -0.174
        d.ctrl[:] = 0
        mujoco.mj_forward(m, d)
        # bodies of the drone for CoM
        self.dbodies = [b for b in range(m.nbody) if m.body_rootid[b] == self.drone]
        self.dmass = np.array([m.body_mass[b] for b in self.dbodies])
        self.mass = self.dmass.sum()
        self.grasp_site = m.site('grasp_site').id
        self.fixed_jaw = m.body('Fixed_Jaw').id
        self.armlow = np.array([m.actuator(n).ctrlrange[0] for n in ARM])
        self.armhigh = np.array([m.actuator(n).ctrlrange[1] for n in ARM])
        self.act_arm = [m.actuator(n).id for n in ARM]
        # controller state
        self.target = np.array([d.qpos[0], d.qpos[1], d.qpos[2] + 0.5])
        self.tvel = np.zeros(3); self.tacc = np.zeros(3); self.eint = np.zeros(3)
        self.yaw = start_yaw
        self.yaw_goal = start_yaw
        self.arm_target = np.array([*STOWED, -0.174])
        self.traj = None; self.traj_t = 0.0
        self.t = 0.0
        self.max_tilt = 0.0
        self.dist_force = dist_force
        self.log = []
        self.thrust = np.zeros(4)
        # helper for metrics
        self.rob_geoms = [g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == self.drone and m.geom_contype[g] + m.geom_conaffinity[g] > 0]
        self.min_z_ever = 9.0
        self.drone_ground_contact = False

    # ----- measurements -----
    def pos(self): return self.d.qpos[0:3].copy()
    def ball(self): return self.d.xpos[self.ball_b].copy()
    def bin(self): return self.d.xpos[self.bin_b].copy()
    def grasp(self): return self.d.site_xpos[self.grasp_site].copy()
    def tilt_deg(self):
        R = quat_to_mat(self.d.qpos[3:7]); return math.degrees(math.acos(min(1, max(-1, R[2, 2]))))

    def jaw_frame(self):
        """world pos & rotation of Fixed_Jaw frame"""
        return self.d.xpos[self.fixed_jaw].copy(), self.d.xmat[self.fixed_jaw].reshape(3, 3).copy()

    def lowest_point(self, include_ball=False, near=None, radius=0.5):
        """conservative lowest z over drone+arm collision geoms (and the ball)"""
        m, d = self.m, self.d
        lo = 9.0
        gs = list(self.rob_geoms)
        if include_ball: gs.append(m.geom('ball_geom').id)
        for g in gs:
            if near is not None and np.linalg.norm(d.geom_xpos[g][:2] - near[:2]) > radius + 0.2:
                continue   # geom centre more than radius+0.2 m away (largest geom half-extent ~0.19)
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                # use mesh vertices
                me = m.geom_dataid[g]
                adr, n = m.mesh_vertadr[me], m.mesh_vertnum[me]
                V = m.mesh_vert[adr:adr+n]
                R = d.geom_xmat[g].reshape(3, 3)
                z = (V @ R.T)[:, 2] + d.geom_xpos[g][2]
                lo = min(lo, z.min())
            else:
                R = d.geom_xmat[g].reshape(3, 3)
                if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                    ext = m.geom_size[g][0]
                else:
                    ext = np.sum(np.abs(R[2, :]) * m.geom_size[g])
                lo = min(lo, d.geom_xpos[g][2] - ext)
        return lo

    # ----- commands -----
    def fly_to(self, p, yaw=None, vmax=None, amax=None):
        """quintic reference from the current reference state to p."""
        P = self.P
        p0 = self.target.copy()
        self.traj = Quintic(p0, self.tvel, self.tacc, p, vmax or P.VMax, amax or P.AMax)
        self.traj_t = 0.0
        if yaw is not None: self.yaw_goal = yaw

    def flying(self):
        return self.traj is not None and self.traj_t < self.traj.T

    def arm_goal(self, goal, speed=None):
        self.arm_goal_v = np.array(goal, float); self.arm_speed = speed or self.P.ArmSpeed

    def arm_at_goal(self, tol=1e-3):
        g = getattr(self, 'arm_goal_v', None)
        if g is None: return True
        return np.all(np.abs(g - self.arm_target[:len(g)]) < tol)

    # ----- controller (faithful port of MotorThrusts) -----
    def motor_thrusts(self, dt):
        d, P = self.d, self.P
        q = d.qpos[:7]; qv = d.qvel[:6]
        nz = self.rng.normal(0, 1, 6) if self.noise else np.zeros(6)
        pos = q[0:3] + nz[0:3] * P.pos_noise
        vel = qv[0:3] + nz[3:6] * P.vel_noise
        ang = qv[3:6]
        R = quat_to_mat(q[3:7])
        self.max_tilt = max(self.max_tilt, math.degrees(math.acos(min(1, max(-1, R[2, 2])))))
        com = (d.xpos[self.dbodies] * self.dmass[:, None]).sum(0) / self.mass
        err = self.target - pos
        dist = np.linalg.norm(err)
        speed = min(P.KpPos * dist, math.sqrt(2 * P.BrakeAcc * dist), P.MaxSpeed)
        vt = (err * (speed / dist) if dist > 1e-6 else np.zeros(3)) + self.tvel
        if dist < P.IntegralRange: self.eint += err * dt
        acc = (vt - vel) * P.KpVel + self.eint * P.KiPos + self.tacc
        h = math.hypot(acc[0], acc[1])
        if h > P.MaxTiltAcc: acc[0] *= P.MaxTiltAcc / h; acc[1] *= P.MaxTiltAcc / h
        acc[2] = max(acc[2], -0.5 * G)
        force = (acc + np.array([0, 0, G])) * self.mass
        total = force @ R[:, 2]
        z = force / np.linalg.norm(force)
        nose = np.array([-math.cos(self.yaw), -math.sin(self.yaw), 0])
        y = np.cross(z, nose); y /= np.linalg.norm(y)
        x = np.cross(y, z)
        Rt = np.stack([x, y, z], axis=1)
        E = Rt.T @ R - R.T @ Rt   # careful: matches E(i,j)=sum_k Rt[k,i]R[k,j]-R[k,i]Rt[k,j]
        att = np.array([E[2, 1], E[0, 2], E[1, 0]]) * 0.5
        torque = -P.KpAtt * att - P.KdAtt * ang
        dd = com - q[0:3]
        comx = R[:, 0] @ dd; comy = R[:, 1] @ dd
        mix = np.zeros((4, 5))
        for i in range(4):
            rx = ROTORS[i, 0] - comx; ry = ROTORS[i, 1] - comy
            mix[:, i] = [1, ry, -rx, ROTORS[i, 2]]
        mix[:, 4] = [total, *torque]
        th = np.clip(solve4(mix), 0, MAX_THRUST)
        return th

    # ----- one rendered frame -----
    def frame(self):
        dt = self.FRAME_DT
        P = self.P
        # reference trajectory
        if self.traj is not None:
            self.traj_t += dt
            self.target, self.tvel, self.tacc = self.traj.at(self.traj_t)
            if self.traj_t >= self.traj.T:
                self.tvel = np.zeros(3); self.tacc = np.zeros(3)
        # yaw slew
        dy = (self.yaw_goal - self.yaw)
        dy = (dy + math.pi) % (2*math.pi) - math.pi
        self.yaw += max(-P.TurnSpeed*dt, min(P.TurnSpeed*dt, dy))
        # arm slew
        g = getattr(self, 'arm_goal_v', None)
        if g is not None:
            sp = self.arm_speed * dt
            for i in range(len(g)):
                self.arm_target[i] += max(-sp, min(sp, g[i] - self.arm_target[i]))
        gg = getattr(self, 'grip_goal', None)
        if gg is not None:
            sp = getattr(self, 'grip_speed', P.ArmSpeed) * dt
            self.arm_target[5] += max(-sp, min(sp, gg - self.arm_target[5]))
        self.arm_target = np.clip(self.arm_target, self.armlow, self.armhigh)
        th = self.motor_thrusts(dt)
        self.thrust = th
        self.d.ctrl[0:4] = th
        for i in range(6): self.d.ctrl[self.act_arm[i]] = self.arm_target[i]
        # physics until end of the frame
        end = self.t + dt
        while self.d.time < end - 1e-9:
            if self.dist_force:
                self.d.xfrc_applied[self.drone, 0:3] = self.rng.normal(0, self.dist_force, 3)
            mujoco.mj_step(self.m, self.d)
        self.t = self.d.time
        self.min_z_ever = min(self.min_z_ever, self.d.qpos[2])
        # drone-ground contact (drone body geoms only)
        for c in self.d.contact[:self.d.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if self.m.geom(a).name == 'ground' and self.m.geom_bodyid[b] == self.drone:
                    self.drone_ground_contact = True

    def run(self, seconds, until=None):
        n = int(seconds / self.FRAME_DT)
        for _ in range(n):
            self.frame()
            if until is not None and until(): return True
        return until is None
