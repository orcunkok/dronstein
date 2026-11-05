"""Autonomous pick of ONE golf ball -> drop in the bin. Offline prototype (MuJoCo + faithful port of DroneTeleop MotorThrusts).
Run:  .venv/bin/python pick_proto.py [n_runs] [--stage a|b|c]
Frames: MuJoCo z up. nose = body -x = world (cos yaw, sin yaw). Arm IK is in the DRONE frame (see ik.py).
"""
import sys, math, time
import numpy as np
import mujoco
from dronelib import *
import ik as IK

# ---------------- design constants (see PICK_DESIGN.md) ----------------
H_PICK    = 0.42     # drone origin height while the fingertips are at ball level
STANDOFF  = 0.24     # horizontal distance drone origin -> ball along the nose
ROLL      = 1.5708   # wrist_roll: jaws open sideways
G_OPEN    = 0.90     # gripper ctrl, open (jaw gap ~7 cm at the pad middle)
G_CLOSE   = -0.174   # gripper ctrl, closed (torque limited to +-3.5 Nm by the actuator)
PRE_DZ    = 0.10     # drone starts the descent this far above H_PICK
DESC_V    = 0.04     # m/s drone descent over the ball (vmax of the reference)
ARM_V     = 0.8      # rad/s arm slew
RIM       = 0.45
SAFE_LOW  = RIM + 0.10 + 0.03   # lowest point near the bin (3 cm margin)
TOOL_DZ   = 0.003    # tool point (ball centre height) sits this far above the ball centre: pads stay >= 6 mm off the ground (contact margin is 3 mm)
ZONE_R    = 1.0      # m, transit line closer than this to the bin centre -> cruise high
Z_STOW_SAFE = SAFE_LOW + 0.133 + 0.02   # drone origin z so the stowed arm's lowest point is >= SAFE_LOW
SQUEEZE_LEAD = 0.02  # rad
GRIP_BALL_MIN, GRIP_BALL_MAX = 0.30, 0.50   # jaw angle window when a 4.27 cm ball is held (measured 0.39-0.40); an empty close goes to -0.174
HOLD_DELTA = None   # rad, squeeze command below the stall angle (None = keep commanding G_CLOSE)
K_IK      = None


def tool_world_to_q(sim, tool_w, roll=ROLL):
    """tool point (world) -> arm joint angles through the drone's ACTUAL pose (closed loop)."""
    global K_IK
    if K_IK is None: K_IK = IK.analytic_consts(roll)
    R = quat_to_mat(sim.d.qpos[3:7])
    t = R.T @ (np.asarray(tool_w) - sim.d.qpos[0:3])
    q = IK.ik_analytic(t, roll, -1, K_IK)
    if q is None or np.any(q < IK.LO[:4] - 1e-3) or np.any(q > IK.HI[:4] + 1e-3):
        raise Unreachable(f'tool_d={np.round(t,3)} q={None if q is None else np.round(q,2)}')
    return q


class Unreachable(Exception): pass


class Pick:
    def __init__(self, sim, verbose=False):
        self.s = sim; self.v = verbose
        self.phase = None; self.t_phase = 0.0
        self.m = dict(phase_t={}, fail=None)
        self.ball0 = sim.ball().copy()
        self.bin0 = sim.bin().copy()
        self.max_tilt = 0.0
        self.max_ball_tool = 0.0
        self.min_clear_bin = 9.0
        self.q_hold = None
        self.noservo = False; self.arm_v = ARM_V; self.q_fixed = None

    def enter(self, name):
        if self.v: print(f'  [{self.s.t:6.2f}] phase {name}  pos={self.s.pos().round(3)}')
        self.phase = name; self.t_phase = self.s.t
        self.m['phase_t'][name] = round(self.s.t, 2)

    def settled(self, tol=0.02, vtol=0.03):
        s = self.s
        return (not s.flying()) and np.linalg.norm(s.pos() - s.traj.p1) < tol and np.linalg.norm(s.d.qvel[0:3]) < vtol

    def ball_in_jaw(self):
        p, R = self.s.jaw_frame()
        return R.T @ (self.s.ball() - p)

    def step(self):
        """one rendered frame"""
        s = self.s
        s.frame()
        self.max_tilt = max(self.max_tilt, s.tilt_deg())

    def run(self, **kw):
        try:
            return self.run_(**kw)
        except Unreachable as e:
            self.m['fail'] = 'ik_unreachable ' + str(e) + f' phase={self.phase} t={self.s.t:.1f}'
            return self.finish()

    def run_(self, ball_xy=None, max_t=120.0):
        s = self.s; m = self.m
        ball = s.ball().copy()
        yaw = s.yaw
        # --- heading: nose points from the ball toward the bin, so the drone body stays on the side AWAY from the bin
        to = self.bin0[:2] - ball[:2]
        yaw = math.atan2(to[1], to[0])
        nose = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        hov = ball - STANDOFF * nose
        z_pre = H_PICK + PRE_DZ
        hov[2] = z_pre
        # =============== TRANSIT: stowed arm. Legs: climb, cruise, descend. Cruise high when the line passes near the bin.
        self.enter('TRANSIT')
        a, b = s.pos()[:2], hov[:2]
        ab = b - a; tt = np.clip(np.dot(self.bin0[:2] - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
        near_bin = np.linalg.norm(a + tt * ab - self.bin0[:2]) < ZONE_R
        z_cruise = max(z_pre, Z_STOW_SAFE) if near_bin else z_pre
        m['transit_high'] = bool(near_bin)
        legs = [[*s.pos()[:2], max(z_cruise, s.pos()[2])], [*hov[:2], z_cruise], hov]
        self.leg = 0
        s.fly_to(legs[0], yaw)
        while True:
            self.step()
            if not s.flying() and np.linalg.norm(s.pos() - s.traj.p1) < 0.04:
                self.leg += 1
                if self.leg >= len(legs): break
                s.fly_to(legs[self.leg], yaw)
            low = s.lowest_point(near=self.bin0, radius=0.5)
            if low < 9: self.min_clear_bin = min(self.min_clear_bin, low - RIM)
            if s.t > max_t: m['fail'] = 'transit_timeout'; return self.finish()
        # rotate the gripper to open early
        while not (self.settled(0.015) and abs(((s.yaw_goal - s.yaw)+math.pi) % (2*math.pi) - math.pi) < 0.02 and s.d.qvel[5] < 0.05):
            self.step()
            if s.t > max_t: m['fail'] = 'transit_timeout'; return self.finish()
        for _ in range(30): self.step()
        # =============== DEPLOY arm to the grasp posture (tool 10 cm above the ball with the drone 10 cm high), gripper open
        self.enter('DEPLOY')
        s.grip_goal = G_OPEN; s.grip_speed = 1.5
        tool_pre = ball + np.array([0, 0, PRE_DZ])
        tool_pre[2] = ball[2] + TOOL_DZ + (s.pos()[2] - H_PICK)       # same drone-frame pose as the final one
        t0 = s.t; dev = 0.0; tilt_d = 0.0; hov_pos = s.pos().copy()
        while True:
            q = tool_world_to_q(s, tool_pre)
            s.arm_goal(np.array([*q, ROLL]), self.arm_v)
            self.step()
            dev = max(dev, float(np.linalg.norm(s.pos() - hov_pos))); tilt_d = max(tilt_d, s.tilt_deg())
            m['deploy_dev_cm'] = dev * 100; m['deploy_tilt_deg'] = tilt_d
            if s.arm_at_goal(0.01) and abs(s.arm_target[5] - G_OPEN) < 1e-2 and s.t - t0 > 0.5: break
            if s.t - t0 > 12: m['fail'] = 'deploy_timeout'; return self.finish()
        for _ in range(45):   # settle (arm reaction)
            q = tool_world_to_q(s, tool_pre); s.arm_goal(np.array([*q, ROLL]), self.arm_v); self.step()
            dev = max(dev, float(np.linalg.norm(s.pos() - hov_pos))); m['deploy_dev_cm'] = dev * 100
        m['deploy_drone_xy_err'] = float(np.linalg.norm(s.pos()[:2] - hov[:2]))
        # =============== DESCEND: drone descends over the ball, arm IK holds the tool on the ball
        self.enter('DESCEND')
        if self.noservo: self.q_fixed = s.arm_target[:5].copy()   # test only: arm frozen at its pre-grasp pose, the drone does all the positioning
        s.fly_to([*s.pos()[:2], H_PICK], yaw, vmax=DESC_V, amax=0.3)
        while True:
            dz = s.target[2] - H_PICK
            tool = ball + np.array([0, 0, dz + TOOL_DZ])
            q = self.q_fixed if self.noservo else tool_world_to_q(s, tool)
            s.arm_goal(q if self.noservo else np.array([*q, ROLL]), ARM_V)
            self.step()
            m['ball_moved_cm'] = float(np.linalg.norm(s.ball()[:2] - ball[:2]) * 100)
            if not s.flying(): break
        for _ in range(60):
            q = self.q_fixed if self.noservo else tool_world_to_q(s, ball + np.array([0, 0, TOOL_DZ]))
            s.arm_goal(q if self.noservo else np.array([*q, ROLL]), ARM_V); self.step()
        m['ball_moved_cm_at_ground'] = float(np.linalg.norm(s.ball() - ball) * 100)
        jl = self.ball_in_jaw()
        m['ball_in_jaw_at_ground'] = [round(float(x), 4) for x in jl]
        m['site_ground_z'] = float(s.grasp()[2])
        m['lowest_pt_ground'] = float(s.lowest_point())
        m['drone_ground_contact'] = s.drone_ground_contact
        # =============== CLOSE
        self.enter('CLOSE')
        self.q_hold = s.arm_target[:5].copy()
        s.arm_goal(self.q_hold, ARM_V)
        # Force-limited close: the command only leads the measured jaw angle by SQUEEZE_LEAD rad, so the jaw torque is
        # kp*lead (50*0.02 = 1 Nm) instead of the saturated 3.5 Nm. Hard squeezing pushes the ball out of the wedge.
        t0 = s.t; stalled = 0
        s.grip_speed = 2.0
        while s.t - t0 < 4.0:
            gq_ = s.d.qpos[s.arm_q['gripper']]
            s.grip_goal = max(G_CLOSE, gq_ - SQUEEZE_LEAD)
            self.step()
            gv = s.d.qvel[s.m.joint('gripper').dofadr[0]]
            stalled = stalled + 1 if abs(gv) < 0.01 else 0
            if stalled > 30: break
        gq_stall = s.d.qpos[s.arm_q['gripper']]
        m['grip_stall_angle'] = float(gq_stall)
        gq = s.d.qpos[s.arm_q['gripper']]
        m['grip_joint_closed'] = float(gq)
        m['ball_moved_cm_after_close'] = float(np.linalg.norm(s.ball()[:2] - ball[:2]) * 100)
        # grasp detection
        jl = self.ball_in_jaw(); m['ball_in_jaw_after_close'] = [round(float(x), 4) for x in jl]
        grasped = (GRIP_BALL_MIN < gq < GRIP_BALL_MAX) and np.linalg.norm(s.ball() - s.grasp()) < 0.03
        m['grasp_detected'] = bool(grasped)
        if not grasped: m['fail'] = 'grasp_not_detected'; return self.finish()
        # =============== LIFT (drone climbs; arm frozen)
        self.enter('LIFT')
        hc = s.pos()[2] + (SAFE_LOW + 0.02 - s.lowest_point(include_ball=True))
        s.fly_to([*s.pos()[:2], hc], yaw)
        self.carry_off = s.ball()[:2] - s.pos()[:2]
        self.carry_phase_ball_tool = []
        while not self.settled(0.02, 0.05):
            self.carry_step()
            if s.t - t0 > 60: m['fail'] = 'lift_timeout'; return self.finish()
        for _ in range(20): self.carry_step()
        m['lift_ball_z'] = float(s.ball()[2])
        if getattr(self, 'hold_extra', 0):
            y0 = self.ball_in_jaw()[1]
            for _ in range(int(self.hold_extra * 60)): self.carry_step()
            m['creep_cm'] = float((self.ball_in_jaw()[1] - y0) * -100)
            m['creep_grip'] = float(s.d.qpos[s.arm_q['gripper']])
        if s.ball()[2] < 0.3: m['fail'] = 'ball_lost_in_lift'; return self.finish()
        # =============== CARRY to the bin
        self.enter('CARRY')
        tgt = np.array([*(self.bin0[:2] - (s.ball()[:2] - s.pos()[:2])), hc])
        s.fly_to(tgt, yaw)
        t1 = s.t
        while not self.settled(0.015, 0.03):
            self.carry_step()
            if s.t - t1 > 90: m['fail'] = 'carry_timeout'; return self.finish()
        for _ in range(30): self.carry_step()
        m['ball_to_bin_xy_cm'] = float(np.linalg.norm(s.ball()[:2] - self.bin0[:2]) * 100)
        # =============== DROP
        self.enter('DROP')
        s.grip_goal = G_OPEN; s.grip_speed = 1.5
        t2 = s.t
        while s.t - t2 < 1.8:
            self.carry_step()
        # =============== RETREAT: climb
        self.enter('RETREAT')
        s.fly_to([*s.pos()[:2], s.pos()[2] + 0.15], yaw)
        while s.t - t2 < 3.5:
            self.carry_step()
        for _ in range(120): self.step()   # let the ball settle
        return self.finish()

    def tool_pt(self):
        p, R = self.s.jaw_frame()
        return p + R @ IK.TOOL

    def carry_step(self):
        s = self.s
        s.arm_goal(self.q_hold, ARM_V)
        self.step()
        attached = s.d.qpos[s.arm_q['gripper']] < G_OPEN - 0.3 and self.phase in ('LIFT', 'CARRY')
        if attached:
            self.max_ball_tool = max(self.max_ball_tool, np.linalg.norm(s.ball() - s.grasp()))
        # Step 2.5 rule: lowest point of drone + arm (+ ball while held), for parts within 0.5 m of the bin
        low = s.lowest_point(include_ball=attached, near=self.bin0, radius=0.5)
        if low < 9: self.min_clear_bin = min(self.min_clear_bin, low - RIM)

    def finish(self):
        s = self.s; m = self.m
        b = s.ball(); binp = s.bin()
        bl = quat_to_mat(s.d.xquat[s.bin_b]).T @ (b - binp)
        inside = abs(bl[0]) < 0.165 - 0.0213 and abs(bl[1]) < 0.165 - 0.0213 and b[2] < RIM and b[2] < 0.1 and b[2] > 0.0
        m['ball_final'] = [round(float(x), 3) for x in b]
        m['ball_in_bin'] = bool(inside)
        m['bin_moved_cm'] = float(np.linalg.norm(binp[:2] - self.bin0[:2]) * 100)
        Rb = quat_to_mat(s.d.xquat[s.bin_b]); m['bin_tilt_deg'] = float(math.degrees(math.acos(min(1, Rb[2, 2]))))
        m['max_tilt_deg'] = self.max_tilt
        m['max_ball_tool_cm'] = self.max_ball_tool * 100
        m['min_clear_above_rim_cm'] = (self.min_clear_bin * 100) if self.min_clear_bin < 9 else None
        m['drone_ground_contact'] = s.drone_ground_contact
        m['t_total'] = round(s.t, 1)
        m['ok'] = bool(inside and m['fail'] is None and m['bin_moved_cm'] < 1 and m['bin_tilt_deg'] < 2)
        return m


def random_scene(seed, ball_fric=None, ball_mass=None):
    rng = np.random.default_rng(seed)
    bx, by = rng.uniform(-1.4, -0.6), rng.uniform(-0.5, 0.5)
    yaw0 = rng.uniform(0, 2 * math.pi)
    sx, sy = rng.uniform(0.5, 1.5), rng.uniform(-0.5, 0.5)
    fric = ball_fric if ball_fric is not None else rng.uniform(0.5, 1.5)
    mass = ball_mass if ball_mass is not None else 0.0459 * rng.uniform(0.5, 1.5)
    return dict(seed=seed, ball_pos=(bx, by), start_yaw=yaw0, start_xy=(sx, sy), ball_fric=fric, ball_mass=mass,
                bin_pos=(-3.0, 0.0))


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 1
    verbose = '-v' in sys.argv
    res = []
    for i in range(n):
        sc = random_scene(i)
        sim = Sim(seed=i, ball_pos=sc['ball_pos'], bin_pos=sc['bin_pos'], start_yaw=sc['start_yaw'], start_xy=sc['start_xy'],
                  ball_fric=sc['ball_fric'], ball_mass=sc['ball_mass'])
        t0 = time.time()
        p = Pick(sim, verbose)
        m = p.run()
        print(f"{i:3d} ok={m['ok']!s:5} fail={m['fail']} moved={m.get('ball_moved_cm_at_ground', -1):.2f}cm grip={m.get('grip_joint_closed', 0):.2f} "
              f"ball-site<={m['max_ball_tool_cm']:.1f}cm inbin={m['ball_in_bin']} bin_mv={m['bin_moved_cm']:.2f}cm tilt<={m['max_tilt_deg']:.0f} "
              f"clear={m['min_clear_above_rim_cm']} T={m['t_total']}s fric={sc['ball_fric']:.2f} mass={sc['ball_mass']*1000:.0f}g")
        res.append(m)
    print('SUCCESS', sum(r['ok'] for r in res), '/', len(res))
