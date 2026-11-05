"""Arm kinematics/IK in the DRONE frame (drone level at origin). Finger (jaw -y axis) points straight down.
Joint order: pan, lift, elbow, wrist_flex, roll.  Tool point = ball centre in the Fixed_Jaw frame."""
import numpy as np, mujoco
from dronelib import load_model, ARM
import copy
M = load_model(); D = mujoco.MjData(M)
QA = [M.joint(n).qposadr[0] for n in ARM]
LO = np.array([M.jnt_range[M.joint(n).id][0] for n in ARM]); HI = np.array([M.jnt_range[M.joint(n).id][1] for n in ARM])
FJ = M.body('Fixed_Jaw').id
TOOL = np.array([-0.030, -0.0857, -0.006])      # ball centre in jaw frame (fixed jaw body frame), metres
D.qpos[:] = M.qpos0; D.qpos[0:7] = [0, 0, 0, 1, 0, 0, 0]

def fk(q4, roll, tool=None, grip=-0.174):
    tool = TOOL if tool is None else tool
    for i in range(4): D.qpos[QA[i]] = q4[i]
    D.qpos[QA[4]] = roll; D.qpos[QA[5]] = grip
    mujoco.mj_kinematics(M, D)
    R = D.xmat[FJ].reshape(3, 3)
    return D.xpos[FJ] + R @ tool, -R[:, 1]    # tool point, finger direction (should be -z)

def ik(target, roll, q0=None, tool=None, iters=60):
    """Newton on [pan,lift,elbow,wrist] so that tool point == target and finger dir == (0,0,-1)."""
    q = np.array(q0 if q0 is not None else [0, -1.3, 1.5, -0.2], float)
    for _ in range(iters):
        def res(qq):
            p, f = fk(qq, roll, tool)
            return np.array([*(p - target), f[0], f[1]])      # 5 equations, 4 unknowns -> least squares
        r = res(q)
        if np.linalg.norm(r) < 1e-6: break
        Jm = np.zeros((5, 4))
        for i in range(4):
            dq = np.zeros(4); dq[i] = 1e-6
            Jm[:, i] = (res(q + dq) - r) / 1e-6
        step = np.linalg.lstsq(Jm, -r, rcond=None)[0]
        n = np.linalg.norm(step)
        if n > 0.3: step *= 0.3 / n
        q = np.clip(q + step, LO[:4], HI[:4])
    r = res(q)
    return q, np.linalg.norm(r)

if __name__ == '__main__':
    for roll in (1.5708, -1.5708):
        print('roll', roll)
        for H in (0.30, 0.35, 0.40, 0.50):
            for Dx in (0.25, 0.30, 0.35, 0.40, 0.45):
                tgt = np.array([-Dx, 0.0, 0.02135 - H])
                best = None
                for g in ([0,-1.3,1.5,-0.2],[0,-2.0,2.0,0.0],[0,-0.8,1.2,-0.5],[0,-1.8,1.0,0.8],[0,-2.5,2.5,0.5]):
                    q, e = ik(tgt, roll, g)
                    if e < 1e-4 and (best is None or True): best = (q, e); break
                print(H, Dx, None if best is None else (best[0].round(2), round(best[1], 6)))


def ik3(target, roll, tool=None):
    """3 unknowns (pan, lift, elbow); wrist = -pi/2 - lift - elbow keeps the finger vertical.
    Returns the solution with the most joint-limit margin, or None."""
    sols = []
    for l0 in np.arange(-3.2, 0.1, 0.2):
        for e0 in np.arange(0.0, 3.1, 0.25):
            q = np.array([0.0, l0, e0])
            ok = False
            for _ in range(40):
                def res(qq):
                    p, f = fk([qq[0], qq[1], qq[2], -np.pi/2 - qq[1] - qq[2]], roll, tool)
                    return p - target
                r = res(q)
                if np.linalg.norm(r) < 1e-7: ok = True; break
                Jm = np.zeros((3, 3))
                for i in range(3):
                    dq = np.zeros(3); dq[i] = 1e-6
                    Jm[:, i] = (res(q + dq) - r) / 1e-6
                try: step = np.linalg.solve(Jm, -r)
                except np.linalg.LinAlgError: break
                n = np.linalg.norm(step)
                if n > 0.3: step *= 0.3 / n
                q = q + step
            if not ok: continue
            q4 = np.array([q[0], q[1], q[2], -np.pi/2 - q[1] - q[2]])
            if np.any(q4 < LO[:4] + 0.02) or np.any(q4 > HI[:4] - 0.02): continue
            margin = np.min(np.minimum(q4 - LO[:4], HI[:4] - q4)[1:])
            if not any(np.allclose(q4, s[0], atol=1e-3) for s in sols): sols.append((q4, margin))
    if not sols: return None
    return max(sols, key=lambda s: s[1])


# ---------------- analytic IK (code-ready for the C# port) ----------------
def _pos0(roll, tool=None):
    for i in range(4): D.qpos[QA[i]] = 0
    D.qpos[QA[4]] = roll; D.qpos[QA[5]] = -0.174
    mujoco.mj_kinematics(M, D)
    b = lambda n: D.xpos[M.body(n).id].copy()
    R = D.xmat[FJ].reshape(3, 3)
    return b('Rotation_Pitch'), b('Upper_Arm'), b('Lower_Arm'), b('Wrist_Pitch_Roll'), D.xpos[FJ] + R @ (TOOL if tool is None else tool)

def Ry(t): c, s = np.cos(t), np.sin(t); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def analytic_consts(roll=1.5708):
    c, S, E0, W0, T0 = _pos0(roll)
    return dict(c=c, S=S, E0=E0, W0=W0, T0=T0, l1=np.linalg.norm(E0 - S), l2=np.linalg.norm(W0 - E0))

def ik_analytic(tool, roll=1.5708, elbow_sign=+1, consts=None, alpha=0.0, tool_pt=None):
    """tool: ball-centre point in the drone frame. Returns [pan, lift, elbow, wrist] or None.
    Finger kept vertical: lift+elbow+wrist = -pi/2."""
    K = consts or analytic_consts(roll)
    c, S, E0, W0, T0 = K['c'], K['S'], K['E0'], K['W0'], K['T0']
    if tool_pt is not None:   # tool point in jaw frame; recompute T0 for it
        T0 = _pos0(roll, tool_pt)[4]
    phi_sum = -np.pi/2 + alpha          # lift+elbow+wrist: -pi/2 = finger straight down, +alpha tilts the tip toward the nose
    w_off = Ry(phi_sum) @ (T0 - W0)          # tool - wrist at pan=0, finger vertical (lateral comp = a)
    a = w_off[1]
    d = np.array([tool[0] - c[0], tool[1] - c[1]])
    rr2 = d @ d - a * a
    if rr2 < 0: return None
    px = c[0] - np.sqrt(rr2)                  # tool x at pan=0 (nose side = -x)
    # tool(pan) = c + Rz(-pan) (P - c); P-c = (px-cx, a)
    ang_t = np.arctan2(d[1], d[0]); ang_p = np.arctan2(a, px - c[0])
    pan = ang_p - ang_t       # Rz(-pan) rotates by -pan: ang_t = ang_p - pan
    Wt = np.array([px, 0, tool[2]]) - np.array([w_off[0], 0, w_off[2]])   # wrist target in plane (y ignored)
    # 2-link in the x-z plane from S to W with Ry rotation: vectors E0-S (length l1), W0-E0 (l2)
    v = np.array([Wt[0] - S[0], Wt[2] - S[2]]); r = np.linalg.norm(v)
    l1, l2 = K['l1'], K['l2']
    if r > l1 + l2 or r < abs(l1 - l2): return None
    cosb = (r*r - l1*l1 - l2*l2) / (2*l1*l2)      # angle between link directions
    # Directions of rest vectors in x-z plane
    def ang(vec): return np.arctan2(vec[2], vec[0])   # angle in (x,z)
    a1_0 = ang(E0 - S); a2_0 = ang(W0 - E0)
    # Ry(t) rotates (x,z) vector by angle -t in (x,z)-polar terms: new angle = ang - t
    # link1 dir angle A1 = a1_0 - lift ; link2 dir angle A2 = a2_0 - (lift+elbow)
    # relative angle A2 - A1 = (a2_0 - a1_0) - elbow  =: beta, cos(beta) = cosb (two signs)
    beta = elbow_sign * np.arccos(np.clip(cosb, -1, 1))
    elbow = (a2_0 - a1_0) - beta
    # A1 from geometry: W-S = l1 e^{iA1} + l2 e^{i(A1+beta)}
    ang_v = np.arctan2(v[1], v[0])
    A1 = ang_v - np.arctan2(l2*np.sin(beta), l1 + l2*np.cos(beta))
    lift = a1_0 - A1
    wrist = phi_sum - lift - elbow
    wrap = lambda x: (x + np.pi) % (2*np.pi) - np.pi
    q = np.array([wrap(pan), wrap(lift), wrap(elbow), wrap(wrist)])
    return q

if __name__ == '__main__' and False:
    pass


def robot_lowest(q5, grip=0.9):
    """lowest z of the arm's collision geoms for joint angles q5 (pan..roll), drone level at origin."""
    for i in range(5): D.qpos[QA[i]] = q5[i]
    D.qpos[QA[5]] = grip
    mujoco.mj_kinematics(M, D)
    lo = 9.0
    for g in range(M.ngeom):
        if M.geom_group[g] != 3 or M.geom_contype[g] + M.geom_conaffinity[g] == 0: continue
        if M.body_rootid[M.geom_bodyid[g]] != M.body('x2').id: continue
        R = D.geom_xmat[g].reshape(3, 3)
        if M.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            me = M.geom_dataid[g]; adr, n = M.mesh_vertadr[me], M.mesh_vertnum[me]
            V = M.mesh_vert[adr:adr+n]
            lo = min(lo, ((V @ R.T)[:, 2] + D.geom_xpos[g][2]).min())
        else:
            lo = min(lo, D.geom_xpos[g][2] - np.sum(np.abs(R[2, :]) * M.geom_size[g]))
    return lo
