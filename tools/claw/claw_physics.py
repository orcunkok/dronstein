"""The Hiwonder claw's MuJoCo parts, made from the same numbers as its look (claw_shape.py), for build_x500v2.py to put
under the X500's body.

- The holder (back plate, two flanges, the servo) is geoms on the X500's own body.
- Each finger is a body hinged at its pivot, with the finger's plates as boxes along the traced band (the two outer
  plates over the whole finger, the two inner ones from INNER_START) and a cylinder for each round tip. A finger is
  built in its closed pose but its body sits turned to the open one, so the joint's zero is fully open (the pose the
  scene starts in) and its range runs from there to closed.
- One position actuator per finger: a servo with a torque limit, so a ball stops the fingers and is squeezed with
  that torque, not with whatever it takes.

The claw hangs at `pos` in the Pod frame (Claw entity Translation) turned 90 degrees about Y (Rotation 0 90 0): its x
axis is the Pod -z, its z axis the Pod +x, so the fingers open across the drone. `to_mj` maps a Pod point to the MJCF
body frame.
"""

import math

from claw_shape import (BRACKET_BOTTOM, BRACKET_TOP, BRACKET_WIDTH, FLANGE_T, FLANGE_Z, HALF_IN, HALF_OUT, INNER_START,
                        LAYER_Z, OPEN_DEG, PATH_STEP, PIVOT_X, PLATE_T, SERVO_BOX, TIP_CAP, finger_path)

D_ALU = 2700.0          # hard aluminium
D_SERVO = 1500.0
STRIDE = 8              # samples (0.75 mm each) per collision box along a finger
SERVO_TORQUE = 1.5      # N m each finger can push with: well under the LDX-335MG's stall, for a firm but safe grip
SERVO_KP = 6.0          # N m per rad of error
SERVO_KV = 0.06         # N m s per rad: damps the finger's swing


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def quat_wxyz(ex, ey, ez):
    """A rotation given by its columns (the box's own axes in the body frame), as MuJoCo's w x y z."""
    m = [[ex[0], ey[0], ez[0]], [ex[1], ey[1], ez[1]], [ex[2], ey[2], ez[2]]]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1) * 2
        q = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    return q


def claw_mjcf(pos, to_mj, body):
    """The claw's parts for the MJCF body named `body`. Returns (geoms for the body itself, finger bodies, actuators);
    geoms are `<geom .../>` strings (the caller adds its common attributes), a finger body is (opening tag, joint,
    geoms), an actuator a string."""
    def pod_point(c):                       # a point in the claw's frame -> Pod frame
        return (pos[0] + c[2], pos[1] + c[1], pos[2] - c[0])

    def pod_dir(c):                         # a direction
        return (c[2], c[1], -c[0])

    def mj_dir(c):
        return tuple(to_mj(pod_dir(c)))

    def mj_point(c):
        return tuple(to_mj(pod_point(c)))

    ax, ay, az = mj_dir((1, 0, 0)), mj_dir((0, 1, 0)), mj_dir((0, 0, 1))   # the claw's axes in the body frame

    def holder_box(name, centre, size, density):   # an axis-aligned box in the claw's frame
        return (f'<geom name="{body}_claw_{name}" type="box" size="{fmt(s / 2 for s in size)}" '
                f'pos="{fmt(mj_point(centre))}" quat="{fmt(quat_wxyz(ax, ay, az))}" density="{density:g}"/>')

    reach = FLANGE_Z + FLANGE_T / 2
    mid = (BRACKET_TOP + BRACKET_BOTTOM) / 2
    geoms = [
        holder_box("back_plate", (0.0, BRACKET_TOP + FLANGE_T / 2, 0.0), (BRACKET_WIDTH, FLANGE_T, 2 * reach), D_ALU),
        holder_box("flange_0", (0.0, mid, -FLANGE_Z), (BRACKET_WIDTH, BRACKET_TOP - BRACKET_BOTTOM, FLANGE_T), D_ALU),
        holder_box("flange_1", (0.0, mid, FLANGE_Z), (BRACKET_WIDTH, BRACKET_TOP - BRACKET_BOTTOM, FLANGE_T), D_ALU),
        holder_box("servo", SERVO_BOX[1], SERVO_BOX[0], D_SERVO),
    ]

    # The finger's band: its centre and half width at each sample, in the finger's own (x, y) (+x inward for the left).
    path, _ = finger_path(PATH_STEP)
    n = len(path) - 1
    centres, widths = [], []
    for i, (x, y) in enumerate(path):
        (bx, by), (cx, cy) = path[max(0, i - 2)], path[min(n, i + 2)]
        t = math.hypot(cx - bx, cy - by)
        tx, ty = (cx - bx) / t, (cy - by) / t
        half_in = HALF_IN[i] * 1e-4
        outer = (x + ty * HALF_OUT, y - tx * HALF_OUT)
        inner = (x - ty * half_in, y + tx * half_in)
        centres.append(((outer[0] + inner[0]) / 2, (outer[1] + inner[1]) / 2))
        widths.append((HALF_OUT + half_in) / 2)

    bodies, actuators = [], []
    opening = math.radians(OPEN_DEG)
    for side, tag in ((1, "l"), (-1, "r")):      # the left finger's pivot is at claw x -PIVOT_X, the right's at +PIVOT_X
        name = f"{body}_claw_{tag}"

        def local(x, y, z):                      # finger (x, y, z) from the pivot -> the body frame's offset, closed pose
            return mj_dir((side * x, y, z))

        axis = mj_dir((0, 0, -side))             # swinging open is a positive turn about this
        s2, c2 = math.sin(opening / 2), math.cos(opening / 2)
        quat = (c2, axis[0] * s2, axis[1] * s2, axis[2] * s2)
        pivot = mj_point((-side * PIVOT_X, 0.0, 0.0))
        joint = (f'<joint name="{name}_hinge" type="hinge" axis="{fmt(axis)}" range="{-OPEN_DEG:.6g} 0" '   # degrees
                 f'damping="0.002" armature="0.0001"/>')
        parts = []
        for z, first in ((LAYER_Z[0], 0), (LAYER_Z[1], INNER_START), (LAYER_Z[2], INNER_START), (LAYER_Z[3], 0)):
            for k in range(first, n, STRIDE):
                i, j = k, min(k + STRIDE, n)
                (x0, y0), (x1, y1) = centres[i], centres[j]
                length = math.hypot(x1 - x0, y1 - y0)
                half_w = sum(widths[i:j + 1]) / (j - i + 1)
                ez = unit(mj_dir((side * (x1 - x0), y1 - y0, 0.0)))
                ex = unit(az)                      # the plate's normal: the claw's z
                ey = cross(ez, ex)
                parts.append(f'<geom name="{name}_plate_{len(parts)}" type="box" '
                             f'size="{fmt((PLATE_T / 2, half_w, length / 2 + 0.0005))}" '
                             f'pos="{fmt(local((x0 + x1) / 2, (y0 + y1) / 2, z))}" '
                             f'quat="{fmt(quat_wxyz(ex, ey, ez))}" density="{D_ALU:g}"/>')
        (tx, ty), tip_r = TIP_CAP
        for z in (LAYER_Z[0], LAYER_Z[3]):         # the round fingertip, one disc per outer plate
            parts.append(f'<geom name="{name}_tip_{len(parts)}" type="cylinder" size="{tip_r:g}" '
                         f'fromto="{fmt(local(tx, ty, z - PLATE_T / 2))} {fmt(local(tx, ty, z + PLATE_T / 2))}" '
                         f'density="{D_ALU:g}"/>')
        bodies.append((f'<body name="{name}" pos="{fmt(pivot)}" quat="{fmt(quat)}">', joint, parts))
        actuators.append(f'<position name="{name}" joint="{name}_hinge" kp="{SERVO_KP:g}" kv="{SERVO_KV:g}" '
                         f'ctrllimited="true" ctrlrange="{-opening:.6g} 0" forcelimited="true" '
                         f'forcerange="{-SERVO_TORQUE:g} {SERVO_TORQUE:g}"/>')
    return geoms, bodies, actuators
