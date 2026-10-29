"""Build the Holybro X500 V2: its look as a Lucky procedural graph (Pod), its physics as MJCF.

Run with the editor open (Edit mode):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 build_x500v2.py <new output dir>

Writes into <out>:
- x500v2.hpodgraph: the visual mesh only. The scene uses it from Assets/Robots/X500V2/source/x500v2.hpodgraph.
- x500v2.mjcf: the MuJoCo body (free joint, collision geoms with the parts' masses) and six motors on its free joint
  that X500Flight.cs drives. Lucky can't push a procedural body (docs/needs-AddBodyForceWorld.md), so the physics
  is MJCF: dronstein.xml includes it from Assets/Robots/X500V2/x500v2.mjcf, and the HolybroX500V2 entity follows it.
After a rebuild copy both over those files and refresh assets.

Every part below is one entry that makes both the visual mesh and its collision geom from the same numbers, so the
colliders sit exactly on the visible tubes and plates.

Frame: Pod/Lucky, Y up, +X forward (front arms at +X), sides along Z, metres. The MJCF body frame is the same frame
turned to MuJoCo's Z up: x forward, y left, z up. Origin is the frame centre, half way between the two plates.

Sources:
- Holybro X500 V2 spec: 500 mm wheelbase, 144x144x2 mm plates 28 mm apart,
  16 mm arm tubes, 16 mm and 10 mm landing gear tubes, 2216 KV920 motors, 1045 props.
- Landing gear, rails and foam measured from PX4's x500_base frame mesh
  (NXP-HGD-CF.dae): legs splay 20 deg outward, skids 250 mm long at +-132.6 mm,
  20 mm foam sleeves, feet 226 mm below the bottom plate.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from pod_authoring import Literal, McpSession

ENDPOINT = "http://127.0.0.1:19331/mcp"
BODY = "x500v2"
SPAWN = (-5.0, 0.0, 0.2423)   # MuJoCo world: resting on the ground (no drop: it twists landing), clear of the golf balls


def mat(r, g, b, metal, rough):
    # Linear RGBA, metallic, roughness. LuckyEngine refuses authored IOR, so no coat channels.
    return Literal("Material", [r, g, b, 1.0, metal, rough])


CARBON = mat(0.018, 0.018, 0.020, 0.0, 0.28)
BLUE = mat(0.02, 0.38, 0.78, 0.0, 0.45)
BLACK = mat(0.02, 0.02, 0.022, 0.0, 0.55)
MOTOR = mat(0.03, 0.03, 0.035, 0.85, 0.35)
METAL = mat(0.62, 0.62, 0.64, 1.0, 0.25)
FOAM = mat(0.025, 0.025, 0.025, 0.0, 0.9)
RED = mat(0.55, 0.02, 0.02, 0.0, 0.8)
PROP = mat(0.02, 0.02, 0.02, 0.0, 0.4)
GPS = mat(0.05, 0.05, 0.055, 0.0, 0.3)
BATTERY = mat(0.35, 0.36, 0.38, 0.0, 0.5)

# Effective densities (kg/m^3) for solid colliders standing in for real parts.
D_PLATE = 1600.0          # solid 2 mm carbon plate
D_TUBE16 = 375.0          # 16/14 mm carbon tube as a solid 16 mm cylinder
D_TUBE10 = 575.0          # 10/8 mm carbon tube as a solid 10 mm cylinder
D_PLASTIC = 600.0         # moulded clamps, mostly hollow
D_FOAM = 250.0            # foam sleeve including the tube inside
D_MOTOR = 3700.0          # 2216 motor, 68 g
D_PROP = 630.0            # 1045 prop, 12 g
D_ELEC = 950.0            # flight controller, GPS puck


# ---------------------------------------------------------------- rotations
def norm(v):
    s = math.sqrt(sum(c * c for c in v))
    return [c / s for c in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def frame_y(d):
    """Rotation matrix whose Y column is d (the axis of a Y-up cylinder)."""
    y = norm(d)
    helper = [1.0, 0.0, 0.0] if abs(y[0]) < 0.9 else [0.0, 0.0, 1.0]
    z = norm(cross(helper, y))
    x = cross(y, z)
    return [[x[i], y[i], z[i]] for i in range(3)]


def yaw(a):
    """Rotation about Y; positive turns +Z toward +X (Pod convention)."""
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def roll(a):
    """Rotation about X; positive turns +Y toward +Z."""
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def euler_xyz(r):
    # Transform applies X, then Y, then Z: R = Rz * Ry * Rx.
    ry = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(r[2][0]) > 0.99999:             # gimbal lock: put it all in X, keep Z at zero
        s = math.sin(ry)
        return [math.atan2(s * r[0][1], s * r[0][2]), ry, 0.0]
    return [math.atan2(r[2][1], r[2][2]), ry, math.atan2(r[1][0], r[0][0])]


def quat_xyzw(r):
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, 0.25 * s]
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        q = [0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s, (r[2][1] - r[1][2]) / s]
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        q = [(r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s, (r[0][2] - r[2][0]) / s]
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        q = [(r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s, (r[1][0] - r[0][1]) / s]
    return norm(q)


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
TO_MJ = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]   # Pod (x, y, z) -> MJCF body (x, -z, y)


def to_mj(p):
    return [sum(TO_MJ[i][j] * p[j] for j in range(3)) for i in range(3)]


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


# ---------------------------------------------------------------- parts
class Builder:
    def __init__(self, edit):
        self.edit = edit
        self.meshes = []
        self.colliders = []
        self.mass = 0.0

    def _place(self, geometry, name, rot, pos, material):
        e = self.edit
        placed = e.node("Transform", f"{name} pose", Geometry=geometry.output("Geometry"),
                        Rotation=euler_xyz(rot), Translation=list(pos))
        shaded = e.node("SetMaterial", f"{name} material", Geometry=placed.output("Geometry"),
                        Material=material, Name=name)
        self.meshes.append(e.node("SetMeshName", name, Geometry=shaded.output("Geometry"), Name=name))

    def box(self, name, size, pos, material, density, rot=IDENTITY, collide=True, bevel=0.0008):
        e = self.edit
        cube = e.node("MeshCube", name, SizeX=size[0], SizeY=size[1], SizeZ=size[2])
        # A bare cube carries Vertex-domain UVs; bevelling drops them (see recipes gotcha).
        rounded = e.node("BevelEdge", f"{name} edges", Geometry=cube.output("Geometry"),
                         Offset=bevel, Segments=2)
        self._place(rounded, name, rot, pos, material)
        if collide:
            x, y, z, w = quat_xyzw(mul(TO_MJ, rot))   # the box's own axes, seen from the MJCF body frame
            self.colliders.append(f'<geom name="{BODY}_{name}" type="box" size="{fmt(s / 2 for s in size)}" '
                                  f'pos="{fmt(to_mj(pos))}" quat="{fmt((w, x, y, z))}" density="{density:g}"/>')
            self.mass += density * size[0] * size[1] * size[2]

    def tube(self, name, p0, p1, radius, material, density, sides=24, collide=True):
        """Cylinder from p0 to p1; visual and collider share the same axis and radius."""
        d = [b - a for a, b in zip(p0, p1)]
        length = math.sqrt(sum(c * c for c in d))
        mid = [(a + b) / 2 for a, b in zip(p0, p1)]
        rot = frame_y(d)
        e = self.edit
        cyl = e.node("MeshCylinder", name, Sides=sides, Radius=radius, Height=length,
                     CapFill=Literal("Menu", "Ngon"))
        # Small rim bevel; also drops the cylinder's UVs so merging stays safe.
        rounded = e.node("BevelEdge", f"{name} rims", Geometry=cyl.output("Geometry"),
                         Offset=min(0.0006, radius * 0.15), Segments=2, AngleThreshold=0.6)
        self._place(rounded, name, rot, mid, material)
        if collide:
            self.colliders.append(f'<geom name="{BODY}_{name}" type="cylinder" size="{radius:g}" '
                                  f'fromto="{fmt(to_mj(p0))} {fmt(to_mj(p1))}" density="{density:g}"/>')
            self.mass += density * math.pi * radius * radius * length

    def join(self):
        e = self.edit
        result = self.meshes[0]
        for piece in self.meshes[1:]:
            result = e.node("JoinGeometry", "assemble", A=result.output("Geometry"), B=piece.output("Geometry"))
        return result


# ---------------------------------------------------------------- the drone
PLATE = 0.144             # plate side
PLATE_T = 0.002
PLATE_Y = 0.015           # plate centres at +-15 mm: 28 mm clear gap
BOTTOM = -(PLATE_Y + PLATE_T / 2)   # underside of the bottom plate, -0.016
ARM_R = 0.008
MOTOR_RADIAL = 0.25       # 500 mm wheelbase / 2
LEG_TILT = math.radians(20.0)

# Landing gear, one side (+Z); measured from the PX4 frame mesh relative to the bottom plate.
LEG_TOP = (0.0547, BOTTOM - 0.0026)          # (lateral, y) where the leg leaves its clamp
SKID_LAT = 0.1326
SKID_Y = BOTTOM - 0.2162                     # skid tube axis
FOAM_R = 0.010
GROUND = SKID_Y - FOAM_R                     # lowest point: foam and T-joint bottoms
LEG_BOTTOM_Y = SKID_Y + 0.040                # leg tube ends inside the T-joint socket


def build(edit):
    b = Builder(edit)

    # Centre plates and electronics.
    for name, y in (("top_plate", PLATE_Y), ("bottom_plate", -PLATE_Y)):
        b.box(name, (PLATE, PLATE_T, PLATE), (0, y, 0), CARBON, D_PLATE, bevel=0.003)
    b.box("flight_controller", (0.084, 0.017, 0.044), (0.010, PLATE_Y + PLATE_T / 2 + 0.0085, 0), BLACK, D_ELEC,
          bevel=0.003)
    mast_x = -0.050
    b.tube("gps_mast", (mast_x, PLATE_Y + PLATE_T / 2, 0), (mast_x, 0.115, 0), 0.004, BLACK, 1200.0, sides=12)
    b.tube("gps_puck", (mast_x, 0.115, 0), (mast_x, 0.131, 0), 0.030, GPS, D_ELEC, sides=40)

    # Arms in X layout: front-right, front-left, back-left, back-right.
    for i, deg in enumerate((45, 135, 225, 315)):
        a = math.radians(deg)
        u = (math.cos(a), 0.0, math.sin(a))                  # radial direction in XZ
        arm_rot = frame_y(u)
        along = yaw(math.atan2(-u[2], u[0]))                 # box X along the arm
        at = lambda r, y=0.0: (u[0] * r, y, u[2] * r)
        b.box(f"arm_clamp_{i}", (0.034, 0.028, 0.030), at(0.094), BLUE, D_PLASTIC, rot=along, bevel=0.003)
        b.tube(f"arm_{i}", at(0.060), at(0.236), ARM_R, CARBON, D_TUBE16)
        b.box(f"motor_mount_{i}", (0.046, 0.022, 0.026), at(0.240), BLUE, D_PLASTIC, rot=along, bevel=0.003)
        b.tube(f"motor_plate_{i}", at(MOTOR_RADIAL, 0.011), at(MOTOR_RADIAL, 0.015), 0.016, BLUE, D_PLASTIC,
               sides=32)
        b.tube(f"motor_{i}", at(MOTOR_RADIAL, 0.015), at(MOTOR_RADIAL, 0.043), 0.0139, MOTOR, D_MOTOR, sides=32)
        b.tube(f"motor_shaft_{i}", at(MOTOR_RADIAL, 0.043), at(MOTOR_RADIAL, 0.056), 0.0025, METAL, 7800.0,
               sides=12, collide=False)
        b.tube(f"prop_hub_{i}", at(MOTOR_RADIAL, 0.044), at(MOTOR_RADIAL, 0.052), 0.008, PROP, D_PROP, sides=20)
        # 1045 prop: two blades, 127 mm tip radius, pitched ~12 deg. Each pair turned differently for looks.
        spin = math.radians(30 + 50 * i)
        for k, side in enumerate((1.0, -1.0)):
            blade_dir = (math.cos(spin) * side, 0.0, math.sin(spin) * side)
            centre = (u[0] * MOTOR_RADIAL + blade_dir[0] * 0.068, 0.048,
                      u[2] * MOTOR_RADIAL + blade_dir[2] * 0.068)
            rot = mul(yaw(math.atan2(-blade_dir[2], blade_dir[0])), roll(math.radians(12) * side))
            b.box(f"prop_{i}_{k}", (0.118, 0.004, 0.020), centre, PROP, D_PROP, rot=rot, bevel=0.0015)

    # Rails under the frame (dual 10 mm x 250 mm) with two hanger blocks and the battery board.
    rail_y = BOTTOM - 0.0133
    for k, z in enumerate((-0.030, 0.030)):
        b.tube(f"rail_{k}", (-0.096, rail_y, z), (0.154, rail_y, z), 0.005, CARBON, D_TUBE10)
    for k, x in enumerate((-0.035, 0.055)):
        b.box(f"rail_hanger_{k}", (0.016, 0.016, 0.082), (x, BOTTOM - 0.008, 0), BLACK, D_PLASTIC, bevel=0.002)
    b.box("battery_board", (0.150, 0.002, 0.090), (0.029, rail_y - 0.0075, 0), CARBON, D_PLATE, bevel=0.002)
    # 4S 3300 mAh pack, 0.36 kg, strapped under the board. With it the model weighs what the X500v2 frame
    # params' hover throttle (0.29 of 4 x 1.15 kgf) lifts: about 1.33 kg.
    battery = (0.135, 0.030, 0.043)
    b.box("battery", battery, (0.029, rail_y - 0.0085 - battery[1] / 2, 0), BATTERY,
          0.36 / (battery[0] * battery[1] * battery[2]), bevel=0.004)

    # Landing gear: per side one 16 mm leg splayed 20 deg, a T-joint, a 10 mm skid with foam and caps.
    for s, side in ((1.0, "left"), (-1.0, "right")):
        lat0, y0 = LEG_TOP
        # Leg axis runs from the clamp down and outward, passing through the skid axis.
        dlat, dy = math.sin(LEG_TILT), -math.cos(LEG_TILT)
        t_bottom = (LEG_BOTTOM_Y - y0) / dy
        leg_top = (0.0, y0, s * lat0)
        leg_bot = (0.0, LEG_BOTTOM_Y, s * (lat0 + dlat * t_bottom))
        leg_dir = (0.0, dy, s * dlat)
        tilt = frame_y(leg_dir)
        b.box(f"leg_clamp_{side}", (0.039, 0.036, 0.030),
              (0.0, BOTTOM - 0.018, s * (lat0 + 0.002)), BLACK, D_PLASTIC, rot=tilt, bevel=0.003)
        b.tube(f"leg_{side}", leg_top, leg_bot, 0.008, CARBON, D_TUBE16)
        # T-joint: a sleeve round the skid plus a socket up the leg.
        t_skid = (SKID_Y - y0) / dy
        skid_hit = (0.0, SKID_Y, s * (lat0 + dlat * t_skid))
        socket_top = (0.0, SKID_Y + 0.048, s * (lat0 + dlat * (SKID_Y + 0.048 - y0) / dy))
        b.tube(f"tjoint_socket_{side}", skid_hit, socket_top, 0.0105, BLACK, D_PLASTIC)
        b.tube(f"tjoint_sleeve_{side}", (-0.020, SKID_Y, s * SKID_LAT), (0.020, SKID_Y, s * SKID_LAT),
               FOAM_R, BLACK, D_PLASTIC)
        b.tube(f"skid_{side}", (-0.112, SKID_Y, s * SKID_LAT), (0.112, SKID_Y, s * SKID_LAT), 0.005, CARBON,
               D_TUBE10)
        for k, (x0, x1) in enumerate(((0.022, 0.112), (-0.112, -0.022))):
            b.tube(f"foam_{side}_{k}", (x0, SKID_Y, s * SKID_LAT), (x1, SKID_Y, s * SKID_LAT), FOAM_R, FOAM,
                   D_FOAM)
            for j in range(4):                                   # red stripes, visual only
                x = x0 + (x1 - x0) * (j + 0.5) / 4
                b.tube(f"foam_stripe_{side}_{k}_{j}", (x - 0.002, SKID_Y, s * SKID_LAT),
                       (x + 0.002, SKID_Y, s * SKID_LAT), FOAM_R + 0.0004, RED, 0.0, collide=False)
            cap = 0.112 + 0.0148 if x1 > 0 else -0.112 - 0.0148
            edge = x1 if x1 > 0 else x0
            b.tube(f"end_cap_{side}_{k}", (edge, SKID_Y, s * SKID_LAT), (cap, SKID_Y, s * SKID_LAT), 0.0055,
                   BLACK, 1100.0)

    edit.output(b.join(), "Geometry")
    return b


def mjcf(colliders):
    """The X500's MuJoCo body and the six motors X500Flight.cs sets: force along world x/y/z (N) and torque about
    the body's x/y/z (N m). Collision geoms are invisible: the procedural mesh is the look."""
    # fluidcoef 0 turns MuJoCo's own air drag (dronstein.xml sets density) off: X500Flight.cs owns the X500's drag.
    geoms = "\n".join(f"      {g[:-2]} group=\"3\" rgba=\"0 0 0 0\" fluidshape=\"ellipsoid\" fluidcoef=\"0 0 0 0 0\"/>"
                       for g in colliders)
    axes = ("fx", "1 0 0 0 0 0"), ("fy", "0 1 0 0 0 0"), ("fz", "0 0 1 0 0 0"), \
           ("tx", "0 0 0 1 0 0"), ("ty", "0 0 0 0 1 0"), ("tz", "0 0 0 0 0 1")
    motors = "\n".join(f'    <motor name="{BODY}_{n}" joint="{BODY}_free" gear="{g}" ctrllimited="false"/>' for n, g in axes)
    return (f"<!-- Holybro X500 V2 physics, written by tools/x500v2/build_x500v2.py: edit that, not this. -->\n"
            f"<mujoco>\n  <worldbody>\n    <body name=\"{BODY}\" pos=\"{fmt(SPAWN)}\">\n"
            f"      <freejoint name=\"{BODY}_free\"/>\n{geoms}\n    </body>\n  </worldbody>\n"
            f"  <actuator>\n{motors}\n  </actuator>\n</mujoco>\n")


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    session = McpSession(ENDPOINT)
    graph = session.graph("Holybro X500 V2", directory=out)
    with graph.edit() as edit:
        b = build(edit)
    path = graph.save(out / "x500v2.hpodgraph")
    (out / "x500v2.mjcf").write_text(mjcf(b.colliders))
    print(f"saved {path} ({len(b.meshes)} meshes) and x500v2.mjcf ({len(b.colliders)} geoms, {b.mass:.3f} kg)")
    print(f"lowest point y={GROUND:.4f} m below the frame centre")


if __name__ == "__main__":
    main()
