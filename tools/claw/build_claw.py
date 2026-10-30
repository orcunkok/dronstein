"""Build the Hiwonder (LewanSoul) aluminium wave-edge robotic claw (Amazon B0CMLRGXP4) as Lucky procedural graphs.

Run with the editor open (Edit mode):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 build_claw.py <new output dir>

Writes into <out>:
- claw_bracket.hpodgraph: the U-shaped holder the fingers hang from: its back plate (the mounting face) and two
  flanges that carry the two pivot pins, and a plain box for the servo inside the U.
- claw_finger_l.hpodgraph, claw_finger_r.hpodgraph: the left and right fingers, pivot at the origin: four stacked
  2 mm plates, two inside the holder's flanges and two outside them. Each is a band along the traced CENTERLINE with
  the drawing's stepped inner edge and a round tip. The two outer plates reach the pivot (round hub); the two inner
  ones are a short ladder that starts at the first pillar, held to the outer two by the brass standoffs (a silver
  screw head at each end). The sector gear is a separate plate outside the top one. The right finger is the left one
  mirrored in x (built
  mirrored: a mirrored entity, scale X -1, shows its plates inside out). Each plate is its real outline: Lucky's
  CurveFromAnchors takes at most 8 points, so the outline is cut into strips and fans of up to 8 points that
  FillCurve fills one by one.
Bring them into the project with Lucky's import (pod_package, authoring_import): a graph already loaded is cached, so
copying over its file does not change the scene. Claw.cs swings the fingers.

Frame: Pod/Lucky, Y up, metres. The claw hangs fingers down: pivots on the X axis at +-PIVOT_X, pivot axes along Z,
the four plates of a finger stacked along Z.

Sources:
- Amazon B0CMLRGXP4 / Hiwonder SKU 21070010: max opening 230 mm, 136 x 57 x 60 mm, 191 g with its LDX-335MG servo,
  hard aluminium. The finger outline, screw and standoff positions, plate spacing, bracket and open angle are
  measured from the Hiwonder dimension drawing (closed view 3.72 px/mm; left pivot at the left hub's nut).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "x500v2"))
from build_x500v2 import ALU_BLACK, IDENTITY, Builder, METAL, mat  # noqa: E402
from pod_authoring import Literal, McpSession  # noqa: E402

ENDPOINT = "http://127.0.0.1:19331/mcp"
BRASS = mat(0.75, 0.52, 0.18, 1.0, 0.3)
SERVO = mat(0.07, 0.07, 0.075, 0.2, 0.5)   # a shade lighter than the black plates
D_ALU = 2700.0

# ---------------------------------------------------------------- dimensions (m)
PIVOT_X = 0.0128          # pivots 25.6 mm apart; their gears mesh
# The finger's centreline in the left finger's frame (+x inward, -y toward the tip), from the pivot up the neck and
# out along the curve to the tip screw: the outer edge of the drawing offset inward by half the band.
CENTERLINE = (
    (0.0, 0.0), (0.0001, -0.0135), (0.0002, -0.0162), (-0.0008, -0.0218), (-0.0038, -0.0262), (-0.006, -0.0293),
    (-0.008, -0.0322), (-0.0098, -0.0353), (-0.0113, -0.0381), (-0.0126, -0.0411), (-0.0137, -0.0438),
    (-0.0146, -0.0467), (-0.0154, -0.0497), (-0.0159, -0.0526), (-0.0163, -0.0554), (-0.0165, -0.0585),
    (-0.0167, -0.0614), (-0.0166, -0.0641), (-0.0164, -0.0671), (-0.0159, -0.07), (-0.0154, -0.073),
    (-0.0147, -0.076), (-0.0139, -0.0789), (-0.0129, -0.0817), (-0.0117, -0.0845), (-0.0102, -0.0874),
    (-0.0084, -0.0903), (-0.0064, -0.0932), (-0.0041, -0.0962), (-0.0014, -0.0992), (0.0056, -0.1062),
)
HALF_OUT = 0.006          # outer edge to the centreline
PATH_STEP = 0.00075       # the centreline is sampled every 0.75 mm along its length (157 points)
# Centreline to the inner edge at each sample, in 0.1 mm: 6 mm up the plain neck, then the drawing's stepped teeth
# (4 to 6.2 mm), measured along the band's normal.
HALF_IN = (
    60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 63, 63, 63, 63, 63, 60, 60, 59, 59, 62, 61, 63, 64, 63, 61,
    60, 56, 54, 49, 50, 50, 55, 61, 62, 56, 45, 40, 42, 46, 52, 57, 56, 52, 45, 41, 40, 42, 46, 53, 57, 55, 49, 45,
    43, 40, 44, 53, 58, 58, 53, 49, 43, 42, 44, 49, 54, 59, 61, 54, 47, 43, 41, 45, 49, 55, 59, 57, 52, 47, 42, 43,
    46, 55, 58, 61, 56, 50, 44, 41, 43, 48, 53, 58, 60, 53, 47, 42, 41, 43, 49, 55, 58, 56, 52, 45, 43, 44, 47, 52,
    57, 58, 55, 50, 44, 41, 44, 47, 50, 57, 58, 51, 47, 40, 40, 41, 47, 53, 56, 54, 49, 41, 39, 40, 43, 47, 51, 54,
    50, 43, 39, 39, 39, 40, 48, 54, 50, 47, 40, 40, 44, 51, 59, 62, 62,
)
TIP_CAP = ((0.0073, -0.1077), 0.0053)    # the round fingertip: centre and radius, offset from the tip screw
PLATE_T = 0.002           # plate thickness
FLANGE_Z, FLANGE_T = 0.024, 0.002     # the holder's two flanges (centres) and their thickness: 50 mm apart outside
LAYER_Z = (-0.029, -0.0098, 0.0098, 0.029)   # finger plates: the outer two just outside the flanges, 60 mm overall
STANDOFF_R = 0.0028       # brass hex standoffs between the plates
SERVO_BOX = ((0.0545, 0.020, 0.0405), (0.0, 0.0075, 0.0))   # the LDX-335MG's body inside the holder: size, centre
SCREW_HEAD = (0.00275, 0.0016)     # M3 button head screwed into each standoff end: radius, height
STANDOFFS = ((-0.0043, -0.0253), (-0.0172, -0.0616), (0.0056, -0.1062))   # the three screws, on the centreline
HUB_R = 0.0102            # plate round the pivot
INNER_START = 29          # the two inner plates start at this sample (21.8 mm along the band): they only reach the first pillar
GEAR_TEETH, GEAR_ROOT, GEAR_TIP = 16, 0.0113, 0.0134   # sector gear on the top plate (the two gears mesh)
GEAR_SECTOR = (-20.0, 110.0)       # degrees, in the left finger's frame: the gear's teeth span the lower inner side
OPEN_DEG = 72.3           # each finger's swing from closed to fully open, measured on the drawing (Claw.cs uses it)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def finger_path(step):
    """CENTERLINE resampled every step metres along its length. Returns the points and the length."""
    seg = [math.dist(a, b) for a, b in zip(CENTERLINE, CENTERLINE[1:])]
    total = sum(seg)
    n = max(1, round(total / step))
    points, k, done = [], 0, 0.0
    for i in range(n + 1):
        s = total * i / n
        while k < len(seg) - 1 and done + seg[k] < s:
            done += seg[k]
            k += 1
        t = min(1.0, (s - done) / seg[k])
        (ax, ay), (bx, by) = CENTERLINE[k], CENTERLINE[k + 1]
        points.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return points, total


def outline(edit, name, pieces, edges, side):
    """A solid plate (PLATE_T thick, from z=0 up), x times side (+1 left finger, -1 right): its faces filled from
    pieces (polygons of 3..8 (x, y) points) and its walls swept along edges (polylines of (x, y) points)."""
    def curves(polylines, label, flip_y=1.0):
        joined = None
        for i, pts in enumerate(polylines):
            anchors = {f"Anchor{j}": [side * x, flip_y * y, 0.0] for j, (x, y) in enumerate(pts)}
            c = edit.node("CurveFromAnchors", f"{name} {label} {i}", AnchorCount=len(pts), **anchors)
            joined = c if joined is None else edit.node("JoinCurves", f"{name} {label} join {i}",
                                                        A=joined.output("Curves"), B=c.output("Curves"))
        return joined

    def fill(label, flip_y):
        return edit.node("FillCurve", f"{name} {label} fill", Curves=curves(pieces, label, flip_y).output("Curves"),
                         Plane=Literal("Menu", "XY"), Mode=Literal("Menu", "Triangles"))

    # FillCurve faces always face +z. The bottom must face -z: fill the outline mirrored in y and turn it half a
    # turn about x, which puts it back in place facing down (a scale of z -1 would not turn the faces round).
    top = edit.node("Transform", f"{name} top", Geometry=fill("top", 1.0).output("Geometry"),
                    Translation=[0.0, 0.0, PLATE_T])
    bottom = edit.node("Transform", f"{name} bottom", Geometry=fill("bottom", -1.0).output("Geometry"),
                       Rotation=[math.pi, 0.0, 0.0])
    # Walls: a thin closed rectangle swept along the edges, then lifted to z 0..PLATE_T. The sweep maps the profile's
    # x to z, up or down with the edge's direction, so the profile is centred on x=0. ExtrudeMesh would be simpler
    # but leaves a fringe of slivers along these filled pieces.
    chunks = [pts[i:i + 8] for pts in edges for i in range(0, len(pts) - 1, 7)]
    w = 0.00005
    profile = edit.node("CurveFromAnchors", f"{name} wall profile", AnchorCount=4, Anchor0=[-PLATE_T / 2, -w, 0.0],
                        Anchor1=[PLATE_T / 2, -w, 0.0], Anchor2=[PLATE_T / 2, w, 0.0], Anchor3=[-PLATE_T / 2, w, 0.0])
    spine = edit.node("SetCurveNormal", f"{name} wall spine", Curves=curves(chunks, "edge").output("Curves"),
                      Mode=Literal("Menu", "ZUp"))
    walls = edit.node("CurveToMesh", f"{name} walls", Curves=spine.output("Curves"),
                      ProfileCurve=profile.output("Curves"), FillCaps=True)
    lifted = edit.node("Transform", f"{name} walls up", Geometry=walls.output("Geometry"),
                       Translation=[0.0, 0.0, PLATE_T / 2])
    faces = edit.node("JoinGeometry", f"{name} faces", A=top.output("Geometry"), B=bottom.output("Geometry"))
    solid = edit.node("JoinGeometry", f"{name} solid", A=faces.output("Geometry"), B=lifted.output("Geometry"))
    return edit.node("SetShading", f"{name} flat", Geometry=solid.output("Geometry"), Mode=Literal("Menu", "Flat"))


def fan(cx, cy, radius_at, n):
    """A round outline about (cx, cy): its fan pieces and its rim (n rim points, radius_at(angle) each)."""
    rim = [(cx + radius_at(a) * math.cos(a), cy + radius_at(a) * math.sin(a))
           for a in (2 * math.pi * i / n for i in range(n + 1))]
    return [[(cx, cy), *rim[i:i + 7]] for i in range(0, n, 6)], rim


def plate_outline(start=0):
    """One finger plate: the band along the centreline with the stepped inner edge and the round tip. From the pivot
    (start 0) it has the round hub; the inner plates start at sample start instead, with a round end there. Returns its
    fill pieces and its edges."""
    path, length = finger_path(PATH_STEP)
    n = len(path) - 1
    outer, inner = [], []
    for i, (x, y) in enumerate(path):
        (ax, ay), (bx, by) = path[max(0, i - 2)], path[min(n, i + 2)]
        t = math.hypot(bx - ax, by - ay)
        tx, ty = (bx - ax) / t, (by - ay) / t      # the outward normal is (ty, -tx)
        half_in = HALF_IN[i] * 1e-4
        outer.append((x + ty * HALF_OUT, y - tx * HALF_OUT))
        inner.append((x - ty * half_in, y + tx * half_in))
    pieces = [outer[i:i + 4] + inner[i:i + 4][::-1] for i in range(start, n, 3)]
    hub, hub_rim = fan(*path[start], lambda a: HUB_R if start == 0 else HALF_OUT, 24)
    (tx, ty), tip_r = TIP_CAP
    tip, tip_rim = fan(tx, ty, lambda a: tip_r, 36)
    # The band's two end corners lie outside the tip circle: a triangle from each to the circle (the outer one to the
    # tangent point, so the outer edge runs on into the tip) closes the outline.
    ex, ey = outer[-1]
    d = math.hypot(tx - ex, ty - ey)
    base, half = math.atan2(ty - ey, tx - ex), math.asin(tip_r / d)
    tangent = min(((ex + math.sqrt(d * d - tip_r * tip_r) * math.cos(base + h),
                    ey + math.sqrt(d * d - tip_r * tip_r) * math.sin(base + h)) for h in (-half, half)),
                  key=lambda p: p[1])
    ix, iy = inner[-1]
    a = math.atan2(iy - ty, ix - tx)
    inner_end = (tx + tip_r * math.cos(a), ty + tip_r * math.sin(a))
    corners = [[(ex, ey), tangent, (tx, ty)], [(ix, iy), inner_end, (tx, ty)]]
    return pieces + hub + tip + corners, [outer[start:], inner[start:], hub_rim, tip_rim, [(ex, ey), tangent],
                                          [inner_end, (ix, iy)]]


def gear_outline(side):
    """The finger's sector gear on the top plate: GEAR_TEETH square-ish teeth over GEAR_SECTOR, the hub round elsewhere.
    The right gear is turned half a tooth so its teeth sit in the left gear's gaps."""
    n = GEAR_TEETH * 4
    shift = 0 if side > 0 else 2

    def radius(a):
        deg = math.degrees(a) % 360.0
        if deg > 180.0:
            deg -= 360.0
        if not GEAR_SECTOR[0] <= deg <= GEAR_SECTOR[1]:
            return HUB_R
        return GEAR_TIP if (int(round(a / (2 * math.pi) * n)) + shift) % 4 < 2 else GEAR_ROOT

    pieces, rim = fan(0.0, 0.0, radius, n)
    return pieces, [rim]


def build_finger(edit, side):
    b = Builder(edit)
    outer_plate = outline(edit, "plate", *plate_outline(), side)
    inner_plate = outline(edit, "inner plate", *plate_outline(INNER_START), side)
    for k, z in enumerate(LAYER_Z):
        b._place(inner_plate if k in (1, 2) else outer_plate, f"plate_{k}", IDENTITY, (0.0, 0.0, z - PLATE_T / 2),
                 ALU_BLACK)
    for j, (sx, sy) in enumerate(STANDOFFS):
        x, y = side * sx, sy
        b.tube(f"standoff_{j}", (x, y, LAYER_Z[0]), (x, y, LAYER_Z[-1]), STANDOFF_R, BRASS, 8500.0, sides=6,
               collide=False)
        r, h = SCREW_HEAD
        for k, (z, dz) in enumerate(((LAYER_Z[-1] + PLATE_T / 2, h), (LAYER_Z[0] - PLATE_T / 2, -h))):
            b.tube(f"screw_{j}_{k}", (x, y, z), (x, y, z + dz), r, METAL, 7800.0, sides=16, collide=False)
    b._place(outline(edit, "gear", *gear_outline(side), side), "gear", IDENTITY, (0.0, 0.0, LAYER_Z[-1] + PLATE_T / 2),
             ALU_BLACK)
    edit.output(b.join(), "Geometry")
    return b


def build_bracket(edit):
    b = Builder(edit)
    top, bottom, width = 0.0214, -0.0121, 0.057
    # The U: the back plate (mounting face, M3 holes not modelled) across the top, and the two flanges hanging from it
    # either side of the inner finger plates, with the pivot pins through them and out to the outer plates.
    reach = FLANGE_Z + FLANGE_T / 2
    b.box("back_plate", (width, FLANGE_T, 2 * reach), (0, top + FLANGE_T / 2, 0), ALU_BLACK, D_ALU, collide=False,
          bevel=0.0008)
    for s in (-1, 1):
        b.box(f"flange_{s}", (width, top - bottom, FLANGE_T), (0, (top + bottom) / 2, s * FLANGE_Z), ALU_BLACK, D_ALU,
              collide=False, bevel=0.0006)
        b.tube(f"pivot_{s}", (s * PIVOT_X, 0, LAYER_Z[0] - PLATE_T), (s * PIVOT_X, 0, LAYER_Z[-1] + 2 * PLATE_T + 0.003),
               0.0025, METAL, 7800.0, sides=16, collide=False)
    b.box("servo", SERVO_BOX[0], SERVO_BOX[1], SERVO, 1500.0, collide=False, bevel=0.001)
    edit.output(b.join(), "Geometry")
    return b


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    session = McpSession(ENDPOINT)
    builds = (("claw_finger_l", lambda e: build_finger(e, 1.0)), ("claw_finger_r", lambda e: build_finger(e, -1.0)),
              ("claw_bracket", build_bracket))
    for name, build in builds:
        graph = session.graph(name, directory=out)
        with graph.edit() as edit:
            b = build(edit)
        print(f"saved {graph.save(out / f'{name}.hpodgraph')} ({len(b.meshes)} meshes)")
    print(f"finger length {finger_path(PATH_STEP)[1]:.4f} m; open {OPEN_DEG} deg")


if __name__ == "__main__":
    main()
