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
from claw_shape import (  # noqa: E402
    BRACKET_BOTTOM, BRACKET_TOP, BRACKET_WIDTH, CENTERLINE, FLANGE_T, FLANGE_Z, GEAR_ROOT, GEAR_SECTOR, GEAR_TEETH,
    GEAR_TIP, HALF_IN, HALF_OUT, HUB_R, INNER_START, LAYER_Z, OPEN_DEG, PATH_STEP, PIVOT_X, PLATE_T, SCREW_HEAD,
    SERVO_BOX, STANDOFF_R, STANDOFFS, TIP_CAP, finger_path,
)

ENDPOINT = "http://127.0.0.1:19331/mcp"
BRASS = mat(0.75, 0.52, 0.18, 1.0, 0.3)
SERVO = mat(0.07, 0.07, 0.075, 0.2, 0.5)   # a shade lighter than the black plates
D_ALU = 2700.0



def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


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
    top, bottom, width = BRACKET_TOP, BRACKET_BOTTOM, BRACKET_WIDTH
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
