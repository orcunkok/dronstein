"""Build the Hiwonder (LewanSoul) aluminium wave-edge robotic claw (Amazon B0CMLRGXP4) as Lucky procedural graphs.

Run with the editor open (Edit mode):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 build_claw.py <new output dir>

Writes into <out>:
- claw_bracket.hpodgraph: the U-shaped holder the fingers hang from: its back plate (the mounting face) and two
  flanges that carry the two pivot pins. The servo (inside the U) is not modelled.
- claw_finger_l.hpodgraph, claw_finger_r.hpodgraph: the left and right fingers, pivot at the origin: four stacked
  3 mm plates, two inside the holder's flanges and two outside them (the top one carries the gear, above the
  holder), whose inner edge is an S-shaped wave and whose tip ends in a plain round,
  brass standoffs between them (a silver screw head at each end) and the gear on the top plate. The right finger is the left one mirrored in x (built
  mirrored: a mirrored entity, scale X -1, shows its plates inside out). Each plate is its real outline: Lucky's
  CurveFromAnchors takes at most 8 points, so the outline is cut into strips and fans of up to 8 points that
  FillCurve fills one by one.
Bring them into the project with Lucky's import (pod_package, authoring_import): a graph already loaded is cached, so
copying over its file does not change the scene. Claw.cs swings the fingers.

Frame: Pod/Lucky, Y up, metres. The claw hangs fingers down: pivots on the X axis at +-PIVOT_X, pivot axes along Z,
the four plates of a finger stacked along Z.

Sources:
- Amazon B0CMLRGXP4 / Hiwonder SKU 21070010: max opening 230 mm, ~145 x 62 x 61 mm closed, 191 g with its LDX-335MG
  servo, hard aluminium. Everything else (plate band, curve, pivot spacing, teeth) is scaled from the product photo:
  re-measure the real claw and fix the constants below.
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
D_ALU = 2700.0

# ---------------------------------------------------------------- dimensions (m)
PIVOT_X = 0.012           # pivots 24 mm apart; their gears mesh
TIP = (0.007, -0.125)     # closed fingertip centreline, from the left pivot: the two tips meet edge to edge at x=0
SAGITTA = 0.025           # how far the finger curve bulges outward from the pivot-to-tip line
BAND = 0.011              # plate width across the curve, outer edge to the wave's crests
PLATE_T = 0.003           # plate thickness
FLANGE_Z, FLANGE_T = 0.0215, 0.0025   # the holder's two flanges (centres) and their thickness
LAYER_Z = (-0.0265, -0.0085, 0.0085, 0.0265)   # finger plates: the outer two just outside the flanges
WAVE = (0.009, 0.0022)    # inner-edge wave: wavelength and depth (crest to trough)
WAVE_START = 0.014        # the wave starts this far along the curve from the pivot (the hub is plain)
STANDOFF_R = 0.0028       # brass hex standoffs between the plates
SCREW_HEAD = (0.00275, 0.0016)     # M3 button head screwed into each standoff end: radius, height
STANDOFFS = (0.4, 0.72, 0.97)      # where they sit along the curve (0 pivot, 1 tip)
HUB_R = 0.009             # plate round the pivot
GEAR_R, GEAR_TEETH = 0.013, 12     # outside radius; 2 mm teeth, so the two gears mesh across the 24 mm pivots
OPEN_DEG = 59.0           # each finger's swing from closed to fully open: tips 230 mm apart (Claw.cs uses it)
SAMPLES_PER_WAVE = 8      # outline points per wavelength along the curve


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def finger_curve():
    """The finger's centreline: a circular arc from the pivot (0, 0) to TIP bulging outward (-x). Returns the arc's
    centre, radius and start/end angles."""
    tx, ty = TIP
    chord = math.hypot(tx, ty)
    radius = (chord * chord / 4 + SAGITTA * SAGITTA) / (2 * SAGITTA)
    nx, ny = ty / chord, -tx / chord           # chord normal pointing inward (+x side)
    if nx < 0:
        nx, ny = -nx, -ny
    cx, cy = tx / 2 + nx * (radius - SAGITTA), ty / 2 + ny * (radius - SAGITTA)
    a0 = math.atan2(-cy, -cx)
    a1 = math.atan2(ty - cy, tx - cx)
    if a1 < a0:
        a1 += 2 * math.pi
    return (cx, cy), radius, a0, a1


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


def plate_outline():
    """One finger plate: the curved band with the S-wave inner edge, the round hub at the pivot and the round tip.
    Returns its fill pieces and its edges."""
    (cx, cy), radius, a0, a1 = finger_curve()
    length = radius * (a1 - a0)
    n = int(length / WAVE[0] * SAMPLES_PER_WAVE)
    outer, inner = [], []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        s = length * i / n
        trough = 0.5 - 0.5 * math.cos(2 * math.pi * (s - WAVE_START) / WAVE[0]) if s > WAVE_START else 0.0
        r_in = radius - BAND / 2 + WAVE[1] * trough   # crests at the band edge, troughs cut WAVE[1] into it
        outer.append((cx + (radius + BAND / 2) * math.cos(a), cy + (radius + BAND / 2) * math.sin(a)))
        inner.append((cx + r_in * math.cos(a), cy + r_in * math.sin(a)))
    pieces = [outer[i:i + 4] + inner[i:i + 4][::-1] for i in range(0, n, 3)]
    hub, hub_rim = fan(0.0, 0.0, lambda a: HUB_R, 24)
    tx, ty = cx + radius * math.cos(a1), cy + radius * math.sin(a1)
    tip, tip_rim = fan(tx, ty, lambda a: BAND / 2, 36)
    return pieces + hub + tip, [outer, inner, hub_rim, tip_rim]


def gear_outline(side):
    """The finger's spur gear on the top plate: GEAR_TEETH square-ish teeth 2 mm deep. The right gear is turned half
    a tooth so its teeth sit in the left gear's gaps."""
    n = GEAR_TEETH * 4
    shift = 0 if side > 0 else 2
    pieces, rim = fan(0.0, 0.0, lambda a: GEAR_R if (int(round(a / (2 * math.pi) * n)) + shift) % 4 < 2
                      else GEAR_R - 0.002, n)
    return pieces, [rim]


def build_finger(edit, side):
    b = Builder(edit)
    plate = outline(edit, "plate", *plate_outline(), side)
    for k, z in enumerate(LAYER_Z):
        b._place(plate, f"plate_{k}", IDENTITY, (0.0, 0.0, z - PLATE_T / 2), ALU_BLACK)
    (cx, cy), radius, a0, a1 = finger_curve()
    for j, f in enumerate(STANDOFFS):
        a = a0 + (a1 - a0) * f
        x, y = side * (cx + radius * math.cos(a)), cy + radius * math.sin(a)
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
    top, bottom, width = 0.016, -0.014, 0.056
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
    (cx, cy), radius, a0, a1 = finger_curve()
    print(f"finger arc r={radius:.4f} centre=({cx:.4f}, {cy:.4f}) {math.degrees(a1 - a0):.1f} deg; open {OPEN_DEG} deg")


if __name__ == "__main__":
    main()
