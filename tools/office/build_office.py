"""Build the Lucky Robots office (535 Minnesota St, San Francisco) as Lucky procedural graphs.

Run with the editor open (Edit mode):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 build_office.py <new output dir>

Writes into <out> four graphs (and the kit assets they use, under <out>/assets):
- office_shell.hpodgraph: floor, tilt-up walls, gable roof, trusses, roof I-beams, skylights, clerestory windows, the
  black stage curtain on its track, roll-up and exit doors, conduit, panels, heaters, fans, LED high-bays.
- office_block.hpodgraph: the restroom box (lower deck) and the mezzanine (upper deck), the small closet south of the box, the
  room under the mezzanine (partition, French doors, transoms, blue sofa, pendants), railings, the two spiral stairs and
  the grey U stair.
- office_kitchen.hpodgraph: the kitchen run on the east wall (two fridges, range, microwave, dishwasher, sink,
  shaker cabinets, pantry, gooseneck lamps).
- office_desks.hpodgraph: the north-west corner (yellow sofa on the west wall, a small desk, the desk row along the
  north wall, the striped rug) and the west wall (a desk by the south-west corner, the grey cases, the robot dogs,
  boxes).
Bring them into the project with Lucky's import (pod_package, authoring_import): see import_office.py. The colliders
are plain MujocoBoxCollider entities in the scene (the drone's physics is MJCF), not part of these graphs.

Frame: Pod/Lucky, Y up, metres; the graphs sit at the scene origin. The plan below is written with x running from
the north wall A (curtain gap, clerestories, test mat) to the south wall C (office box, mezzanine, roll-up door) and
z from the west wall D (desk bay) to the east wall B (kitchen); each graph is mirrored in x at the end, so in the scene
the north wall is at x = +15 and the south wall at x = -15. The black curtain splits the floor in two bays; the drone flies in the east bay.

Sizes: photo scale from known objects (doors 0.91 x 2.13 m, counters 0.91 m, desks, the appliances below) and the
public record of 535-555 Minnesota St (8,235 SF unit in a 21,620 SF building, 22' ceiling). They are estimates; all
the room's measurements are the constants below, so real measurements can replace them.
Appliances (black stainless GE Profile-style French-door fridges, 30" pro range, 24" dishwasher, over-the-range
microwave) use their makers' standard sizes.
"""

from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "x500v2"))
from build_x500v2 import IDENTITY, euler_xyz, frame_y, mat, roll, yaw  # noqa: E402
from pod_authoring import Literal, McpSession, assets  # noqa: E402

ENDPOINT = "http://127.0.0.1:19331/mcp"
KIT_ASSETS = Path(next(Path("/tmp").glob(".mount_Lucky*/Data/Authoring/assets")))

# ---------------------------------------------------------------- the room (interior faces, metres)
# The main measurements live in office_params.toml next to this file: edit them there.
P = tomllib.loads((Path(__file__).resolve().parent / "office_params.toml").read_text())
X0, X1 = P["hall"]["x0"], P["hall"]["x1"]
Z0, Z1 = P["hall"]["z0"], P["hall"]["z1"]
H = P["hall"]["wall_height"]
RIDGE = P["hall"]["ridge"]
ZC = (Z0 + Z1) / 2            # middle of the roof, along x
WALL_T = P["hall"]["wall_thickness"]
ROOF_T = P["hall"]["roof_thickness"]
TRUSS_X = P["hall"]["truss_x"]
CURTAIN_Z = P["curtain"]["z"]
CURTAIN_X = (P["curtain"]["x0"], P["curtain"]["x1"])
CURTAIN_TOP = P["curtain"]["top"]
# Roof I-beams north to south, the size of the west wall's I-beam columns: one over the columns, centred on them, its
# mirror on the east wall, eight equally spaced between.
BEAM = P["hall"]["west_beam"]                  # flange, depth, gap to the wall
BEAMS_Z = [Z0 + BEAM[2] + BEAM[1] / 2 + k * (Z1 - Z0 - 2 * (BEAM[2] + BEAM[1] / 2)) / 9 for k in range(10)]
COLUMN_D = 0.25                                # the north wall's concrete columns stand this far into the hall
# LED high-bays, four columns of three in line with the skylight rows: on the roof beam outside the west skylights, its
# mirror on the kitchen side, and under the two thin north-south trusses (user).
SKY_EDGE = P["desk_corner"]["row_wall_gap"] + 1.3   # skylight rows: outer frame edge in line with the desk row's back
LIGHT_X = (X0 + SKY_EDGE, (X0 + X1) / 2, X1 - SKY_EDGE)
LIGHT_Z = (BEAMS_Z[1], BEAMS_Z[3], BEAMS_Z[6], BEAMS_Z[-2])
# all but the kitchen-side one by the tall box on the lower deck (user)
LIGHTS = [(x, z) for x in LIGHT_X for z in LIGHT_Z if not (x > P["mezzanine"]["front_x"] and z == BEAMS_Z[-2])]

# Restroom box and mezzanine at the south end, two tiers: the box's roof is the lower deck, the mezzanine west of it
# (from the U stair to the east wall) the upper deck.
BOX = (P["box"]["x0"], P["box"]["x1"], P["box"]["z0"], P["box"]["z1"])
BOX_DECK = P["box"]["deck"]
MEZZ = (P["mezzanine"]["front_x"], X1, P["mezzanine"]["z0"], P["mezzanine"]["z1"])
DECK = P["mezzanine"]["deck"]
PARTITION_X = P["partition"]["x"]

# Kitchen run on the east wall, north to south from KITCHEN_X.
KITCHEN_X = P["kitchen"]["x0"]

# ---------------------------------------------------------------- materials (linear RGB, metallic, roughness)
MATS = {
    "epoxy_floor": mat(0.30, 0.31, 0.32, 0.0, 0.45),
    "test_mat": mat(0.50, 0.50, 0.47, 0.0, 0.8),
    "tape_yellow": mat(0.85, 0.65, 0.02, 0.0, 0.6),
    "concrete_white": mat(0.85, 0.85, 0.82, 0.0, 0.9),
    "paint_white": mat(0.85, 0.85, 0.83, 0.0, 0.7),
    "timber_white": mat(0.74, 0.70, 0.66, 0.0, 0.85),
    "steel_white": mat(0.78, 0.78, 0.76, 0.2, 0.5),
    "curtain_white": mat(0.70, 0.70, 0.68, 0.0, 0.9),
    "steel_dark": mat(0.05, 0.05, 0.055, 0.6, 0.45),
    "steel_grey": mat(0.45, 0.46, 0.47, 0.8, 0.4),
    "conduit": mat(0.55, 0.56, 0.57, 0.9, 0.35),
    "conduit_red": mat(0.55, 0.03, 0.02, 0.3, 0.5),
    "panel_grey": mat(0.42, 0.43, 0.43, 0.3, 0.5),
    "glass_dark": mat(0.03, 0.04, 0.05, 0.0, 0.1),
    "rollup_grey": mat(0.42, 0.43, 0.44, 0.3, 0.5),
    "heater_brown": mat(0.20, 0.10, 0.06, 0.4, 0.5),
    "led_white": mat(1.0, 1.0, 1.0, 0.0, 0.3),
    "yellow_safety": mat(0.85, 0.70, 0.02, 0.0, 0.5),
    "deck_fascia": mat(0.12, 0.13, 0.14, 0.3, 0.5),
    "wood_tread": mat(0.30, 0.16, 0.07, 0.0, 0.6),
    "wood_door": mat(0.55, 0.30, 0.10, 0.0, 0.5),
    "glass_light": mat(0.55, 0.58, 0.60, 0.0, 0.1),
    "cabinet_white": mat(0.82, 0.82, 0.80, 0.0, 0.4),
    "counter_black": mat(0.02, 0.02, 0.022, 0.0, 0.25),
    "nickel": mat(0.70, 0.70, 0.68, 1.0, 0.3),
    "backsplash": mat(0.07, 0.07, 0.075, 0.0, 0.3),
    "red_cross": mat(0.6, 0.02, 0.02, 0.0, 0.5),
    "first_aid_cream": mat(0.78, 0.72, 0.55, 0.0, 0.6),
    "alu_matte": mat(0.60, 0.61, 0.62, 0.7, 0.55),
    "sofa_teal": mat(0.26, 0.42, 0.46, 0.0, 0.95),
    "wood_leg": mat(0.18, 0.06, 0.03, 0.0, 0.5),
    "panel_green": mat(0.15, 0.55, 0.20, 0.0, 0.5),
    "clock_face": mat(0.90, 0.90, 0.90, 0.0, 0.4),
    "anthracite": mat(0.13, 0.13, 0.14, 0.5, 0.5),
    "rug_diamond": mat(0.66, 0.64, 0.60, 0.0, 0.95),
    "stainless": mat(0.50, 0.51, 0.53, 0.8, 0.45),
    "desk_white": mat(0.85, 0.85, 0.84, 0.0, 0.4),
    "desk_frame": mat(0.55, 0.56, 0.57, 0.8, 0.35),
    "pc_black": mat(0.02, 0.02, 0.02, 0.2, 0.4),
    "cardboard": mat(0.45, 0.30, 0.15, 0.0, 0.9),
    "case_grey": mat(0.30, 0.30, 0.31, 0.0, 0.6),
    "stair_grey": mat(0.14, 0.145, 0.15, 0.0, 0.7),
    "glass_teal": mat(0.08, 0.25, 0.30, 0.0, 0.1),
    "glass_sky": mat(0.45, 0.50, 0.55, 0.0, 0.1),
    "exit_green": mat(0.10, 0.70, 0.20, 0.0, 0.4),
    "sign_blue": mat(0.02, 0.06, 0.35, 0.0, 0.4),
    "alu_case": mat(0.70, 0.70, 0.72, 0.9, 0.35),
    "poster": mat(0.12, 0.10, 0.09, 0.0, 0.5),
    "drinks": mat(0.30, 0.45, 0.60, 0.0, 0.6),
    "stripe_grey": mat(0.12, 0.12, 0.12, 0.0, 0.9),
    "glass_skylight": mat(0.10, 0.32, 0.85, 0.0, 0.1),
    "cable_blue": mat(0.02, 0.10, 0.55, 0.0, 0.5),
}
SOFA_YELLOW = mat(0.55, 0.38, 0.03, 0.0, 0.8)    # mustard tufted velvet
BLACK_STAINLESS = mat(0.06, 0.06, 0.065, 0.9, 0.35)
RUG_CREAM = mat(0.60, 0.56, 0.48, 0.0, 0.95)
RUG_GREY = mat(0.50, 0.49, 0.46, 0.0, 0.95)     # the diamond rug's base
SILVER = mat(0.75, 0.75, 0.77, 0.9, 0.3)     # studio display backs
STAINLESS = mat(0.50, 0.51, 0.53, 0.8, 0.45)   # range, dishwasher, microwave


class Graph:
    """One unit cube and one unit cylinder, placed by Transform; all parts of a material join into one mesh."""

    def __init__(self, edit, session, catalog, root):
        self.e, self.session, self.catalog, self.root = edit, session, catalog, root
        self.cube = edit.node("MeshCube", "unit cube", SizeX=1.0, SizeY=1.0, SizeZ=1.0)
        self.cyl = edit.node("MeshCylinder", "unit cylinder", Sides=16, Radius=1.0, Height=1.0,
                             CapFill=Literal("Menu", "Ngon"))
        self.groups = {}
        self.extra = []
        self.n = 0

    def place(self, geometry, key, scale, pos, rot=IDENTITY):
        self.n += 1
        t = self.e.node("Transform", f"{key} {self.n}", Geometry=geometry.output("Geometry"),
                        Scale=[float(s) for s in scale], Rotation=euler_xyz(rot), Translation=[float(p) for p in pos])
        self.groups.setdefault(key, []).append(t)

    def box(self, key, size, pos, rot=IDENTITY):
        self.place(self.cube, key, size, pos, rot)

    def span(self, key, x0, x1, y0, y1, z0, z1):
        """An axis-aligned box from its extents."""
        self.box(key, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))

    def strut(self, key, p0, p1, w, d=None):
        """A square member from p0 to p1."""
        v = [b - a for a, b in zip(p0, p1)]
        length = math.sqrt(sum(c * c for c in v))
        self.box(key, (w, length, d or w), [(a + b) / 2 for a, b in zip(p0, p1)], frame_y(v))

    def tube(self, key, p0, p1, r):
        v = [b - a for a, b in zip(p0, p1)]
        length = math.sqrt(sum(c * c for c in v))
        self.place(self.cyl, key, (r, length, r), [(a + b) / 2 for a, b in zip(p0, p1)], frame_y(v))

    def kit(self, name, pos, turn=0.0, material=None, **overrides):
        """A kit asset (origin on the floor, front +Z), turned about Y by turn and moved to pos."""
        self.n += 1
        inst = assets.use(self.e, self.session, self.catalog, name, self.root, **overrides)
        geometry = inst
        if material is not None:
            geometry = self.e.node("SetMaterial", f"kit {self.n} material", Geometry=inst.output("Geometry"),
                                   Material=material, Name=f"kit_{self.n}")
        self.extra.append(self.e.node("Transform", f"kit {self.n}", Geometry=geometry.output("Geometry"),
                                      Rotation=[0.0, turn, 0.0], Translation=[float(p) for p in pos]))

    def finish(self):
        e = self.e
        parts = []
        for key, nodes in self.groups.items():
            joined = nodes[0]
            for i, node in enumerate(nodes[1:]):
                joined = e.node("JoinGeometry", f"{key} join {i}", A=joined.output("Geometry"),
                                B=node.output("Geometry"))
            shaded = e.node("SetMaterial", f"{key} material", Geometry=joined.output("Geometry"),
                            Material=MATS[key], Name=key)
            parts.append(e.node("SetMeshName", key, Geometry=shaded.output("Geometry"), Name=key))
        parts += self.extra
        result = parts[0]
        for i, node in enumerate(parts[1:]):
            result = e.node("JoinGeometry", f"assemble {i}", A=result.output("Geometry"), B=node.output("Geometry"))
        # The plan below is written with x north to south and z west to east, which is left-handed in Lucky's
        # frame: mirror it in x (north wall at +15) and turn the faces back the right way out.
        mirrored = e.node("Transform", "mirror", Geometry=result.output("Geometry"), Scale=[-1.0, 1.0, 1.0])
        e.output(e.node("FlipFaces", "unmirror faces", Geometry=mirrored.output("Geometry")), "Geometry")


# ---------------------------------------------------------------- shell
def roof_y(z):
    """Roof underside height above z."""
    return RIDGE - (RIDGE - H) * max(0.0, abs(z - ZC) - FLAT) / (ZC - Z0 - FLAT)


def slope_frame(side):
    """side -1: west slope (rises toward +z), +1: east slope. Returns rotation, unit normal (up/out), angle."""
    a = math.atan2(RIDGE - H, ZC - Z0 - FLAT)
    r = roll(-a) if side < 0 else roll(a)
    return r, [r[i][1] for i in range(3)], a


def on_roof(g, key, size, x, z, below=0.0):
    """A box lying under the roof slope at (x, z), its top `below` metres under the roof underside."""
    if abs(z - ZC) <= FLAT:
        r, n = IDENTITY, [0.0, 1.0, 0.0]
    else:
        r, n, _ = slope_frame(-1 if z < ZC else 1)
    y = roof_y(z)
    off = below + size[1] / 2
    g.box(key, size, (x - n[0] * off, y - n[1] * off, z - n[2] * off), r)


def sections(name):
    """The window sections of one end wall as (z0, z1, units, cols, rows), spaced with equal gaps along the wall. On the
    north wall the concrete column in each gap stands under the nearest roof beam, and the side groups move with their
    column; the middle one stays (user)."""
    pw = P["south_wall"]["pane"][0]
    rows = P["south_wall"][name]
    widths = [n * c * pw for n, c, _ in rows]
    gap = (Z1 - Z0 - sum(widths)) / (len(rows) + 1)
    out, z = [], Z0 + gap
    for (n, c, r), w in zip(rows, widths):
        out.append((z, z + w, n, c, r))
        z += w + gap
    if name == "north_sections":
        for i, k in ((0, 0), (len(out) - 1, len(out) - 2)):
            c = (out[k][1] + out[k + 1][0]) / 2
            d = min(BEAMS_Z, key=lambda zb: abs(zb - c)) - c
            out[i] = (out[i][0] + d, out[i][1] + d, *out[i][2:])
    return out


def north_columns():
    """The north wall's concrete columns: one in each gap between window groups, under the nearest roof beam."""
    n = sections("north_sections")
    return [min(BEAMS_Z, key=lambda zb: abs(zb - (a[1] + b[0]) / 2)) for a, b in zip(n, n[1:])]


# The roof is flat across the middle, from over the north wall's column on one side of the middle window group to the
# one on the other (user), and slopes down to the long walls on either side.
FLAT = ZC - north_columns()[0]


def end_windows():
    """Every steel-sash window unit as (wall, z0, cols, rows): wall -1 north, +1 south."""
    pw = P["south_wall"]["pane"][0]
    return [(w, z0 + k * c * pw, c, r) for w, name in ((-1, "north_sections"), (1, "south_sections"))
            for z0, _, n, c, r in sections(name) for k in range(n)]


def window_rect(z0, cols, rows):
    pw, ph = P["south_wall"]["pane"]
    top = P["south_wall"]["window_top"]
    return z0, z0 + cols * pw, top - rows * ph, top


def holed_wall(g, x0, x1, za, zb, h, holes):
    """A wall slab x0..x1, z za..zb, 0..h with rectangular openings (z0, z1, y0, y1), built from horizontal bands."""
    ys = sorted({0.0, h, *(y for r in holes for y in r[2:])})
    for ya, yb in zip(ys, ys[1:]):
        z = za
        for z0, z1, _, _ in sorted(r for r in holes if r[2] <= ya and r[3] >= yb):
            if z0 > z:
                g.span("concrete_white", x0, x1, ya, yb, z, z0)
            z = max(z, z1)
        if z < zb:
            g.span("concrete_white", x0, x1, ya, yb, z, zb)


def sash(g, xw, inward, z0, cols, rows):
    """A steel-sash window in the opening of the wall whose inner face is x = xw (inward = +1 on the north wall, -1 on
    the south wall), set a couple of inches back into the wall, its top at the common window top: clear panes of
    P["south_wall"]["pane"], white bars between them, a heavier frame round it."""
    pw, ph = P["south_wall"]["pane"]
    z0, z1, y0, y1 = window_rect(z0, cols, rows)
    a, b = sorted((xw - inward * 0.07, xw - inward * 0.09))
    g.span("glass_sky", a, b, y0, y1, z0, z1)
    a, b = sorted((xw - inward * 0.05, xw - inward * 0.11))
    for k in range(cols + 1):
        w = 0.035 if 0 < k < cols else 0.06
        g.span("steel_white", a, b, y0, y1, z0 + k * pw - w / 2, z0 + k * pw + w / 2)
    for k in range(rows + 1):
        w = 0.035 if 0 < k < rows else 0.06
        g.span("steel_white", a, b, y0 + k * ph - w / 2, y0 + k * ph + w / 2, z0, z1)


def ibeam(g, x0, x1, y0, y1, z0, z1, web):
    """A white steel I-beam filling the box x0..x1, y0..y1, z0..z1: flanges at both ends of the `web` axis (0 x, 1 y,
    2 z), a thin web between them across the narrower of the other two axes."""
    lo, hi = [x0, y0, z0], [x1, y1, z1]
    t, w = 0.017, 0.01
    for end in (0, 1):
        a, b = lo[:], hi[:]
        if end:
            a[web] = hi[web] - t
        else:
            b[web] = lo[web] + t
        g.span("steel_white", a[0], b[0], a[1], b[1], a[2], b[2])
    thin = min((i for i in range(3) if i != web), key=lambda i: hi[i] - lo[i])
    a, b = lo[:], hi[:]
    c = (lo[thin] + hi[thin]) / 2
    a[thin], b[thin] = c - w / 2, c + w / 2
    g.span("steel_white", a[0], b[0], a[1], b[1], a[2], b[2])


def angle(g, p0, p1, leg, t=0.008):
    """A thin white steel L-angle from p0 to p1 (x, y, z) in a vertical plane z = const: one leg in the plane, one
    along z."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    a = math.atan2(-dx, dy)
    c, s_ = math.cos(a), math.sin(a)
    r = [[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]
    m = [(u + v) / 2 for u, v in zip(p0, p1)]
    length = math.hypot(dx, dy)
    o = leg / 2 - t / 2
    g.box("steel_white", (leg, length, t), (m[0], m[1], m[2] + t / 2), r)
    g.box("steel_white", (t, length, leg), (m[0] - c * o, m[1] - s_ * o, m[2] + leg / 2), r)


def slope_ibeam(g, x, za, zb, below, d=0.2, f=0.135, t=0.01):
    """A small white I-beam along the roof slope from za to zb (one side of the ridge), its top `below` m under the
    roof underside (measured square to the slope), web in the plane x."""
    a = slope_frame(-1 if (za + zb) / 2 < ZC else 1)[2]
    length = abs(zb - za) / math.cos(a)
    zm = (za + zb) / 2
    on_roof(g, "steel_white", (f, t, length), x, zm, below)
    on_roof(g, "steel_white", (0.008, d - 2 * t, length), x, zm, below + t)
    on_roof(g, "steel_white", (f, t, length), x, zm, below + d - t)


def build_shell(g):
    length, width = X1 - X0, Z1 - Z0
    xm, zm = (X0 + X1) / 2, (Z0 + Z1) / 2
    # Floor slab, test mat and tape.
    g.box("epoxy_floor", (length + 2 * WALL_T, 0.2, width + 2 * WALL_T), (xm, -0.1, zm))
    g.span("test_mat", X0 + 0.5, X0 + 8.5, 0.0, 0.003, Z1 - 8.5, Z1 - 0.5)
    g.span("tape_yellow", X0, CURTAIN_X[1] + 1.0, 0.0, 0.002, CURTAIN_Z + 2.45, CURTAIN_Z + 2.5)
    g.span("tape_yellow", X0 + 6.5, X0 + 6.55, 0.0, 0.002, CURTAIN_Z + 2.5, Z1)

    # Walls: the end walls up to the eave, the gable above them in strips that end inside the roof slab.
    for x, side in ((X0 - WALL_T / 2, -1), (X1 + WALL_T / 2, 1)):
        holes = [window_rect(z0, c, r) for w, z0, c, r in end_windows() if w == side]
        holed_wall(g, x - WALL_T / 2, x + WALL_T / 2, Z0 - WALL_T, Z1 + WALL_T, H, holes)
        steps = 12
        for i in range(steps):
            y0 = H + (RIDGE - H) * i / steps
            half = FLAT + (ZC - Z0 - FLAT) * (1 - i / steps) + WALL_T
            g.box("concrete_white", (WALL_T, (RIDGE - H) / steps + 0.001, 2 * half),
                  (x, y0 + (RIDGE - H) / steps / 2, ZC))
    for z in (Z0 - WALL_T / 2, Z1 + WALL_T / 2):
        g.box("concrete_white", (length + 2 * WALL_T, H + ROOF_T, WALL_T), (xm, (H + ROOF_T) / 2, z))

    # Roof beams' places and the skylights' (between two beams, see below), then the white timber roof deck: the two
    # slopes and the flat middle, with an opening up through it over each skylight.
    n, (bf, bd, bg), beams_z = P["hall"]["west_beams"], BEAM, BEAMS_Z
    sky_w = beams_z[1] - beams_z[0] - bf
    sky_x = LIGHT_X
    sky_z = ((beams_z[1] + beams_z[2]) / 2, (beams_z[4] + beams_z[5]) / 2)
    a = slope_frame(-1)[2]
    eave = Z0 - WALL_T - 0.2
    for za, zb in ((eave, ZC - FLAT), (ZC - FLAT, ZC + FLAT), (ZC + FLAT, 2 * ZC - eave)):
        zs = [za, zb]
        for z in sky_z:
            if za < z < zb:
                zs = [za, z - sky_w / 2, z + sky_w / 2, zb]
        for i, (z0, z1) in enumerate(zip(zs, zs[1:])):
            xs = [X0 - WALL_T] + ([v for x in sky_x for v in (x - 1.2, x + 1.2)] if i == 1 else []) + [X1 + WALL_T]
            zm = (z0 + z1) / 2
            along = (z1 - z0) / (1.0 if abs(zm - ZC) < FLAT else math.cos(a))
            for x0, x1 in zip(xs[::2], xs[1::2]):
                on_roof(g, "timber_white", (x1 - x0, ROOF_T, along), (x0 + x1) / 2, zm, -ROOF_T)

    # Roof framing (IMG_3106-3108, user): ten white steel I-beams north to south right under the roof, the size of the
    # west wall's I-beam columns: one on top of the columns, centred on them, its mirror on the east wall, eight equally
    # spaced between. Big I-beams on the end walls over the windows (the south one half in the wall, the north one
    # out to the columns' faces); the curtain beam is centred on
    # them and the two east-west A-frame trusses sit on it: small I-beam chords, the top one under the roof beams,
    # small struts between, a pilaster under their east ends. Two thin trusses of steel angles hang under the fourth
    # and seventh roof beams, north wall to south wall.
    # The roof beams frame into one more along the top of the north wall, following the roof, flush with the faces of
    # the wall's concrete columns (user).
    xn = X0 + COLUMN_D
    for z in beams_z:
        ibeam(g, xn, X1, roof_y(z) - bd, roof_y(z), z - bf / 2, z + bf / 2, 1)
    for za, zb in ((Z0, ZC - FLAT), (ZC + FLAT, Z1)):
        slope_ibeam(g, xn - bf / 2, za, zb, 0.0, bd, bf)
    ibeam(g, xn - bf, xn, RIDGE - bd, RIDGE, ZC - FLAT, ZC + FLAT, 1)
    for k in range(1, n + 1):
        x = X0 + k * length / (n + 1)
        ibeam(g, x - bf / 2, x + bf / 2, 0.0, roof_y(beams_z[0]) - bd, Z0 + bg, Z0 + bg + bd, 2)

    wt = P["south_wall"]["window_top"]
    end_beam = (wt + 0.475, wt + 1.075)           # ~W24, 0.6 m deep, 0.23 m flange; the curtain beam's top at 7.4 m (user)
    ibeam(g, xn - 0.23, xn, end_beam[0], end_beam[1], Z0, Z1, 1)     # north: its face flush with the columns' (user)
    ibeam(g, X1 - 0.115, X1 + 0.115, end_beam[0], end_beam[1], Z0, Z1, 1)
    curtain_beam = (sum(end_beam) / 2 - 0.225, sum(end_beam) / 2 + 0.225)   # ~W18
    bottom = curtain_beam[1]                       # the A-frames' bottom chord sits on the curtain beam
    chord, cf = 0.2, 0.135                         # small I-beam chords (~W8)
    a = slope_frame(-1)[2]
    # Both chords run between the columns on the side walls (the west wall's I-beam, the east wall's pilaster), the top
    # one under the roof beams up the slope and flat across the middle (user); between the chords a vertical strut
    # under each of the eight inner roof beams, one diagonal per panel leaning out from the middle, and an X in the
    # middle panel (user).
    zw, ze = Z0 + bg + bd, Z1 - 0.3            # the columns' inner faces
    vz = beams_z[1:-1]
    for x in TRUSS_X:
        ibeam(g, x - cf / 2, x + cf / 2, bottom, bottom + chord, zw, ze, 1)
        for za, zb in ((zw, ZC - FLAT), (ZC + FLAT, ze)):
            slope_ibeam(g, x, za, zb, bd * math.cos(a), chord, cf)
        ibeam(g, x - cf / 2, x + cf / 2, RIDGE - bd - chord, RIDGE - bd, ZC - FLAT, ZC + FLAT, 1)
        for z in vz:
            g.strut("steel_white", (x, bottom + chord, z), (x, roof_y(z) - bd - chord, z), 0.08)
        for za, zb in zip(vz, vz[1:]):
            ends = [(za, zb), (zb, za)] if za < ZC < zb else [(zb, za) if zb < ZC else (za, zb)]
            for z0, z1 in ends:                # from the bottom chord at z0 up to the top chord at z1
                g.strut("steel_white", (x, bottom + chord, z0), (x, roof_y(z1) - bd - chord, z1), 0.07)
        g.box("steel_white", (0.3, H, 0.3), (x, H / 2, Z1 - 0.15))
    # The thin trusses' bottoms line up with the A-frames' bottom, a square white plate under each crossing (user).
    for z in (beams_z[3], beams_z[6]):
        yt, yb = roof_y(z) - bd, bottom + 0.04
        panels = 12
        # From each end wall: one three-panel bay with a V and a Λ, then a panel with an X up to the A-frame; between
        # the A-frames no verticals, one zigzag \/\ of three diagonals (user).
        ka = round((TRUSS_X[0] - X0) / (length / panels))
        xs = [X0 + length * k / panels for k in (0, ka - 1, ka, panels - ka, panels - ka + 1, panels)]
        angle(g, (X0, yb, z), (X1, yb, z), 0.08)
        for x in xs:
            angle(g, (x, yb, z), (x, yt, z), 0.06)
        for i, (x, xn) in enumerate(zip(xs, xs[1:])):
            out, xc = min(i, len(xs) - 2 - i), (x + xn) / 2
            if out == 0:
                pairs = [((x, yt), (xc, yb)), ((xn, yt), (xc, yb)), ((x, yb), (xc, yt)), ((xn, yb), (xc, yt))]
            elif out == 1:
                pairs = [((x, yb), (xn, yt)), ((x, yt), (xn, yb))]
            else:
                zig = [(x + (xn - x) * j / 3, (yt, yb)[j % 2]) for j in range(4)]
                pairs = list(zip(zig, zig[1:]))
            for (xa, ya), (xb, yb_) in pairs:
                angle(g, (xa, ya, z), (xb, yb_, z), 0.06)
        for x in TRUSS_X:
            g.span("steel_white", x - 0.25, x + 0.25, bottom - 0.02, bottom, z - 0.25, z + 0.25)

    # Skylights: three rows north to south, between the trusses, in a west and a middle column, none on the east side
    # over the mezzanine (user). Each fills the gap between two roof beams (the second gap from the west wall, on the
    # slope, and the middle one, in the flat): blue glass up at the top of its opening in the deck, louvre bars just
    # under it, a white frame flush with the ceiling round the opening. The north row's outer frame edge lines up with
    # the back of the desk row by the north wall, the south row's sits as far from the south wall (user).
    for x in sky_x:
        for z in sky_z:
            on_roof(g, "glass_skylight", (2.4, 0.02, sky_w), x, z, below=0.03 - ROOF_T)
            for k in range(-3, 4):
                on_roof(g, "paint_white", (0.05, 0.05, sky_w), x + k * 0.33, z, below=0.08 - ROOF_T)
            for dx in (-1.25, 1.25):
                on_roof(g, "paint_white", (0.1, 0.03, sky_w + 0.2), x + dx, z, below=-0.01)
            for dz in (-1, 1):
                on_roof(g, "paint_white", (2.6, 0.03, 0.1), x, z + dz * (sky_w / 2 + 0.05), below=-0.01)

    # Steel-sash windows in their wall openings (end_windows): on the north wall three groups of three 4 x 4 pane units,
    # symmetric, one concrete column in each gap between groups (IMG_3053/3063, user); on the south wall three 4 x 4
    # units over the gate and the 8 pane wide windows from the exit door to the U stair's inner wall and over the upper
    # deck (IMG_3059/3077/3088). The big end-wall beams run above them (roof framing).
    for w, z0, cols, rows in end_windows():
        sash(g, X0 if w < 0 else X1, -w, z0, cols, rows)
    for c in north_columns():
        g.span("concrete_white", X0, X0 + COLUMN_D, 0.0, roof_y(c) - bd, c - 0.15, c + 0.15)

    # Stage curtain: one greyish-white pleated sheet on a black track under the white curtain I-beam, which runs the
    # whole hall north wall to south wall, centred on the end-wall beams (user, IMG_3077/3078). The track follows the
    # beam to the mezzanine front, bends west, then south to the gate wall in line with the fire extinguisher, hung on
    # rods from the roof where it leaves the beam (user).
    cl = CURTAIN_X[1] - CURTAIN_X[0]
    ch = CURTAIN_TOP - 0.05
    e = g.e
    grid = e.node("MeshGrid", "curtain sheet", SizeX=cl, SizeZ=ch, SegmentsX=int(cl / 0.04), SegmentsZ=4)
    px = e.node("SeparateXYZField", "curtain x", Source=e.node("PositionField", "curtain position").output("Value"))
    phase = e.node("MathField", "curtain phase", A=px.output("X"), B=2 * math.pi / 0.3,
                   Operation=Literal("Menu", "Multiply"))
    wave = e.node("MathField", "curtain wave", A=phase.output("Value"), Operation=Literal("Menu", "Sin"))
    pleated = e.node("DisplacePoints", "curtain pleats", Geometry=grid.output("Geometry"),
                     Offset=wave.output("Value"), Scale=0.06)
    cloth = e.node("Solidify", "curtain cloth", Geometry=pleated.output("Geometry"), Thickness=0.01)
    thick = e.node("SetShading", "curtain smooth", Geometry=cloth.output("Geometry"),
                   Mode=Literal("Menu", "Smooth"))
    g.place(thick, "curtain_white", (1, 1, 1), ((CURTAIN_X[0] + CURTAIN_X[1]) / 2, 0.05 + ch / 2, CURTAIN_Z),
            roll(math.pi / 2))
    b0, b1 = curtain_beam
    for y0, y1, w in ((b0, b0 + 0.011, 0.076), (b0 + 0.011, b1 - 0.011, 0.004), (b1 - 0.011, b1, 0.076)):
        g.span("steel_white", X0, X1, y0, y1, CURTAIN_Z - w, CURTAIN_Z + w)
    ty = CURTAIN_TOP + 0.03                        # track centre
    xt, zt, rb = P["mezzanine"]["front_x"], P["exit_door"]["z0"] - 0.36, 0.6
    track = [(X0, CURTAIN_Z)]
    for cx, cz, f0, f1 in ((xt - rb, CURTAIN_Z - rb, math.pi / 2, 0.0), (xt + rb, zt + rb, math.pi, 1.5 * math.pi)):
        track += [(cx + rb * math.cos(f0 + (f1 - f0) * k / 6), cz + rb * math.sin(f0 + (f1 - f0) * k / 6))
                  for k in range(7)]
    track.append((X1, zt))
    for (xa, za), (xb, zb) in zip(track, track[1:]):
        g.strut("steel_dark", (xa, ty, za), (xb, ty, zb), 0.05, 0.05)
    for k in range(int((xt - rb - X0) / 1.0) + 1):
        x = X0 + 0.3 + k * 1.0
        g.tube("steel_dark", (x, ty, CURTAIN_Z), (x, b0, CURTAIN_Z), 0.008)
    for x, z in ((xt, (CURTAIN_Z + zt) / 2), (xt + 1.5, zt), (xt + 3.0, zt), (X1 - 0.4, zt)):
        g.tube("steel_dark", (x, ty, z), (x, roof_y(z), z), 0.008)

    # South wall, east to west behind the U stair: the big steel-sash window, the fire alarm gear, the steel exit door,
    # plain wall, the closed roll-up gate with its hood and three window units above it. Sizes from office_params.toml.
    G, D, S = P["gate"], P["exit_door"], P["south_wall"]
    rz0, rz1, gh = G["z0"], G["z1"], G["height"]
    sx = X1 - 0.03
    g.span("rollup_grey", sx - 0.02, sx, 0.0, gh, rz0, rz1)
    for k in range(int(gh / 0.15)):
        g.box("steel_dark", (0.03, 0.015, rz1 - rz0), (sx - 0.03, 0.15 + k * 0.15, (rz0 + rz1) / 2))
    hood_r = (G["hood_top"] - gh) / 2
    g.tube("rollup_grey", (sx - hood_r, gh + hood_r, rz0 - 0.2), (sx - hood_r, gh + hood_r, rz1 + 0.2), hood_r)
    for z in (rz0 - 0.1, rz1 + 0.1):
        g.span("steel_dark", sx - 0.12, sx, 0.0, gh + 0.05, z - 0.06, z + 0.06)
    # Exit door: grey steel leaf in a frame, lever, EXIT sign; fire extinguisher west of it. Between the door and the
    # stair the fire alarm gear (IMG_3089): a red bell box up high, the red alarm panel, a keypad, a pull station.
    dz0, dz1, dh = D["z0"], D["z1"], D["height"]
    g.span("panel_grey", sx - 0.04, sx, 0.0, dh, dz0, dz1)
    for z0, z1 in ((dz0 - 0.05, dz0), (dz1, dz1 + 0.05)):
        g.span("panel_grey", sx - 0.06, sx, 0.0, dh + 0.05, z0, z1)
    g.span("panel_grey", sx - 0.06, sx, dh, dh + 0.05, dz0 - 0.05, dz1 + 0.05)
    g.span("nickel", sx - 0.1, sx - 0.04, 1.0, 1.04, dz1 - 0.2, dz1 - 0.05)
    # EXIT sign centred over the door, under the red conduit; fire extinguisher low west of the door, a red arrow sign
    # over it (IMG_3089).
    g.span("exit_green", sx - 0.08, sx, 2.3, 2.48, (dz0 + dz1) / 2 - 0.18, (dz0 + dz1) / 2 + 0.18)
    g.tube("red_cross", (sx - 0.1, 0.75, dz0 - 0.36), (sx - 0.1, 1.12, dz0 - 0.36), 0.075)
    g.span("red_cross", sx - 0.02, sx, 1.64, 1.94, dz0 - 0.43, dz0 - 0.33)
    # Fire alarm gear between the door and the stair, measured from the door frame (IMG_3089): the red panel with signs
    # over it, a keypad and a small grey box on its door side, the red pull station under it. Two red conduits leave the
    # panel top and run west over the door; past it one rises to the roof, the other to just over the gate hood and
    # across the gate to the west wall (IMG_3089/3059).
    for y0, y1, a, b, key in ((1.31, 1.79, 0.35, 0.68, "red_cross"), (1.8, 2.06, 0.55, 0.72, "clock_face"),
                              (1.26, 1.39, 0.14, 0.33, "panel_grey"), (1.28, 1.39, 0.07, 0.13, "panel_grey"),
                              (1.15, 1.29, 0.33, 0.43, "red_cross")):
        g.span(key, sx - 0.1, sx, y0, y1, dz1 + a, dz1 + b)
    for zp, yh, zr, top in ((dz1 + 0.44, 2.6, dz0 - 0.17, H - 0.1),
                            (dz1 + 0.5, 2.54, dz0 - 0.31, G["hood_top"] + 0.23)):
        pts = [(1.79, zp), (yh, zp), (yh, zr), (top, zr)] + ([(top, Z0)] if top < 6 else [])
        for (ya, za), (yb, zb) in zip(pts, pts[1:]):
            g.tube("conduit_red", (sx - 0.03, ya, za), (sx - 0.03, yb, zb), 0.012)

    # Conduit runs, electrical panels, extinguisher, bollards.
    g.tube("conduit", (X0 + 0.05, 1.25, Z0), (X0 + 0.05, 1.25, Z1), 0.012)
    g.tube("conduit", (X0, 1.15, Z0 + 0.05), (X1, 1.15, Z0 + 0.05), 0.012)
    g.tube("conduit", (X0, 6.1, Z0 + 0.06), (X1, 6.1, Z0 + 0.06), 0.015)
    g.tube("conduit", (X0, 1.2, Z1 - 0.05), (KITCHEN_X - 0.3, 1.2, Z1 - 0.05), 0.012)
    g.tube("conduit_red", (X0 + 6.0, 2.85, Z1 - 0.06), (BOX[0], 2.85, Z1 - 0.06), 0.01)
    g.span("panel_grey", X0, X0 + 0.15, 1.1, 1.85, CURTAIN_Z + 0.4, CURTAIN_Z + 0.9)
    g.span("panel_grey", 3.9, 4.5, 1.0, 1.9, Z0, Z0 + 0.15)
    g.tube("red_cross", (X0 + 0.12, 0.9, CURTAIN_Z + 1.2), (X0 + 0.12, 1.4, CURTAIN_Z + 1.2), 0.07)
    for x in (X1 - 0.4, X1 - 0.9):     # between the gate and the west corner
        z = (P["gate"]["z0"] + Z0) / 2
        g.tube("yellow_safety", (x, 0.0, z), (x, 1.1, z), 0.07)

    # Two unit heaters hung on rods from the north east-west truss, in the outer bays, aimed at the middle of the south
    # wall: a thin tall brown box, an aluminium fan panel standing out from its back, an aluminium exhaust hood under it
    # reaching out past its front (user, IMG_3106).
    # Ceiling fans, LED high-bays.
    xh = TRUSS_X[0]
    for z in (Z0 + width / 6, Z1 - width / 6):
        a = math.atan2(-(ZC - z), X1 - xh)
        r = yaw(a)

        def at(u, y, w):
            return (xh + u * math.cos(a) + w * math.sin(a), y, z - u * math.sin(a) + w * math.cos(a))

        g.box("heater_brown", (0.35, 0.9, 0.6), at(0.0, 6.05, 0.0), r)
        g.box("alu_matte", (0.04, 1.0, 0.7), at(-0.21, 6.05, 0.0), r)
        g.tube("alu_matte", at(-0.23, 6.05, 0.0), at(-0.33, 6.05, 0.0), 0.26)
        g.box("alu_matte", (0.5, 0.18, 0.7), at(0.06, 5.51, 0.0), r)
        for k in range(3):
            g.box("steel_dark", (0.01, 0.025, 0.64), at(0.315, 5.45 + 0.055 * k, 0.0), r)
        for w in (-0.22, 0.22):
            p = at(0.0, 6.5, w)
            g.tube("conduit", p, (p[0], bottom, p[2]), 0.01)
    # Ceiling fans: one hanging under each white plate where a thin truss crosses an A-frame (user).
    for x, z in ((x, z) for x in TRUSS_X for z in (beams_z[3], beams_z[6])):
        y = bottom - 0.7
        g.tube("steel_white", (x, y, z), (x, bottom - 0.02, z), 0.02)
        g.tube("steel_white", (x, y - 0.1, z), (x, y + 0.05, z), 0.1)
        for k in range(3):
            a = 2 * math.pi * k / 3
            g.box("steel_white", (0.7, 0.012, 0.14), (x + 0.45 * math.cos(a), y - 0.05, z - 0.45 * math.sin(a)),
                  yaw(a))
    for x, z in LIGHTS:
        top = bottom if z in (beams_z[3], beams_z[6]) else roof_y(z) - bd     # a truss or the roof beam above
        g.span("led_white", x - 0.3, x + 0.3, 6.35, 6.42, z - 0.12, z + 0.12)
        for xe in (x - 0.27, x + 0.27):        # two grey ropes from its narrow ends (user)
            g.tube("conduit", (xe, 6.42, z), (xe, top, z), 0.004)


# ---------------------------------------------------------------- office block and mezzanine
def balusters(g, key, p0, p1, base, height, step, w=0.022):
    n = max(1, int(math.dist(p0, p1) / step))
    for i in range(n + 1):
        t = i / n
        x, z = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
        g.box(key, (w, height, w), (x, base + height / 2, z))


def deck_rail(g, p0, p1):
    """Close steel balusters (0.1 m) from p0 to p1 (x, z) on the box deck, a flat top rail over them."""
    balusters(g, "steel_grey", p0, p1, BOX_DECK, 1.0, 0.1, 0.02)
    g.strut("steel_grey", (p0[0], BOX_DECK + 1.0, p0[1]), (p1[0], BOX_DECK + 1.0, p1[1]), 0.05, 0.03)


def cable_rail(g, p0, p1, out=(-1.0, 0.0)):
    """Galvanised tube posts ~1.5 m apart, each bolted with a plate to the fascia on the `out` side (x, z), a tube top
    rail, nine steel cables."""
    n = max(1, round(math.dist(p0, p1) / 1.5))
    for i in range(n + 1):
        t = i / n
        x, z = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
        g.tube("steel_grey", (x, DECK - 0.25, z), (x, DECK + 1.07, z), 0.024)
        px, pz = x + out[0] * 0.06, z + out[1] * 0.06
        g.span("steel_grey", px - 0.01 - 0.06 * abs(out[1]), px + 0.01 + 0.06 * abs(out[1]), DECK - 0.28, DECK - 0.02,
               pz - 0.01 - 0.06 * abs(out[0]), pz + 0.01 + 0.06 * abs(out[0]))
    g.tube("steel_grey", (p0[0], DECK + 1.07, p0[1]), (p1[0], DECK + 1.07, p1[1]), 0.024)
    for k in range(9):
        y = DECK + 0.12 + k * 0.1
        g.tube("panel_grey", (p0[0], y, p0[1]), (p1[0], y, p1[1]), 0.006)


def slat_rail(g, p0, p1):
    """A grey slat railing over the nosing line p0-p1 (x, y, z): a post at each end and five boards parallel to it."""
    for p in (p0, p1):
        g.box("stair_grey", (0.06, 1.05, 0.06), (p[0], p[1] + 0.525, p[2]))
    for h in (0.25, 0.45, 0.65, 0.85, 1.05):
        g.strut("stair_grey", (p0[0], p0[1] + h, p0[2]), (p1[0], p1[1] + h, p1[2]), 0.04, 0.12)


def spiral(g, cx, cz, r, y0, y1, treads, start, turn):
    """A spiral stair round an anthracite post from y0 to y1: per step an anthracite collar clamped on the post, one
    rectangular wood tread from the collar out, a slim black bracket under it from the collar, two balusters near its
    outer corners (rail coloured) right up into the handrail; the bottom tread points along yaw `start`, the top
    one `turn` radians further round."""
    rise = (y1 - y0) / treads
    step = turn / (treads - 1)
    hub = 0.065
    rr = r - 0.03                      # handrail radius; the balusters stand on it, 0.1 m either side of each tread's middle
    da = 0.1 / rr

    def rail_y(a):
        return y0 + rise * (1 + (treads - 1) * (a - start) / turn) + 0.9

    g.tube("anthracite", (cx, y0, cz), (cx, y1 + 0.9, cz), 0.05)
    for i in range(treads):
        a = start + step * i
        y = y0 + rise * (i + 1)
        d = (math.cos(a), -math.sin(a))
        g.tube("anthracite", (cx, y - 0.12, cz), (cx, y, cz), hub)
        mid = (hub + r) / 2
        g.box("wood_tread", (r - hub, 0.035, 0.24), (cx + d[0] * mid, y - 0.018, cz + d[1] * mid), yaw(a))
        mid = (hub + r - 0.08) / 2
        g.box("steel_dark", (r - 0.08 - hub, 0.06, 0.03), (cx + d[0] * mid, y - 0.065, cz + d[1] * mid), yaw(a))
        for side in (-1, 1):
            b = a + side * da
            foot = (cx + math.cos(b) * rr, y, cz - math.sin(b) * rr)
            g.tube("steel_grey", foot, (foot[0], rail_y(b), foot[2]), 0.009)
    # Handrail: a smooth helix through the baluster tops, a little past the first and last ones.
    m = 4 * (treads - 1)
    a0, a1 = start - math.copysign(da, turn), start + turn + math.copysign(da, turn)
    prev = None
    for j in range(m + 1):
        a = a0 + (a1 - a0) * j / m
        pt = (cx + math.cos(a) * rr, rail_y(a), cz - math.sin(a) * rr)
        if prev:
            g.tube("steel_grey", prev, pt, 0.02)
        prev = pt


def u_stair(g):
    """The grey dog-leg stair west of the mezzanine: a top platform at the mezzanine front, the upper flight south down
    to a landing against the south wall at half the deck height, the lower flight back north to the floor, slat
    railings, closed risers and a white closure wall under the lower flight."""
    U = P["u_stair"]
    n = U["risers"]
    rise, land = DECK / (2 * n), DECK / 2
    (px0, px1), (za, zb) = U["platform_x"], U["upper_z"]
    (xl, xe), (lz0, lz1), (zc, zd) = U["landing_x"], U["landing_z"], U["lower_z"]
    go = (xl - px1) / n
    g.span("stair_grey", px0, px1, DECK - 0.15, DECK, za, zb)
    g.span("stair_grey", xl, xe, land - 0.2, land, lz0, lz1)
    for k in range(1, n):
        x0 = px1 + go * (k - 1)
        g.span("stair_grey", x0, x0 + go, DECK - rise * k - 0.05, DECK - rise * k, za, zb)
        g.span("stair_grey", x0, x0 + 0.03, DECK - rise * k, DECK - rise * (k - 1), za, zb)
        x0, x1 = xl - go * k, xl - go * (k - 1)
        g.span("stair_grey", x0, x1, land - rise * k - 0.05, land - rise * k, zc, zd)
        g.span("stair_grey", x1 - 0.03, x1, land - rise * k, land - rise * (k - 1), zc, zd)
        g.span("paint_white", x0, x1, 0.0, land - rise * k - 0.05, zc - 0.05, zc)
    g.span("paint_white", xl, xe, 0.0, land - 0.2, lz0 - 0.05, lz0)
    for z in (za, zb):
        g.strut("stair_grey", (px1, DECK - 0.1, z), (xl, land - 0.1, z), 0.06, 0.3)
    for z in (zc, zd):
        g.strut("stair_grey", (xl, land - 0.1, z), (xl - go * n, 0.0, z), 0.06, 0.3)
    for z in (zc + 0.05, za - 0.05):
        g.box("stair_grey", (0.08, land, 0.08), (xl + 0.05, land / 2, z))
    slat_rail(g, (px1, DECK, za), (xl, land, za))
    slat_rail(g, (xl, land, zc), (xl - go * (n - 1), rise, zc))
    slat_rail(g, (xl, land, zd), (xl - go * (n - 1), rise, zd))
    slat_rail(g, (px0, DECK, za), (px0, DECK, zb))
    slat_rail(g, (px0, DECK, za), (px1, DECK, za))
    slat_rail(g, (xl, land, lz0), (xe, land, lz0))


def sofa(g, xw, zc, width=1.85, depth=0.8):
    """The teal mid-century sofa (IMG_3070/3092), its back against the wall plane x = xw, facing -x: splayed wood legs,
    a base, two seat cushions, a low back between rounded square arms, and its red tube frame: one bent tube hugging
    the back, round corners just above it, its legs down behind the arms to the floor."""
    x0, x1, z0, z1 = xw - 0.06 - depth, xw - 0.06, zc - width / 2, zc + width / 2
    for x in (x0 + 0.08, x1 - 0.08):
        for z in (z0 + 0.1, z1 - 0.1):
            g.tube("wood_leg", (x, 0.0, z + 0.03 * (1 if z < zc else -1)), (x, 0.15, z), 0.02)
    g.span("sofa_teal", x0, x1, 0.15, 0.3, z0, z1)
    arm = 0.15
    for za, zb in ((z0, z0 + arm), (z1 - arm, z1)):
        g.span("sofa_teal", x0, x1, 0.3, 0.44, za, zb)
        g.tube("sofa_teal", (x0, 0.44, (za + zb) / 2), (x1, 0.44, (za + zb) / 2), arm / 2)
    cw = (width - 2 * arm) / 2
    for k in range(2):
        za, zb = z0 + arm + k * cw + 0.01, z0 + arm + (k + 1) * cw - 0.01
        g.span("sofa_teal", x0 + 0.02, x1 - 0.18, 0.3, 0.42, za, zb)
        g.span("sofa_teal", x1 - 0.18, x1, 0.3, 0.8, za, zb)
        g.tube("sofa_teal", (x1 - 0.09, 0.8, za), (x1 - 0.09, 0.8, zb), 0.09)
    # Red frame: legs at the arms' inner edges, a 0.1 m bend at each top corner.
    xr, top, rb = x1 + 0.025, 0.92, 0.1
    za, zb = z0 + arm, z1 - arm
    pts = [(za, 0.0), (za, top - rb)]
    pts += [(za + rb - rb * math.cos(a), top - rb + rb * math.sin(a)) for a in (0.4, 0.8, 1.2, math.pi / 2)]
    pts += [(zb - rb + rb * math.cos(a), top - rb + rb * math.sin(a)) for a in (math.pi / 2, 1.2, 0.8, 0.4)]
    pts += [(zb, top - rb), (zb, 0.0)]
    for (za_, ya), (zb_, yb) in zip(pts, pts[1:]):
        g.tube("conduit_red", (xr, ya, za_), (xr, yb, zb_), 0.016)


def build_block(g):
    bx0, bx1, bz0, bz1 = BOX
    mx0, mx1, mz0, mz1 = MEZZ
    B, M, Q, S, T = P["box"], P["mezzanine"], P["partition"], P["spiral"], P["spiral_upper"]
    # The restroom box; north of the mezzanine front its flat roof is the lower deck, with the tall box (against the
    # mezzanine) and the mid box standing on it; south of the front it runs up under the upper deck.
    under = DECK - 0.35
    g.span("paint_white", bx0, mx0, 0.0, BOX_DECK - 0.15, bz0, bz1)
    g.span("deck_fascia", bx0 - 0.02, mx0, BOX_DECK - 0.15, BOX_DECK, bz0 - 0.02, bz1)
    g.span("paint_white", mx0, bx1, 0.0, under, bz0, bz1)
    for x0, x1, z0, z1, top in (B["tall_box"], B["mid_box"]):
        g.span("paint_white", x0, x1, BOX_DECK, top, z0, z1)
    mb = B["mid_box"]
    g.span("paint_white", mb[0] + 0.25, mb[1] - 0.3, mb[4] - 1.0, mb[4] - 0.35, mb[2] - 0.02, mb[2])
    g.span("nickel", mb[1] - 0.42, mb[1] - 0.38, mb[4] - 0.7, mb[4] - 0.62, mb[2] - 0.05, mb[2] - 0.02)
    # North face, facing the kitchen: the MEN door. West face, facing the hall: a door with the first-aid box beside it.
    zc = B["men_door_z"]
    g.span("paint_white", bx0 - 0.03, bx0, 0.0, 2.13, zc - 0.455, zc + 0.455)
    g.span("nickel", bx0 - 0.09, bx0 - 0.03, 1.0, 1.03, zc + 0.25, zc + 0.4)
    g.span("steel_white", bx0 - 0.02, bx0, 2.13, 2.18, zc - 0.5, zc + 0.5)
    g.span("sign_blue", bx0 - 0.04, bx0 - 0.03, 1.45, 1.7, zc - 0.12, zc + 0.12)
    xc = B["west_door_x"]
    # (IMG_3087/3092: a grey sign centred on the door, a red square at its left end; the cabinet hangs a little north of
    # the door's casing, its cross white on a red square, red FIRST AID lettering under it.)
    g.span("paint_white", xc - 0.455, xc + 0.455, 0.0, 2.13, bz0 - 0.03, bz0)
    for x0, x1 in ((xc - 0.51, xc - 0.455), (xc + 0.455, xc + 0.51)):
        g.span("steel_white", x0, x1, 0.0, 2.18, bz0 - 0.025, bz0)
    g.span("steel_white", xc - 0.51, xc + 0.51, 2.13, 2.18, bz0 - 0.025, bz0)
    g.span("nickel", xc - 0.4, xc - 0.25, 0.97, 1.0, bz0 - 0.09, bz0 - 0.03)
    g.span("panel_grey", xc - 0.12, xc + 0.12, 1.66, 1.73, bz0 - 0.035, bz0 - 0.03)
    g.span("red_cross", xc - 0.105, xc - 0.055, 1.67, 1.72, bz0 - 0.04, bz0 - 0.035)
    fa0, fa1 = xc - 1.0, xc - 0.61
    g.span("first_aid_cream", fa0, fa1, 1.35, 1.82, bz0 - 0.12, bz0)
    fm = (fa0 + fa1) / 2
    g.span("red_cross", fm - 0.08, fm + 0.08, 1.6, 1.76, bz0 - 0.125, bz0 - 0.12)
    g.span("clock_face", fm - 0.018, fm + 0.018, 1.62, 1.74, bz0 - 0.128, bz0 - 0.125)
    g.span("clock_face", fm - 0.06, fm + 0.06, 1.662, 1.698, bz0 - 0.128, bz0 - 0.125)
    g.span("red_cross", fm - 0.09, fm + 0.09, 1.52, 1.56, bz0 - 0.125, bz0 - 0.12)
    # Red conduit round the box at 2.6 m with grey junction boxes (IMG_3104); south of the mezzanine front a riser
    # goes up the west face to the deck, blue cables climbing beside it (IMG_3103).
    g.tube("conduit_red", (bx0 - 0.03, 2.6, bz0), (bx0 - 0.03, 2.6, bz1), 0.01)
    g.tube("conduit_red", (bx0, 2.6, bz0 - 0.03), (bx1, 2.6, bz0 - 0.03), 0.01)
    xr = mx0 + 0.3
    for x in (xc - 1.45, xc, xr):
        g.span("panel_grey", x - 0.06, x + 0.06, 2.54, 2.66, bz0 - 0.06, bz0)
    g.tube("conduit_red", (xr, 2.66, bz0 - 0.03), (xr, under, bz0 - 0.03), 0.01)
    for k in range(3):
        g.tube("cable_blue", (xr + 0.08 + 0.03 * k, 2.62, bz0 - 0.03), (xr + 0.45 + 0.03 * k, under, bz0 - 0.03), 0.006)
    # Lower deck edge: a dark bolted steel channel round its north and west edges, close silver balusters on it
    # (IMG_3104/3087), open where the big spiral's landing joins the deck.
    north_end = B["mid_box"][2]
    g.span("steel_dark", bx0 - 0.05, bx0, BOX_DECK - 0.22, BOX_DECK + 0.03, bz0 - 0.05, bz1)
    # The big spiral comes up onto a square wood landing on a dark steel frame over its pole, between its top tread and
    # the deck edge, railed on its north and west sides (IMG_3103/3087).
    lx0, lx1, lz0 = S["x"] - S["radius"], S["x"] + 0.12, S["z"] - 0.12
    for x0, x1 in ((bx0, lx0), (lx1, mx0)):
        g.span("steel_dark", x0, x1, BOX_DECK - 0.22, BOX_DECK + 0.03, bz0 - 0.05, bz0)
        for x in [x0 + 0.15 + 0.3 * k for k in range(int((x1 - x0) / 0.3))]:
            for y in (BOX_DECK - 0.16, BOX_DECK - 0.03):
                g.span("nickel", x - 0.012, x + 0.012, y - 0.012, y + 0.012, bz0 - 0.06, bz0 - 0.05)
    deck_rail(g, (bx0 + 0.03, bz0), (bx0 + 0.03, north_end))
    g.span("wood_tread", lx0, lx1, BOX_DECK - 0.04, BOX_DECK + 0.005, lz0, bz0)
    for x0, x1, z0, z1 in ((lx0, lx0 + 0.03, lz0, bz0), (lx1 - 0.03, lx1, lz0, bz0), (lx0, lx1, lz0, lz0 + 0.03)):
        g.span("steel_dark", x0, x1, BOX_DECK - 0.14, BOX_DECK - 0.04, z0, z1)
    deck_rail(g, (lx0 + 0.02, lz0 + 0.02), (lx0 + 0.02, bz0))
    deck_rail(g, (lx0 + 0.02, lz0 + 0.02), (S["x"] - 0.08, lz0 + 0.02))
    for x0, x1 in ((bx0, lx0), (lx1, mx0)):
        deck_rail(g, (x0, bz0 + 0.03), (x1, bz0 + 0.03))

    # Upper deck: one slab from its front to the south wall, from the U stair to the east wall; over the hall a front
    # beam on two white posts and the cable railing, over the box deck open (the small spiral comes up there and the
    # tall box stands against it).
    t = M["thickness"]
    g.span("deck_fascia", mx0, mx1, DECK - 0.3, DECK, mz0, Z1)
    g.span("paint_white", mx0 + 0.02, mx1, under, DECK - 0.3, mz0, Z1)
    # Steel I-beams: the front beam on two posts.
    bf, bd = M["beam"]
    ibeam(g, mx0, mx0 + bf, DECK - t, DECK - 0.3, mz0, mz1, 1)
    for z in M["posts_z"]:
        ibeam(g, mx0, mx0 + bd, 0.0, DECK - t, z - bf / 2, z + bf / 2, 0)
    # Cable railing along the front from the U stair to the small spiral (over the box deck too), and on the deck's
    # inner edge above the upper flight down to the south wall (IMG_3066/3087/3089).
    cable_rail(g, (mx0 + 0.05, mz0 + 0.05), (mx0 + 0.05, T["z"] - T["radius"] - 0.05))
    cable_rail(g, (P["u_stair"]["platform_x"][1], mz0 + 0.05), (mx1, mz0 + 0.05), out=(0.0, -1.0))

    # Under it, all walls up to the deck's underside: the room's west wall from the front wall to the south wall (the U
    # stair flush against it; open beside the sofa), the front wall with the closed French doors, transoms, pendants and
    # the sofa; east of the box's west face line it goes on as the south wall of a small open closet (a glazed side
    # door in it), which ends at a wall with a dark doorway.
    px, top = PARTITION_X, under
    (d0, d1), (sz0, sz1), ce = Q["doors_z"], Q["side_door_z"], Q["closet_z1"]
    g.span("paint_white", px, X1, 0.0, top, mz0, mz0 + 0.12)
    g.span("paint_white", bx1, px, 0.0, top, ce, ce + 0.12)
    for z0, z1 in ((mz0, d0), (d1, sz0), (sz1, Z1)):
        g.span("paint_white", px, px + 0.15, 0.0, top, z0, z1)
    for z0, z1 in ((d0, d1), (sz0, sz1)):
        g.span("paint_white", px, px + 0.15, 2.21, top, z0, z1)
    g.span("paint_white", px + 2.0, px + 2.1, 0.0, top, d0 - 0.5, d1 + 0.5)
    # French doors, closed: a wood frame round the opening, two wood leaves, each with a column of seven panes.
    for z0, z1 in ((d0, d0 + 0.08), (d1 - 0.08, d1)):
        g.span("wood_door", px - 0.04, px + 0.19, 0.0, 2.21, z0, z1)
    g.span("wood_door", px - 0.04, px + 0.19, 2.13, 2.21, d0, d1)
    leaf = (d1 - d0 - 0.16) / 2
    ph = (2.13 - 0.3 - 6 * 0.035) / 7
    for z0 in (d0 + 0.08, d0 + 0.08 + leaf):
        g.span("wood_door", px + 0.05, px + 0.1, 0.0, 2.13, z0 + 0.003, z0 + leaf - 0.003)
        pz0, pz1 = z0 + leaf * 0.3, z0 + leaf * 0.7
        for i in range(7):
            y = 0.2 + i * (ph + 0.035)
            g.span("glass_dark", px + 0.045, px + 0.105, y, y + ph, pz0, pz1)
    (ty0, ty1) = Q["transom_y"]
    # Transoms: glass in a matte aluminium frame on all four sides (outer size from the params).
    fw = 0.045
    for zc, w in zip(Q["transoms_z"], Q["transom_w"]):
        za, zb = zc - w / 2, zc + w / 2
        g.span("glass_teal", px - 0.01, px, ty0, ty1, za, zb)
        for y0, y1, z0, z1 in ((ty0, ty0 + fw, za, zb), (ty1 - fw, ty1, za, zb), (ty0, ty1, za, za + fw),
                               (ty0, ty1, zb - fw, zb)):
            g.span("alu_matte", px - 0.035, px, y0, y1, z0, z1)
    sofa(g, px, Q["sofa_z"])
    # White enamel barn pendants on a cord from the joists.
    for z in Q["pendants_z"]:
        g.tube("pc_black", (px - 0.8, 2.6, z), (px - 0.8, top - 0.25, z), 0.004)
        g.tube("clock_face", (px - 0.8, 2.5, z), (px - 0.8, 2.6, z), 0.07)
        g.tube("clock_face", (px - 0.8, 2.45, z), (px - 0.8, 2.5, z), 0.2)
    g.span("glass_light", px + 0.07, px + 0.08, 0.1, 2.05, sz0 + 0.05, sz1 - 0.05)
    for z0 in (sz0, sz1 - 0.06):
        g.span("wood_door", px - 0.02, px + 0.17, 0.0, 2.21, z0, z0 + 0.06)
    g.span("wood_door", px - 0.02, px + 0.17, 2.13, 2.21, sz0, sz1)
    g.span("backsplash", bx1 + 0.3, px - 0.3, 0.0, 2.13, ce - 0.01, ce)
    g.span("panel_green", bx1, bx1 + 0.06, 1.2, 1.6, ce - 0.7, ce - 0.4)
    # Under the deck in front of the room (IMG_3070/3089/3091), all at the front beam's level under the grey deck: two
    # north-south I-beams from the front beam back to the front wall, one by the stair-side wall and one on the box's
    # west face line; white open-web joists ~0.7 m apart hung between them; the red sprinkler main below the joists
    # along the front wall, with a drop at its east end and two heads.
    bt = DECK - 0.3
    for z0, z1 in ((mz0, mz0 + bf), (mz1 - bf, mz1)):
        ibeam(g, mx0 + bf, px, bt - bd, bt, z0, z1, 1)
    ja, jb = mz0 + bf, mz1 - bf
    for k in range(int((px - mx0 - 0.4) / 0.7)):
        x = mx0 + 0.6 + k * 0.7
        for y in (bt - 0.04, bt - bd):
            g.span("paint_white", x - 0.03, x + 0.03, y, y + 0.04, ja, jb)
        n = max(1, round((jb - ja) / 0.5))
        for i in range(n):
            za, zb = ja + i * (jb - ja) / n, ja + (i + 1) * (jb - ja) / n
            lo, hi = bt - bd + 0.02, bt - 0.02
            p0, p1 = ((x, lo, za), (x, hi, zb)) if i % 2 == 0 else ((x, hi, za), (x, lo, zb))
            g.strut("paint_white", p0, p1, 0.025)
    rx, ry = px - 0.35, bt - bd - 0.1
    g.tube("conduit_red", (rx, ry, mz1 - 0.1), (rx, ry, -0.3), 0.025)
    g.tube("conduit_red", (rx, ry, 2.0), (rx, ry - 0.4, 2.0), 0.025)
    for z in (1.0, -0.3):
        g.tube("conduit_red", (rx, ry, z), (rx, ry - 0.12, z), 0.012)
        g.tube("nickel", (rx, ry - 0.12, z), (rx, ry - 0.14, z), 0.04)
    u_stair(g)

    # Spiral stairs: floor -> lower deck just west of the box, lower deck -> upper deck where the mezzanine starts.
    for R, y0, y1 in ((S, 0.0, BOX_DECK), (T, BOX_DECK, DECK)):
        spiral(g, R["x"], R["z"], R["radius"], y0, y1, R["treads"], R["start"], R["turn"])


# ---------------------------------------------------------------- kitchen
def shaker_door(g, x0, x1, y0, y1, zf):
    """A white shaker door in front of plane z=zf (facing -z): slab, raised rails and a nickel handle."""
    g.span("cabinet_white", x0 + 0.003, x1 - 0.003, y0 + 0.003, y1 - 0.003, zf - 0.02, zf)
    for a, b, c, d in ((x0 + 0.003, x0 + 0.07, y0, y1), (x1 - 0.07, x1 - 0.003, y0, y1),
                       (x0, x1, y0 + 0.003, y0 + 0.07), (x0, x1, y1 - 0.07, y1 - 0.003)):
        g.span("cabinet_white", a, b, c, d, zf - 0.03, zf - 0.02)


def base_run(g, x0, x1, doors, zf, counter=True, drawers=False):
    g.span("backsplash", x0, x1, 0.0, 0.1, zf + 0.05, Z1)
    g.span("cabinet_white", x0, x1, 0.1, 0.876, zf, Z1)
    w = (x1 - x0) / doors
    if drawers:
        for k in range(4):
            y0 = 0.1 + k * 0.194
            shaker_door(g, x0, x1, y0, y0 + 0.19, zf)
            g.span("nickel", (x0 + x1) / 2 - 0.08, (x0 + x1) / 2 + 0.08, y0 + 0.14, y0 + 0.155, zf - 0.06, zf - 0.03)
    else:
        for k in range(doors):
            a = x0 + k * w
            shaker_door(g, a, a + w, 0.1, 0.876, zf)
            hx = a + w - 0.06 if k % 2 == 0 else a + 0.05
            g.span("nickel", hx, hx + 0.012, 0.62, 0.8, zf - 0.06, zf - 0.03)
    if counter:
        g.span("counter_black", x0, x1, 0.876, 0.91, zf - 0.03, Z1)


def upper_run(g, x0, x1, y0, y1, doors):
    zf = Z1 - 0.33
    g.span("cabinet_white", x0, x1, y0, y1, zf, Z1)
    w = (x1 - x0) / doors
    for k in range(doors):
        a = x0 + k * w
        shaker_door(g, a, a + w, y0, y1, zf)
        hx = a + w - 0.06 if k % 2 == 0 else a + 0.05
        g.span("nickel", hx, hx + 0.012, y0 + 0.03, y0 + 0.2, zf - 0.06, zf - 0.03)


def build_kitchen(g):
    x = KITCHEN_X
    zf = Z1 - 0.61                    # base cabinet fronts
    f1, f2 = x, x + 0.908             # fridges: 35.75" x 69.875" (GE Profile PFE28K-style), black stainless
    c1 = x + 2 * 0.908
    rng = c1 + 0.45                   # 30" range
    dr = rng + 0.762
    dw = dr + 0.6                     # 24" dishwasher
    sk = dw + 0.61
    pa = sk + 0.9
    end = pa + 0.9
    for fx in (f1, f2):
        fc = fx + 0.454
        g.kit("appliances/fridge_v2", (fc, 0.0, Z1 - 0.45), turn=math.pi,
              width=0.88, height=1.775, depth=0.86, steel=BLACK_STAINLESS)
        # Stainless pulls: two on the French doors, one across the freezer drawer.
        for hx in (fc - 0.06, fc + 0.06):
            g.span("nickel", hx - 0.012, hx + 0.012, 0.8, 1.55, Z1 - 0.97, Z1 - 0.93)
        g.span("nickel", fc - 0.3, fc + 0.3, 0.6, 0.625, Z1 - 0.97, Z1 - 0.93)
    base_run(g, c1, rng, 1, zf)
    # 30" stainless pro range: oven door with a small window and a pull bar, a knob strip, black grates on top.
    rf = Z1 - 0.64
    g.span("stainless", rng + 0.005, dr - 0.005, 0.1, 0.91, rf, Z1)
    g.span("glass_dark", rng + 0.14, dr - 0.14, 0.3, 0.55, rf - 0.005, rf)
    g.tube("nickel", (rng + 0.08, 0.66, rf - 0.05), (dr - 0.08, 0.66, rf - 0.05), 0.012)
    g.span("panel_grey", rng + 0.005, dr - 0.005, 0.74, 0.88, rf - 0.01, rf)
    for k in range(6):
        kx = rng + 0.1 + k * (dr - rng - 0.2) / 5
        g.tube("pc_black", (kx, 0.81, rf - 0.01), (kx, 0.81, rf - 0.04), 0.02)
    g.span("pc_black", rng + 0.02, dr - 0.02, 0.91, 0.935, rf + 0.03, Z1 - 0.05)
    base_run(g, dr, dw, 1, zf, drawers=True)
    g.kit("appliances/dishwasher_v2", ((dw + sk) / 2, 0.005, Z1 - 0.31), turn=math.pi,
          width=0.61, height=0.86, depth=0.6, steel=STAINLESS)
    g.span("stainless", dw + 0.01, sk - 0.01, 0.1, 0.86, Z1 - 0.63, Z1 - 0.61)
    g.tube("nickel", (dw + 0.08, 0.78, Z1 - 0.66), (sk - 0.08, 0.78, Z1 - 0.66), 0.01)
    g.span("counter_black", dw, sk, 0.876, 0.91, zf - 0.03, Z1)
    base_run(g, sk, pa, 2, zf)
    # Sink basin and faucet.
    sc = (sk + pa) / 2
    g.span("steel_dark", sc - 0.3, sc + 0.3, 0.911, 0.913, zf + 0.06, zf + 0.46)
    g.tube("nickel", (sc, 0.91, Z1 - 0.08), (sc, 1.25, Z1 - 0.08), 0.015)
    g.tube("nickel", (sc, 1.25, Z1 - 0.08), (sc, 1.18, Z1 - 0.32), 0.012)
    # Backsplash, microwave over the range, upper cabinets, pantry.
    g.span("backsplash", c1, pa, 0.91, 1.42, Z1 - 0.01, Z1)
    g.kit("appliances/microwave_v2", ((rng + dr) / 2, 1.48, Z1 - 0.21), turn=math.pi, steel=STAINLESS)
    # Wall clock over the cabinets (IMG_3094): black rim, white face, two hands.
    cx = x + 3.35
    g.tube("pc_black", (cx, 2.75, Z1), (cx, 2.75, Z1 - 0.05), 0.2)
    g.tube("clock_face", (cx, 2.75, Z1 - 0.05), (cx, 2.75, Z1 - 0.055), 0.18)
    g.box("pc_black", (0.012, 0.13, 0.006), (cx, 2.80, Z1 - 0.058))
    g.box("pc_black", (0.09, 0.012, 0.006), (cx + 0.04, 2.75, Z1 - 0.058))
    upper_run(g, f1, c1, 1.88, 2.42, 4)
    upper_run(g, c1, rng, 1.42, 2.42, 1)
    upper_run(g, rng, dr, 1.9, 2.42, 2)
    upper_run(g, dr, sk, 1.42, 2.42, 2)
    upper_run(g, sk, pa, 1.42, 2.42, 2)
    g.span("cabinet_white", pa, end, 0.0, 2.42, zf, Z1)
    for y0, y1 in ((0.1, 1.5), (1.52, 2.42)):
        for a in (pa, pa + 0.45):
            shaker_door(g, a, a + 0.45, y0, y1, zf)
    g.span("nickel", pa + 0.4, pa + 0.412, 0.9, 1.3, zf - 0.06, zf - 0.03)
    g.span("nickel", pa + 0.488, pa + 0.5, 0.9, 1.3, zf - 0.06, zf - 0.03)
    # Gooseneck barn lamps above the run.
    for lx in (x + 0.7, x + 3.1, x + 5.5):
        g.tube("steel_dark", (lx, 2.95, Z1), (lx, 3.05, Z1 - 0.35), 0.012)
        g.tube("steel_dark", (lx, 3.05, Z1 - 0.35), (lx, 2.92, Z1 - 0.5), 0.012)
        g.tube("steel_dark", (lx, 2.84, Z1 - 0.5), (lx, 2.92, Z1 - 0.5), 0.17)


# ---------------------------------------------------------------- desk bay
def standing_desk(g, x, z, along_z=True, length=1.5, depth=0.75):
    """A white sit-stand desk, length x depth x 0.74 m, its length along z (or x)."""
    lx, lz = (depth, length) if along_z else (length, depth)
    g.span("desk_white", x - lx / 2, x + lx / 2, 0.715, 0.74, z - lz / 2, z + lz / 2)
    a = length / 2 - 0.15
    for s in (-1, 1):
        cx, cz = (x, z + s * a) if along_z else (x + s * a, z)
        g.box("desk_frame", (0.07, 0.68, 0.07), (cx, 0.37, cz))
        g.box("desk_frame", (depth - 0.07, 0.04, 0.07) if along_z else (0.07, 0.04, depth - 0.07), (cx, 0.02, cz))
    g.box("desk_frame", (0.05, 0.05, 2 * a) if along_z else (2 * a, 0.05, 0.05), (x, 0.68, z))


def beni_rug(g, x0, x1, z0, z1, y=0.0):
    """A cream Beni Ourain rug with thin dark stripes across its long side (z)."""
    g.kit("decor/rug_v2", ((x0 + x1) / 2, y, (z0 + z1) / 2), material=RUG_CREAM, width=x1 - x0, depth=z1 - z0)
    for k in range(6):
        z = z0 + 0.3 + k * (z1 - z0 - 0.6) / 5
        g.span("stripe_grey", x0 + 0.2, x1 - 0.2, y + 0.012, y + 0.014, z - 0.012, z + 0.012)


def diamond_rug(g, x0, x1, z0, z1):
    """A grey rug with a grid of off-white diamonds, close to its own colour."""
    g.kit("decor/rug_v2", ((x0 + x1) / 2, 0.0, (z0 + z1) / 2), material=RUG_GREY, width=x1 - x0, depth=z1 - z0)
    nx, nz = max(1, round((x1 - x0 - 0.4) / 0.6)), max(1, round((z1 - z0 - 0.4) / 0.6))
    for i in range(nx):
        for k in range(nz):
            x = x0 + 0.2 + (i + 0.5) * (x1 - x0 - 0.4) / nx
            z = z0 + 0.2 + (k + 0.5) * (z1 - z0 - 0.4) / nz
            g.box("rug_diamond", (0.32, 0.002, 0.32), (x, 0.013, z), yaw(math.pi / 4))


def crane(g, cx, cz):
    """The G1's engine-hoist crane (IMG_3097): black frame on two short wheeled base legs running south, a post ending
    at the yellow winch on its back, a boom from the post top reaching south and up, a steel cable down to a red hook."""
    top = 1.75
    for z in (cz - 0.35, cz + 0.35):
        g.span("pc_black", cx - 0.15, cx + 1.0, 0.06, 0.14, z - 0.04, z + 0.04)
        for x in (cx - 0.1, cx + 0.95):
            g.tube("pc_black", (x, 0.0, z - 0.03), (x, 0.0, z + 0.03), 0.04)
    g.span("pc_black", cx - 0.15, cx - 0.05, 0.06, 0.14, cz - 0.4, cz + 0.4)
    g.span("pc_black", cx - 0.05, cx + 0.05, 0.06, top, cz - 0.05, cz + 0.05)
    g.strut("pc_black", (cx, top - 0.05, cz), (cx + 0.85, top + 0.6, cz), 0.07)
    g.tube("yellow_safety", (cx - 0.15, top - 0.1, cz - 0.06), (cx - 0.15, top - 0.1, cz + 0.06), 0.12)
    g.tube("conduit", (cx + 0.85, top + 0.6, cz), (cx + 0.85, 1.5, cz), 0.006)
    g.span("red_cross", cx + 0.81, cx + 0.89, 1.35, 1.5, cz - 0.03, cz + 0.03)


def build_desks(g):
    C = P["desk_corner"]
    # Work area in the north-west corner (lucky_office/work-area): the corner desk against the west wall, its user on
    # the north side facing south; the yellow tufted sofa south of it, long side north-south, facing east onto the Beni
    # rug; the grey diamond rug between that rug and the curtain, a stanchion on the Beni rug's north edge.
    g.kit("seating/sofa_v2", (C["sofa_x"], 0.0, C["sofa_z"]), material=SOFA_YELLOW, width=2.1, depth=0.8)
    dx, dz = C["small_desk_x"], C["small_desk_z"]
    standing_desk(g, dx, dz, length=C["small_desk"][0], depth=C["small_desk"][1])
    g.kit("appliances/monitor_v2", (dx + 0.12, 0.74, dz), turn=-math.pi / 2)
    g.kit("seating/office_chair_v2", (dx - 0.55, 0.0, dz), turn=math.pi / 2)
    (rx0, rx1), (rz0, rz1) = C["rug_x"], C["rug_z"]
    beni_rug(g, rx0, rx1, rz0, rz1)
    diamond_rug(g, *C["rug2_x"], *C["rug2_z"])
    sx, sz = rx0 + 0.15, (rz0 + rz1) / 2
    g.tube("pc_black", (sx, 0.0, sz), (sx, 1.0, sz), 0.02)
    g.tube("pc_black", (sx, 0.0, sz), (sx, 0.03, sz), 0.2)
    # Behind the row, near the north wall (IMG_3098): the grey metal workbench (a rolling cart) with two speakers on it,
    # boxes and drink stacks by it.
    b0, b1 = C["bench_z"]
    bx = X0 + C["bench_wall_gap"]
    g.span("panel_grey", bx, bx + 0.75, 0.86, 0.9, b0, b1)
    g.span("panel_grey", bx, bx + 0.75, 0.2, 0.23, b0, b1)
    for x in (bx + 0.03, bx + 0.72):
        for z in (b0 + 0.03, b1 - 0.03):
            g.box("desk_frame", (0.04, 0.88, 0.04), (x, 0.44, z))
    for z in (b0 + 0.25, b1 - 0.25):
        g.box("pc_black", (0.25, 0.32, 0.22), (bx + 0.3, 1.06, z))
    for z, s in ((b1 + 0.3, (0.5, 0.45, 0.45)), (b1 + 0.75, (0.45, 0.35, 0.4))):
        g.box("cardboard", s, (X0 + 0.3, s[1] / 2, z))
    # The blue drink stacks in the north-west corner, flush with the north wall, 1 m off the west wall.
    for z in (Z0 + 1.2, Z0 + 1.65):
        g.box("drinks", (0.35, 0.9, 0.4), (X0 + 0.175, 0.45, z))
    # A black saddle stool between the workbench and the row (IMG_3098).
    sx, sz = X0 + 2.4, (b0 + b1) / 2
    g.tube("pc_black", (sx, 0.0, sz), (sx, 0.03, sz), 0.25)
    g.tube("pc_black", (sx, 0.03, sz), (sx, 0.55, sz), 0.025)
    g.box("pc_black", (0.42, 0.08, 0.34), (sx, 0.59, sz))
    # Three white desks along the north wall, half a centimetre apart (IMG_3096/3098): two silver studio displays on
    # the west desk facing the wall, an inch apart; a black monitor on the east one facing south, its chair on the
    # south side; PCs under the desks; in line with the west end the low grey flight case (0.4 x 0.8 x 0.3 m) and the
    # black bin (0.25 x 0.5 x 0.75 m) side by side; the G1's crane past the east end.
    (w0, w1), (dl, dd), (sl, sd) = C["row_z"], C["row_desk"], C["small_desk"]
    xd = X0 + C["row_wall_gap"] + dd / 2
    for k in range(2):
        standing_desk(g, xd, w0 + dl / 2 + k * (dl + 0.005), length=dl, depth=dd)
    ez = w0 + 2 * (dl + 0.005) + sl / 2
    standing_desk(g, xd, ez, length=sl, depth=sd)
    g.kit("seating/office_chair_v2", (xd + 0.7, 0.0, ez), turn=-math.pi / 2)
    for z in (w0 + dl / 2 - 0.3225, w0 + dl / 2 + 0.3225):
        g.kit("appliances/monitor_v2", (xd + 0.1, 0.74, z), turn=-math.pi / 2, material=SILVER)
    g.kit("appliances/monitor_v2", (xd - 0.1, 0.74, ez), turn=math.pi / 2)
    for z in (w0 + 0.6, w0 + 2.0):                          # full-tower gaming PCs at the desks' front edge
        g.span("pc_black", xd + dd / 2 - 0.6, xd + dd / 2, 0.0, 0.6, z - 0.125, z + 0.125)
    crane(g, *C["crane"])
    g.box("alu_case", (0.8, 0.3, 0.4), (xd, 0.15, w0 - 0.22))
    g.box("pc_black", (0.5, 0.75, 0.25), (xd, 0.375, w0 - 0.565))
    # Posters flush on the north wall: the two brown ones side by side 2 m from the west wall, two more east with
    # more space between them (IMG_3096).
    for z in (Z0 + 2.3, Z0 + 2.92, w1 - 1.7, w1 - 0.6):
        g.box("poster", (0.03, 0.9, 0.6), (X0 + 0.015, 0.45, z))
    # A long box on the east-bay floor.
    g.box("cardboard", (1.2, 0.2, 0.32), (X0 + 4.0, 0.1, 3.0))
    # West wall, south to north: a desk north of the gate's west end, its user facing into the hall, boxes on its gate
    # side (IMG_3059, user), the grey cases north of the pilaster with one open on the floor, more boxes (the robot
    # dogs are Unitree Go2 packs in the scene).
    # The desk group: 6.7 m north of the gate wall, 1.2 m east of the gate's west side (user).
    gx, gz = X1 - 6.7, P["gate"]["z0"] + 1.2
    standing_desk(g, gx, gz + 0.85, along_z=False, length=1.2, depth=0.55)
    g.kit("seating/office_chair_v2", (gx, 0.0, gz + 0.24))
    # Cardboard boxes on the desk's gate side, various sizes, some stacked: (x, z, size, base height, turn).
    for x, z, size, y, turn in ((1.2, 0.5, (0.5, 0.35, 0.45), 0.0, 0.1),
                                (1.2, 0.5, (0.4, 0.3, 0.35), 0.35, -0.2),
                                (1.15, 1.25, (0.6, 0.45, 0.6), 0.0, 0.0),
                                (1.15, 1.2, (0.45, 0.3, 0.4), 0.45, 0.3),
                                (1.85, 0.85, (0.45, 0.4, 0.5), 0.0, -0.15),
                                (1.85, 0.85, (0.35, 0.25, 0.3), 0.4, 0.25),
                                (1.8, 1.6, (0.3, 0.25, 0.35), 0.0, 0.4)):
        g.box("cardboard", size, (gx + x, y + size[1] / 2, gz + z), yaw(turn))
    for x, z, s in ((-5.9, Z0 + 0.4, (0.6, 0.45, 0.6)), (-6.5, Z0 + 0.4, (0.5, 0.5, 0.5)),
                    (-6.2, Z0 + 1.1, (0.5, 0.35, 0.45))):
        g.box("cardboard", s, (x, s[1] / 2, z))
    for k in range(7):
        g.box("case_grey", (0.45, 0.75, 0.32), (3.4 - k * 0.48, 0.375, Z0 + 0.2))
    g.box("case_grey", (0.75, 0.25, 0.5), (2.6, 0.125, Z0 + 1.0))


BUILDS = (("office_shell", build_shell), ("office_block", build_block), ("office_kitchen", build_kitchen),
          ("office_desks", build_desks))


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    session = McpSession(ENDPOINT, timeout=300)
    catalog = assets.open_catalog(session, root=KIT_ASSETS)
    for name, build in BUILDS:
        graph = session.graph(name, directory=out)
        with graph.edit() as edit:
            g = Graph(edit, session, catalog, out)
            build(g)
            g.finish()
        print(f"saved {graph.save(out / f'{name}.hpodgraph')} ({g.n} parts)")


if __name__ == "__main__":
    main()
