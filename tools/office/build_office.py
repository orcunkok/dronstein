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
KITCHEN_END = BOX[0] - P["kitchen"]["box_gap"]

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
    "curtain_denim": mat(0.34, 0.38, 0.44, 0.0, 0.9),   # the curtain's kitchen (east) side: denim-like grey (user)
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
    "chair_metal": mat(0.45, 0.45, 0.50, 1.0, 0.30),  # the office chair kit's metal, for its added crossing leg
    "gate_rail": mat(0.30, 0.31, 0.32, 0.6, 0.5),     # roll-up gate's guide rails: grey, darker than the door (user)
    "gate_alu": mat(0.80, 0.81, 0.83, 0.3, 0.5),     # roll-up gate's curtain: aluminium colour, not shiny (user)
    "alu_sink": mat(0.78, 0.79, 0.80, 1.0, 0.3),      # brushed aluminium sink basin (user)
    "lamp_grey": mat(0.50, 0.51, 0.52, 0.85, 0.25),   # the kitchen's gooseneck lamps: slightly polished grey (user)
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
    "rail_brown": mat(0.10, 0.06, 0.035, 0.3, 0.5),   # the spiral stairs' and the lower deck's balusters
    "plywood": mat(0.55, 0.38, 0.20, 0.0, 0.75),      # the upper and lower decks' floors, lighter than the treads
}
SOFA_YELLOW = mat(0.55, 0.38, 0.03, 0.0, 0.8)    # mustard tufted velvet
BLACK_STAINLESS = mat(0.06, 0.06, 0.065, 0.9, 0.35)
RUG_CREAM = mat(0.60, 0.56, 0.48, 0.0, 0.95)
RUG_GREY = mat(0.50, 0.49, 0.46, 0.0, 0.95)     # the diamond rug's base
SILVER = mat(0.75, 0.75, 0.77, 0.9, 0.3)     # studio display backs
STAINLESS = mat(0.50, 0.51, 0.53, 0.8, 0.45)   # fridges, range, dishwasher, microwave


class Graph:
    """One unit cube and one unit cylinder, placed by Transform; all parts of a material join into one mesh."""

    def __init__(self, edit, session, catalog, root):
        self.e, self.session, self.catalog, self.root = edit, session, catalog, root
        self.cube = edit.node("MeshCube", "unit cube", SizeX=1.0, SizeY=1.0, SizeZ=1.0)
        self.cyl = edit.node("MeshCylinder", "unit cylinder", Sides=16, Radius=1.0, Height=1.0,
                             CapFill=Literal("Menu", "Ngon"))
        self.cyl_hi = edit.node("MeshCylinder", "unit cylinder smooth", Sides=64, Radius=1.0, Height=1.0,
                                CapFill=Literal("Menu", "Ngon"))   # for big, close-up drums (the gate's roller hood)
        self.shade = edit.node("MeshCone", "lamp shade", Sides=32, BottomRadius=1.0, TopRadius=0.28, Height=1.0)
        self.groups = {}
        self.extra = []
        self.n = 0
        self.fields = None

    def plane_fields(self):
        """Shared point fields x, z and x*z of whatever geometry consumes them (made once per graph)."""
        if self.fields is None:
            xyz = self.e.node("SeparateXYZField", "point xyz",
                              Source=self.e.node("PositionField", "point position").output("Value"))
            xz = self.e.node("MathField", "point x*z", A=xyz.output("X"), B=xyz.output("Z"),
                             Operation=Literal("Menu", "Multiply"))
            self.fields = (xyz.output("X"), xyz.output("Z"), xz.output("Value"))
        return self.fields

    def place(self, geometry, key, scale, pos, rot=IDENTITY):
        self.n += 1
        t = self.e.node("Transform", f"{key} {self.n}", Geometry=geometry.output("Geometry"),
                        Scale=[float(s) for s in scale], Rotation=euler_xyz(rot), Translation=[float(p) for p in pos])
        self.groups.setdefault(key, []).append(t)

    def box(self, key, size, pos, rot=IDENTITY):
        self.place(self.cube, key, size, pos, rot)

    def cut_box(self, key, size, pos, tool_size, tool_pos, tool_rot):
        """A box with a (rotated) box cut out of it, e.g. a wall with a sloped top."""
        self.n += 1
        a = self.e.node("Transform", f"{key} {self.n}", Geometry=self.cube.output("Geometry"),
                        Scale=[float(v) for v in size], Translation=[float(v) for v in pos])
        b = self.e.node("Transform", f"{key} {self.n} tool", Geometry=self.cube.output("Geometry"),
                        Scale=[float(v) for v in tool_size], Rotation=euler_xyz(tool_rot),
                        Translation=[float(v) for v in tool_pos])
        cut = self.e.node("MeshBoolean", f"{key} {self.n} cut", A=a.output("Geometry"), B=b.output("Geometry"),
                          Operation=Literal("Menu", "Difference"))
        self.groups.setdefault(key, []).append(cut)

    def span(self, key, x0, x1, y0, y1, z0, z1):
        """An axis-aligned box from its extents."""
        self.box(key, (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))

    def strut(self, key, p0, p1, w, d=None):
        """A square member from p0 to p1."""
        v = [b - a for a, b in zip(p0, p1)]
        length = math.sqrt(sum(c * c for c in v))
        self.box(key, (w, length, d or w), [(a + b) / 2 for a, b in zip(p0, p1)], frame_y(v))

    def tube(self, key, p0, p1, r, smooth=False):
        v = [b - a for a, b in zip(p0, p1)]
        length = math.sqrt(sum(c * c for c in v))
        self.place(self.cyl_hi if smooth else self.cyl, key, (r, length, r), [(a + b) / 2 for a, b in zip(p0, p1)], frame_y(v))

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
def roof_part(x, z):
    """The part of the roof over (x, z): the west or east slope, the flat middle, or one of the flat middle's ends
    sloping down to the north or south wall."""
    if abs(z - ZC) >= FLAT - 1e-6:
        return "west" if z < ZC else "east"
    return "north" if x < DROP_X[0] else "south" if x > DROP_X[1] else "flat"


def strip_y(x):
    """Height of the flat middle at x: level between the drop lines, down to DROP_END at the end walls."""
    d = max(0.0, DROP_X[0] - x, x - DROP_X[1]) / (DROP_X[0] - X0)
    return RIDGE - (RIDGE - DROP_END) * d


def part_y(f, x, z):
    """Height of roof part f's underside over (x, z), carried on past the part's own edges. The side slopes run from
    the eave up to the flat middle's edge, so near the end walls they slope down both ways (user)."""
    if f == "west":
        return H + (strip_y(x) - H) * (z - Z0) / (ZC - FLAT - Z0)
    if f == "east":
        return H + (strip_y(x) - H) * (Z1 - z) / (Z1 - ZC - FLAT)
    return strip_y(x)


def roof_y(x, z):
    """Roof underside height over (x, z)."""
    return part_y(roof_part(x, z), x, z)


def roof_frame(f, x, z):
    """Roof part f's tangent frame at (x, z): rotation (x along the roof over plan x, y the unit normal up), the normal,
    and how much a plan length along x and along z stretches on it there."""
    e = 1e-3
    fx = (part_y(f, x + e, z) - part_y(f, x - e, z)) / (2 * e)
    fz = (part_y(f, x, z + e) - part_y(f, x, z - e)) / (2 * e)
    ln, lx = math.sqrt(1 + fx * fx + fz * fz), math.sqrt(1 + fx * fx)
    n = [-fx / ln, 1 / ln, -fz / ln]
    ax = [1 / lx, fx / lx, 0.0]
    az = [ax[1] * n[2] - ax[2] * n[1], ax[2] * n[0] - ax[0] * n[2], ax[0] * n[1] - ax[1] * n[0]]
    return [[ax[i], n[i], az[i]] for i in range(3)], n, (lx, math.sqrt(1 + fz * fz))


def on_roof(g, key, size, x, z, below=0.0, f=None):
    """A box lying under roof part f (the one over (x, z) if not given), its top `below` metres under the roof
    underside, square to it; sizes along a slope are measured along it."""
    f = f or roof_part(x, z)
    r, n, _ = roof_frame(f, x, z)
    y = part_y(f, x, z)
    off = below + size[1] / 2
    g.box(key, size, (x - n[0] * off, y - n[1] * off, z - n[2] * off), r)


def roof_patch(g, f, x0, x1, z0, z1, segments=8, key="timber_white"):
    """One smooth piece of the roof deck on roof part f over the plan box x0..x1, z0..z1: a grid bent to the part's
    surface through its four corner heights (exact for flat, sloped and twisted parts, so neighbouring patches meet
    without seams), ROOF_T thick down to the ceiling, smooth shaded."""
    e = g.e
    hx, hz = (x1 - x0) / 2, (z1 - z0) / 2
    y = [[part_y(f, x, z) + ROOF_T for z in (z0, z1)] for x in (x0, x1)]       # the deck's top at the corners
    a = (y[0][0] + y[0][1] + y[1][0] + y[1][1]) / 4
    b = (y[1][0] + y[1][1] - y[0][0] - y[0][1]) / (4 * hx)
    c = (y[0][1] + y[1][1] - y[0][0] - y[1][0]) / (4 * hz)
    d = (y[1][1] - y[1][0] - y[0][1] + y[0][0]) / (4 * hx * hz)
    px, pz, pxz = g.plane_fields()
    g.n += 1
    k = g.n
    mad = Literal("Menu", "MultiplyAdd")
    t = e.node("MathField", f"roof {k} x", A=px, B=b, C=a, Operation=mad)
    t = e.node("MathField", f"roof {k} z", A=pz, B=c, C=t.output("Value"), Operation=mad)
    t = e.node("MathField", f"roof {k} xz", A=pxz, B=d, C=t.output("Value"), Operation=mad)
    grid = e.node("MeshGrid", f"roof {k}", SizeX=2 * hx, SizeZ=2 * hz, SegmentsX=segments, SegmentsZ=segments)
    lift = e.node("SetPosition", f"roof {k} lift", Geometry=grid.output("Geometry"),
                  Offset=e.node("CombineXYZField", f"roof {k} y", Y=t.output("Value")).output("Value"))
    deck = e.node("Solidify", f"roof {k} deck", Geometry=lift.output("Geometry"), Thickness=ROOF_T)
    smooth = e.node("SetShading", f"roof {k} smooth", Geometry=deck.output("Geometry"), Mode=Literal("Menu", "Smooth"))
    g.place(smooth, key, (1, 1, 1), ((x0 + x1) / 2, 0.0, (z0 + z1) / 2))


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
    if name == "south_sections":                   # over the gate centred on it, the big window just east of it
        g0, g1 = P["gate"]["z0"], P["gate"]["z1"]
        for i, a in ((0, (g0 + g1) / 2 - (out[0][1] - out[0][0]) / 2), (1, g1 + P["south_wall"]["big_window_gap"])):
            out[i] = (a, a + out[i][1] - out[i][0], *out[i][2:])
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
# The flat middle slopes down to the north and south walls, to DROP_END there, from the thin north-south trusses'
# outer verticals (user); the side slopes stay as they are.
THIN_PANELS = 12
_ka = round((TRUSS_X[0] - X0) / ((X1 - X0) / THIN_PANELS))
DROP_X = (X0 + (X1 - X0) * (_ka - 1) / THIN_PANELS, X1 - (X1 - X0) * (_ka - 1) / THIN_PANELS)
DROP_END = P["hall"]["drop_end"]


def end_windows():
    """Every steel-sash window unit as (wall, z0, cols, rows): wall -1 north, +1 south."""
    pw = P["south_wall"]["pane"][0]
    return [(w, z0 + k * c * pw, c, r) for w, name in ((-1, "north_sections"), (1, "south_sections"))
            for z0, _, n, c, r in sections(name) for k in range(n)]


def window_rect(z0, cols, rows):
    pw, ph = P["south_wall"]["pane"]
    top = P["south_wall"]["window_top"]
    return z0, z0 + cols * pw, top - rows * ph, top


def holed_wall(g, x0, x1, za, zb, h, holes, material="concrete_white"):
    """A wall slab x0..x1, z za..zb, 0..h with rectangular openings (z0, z1, y0, y1), built from horizontal bands."""
    ys = sorted({0.0, h, *(y for r in holes for y in r[2:])})
    for ya, yb in zip(ys, ys[1:]):
        z = za
        for z0, z1, _, _ in sorted(r for r in holes if r[2] <= ya and r[3] >= yb):
            if z0 > z:
                g.span(material, x0, x1, ya, yb, z, z0)
            z = max(z, z1)
        if z < zb:
            g.span(material, x0, x1, ya, yb, z, zb)


def sash(g, xw, inward, z0, cols, rows):
    """A steel-sash window in the opening of the wall whose inner face is x = xw (inward = +1 on the north wall, -1 on
    the south wall), set a couple of inches back into the wall, its top at the common window top: clear panes of
    P["south_wall"]["pane"], white bars between them, a heavier frame round it. No glass: the engine renders
    opaque materials only, so a pane would close the view; the opening is left clear (user)."""
    pw, ph = P["south_wall"]["pane"]
    z0, z1, y0, y1 = window_rect(z0, cols, rows)
    a, b = sorted((xw - inward * 0.07, xw - inward * 0.09))
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


def run_ibeam(g, p0, p1, d=0.2, f=0.135, t=0.01):
    """A white I-beam along the straight line p0-p1 (x, y, z) on its top edge, its web in the vertical plane through
    the line: d deep, flanges f wide and t thick."""
    v = [b - a for a, b in zip(p0, p1)]
    length = math.sqrt(sum(c * c for c in v))
    u = [c / length for c in v]
    h = [-u[2], 0.0, u[0]]                         # horizontal, square to the line
    hl = math.hypot(h[0], h[2])
    h = [c / hl for c in h]
    up = [h[1] * u[2] - h[2] * u[1], h[2] * u[0] - h[0] * u[2], h[0] * u[1] - h[1] * u[0]]
    side = [up[1] * u[2] - up[2] * u[1], up[2] * u[0] - up[0] * u[2], up[0] * u[1] - up[1] * u[0]]
    r = [[side[i], up[i], u[i]] for i in range(3)]
    m = [(a + b) / 2 for a, b in zip(p0, p1)]
    for size, off in (((f, t, length), t / 2), ((0.008, d - 2 * t, length), d / 2), ((f, t, length), d - t / 2)):
        g.box("steel_white", size, [m[i] - up[i] * off for i in range(3)], r)


def roof_ibeam(g, f, x0, x1, z0, z1, below=0.0, d=0.2, t=0.01):
    """A white I-beam under the sloping roof part f over the plan box x0..x1, z0..z1, running along its longer plan
    side, its top `below` m under the roof, square to it."""
    xm, zm = (x0 + x1) / 2, (z0 + z1) / 2
    kx, kz = roof_frame(f, xm, zm)[2]
    sx, sz = (x1 - x0) * kx, (z1 - z0) * kz
    web = (sx, d - 2 * t, 0.008) if x1 - x0 > z1 - z0 else (0.008, d - 2 * t, sz)
    for size, b in (((sx, t, sz), below), (web, below + t), ((sx, t, sz), below + d - t)):
        on_roof(g, "steel_white", size, xm, zm, b, f)


def build_shell(g):
    length, width = X1 - X0, Z1 - Z0
    xm, zm = (X0 + X1) / 2, (Z0 + Z1) / 2
    # Floor slab, test mat and tape.
    g.box("epoxy_floor", (length + 2 * WALL_T, 0.2, width + 2 * WALL_T), (xm, -0.1, zm))
    g.span("test_mat", X0 + 0.5, X0 + 8.5, 0.0, 0.003, Z1 - 8.5, Z1 - 0.5)
    g.span("tape_yellow", X0, CURTAIN_X[1] + 1.0, 0.0, 0.002, CURTAIN_Z + 2.45, CURTAIN_Z + 2.5)
    # From its end east to the test mat's middle line (user).
    g.span("tape_yellow", CURTAIN_X[1] + 0.95, CURTAIN_X[1] + 1.0, 0.0, 0.002, CURTAIN_Z + 2.45, Z1 - 4.5)

    # Walls: the end walls up to the eave, above it up the side slopes to DROP_END in strips that end inside the roof
    # slab.
    for x, side in ((X0 - WALL_T / 2, -1), (X1 + WALL_T / 2, 1)):
        holes = [window_rect(z0, c, r) for w, z0, c, r in end_windows() if w == side]
        if side > 0:                               # the roll-up gate's opening in the south wall
            holes.append((P["gate"]["z0"], P["gate"]["z1"], 0.0, P["gate"]["height"]))
        holed_wall(g, x - WALL_T / 2, x + WALL_T / 2, Z0 - WALL_T, Z1 + WALL_T, H, holes)
        steps = 12
        hs = (DROP_END - H) / steps
        for i in range(steps):
            half = FLAT + (ZC - Z0 - FLAT) * (1 - i / steps) + WALL_T
            g.box("concrete_white", (WALL_T, hs + 0.001, 2 * half), (x, H + hs * (i + 0.5), ZC))
        g.span("concrete_white", x - WALL_T / 2, x + WALL_T / 2, DROP_END, DROP_END + ROOF_T, ZC - FLAT, ZC + FLAT)
    for z in (Z0 - WALL_T / 2, Z1 + WALL_T / 2):
        g.box("concrete_white", (length + 2 * WALL_T, H + ROOF_T, WALL_T), (xm, (H + ROOF_T) / 2, z))

    # Roof beams' places and the skylights' (between two beams, see below), then the white timber roof deck: one slab
    # per roof part, each with an opening at every skylight; where the flat middle's ends come down below the side
    # slopes the roof is left open (user).
    n, (bf, bd, bg), beams_z = P["hall"]["west_beams"], BEAM, BEAMS_Z
    sky_w = beams_z[1] - beams_z[0] - bf
    sky_x = LIGHT_X
    sky_z = ((beams_z[1] + beams_z[2]) / 2, (beams_z[4] + beams_z[5]) / 2)
    o = WALL_T + 0.2
    zw, ze = (Z0 - o, ZC - FLAT), (ZC + FLAT, Z1 + o)
    xn_, xm_, xs_ = (X0 - o, DROP_X[0]), (DROP_X[0], DROP_X[1]), (DROP_X[1], X1 + o)
    # (part, x range, z range, twisted): the side slopes' corners near the end walls are twisted and made as smooth
    # bent patches (roof_patch); the other parts are flat planes, one board each. Every piece stops at the skylights'
    # openings.
    parts = [("west", xn_, zw, True), ("west", xm_, zw, False), ("west", xs_, zw, True),
             ("east", xn_, ze, True), ("east", xm_, ze, False), ("east", xs_, ze, True),
             ("north", xn_, (ZC - FLAT, ZC + FLAT), False), ("flat", xm_, (ZC - FLAT, ZC + FLAT), False),
             ("south", xs_, (ZC - FLAT, ZC + FLAT), False)]
    holes = [(x - 1.2, x + 1.2, z - sky_w / 2, z + sky_w / 2) for x in sky_x for z in sky_z]
    for f, (x0, x1), (z0, z1), twisted in parts:
        xs = sorted({x0, x1, *(min(max(h[i], x0), x1) for h in holes for i in (0, 1))})
        zs = sorted({z0, z1, *(min(max(h[i], z0), z1) for h in holes for i in (2, 3))})
        for xa, xb in zip(xs, xs[1:]):
            for za, zb in zip(zs, zs[1:]):
                if xb - xa < 1e-6 or zb - za < 1e-6:
                    continue
                if any(h[0] <= xa and xb <= h[1] and h[2] <= za and zb <= h[3] for h in holes):
                    continue
                if twisted:
                    roof_patch(g, f, xa, xb, za, zb)
                else:
                    xc, zc = (xa + xb) / 2, (za + zb) / 2
                    kx, kz = roof_frame(f, xc, zc)[2]
                    on_roof(g, "timber_white", ((xb - xa) * kx, ROOF_T, (zb - za) * kz), xc, zc, -ROOF_T, f)

    # Roof framing (IMG_3106-3108, user): ten white steel I-beams north to south right under the roof, the size of the
    # west wall's I-beam columns: one on top of the columns, centred on them, its mirror on the east wall, eight equally
    # spaced between. Big I-beams on the end walls over the windows (the south one half in the wall, the north one
    # out to the columns' faces); the curtain beam is centred on
    # them and the two east-west A-frame trusses sit on it: small I-beam chords, the top one under the roof beams,
    # small struts between, ending at the side walls' I-beam columns. Two thin trusses of steel angles hang under the
    # fourth and seventh roof beams, north wall to south wall.
    # The roof beams frame into one more along the top of the north wall, following the roof, flush with the faces of
    # the wall's concrete columns, and one along the top of the south wall (user).
    xn, xsw = X0 + COLUMN_D, X1 - bf           # the faces of the beams along the north and south walls' tops
    # Each roof beam is three straight pieces with an upright web, meeting under the roof where it bends: over the side
    # walls at the columns, the others at the drop lines (user).
    cx = [X0 + k * length / (n + 1) for k in range(1, n + 1)]
    for z in beams_z:
        if z in (beams_z[0], beams_z[-1]):
            ends = [xn, *(c + sd * bf / 2 for c in cx for sd in (-1, 1)), xsw]
            runs = list(zip(ends[::2], ends[1::2]))
        else:
            ends = [xn, DROP_X[0], DROP_X[1], xsw]
            runs = list(zip(ends, ends[1:]))
        for xa, xb in runs:
            run_ibeam(g, (xa, roof_y(xa, z), z), (xb, roof_y(xb, z), z), bd, bf)
    # The beams along the north and south walls' tops, under the roof's edge there: up the side slopes, level across
    # the flat middle's end (user).
    zw = Z0 + (ZC - FLAT - Z0) * 0.999
    for xa, xb, end in ((xn - bf, xn, "north"), (xsw, X1, "south")):
        ye = part_y(end, (xa + xb) / 2, ZC)
        for f, za, zb in (("west", Z0, zw), ("east", 2 * ZC - zw, Z1)):
            roof_ibeam(g, f, xa, xb, za, zb, 0.0, bd)
        ibeam(g, xa, xb, ye - bd, ye, ZC - FLAT, ZC + FLAT, 1)
    # The side walls' I-beam columns, the east ones mirroring the west, up to the roof (user).
    for k in range(1, n + 1):
        x = X0 + k * length / (n + 1)
        for z0, z1 in ((Z0 + bg, Z0 + bg + bd), (Z1 - bg - bd, Z1 - bg)):
            ibeam(g, x - bf / 2, x + bf / 2, 0.0, roof_y(x, (z0 + z1) / 2), z0, z1, 2)

    wt = P["south_wall"]["window_top"]
    end_beam = (wt + 0.475, wt + 1.075)   # ~W24, 0.6 m deep, 0.23 m flange; the curtain beam's top at 7.4 m (user)
    ibeam(g, xn - 0.23, xn, end_beam[0], end_beam[1], Z0, Z1, 1)     # north: its face flush with the columns' (user)
    ibeam(g, X1 - 0.115, X1 + 0.115, end_beam[0], end_beam[1], Z0, Z1, 1)
    curtain_beam = (sum(end_beam) / 2 - 0.225, sum(end_beam) / 2 + 0.225)   # ~W18
    bottom = curtain_beam[1]                       # the A-frames' bottom chord sits on the curtain beam
    chord, cf = 0.2, 0.135                         # small I-beam chords (~W8)
    # Both chords run between the side walls' I-beam columns, the top one under the roof beams up the slope and flat
    # across the middle (user); between the chords a vertical strut
    # under each of the eight inner roof beams, one diagonal per panel leaning out from the middle, and an X in the
    # middle panel (user).
    zw, ze = Z0 + bg + bd, Z1 - bg - bd        # the columns' inner faces
    vz = beams_z[1:-1]
    for x in TRUSS_X:
        ibeam(g, x - cf / 2, x + cf / 2, bottom, bottom + chord, zw, ze, 1)
        for za, zb in ((zw, ZC - FLAT), (ZC + FLAT, ze)):      # the sloped parts just under the side beams
            run_ibeam(g, (x, roof_y(x, za) - bd, za), (x, roof_y(x, zb) - bd, zb), chord, cf)
        ibeam(g, x - cf / 2, x + cf / 2, RIDGE - bd - chord, RIDGE - bd, ZC - FLAT, ZC + FLAT, 1)
        for z in vz:
            g.strut("steel_white", (x, bottom + chord, z), (x, roof_y(x, z) - bd - chord, z), 0.08)
        for za, zb in zip(vz, vz[1:]):
            ends = [(za, zb), (zb, za)] if za < ZC < zb else [(zb, za) if zb < ZC else (za, zb)]
            for z0, z1 in ends:                # from the bottom chord at z0 up to the top chord at z1
                g.strut("steel_white", (x, bottom + chord, z0), (x, roof_y(x, z1) - bd - chord, z1), 0.07)
    # The thin trusses' bottoms line up with the A-frames' bottom, a square white plate under each crossing (user).
    for z in (beams_z[3], beams_z[6]):
        yb = bottom + 0.04
        panels = THIN_PANELS

        def yt(x):                             # under the roof beam, which comes down at the ends
            return roof_y(x, z) - bd

        # From each end wall: one three-panel bay with a V and a Λ, then a panel with an X up to the A-frame; between
        # the A-frames no verticals, one zigzag \/\ of three diagonals (user).
        ka = round((TRUSS_X[0] - X0) / (length / panels))
        xs = [X0 + length * k / panels for k in (0, ka - 1, ka, panels - ka, panels - ka + 1, panels)]
        angle(g, (X0, yb, z), (X1, yb, z), 0.08)
        for x in xs:
            angle(g, (x, yb, z), (x, yt(x), z), 0.06)
        for i, (x, xn) in enumerate(zip(xs, xs[1:])):
            out, xc = min(i, len(xs) - 2 - i), (x + xn) / 2
            if out == 0:
                pairs = [((x, yt(x)), (xc, yb)), ((xn, yt(xn)), (xc, yb)), ((x, yb), (xc, yt(xc))),
                         ((xn, yb), (xc, yt(xc)))]
            elif out == 1:
                pairs = [((x, yb), (xn, yt(xn))), ((x, yt(x)), (xn, yb))]
            else:
                zig = [(x + (xn - x) * j / 3, (yt(x + (xn - x) * j / 3), yb)[j % 2]) for j in range(4)]
                pairs = list(zip(zig, zig[1:]))
            for (xa, ya), (xb, yb_) in pairs:
                angle(g, (xa, ya, z), (xb, yb_, z), 0.06)
        for x in TRUSS_X:
            g.span("steel_white", x - 0.25, x + 0.25, bottom - 0.02, bottom, z - 0.25, z + 0.25)

    # Skylights: three rows north to south, between the trusses, in a west and a middle column, none on the east side
    # over the mezzanine (user). Each fills the gap between two roof beams (the second gap from the west wall, on the
    # slope, and the middle one, in the flat): an open hole in the deck (no glass: the engine has no transparency, so a
    # pane would read as an opaque blue sheet; user), louvre bars just
    # under it, a white frame flush with the ceiling round the opening. The north row's outer frame edge lines up with
    # the back of the desk row by the north wall, the south row's sits as far from the south wall (user).
    for x in sky_x:
        for z in sky_z:
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
        g.span("concrete_white", X0, X0 + COLUMN_D, 0.0, roof_y(X0, c) - bd, c - 0.15, c + 0.15)

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
    g.place(thick, "curtain_denim", (1, 1, 1), ((CURTAIN_X[0] + CURTAIN_X[1]) / 2, 0.05 + ch / 2, CURTAIN_Z + 0.012),
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
        g.tube("steel_dark", (x, ty, z), (x, roof_y(x, z), z), 0.008)

    # South wall, east to west behind the U stair: the big steel-sash window, the fire alarm gear, the steel exit door,
    # plain wall, the closed roll-up gate with its hood and three window units above it. Sizes from office_params.toml.
    G, D, S = P["gate"], P["exit_door"], P["south_wall"]
    rz0, rz1, gh = G["z0"], G["z1"], G["height"]
    sx = X1 - 0.03
    # Two grey guide channels stand in the opening's sides, in front of the wall's inner face, one rail depth into the hall (user). The curtain is
    # as thick as they are and runs in them, rolled up part way (user: half open), its bottom bar over the open part,
    # the same black slats on both its faces (user), recessed in the opening on the outside (IMG_3113).
    gb, gw = gh * G["open"], 0.12
    gx = X1 - gw                                      # rails and door stand one rail depth inside the hall (user)
    for z0, z1 in ((rz0, rz0 + gw), (rz1 - gw, rz1)):
        g.span("gate_rail", gx, gx + gw, 0.0, gh + 0.05, z0, z1)
    # Behind each rail (outside it) the wall fills the opening through its thickness, so the opening's sides are flush
    # with the rails' inner sides (user).
    for z0, z1 in ((rz0, rz0 + gw), (rz1 - gw, rz1)):
        g.span("concrete_white", X1, X1 + WALL_T, 0.0, gh, z0, z1)
    ca, cb = rz0 + gw, rz1 - gw                       # exactly between the rails, no overlap (it z-fought with them)
    g.span("gate_alu", gx, gx + gw, gb, gh, ca, cb)
    g.span("steel_dark", gx - 0.01, gx + gw + 0.01, gb, gb + 0.06, ca, cb)
    for k in range(int((gh - gb) / 0.15)):
        for sx_ in (gx - 0.0075, gx + gw + 0.0075):
            g.box("steel_dark", (0.015, 0.015, cb - ca), (sx_, gb + 0.15 + k * 0.15, (ca + cb) / 2))
    hood_r = (G["hood_top"] - gh) / 2
    # Roller hood: a true (64-sided) drum against the wall's inner face, lowered 0.1 m to overlap the opening's top
    # (user: it looked detached).
    hx = X1 - hood_r + 0.03                           # 3 cm into the wall (user)
    g.tube("rollup_grey", (hx, gh + hood_r - 0.1, rz0 - 0.2), (hx, gh + hood_r - 0.1, rz1 + 0.2), hood_r,
           smooth=True)
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
    pe = P["u_stair"]["lower_z"][0] - 0.12 - 0.16 - dz1     # the red panel ends 16 cm short of the U stair (user)
    for y0, y1, a, b, key in ((1.31, 1.79, 0.35, pe, "red_cross"), (1.8, 2.06, 0.55, 0.72, "clock_face"),
                              (1.26, 1.39, 0.14, 0.33, "panel_grey"), (1.28, 1.39, 0.07, 0.13, "panel_grey"),
                              (1.15, 1.29, 0.33, 0.43, "red_cross")):
        g.span(key, sx - 0.1, sx, y0, y1, dz1 + a, dz1 + b)
    for zp, yh, zr, top in ((dz1 + 0.44, 2.6, dz0 - 0.17, H - 0.1),
                            (dz1 + 0.5, 2.54, dz0 - 0.31, G["hood_top"] + 0.23)):
        pts = [(1.79, zp), (yh, zp), (yh, zr), (top, zr)] + ([(top, Z0)] if top < 6 else [])
        for (ya, za), (yb, zb) in zip(pts, pts[1:]):
            g.tube("conduit_red", (sx - 0.03, ya, za), (sx - 0.03, yb, zb), 0.012)
    # A grey junction box on the lower one over the exit door, where it turns up west of the door (IMG_3111, user).
    jy, jz = 2.54, dz0 - 0.31
    g.span("panel_grey", sx - 0.06, sx, jy - 0.06, jy + 0.06, jz - 0.06, jz + 0.06)
    # The first one goes on from its top west under the south wall's top beam to the next roof beam, then north under
    # that beam to between the north skylights' south edge and where the ceiling starts sloping down (user).
    zb2 = BEAMS_Z[2]
    xe = (X0 + P["desk_corner"]["row_wall_gap"] + 2.6 + DROP_X[0]) / 2

    def under(x, z):
        return roof_y(x, z) - BEAM[1] - 0.03

    route = [(sx - 0.03, H - 0.1, dz0 - 0.17), (sx - 0.03, under(X1, zb2), zb2),
             *((x, under(x, zb2), zb2) for x in (DROP_X[1], DROP_X[0], xe))]
    for a, b in zip(route, route[1:]):
        g.tube("conduit_red", a, b, 0.012)

    # Conduit runs, electrical panels, extinguisher, bollards.
    g.tube("conduit", (X0 + 0.05, 1.25, Z0), (X0 + 0.05, 1.25, Z1), 0.012)
    g.tube("conduit", (X0, 1.15, Z0 + 0.05), (X1, 1.15, Z0 + 0.05), 0.012)
    g.tube("conduit", (X0, 6.1, Z0 + 0.06), (X1, 6.1, Z0 + 0.06), 0.015)
    g.tube("conduit", (X0, 1.2, Z1 - 0.05), (KITCHEN_X - 0.3, 1.2, Z1 - 0.05), 0.012)
    g.tube("conduit_red", (X0 + 6.0, 2.85, Z1 - 0.06), (BOX[0], 2.85, Z1 - 0.06), 0.01)
    g.span("panel_grey", X0, X0 + 0.15, 1.1, 1.85, CURTAIN_Z + 0.4, CURTAIN_Z + 0.9)
    g.span("panel_grey", 3.9, 4.5, 1.0, 1.9, Z0, Z0 + 0.15)
    g.tube("red_cross", (X0 + 0.12, 0.9, CURTAIN_Z + 1.2), (X0 + 0.12, 1.4, CURTAIN_Z + 1.2), 0.07)
    for x in (X1 - 0.4, X1 - 0.9):     # in front of the gate's west guide channel, west faces flush with its west side (user)
        z = P["gate"]["z0"] + 0.07
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
        # Back panel and exhaust hood flush with the brown box's sides (user); the hood's back with its back too.
        g.box("alu_matte", (0.04, 0.9, 0.6), at(-0.195, 6.05, 0.0), r)
        g.tube("alu_matte", at(-0.215, 6.05, 0.0), at(-0.315, 6.05, 0.0), 0.26)
        g.box("alu_matte", (0.485, 0.18, 0.6), at(0.0675, 5.51, 0.0), r)
        for k in range(3):
            g.box("steel_dark", (0.01, 0.025, 0.56), at(0.315, 5.45 + 0.055 * k, 0.0), r)
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
        top = bottom if z in (beams_z[3], beams_z[6]) else roof_y(x, z) - bd  # a truss or the roof beam above
        g.span("led_white", x - 0.3, x + 0.3, 6.35, 6.42, z - 0.12, z + 0.12)
        for xe in (x - 0.27, x + 0.27):        # two grey ropes from its narrow ends (user)
            g.tube("conduit", (xe, 6.42, z), (xe, top, z), 0.004)


# ---------------------------------------------------------------- office block and mezzanine
def deck_rail(g, p0, p1, base=BOX_DECK, height=1.0, first=True):
    """A railing from p0 to p1 (x, z) standing on `base`: dark brown round balusters 0.1 m apart, the same as the
    spiral stairs' (user), under a round anthracite top rail `height` up. first=False leaves out the baluster at p0
    (when p0 is inside a post)."""
    n = max(1, int(math.dist(p0, p1) / 0.1))
    for i in range(0 if first else 1, n + 1):
        x, z = p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n
        g.tube("rail_brown", (x, base, z), (x, base + height, z), 0.012)
    g.tube("anthracite", (p0[0], base + height, p0[1]), (p1[0], base + height, p1[1]), 0.025)


def cable_rail(g, p0, p1, out=(-1.0, 0.0)):
    """Galvanised tube posts ~1.5 m apart on the line p0-p1 just outside the deck's fascia, each bolted to it with a
    plate on its inner side (`out` points away from the deck, in x, z), a tube top rail, nine steel cables."""
    n = max(1, round(math.dist(p0, p1) / 1.5))
    for i in range(n + 1):
        t = i / n
        x, z = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
        g.tube("steel_grey", (x, DECK - 0.25, z), (x, DECK + 1.07, z), 0.024)
        px, pz = x - out[0] * 0.035, z - out[1] * 0.035
        g.span("steel_grey", px - 0.01 - 0.06 * abs(out[1]), px + 0.01 + 0.06 * abs(out[1]), DECK - 0.28, DECK - 0.02,
               pz - 0.01 - 0.06 * abs(out[0]), pz + 0.01 + 0.06 * abs(out[0]))
    g.tube("steel_grey", (p0[0], DECK + 1.07, p0[1]), (p1[0], DECK + 1.07, p1[1]), 0.024)
    for k in range(9):
        y = DECK + 0.12 + k * 0.1
        g.tube("panel_grey", (p0[0], y, p0[1]), (p1[0], y, p1[1]), 0.006)


def slat_rail(g, p0, p1, inward, outside=False, first=True, ext0=0.0, ext1=0.0, ends_only=False):
    """A grey railing along the nosing or edge line p0-p1 (x, y, z): square posts no more than 1.5 m apart, flush with
    the edge on its inner side (`inward`, in x, z), or bolted outside a fascia board and reaching down its face, and
    a flat handrail lying on the posts' tops, four upright flat bars under it on the posts' inner side (IMG_3066/3069,
    user)."""
    s, t = 0.11, 0.3 if outside else 0.0          # square posts as wide as the handrail (user); how far down a fascia
    off = -(0.05 + s / 2) if outside else s / 2
    n = 1 if ends_only else max(1, math.ceil(math.dist(p0, p1) / 1.5))   # sloped flights: end posts only (user)
    for i in range(0 if first else 1, n + 1):
        x, y, z = (a + (b - a) * i / n for a, b in zip(p0, p1))
        g.box("stair_grey", (s, 1.05 + t, s), (x + inward[0] * off, y + (1.05 - t) / 2, z + inward[1] * off))
    bo = off + s / 2 + 0.0125
    a = (p0[0] + inward[0] * off, p0[1] + 1.05 + 0.0125, p0[2] + inward[1] * off)      # flat handrail on the posts
    b = (p1[0] + inward[0] * off, p1[1] + 1.05 + 0.0125, p1[2] + inward[1] * off)
    u = [q - p_ for p_, q in zip(a, b)]                # it runs on ext0 / ext1 past the end posts (free ends only, so
    ul = math.sqrt(sum(c * c for c in u))              # joined rails don't overrun each other), wider than them (user)
    a = tuple(p_ - c / ul * ext0 for p_, c in zip(a, u))
    b = tuple(q + c / ul * ext1 for q, c in zip(b, u))
    flat_bar(g, "stair_grey", a, b, 0.11, 0.025)
    for h in (0.25, 0.45, 0.65, 0.85):
        a = (p0[0] + inward[0] * bo, p0[1] + h - 0.033, p0[2] + inward[1] * bo)
        b = (p1[0] + inward[0] * bo, p1[1] + h - 0.033, p1[2] + inward[1] * bo)
        flat_bar(g, "stair_grey", a, b, 0.025, 0.065)


def flat_bar(g, key, p0, p1, thick, tall):
    """A flat bar along p0-p1 standing on edge: `thick` sideways, `tall` up, its middle on the line."""
    u = [b - a for a, b in zip(p0, p1)]
    length = math.sqrt(sum(c * c for c in u))
    u = [c / length for c in u]
    hl = math.hypot(u[0], u[2])
    side = [-u[2] / hl, 0.0, u[0] / hl]
    up = [u[1] * side[2] - u[2] * side[1], u[2] * side[0] - u[0] * side[2], u[0] * side[1] - u[1] * side[0]]
    if up[1] < 0:                                  # keep it right-handed (side x up = along) with up pointing up
        up, side = [-c for c in up], [-c for c in side]
    g.box(key, (thick, tall, length), [(a + b) / 2 for a, b in zip(p0, p1)],
          [[side[i], up[i], u[i]] for i in range(3)])


def spiral(g, cx, cz, r, y0, y1, treads, start, turn, post_top=1.0):
    """A spiral stair round one anthracite post from y0 up to the top rails' height: per step a rectangular wood tread
    from the post out, a slim black bracket under it, two dark brown balusters near its outer corners right up into
    the handrail; the bottom tread points along yaw `start`, the top one `turn` radians further round."""
    rise = (y1 - y0) / treads
    step = turn / (treads - 1)
    hub = 0.065
    rr = r - 0.03                      # handrail radius; the balusters stand on it, 0.1 m either side of each tread's middle
    da = 0.1 / rr

    def rail_y(a):
        return y0 + rise * (1 + (treads - 1) * (a - start) / turn) + 0.9

    g.tube("anthracite", (cx, y0, cz), (cx, y1 + post_top, cz), hub)  # one post, up to the top rails (user)
    for i in range(treads):
        a = start + step * i
        y = y0 + rise * (i + 1)
        d = (math.cos(a), -math.sin(a))
        mid = (hub + r) / 2
        g.box("wood_tread", (r - hub, 0.035, 0.24), (cx + d[0] * mid, y - 0.018, cz + d[1] * mid), yaw(a))
        mid = (hub + r - 0.08) / 2
        g.box("steel_dark", (r - 0.08 - hub, 0.06, 0.03), (cx + d[0] * mid, y - 0.065, cz + d[1] * mid), yaw(a))
        for side in (-1, 1):
            b = a + side * da
            foot = (cx + math.cos(b) * rr, y, cz - math.sin(b) * rr)
            g.tube("rail_brown", foot, (foot[0], rail_y(b), foot[2]), 0.012)
    # Handrail: a smooth helix through the baluster tops, a little past the first and last ones.
    m = 4 * (treads - 1)
    a0, a1 = start - math.copysign(da, turn), start + turn + math.copysign(da, turn)
    prev = None
    for j in range(m + 1):
        a = a0 + (a1 - a0) * j / m
        pt = (cx + math.cos(a) * rr, rail_y(a), cz - math.sin(a) * rr)
        if prev:
            g.tube("anthracite", prev, pt, 0.025)
        prev = pt
    return prev


def u_stair(g):
    """The grey dog-leg stair west of the mezzanine: a top platform at the mezzanine front, the upper flight south down
    to a landing against the south wall at half the deck height, the lower flight back north to the floor, slat
    railings, closed risers and a white closure wall under the lower flight."""
    U = P["u_stair"]
    n = U["risers"]
    rise, land = DECK / (2 * n), DECK / 2
    (px0, px1), (pa, pb) = U["platform_x"], U["upper_z"]
    # The flights and the landing sit one fascia and one post out from the top platform, so their railings line up
    # with the platform's, which stands outside its fascia (user).
    d = -(0.05 + 0.07)
    za, zb = pa - 0.02, pb + d      # upper flight: its west stringer's outer face flush with the platform's fascia (user)
    (xl, xe), (lz0, lz1), (zc, zd) = U["landing_x"], [v + d for v in U["landing_z"]], [v + d for v in U["lower_z"]]
    go = (xl - px1) / n
    g.span("stair_grey", px0, px1, DECK - 0.15, DECK, pa, pb)
    g.span("stair_grey", xl, xe, land - 0.2, land, lz0, lz1)
    gu = (xl - px1) / (n - 1)          # the upper flight's n - 1 treads fill its whole run (no gap above the landing)
    for k in range(1, n + 1):
        x0 = px1 + gu * (k - 1)
        if k < n:
            g.span("stair_grey", x0, x0 + gu, DECK - rise * k - 0.05, DECK - rise * k, za, zb)
        g.span("stair_grey", x0, x0 + 0.03, DECK - rise * k, DECK - rise * (k - 1), za, zb)
        x0, x1 = xl - go * k, xl - go * (k - 1)
        g.span("stair_grey", x0, x1, land - rise * k - 0.05, land - rise * k, zc, zd)
        g.span("stair_grey", x1 - 0.03, x1, land - rise * k, land - rise * (k - 1), zc, zd)
    g.span("paint_white", xl, xe, 0.0, land - 0.2, lz0 - 0.05, lz0)
    for z in (za, zb):                             # stringers: plates on edge along each flight's sides
        flat_bar(g, "stair_grey", (px1, DECK - 0.15, z), (xl, land - 0.15, z), 0.06, 0.3)
    xs = xl - go * (n - 1)                              # the lower flight's first post
    for z in (zc, zd):                                  # they end at the first step's post (user)
        ys = land - 0.15 - (land - 0.1) * (n - 1) / n
        flat_bar(g, "stair_grey", (xl, land - 0.15, z), (xs, ys, z), 0.06, 0.3)
    # White closure wall under the lower flight: one triangle-like panel, its top a straight slope along the stringer's
    # lower edge, its outer face flush with the stringer's, from the landing down to the flight's first post (IMG_3111,
    # user: no sawtooth).
    sl = (land - 0.15 + 0.05) / (go * n)                # stringer slope
    th = math.atan(sl)
    yl = land - 0.15 - 0.15 / math.cos(th)              # stringer's lower edge over x = xl
    c, s_ = math.cos(th), math.sin(th)
    wz0, wz1 = zc - 0.03, zc + 0.05
    g.cut_box("paint_white", (xl - xs, yl, wz1 - wz0), ((xs + xl) / 2, yl / 2, (wz0 + wz1) / 2),
              (20.0, 4.0, 1.0), (xl - s_ * 2.0, yl + c * 2.0, (wz0 + wz1) / 2), [[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    for z in (zc + 0.05, za - 0.05):
        g.box("stair_grey", (0.08, land, 0.08), (xl + 0.05, land / 2, z))
    # Upper flight's railing bolted outside its west stringer, in line with the platform's (the top platform's post is
    # its first); the lower flight's run on 15 cm past their bottom posts (user).
    slat_rail(g, (px1, DECK, pa), (xl, land, pa), (0.0, 1.0), outside=True, first=False, ends_only=True)
    slat_rail(g, (xl, land, zc), (xs, rise, zc), (0.0, 1.0), ext1=0.15, ends_only=True)
    slat_rail(g, (xl, land, zd), (xs, rise, zd), (0.0, -1.0), ext1=0.15, ends_only=True)
    # One post at the middle of the upper flight, outside its stringer, from the floor up to the handrail (IMG_3111).
    mxp, myp = (px1 + xl) / 2, (DECK + land) / 2 + 1.05
    g.box("stair_grey", (0.11, myp, 0.11), (mxp, myp / 2, pa - 0.105))
    g.span("deck_fascia", px0, px1, DECK - 0.3, DECK, pa - 0.05, pa)      # the top platform's side, like the deck's
    slat_rail(g, (px0, DECK, pa), (px1, DECK, pa), (0.0, 1.0), outside=True)
    slat_rail(g, (xl, land, lz0), (xe - 0.055, land, lz0), (0.0, 1.0), ext1=0.055)   # ends flush with the wall (user)


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
    # goes up the west face to the deck (IMG_3103).
    g.tube("conduit_red", (bx0 - 0.03, 2.6, bz0), (bx0 - 0.03, 2.6, bz1), 0.01)
    g.tube("conduit_red", (bx0, 2.6, bz0 - 0.03), (bx1, 2.6, bz0 - 0.03), 0.01)
    xr = mx0 + 0.3
    for x in (xc - 1.45, xc, xr):
        g.span("panel_grey", x - 0.06, x + 0.06, 2.54, 2.66, bz0 - 0.06, bz0)
    g.tube("conduit_red", (xr, 2.66, bz0 - 0.03), (xr, under, bz0 - 0.03), 0.01)
    # Lower deck edge: a dark bolted steel channel round its north and west edges, the railing on it (IMG_3104/3087).
    north_end = B["mid_box"][2]
    g.span("steel_dark", bx0 - 0.05, bx0, BOX_DECK - 0.22, BOX_DECK + 0.03, bz0 - 0.05, bz1)
    # The big spiral comes up onto a square wood landing on a dark steel frame in the corner between its post, the
    # box and the upper deck's front (flush with it); the deck's railing turns 90 degrees along the landing's north
    # edge to the post, where the spiral's handrail goes on (IMG_3119, user).
    lx0, lx1, lz0 = S["x"], mx0, S["z"]
    g.span("steel_dark", bx0, lx0, BOX_DECK - 0.22, BOX_DECK + 0.03, bz0 - 0.05, bz0)
    for x in [bx0 + 0.15 + 0.3 * k for k in range(int((lx0 - bx0) / 0.3))]:
        for y in (BOX_DECK - 0.16, BOX_DECK - 0.03):
            g.span("nickel", x - 0.012, x + 0.012, y - 0.012, y + 0.012, bz0 - 0.06, bz0 - 0.05)
    deck_rail(g, (bx0 + 0.03, bz0), (bx0 + 0.03, north_end))
    g.span("wood_tread", lx0, lx1, BOX_DECK - 0.04, BOX_DECK + 0.005, lz0, bz0)
    for x0, x1, z0, z1 in ((lx0, lx0 + 0.03, lz0, bz0), (lx0, lx1, lz0, lz0 + 0.03)):
        g.span("steel_dark", x0, x1, BOX_DECK - 0.14, BOX_DECK - 0.04, z0, z1)
    deck_rail(g, (bx0, bz0 + 0.03), (lx0 + 0.02, bz0 + 0.03))
    deck_rail(g, (lx0 + 0.02, bz0 + 0.03), (lx0 + 0.02, lz0))

    # Upper deck: one slab from its front to the south wall, from the U stair to the east wall; over the hall a front
    # beam on two white posts and the cable railing, over the box deck open (the small spiral comes up there and the
    # tall box stands against it).
    t = M["thickness"]
    g.span("deck_fascia", mx0, mx1, DECK - 0.3, DECK, mz0, Z1)
    g.span("plywood", mx0, mx1, DECK, DECK + 0.005, mz0, Z1)              # plywood floors (user)
    g.span("plywood", bx0, mx0, BOX_DECK, BOX_DECK + 0.005, bz0, bz1)
    g.span("paint_white", mx0 + 0.02, mx1, under, DECK - 0.3, mz0, Z1)
    # Steel I-beams: two posts up to the deck, the front beam between and beside them, joining their sides (IMG_3089).
    bf, bd = M["beam"]
    pz = sorted(M["posts_z"])
    ends = [mz0, *(z + sd * bf / 2 for z in pz for sd in (-1, 1)), mz1]
    for z0, z1 in zip(ends[::2], ends[1::2]):
        if z1 - z0 > 0.01:
            ibeam(g, mx0, mx0 + bf, DECK - t, DECK - 0.3, z0, z1, 1)
    for z in pz:
        ibeam(g, mx0, mx0 + bd, 0.0, DECK - 0.3, z - bf / 2, z + bf / 2, 0)
    # Cable railing along the front from the U stair to the small spiral (over the box deck too), and on the deck's
    # inner edge above the upper flight down to the south wall (IMG_3066/3087/3089).
    # The grey fascia board and the cable railing go on along the front over the U stair's top platform (IMG_3069).
    za_ = P["u_stair"]["upper_z"][0]
    g.span("deck_fascia", mx0, mx0 + 0.05, DECK - 0.3, DECK, za_, mz0)
    cable_rail(g, (mx0 - 0.06, za_), (mx0 - 0.06, T["z"]))
    cable_rail(g, (P["u_stair"]["platform_x"][1], mz0 - 0.06), (mx1, mz0 - 0.06), out=(0.0, -1.0))

    # Under it, all walls up to the deck's underside: the room's west wall from the front wall to the south wall (the U
    # stair flush against it; open beside the sofa), the front wall with the closed French doors, transoms, pendants and
    # the sofa; east of the box's west face line it goes on as the south wall of a small open closet (a glazed side
    # door in it), which ends at a wall with a dark doorway.
    px, top = PARTITION_X, under
    (d0, d1), (sz0, sz1), ce = Q["doors_z"], Q["side_door_z"], Q["closet_z1"]
    g.span("paint_white", px, X1, 0.0, top, mz0, mz0 + 0.12)
    g.span("paint_white", bx1, px, 0.0, top, ce, ce + 0.12)
    (ty0, ty1) = Q["transom_y"]
    transoms = [(zc - w / 2, zc + w / 2) for zc, w in zip(Q["transoms_z"], Q["transom_w"])]
    holed_wall(g, px, px + 0.15, mz0, Z1, top, [(d0, d1, 0.0, 2.21), (sz0, sz1, 0.0, 2.21),
                                                *((za, zb, ty0, ty1) for za, zb in transoms)], "paint_white")
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
    # Transoms: open in the wall (no glass, the engine renders opaque only; user), a matte aluminium frame on all four
    # sides (outer size from the params).
    fw = 0.045
    for za, zb in transoms:
        for y0, y1, z0, z1 in ((ty0, ty0 + fw, za, zb), (ty1 - fw, ty1, za, zb), (ty0, ty1, za, za + fw),
                               (ty0, ty1, zb - fw, zb)):
            g.span("alu_matte", px - 0.035, px, y0, y1, z0, z1)
    sofa(g, px, Q["sofa_z"])
    # White enamel barn pendants on a cord from the joists: a narrow neck, the shade flaring out from it, a short
    # straight rim at the bottom (IMG_3070/3091, user).
    for z in Q["pendants_z"]:
        x = px - 0.8
        g.tube("conduit", (x, 2.7, z), (x, top, z), 0.004)    # grey cord up to the deck's underside (user)
        g.tube("clock_face", (x, 2.62, z), (x, 2.7, z), 0.03)
        g.place(g.shade, "clock_face", (0.18, 0.14, 0.18), (x, 2.55, z))
        g.tube("clock_face", (x, 2.43, z), (x, 2.48, z), 0.18)
    g.span("glass_light", px + 0.07, px + 0.08, 0.1, 2.05, sz0 + 0.05, sz1 - 0.05)
    for z0 in (sz0, sz1 - 0.06):
        g.span("wood_door", px - 0.02, px + 0.17, 0.0, 2.21, z0, z0 + 0.06)
    g.span("wood_door", px - 0.02, px + 0.17, 2.13, 2.21, sz0, sz1)
    g.span("backsplash", bx1 + 0.3, px - 0.3, 0.0, 2.13, ce - 0.01, ce)
    g.span("panel_green", bx1, bx1 + 0.06, 1.2, 1.6, ce - 0.7, ce - 0.4)
    # Under the deck in front of the room (IMG_3070/3089/3091), under the grey deck: from each post a deeper
    # north-south I-beam back to the front wall; white open-web joists ~0.7 m apart hung between them; the red
    # sprinkler main below them along the front wall, with a drop at its east end and two heads.
    bt, cb = DECK - 0.3, M["cross_beam"]
    for z in pz:                                   # from each post back to the front wall, deeper (IMG_3091, user)
        ibeam(g, mx0 + bf, px, bt - cb, bt, z - bf / 2, z + bf / 2, 1)
    ja, jb = pz[0] + bf / 2, pz[1] - bf / 2
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
    rx, ry = px - 0.35, bt - cb - 0.1
    g.tube("conduit_red", (rx, ry, mz1 - 0.1), (rx, ry, -0.3), 0.025)
    g.tube("conduit_red", (rx, ry, 2.0), (rx, ry - 0.4, 2.0), 0.025)
    for z in (1.0, -0.3):
        g.tube("conduit_red", (rx, ry, z), (rx, ry - 0.12, z), 0.012)
        g.tube("nickel", (rx, ry - 0.12, z), (rx, ry - 0.14, z), 0.04)
    u_stair(g)

    # Spiral stairs: floor -> lower deck just west of the box, lower deck -> upper deck where the mezzanine starts.
    spiral(g, S["x"], S["z"], S["radius"], 0.0, BOX_DECK, S["treads"], S["start"], S["turn"])
    # Its post reaches up to the upper deck's grey top rail (DECK + 1.07, user); no bar from the handrail's end to it.
    hand = spiral(g, T["x"], T["z"], T["radius"], BOX_DECK, DECK, T["treads"], T["start"], T["turn"], post_top=1.07)
    # The small spiral's top: a square wood walk-in platform on a dark steel frame at the upper deck's level, south of
    # its post between the top tread and the upper deck; a railing along its back by the tall box, going on from the
    # stair's handrail at its height, and one along its west side from the post, meeting the upper deck's railing
    # (IMG_3116, user).
    tx, tz, tr = T["x"], T["z"], T["radius"]
    g.span("wood_tread", tx, mx0, DECK - 0.045, DECK, tz, tz + tr)
    for x0, x1, z0, z1 in ((tx, tx + 0.03, tz, tz + tr), (tx, mx0, tz + tr - 0.03, tz + tr)):
        g.span("steel_dark", x0, x1, DECK - 0.14, DECK - 0.045, z0, z1)
    deck_rail(g, (hand[0], hand[2]), (mx0, hand[2]), DECK, hand[1] - DECK)
    deck_rail(g, (tx, tz), (mx0 - 0.06, tz), DECK, 1.07, first=False)   # no baluster inside the post (user)


# ---------------------------------------------------------------- kitchen
def shaker_door(g, x0, x1, y0, y1, zf):
    """A white shaker door in front of plane z=zf (facing -z): slab, raised rails and a nickel handle."""
    g.span("cabinet_white", x0 + 0.003, x1 - 0.003, y0 + 0.003, y1 - 0.003, zf - 0.02, zf)
    for a, b, c, d in ((x0 + 0.003, x0 + 0.07, y0, y1), (x1 - 0.07, x1 - 0.003, y0, y1),
                       (x0, x1, y0 + 0.003, y0 + 0.07), (x0, x1, y1 - 0.07, y1 - 0.003)):
        g.span("cabinet_white", a, b, c, d, zf - 0.03, zf - 0.02)


def base_run(g, x0, x1, doors, zf, counter=True, drawers=False, top=0.876):
    g.span("backsplash", x0, x1, 0.0, 0.1, zf + 0.05, Z1)
    g.span("cabinet_white", x0, x1, 0.1, top, zf, Z1)   # top < 0.876 leaves room for a sink basin
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


def microwave(g, x0, x1, y0, zf, h=0.42, d=0.40):
    """An over-the-range stainless microwave over x0..x1 from y0 up, its front at zf (IMG_3105): a dark vent strip
    across the top, the door's big dark window on the left, a straight stainless bar handle at the door's right edge,
    a dark control panel on the right."""
    xd = x0 + (x1 - x0) * 0.78                     # the door's right edge
    g.span("stainless", x0, x1, y0, y0 + h, zf, zf + d)
    g.span("backsplash", x0 + 0.01, x1 - 0.01, y0 + h - 0.07, y0 + h - 0.01, zf - 0.004, zf)
    g.span("glass_dark", x0 + 0.05, xd - 0.09, y0 + 0.06, y0 + h - 0.11, zf - 0.004, zf)
    g.span("backsplash", xd + 0.02, x1 - 0.03, y0 + 0.06, y0 + h - 0.11, zf - 0.004, zf)
    hx = xd - 0.045
    ends = [(hx, y0 + 0.08), (hx, y0 + h - 0.13)]   # one straight vertical bar on two standoffs (user)
    g.tube("stainless", (hx, ends[0][1], zf - 0.04), (hx, ends[1][1], zf - 0.04), 0.011)
    for xa, ya in ends:
        g.tube("stainless", (xa, ya, zf - 0.04), (xa, ya, zf), 0.008)


def build_kitchen(g):
    x = KITCHEN_X
    zf = Z1 - 0.61                    # base cabinet fronts left of the range
    zr = Z1 - 0.70                    # right of it the cabinets, counter and pantry are 0.70 m deep (user)
    f1, f2 = x, x + 0.88              # fridges 0.88 m wide each (user), stainless like the other appliances
    c1 = x + 2 * 0.88
    rng = c1 + 0.45                   # range 0.80 m wide, the microwave over it the same (user)
    dr = rng + 0.80
    dw = dr + 0.6                     # 24" dishwasher
    sk = dw + 0.61
    pa = sk + 1.28                    # sink cabinets
    end = KITCHEN_END                 # the pantry fills the rest, ending short of the restroom (user)
    for fx in (f1, f2):
        fc = fx + 0.454
        g.kit("appliances/fridge_v2", (fc, 0.0, Z1 - 0.45), turn=math.pi,
              width=0.88, height=1.775, depth=0.86, steel=STAINLESS)   # same stainless as the others (user)
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
    base_run(g, dr, dw, 1, zr, drawers=True)
    g.kit("appliances/dishwasher_v2", ((dw + sk) / 2, 0.005, Z1 - 0.40), turn=math.pi,
          width=0.61, height=0.86, depth=0.6, steel=STAINLESS)
    g.span("stainless", dw + 0.01, sk - 0.01, 0.1, 0.86, zr - 0.02, zr)
    g.tube("nickel", (dw + 0.08, 0.78, zr - 0.05), (sk - 0.08, 0.78, zr - 0.05), 0.01)
    g.span("counter_black", dw, sk, 0.876, 0.91, zr - 0.03, Z1)
    # Sink: a brushed aluminium undermount basin 0.6 x 0.42 x 0.2 m in a hole in the counter (user); the sink cabinet's
    # top is lowered to make room and filled back up round the basin.
    sc = (sk + pa) / 2
    bx0, bx1, bz0, bz1, by = sc - 0.3, sc + 0.3, zr + 0.08, zr + 0.50, 0.70
    base_run(g, sk, pa, 2, zr, counter=False, top=by - 0.02)
    for a, b, c, d in ((sk, pa, zr, bz0), (sk, pa, bz1, Z1), (sk, bx0, bz0, bz1), (bx1, pa, bz0, bz1)):
        g.span("cabinet_white", a, b, by - 0.02, 0.876, max(c, zr), d)
        g.span("counter_black", a, b, 0.876, 0.91, zr - 0.03 if c == zr else c, d)
    g.span("alu_sink", bx0, bx1, by - 0.01, by, bz0, bz1)                       # floor
    for a, b, c, d in ((bx0, bx1, bz0, bz0 + 0.01), (bx0, bx1, bz1 - 0.01, bz1),
                       (bx0, bx0 + 0.01, bz0, bz1), (bx1 - 0.01, bx1, bz0, bz1)):
        g.span("alu_sink", a, b, by - 0.01, 0.91, c, d)                       # walls
    for a, b, c, d in ((bx0 - 0.02, bx1 + 0.02, bz0 - 0.02, bz0), (bx0 - 0.02, bx1 + 0.02, bz1, bz1 + 0.02),
                       (bx0 - 0.02, bx0, bz0, bz1), (bx1, bx1 + 0.02, bz0, bz1)):
        g.span("alu_sink", a, b, 0.91, 0.913, c, d)                           # lip on the counter
    g.tube("pc_black", (sc, by, (bz0 + bz1) / 2), (sc, by + 0.003, (bz0 + bz1) / 2), 0.04)  # drain
    # Faucet, thicker (user): a base, the riser and the gooseneck spout reaching over the basin.
    g.tube("nickel", (sc, 0.91, Z1 - 0.08), (sc, 0.94, Z1 - 0.08), 0.04)
    # Inverted J (user): the riser, a half-circle arc over to the front, a short straight drop over the basin.
    g.tube("nickel", (sc, 0.94, Z1 - 0.08), (sc, 1.20, Z1 - 0.08), 0.025)
    R, cz = 0.11, Z1 - 0.08 - 0.11
    arc = [(1.20 + R * math.sin(math.pi * k / 10), cz + R * math.cos(math.pi * k / 10)) for k in range(11)]
    for (ya, za), (yb, zb) in zip(arc, arc[1:]):
        g.tube("nickel", (sc, ya, za), (sc, yb, zb), 0.022)
    g.tube("nickel", (sc, 1.20, cz - R), (sc, 1.13, cz - R), 0.022)
    # Backsplash, microwave over the range, upper cabinets, pantry.
    g.span("backsplash", c1, pa, 0.91, 1.42, Z1 - 0.01, Z1)
    microwave(g, rng, dr, 1.48, Z1 - 0.41)
    # Clock (IMG_3094): black rim, white face, two hands; it stands on top of the upper cabinets, leaning back 10
    # degrees with its top edge against the wall (user). u is the face's up, n its outward normal.
    cx, tilt = x + 3.35, math.radians(10)
    u, n = (0.0, math.cos(tilt), math.sin(tilt)), (0.0, math.sin(tilt), -math.cos(tilt))

    def on_face(c, a=0.0, b=0.0, d=0.0):  # c along u, a along x, d along n, from the rim's back centre
        return (cx + a, 2.42 + 0.2 * u[1] + c * u[1] + d * n[1], Z1 - 0.2 * u[2] + c * u[2] + d * n[2] + b)

    g.tube("pc_black", on_face(0.0), on_face(0.0, d=0.05), 0.2)
    g.tube("clock_face", on_face(0.0, d=0.05), on_face(0.0, d=0.055), 0.18)
    g.strut("pc_black", on_face(-0.015, d=0.058), on_face(0.115, d=0.058), 0.012, 0.006)
    g.strut("pc_black", on_face(0.0, a=-0.005, d=0.058), on_face(0.0, a=0.085, d=0.058), 0.012, 0.006)
    upper_run(g, f1, c1, 1.88, 2.42, 4)
    upper_run(g, c1, rng, 1.42, 2.42, 1)
    upper_run(g, rng, dr, 1.9, 2.42, 2)
    upper_run(g, dr, sk, 1.42, 2.42, 2)
    upper_run(g, sk, pa, 1.42, 2.42, 2)
    g.span("cabinet_white", pa, end, 0.0, 2.42, zr, Z1)
    for y0, y1 in ((0.1, 1.5), (1.52, 2.42)):
        for a in (pa, (pa + end) / 2):
            shaker_door(g, a, a + (end - pa) / 2, y0, y1, zr)
    pm = (pa + end) / 2
    g.span("nickel", pm - 0.05, pm - 0.038, 0.9, 1.3, zr - 0.06, zr - 0.03)
    g.span("nickel", pm + 0.038, pm + 0.05, 0.9, 1.3, zr - 0.06, zr - 0.03)
    # Gooseneck barn lamps above the run.
    for lx in (x + 0.7, x + 3.1, x + 5.5):
        g.tube("lamp_grey", (lx, 2.95, Z1), (lx, 3.05, Z1 - 0.35), 0.012)
        g.tube("lamp_grey", (lx, 3.05, Z1 - 0.35), (lx, 2.92, Z1 - 0.5), 0.012)
        g.tube("lamp_grey", (lx, 2.84, Z1 - 0.5), (lx, 2.92, Z1 - 0.5), 0.17)


# ---------------------------------------------------------------- desk bay
def office_chair(g, pos, turn=0.0):
    """The kit office chair, plus a second base bar across its own one: the kit's two mirrors leave a single straight
    bar (both spokes on one line), so this one at right angles makes the base an X (user)."""
    g.kit("seating/office_chair_v2", pos, turn=turn)
    g.box("chair_metal", (0.06, 0.04, 0.6), (pos[0], 0.02, pos[2]), yaw(turn))


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
    office_chair(g, (dx - 0.55, 0.0, dz), turn=math.pi / 2)
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
    office_chair(g, (xd + 0.7, 0.0, ez), turn=-math.pi / 2)
    for z in (w0 + dl / 2 - 0.3225, w0 + dl / 2 + 0.3225):
        g.kit("appliances/monitor_v2", (xd + 0.1, 0.74, z), turn=-math.pi / 2, material=SILVER)
    g.kit("appliances/monitor_v2", (xd - 0.1, 0.74, ez), turn=math.pi / 2)
    for z in (w0 + 0.6, w0 + 2.0):                          # full-tower gaming PCs at the desks' front edge
        g.span("pc_black", xd + dd / 2 - 0.6, xd + dd / 2, 0.0, 0.6, z - 0.125, z + 0.125)
    crane(g, *C["crane"])
    # A big carton dropped north of the crane, 0.5 m in front of the north wall's column there, a bit skewed (user).
    col = min(north_columns(), key=lambda c: abs(c - C["crane"][1]))
    g.box("cardboard", (0.6, 0.7, 0.8), (X0 + COLUMN_D + 0.5 + 0.33, 0.35, col + 0.06), yaw(0.21))
    g.box("alu_case", (0.8, 0.3, 0.4), (xd, 0.15, w0 - 0.22))
    g.box("pc_black", (0.5, 0.75, 0.25), (xd, 0.375, w0 - 0.565))
    # Posters flush on the north wall: the two brown ones side by side 2 m from the west wall, two more east with
    # more space between them (IMG_3096).
    for z in (Z0 + 2.3, Z0 + 2.92, w1 - 1.7, w1 - 0.6):
        g.box("poster", (0.03, 0.9, 0.6), (X0 + 0.015, 0.45, z))
    # A long box on the east-bay floor, at the hall's centre: it swapped places with the trash can, which now stands in
    # the middle of the test mat (user).
    g.box("cardboard", (1.2, 0.2, 0.32), (0.0, 0.1, 0.0))
    # West wall, south to north: a desk north of the gate's west end, its user facing into the hall, boxes on its gate
    # side (IMG_3059, user), the grey cases north of the pilaster with one open on the floor, more boxes (the robot
    # dogs are Unitree Go2 packs in the scene).
    # The desk group: 5.7 m north of the gate wall, 1.2 m east of the gate's west side (user).
    gx, gz = X1 - 5.7, P["gate"]["z0"] + 0.2   # 1 m further west than before (user)
    standing_desk(g, gx, gz + 0.85, along_z=False, length=1.2, depth=0.55)
    office_chair(g, (gx, 0.0, gz + 0.24))
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
