"""Build the Lucky Robots office (535 Minnesota St, San Francisco) as Lucky procedural graphs.

Run with the editor open (Edit mode):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 build_office.py <new output dir>

Writes into <out> four graphs (and the kit assets they use, under <out>/assets):
- office_shell.hpodgraph: floor, tilt-up walls, gable roof, trusses, purlins, skylights, clerestory windows, the
  black stage curtain on its track, roll-up and exit doors, conduit, panels, heaters, fans, LED high-bays.
- office_block.hpodgraph: the restroom box (lower deck) and the mezzanine (upper deck), the alley between them, the
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
ZC = (Z0 + Z1) / 2            # ridge line, along x
WALL_T = P["hall"]["wall_thickness"]
ROOF_T = P["hall"]["roof_thickness"]
TRUSS_X = P["hall"]["truss_x"]
CURTAIN_Z = P["curtain"]["z"]
CURTAIN_X = (P["curtain"]["x0"], P["curtain"]["x1"])
CURTAIN_TOP = P["curtain"]["top"]

# Restroom box and mezzanine at the south end, two tiers: the box's roof is the lower deck, the mezzanine west of it
# (and over the alley south of it) the upper deck.
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
    "curtain_black": mat(0.05, 0.045, 0.042, 0.0, 0.8),
    "curtain_white": mat(0.78, 0.78, 0.76, 0.0, 0.9),
    "steel_dark": mat(0.05, 0.05, 0.055, 0.6, 0.45),
    "steel_grey": mat(0.45, 0.46, 0.47, 0.8, 0.4),
    "conduit": mat(0.55, 0.56, 0.57, 0.9, 0.35),
    "conduit_red": mat(0.55, 0.03, 0.02, 0.3, 0.5),
    "panel_grey": mat(0.42, 0.43, 0.43, 0.3, 0.5),
    "glass_dark": mat(0.03, 0.04, 0.05, 0.0, 0.1),
    "shade_cream": mat(0.75, 0.73, 0.66, 0.0, 0.9),
    "frame_beige": mat(0.55, 0.50, 0.40, 0.2, 0.6),
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
    "desk_white": mat(0.85, 0.85, 0.84, 0.0, 0.4),
    "desk_frame": mat(0.55, 0.56, 0.57, 0.8, 0.35),
    "pc_black": mat(0.02, 0.02, 0.02, 0.2, 0.4),
    "cardboard": mat(0.45, 0.30, 0.15, 0.0, 0.9),
    "case_grey": mat(0.30, 0.30, 0.31, 0.0, 0.6),
    "stair_grey": mat(0.14, 0.145, 0.15, 0.0, 0.7),
    "glass_teal": mat(0.08, 0.25, 0.30, 0.0, 0.1),
    "glass_sky": mat(0.45, 0.50, 0.55, 0.0, 0.1),
    "curtain_beige": mat(0.60, 0.50, 0.35, 0.0, 0.9),
    "exit_green": mat(0.10, 0.70, 0.20, 0.0, 0.4),
    "sign_blue": mat(0.02, 0.06, 0.35, 0.0, 0.4),
    "alu_case": mat(0.70, 0.70, 0.72, 0.9, 0.35),
    "poster": mat(0.12, 0.10, 0.09, 0.0, 0.5),
    "drinks": mat(0.30, 0.45, 0.60, 0.0, 0.6),
    "stripe_grey": mat(0.12, 0.12, 0.12, 0.0, 0.9),
}
SOFA_YELLOW = mat(0.55, 0.38, 0.03, 0.0, 0.8)    # mustard tufted velvet
SOFA_BLUE = mat(0.22, 0.38, 0.52, 0.0, 0.9)
BLACK_STAINLESS = mat(0.06, 0.06, 0.065, 0.9, 0.35)
RUG_CREAM = mat(0.60, 0.56, 0.48, 0.0, 0.95)
SILVER = mat(0.75, 0.75, 0.77, 0.9, 0.3)     # studio display backs


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
    return RIDGE - (RIDGE - H) * abs(z - ZC) / (ZC - Z0)


def slope_frame(side):
    """side -1: west slope (rises toward +z), +1: east slope. Returns rotation, unit normal (up/out), angle."""
    a = math.atan2(RIDGE - H, ZC - Z0)
    r = roll(-a) if side < 0 else roll(a)
    return r, [r[i][1] for i in range(3)], a


def on_roof(g, key, size, x, z, below=0.0):
    """A box lying under the roof slope at (x, z), its top `below` metres under the roof underside."""
    side = -1 if z < ZC else 1
    r, n, _ = slope_frame(side)
    y = roof_y(z)
    off = below + size[1] / 2
    g.box(key, size, (x - n[0] * off, y - n[1] * off, z - n[2] * off), r)


def build_shell(g):
    length, width = X1 - X0, Z1 - Z0
    xm, zm = (X0 + X1) / 2, (Z0 + Z1) / 2
    # Floor slab, test mat and tape.
    g.box("epoxy_floor", (length + 2 * WALL_T, 0.2, width + 2 * WALL_T), (xm, -0.1, zm))
    g.span("test_mat", X0 + 0.5, X0 + 8.5, 0.0, 0.003, Z1 - 8.5, Z1 - 0.5)
    g.span("tape_yellow", X0, CURTAIN_X[1] + 1.0, 0.0, 0.002, CURTAIN_Z + 2.45, CURTAIN_Z + 2.5)
    g.span("tape_yellow", X0 + 6.5, X0 + 6.55, 0.0, 0.002, CURTAIN_Z + 2.5, Z1)

    # Walls: the end walls up to the eave, the gable above them in strips that end inside the roof slab.
    for x in (X0 - WALL_T / 2, X1 + WALL_T / 2):
        g.box("concrete_white", (WALL_T, H, width + 2 * WALL_T), (x, H / 2, zm))
        steps = 12
        for i in range(steps):
            y0 = H + (RIDGE - H) * i / steps
            half = (ZC - Z0) * (1 - i / steps) + WALL_T
            g.box("concrete_white", (WALL_T, (RIDGE - H) / steps + 0.001, 2 * half),
                  (x, y0 + (RIDGE - H) / steps / 2, ZC))
    for z in (Z0 - WALL_T / 2, Z1 + WALL_T / 2):
        g.box("concrete_white", (length + 2 * WALL_T, H + ROOF_T, WALL_T), (xm, (H + ROOF_T) / 2, z))

    # Roof slabs (white timber deck).
    for side in (-1, 1):
        r, n, a = slope_frame(side)
        run = ZC - Z0 + WALL_T + 0.2
        slab = run / math.cos(a)
        zmid = ZC + side * run / 2
        ymid = RIDGE - (RIDGE - H) * (run / 2) / (ZC - Z0)
        g.box("timber_white", (length + 2 * WALL_T, ROOF_T, slab),
              (xm + 0.0, ymid + n[1] * ROOF_T / 2, zmid + n[2] * ROOF_T / 2), r)
        # Purlins along x, every ~2 m up the slope.
        for k in range(1, 5):
            z = ZC + side * (ZC - Z0) * k / 5
            on_roof(g, "timber_white", (length, 0.22, 0.1), xm, z)

    # Trusses (white steel), pilasters under their ends, eave beams along the long walls.
    bottom = H - 0.3
    for x in TRUSS_X:
        g.strut("steel_white", (x, bottom, Z0), (x, bottom, Z1), 0.2, 0.25)
        for side, ze in ((-1, Z0), (1, Z1)):
            top_e, top_r = (x, H - 0.05, ze), (x, RIDGE - 0.12, ZC)
            g.strut("steel_white", top_e, top_r, 0.2, 0.25)
            for k in range(1, 4):
                f = k / 4
                z = ze + (ZC - ze) * f
                yt = top_e[1] + (top_r[1] - top_e[1]) * f
                g.strut("steel_white", (x, bottom, z), (x, yt, z), 0.12)
                zn = ze + (ZC - ze) * (f + 0.25)
                ytn = top_e[1] + (top_r[1] - top_e[1]) * (f + 0.25)
                g.strut("steel_white", (x, bottom, z), (x, ytn, zn), 0.1)
            g.box("steel_white", (0.3, H, 0.3), (x, H / 2, ze - side * 0.15))
        g.strut("steel_white", (x, bottom, ZC), (x, RIDGE - 0.12, ZC), 0.12)
    for z in (Z0 + 0.15, Z1 - 0.15):
        g.box("steel_white", (length, 0.4, 0.3), (xm, H - 0.2, z))

    # Skylights: a dark louvred well in the roof between trusses.
    for x, z in ((-7.5, ZC - 4.0), (0.0, ZC - 4.0), (-7.5, ZC + 4.0), (7.5, ZC + 4.0)):
        on_roof(g, "glass_dark", (2.4, 0.02, 1.5), x, z, below=-0.01)
        for dx in (-1.25, 1.25):
            on_roof(g, "paint_white", (0.1, 0.3, 1.6), x + dx, z)
        for dz in (-0.8, 0.8):
            on_roof(g, "paint_white", (2.6, 0.3, 0.1), x, z + dz)
        for k in range(-3, 4):
            on_roof(g, "paint_white", (0.05, 0.22, 1.5), x + k * 0.33, z)

    # North wall clerestories: steel-sash grid windows over the west bay, roller-shade windows over the east bay.
    wx = X0 + 0.03
    for zc in (-10.5, -8.5, -6.5):
        g.box("glass_sky", (0.02, 1.3, 1.9), (wx, 5.65, zc))
        for k in range(5):
            g.box("steel_white", (0.05, 1.32, 0.04), (wx + 0.01, 5.65, zc - 0.95 + k * 0.475))
        for k in range(4):
            g.box("steel_white", (0.05, 0.04, 1.92), (wx + 0.01, 5.0 + k * 0.4333, zc))
    for zc in (-2.6, -0.95, 1.6, 3.25, 4.9):
        g.box("shade_cream", (0.03, 1.15, 1.55), (wx, 4.9, zc))
        for dz in (-0.79, 0.79):
            g.box("frame_beige", (0.05, 1.2, 0.04), (wx + 0.01, 4.9, zc + dz))
        for dy in (-0.6, 0.6):
            g.box("frame_beige", (0.05, 0.04, 1.62), (wx + 0.01, 4.9 + dy, zc))
    g.box("frame_beige", (0.12, 0.12, 4.0), (wx + 0.04, 5.6, -1.78))
    g.box("frame_beige", (0.12, 0.12, 5.0), (wx + 0.04, 5.6, 3.25))

    # Black stage curtain: a pleated sheet on a ceiling track, white binding on its free end.
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
    g.place(thick, "curtain_black", (1, 1, 1), ((CURTAIN_X[0] + CURTAIN_X[1]) / 2, 0.05 + ch / 2, CURTAIN_Z),
            roll(math.pi / 2))
    # Its desk-bay (west) face is white: IMG_3077/3078.
    g.place(thick, "curtain_white", (1, 1, 1), ((CURTAIN_X[0] + CURTAIN_X[1]) / 2, 0.05 + ch / 2, CURTAIN_Z - 0.14),
            roll(math.pi / 2))
    g.span("paint_white", CURTAIN_X[1] - 0.05, CURTAIN_X[1] + 0.05, 0.05, CURTAIN_TOP, CURTAIN_Z - 0.08,
           CURTAIN_Z + 0.08)
    g.span("paint_white", CURTAIN_X[0] - 0.02, CURTAIN_X[0] + 0.03, 0.05, CURTAIN_TOP, CURTAIN_Z - 0.08,
           CURTAIN_Z + 0.08)
    g.tube("conduit", (CURTAIN_X[0] - 0.2, CURTAIN_TOP + 0.05, CURTAIN_Z), (CURTAIN_X[1] + 0.2, CURTAIN_TOP + 0.05,
           CURTAIN_Z), 0.03)
    for x in TRUSS_X[:2]:
        g.tube("conduit", (x, CURTAIN_TOP + 0.05, CURTAIN_Z), (x, H - 0.3, CURTAIN_Z), 0.01)

    # South wall, east to west behind the U stair: shaded steel-sash window over a strip of small panes, the fire
    # panel, the steel exit door with its EXIT sign, a white column, the closed roll-up gate with its hood and three
    # shades above it. Sizes from office_params.toml.
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
    w = (rz1 - rz0) / 3
    y0, y1 = G["shades_y"]
    for k in range(3):
        z0 = rz0 + k * w
        g.span("shade_cream", sx - 0.03, sx, y0, y1, z0 + 0.03, z0 + w - 0.03)
        g.span("frame_beige", sx - 0.05, sx, y0, y1, z0, z0 + 0.04)
    g.span("frame_beige", sx - 0.05, sx, y0 - 0.04, y0 + 0.02, rz0, rz1)
    pz, pw = S["pilaster_z"], S["pilaster_width"]
    g.span("concrete_white", X1 - 0.3, X1, 0.0, H, pz - pw / 2, pz + pw / 2)
    dz0, dz1 = D["z0"], D["z1"]
    g.span("panel_grey", sx - 0.04, sx, 0.0, D["height"], dz0, dz1)
    g.span("nickel", sx - 0.1, sx - 0.04, 1.0, 1.04, dz1 - 0.2, dz1 - 0.05)
    g.span("exit_green", sx - 0.08, sx, D["height"] + 0.17, D["height"] + 0.37, (dz0 + dz1) / 2 - 0.15,
           (dz0 + dz1) / 2 + 0.15)
    g.span("red_cross", sx - 0.12, sx, 1.3, 1.9, dz1 + 0.05, dz1 + 0.3)
    (wz0, wz1), (wy0, wy1), (sy0, sy1), (qz0, qz1) = S["window_z"], S["window_y"], S["strip_y"], S["strip_z"]
    g.span("shade_cream", sx - 0.03, sx, wy0, wy1, wz0, wz1)
    for k in range(7):
        z = wz0 + k * (wz1 - wz0) / 6
        g.span("frame_beige", sx - 0.05, sx, wy0, wy1, z - 0.025, z + 0.025)
    for k in range(4):
        y = wy0 + k * (wy1 - wy0) / 3
        g.span("frame_beige", sx - 0.05, sx, y - 0.02, y + 0.02, wz0, wz1)
    g.span("glass_sky", sx - 0.02, sx, sy0, sy1, qz0, qz1)
    for k in range(8):
        z = qz0 + k * (qz1 - qz0) / 7
        g.span("frame_beige", sx - 0.04, sx, sy0, sy1, z - 0.02, z + 0.02)

    # Two beige stage curtains rolled up and hung from the roof, over the mezzanine and over the stair.
    for x, z, bottom in ((10.0, -1.0, 5.6), (14.0, -4.5, 4.8)):
        g.tube("curtain_beige", (x, bottom, z), (x, roof_y(z) - 0.2, z), 0.18)

    # Conduit runs, electrical panels, extinguisher, bollards.
    g.tube("conduit", (X0 + 0.05, 1.25, Z0), (X0 + 0.05, 1.25, Z1), 0.012)
    g.tube("conduit", (X0, 1.15, Z0 + 0.05), (X1, 1.15, Z0 + 0.05), 0.012)
    g.tube("conduit", (X0, 6.1, Z0 + 0.06), (X1, 6.1, Z0 + 0.06), 0.015)
    g.tube("conduit", (X0, 1.2, Z1 - 0.05), (KITCHEN_X - 0.3, 1.2, Z1 - 0.05), 0.012)
    g.tube("conduit_red", (X0 + 6.0, 2.85, Z1 - 0.06), (BOX[0], 2.85, Z1 - 0.06), 0.01)
    g.span("panel_grey", X0, X0 + 0.15, 1.1, 1.85, CURTAIN_Z + 0.4, CURTAIN_Z + 0.9)
    g.span("panel_grey", 4.2, 4.8, 1.0, 1.9, Z0, Z0 + 0.15)
    g.tube("red_cross", (X0 + 0.12, 0.9, CURTAIN_Z + 1.2), (X0 + 0.12, 1.4, CURTAIN_Z + 1.2), 0.07)
    for x in (X1 - 0.4, X1 - 0.9):     # between the gate and the west corner
        z = (P["gate"]["z0"] + Z0) / 2
        g.tube("yellow_safety", (x, 0.0, z), (x, 1.1, z), 0.07)

    # Unit heaters (hung from the roof), ceiling fans, LED high-bays.
    for x, z in ((-11.0, Z1 - 2.5), (-3.0, Z1 - 2.5)):
        g.span("heater_brown", x - 0.3, x + 0.3, 5.4, 6.0, z - 0.25, z + 0.25)
        g.tube("conduit", (x, 6.0, z), (x, roof_y(z), z), 0.02)
    for x, z in ((-7.5, ZC + 3.0), (0.0, ZC - 5.0)):
        y = 7.4
        g.tube("steel_white", (x, y, z), (x, roof_y(z), z), 0.02)
        g.tube("steel_white", (x, y - 0.1, z), (x, y + 0.05, z), 0.1)
        for k in range(3):
            a = 2 * math.pi * k / 3
            g.box("steel_white", (0.7, 0.012, 0.14), (x + 0.45 * math.cos(a), y - 0.05, z - 0.45 * math.sin(a)),
                  yaw(a))
    for x in (-9.5, -1.5, 6.5):
        for z in (ZC - 5.5, ZC + 4.5):
            g.span("led_white", x - 0.3, x + 0.3, 6.35, 6.42, z - 0.12, z + 0.12)
            g.tube("conduit", (x, 6.42, z), (x, H - 0.3, z), 0.006)


# ---------------------------------------------------------------- office block and mezzanine
def balusters(g, key, p0, p1, base, height, step, w=0.022):
    n = max(1, int(math.dist(p0, p1) / step))
    for i in range(n + 1):
        t = i / n
        x, z = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
        g.box(key, (w, height, w), (x, base + height / 2, z))


def cable_rail(g, p0, p1):
    n = max(1, round(math.dist(p0, p1) / 1.5))
    for i in range(n + 1):
        t = i / n
        x, z = p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
        g.box("steel_grey", (0.05, 1.07, 0.05), (x, DECK + 0.535, z))
    g.tube("steel_grey", (p0[0], DECK + 1.07, p0[1]), (p1[0], DECK + 1.07, p1[1]), 0.022)
    for k in range(9):
        y = DECK + 0.12 + k * 0.1
        g.tube("conduit", (p0[0], y, p0[1]), (p1[0], y, p1[1]), 0.003)


def slat_rail(g, p0, p1):
    """A grey slat railing over the nosing line p0-p1 (x, y, z): a post at each end and five boards parallel to it."""
    for p in (p0, p1):
        g.box("stair_grey", (0.06, 1.05, 0.06), (p[0], p[1] + 0.525, p[2]))
    for h in (0.25, 0.45, 0.65, 0.85, 1.05):
        g.strut("stair_grey", (p0[0], p0[1] + h, p0[2]), (p1[0], p1[1] + h, p1[2]), 0.04, 0.12)


def spiral(g, cx, cz, r, y0, y1, treads, end):
    """A spiral stair round a post from y0 to y1: wood treads 24 degrees apart, a baluster and a handrail segment at
    each tread; the top tread points along yaw `end`."""
    rise = (y1 - y0) / treads
    g.tube("steel_dark", (cx, y0, cz), (cx, y1 + 1.0, cz), 0.05)
    prev = None
    for i in range(treads):
        a = end + math.radians(24.0 * (i - treads + 1))
        y = y0 + rise * (i + 1)
        d = (math.cos(a), -math.sin(a))
        g.box("wood_tread", (r - 0.06, 0.04, 0.3), (cx + d[0] * (r / 2), y - 0.02, cz + d[1] * (r / 2)), yaw(a))
        foot = (cx + d[0] * (r - 0.03), y, cz + d[1] * (r - 0.03))
        top = (foot[0], y + 0.9, foot[2])
        g.tube("steel_dark", foot, top, 0.009)
        if prev:
            g.tube("steel_grey", prev, top, 0.02)
        prev = top


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
    for x, z, top in ((px0 + 0.05, za + 0.05, DECK), (xl + 0.05, zc + 0.05, land), (xl + 0.05, za - 0.05, land)):
        g.box("stair_grey", (0.08, top, 0.08), (x, top / 2, z))
    slat_rail(g, (px1, DECK, za), (xl, land, za))
    slat_rail(g, (xl, land, zc), (xl - go * (n - 1), rise, zc))
    slat_rail(g, (xl, land, zd), (xl - go * (n - 1), rise, zd))
    slat_rail(g, (px0, DECK, za), (px0, DECK, zb))
    slat_rail(g, (px0, DECK, za), (px1, DECK, za))
    slat_rail(g, (xl, land, lz0), (xe, land, lz0))


def build_block(g):
    bx0, bx1, bz0, bz1 = BOX
    mx0, mx1, mz0, mz1 = MEZZ
    B, M, Q, S, T = P["box"], P["mezzanine"], P["partition"], P["spiral"], P["spiral_upper"]
    # The restroom box; its flat roof is the lower deck, with the tall box and the mid box standing on it.
    g.span("paint_white", bx0, bx1, 0.0, BOX_DECK - 0.15, bz0, bz1)
    g.span("deck_fascia", bx0 - 0.02, bx1, BOX_DECK - 0.15, BOX_DECK, bz0 - 0.02, bz1)
    for x0, x1, z0, z1, top in (B["tall_box"], B["mid_box"]):
        g.span("paint_white", x0, x1, BOX_DECK, top, z0, z1)
    # North face, facing the kitchen: the MEN door. West face, facing the hall: a door with the first-aid box beside it.
    zc = B["men_door_z"]
    g.span("paint_white", bx0 - 0.03, bx0, 0.0, 2.13, zc - 0.455, zc + 0.455)
    g.span("nickel", bx0 - 0.09, bx0 - 0.03, 1.0, 1.03, zc + 0.25, zc + 0.4)
    g.span("steel_white", bx0 - 0.02, bx0, 2.13, 2.18, zc - 0.5, zc + 0.5)
    g.span("sign_blue", bx0 - 0.04, bx0 - 0.03, 1.45, 1.7, zc - 0.12, zc + 0.12)
    xc = B["west_door_x"]
    g.span("paint_white", xc - 0.455, xc + 0.455, 0.0, 2.13, bz0 - 0.03, bz0)
    g.span("nickel", xc - 0.4, xc - 0.25, 1.0, 1.03, bz0 - 0.09, bz0 - 0.03)
    g.span("steel_white", xc - 0.5, xc + 0.5, 2.13, 2.18, bz0 - 0.02, bz0)
    g.span("paint_white", xc - 1.3, xc - 1.0, 1.45, 1.8, bz0 - 0.08, bz0)
    g.span("red_cross", xc - 1.17, xc - 1.13, 1.55, 1.7, bz0 - 0.09, bz0 - 0.08)
    g.span("red_cross", xc - 1.22, xc - 1.08, 1.61, 1.64, bz0 - 0.09, bz0 - 0.08)
    g.tube("conduit_red", (bx0 - 0.03, 2.6, bz0), (bx0 - 0.03, 2.6, bz1), 0.01)
    g.tube("conduit_red", (bx0, 2.6, bz0 - 0.03), (bx1, 2.6, bz0 - 0.03), 0.01)
    # Black baluster railing round the lower deck's open north and west edges, open where the big spiral arrives; south
    # of the mezzanine front the upper deck's railing takes over.
    north_end = B["mid_box"][2]
    balusters(g, "steel_dark", (bx0 + 0.03, bz0), (bx0 + 0.03, north_end), BOX_DECK, 1.0, 0.13)
    g.span("steel_dark", bx0 - 0.01, bx0 + 0.07, BOX_DECK + 1.0, BOX_DECK + 1.05, bz0, north_end)
    for x0, x1 in ((bx0, S["x"] - 0.45), (S["x"] + 0.45, mx0)):
        balusters(g, "steel_dark", (x0, bz0 + 0.03), (x1, bz0 + 0.03), BOX_DECK, 1.0, 0.13)
        g.span("steel_dark", x0, x1, BOX_DECK + 1.0, BOX_DECK + 1.05, bz0 - 0.01, bz0 + 0.07)

    # Upper deck: west of the box from its front to the south wall, and east of that over the alley and the south rooms
    # up to the east wall; a front beam on two white posts, cable railing along the edges open to the hall and the box
    # deck (gap where the small spiral arrives).
    t = M["thickness"]
    for x0, z0, z1 in ((mx0, mz0, mz1), (bx1, mz1, Z1)):
        g.span("deck_fascia", x0, mx1, DECK - 0.3, DECK, z0, z1)
        g.span("paint_white", x0 + 0.02, mx1, DECK - 0.35, DECK - 0.3, z0, z1)
    g.span("steel_white", mx0, mx0 + 0.25, DECK - t, DECK - 0.3, mz0, mz1)
    for z in M["posts_z"]:
        g.span("steel_white", mx0, mx0 + 0.2, 0.0, DECK - t, z - 0.1, z + 0.1)
    cable_rail(g, (mx0 + 0.05, mz0 + 0.05), (mx0 + 0.05, mz1))
    cable_rail(g, (mx0 + 0.05, mz1), (bx1, mz1))
    cable_rail(g, (bx1 + 0.05, T["z"] + T["radius"] + 0.1), (bx1 + 0.05, Z1))
    slat_rail(g, (P["u_stair"]["platform_x"][1], DECK, mz0), (mx1, DECK, mz0))

    # Under it: the front wall with the French doors folded open, transoms and the blue sofa, the room's west wall (the U
    # stair flush against it) at its west end; east of the box's west face line it goes on as the alley's south wall,
    # with a glazed side door near the alley's mouth. The alley ends at a dark doorway in the east wall.
    px, top = PARTITION_X, DECK - t
    (d0, d1), (sz0, sz1) = Q["doors_z"], Q["side_door_z"]
    g.span("paint_white", px, X1, 0.0, top, mz0, mz0 + 0.12)
    for z0, z1 in ((mz0, d0), (d1, sz0), (sz1, Z1)):
        g.span("paint_white", px, px + 0.15, 0.0, top, z0, z1)
    for z0, z1 in ((d0, d1), (sz0, sz1)):
        g.span("paint_white", px, px + 0.15, 2.21, top, z0, z1)
    g.span("paint_white", px + 2.0, px + 2.1, 0.0, top, d0 - 0.5, d1 + 0.5)
    leaf = 0.45
    for z0 in (d0, d1 - leaf):
        g.span("glass_light", px + 0.05, px + 0.06, 0.0, 2.13, z0, z0 + leaf)
        for z in (z0, z0 + leaf - 0.08):
            g.span("wood_door", px + 0.03, px + 0.08, 0.0, 2.13, z, z + 0.08)
        for i in range(7):
            g.span("wood_door", px + 0.04, px + 0.07, i * 0.345, i * 0.345 + 0.06, z0, z0 + leaf)
    g.span("wood_door", px - 0.05, px, 2.13, 2.21, d0 - 0.06, d1 + 0.06)
    (ty0, ty1) = Q["transom_y"]
    for zc, w in zip(Q["transoms_z"], Q["transom_w"]):
        g.span("glass_teal", px - 0.01, px, ty0, ty1, zc - w / 2, zc + w / 2)
        g.span("panel_grey", px - 0.03, px, ty0 - 0.03, ty0, zc - w / 2 - 0.03, zc + w / 2 + 0.03)
        g.span("panel_grey", px - 0.03, px, ty1, ty1 + 0.03, zc - w / 2 - 0.03, zc + w / 2 + 0.03)
    g.kit("seating/sofa_v2", (px - 0.5, 0.0, Q["sofa_z"]), turn=-math.pi / 2, material=SOFA_BLUE)
    for z in (Q["sofa_z"], (d0 + d1) / 2):
        g.kit("lighting/pendant_lamp_v2", (px - 0.8, top - 2.4, z))
    g.span("glass_light", px + 0.07, px + 0.08, 0.1, 2.05, sz0 + 0.05, sz1 - 0.05)
    for z0 in (sz0, sz1 - 0.06):
        g.span("wood_door", px - 0.02, px + 0.17, 0.0, 2.21, z0, z0 + 0.06)
    g.span("wood_door", px - 0.02, px + 0.17, 2.13, 2.21, sz0, sz1)
    g.span("backsplash", bx1 + 0.3, px - 0.3, 0.0, 2.13, Z1 - 0.01, Z1)
    u_stair(g)

    # Spiral stairs: floor -> lower deck just west of the box (top tread east onto the deck), lower deck -> upper deck
    # by the box deck's south-west corner (top tread south onto the upper deck).
    spiral(g, S["x"], S["z"], S["radius"], 0.0, BOX_DECK, S["treads"], -math.pi / 2)
    spiral(g, T["x"], T["z"], T["radius"], BOX_DECK, DECK, T["treads"], 0.0)


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
        g.kit("appliances/fridge_v2", (fx + 0.454, 0.0, Z1 - 0.45), turn=math.pi,
              width=0.908, height=1.775, depth=0.86, steel=BLACK_STAINLESS)
    base_run(g, c1, rng, 1, zf)
    g.kit("appliances/oven_v2", ((rng + dr) / 2, 0.132, Z1 - 0.34), turn=math.pi)
    base_run(g, dr, dw, 1, zf, drawers=True)
    g.kit("appliances/dishwasher_v2", ((dw + sk) / 2, 0.005, Z1 - 0.31), turn=math.pi,
          width=0.61, height=0.86, depth=0.6)
    g.span("counter_black", dw, sk, 0.876, 0.91, zf - 0.03, Z1)
    base_run(g, sk, pa, 2, zf)
    # Sink basin and faucet.
    sc = (sk + pa) / 2
    g.span("steel_dark", sc - 0.3, sc + 0.3, 0.911, 0.913, zf + 0.06, zf + 0.46)
    g.tube("nickel", (sc, 0.91, Z1 - 0.08), (sc, 1.25, Z1 - 0.08), 0.015)
    g.tube("nickel", (sc, 1.25, Z1 - 0.08), (sc, 1.18, Z1 - 0.32), 0.012)
    # Backsplash, microwave over the range, upper cabinets, pantry.
    g.span("backsplash", c1, pa, 0.91, 1.42, Z1 - 0.01, Z1)
    g.kit("appliances/microwave_v2", ((rng + dr) / 2, 1.48, Z1 - 0.21), turn=math.pi)
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
def standing_desk(g, x, z, along_z=True):
    """A white sit-stand desk, 1.5 x 0.75 x 0.74 m, its length along z (or x)."""
    lx, lz = (0.75, 1.5) if along_z else (1.5, 0.75)
    g.span("desk_white", x - lx / 2, x + lx / 2, 0.715, 0.74, z - lz / 2, z + lz / 2)
    for s in (-1, 1):
        cx, cz = (x, z + s * 0.6) if along_z else (x + s * 0.6, z)
        g.box("desk_frame", (0.07, 0.68, 0.07), (cx, 0.37, cz))
        g.box("desk_frame", (0.68, 0.04, 0.07) if along_z else (0.07, 0.04, 0.68), (cx, 0.02, cz))
    g.box("desk_frame", (0.05, 0.05, 1.2) if along_z else (1.2, 0.05, 0.05), (x, 0.68, z))


def beni_rug(g, x0, x1, z0, z1, y=0.0):
    """A cream Beni Ourain rug with thin dark stripes across its long side (z)."""
    g.kit("decor/rug_v2", ((x0 + x1) / 2, y, (z0 + z1) / 2), material=RUG_CREAM, width=x1 - x0, depth=z1 - z0)
    for k in range(6):
        z = z0 + 0.3 + k * (z1 - z0 - 0.6) / 5
        g.span("stripe_grey", x0 + 0.2, x1 - 0.2, y + 0.012, y + 0.014, z - 0.012, z + 0.012)


def build_desks(g):
    C = P["desk_corner"]
    # North-west corner (IMG_3074/3078): the corner desk near the west wall, its user on the north side facing the hall;
    # the yellow tufted sofa just south of it, long side east-west, facing south; two striped rugs east of the sofa.
    g.kit("seating/sofa_v2", (C["sofa_x"], 0.0, C["sofa_z"]), turn=math.pi / 2, material=SOFA_YELLOW)
    dx, dz = C["small_desk_x"], C["small_desk_z"]
    standing_desk(g, dx, dz)
    g.kit("appliances/monitor_v2", (dx + 0.2, 0.74, dz), turn=-math.pi / 2)
    g.kit("seating/office_chair_v2", (dx - 0.55, 0.0, dz), turn=math.pi / 2)
    beni_rug(g, *C["rug_x"], *C["rug_z"])
    (qx0, qx1), (qz0, qz1) = C["rug2_x"], C["rug2_z"]
    beni_rug(g, qx0, qx1, qz0, qz1, y=0.002)
    g.tube("pc_black", (qx0 + 0.3, 0.0, qz0 + 0.3), (qx0 + 0.3, 1.0, qz0 + 0.3), 0.02)
    g.tube("pc_black", (qx0 + 0.3, 0.0, qz0 + 0.3), (qx0 + 0.3, 0.03, qz0 + 0.3), 0.2)
    # West end of the row (IMG_3078): a grey metal workbench against the north wall, its user facing the wall, a speaker,
    # boxes and drink stacks by it.
    b0, b1 = C["bench_z"]
    bx = X0 + C["bench_wall_gap"]
    g.span("panel_grey", bx, bx + 0.75, 0.86, 0.9, b0, b1)
    g.span("panel_grey", bx, bx + 0.75, 0.2, 0.23, b0, b1)
    for x in (bx + 0.03, bx + 0.72):
        for z in (b0 + 0.03, b1 - 0.03):
            g.box("desk_frame", (0.04, 0.88, 0.04), (x, 0.44, z))
    g.box("pc_black", (0.25, 0.35, 0.3), (bx + 0.2, 1.075, b0 + 0.2))
    g.kit("appliances/monitor_v2", (bx + 0.3, 0.9, (b0 + b1) / 2), turn=math.pi / 2)
    g.kit("seating/office_chair_v2", (bx + 1.15, 0.0, (b0 + b1) / 2), turn=-math.pi / 2)
    for z, s in ((b1 + 0.3, (0.5, 0.45, 0.45)), (b1 + 0.75, (0.45, 0.35, 0.4))):
        g.box("cardboard", s, (X0 + 0.3, s[1] / 2, z))
    for z in (b0 - 0.9, b0 - 0.45):
        g.box("drinks", (0.35, 0.9, 0.4), (X0 + 0.25, 0.45, z))
    # Three white desks along the north wall: PCs, monitor arms, two silver studio displays facing east to the user at
    # the east end, a crane arm on a stand; bin and flight case in front at the west end.
    (w0, w1) = C["row_z"]
    xd = X0 + C["row_wall_gap"] + 0.375
    n = round((w1 - w0) / 1.4)
    for k in range(n):
        standing_desk(g, xd, w0 + 0.7 + k * (w1 - w0 - 1.4) / max(1, n - 1))
    g.kit("seating/office_chair_v2", (xd - 1.0, 0.0, w0 + 1.4), turn=math.pi / 2)
    g.kit("seating/office_chair_v2", (xd, 0.0, w1 + 0.45), turn=math.pi)
    for z in C["displays_z"]:
        g.kit("appliances/monitor_v2", (xd, 0.74, z), material=SILVER)
    for z in (w0 + 0.6, w0 + 2.2, w0 + 3.6):
        g.span("pc_black", xd + 0.1, xd + 0.35, 0.0, 0.45, z - 0.11, z + 0.11)
    for z in (w1 - 1.9, w1 - 1.5):
        g.tube("steel_white", (xd - 0.2, 0.74, z), (xd - 0.2, 1.1, z), 0.02)
        g.strut("steel_white", (xd - 0.2, 1.1, z), (xd + 0.15, 1.25, z + 0.1), 0.03)
    # The one crane (black pole, arm and yellow block) stands past the row's east end.
    cx, cz = C["crane"]
    g.tube("pc_black", (cx, 0.0, cz), (cx, 0.03, cz), 0.25)
    g.tube("pc_black", (cx, 0.0, cz), (cx, 1.6, cz), 0.03)
    g.strut("pc_black", (cx, 1.6, cz), (cx + 0.3, 2.3, cz + 0.4), 0.05)
    g.box("yellow_safety", (0.12, 0.12, 0.12), (cx, 1.6, cz))
    g.box("pc_black", (0.4, 0.75, 0.32), (C["bin"][0], 0.375, C["bin"][1]))
    g.box("alu_case", (0.45, 0.42, 0.7), (C["case"][0], 0.21, C["case"][1]))
    for z in (w0 + 1.2, w1 - 1.5):
        g.box("poster", (0.03, 0.9, 0.6), (X0 + 0.05, 0.45, z))
    # A long box on the east-bay floor.
    g.box("cardboard", (1.2, 0.2, 0.32), (X0 + 4.0, 0.1, 3.0))
    # West wall, south to north: a desk by the south-west corner with boxes near the bollards, the grey cases north
    # of the pilaster with one open on the floor, more boxes (the robot dogs are Unitree Go2 packs in the scene).
    standing_desk(g, 12.8, Z0 + 0.8, along_z=False)
    g.kit("seating/office_chair_v2", (12.9, 0.0, Z0 + 1.4), turn=math.pi)
    for x, z, s in ((14.0, -12.4, (0.5, 0.35, 0.45)), (13.6, -12.5, (0.45, 0.3, 0.4)),
                    (-7.3, Z0 + 0.4, (0.6, 0.45, 0.6)), (-7.9, Z0 + 0.4, (0.5, 0.5, 0.5)),
                    (-7.6, Z0 + 1.1, (0.5, 0.35, 0.45))):
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
