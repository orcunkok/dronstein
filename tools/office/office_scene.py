"""Add or update the office's lights and colliders in the open scene (after import_office.py; safe to run again).

Run with the editor open (Edit mode, scene Main):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 office_scene.py

Creates at the root of the scene, or moves if they already exist (found by name):
- OfficeLight_*: point lights under the LED high-bays (the roof blocks the sun); OfficeFill_*: one fill per bay.
- OfficeCollider_*: static MujocoBoxCollider entities (size = Transform Scale) for the walls, the ceiling under the
  trusses, the curtain, the restroom box, the mezzanine, the spiral and U stairs, the kitchen run, the sofa and the desks,
  so the drone can't fly through them.
Resizes Ground (the floor collider) to the office floor and removes the old outdoor floor visuals (GroundSurface,
GroundSlab). Positions are scene coordinates: the graphs' plan mirrored in x (north wall at x = +15).
"""

from __future__ import annotations

import json
import re

from build_office import BOX, BOX_DECK, CURTAIN_TOP, CURTAIN_X, CURTAIN_Z, DECK, H, KITCHEN_X, MEZZ, P, PARTITION_X, \
    X0, X1, Z0, Z1, ZC
from pod_authoring import McpSession

ENDPOINT = "http://127.0.0.1:19331/mcp"
GROUND = "7004567898520840787"
OLD_FLOOR = ("GroundSurface", "GroundSlab")
LIGHT_X = (-9.5, -1.5, 6.5)                                 # plan x of the LED high-bays (build_shell)
LIGHT_Z = (ZC - 5.5, ZC + 4.5)
FILL = ((-5.0, 1.5), (-6.0, -8.5), (-12.0, -9.0))           # plan (x, z): one fill per bay, one in the desk corner


def span(x0, x1, y0, y1, z0, z1):
    """Plan extents -> (scene centre, size); the plan is mirrored in x."""
    return (-(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (abs(x1 - x0), y1 - y0, z1 - z0)


T = 0.3
TOP = 9.5                   # colliders reach above the ridge
COLLIDERS = {
    "WallNorth": span(X0 - T, X0, 0, TOP, Z0 - T, Z1 + T),
    "WallSouth": span(X1, X1 + T, 0, TOP, Z0 - T, Z1 + T),
    "WallWest": span(X0 - T, X1 + T, 0, TOP, Z0 - T, Z0),
    "WallEast": span(X0 - T, X1 + T, 0, TOP, Z1, Z1 + T),
    "Ceiling": span(X0, X1, H - 0.3, H, Z0, Z1),
    "Curtain": span(CURTAIN_X[0], CURTAIN_X[1], 0, CURTAIN_TOP, CURTAIN_Z - 0.08, CURTAIN_Z + 0.08),
    "Block": span(BOX[0], BOX[1], 0, BOX_DECK + 1.05, BOX[2], BOX[3]),
    "UpperBox": span(*P["box"]["tall_box"][:2], BOX_DECK, P["box"]["tall_box"][4], *P["box"]["tall_box"][2:4]),
    "MidBox": span(*P["box"]["mid_box"][:2], BOX_DECK, P["box"]["mid_box"][4], *P["box"]["mid_box"][2:4]),
    "Mezzanine": span(MEZZ[0], MEZZ[1], DECK - P["mezzanine"]["thickness"], DECK + 1.07, MEZZ[2], MEZZ[3]),
    "MezzanineEast": span(BOX[1], X1, DECK - P["mezzanine"]["thickness"], DECK + 1.07, MEZZ[3], Z1),
    "Partition": span(PARTITION_X, PARTITION_X + 0.15, 0, DECK - P["mezzanine"]["thickness"], MEZZ[2], Z1),
    "RoomWall": span(PARTITION_X, X1, 0, DECK - P["mezzanine"]["thickness"], MEZZ[2], MEZZ[2] + 0.12),
    "Spiral": span(P["spiral"]["x"] - 0.8, P["spiral"]["x"] + 0.8, 0, BOX_DECK + 1.0,
                   P["spiral"]["z"] - 0.8, P["spiral"]["z"] + 0.8),
    "SpiralUpper": span(P["spiral_upper"]["x"] - 0.65, P["spiral_upper"]["x"] + 0.65, BOX_DECK, DECK + 1.0,
                        P["spiral_upper"]["z"] - 0.65, P["spiral_upper"]["z"] + 0.65),
    "Kitchen": span(KITCHEN_X, KITCHEN_X + 6.04, 0, 2.42, Z1 - 0.95, Z1),
    "SofaYellow": span(P["desk_corner"]["sofa_x"] - 0.475, P["desk_corner"]["sofa_x"] + 0.475, 0, 0.65,
                       P["desk_corner"]["sofa_z"] - 1.05, P["desk_corner"]["sofa_z"] + 1.05),
    "Bench": span(X0 + P["desk_corner"]["bench_wall_gap"], X0 + P["desk_corner"]["bench_wall_gap"] + 0.75, 0, 1.25,
                  *P["desk_corner"]["bench_z"]),
    "Crane": span(P["desk_corner"]["crane"][0] - 0.25, P["desk_corner"]["crane"][0] + 0.35, 0, 2.35,
                  P["desk_corner"]["crane"][1] - 0.25, P["desk_corner"]["crane"][1] + 0.45),
    "Desks": span(X0 + P["desk_corner"]["row_wall_gap"], X0 + P["desk_corner"]["row_wall_gap"] + 0.75, 0, 0.76, *P["desk_corner"]["row_z"]),
    "Stair": span(P["u_stair"]["platform_x"][0], P["u_stair"]["landing_x"][1], 0, DECK + 1.05,
                  P["u_stair"]["landing_z"][0], P["u_stair"]["upper_z"][1]),
}


def main():
    with McpSession(ENDPOINT, timeout=120) as s:
        def call(tool, **args):
            raw = s._rpc("tools/call", {"name": tool, "arguments": args})
            text = raw["content"][0]["text"]
            assert not raw.get("isError") and "ERROR" not in text, f"{tool} {args}: {text}"
            return text

        def find(name):
            m = re.search(rf"- {re.escape(name)} \(UUID: (\d+)\)", call("search_entities", namePattern=name))
            return m.group(1) if m else None

        def entity(name, component):
            """The entity called name, created with the component if it doesn't exist yet."""
            uuid = find(name)
            if uuid is None:
                uuid = re.search(r"\b(\d{10,})\b", call("create_entity", name=name)).group(1)
                call("add_component", entityID=uuid, componentType=component)
            return uuid

        def place(uuid, pos, scale=None):
            call("set_entity_property", entityID=uuid, componentName="Transform", propertyName="Translation",
                 value=" ".join(f"{v:g}" for v in pos))
            if scale:
                call("set_entity_property", entityID=uuid, componentName="Transform", propertyName="Scale",
                     value=" ".join(f"{v:g}" for v in scale))

        for i, x in enumerate(LIGHT_X):
            for j, z in enumerate(LIGHT_Z):
                light = entity(f"OfficeLight_{i}{j}", "PointLight")
                place(light, (-x, 5.5, z))
                for prop, value in (("Intensity", "60"), ("Radius", "25"), ("Radiance", "1 0.96 0.9")):
                    call("set_entity_property", entityID=light, componentName="PointLight", propertyName=prop,
                         value=value)
        for k, (x, z) in enumerate(FILL):
            light = entity(f"OfficeFill_{k}", "PointLight")
            place(light, (-x, 3.0, z))
            for prop, value in (("Intensity", "15"), ("Radius", "15"), ("Radiance", "1 0.96 0.9")):
                call("set_entity_property", entityID=light, componentName="PointLight", propertyName=prop,
                     value=value)
        for name, (pos, size) in COLLIDERS.items():
            box = entity(f"OfficeCollider_{name}", "MujocoBoxCollider")
            place(box, pos, size)
        pos, size = span(X0 - T, X1 + T, -1.0, 0.0, Z0 - T, Z1 + T)
        place(GROUND, pos, size)
        for name in OLD_FLOOR:
            if uuid := find(name):
                call("destroy_entity", entityID=uuid)
        print(json.dumps({"lights": len(LIGHT_X) * len(LIGHT_Z), "colliders": len(COLLIDERS)}))


if __name__ == "__main__":
    main()
