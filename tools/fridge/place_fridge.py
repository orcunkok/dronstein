"""Set up the imported fridge parts in the open scene (Edit mode): the body a static MuJoCo body (its mesh collider
welded, so drones can't fly through it), the doors on hinges and the freezer, drawer and tray on slides, from
joints.json (convert_fridge.py). The moving parts don't collide with anything (their convex hulls would overlap the
body's and be pushed out); the force tool moves them along their joints.

    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 place_fridge.py <joints.json> [x z]
"""

from __future__ import annotations

import json
import re
import sys

from pod_authoring import McpSession

ENTITY = {"Body": "Fridge", "F_Door": "Fridge_F_Door", "RL_Door": "Fridge_RL_Door", "RR_Door": "Fridge_RR_Door",
          "LowerDrawer": "Fridge_LowerDrawer", "FreezerDecorTray": "Fridge_FreezerTray"}


def main():
    joints = json.loads(open(sys.argv[1]).read())
    ox, oz = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.0, 0.0)
    with McpSession("http://127.0.0.1:19331/mcp", timeout=120) as s:
        def call(tool, **args):
            raw = s._rpc("tools/call", {"name": tool, "arguments": args})
            text = raw["content"][0]["text"]
            assert not raw.get("isError") and "ERROR" not in text, f"{tool} {args}: {text}"
            return text

        def find(name):
            return re.search(rf"- {re.escape(name)} \(UUID: (\d+)\)", call("search_entities", namePattern=name)).group(1)

        def setp(uuid, comp, prop, value):
            call("set_entity_property", entityID=uuid, componentName=comp, propertyName=prop, value=value)

        for part, name in ENTITY.items():
            uuid = find(name)
            props = call("get_entity_properties", entityID=uuid)
            j = joints.get(part)
            pos = j["origin"] if j else (0.0, 0.0, 0.0)
            setp(uuid, "Transform", "Translation", f"{pos[0] + ox:g} {pos[1]:g} {pos[2] + oz:g}")
            setp(uuid, "Transform", "Rotation", "0 0 0")
            if j is None:                                  # the body: static (collider only, welded to the world)
                if "MujocoJointComponent" in props:
                    call("remove_component", entityID=uuid, componentType="MujocoJoint")
                continue
            setp(uuid, "MujocoMeshCollider", "Contype", "0")
            setp(uuid, "MujocoMeshCollider", "Conaffinity", "0")
            setp(uuid, "MujocoMeshCollider", "Density", "100")
            lo, hi = j["limit"]
            for prop, value in (("Type", "Hinge" if j["type"] == "hinge" else "Slide"), ("Position", "0 0 0"),
                                ("Axis", " ".join(f"{a:g}" for a in j["axis"])), ("Limited", "true"),
                                ("LimitMin", f"{lo:g}"), ("LimitMax", f"{hi:g}"),
                                ("Damping", "2" if j["type"] == "hinge" else "5")):
                setp(uuid, "MujocoJoint", prop, value)
            print(part, j["type"], "ok")


if __name__ == "__main__":
    main()
