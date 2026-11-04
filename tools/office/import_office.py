"""Import the office graphs built by build_office.py into the open project and place them in the scene.

Run with the editor open (Edit mode, scene Main):
    PYTHONPATH=<LuckyEngine>/Data/Authoring/python python3 import_office.py <build dir> <tag> [old entity ids...]

Each graph is packaged and imported to Assets/Office/<Graph><tag>/source/<graph>.hpodgraph (a fresh tag each time:
a loaded graph is cached, so re-importing over it would not change the scene), then one procedural-mesh entity per
graph is created at the origin under a new `Office` container. Old entities named on the command line (a previous
Office container) are destroyed afterwards. The scene is saved.
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path

from pod_authoring import McpSession, PackageClient

ENDPOINT = "http://127.0.0.1:19331/mcp"
NAMES = ("office_shell", "office_block", "office_kitchen", "office_desks")
ORIGIN = {key: {"kind": "Float3", "value": value} for key, value in
          (("translation", [0, 0, 0]), ("rotation_euler", [0, 0, 0]), ("scale", [1, 1, 1]))}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    build, tag, old = Path(sys.argv[1]).resolve(), sys.argv[2], sys.argv[3:]
    with McpSession(ENDPOINT, timeout=300, max_response_bytes=128 * 1024 * 1024) as session:
        state = session.authoring_state()
        assert state["edit_ready"], "Lucky is not in Edit mode"
        context = state["context_id"]
        office = Path(state["project_root"]) / "Assets" / "Office"
        office.mkdir(exist_ok=True)

        def scene(request):
            return session.scene_authoring(context, {"schema_version": 1, **request})

        sources = {}
        for name in NAMES:
            entry = f"{name}.hpodgraph"
            package = PackageClient(session).publish(build, entry, build / f"package-{name}-{tag}")
            sources[name] = {"package_root": str(package.package_root), "expected_generation": package.generation,
                             "entry": entry, "expected_source_sha256": digest(build / entry), "asset_entry": entry,
                             "destination": f"Office/{''.join(p.title() for p in name.split('_'))}{tag}"}
            print("packaged", name, flush=True)
        operation = f"office-{tag}"
        result = session.authoring_import(context, operation_id=operation, workspace=str(build / f"import-{tag}"),
                                          sources=sources)
        deadline = time.monotonic() + 600
        while result["phase"] not in ("ready", "failed", "cancelled"):
            assert time.monotonic() < deadline, "import still running; inspect it, do not repeat it"
            time.sleep(1.0)
            result = session.authoring_import_status(context, operation)
        assert result["phase"] == "ready", json.dumps(result)[:2000]
        imported = result["sources"]
        graphs = scene({"action": "inspect_graphs", "graphs": [row["graph"] for row in imported.values()]})
        stamps = {str(row["handle"]): row for row in graphs["graphs"]}
        entities = [{"key": "office", "name": "Office", "role": {"type": "container"}}]
        for name in NAMES:
            entities.append({"key": name, "name": "".join(p.title() for p in name.split("_")),
                             "parent": {"key": "office"}, "transform": ORIGIN,
                             "role": {"type": "pod", "source": stamps[str(imported[name]["graph"])],
                                      "parameters": {}}})
        created = scene({"action": "create_entities", "expected": graphs["stamp"], "parent_preconditions": [],
                         "entities": entities})["created"]
        print("created", json.dumps(created), flush=True)
        for entity in old:
            raw = session._rpc("tools/call", {"name": "destroy_entity", "arguments": {"entityID": entity}})
            print("destroyed", entity, raw.get("isError"), flush=True)
        ids = list(created.values())
        deadline = time.monotonic() + 600
        while True:
            observed = scene({"action": "inspect", "entities": ids})
            arguments = {"context_id": context, "expected": observed["stamp"], "preconditions": [
                {"entity_id": row["entity_id"], "fingerprint": row["fingerprint"]} for row in observed["entities"]]}
            if session.authoring_cooks(**arguments)["ready"]:
                break
            assert time.monotonic() < deadline, "still cooking; nothing saved"
            time.sleep(1.0)
        print("saved", session.authoring_save(**arguments)["scene_saved"])


if __name__ == "__main__":
    main()
