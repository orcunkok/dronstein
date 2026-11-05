"""Convert a Wavefront OBJ (positions, normals, triangles) to a minimal GLB, axes unchanged.

    python3 obj_to_glb.py <in.obj> <out.glb>

Used to turn the Go2 pack's meshes (LuckyEngine/Assets/ContentVault/Robots/UnitreeGo2Pod/assets) into files Lucky's
mesh import takes, for the static robot dogs (their parts' transforms already turn MuJoCo z-up into Lucky y-up).
"""

import json
import struct
import sys


def read_obj(path):
    vs, ns, verts, normals, index, seen = [], [], [], [], [], {}
    for line in open(path, encoding="utf-8", errors="replace"):
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            x, y, z = map(float, p[1:4])
            vs.append((x, y, z))
        elif p[0] == "vn":
            x, y, z = map(float, p[1:4])
            ns.append((x, y, z))
        elif p[0] == "f":
            corners = []
            for c in p[1:]:
                a = c.split("/")
                key = (int(a[0]), int(a[2]) if len(a) > 2 and a[2] else 0)
                if key not in seen:
                    seen[key] = len(verts)
                    verts.append(vs[key[0] - 1])
                    normals.append(ns[key[1] - 1] if key[1] else (0.0, 1.0, 0.0))
                corners.append(seen[key])
            for k in range(1, len(corners) - 1):
                index += [corners[0], corners[k], corners[k + 1]]
    return verts, normals, index


def write_glb(path, verts, normals, index):
    pos = b"".join(struct.pack("<3f", *v) for v in verts)
    nor = b"".join(struct.pack("<3f", *n) for n in normals)
    idx = b"".join(struct.pack("<I", i) for i in index)
    blob = pos + nor + idx
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    gltf = {
        "asset": {"version": "2.0"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2}]}],
        "buffers": [{"byteLength": len(blob)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(pos), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pos), "byteLength": len(nor), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pos) + len(nor), "byteLength": len(idx), "target": 34963}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(verts), "type": "VEC3", "min": lo, "max": hi},
                      {"bufferView": 1, "componentType": 5126, "count": len(normals), "type": "VEC3"},
                      {"bufferView": 2, "componentType": 5125, "count": len(index), "type": "SCALAR"}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    blob += b"\0" * (-len(blob) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(js) + 8 + len(blob)))
        f.write(struct.pack("<I4s", len(js), b"JSON") + js)
        f.write(struct.pack("<I4s", len(blob), b"BIN\0") + blob)


if __name__ == "__main__":
    write_glb(sys.argv[2], *read_obj(sys.argv[1]))
