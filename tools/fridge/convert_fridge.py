"""Convert the NVIDIA SimReady LG fridge (USD) into one GLB per moving part for Lucky, plus joints.json.

    <venv with usd-core, trimesh, pillow>/bin/python convert_fridge.py <fridge.usd> <out dir>

USD is Z-up with the front toward -Y; Lucky is Y-up with the front toward +Z: (x, y, z) -> (x, z, -y). Each part's GLB
has its origin on its joint's pivot (the hinge axis for the doors), so the part's entity sits there and turns about its
own origin. Materials come from the UsdPreviewSurface networks (the MDL ones are Omniverse only); see-through glass
(opacity < 0.5) is left out, since the engine draws no transparency and it would be an opaque sheet.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = "/World/lf24z6330s"
PARTS = ("Body", "F_Door", "RL_Door", "RR_Door", "UpperLeftDrawer", "UpperRightDrawer", "LowerDrawer",
         "FreezerDecorTray")


def to_lucky(p):
    p = np.asarray(p, dtype=np.float64)
    return np.stack([p[..., 0], p[..., 2], -p[..., 1]], axis=-1)


def preview(material):
    """The UsdPreviewSurface's inputs (constants and texture files) of a material."""
    out = {}
    for prim in Usd.PrimRange(material.GetPrim()):
        sh = UsdShade.Shader(prim)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        for inp in sh.GetInputs():
            name = inp.GetBaseName()
            if inp.HasConnectedSource():
                src = UsdShade.Shader(inp.GetConnectedSource()[0].GetPrim())
                f = src.GetInput("file")
                if f and f.Get():
                    out[name + "_tex"] = f.Get().resolvedPath
            elif inp.Get() is not None:
                out[name] = inp.Get()
    return out


_tex_cache: dict[str, Image.Image] = {}


def gltf_material(name, pv):
    def img(key):
        path = pv.get(key)
        if not path:
            return None
        if path not in _tex_cache:
            _tex_cache[path] = Image.open(path).convert("RGBA" if key == "diffuseColor_tex" else "RGB")
        return _tex_cache[path]
    c = pv.get("diffuseColor", (0.8, 0.8, 0.8))
    return trimesh.visual.material.PBRMaterial(
        name=name, baseColorFactor=[float(c[0]), float(c[1]), float(c[2]), 1.0],
        metallicFactor=float(pv.get("metallic", 0.0)), roughnessFactor=float(pv.get("roughness", 0.5)),
        baseColorTexture=img("diffuseColor_tex"), normalTexture=img("normal_tex"), doubleSided=True)


def corners(mesh, attr_vals, interp, counts, indices):
    """Per face-corner values of a primvar/attribute (faceVarying, vertex/varying, uniform or constant)."""
    vals = np.asarray(attr_vals)
    if interp == "faceVarying":
        return vals
    if interp in ("vertex", "varying"):
        return vals[indices]
    if interp == "uniform":
        return np.repeat(vals, counts, axis=0)
    return np.repeat(vals[:1], len(indices), axis=0)


def mesh_parts(stage, part, origin, cache):
    """trimesh pieces of a part's visual meshes, in Lucky axes, relative to origin; grouped later by material."""
    pieces = []
    start = stage.GetPrimAtPath(f"{ROOT}/{part}/visuals")
    for prim in Usd.PrimRange(start, Usd.TraverseInstanceProxies()):
        if prim.GetTypeName() != "Mesh" or UsdGeom.Imageable(prim).ComputeVisibility() == "invisible":
            continue
        m = UsdGeom.Mesh(prim)
        counts = np.asarray(m.GetFaceVertexCountsAttr().Get())
        idx = np.asarray(m.GetFaceVertexIndicesAttr().Get())
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
        if len(counts) == 0:
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        pv = preview(mat) if mat else {}
        if float(pv.get("opacity", 1.0)) < 0.5:
            continue
        xf = np.array(cache.GetLocalToWorldTransform(prim), dtype=np.float64)   # row-vector convention
        wp = np.c_[pts, np.ones(len(pts))] @ xf
        cp = to_lucky(wp[:, :3])[idx] - origin                                 # per corner
        nrm = None
        na = m.GetNormalsAttr()
        npv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("normals")
        if npv and npv.HasValue():
            nrm = corners(m, npv.ComputeFlattened(), npv.GetInterpolation(), counts, idx)
        elif na.HasValue():
            nrm = corners(m, na.Get(), m.GetNormalsInterpolation(), counts, idx)
        if nrm is not None and len(nrm) == len(idx):
            nrm = np.asarray(nrm) @ np.linalg.inv(xf[:3, :3]).T
            nrm = to_lucky(nrm)
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
        else:
            nrm = None
        uv = None
        st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        if st and st.HasValue():
            uv = corners(m, st.ComputeFlattened(), st.GetInterpolation(), counts, idx)
            if len(uv) != len(idx):
                uv = None
        # Triangulate as fans over the corners.
        starts = np.r_[0, np.cumsum(counts)[:-1]]
        tri = [np.stack([s + np.zeros(c - 2, int), s + np.arange(1, c - 1), s + np.arange(2, c)], 1)
               for s, c in zip(starts, counts) if c >= 3]
        faces = np.concatenate(tri)
        if np.linalg.det(xf[:3, :3]) < 0:
            faces = faces[:, ::-1]
        key = mat.GetPrim().GetName() if mat else "default"
        pieces.append((key, pv, cp, nrm, uv, faces))
    return pieces


def build(stage, part, origin, cache, out):
    groups: dict[str, list] = {}
    for key, pv, cp, nrm, uv, faces in mesh_parts(stage, part, origin, cache):
        groups.setdefault(key, [pv, []])[1].append((cp, nrm, uv, faces))
    scene = trimesh.Scene()
    tris = 0
    for key, (pv, items) in groups.items():
        off, V, N, U, F = 0, [], [], [], []
        has_n = all(n is not None for _, n, _, _ in items)
        has_uv = all(u is not None for _, _, u, _ in items)
        for cp, nrm, uv, faces in items:
            V.append(cp); F.append(faces + off); off += len(cp)
            if has_n: N.append(nrm)
            if has_uv: U.append(uv)
        V, F = np.concatenate(V), np.concatenate(F)
        tris += len(F)
        mat = gltf_material(key, pv)
        visual = trimesh.visual.TextureVisuals(uv=np.concatenate(U) if has_uv else None, material=mat)
        tm = trimesh.Trimesh(vertices=V, faces=F, vertex_normals=np.concatenate(N) if has_n else None,
                             visual=visual, process=False)
        if has_uv:                      # glTF's v runs down the image, USD's st up
            tm.visual.uv = np.c_[tm.visual.uv[:, 0], 1.0 - tm.visual.uv[:, 1]]
        scene.add_geometry(tm, node_name=key, geom_name=key)
    if tris:
        scene.export(out / f"fridge_{part}.glb")
    return tris, len(groups)


def main():
    stage = Usd.Stage.Open(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    cache = UsdGeom.XformCache()
    body = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{ROOT}/Body")), dtype=np.float64)
    joints = {}
    for prim in stage.GetPrimAtPath(f"{ROOT}/joints").GetChildren():
        j = UsdPhysics.Joint(prim)
        part = j.GetBody1Rel().GetTargets()[0].name
        lp0 = np.array([*j.GetLocalPos0Attr().Get(), 1.0]) @ body
        kind = "hinge" if prim.GetTypeName() == "PhysicsRevoluteJoint" else "slide"
        axis = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[prim.GetAttribute("physics:axis").Get()]
        lo, hi = prim.GetAttribute("physics:lowerLimit").Get(), prim.GetAttribute("physics:upperLimit").Get()
        if kind == "hinge":
            lo, hi = np.radians(lo), np.radians(hi)
        axis = np.array(axis, dtype=np.float64) @ body[:3, :3]        # the joint frame is the (turned) body's
        joints[part] = {"type": kind, "pivot": to_lucky(lp0[:3]).tolist(),
                        "axis": np.round(to_lucky(axis), 6).tolist(),
                        "limit": [float(lo), float(hi)]}
    report = {}
    for part in PARTS:
        origin = np.array(joints[part]["pivot"]) if part in joints and joints[part]["type"] == "hinge" else np.zeros(3)
        if part in joints:
            joints[part]["origin"] = origin.tolist()
        report[part] = build(stage, part, origin, cache, out)
        print(part, "tris %d materials %d" % report[part], flush=True)
    (out / "joints.json").write_text(json.dumps(joints, indent=1))


if __name__ == "__main__":
    main()
