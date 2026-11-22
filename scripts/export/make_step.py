"""Export the full assembly to STEP (one named solid per body, faceted from the STLs).
Usage: python scripts/export/make_step.py [out.step]   (needs cadquery + trimesh)
Meshes carry no analytic surfaces, so every triangle becomes a planar face: fine for
fit checks/import, not for parametric editing. Run from repo root."""
import sys, os, glob, time, trimesh, cadquery as cq
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace,
                                BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid)
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.ShapeFix import ShapeFix_Solid

MAXF = 30000  # sewing is very slow above this; merge near-coplanar triangles with manifold simplify

def _simplify(m):
    import numpy as np, manifold3d as M
    mf = M.Manifold(M.Mesh(vert_properties=np.array(m.vertices, dtype=np.float32),
                           tri_verts=np.array(m.faces, dtype=np.uint32)))
    for tol in (0.001, 0.02, 0.05, 0.1):  # mm; 0.001 also removes sliver faces that make OCC solids invalid
        r = mf.simplify(tol)
        if r.num_tri() <= MAXF: break
    o = r.to_mesh()
    return trimesh.Trimesh(np.array(o.vert_properties)[:, :3], np.array(o.tri_verts))

def _build(m):
    m = _simplify(m)
    sew = BRepBuilderAPI_Sewing(1e-4)
    V = m.vertices
    for f in m.faces:
        p = BRepBuilderAPI_MakePolygon(*[gp_Pnt(*map(float, V[i])) for i in f], True)
        if p.IsDone():
            fc = BRepBuilderAPI_MakeFace(p.Wire(), True)
            if fc.IsDone(): sew.Add(fc.Face())
    sew.Perform()
    ex = TopExp_Explorer(sew.SewedShape(), TopAbs_SHELL)
    if not ex.More(): return cq.Shape.cast(sew.SewedShape())
    ms = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(ex.Current()))
    fx = ShapeFix_Solid(ms.Solid()); fx.Perform()
    sol = cq.Shape.cast(fx.Solid())
    if not sol.isValid():
        from OCP.ShapeFix import ShapeFix_Shape
        f = ShapeFix_Shape(sol.wrapped); f.SetPrecision(1e-3); f.Perform(); sol = cq.Shape.cast(f.Shape())
    return sol

def _snap(m, g=0.005):
    import numpy as np
    mm = trimesh.Trimesh(np.round(m.vertices / g) * g, m.faces.copy(), process=False); mm.merge_vertices()
    f = mm.faces; mm.update_faces((f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2]))
    mm.remove_unreferenced_vertices(); return mm

def mesh_to_shape(m):
    sh = _build(m)
    if len(sh.Solids()) != 1 or not sh.isValid():  # zero-area faces: snap vertices to a 5 um grid and retry
        sh = _build(_snap(m))
    return sh

out = sys.argv[1] if len(sys.argv) > 1 else "scx30_sct_assembly.step"
files = {}
for d in ("stl/v3", "stl/v4"):
    for f in sorted(glob.glob(d + "/*.stl")): files[os.path.basename(f)[:-4]] = f
EXCLUDE = {"IFS_Bulkhead_Convertible"}  # fused into Chassis_Tub (intermediate input of scripts/18_tub.py)
files = {k: v for k, v in files.items() if k not in EXCLUDE}
asm = cq.Assembly(name="scx30_sct")
for i, (n, f) in enumerate(sorted(files.items())):
    t = time.time(); m = trimesh.load(f)
    sh = mesh_to_shape(m); asm.add(sh, name=n)
    if len(sh.Solids()) != 1 or not sh.isValid(): print('  WARNING invalid/not solid:', n, flush=True)
    print(f"{i+1}/{len(files)} {n} {len(m.faces)}f {time.time()-t:.1f}s", flush=True)
asm.export(out, "STEP")
print("wrote", out, os.path.getsize(out) // 1024, "KB")
