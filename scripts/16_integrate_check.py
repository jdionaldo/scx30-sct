"""Rest-pose interference over the full v3 assembly (v3 > v2 > .sc exports). BODY_DIR=<dir> python scripts/16_integrate_check.py"""
import os, sys, trimesh
from trimesh.collision import CollisionManager
BD = os.environ["BODY_DIR"]; M = {}
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".")); from make_viewer_3mf import DROP   # the ONE active-set list (scripts/make_viewer_3mf.py)
for d in ((os.environ.get("V3_DIR", "stl/v3/").rstrip("/"), "stl/v4") if os.environ.get("USE_V4") else (BD, "stl/v2", os.environ.get("V3_DIR", "stl/v3/").rstrip("/"))):
    for f in os.listdir(d):
        if f.endswith(".stl"): M[f[:-4]] = os.path.join(d, f)
M.pop("Tacoma_Trophy_Body_Shell_old", None)
cm = CollisionManager(); names = []
for n, p in M.items():
    if n in DROP or n.startswith("Ref_Wheel_Spare"): continue
    cm.add_object(n, trimesh.load(p)); names.append(n)
h, pr = cm.in_collision_internal(return_names=True)
pairs = sorted({tuple(sorted(x)) for x in pr})
print(len(names), "bodies;", len(pairs), "overlapping pairs")
for a, b in pairs:
    tag = "HW" if (a.startswith("HW_") or b.startswith("HW_")) else "  "
    print(tag, a, "|", b)
