"""SCX30 SCT kit, shown in SOMA from the current meshes.

The kit is built by the scripts in scripts/ (see AGENTS.md for the rebuild order), which write STL files to
stl/v3 (front end) and stl/v4 (tub, cage, shell, rear). This file only reads those STL files and registers each
one as a part, so the SOMA viewer, `soma render` and `output/model.step` show the kit as it is on disk.
It never writes to stl/. Rebuild the kit with the scripts, then run `soma build` again to refresh the view.

Part list, colours and the parts left out (old or keep-out bodies) come from scripts/make_viewer_3mf.py, the same
source as the viewer 3MF. Units mm; X forward, Y left, Z up (kit frame, not SOMA's front = -Y convention).
"""
import os
import sys

import cadquery as cq
from OCP.StlAPI import StlAPI_Reader
from OCP.TopoDS import TopoDS_Shape
from soma import add_part, param

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "scripts"))
import make_viewer_3mf as mv  # noqa: E402  (DROP list, MATERIALS, mat_index)

MESH_DIRS = ("stl/v3", "stl/v4")   # later folders win on equal names
SHOW_SHELL = param(True, "Show body shell", group="View")
SHOW_REFS = param(True, "Show reference parts (Ref_*)", group="View")
SHOW_HW = param(True, "Show hardware (HW_*)", group="View")


def hex_rgb(h):
    return tuple(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5))


def load_stl(path):
    shape = TopoDS_Shape()
    if not StlAPI_Reader().Read(shape, path):
        raise ValueError(f"Could not read {path}")
    return cq.Shape.cast(shape)


src = {}
for d in MESH_DIRS:
    for f in sorted(os.listdir(os.path.join(HERE, d))):
        if f.endswith(".stl"):
            src[f[:-4]] = os.path.join(HERE, d, f)

names = sorted(n for n in src if n not in mv.DROP and not n.startswith("Ref_Wheel_Spare"))
for name in names:
    if "Shell" in name and not SHOW_SHELL:
        continue
    if name.startswith("Ref_") and not SHOW_REFS:
        continue
    if name.startswith("HW_") and not SHOW_HW:
        continue
    group, col = mv.MATERIALS[mv.mat_index(name)]
    role = "reference" if name.startswith("Ref_") else "hardware" if name.startswith("HW_") else "part"
    add_part(name, load_stl(src[name]), role=role, color=hex_rgb(col), group=group,
             alpha=1 if role == "reference" else None)
