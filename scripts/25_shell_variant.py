"""Kit shell = the SCT Cyber Trophy variant (sct_shell_variant/trophy_shell.py, owner's choice 2026-11-24).
Called by scripts/19_shell_v4.py (SHELL_SOURCE=variant, the default). Writes three printed parts:
  OUT_STL (default stl/v4/Tacoma_Trophy_Body_Shell.stl)   the body. The kit file name is kept, because 17 scripts
                                                           (plates, parts list, checks, sweeps) refer to the shell by it.
  stl/v4/SCT_Hex_Mesh_Grille.stl, stl/v4/SCT_Cyan_DRL_Bar.stl   the snap-in grille and LED brow bar.
The kit clearance changes (cabin opening, roof, nose, rocker, relief cuts) live in trophy_shell.py as named constants.
"""
import os, sys, time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
import cadquery as cq  # noqa: E402
from sct_shell_variant.trophy_shell import build_cyber_trophy_truck  # noqa: E402

V4 = os.path.join(ROOT, "stl", "v4")
OUT = {"SCT_Cyber_Trophy_Body": os.environ.get("OUT_STL", os.path.join(V4, "Tacoma_Trophy_Body_Shell.stl")),
       "SCT_Hex_Mesh_Grille": os.path.join(V4, "SCT_Hex_Mesh_Grille.stl"),
       "SCT_Cyan_DRL_Bar": os.path.join(V4, "SCT_Cyan_DRL_Bar.stl")}

t = time.time()
parts = build_cyber_trophy_truck()
for name, sol in parts.items():
    if not sol.isValid() or len(sol.Solids()) != 1:
        raise RuntimeError(f"{name}: not one valid solid")
    cq.exporters.export(sol, OUT[name], tolerance=0.03, angularTolerance=0.1)      # same mesh settings as sct_shell_variant/build.py
    print(f"  {name:22s} vol {sol.Volume():8.1f} mm3 -> {os.path.relpath(OUT[name], ROOT)}")
print(f"variant shell built in {time.time() - t:.1f} s")
