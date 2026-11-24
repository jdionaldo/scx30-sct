"""List (and store the two reference meshes for) the material the owner's SOMA session (2026-11-21/22) added to and removed from the unlevelled builder shell.
Usage: LEVEL_WALL=0 OWNER_FIXES=0 OUT_STL=base.stl python scripts/19_shell_v4.py ; python scripts/shell_soma_patch.py base.stl soma_shell.stl
(soma_shell.stl = stl/v4/Tacoma_Trophy_Body_Shell.stl inside output/model.soma). Pieces under MIN_VOL are float noise of the STL round trip and are dropped."""
import sys, os, trimesh
MIN_VOL = 0.02
base, soma = trimesh.load(sys.argv[1]), trimesh.load(sys.argv[2])
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "shell_soma"); os.makedirs(out, exist_ok=True)
for name, a, b in (("add", soma, base), ("cut", base, soma)):
    d = trimesh.boolean.difference([a, b], engine="manifold")
    cs = [c for c in d.split(only_watertight=False) if abs(c.volume) >= MIN_VOL]
    print(name, "pieces", len(cs), "volume %.1f" % sum(abs(c.volume) for c in cs), "(dropped %.2f mm3 of noise)" % (abs(d.volume) - sum(abs(c.volume) for c in cs)))
    for c in sorted(cs, key=lambda c: -abs(c.volume)):
        if c.bounds[1][1] > 0: print("   %7.2f mm3  X %6.1f..%6.1f  Y %5.1f..%5.1f  Z %5.1f..%5.1f" % (abs(c.volume), *c.bounds.T.flatten().round(1)))
import shutil
shutil.copy(sys.argv[1], os.path.join(out, "base.stl")); shutil.copy(sys.argv[2], os.path.join(out, "soma.stl"))
