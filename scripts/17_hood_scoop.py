"""Hood scoop for the N30 servo (+ bulkhead cradle) and shock pockets in the v3 shell. python scripts/17_hood_scoop.py"""
import numpy as np, trimesh
F = "stl/v3/Tacoma_Trophy_Body_Shell.stl"
SERVO = dict(x0=40.7, x1=50.6, y0=1.0, y1=16.8, z0=10.0, z1=30.73)   # from FRONT_V3.md
CLR, WALL, ZH = 0.5, 1.2, 14.0                                         # scoop starts below the hood (Z 14)
def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(extents=[x1-x0, y1-y0, z1-z0], transform=trimesh.transformations.translation_matrix([(x0+x1)/2, (y0+y1)/2, (z0+z1)/2]))
U = lambda *m: trimesh.boolean.union(list(m), engine="manifold")
D = lambda a, *m: trimesh.boolean.difference([a, *m], engine="manifold")
sh = trimesh.load(F); s = SERVO
# inside: servo case + cradle (bulkhead reaches X 36.9..~55, Y to 18.4, Z up to 19.9) + clearance
inner = box(36.4, 55.0 + CLR, -1.4, 18.9, ZH, s["z1"] + 1.0)
outer = box(36.4 - WALL, 55.0 + CLR + WALL, -1.4 - WALL, 18.9 + WALL, ZH + 1.0, s["z1"] + 1.0 + WALL)
scoop = D(outer, inner)
sh = U(sh, scoop)
sh = D(sh, box(36.4, 55.0 + CLR, -1.4, 18.9, ZH, s["z1"] + 1.0))      # open the hood under the scoop
# bulkhead/cradle below the hood line
sh = D(sh, box(36.9 - CLR, 55.0 + CLR, -1.4, 18.9, 8.0, ZH + 0.5))
# rear shock pockets follow the shocks (+1 mm Y): cut dilated shock refs
for n in ("Ref_Shock_RL", "Ref_Shock_RR"):
    m = trimesh.load("stl/v3/%s.stl" % n); parts = [m]
    for d in ([CLR,0,0],[-CLR,0,0],[0,CLR,0],[0,-CLR,0],[0,0,CLR],[0,0,-CLR]):
        c = m.copy(); c.apply_translation(d); parts.append(c)
    sh = D(sh, U(*parts))
parts = [p for p in sh.split(only_watertight=False) if abs(p.volume) > 20]; sh = parts[0]
sh.export(F); q = trimesh.load(F); e = q.edges_sorted; u, c = np.unique(e, axis=0, return_counts=True)
print("shell watertight", q.is_watertight, "open edges", int((c == 1).sum()), "shells", len(q.split(only_watertight=False)), "vol %.0f" % q.volume, "bounds", q.bounds.round(1).tolist())
