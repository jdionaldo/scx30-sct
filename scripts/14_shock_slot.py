"""Rear trailing arm with a real lower-shock clevis (slot + cheeks + screw bore). BODY_DIR=<per-body stl dir> python scripts/14_shock_slot.py
Eye width/hole are ASSUMPTIONS (SCX24 oil shock lower eye): change EYE_W / EYE_D after measuring the real shock."""
import os, math, numpy as np, trimesh
BD = os.environ["BODY_DIR"]; OUT = "stl/v3/"
SHIFT = float(os.environ.get('ARM_SHIFT', 2.0))   # arm moves outboard with the rails (rail outer face Y 18.5 in the rotated-gearbox layout)
PX, PZ = -44.0, 0.5         # lower shock screw axis (X,Z), along Y
SHOCK_UP = np.array([-26.0, 21.5])   # upper mount (X,Z)
EYE_W, EYE_D, CLR = 2.0, 1.6, 0.4    # eye width, screw clearance bore, side clearance
YA0, YA1 = 16.0 + SHIFT, 19.5 + SHIFT  # arm Y span after shift (L side)
Y_IN, Y_OUT = YA0, YA0 + 5.4               # boss: outboard only (inboard is the rail)
SLOT_Y0 = YA0 + 1.05; SLOT_Y1 = SLOT_Y0 + EYE_W + CLR
YC = 0.5 * (SLOT_Y0 + SLOT_Y1)          # shock centre line in Y
def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=trimesh.transformations.translation_matrix([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))
def cylY(x, z, r, y0, y1):
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=48); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])); c.apply_translation([x, (y0 + y1) / 2, z]); return c
def U(*m): return trimesh.boolean.union(list(m), engine="manifold")
def D(a, *m): return trimesh.boolean.difference([a, *m], engine="manifold")
arm = trimesh.load(os.path.join(BD, "Rear_Trailing_Arm_L.stl")); arm.apply_translation([0, SHIFT, 0])
zt = 2.2   # boss top flush with the arm top at the shock mount
boss = box(PX - 3.4, PX + 3.0, Y_IN, Y_OUT, arm.bounds[0][2] + 0.3, zt)       # clevis cheeks (outboard)
a2 = U(arm, boss)
# slot: round bottom about the screw + wedge along the shock axis (eye can swing +/- 25 deg)
ax = (SHOCK_UP - np.array([PX, PZ])); ax /= np.linalg.norm(ax); ang = math.atan2(ax[1], ax[0])
R = EYE_D / 2 + 1.0 + 0.2
def stadium(a):
    ex = np.array([math.cos(a), math.sin(a)]) * 6.0
    return trimesh.convex_hull.convex_hull(np.vstack([cylY(PX, PZ, R, SLOT_Y0, SLOT_Y1).vertices, cylY(PX + ex[0], PZ + ex[1], R, SLOT_Y0, SLOT_Y1).vertices])) if hasattr(trimesh, "convex_hull") else trimesh.Trimesh(np.vstack([cylY(PX, PZ, R, SLOT_Y0, SLOT_Y1).vertices, cylY(PX + ex[0], PZ + ex[1], R, SLOT_Y0, SLOT_Y1).vertices])).convex_hull
a3 = a2
for da in (0, 12, -12, 25, -25):
    a3 = D(a3, stadium(ang + math.radians(da)))
# screw bore: clear Ø1.6 through inner cheek, Ø1.25 self-tap pilot in outer cheek (no nut trap)
a3 = D(a3, cylY(PX, PZ, EYE_D / 2, Y_IN - 0.1, SLOT_Y0 + 0.01), cylY(PX, PZ, 0.62, SLOT_Y1 - 0.01, Y_OUT + 0.1))
# arm's original bore at this point is replaced by the new one
r = trimesh.load_mesh(os.path.join(BD, "Rear_Trailing_Arm_L.stl")) if False else None
for side in ("L", "R"):
    m = a3.copy()
    if side == "R": m.apply_transform(np.diag([1, -1, 1, 1.0]))
    f = OUT + "Rear_Trailing_Arm_%s.stl" % side; m.export(f); q = trimesh.load(f)
    e = q.edges_sorted; u, c = np.unique(e, axis=0, return_counts=True)
    print(side, "watertight", q.is_watertight, "open edges", int((c == 1).sum()), "shells", len(q.split(only_watertight=False)), "vol %.1f" % q.volume, "bounds", q.bounds.round(2).tolist())
print("shock centre line Y=%.2f (was 17.75) slot Y %.2f..%.2f, cheeks in %.2f out %.2f; eye axis (%.1f,%.1f)" % (YC, SLOT_Y0, SLOT_Y1, SLOT_Y0 - Y_IN, Y_OUT - SLOT_Y1, PX, PZ))
# shock reference + hardware shifted to the new centre line (Y +%.2f)
dy = YC - 17.75
for n in ("Ref_Shock_RL", "HW_Rear_Shock_Lower_Screw_L", "HW_Rear_Shock_Upper_Screw_L"):
    m = trimesh.load(os.path.join(BD, n + ".stl")); m.apply_translation([0, dy, 0]); m.export(OUT + n + ".stl")
    mm = m.copy(); mm.apply_transform(np.diag([1, -1, 1, 1.0]))
    mm.export(OUT + (n[:-2] + "_R" if n.endswith("_L") else n.replace("RL", "RR")) + ".stl")
print("shock ref/screws moved +%.2f in Y" % dy)

for side, sg in (("L", 1), ("R", -1)):                       # arm-to-axle hardware travels with the arms
    for k in ("Screw", "Nut"):
        m = trimesh.load(os.path.join(BD, "HW_Rear_Trailing_Axle_%s_%s.stl" % (k, side))); m.apply_translation([0, SHIFT * sg, 0]); m.export(OUT + "HW_Rear_Trailing_Axle_%s_%s.stl" % (k, side))
print("axle hardware shifted +%.1f outboard" % SHIFT)
