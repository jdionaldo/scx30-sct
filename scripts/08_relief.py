"""Carve travel relief (rear bump + front lock/bump) into chassis rails, sway bridge, cage, spare carrier and shell.
usage: BODY_DIR=<per-body stl dir> python scripts/08_relief.py   (run from repo root)"""
import os, sys, math, runpy, numpy as np, trimesh
BD = os.environ["BODY_DIR"]
sys.argv = ["07", BD]
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()): ns = runpy.run_path("scripts/07_front_v2.py")
sys.path.insert(0, "scripts"); import rearlib as R
OUT = "stl/v2/"; CLR = 0.5
shell = trimesh.load("stl/tacoma_trophy_shell.stl")
R.M["Tacoma_Trophy_Body_Shell"] = shell
for n in list(R.M):
    if os.path.exists(OUT + n + ".stl"): R.M[n] = trimesh.load(OUT + n + ".stl")   # re-runs see cut parts
TARGETS = ["Chassis_Rail_L", "Chassis_Rail_R", "Rear_Sway_Bridge", "Roll_Cage", "Spare_Tire_Carrier", "Tacoma_Trophy_Body_Shell", "Slider_L", "Slider_R"]
BUMP = float(os.environ.get("BUMP_TA", 7))
BRIDGE_LIFT = 10.0   # sway bridge raised so the axle housing clears it through +7 deg bump
b0 = trimesh.load(BD + "/Rear_Sway_Bridge.stl"); b0.apply_translation([0, 0, BRIDGE_LIFT]); R.M["Rear_Sway_Bridge"] = b0; b0.export(OUT + "Rear_Sway_Bridge.stl")
def dil(m):
    ms = [m]
    for d in ([CLR,0,0],[-CLR,0,0],[0,CLR,0],[0,-CLR,0],[0,0,CLR],[0,0,-CLR]):
        c = m.copy(); c.apply_translation(d); ms.append(c)
    return trimesh.boolean.union(ms, engine="manifold")
def hits(sc): return R.hits(sc)
rest = None
def cut(name, movers):
    base = R.M[name]
    # movers: list of (mesh, T) posed meshes
    sw = []
    for m, T in movers:
        c = m.copy(); c.apply_transform(T); sw.append(dil(c))
    swept = trimesh.boolean.union(sw, engine="manifold")
    out = trimesh.boolean.difference([base, swept], engine="manifold")
    parts = [p for p in out.split(only_watertight=False) if abs(p.volume) > 20]
    return trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
# ---- rear: which movers touch which targets during bump 0..BUMP
rest = R.hits(R.scene(0))
touch = {}
for ta in np.arange(0, BUMP + .01, 1.0):
    sc = R.scene(float(ta))
    for p in R.hits(sc):
        if p in rest or any(n.startswith("HW_") for n in p): continue
        for a, b in (p, p[::-1]):
            if a in TARGETS and b not in TARGETS: touch.setdefault(a, set()).add(b)
print("rear contacts to relieve:", {k: sorted(v) for k, v in touch.items()})
newm = {}
for t, movs in touch.items():
    mv = []
    for ta in np.arange(0, BUMP + .01, 0.5):
        sc = R.scene(float(ta))
        mv += [sc[n] for n in movs if n in sc]
    newm[t] = cut(t, mv)
# ---- front: wheel envelope vs shell/chassis over lock and travel
fw = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, -1.4, 0.0, 1.4, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: continue
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            if k in sc: fw.append(sc[k])
for t in ("Tacoma_Trophy_Body_Shell",):
    base = newm.get(t, R.M[t]); sw = []
    for m, T in fw:
        c = m.copy(); c.apply_transform(T); sw.append(dil(c))
    newm[t] = trimesh.boolean.difference([base, trimesh.boolean.union(sw, engine="manifold")], engine="manifold")
for t, m in newm.items():
    if m is None: continue
    keep = R.M[t]
    print(t, "vol %.0f -> %.0f  watertight %s shells %d" % (keep.volume, m.volume, m.is_watertight, len(m.split(only_watertight=False))))
    m.export(OUT + t + ".stl")
