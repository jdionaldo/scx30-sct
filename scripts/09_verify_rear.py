"""Rear travel verification against the printed (relieved) parts. BODY_DIR=<per-body stl dir> python scripts/09_verify_rear.py"""
import os, sys, math, numpy as np, trimesh
sys.path.insert(0, "scripts"); import rearlib as R
for n in list(R.M):
    for d in ("stl/v2", "stl/v3"):
        p = "%s/%s.stl" % (d, n)
        if os.path.exists(p): R.M[n] = trimesh.load(p)
rest = R.hits(R.scene(0))
def real(p): return sorted(x for x in p if x not in rest and not any(n.startswith("HW_") for n in x))
for d, rng in (("bump", range(0, 12)), ("droop", range(0, -12, -1))):
    last = 0
    for ta in rng:
        p = real(R.hits(R.scene(float(ta))))
        if p: print(d, "first contact ta=%d" % ta, p[:5]); break
        last = ta
    a, u, T = R.solve(last); print(d, "free to ta=%+d  axle dZ %+.2f mm" % (last, T[2, 3]))
for ta in (5, 0, -8): print("ta", ta, "contacts:", real(R.hits(R.scene(float(ta)))))
