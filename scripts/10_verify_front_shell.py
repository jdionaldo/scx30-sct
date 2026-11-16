"""Front wheels vs relieved shell over steering lock and bump/droop. BODY_DIR=<dir> python scripts/10_verify_front_shell.py"""
import os, sys, io, contextlib, runpy, numpy as np, trimesh
from trimesh.collision import CollisionManager
sys.argv = ["07", os.environ["BODY_DIR"]]
with contextlib.redirect_stdout(io.StringIO()): ns = runpy.run_path("scripts/07_front_v2.py")
shell = trimesh.load("stl/v2/Tacoma_Trophy_Body_Shell.stl")
bad = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, 0.0, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: bad.append((th, s, "no solve")); continue
        cm = CollisionManager(); cm.add_object("shell", shell)
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"): cm.add_object(k, sc[k][0], transform=sc[k][1])
        if cm.in_collision_internal() or cm.min_distance_single(shell) < 0: pass
        h, n = cm.in_collision_internal(return_names=True)
        if any("shell" in p for p in n): bad.append((th, s, sorted(n)))
print("front wheel/shell contacts:", bad if bad else "none over arm -8..+8 deg x rack +/-2.75")
