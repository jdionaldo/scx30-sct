"""Rest-pose interference over the full v3 assembly (v3 > v2 > .sc exports). BODY_DIR=<dir> python scripts/16_integrate_check.py"""
import os, sys, trimesh
from trimesh.collision import CollisionManager
BD = os.environ["BODY_DIR"]; M = {}
DROP = ({"Chassis_Rail_L","Chassis_Rail_R","Slider_L","Slider_R","Rear_Sway_Bridge_old","Rear_Sway_Bridge","Spare_Tire_Carrier"} if os.environ.get("USE_V4") else set()) | {"Drag_Link","Steering_Ackermann_Bar","Steering_Bell_Crank_L","Steering_Bell_Crank_R","Steering_Pivot_Pin_L","Steering_Pivot_Pin_R","IFS_Knuckle_L","IFS_Knuckle_R","Servo_Ref","Ref_Front_Diff_Ring_Gear","Ref_Front_Input_Worm_Gear","Ref_Front_Center_Driveshaft"}
for d in (BD, "stl/v2", "stl/v3") + (("stl/v4",) if os.environ.get("USE_V4") else ()):
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
