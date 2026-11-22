#!/usr/bin/env python3
"""Build a color-coded, named multi-part 3MF of the current assembly for the SOMA 3D viewer.

Usage: python scripts/make_viewer_3mf.py [--out scx30_sct_assembly.3mf]
"""
import argparse, os, zipfile
import numpy as np, trimesh

DROP = {
    "Chassis_Rail_L", "Chassis_Rail_R", "Slider_L", "Slider_R",
    "Rear_Sway_Bridge_old", "Rear_Sway_Bridge", "Spare_Tire_Carrier",
    "Drag_Link", "Steering_Ackermann_Bar", "Steering_Bell_Crank_L",
    "Steering_Bell_Crank_R", "Steering_Pivot_Pin_L", "Steering_Pivot_Pin_R",
    "IFS_Knuckle_L", "IFS_Knuckle_R", "Tacoma_Trophy_Body_Shell_old",
    "Front_Wheel_Envelope", "IFS_Bulkhead_Convertible", "Ref_Servo_Envelope",
}

MATERIALS = [
    ("Shell",       "#D9532F"),  # 0: Tacoma trophy shell (terracotta/orange)
    ("Cage",        "#3B82F6"),  # 1: Rear & cabin cages (blue)
    ("Tub",         "#374151"),  # 2: Chassis tub & shelves (dark slate)
    ("Front_Susp",  "#64748B"),  # 3: Bulkhead, arms, carriers, spindles, bumper
    ("Steering",    "#10B981"),  # 4: Rack, horn, drag link, bell crank, tie rods
    ("Rear_Susp",   "#475569"),  # 5: Trailing arms, upper links, bracket
    ("Shocks",      "#F59E0B"),  # 6: Shocks & bump stops (amber/gold)
    ("Wheels",      "#1F2937"),  # 7: Tyres/wheels (charcoal)
    ("Drivetrain",  "#9CA3AF"),  # 8: Axle, gearbox, motor, CVD, bearings, shafts
    ("Electronics", "#6366F1"),  # 9: Servo, ESC, RX (indigo)
    ("Hardware",    "#D1D5DB"),  # 10: Screws, nuts, pins (steel silver)
]


def mat_index(name):
    if "Shell" in name: return 0
    if "Cage" in name: return 1
    if name.startswith("Chassis_Tub"): return 2
    if name.startswith(("Steering_", "Tie_Rod_", "Servo_Horn")): return 4
    if name.startswith(("Rear_Trailing", "Rear_Upper", "Rear_Axle_Top")): return 5
    if "Shock" in name or "Bump_Stop" in name: return 6
    if "Wheel" in name: return 7
    if name.startswith(("Servo_", "Ref_ESC", "Ref_RX")): return 9
    if name.startswith("HW_"): return 10
    if name.startswith(("Ref_", "CVD_", "Front_Bearing")): return 8
    return 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="scx30_sct_assembly.3mf")
    args = ap.parse_args()

    src = {}
    for d in ("stl/v3", "stl/v4"):
        for f in sorted(os.listdir(d)):
            if f.endswith(".stl"):
                src[f[:-4]] = os.path.join(d, f)
    names = sorted(n for n in src if n not in DROP and not n.startswith("Ref_Wheel_Spare"))

    ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<model unit="millimeter" xml:lang="en-US" xmlns="{ns}">',
        ' <metadata name="Title">SCX30 SCT Assembly</metadata>',
        ' <metadata name="Designer">scx30-sct</metadata>',
        ' <resources>',
        '  <basematerials id="1">',
    ]
    for mname, col in MATERIALS:
        lines.append(f'   <base name="{mname}" displaycolor="{col}"/>')
    lines.append('  </basematerials>')

    total_v = 0
    total_f = 0
    obj_ids = []
    for idx, name in enumerate(names, 2):
        m = trimesh.load(src[name], process=True)
        verts = m.vertices.astype(np.float32)
        faces = m.faces
        total_v += len(verts)
        total_f += len(faces)
        pidx = mat_index(name)
        lines.append(f'  <object id="{idx}" name="{name}" type="model" pid="1" pindex="{pidx}"><mesh><vertices>')
        lines.extend(f'   <vertex x="{v[0]:.9g}" y="{v[1]:.9g}" z="{v[2]:.9g}"/>' for v in verts)
        lines.append('  </vertices><triangles>')
        lines.extend(f'   <triangle v1="{f[0]}" v2="{f[1]}" v3="{f[2]}"/>' for f in faces)
        lines.append('  </triangles></mesh></object>')
        obj_ids.append(idx)

    lines.append(' </resources><build>')
    lines.extend(f'  <item objectid="{oid}"/>' for oid in obj_ids)
    lines.append(' </build></model>')

    ct = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
        '</Types>'
    )
    rels = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
        '</Relationships>'
    )

    with zipfile.ZipFile(args.out, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", "\n".join(lines))

    sz_mb = os.path.getsize(args.out) / (1024 * 1024)
    print(f"Wrote {args.out}: {len(names)} parts, {total_v:,} vertices, {total_f:,} triangles ({sz_mb:.1f} MB)")


if __name__ == "__main__":
    main()
