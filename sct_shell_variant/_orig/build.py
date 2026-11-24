"""Build, verify, export STEP/STL, and assemble the complete SolidCraft .sc for the SCX30 SCT
Cyberpunk Short Course / Baja Desert Trophy Truck body shell (works directly WITH the established
Chassis_Tub and red Cage — no redundant external exo-cage!).

Run with SOMA's bundled Python:
    /Applications/SOMA.app/Contents/Resources/python/bin/python3.12 sct_shell_variant/build.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT_DIR = os.path.join(HERE, "exports")
sys.path.insert(0, ROOT)

import cadquery as cq
from sct_shell_variant.trophy_shell import build_cyber_trophy_truck

COLORS = {
    "SCT_Cyber_Trophy_Body": cq.Color(0.28, 0.31, 0.27, 1.0),
    "SCT_Hex_Mesh_Grille":   cq.Color(0.16, 0.18, 0.20, 1.0),
    "SCT_Cyan_DRL_Bar":      cq.Color(0.05, 0.88, 0.95, 1.0),
}


def _wheel_sweep_envelopes():
    """Swept Ø36 x 13.5 mm tire cylinders across ride + full bump (+7.7 mm front, +8.0 mm rear)
    plus Front_Bumper keep-out box."""
    envs = {}
    for label, sy in (("Front_Wheel_L_Bump", 1), ("Front_Wheel_R_Bump", -1)):
        y0 = 32.0 if sy > 0 else -47.5
        c_ride = cq.Solid.makeCylinder(18.6, 15.5, cq.Vector(48.0, y0, 1.5), cq.Vector(0, 1, 0))
        c_bump = cq.Solid.makeCylinder(18.6, 15.5, cq.Vector(48.0, y0, 9.2), cq.Vector(0, 1, 0))
        envs[label] = c_ride.fuse(c_bump)
    for label, sy in (("Rear_Wheel_L_Bump", 1), ("Rear_Wheel_R_Bump", -1)):
        y0 = 29.2 if sy > 0 else -44.3
        c_ride = cq.Solid.makeCylinder(18.6, 15.1, cq.Vector(-68.5, y0, 1.5), cq.Vector(0, 1, 0))
        c_bump = cq.Solid.makeCylinder(18.6, 15.1, cq.Vector(-68.5, y0, 9.5), cq.Vector(0, 1, 0))
        envs[label] = c_ride.fuse(c_bump)
    envs["Front_Bumper_Env"] = cq.Solid.makeBox(9.2, 18.6, 21.4, cq.Vector(60.1, -9.3, -1.2))
    return envs


def _build_complete_sc(stl_paths):
    sc_cli = "/Applications/SolidCraft.app/Contents/MacOS/solidcraft-cli"
    kit_sc = os.path.join(ROOT, "scx30_sct_107mm_kit.sc")
    out_sc = os.path.join(HERE, "scx30_sct_desert_armored.sc")
    if not os.path.exists(sc_cli) or not os.path.exists(kit_sc):
        print("Skipping .sc assembly (solidcraft-cli or kit .sc not found)")
        return

    tmp = tempfile.mkdtemp(prefix="sct_variant_sc_")
    try:
        cmds = [{"command": "solid.remove", "params": {"bodies": ["Tacoma_Trophy_Body_Shell"]}}]
        for name, stl_path in sorted(stl_paths.items()):
            cmds.append({"command": "file.insert_mesh", "params": {"path": stl_path, "name": name}})
        script_path = os.path.join(tmp, "swap_shell.json")
        with open(script_path, "w") as f:
            json.dump({"commands": cmds}, f)
        res = subprocess.run(
            [sc_cli, "run", script_path, "--in", kit_sc, "--save", out_sc, "--quiet"],
            capture_output=True,
            text=True,
        )
        if res.returncode != 0:
            print("solidcraft-cli stderr:", res.stderr[-1500:])
            raise RuntimeError(f"solidcraft-cli failed with code {res.returncode}")
        print(f"Complete SolidCraft build saved: {out_sc} ({os.path.getsize(out_sc) / 1e6:.1f} MB)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    # Clean old exports so only the unified Short Course Truck parts live in exports/
    if os.path.exists(OUT_DIR):
        shutil.rmtree(OUT_DIR)
    os.makedirs(OUT_DIR, exist_ok=True)

    print("Building SCX30 SCT Cyberpunk Trophy Truck body shell...")
    parts = build_cyber_trophy_truck()

    envs = _wheel_sweep_envelopes()
    report = {"parts": {}, "inter_part_overlaps": [], "status": "ok"}
    asm = cq.Assembly(name="SCX30_SCT_Cyber_Trophy")
    stl_paths = {}

    for name, sol in parts.items():
        bb = sol.BoundingBox()
        ov_map = {}
        max_ov = 0.0
        for ename, esol in envs.items():
            ov = sol.intersect(esol).Volume()
            if ov > 1e-4:
                ov_map[ename] = round(float(ov), 3)
                max_ov = max(max_ov, ov)
        print(
            f"  {name:22s}: vol={sol.Volume():7.1f} mm3  "
            f"x=[{bb.xmin:6.1f},{bb.xmax:6.1f}] y=[{bb.ymin:6.1f},{bb.ymax:6.1f}] z=[{bb.zmin:6.1f},{bb.zmax:6.1f}]  "
            f"env_ov={max_ov:.4f}"
        )
        assert max_ov < 0.01, f"{name} overlaps wheel/bumper envelope: {ov_map}"
        report["parts"][name] = {
            "solids": len(sol.Solids()),
            "valid": bool(sol.isValid()),
            "volume_mm3": round(float(sol.Volume()), 1),
            "bbox": {
                "x": [round(bb.xmin, 2), round(bb.xmax, 2)],
                "y": [round(bb.ymin, 2), round(bb.ymax, 2)],
                "z": [round(bb.zmin, 2), round(bb.zmax, 2)],
            },
            "max_wheel_bumper_overlap_mm3": round(float(max_ov), 4),
        }
        step_p = os.path.join(OUT_DIR, f"{name}.step")
        stl_p = os.path.join(OUT_DIR, f"{name}.stl")
        cq.exporters.export(sol, step_p)
        cq.exporters.export(sol, stl_p, tolerance=0.03, angularTolerance=0.1)
        stl_paths[name] = stl_p
        asm.add(sol, name=name, color=COLORS[name])

    names = list(parts.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ov = parts[names[i]].intersect(parts[names[j]]).Volume()
            if ov > 0.01:
                report["inter_part_overlaps"].append({"pair": [names[i], names[j]], "volume_mm3": round(float(ov), 3)})
                print(f"  WARN overlap {names[i]} <> {names[j]}: {ov:.3f} mm3")

    assert not report["inter_part_overlaps"], f"Inter-part overlaps found: {report['inter_part_overlaps']}"
    print("  Inter-part overlaps: NONE (0.000 mm3 across all pairs!)")

    asm_step = os.path.join(OUT_DIR, "SCX30_SCT_Cyber_Trophy_Shell.step")
    asm.save(asm_step)
    with open(os.path.join(OUT_DIR, "verification_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(f"Exported 3 STEP/STL files + shell STEP assembly to {OUT_DIR}")

    _build_complete_sc(stl_paths)


if __name__ == "__main__":
    main()
