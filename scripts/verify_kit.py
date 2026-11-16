"""Regression gate: CadQuery shell validity, 0 overlapping holes, full build, interference check, and close-up renders."""
import glob, json, math, os, subprocess, sys

BASE = "/Users/johndionaldo/GitHub/scx30-sct"
ART = "/Users/johndionaldo/.gemini/antigravity/brain/51089506-1ea2-4a48-b78b-7db34dfb5b53"

# 1. CadQuery shell build + isValid()
subprocess.run([f"{BASE}/.venv/bin/python", f"{BASE}/scripts/build_shell_cadquery.py"], check=True)

# 2. Overlapping hole check across all module scripts
overlaps = 0
for path in sorted(glob.glob(f"{BASE}/scripts/0*.json")):
    cmds = json.load(open(path))["commands"]
    holes = [(i, c["params"].get("name",""), c["params"].get("targets"), c["params"].get("base"), c["params"].get("diameter"), c["params"].get("axis"))
             for i, c in enumerate(cmds) if c["command"] == "solid.cylinder" and c["params"].get("operation") == "cut"]
    for a in range(len(holes)):
        for b in range(a+1, len(holes)):
            ia, na, ta, ba, da, axa = holes[a]
            ib, nb, tb, bb, db, axb = holes[b]
            if ta == tb and axa == axb and isinstance(axa, str):
                idx = {"X": (1,2), "Y": (0,2), "Z": (0,1)}[axa]
                dist = math.hypot(ba[idx[0]]-bb[idx[0]], ba[idx[1]]-bb[idx[1]])
                if dist < (da+db)/2 + 0.25 and dist > 1e-4:
                    print("FAIL OVERLAP:", os.path.basename(path), ia, na, "vs", ib, nb, f"dist={dist:.3f}")
                    overlaps += 1
if overlaps:
    sys.exit(1)
print("PASS: 0 overlapping holes across all scripts")

# 3. Full assembly rebuild + interference check
subprocess.run(["solidcraft-cli", "run", "/tmp/full_cq.json", "--quiet"], check=True)
subprocess.run(["solidcraft-cli", "run", "/tmp/check_interf.json", "--in", f"{BASE}/scx30_sct_107mm_kit.sc", "--results", "--json-out", "/tmp/interf_res.json", "--quiet"], check=True)
res = json.load(open("/tmp/interf_res.json"))[0]
print(f"INTERFERENCE: checked={res['checked']}, count={res['count']}")
for it in res["interferences"]:
    print("  ", it["a"], "<->", it["b"], "vol=", round(it["volume_mm3"], 4))

# 4. Render close-up of front fender flare
subprocess.run([
    "solidcraft-cli", "snapshot", f"{BASE}/scx30_sct_107mm_kit.sc",
    "--out", f"{ART}/v23_front_fender_verified.png",
    "--width", "1280", "--height", "800", "--quality", "fine",
    "--camera", '{"target":[40.0,20.0,13.5],"yaw":0.55,"pitch":0.10,"distance":28.0}'
], check=True)
print("PASS: verify_kit.py complete")
