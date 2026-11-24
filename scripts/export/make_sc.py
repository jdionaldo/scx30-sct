"""Regenerate scx30_sct_107mm_kit.sc from stl/v3 + stl/v4. Run it LAST, after every geometry batch (see AGENTS.md).
Usage (repo root):  python scripts/export/make_sc.py [--dry]      needs solidcraft-cli on PATH.

The .sc keeps its older modelled history (sketches, extrudes); only the mesh bodies are swapped:
  1. measure the current file: body names and its MeshFeature ids;
  2. remove every current body, insert every active STL (active set and source folder = scripts/make_viewer_3mf.py; stl/v4 wins);
  3. delete the old MeshFeatures and the RemoveFeatures of step 2 from the timeline, so the file does not grow;
  4. check: body names equal the STL names, no duplicates, every bounding box within 0.05 mm of its STL.
Work files go to a temp folder; the repo file is replaced only when step 4 passes.
"""
import collections, json, os, shutil, subprocess, sys, tempfile
import trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from make_viewer_3mf import DROP  # noqa: E402  (the ONE active-set list)

KIT = os.path.join(ROOT, "scx30_sct_107mm_kit.sc")
BBOX_TOL = 0.05


def cli(*a):
    r = subprocess.run(["solidcraft-cli", *a], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("solidcraft-cli %s failed:\n%s" % (a[0], r.stderr[-1500:]))


def load(p):
    j = json.load(open(p))
    return j if isinstance(j, list) else [j]


def main():
    dry = "--dry" in sys.argv
    T = tempfile.mkdtemp(prefix="make_sc_")
    src = {}
    for d in ("stl/v3", "stl/v4"):
        for f in sorted(os.listdir(os.path.join(ROOT, d))):
            n = f[:-4]
            if f.endswith(".stl") and n not in DROP and not n.startswith("Ref_Wheel_Spare"):
                src[n] = os.path.join(ROOT, d, f)
    snap = os.path.join(T, "snap"); os.makedirs(snap)
    for n, p in src.items():
        shutil.copy(p, os.path.join(snap, n + ".stl"))      # snapshot: a rebuild running meanwhile cannot change what goes in

    measure = os.path.join(T, "m.json"); json.dump({"commands": [{"command": "inspect.measure", "params": {}}]}, open(measure, "w"))
    cli("eval", measure, "--in", KIT, "--json-out", os.path.join(T, "cur.json"))
    cur = load(os.path.join(T, "cur.json"))[-1]
    cur_names = [b["name"] for b in cur["bodies"]]
    old_mesh = [str(f["id"]) for f in cur["timeline"] if f["type"] == "MeshFeature"]
    print("%d STL bodies; .sc has %d bodies, %d mesh features; leaving: %s; new: %s" % (
        len(src), len(cur_names), len(old_mesh), sorted(set(cur_names) - set(src)), sorted(set(src) - set(cur_names))))
    if dry:
        return

    cm = [{"command": "solid.remove", "params": {"bodies": [n]}} for n in sorted(cur_names)]
    cm += [{"command": "file.insert_mesh", "params": {"path": os.path.join(snap, n + ".stl"), "name": n}} for n in sorted(src)]
    swap = os.path.join(T, "swap.json"); json.dump({"commands": cm}, open(swap, "w"))
    cli("run", swap, "--in", KIT, "--save", os.path.join(T, "a.sc"), "--results", "--json-out", os.path.join(T, "swap_out.json"))
    removes = [str(r["feature"]) for r in load(os.path.join(T, "swap_out.json")) if "feature" in r and "path" not in r]

    prune = os.path.join(T, "prune.json")
    json.dump({"commands": [{"command": "timeline.delete", "params": {"features": old_mesh + removes}}, {"command": "inspect.measure", "params": {}}]}, open(prune, "w"))
    cli("run", prune, "--in", os.path.join(T, "a.sc"), "--save", os.path.join(T, "b.sc"), "--results", "--json-out", os.path.join(T, "prune_out.json"))
    new = load(os.path.join(T, "prune_out.json"))[-1]["bodies"]

    names = [b["name"] for b in new]
    dups = [k for k, v in collections.Counter(names).items() if v > 1]
    bad = []
    for b in new:
        if b["name"] in src:
            m = trimesh.load(os.path.join(snap, b["name"] + ".stl"), process=False)
            d = max(abs(p - q) for p, q in zip(b["bbox"]["min"] + b["bbox"]["max"], m.bounds.flatten()))
            if d > BBOX_TOL: bad.append((b["name"], round(float(d), 3)))
    print("bodies %d; duplicates %s; only in .sc %s; only in STL %s; bbox mismatches %s" % (
        len(names), dups, sorted(set(names) - set(src)), sorted(set(src) - set(names)), bad))
    if dups or bad or set(names) != set(src):
        sys.exit("check failed: the repo .sc was NOT replaced (work files in %s)" % T)
    shutil.copy(os.path.join(T, "b.sc"), KIT)
    print("wrote %s (%.1f MB)" % (KIT, os.path.getsize(KIT) / 1e6))
    shutil.rmtree(T)


if __name__ == "__main__":
    main()
