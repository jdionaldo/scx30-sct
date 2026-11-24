"""Mesh hygiene pass: remove degenerate / zero-area faces and near-duplicate vertices from the printed part STLs
in stl/v3 and stl/v4 (stl/v4 wins), without remeshing.  Run from anywhere:  python scripts/20_mesh_clean.py [--dry-run]

A part is rewritten only if the cleaned mesh, AFTER a float32 binary STL write and reload, has
  0 degenerate faces (battery.py test: trimesh.load, area_faces < 1e-4), watertight, 1 body, consistent winding,
  manifold3d NoError, |dV| <= 0.01 mm3 (or 2 ppm of the part volume if larger: 0.027 for the shell), Hausdorff both ways (20k samples + all vertices) <= 0.002 mm (MAX_DEV_PART: tub 0.004), bounds 1e-4.
Otherwise the file is left untouched and the reason is printed.  Idempotent: clean parts are not rewritten.
Exit code 1 if a written file fails re-verification.  The active set comes from scripts/make_viewer_3mf.py (DROP)."""
import argparse, os, sys
from collections import defaultdict
from itertools import permutations
import numpy as np, trimesh, manifold3d as m3
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIRS = ("stl/v3", "stl/v4")                       # later folder wins, as in battery.py
NOT_PRINTED = ("HW_", "Ref_", "CVD_", "Front_Bearing", "Servo_Ref", "_")
DEG = 1e-4                                        # mm2, battery.py section B
MAX_DVOL, MAX_DEV, MAX_DBOUND, N_SAMP = 0.01, 0.002, 1e-4, 20000
MAX_DEV_PART = {"Chassis_Tub": 0.004}             # owner-approved override: 71 real micro-facets need ~0.003 mm
BUDGETS = (1e-5, 3e-4, 1e-3, 1.8e-3, 4e-3, 8e-3)  # local deviation allowed per edit, mm; tried while <= 2 x the part limit

def printed_parts():
    sys.path.insert(0, os.path.join(REPO, "scripts")); from make_viewer_3mf import DROP as drop
    src = {}
    for d in DIRS:
        p = os.path.join(REPO, d)
        for f in sorted(os.listdir(p)) if os.path.isdir(p) else ():
            if f.endswith(".stl"): src[f[:-4]] = os.path.join(p, f)
    return {n: src[n] for n in sorted(src) if n not in drop and not n.startswith(NOT_PRINTED)}

def n_deg(m):
    return int((m.area_faces < DEG).sum())

def one_way(a, b):
    """Max distance from a to b: N_SAMP surface samples of a plus every vertex of a."""
    pts = np.vstack([trimesh.sample.sample_surface(a, N_SAMP, seed=1)[0], a.vertices])
    return float(trimesh.proximity.closest_point(b, pts)[1].max())

def problems(m, ref=None):
    """Reasons m (as reloaded from disk) is not acceptable; empty list = ok.  ref = the mesh before cleaning."""
    out = [f"{n_deg(m)} degenerate"] if n_deg(m) else []
    if not m.is_watertight: out.append("not watertight")
    elif len(m.split(only_watertight=False)) != 1: out.append("bodies != 1")
    if not m.is_winding_consistent: out.append("winding")
    man = m3.Manifold(m3.Mesh(vert_properties=np.asarray(m.vertices, dtype=np.float32), tri_verts=np.asarray(m.faces, dtype=np.uint32)))
    if not out and man.status() != m3.Error.NoError: out.append("manifold3d error")
    if ref is not None and not out:
        if abs(m.volume - ref.volume) > max(MAX_DVOL, 2e-6*abs(ref.volume)): out.append(f"dV {m.volume - ref.volume:+.4f}")
        if np.abs(m.bounds - ref.bounds).max() > MAX_DBOUND: out.append(f"bounds moved {np.abs(m.bounds - ref.bounds).max():.5f}")
    return out

def _area_n(T):
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    return np.linalg.norm(n, axis=1) / 2, n

def _fan_dist(Ta, Tb):
    """Max distance from triangle fan Ta (corners, edge midpoints, centroids) to triangle fan Tb."""
    P = np.vstack([Ta.reshape(-1, 3), (Ta + np.roll(Ta, 1, axis=1)).reshape(-1, 3) / 2, Ta.mean(1)])
    Pt = np.tile(P, (len(Tb), 1))
    return np.linalg.norm(trimesh.triangles.closest_point(np.repeat(Tb, len(P), axis=0), Pt) - Pt, axis=1).reshape(len(Tb), len(P)).min(0).max()

def _score(V, F, A, N, old, G, ref, v=None, pos=None):
    """Local deviation of replacing faces `old` by the triangles G (vertex v at `pos`), or None if the edit does not
    reduce the degenerate count or turns a healthy face over.  ref[i] = the old face new face i is compared with."""
    Tn = V[G]
    if v is not None: Tn[G == v] = pos
    a2, n2 = _area_n(Tn)
    if (a2 < DEG).sum() >= (A[old] < DEG).sum() or (((n2 * N[ref]).sum(1) <= 0) & (A[ref] >= DEG)).any(): return None
    return max(_fan_dist(V[F[old]], Tn), _fan_dist(Tn, V[F[old]]))

def repair(V, F, budget):
    """Targeted edits on degenerate faces only: collapse one of its edges (u merged into v, v kept or moved to the float32
    midpoint), flip its longest edge, or grow it just over DEG.  Each edit keeps a 2-manifold within `budget` mm."""
    V, F = V.copy(), F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])].copy()   # drop repeated-vertex faces
    for _ in range(80):
        A, N = _area_n(V[F]); bad = np.where(A < DEG)[0]
        vf, dead, lock = defaultdict(list), [], set()
        for i, tri in enumerate(F.tolist()):
            for w in tri: vf[w].append(i)
        ring = lambda w: set(F[vf[w]].ravel().tolist()) - {w}
        for f in bad[np.argsort(A[bad])]:
            tri, ops = F[f].tolist(), []
            if lock & set(tri): continue
            for u, v in permutations(tri, 2):                                    # collapse u -> v
                die = [i for i in vf[u] if i in vf[v]]
                if len(die) != 2 or len(ring(u) & ring(v)) != 2: continue
                keep = [i for i in vf[u] + vf[v] if i not in die]; G = np.where(F[keep] == u, v, F[keep])
                for pos in (V[v], ((V[u] + V[v]) / 2).astype(np.float32).astype(float))[:2 if u < v else 1]:
                    dev = _score(V, F, A, N, keep + die, G, keep, v, pos)
                    if dev is not None: ops.append((dev, len(ops), keep + die, G, v, pos, ring(u) | ring(v) | {u, v}))
            e = int(np.argmax(np.linalg.norm(V[tri] - V[np.roll(tri, -1)], axis=1)))   # flip the longest edge a-b
            a, b, c = tri[e], tri[(e + 1) % 3], tri[(e + 2) % 3]
            g = [i for i in vf[a] if i in vf[b] and i != f]
            d = [w for w in F[g[0]].tolist() if w not in (a, b)][0] if len(g) == 1 else c
            if d not in ring(c) | {c}:
                G = np.array([[a, d, c], [d, b, c]]); dev = _score(V, F, A, N, [f, g[0]], G, [g[0], g[0]])
                if dev is not None: ops.append((dev, len(ops), [f, g[0]], G, None, None, {a, b, c, d}))
            for u in tri:                                                        # grow: push u away from the opposite edge
                o = V[[w for w in tri if w != u]]; ed = o[1] - o[0]; h = V[u] - o[0]; h = h - ed * (h @ ed) / (ed @ ed)
                pos = (V[u] + h / max(np.linalg.norm(h), 1e-12) * (2.1 * DEG / np.linalg.norm(ed) - np.linalg.norm(h))).astype(np.float32).astype(float)
                dev = _score(V, F, A, N, vf[u], F[vf[u]], vf[u], u, pos)
                if dev is not None: ops.append((dev, len(ops), vf[u], F[vf[u]], u, pos, ring(u) | {u}))
            if ops and min(ops)[0] <= budget:
                _, _, old, G, v, pos, touched = min(ops)
                F[old[:len(G)]] = G; dead += old[len(G):]; lock |= touched
                if v is not None: V[v] = pos
        if not lock: break
        F = np.delete(F, dead, axis=0)
    q = trimesh.Trimesh(V, F, process=False); q.remove_unreferenced_vertices()
    return q

def clean(name, path, dry):
    """Returns (report line, written, failed re-verify)."""
    ref, lim = trimesh.load(path), MAX_DEV_PART.get(name, MAX_DEV)   # process=True: the same view battery.py has
    f0, d0 = len(ref.faces), n_deg(ref)
    head = f"{name:28s} {os.path.basename(os.path.dirname(path))} faces {f0:6d}"
    if not problems(ref): return f"{head} -> {f0:6d}  deg    0 ->    0  ok (already clean)", False, False
    tmp, why = os.path.join(os.path.dirname(path), f"_clean_{name}.stl"), ""
    try:
        for budget in [b for b in BUDGETS if b <= 2 * lim]:   # mildest first
            q = repair(np.asarray(ref.vertices), np.asarray(ref.faces), budget)
            if n_deg(q): why = f"{n_deg(q)} degenerate left at budget {budget:g}"; continue
            q.export(tmp); r = trimesh.load(tmp)  # float32 binary STL round trip
            bad = problems(r, ref)
            dev = max(one_way(ref, r), one_way(r, ref)) if not bad else 0.0
            if dev > lim: bad = [f"deviation {dev:.4f}"]
            if bad: why = f"budget {budget:g}: " + ", ".join(bad); continue
            line = f"{head} -> {len(r.faces):6d}  deg {d0:4d} ->    0  dV {r.volume - ref.volume:+.5f}  dev {dev:.5f}  "
            if dry: return line + f"ok (dry run, budget {budget:g})", False, False
            os.replace(tmp, path)
            again = problems(trimesh.load(path), ref)
            return line + (f"FAILED re-verify: {', '.join(again)}" if again else f"ok (written, budget {budget:g})"), True, bool(again)
        return f"{head} -> {f0:6d}  deg {d0:4d} -> {d0:4d}  skipped, file untouched: {why}", False, False
    finally:
        if os.path.exists(tmp): os.remove(tmp)

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry-run", action="store_true", help="report only, write nothing")
    ap.add_argument("--only", nargs="*", help="part names to process (default: all printed parts)")
    a = ap.parse_args()
    parts = {n: p for n, p in printed_parts().items() if not a.only or n in a.only}
    written = failed = skipped = 0
    for name, path in parts.items():
        line, w, f = clean(name, path, a.dry_run)
        print(line, flush=True)
        written += w; failed += f; skipped += "skipped" in line
    print(f"{len(parts)} printed parts: {written} written, {skipped} skipped (still dirty), {failed} failed re-verify")
    sys.exit(1 if failed else 0)
if __name__ == "__main__": main()
