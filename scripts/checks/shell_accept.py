"""Shell acceptance: one fixed set of checks for every candidate body shell.
usage (repo root): .soma/env/bin/python shell_accept.py <candidate.stl> [--ref scripts/data/shell_soma/soma.stl] [--out DIR]
Prints one line per check ending in PASS or FAIL, then `SHELL ACCEPT: PASS` or `SHELL ACCEPT: FAIL (n)`; exit code 1 on FAIL.
Read-only: loads stl/v3, stl/v4, scripts/make_viewer_3mf.py (DROP list) and scripts/export/make_step.py (mesh_to_shape); writes only into --out.
Units mm; X forward, Y left, Z up. Fixed seeds: two runs give the same numbers."""
import sys, os, re, math, time, argparse, warnings
import numpy as np, trimesh
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.optimize import brentq
from trimesh.collision import CollisionManager
warnings.filterwarnings("ignore")

# ---------------- limits ----------------
BAND = (1.2, 1.6); OUTER_SHARE_MIN = 95.0; EXPO_OUTER = 0.35; N_SAMP = 500000
THIN_LIST, THIN_NEW, THIN_AREA = 0.8, 0.45, 1.0
FID_TOL, FID_SHARE_MIN = 0.15, 99.0
TOOTH_H, TOOTH_TURN, TOOTH_SEG, TOOTH_N, TOOTH_R = 0.1, 25.0, 1.5, 4, 1.2      # chain = >= 4 tooth vertices within 1.2 mm of each other
CLR_BODY, CLR_SHOCK, CLR_WHEEL = 1.4, 2.0, 3.0      # body limit 1.4 (was 1.5): the owner-approved SOMA shell is 1.430 from the tub at the rocker bottom (X 15..22, |Y| 24.43, Z 6.7)
SYM_TOL = 1e-3
# declared 0.30 mm contacts (|Y| boxes, lo/hi): rear tilt hinge on the cage, front magnet seats on the tub / bulkhead
DECL = {"hinge (X about -65)": ([-68.5, 15.0, 22.0], [-58.0, 27.5, 37.0]), "magnet seats": ([51.0, 1.0, 20.0], [63.5, 14.0, 36.0])}
TILT_AXIS = (-64.0, 29.0); TILT_DEG = (0, 0.5, 1, 2, 3, 5, 10, 20, 35, 45)
TA_POSES = (-16, -14, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 7, 8)
# rear kinematics = scripts/18_tub.py (SH 9; A point X -55 - SH)
SH = 9.0
PL = np.array([-26.0 - SH, 2.4]); PU = np.array([-24.53 - SH, 12.0]); A = np.array([-55.0 - SH, -0.5]); Bp = np.array([-57.5 - SH, 8.85])
SHK_LO = np.array([-44.0 - SH, 0.5]); SHK_UP = np.array([-40.2, 28.8])
V3, V4 = "stl/v3/", "stl/v4/"

RES = []
def report(name, ok, text):
    RES.append(ok); print("%-34s %s  %s" % (name, text, "PASS" if ok else "FAIL")); sys.stdout.flush()
def info(text): print("    " + text); sys.stdout.flush()
def ld(p): return trimesh.load(p, process=True)
def fib(n):
    i = np.arange(n) + 0.5; ph = np.arccos(1 - 2*i/n); th = np.pi*(1 + 5**0.5)*i; return np.c_[np.cos(th)*np.sin(ph), np.sin(th)*np.sin(ph), np.cos(ph)]
def bnd(q): return "X %.1f..%.1f Y %.1f..%.1f Z %.1f..%.1f" % (q[:, 0].min(), q[:, 0].max(), q[:, 1].min(), q[:, 1].max(), q[:, 2].min(), q[:, 2].max())
def exact(m, p, ch=20000):
    out = np.empty(len(p))
    for i in range(0, len(p), ch): out[i:i+ch] = trimesh.proximity.closest_point(m, p[i:i+ch])[1]
    return out
def clusters(p, r):
    if len(p) == 0: return np.zeros(0, int)
    pr = cKDTree(p).query_pairs(r, output_type="ndarray")
    if len(pr) == 0: return np.arange(len(p))
    g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(p), len(p))); return connected_components(g, directed=False)[1]
def sample(m, n, seed):
    np.random.seed(seed)
    try: p, fi = trimesh.sample.sample_surface(m, n, seed=seed)
    except TypeError: p, fi = trimesh.sample.sample_surface(m, n)
    return np.asarray(p), np.asarray(fi)

# ---------------- 1. mesh ----------------
def check_mesh(path, m):
    raw = trimesh.load(path, process=False); a = raw.area_faces; nz = int((a < 1e-6).sum())
    report("1a watertight", bool(m.is_watertight and m.is_winding_consistent), "watertight %s, winding %s, volume %.1f mm3, %d faces" % (m.is_watertight, m.is_winding_consistent, m.volume, len(raw.faces)))
    report("1b single body", m.body_count == 1, "bodies %d (limit 1)" % m.body_count)
    report("1c degenerate faces", nz == 0, "%d faces < 1e-6 mm2 (limit 0; smallest %.3g)" % (nz, a.min()))
    try:
        import manifold3d as M
        st = M.Manifold(M.Mesh(vert_properties=np.array(m.vertices, dtype=np.float32), tri_verts=np.array(m.faces, dtype=np.uint32))).status(); ok = st == M.Error.NoError; txt = str(st).split(".")[-1]
    except Exception as e: ok, txt = False, "exception %s" % e
    report("1d manifold3d", ok, "status %s (limit NoError)" % txt)
    v = m.vertices*np.array([1.0, -1.0, 1.0]); d = exact(m, v); k = int(np.argmax(d))
    report("1e mirror symmetry Y=0", d.max() <= SYM_TOL, "max mirrored-vertex-to-surface %.5f mm at (%.2f, %.2f, %.2f) (limit %.0e); %d of %d vertices over" % (d.max(), m.vertices[k][0], m.vertices[k][1], m.vertices[k][2], SYM_TOL, int((d > SYM_TOL).sum()), len(v)))
    try:
        src = open("scripts/export/make_step.py").read().split("\n"); cut = next(i for i, l in enumerate(src) if l.startswith("out = sys.argv")); ns = {"__name__": "make_step_part"}
        exec(compile("\n".join(src[:cut]), "scripts/export/make_step.py", "exec"), ns); t = time.time(); shp = ns["mesh_to_shape"](trimesh.load(path)); nsol = len(shp.Solids()); val = bool(shp.isValid())
        report("1f STEP converter solid", nsol == 1 and val, "solids %d (limit 1), isValid %s (limit True), %.0f s" % (nsol, val, time.time() - t))
    except Exception as e: report("1f STEP converter solid", False, "could not run mesh_to_shape: %s: %s" % (type(e).__name__, e))

# ---------------- 2/3. wall + thin ----------------
def thick(m, p, n, ch=100000):
    t = np.full(len(p), np.nan); fn = m.face_normals
    for i in range(0, len(p), ch):
        o = p[i:i+ch] - n[i:i+ch]*0.002; d = -n[i:i+ch]; loc, ir, it = m.ray.intersects_location(o, d, multiple_hits=True)
        ok = np.einsum('ij,ij->i', fn[it], d[ir]) > 0; loc, ir = loc[ok], ir[ok]; dist = np.linalg.norm(loc - o[ir], axis=1) + 0.002
        tt = np.full(len(o), np.inf); np.minimum.at(tt, ir, dist); tt[np.isinf(tt)] = np.nan; t[i:i+ch] = tt
    return t
def expo(m, p, n, nd=32):
    z = m.bounds[0, 2] - 0.05; L = 500.0; fl = trimesh.Trimesh([[-L, -L, z], [L, -L, z], [L, L, z], [-L, L, z]], [[0, 1, 2], [0, 2, 3]], process=False); mf = trimesh.util.concatenate([m, fl])
    esc = np.zeros(len(p)); val = np.zeros(len(p)); o = p + n*0.02
    D = fib(nd); D = np.vstack([D, D*np.array([1.0, -1.0, 1.0])])      # direction set mirrored about Y=0, so left and right are classified alike
    for d in D:
        k = np.where(n @ d > 0.05)[0]
        if len(k): h = mf.ray.intersects_any(o[k], np.tile(d, (len(k), 1))); val[k] += 1; esc[k] += ~h
    return esc/np.maximum(val, 1)
def survey(m):
    p, fi = sample(m, N_SAMP, 1); n = m.face_normals[fi]; t = thick(m, p, n); ex = expo(m, p, n); return dict(p=p, n=n, t=t, ex=ex, da=m.area/N_SAMP)
def wall_line(s):
    t = s["t"]; fin = ~np.isnan(t); outer = s["ex"] > EXPO_OUTER; band = (t >= BAND[0]) & (t <= BAND[1]); so = lambda sel: (100.0*(band & sel & fin).sum()/max(1, (sel & fin).sum()), int((sel & fin).sum()))
    return so(np.ones(len(t), bool)), so(outer), so(~outer)
def check_wall(sc, sr):
    (a, na), (o, no), (i, ni) = wall_line(sc); (ra, _), (ro, _), (ri, _) = wall_line(sr); t = sc["t"]; outer = sc["ex"] > EXPO_OUTER
    report("2 wall %.1f..%.1f outer skin" % BAND, o >= OUTER_SHARE_MIN, "outer %.1f%% of %d samples (limit >= %.0f%%); all surface %.1f%% of %d; inner-facing %.1f%% of %d; %.4f mm2 per sample" % (o, no, OUTER_SHARE_MIN, a, na, i, ni, sc["da"]))
    info("outer skin: < %.1f: %.1f%%, > %.1f: %.1f%%, median %.3f | reference: outer %.1f%%, all %.1f%%, inner-facing %.1f%%" % (BAND[0], 100*((t < BAND[0]) & outer).sum()/max(1, outer.sum()), BAND[1], 100*((t > BAND[1]) & outer).sum()/max(1, outer.sum()), np.nanmedian(t[outer]), ro, ra, ri))
def thin_clusters(s, lim):
    k = np.where(s["t"] < lim)[0]; p = s["p"][k]; lb = clusters(p, 0.5); out = []
    for c in np.unique(lb):
        j = lb == c; ar = j.sum()*s["da"]
        if ar >= THIN_AREA: out.append((ar, p[j], s["t"][k][j], s["ex"][k][j]))
    return sorted(out, key=lambda r: -r[0])
def check_thin(sc, sr):
    tr = cKDTree(sr["p"][sr["t"] < THIN_NEW]) if (sr["t"] < THIN_NEW).any() else None; rows = thin_clusters(sc, THIN_NEW); new = []
    for ar, q, tq, eq in rows:
        match = np.mean(tr.query(q)[0] < 0.6) if tr is not None else 0.0
        if match < 0.5: new.append((ar, q, tq, eq, match))
    report("3a thin < %.2f new vs reference" % THIN_NEW, len(new) == 0, "%d new regions >= %.0f mm2 (limit 0); %d regions in all, %.1f mm2 below %.2f (reference %.1f mm2)" % (len(new), THIN_AREA, len(rows), (sc["t"] < THIN_NEW).sum()*sc["da"], THIN_NEW, (sr["t"] < THIN_NEW).sum()*sr["da"]))
    for ar, q, tq, eq, match in new: info("new thin %.2f mm2  %s  t min %.3f median %.3f  exposure %.2f  reference match %.0f%%" % (ar, bnd(q), tq.min(), np.median(tq), np.median(eq), 100*match))
    rows8 = thin_clusters(sc, THIN_LIST); r8 = thin_clusters(sr, THIN_LIST)
    print("%-34s %d regions >= %.0f mm2, %.1f mm2 in all (reference: %d regions, %.1f mm2)  INFO" % ("3b thin < %.2f (listed)" % THIN_LIST, len(rows8), THIN_AREA, (sc["t"] < THIN_LIST).sum()*sc["da"], len(r8), (sr["t"] < THIN_LIST).sum()*sr["da"]))
    for ar, q, tq, eq in rows8: info("thin<%.1f %.2f mm2  %s  t min %.3f median %.3f  exposure %.2f" % (THIN_LIST, ar, bnd(q), tq.min(), np.median(tq), np.median(eq)))

# ---------------- 4. exterior fidelity ----------------
def fidelity(src, dst, s_src, lab, num):
    p = s_src["p"][s_src["ex"] > EXPO_OUTER]; d = exact(dst, p)      # exact closest-point distance for every sample
    share = 100.0*np.mean(d <= FID_TOL)
    report("4%s exterior %s" % (num, lab), share >= FID_SHARE_MIN, "%.2f%% of %d outside-visible samples within %.2f mm (limit >= %.0f%%); max %.3f mm, p99 %.3f" % (share, len(p), FID_TOL, FID_SHARE_MIN, d.max(), np.quantile(d, 0.99)))
    b = p[d > FID_TOL]; bd = d[d > FID_TOL]; lb = clusters(b, 1.0); rows = []
    for cl in np.unique(lb):
        j = lb == cl; rows.append((j.sum()*s_src["da"], b[j], bd[j].max()))
    for ar, q, mx in sorted(rows, key=lambda r: -r[0])[:10]: info("deviation cluster %.1f mm2  %s  max %.3f" % (ar, bnd(q), mx))

# ---------------- 5. ragged edges ----------------
def sharp_edges(m, deg=20.0): return m.face_adjacency_edges[m.face_adjacency_angles > np.radians(deg)]
def check_ragged(cand, ref):
    from collections import defaultdict
    e = sharp_edges(cand); V = cand.vertices; n_all = len(e); eo = sharp_edges(ref)
    if len(eo):      # edges lying on a sharp edge of the reference are the reference's own design, not fringe
        pts = np.vstack([ref.vertices[eo[:, 0]] + (ref.vertices[eo[:, 1]] - ref.vertices[eo[:, 0]])*t for t in np.linspace(0, 1, 41)]); tr = cKDTree(pts)
        d = np.maximum(tr.query(V[e].mean(axis=1))[0], np.maximum(tr.query(V[e[:, 0]])[0], tr.query(V[e[:, 1]])[0])); e = e[d > 0.05]
    nb = defaultdict(list)
    for a, b in e: nb[a].append(b); nb[b].append(a)
    P, H = [], []
    for v, ns in nb.items():
        if len(ns) != 2: continue
        a, b, p = V[ns[0]], V[ns[1]], V[v]; l1, l2 = np.linalg.norm(p - a), np.linalg.norm(p - b)
        if l1 > TOOTH_SEG or l2 > TOOTH_SEG or l1 < 1e-9 or l2 < 1e-9: continue
        turn = np.degrees(np.arccos(np.clip(np.dot(p - a, b - p)/(l1*l2), -1, 1))); h = np.linalg.norm(np.cross(p - a, b - a))/(np.linalg.norm(b - a) + 1e-12)
        if turn > TOOTH_TURN and h > TOOTH_H: P.append(p); H.append(h)
    P, H = np.array(P).reshape(-1, 3), np.array(H); lb = clusters(P, TOOTH_R); rows = []
    for c in np.unique(lb):
        j = lb == c
        if j.sum() >= TOOTH_N: rows.append((int(j.sum()), P[j], H[j]))
    report("5 ragged edge chains", len(rows) == 0, "%d chains with >= %d teeth over %.1f mm (limit 0); %d tooth vertices on %d sharp edges not shared with the reference (%d sharp edges in all)" % (len(rows), TOOTH_N, TOOTH_H, len(P), len(e), n_all))
    for n_, q, h in sorted(rows, key=lambda r: -r[0]*np.median(r[2])): info("chain %d teeth  %s  extent %.1f mm  tooth height median %.2f max %.2f" % (n_, bnd(q), np.linalg.norm(np.ptp(q, axis=0)), np.median(h), h.max()))

# ---------------- 6/7. clearances ----------------
def cmo(name, m, T=None):
    c = CollisionManager(); c.add_object(name, m, transform=T); return c
def dist(ca, cb, name_a):
    d, _, dat = ca.min_distance_other(cb, return_names=True, return_data=True); return d, bool(ca.in_collision_other(cb)), np.round(dat.point(name_a), 2)
def split_decl(m):
    """shell faces outside / inside the declared contact boxes; faces that reach into a box are first cut to <= 0.5 mm edges so a large triangle crossing a box is divided at its border (to 0.5 mm)"""
    def inbox(t, lo, hi): q = np.concatenate([t[:, :, :1], np.abs(t[:, :, 1:2]), t[:, :, 2:]], axis=2); return np.all((q >= lo) & (q <= hi), axis=2).any(axis=1)
    t = m.triangles; tl = np.c_[t[:, :, 0].min(axis=1), np.abs(t[:, :, 1]).min(axis=1), t[:, :, 2].min(axis=1)]; th = np.c_[t[:, :, 0].max(axis=1), np.abs(t[:, :, 1]).max(axis=1), t[:, :, 2].max(axis=1)]
    near = np.zeros(len(t), bool)
    for lo, hi in DECL.values(): near |= np.all((th >= lo) & (tl <= hi), axis=1)
    near |= (t[:, :, 1].min(axis=1) < 0) & (t[:, :, 1].max(axis=1) > 0) & (th[:, 0] > 50)      # faces crossing Y=0 at the nose (|Y| boxes)
    v2, f2 = trimesh.remesh.subdivide_to_size(m.vertices, m.faces[near], 0.5) if near.any() else (np.zeros((0, 3)), np.zeros((0, 3), int))
    fine = trimesh.Trimesh(v2, f2, process=False); far = trimesh.Trimesh(m.vertices, m.faces[~near], process=False); anyz = np.zeros(len(f2), bool); zones = {}
    for k, (lo, hi) in DECL.items():
        j = inbox(fine.triangles, lo, hi) if len(f2) else anyz; anyz |= j; zones[k] = trimesh.Trimesh(v2, f2[j], process=False) if j.any() else None
    rest = trimesh.util.concatenate([far, trimesh.Trimesh(v2, f2[~anyz], process=False)]) if (~anyz).any() else far
    return rest, zones
def check_rest(cand):
    rest, zones = split_decl(cand); cr = cmo("s", rest); B = {}
    for n, d in (("Cage", V4), ("Chassis_Tub", V4), ("IFS_Bulkhead_Convertible", V3)):
        if not os.path.exists(d + n + ".stl"): report("6 rest %s" % n, False, "body file missing"); continue
        b = cmo("b", ld(d + n + ".stl")); dd, hit, pt = dist(cr, b, "s"); zt = []; zok = True
        for k, zm in zones.items():
            if zm is None: zt.append("%s: no shell faces" % k); continue
            zd, zh, zp = dist(cmo("s", zm), b, "s"); zok &= not zh; zt.append("%s: %.3f%s at %s" % (k, zd, " CONTACT" if zh else "", zp.tolist()))
        report("6 rest %s" % n, dd >= CLR_BODY and not hit and zok, "%.3f mm at %s outside the declared zones (limit >= %.1f, no contact inside them) | declared: %s" % (dd, pt.tolist(), CLR_BODY, "; ".join(zt)))
    cs = cmo("s", cand)
    for names, d, lim, lab in ((("Ref_Shock_FL", "Ref_Shock_FR"), V3, CLR_SHOCK, "front shocks"), (("Ref_Shock_RL", "Ref_Shock_RR", "Ref_Shock_Eye_L", "Ref_Shock_Eye_R"), V4, CLR_SHOCK, "rear shocks"), (("Ref_Wheel_FL", "Ref_Wheel_FR"), V3, CLR_WHEEL, "front wheels"), (("Ref_Wheel_RL", "Ref_Wheel_RR"), V4, CLR_WHEEL, "rear wheels")):
        r = []; miss = [n for n in names if not os.path.exists(d + n + ".stl")]
        for n in names:
            if n not in miss: dd, hit, pt = dist(cs, cmo("b", ld(d + n + ".stl")), "s"); r.append((dd, hit, n, pt))
        if miss or not r: report("6 rest %s" % lab, False, "missing %s" % miss); continue
        w = min(r, key=lambda x: x[0]); report("6 rest %s" % lab, w[0] >= lim and not any(x[1] for x in r), "min %.3f mm (%s at %s; limit >= %.1f) | %s" % (w[0], w[2], w[3].tolist(), lim, ", ".join("%s %.3f" % (x[2].replace("Ref_", ""), x[0]) for x in r)))
def roty(a, x0, z0):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[0, 0] = c; M[0, 2] = s; M[2, 0] = -s; M[2, 2] = c; M[0, 3] = x0 - (c*x0 + s*z0); M[2, 3] = z0 - (-s*x0 + c*z0); return M
def rot(p, a, o): d = p - o; return o + np.array([d[0]*math.cos(a) + d[1]*math.sin(a), -d[0]*math.sin(a) + d[1]*math.cos(a)])
def solve(ta):
    a = math.radians(ta); L = rot(A, a, PL); d0 = np.linalg.norm(Bp - A); f = lambda u: np.linalg.norm(rot(Bp, u, PU) - L) - d0
    xs = np.radians(np.linspace(-40, 40, 321)); fv = [f(x) for x in xs]; best = None
    for i in range(len(xs) - 1):
        if fv[i]*fv[i+1] <= 0:
            r = brentq(f, xs[i], xs[i+1])
            if best is None or abs(r - a) < abs(best - a): best = r
    U = rot(Bp, best, PU); v0 = Bp - A; v1 = U - L; g = -(math.atan2(v1[1], v1[0]) - math.atan2(v0[1], v0[0]))
    Tt = np.eye(4); Tt[0, 3] = L[0] - A[0]; Tt[2, 3] = L[1] - A[1]; return a, best, Tt @ roty(g, A[0], A[1])
def shock_T(a):
    lo_t = rot(SHK_LO, a, PL); d0 = SHK_UP - SHK_LO; dt = SHK_UP - lo_t
    M = roty(math.atan2(d0[1], d0[0]) - math.atan2(dt[1], dt[0]), SHK_LO[0], SHK_LO[1]); M[0, 3] += lo_t[0] - SHK_LO[0]; M[2, 3] += lo_t[1] - SHK_LO[1]; return M
def check_travel(cand):
    names = ("Rear_Trailing_Arm_L", "Rear_Trailing_Arm_R", "Rear_Upper_Link_L", "Rear_Upper_Link_R", "Ref_Shock_RL", "Ref_Shock_RR", "Ref_Wheel_RL", "Ref_Wheel_RR", "Ref_Rear_Axle", "Rear_Axle_Top_Bracket")
    miss = [n for n in names if not os.path.exists(V4 + n + ".stl")]
    if miss: report("7a rear travel", False, "missing %s" % miss); return
    MV = {n: ld(V4 + n + ".stl") for n in names}; cs = cmo("s", cand); worst = {}; hits = []
    for ta in TA_POSES:
        a, u, Tax = solve(ta)
        for n, m in MV.items():
            T = roty(a, PL[0], PL[1]) if n.startswith("Rear_Trailing_Arm") else roty(u, PU[0], PU[1]) if n.startswith("Rear_Upper_Link") else shock_T(a) if n.startswith("Ref_Shock_R") else Tax
            o = cmo("m", m, T); d = cs.min_distance_other(o)
            if cs.in_collision_other(o): hits.append((ta, n))
            if n not in worst or d < worst[n][0]: worst[n] = (d, ta)
    w = min(worst.items(), key=lambda kv: kv[1][0])
    report("7a rear travel ta %+d..%+d" % (TA_POSES[0], TA_POSES[-1]), not hits, "%d poses x %d movers, contacts %s (limit none); min %.3f mm (%s at ta %+d)" % (len(TA_POSES), len(MV), hits or "none", w[1][0], w[0], w[1][1]))
    info("min per mover: " + ", ".join("%s %.2f@%+d" % (n.replace("Rear_", "").replace("Ref_", ""), d, ta) for n, (d, ta) in sorted(worst.items(), key=lambda kv: kv[1][0])))
def check_tilt(cand):
    exec(re.search(r"DROP = \{.*?\}", open("scripts/make_viewer_3mf.py").read(), re.S).group(0), globals()); src = {}
    for d in (V3, V4):
        for f in sorted(os.listdir(d)):
            if f.endswith(".stl"): src[f[:-4]] = d + f
    # SCT_* (the variant shell's grille and LED bar) are fixed to the shell and tilt with it, so they are not obstacles

    names = sorted(n for n in src if n not in DROP and not n.startswith(("Ref_Wheel_Spare", "HW_Rear_Hinge_Screw", "SCT_")) and n != "Tacoma_Trophy_Body_Shell")
    if "IFS_Bulkhead_Convertible" in src: names.append("IFS_Bulkhead_Convertible")
    allc = CollisionManager()
    for n in names: allc.add_object(n, ld(src[n]))
    hits = []; row = []
    for deg in TILT_DEG:
        T = trimesh.transformations.rotation_matrix(-math.radians(deg), [0, 1, 0], [TILT_AXIS[0], 0.0, TILT_AXIS[1]]); o = cmo("s", cand, T); hit, nm = allc.in_collision_other(o, return_names=True); d, dn = allc.min_distance_other(o, return_names=True); other = [x for x in dn if x != "s"][0]
        if hit: hits.append((deg, sorted({x for p in nm for x in p if x != "s"})))
        row.append("%g:%.2f %s" % (deg, d, other))
    report("7b tilt-open 0..45 deg", not hits, "%d angles x %d bodies (hinge screws = the pivot, left out), contacts %s (limit none)" % (len(TILT_DEG), len(names), hits or "none"))
    info("deg:min mm nearest: " + " | ".join(row))

# ---------------- 8. pictures ----------------
def render(m, eye, target, fov_deg, w=760, h=520, up=(0, 0, 1)):
    eye, target = np.array(eye, float), np.array(target, float); f = target - eye; f /= np.linalg.norm(f); r = np.cross(f, up); r /= np.linalg.norm(r); u = np.cross(r, f)
    t = np.tan(np.radians(fov_deg)/2); X, Y = np.meshgrid(np.linspace(-t, t, w)*(w/h), np.linspace(t, -t, h)); d = f[None, :] + X.ravel()[:, None]*r[None, :] + Y.ravel()[:, None]*u[None, :]; d /= np.linalg.norm(d, axis=1)[:, None]
    loc, ir, it = m.ray.intersects_location(np.tile(eye, (len(d), 1)), d, multiple_hits=False)
    img = np.ones((h*w, 3))*np.array([0.25, 0.35, 0.55]); dep = np.full(h*w, np.nan); nrm = np.zeros((h*w, 3)); n = m.face_normals[it].copy(); n[np.einsum('ij,ij->i', n, d[ir]) > 0] *= -1
    key = np.array([0.3, -0.5, 0.8]); key /= np.linalg.norm(key); shade = 0.55 + 0.45*np.clip(np.einsum('ij,ij->i', n, -d[ir]), 0, 1) + 0.15*np.clip(n @ key, 0, 1)
    img[ir] = np.clip(0.8*shade[:, None], 0, 1); dep[ir] = np.linalg.norm(loc - eye, axis=1); nrm[ir] = n; img = img.reshape(h, w, 3); dep = dep.reshape(h, w); nrm = nrm.reshape(h, w, 3); e = np.zeros((h, w), bool)
    for ax in (0, 1):
        k = (np.nan_to_num(np.abs(np.diff(dep, axis=ax)), nan=9) > 0.12) | ((1 - np.einsum('ijk,ijk->ij', nrm[1:] if ax == 0 else nrm[:, 1:], nrm[:-1] if ax == 0 else nrm[:, :-1])) > 0.06)
        if ax == 0: e[1:] |= k
        else: e[:, 1:] |= k
    img[e] *= 0.25; return img
VIEWS = {      # name: (eye, target, fov deg)
    "01_window_side_up_to_roof": ((-5, -75, 20), (-5, 0, 43), 28), "02_window_windscreen_from_front": ((75, -12, 52), (2, 4, 38), 26), "03_window_rear_from_behind": ((-90, 10, 48), (-10, 0, 40), 26),
    "04_arch_front_L_side_low": ((45, 66.5, 3.8), (45, 26, 22), 26), "05_arch_rear_L_side_low": ((-45, 71, 9), (-45, 26, 27), 26),
    "06_flare_front_underside": ((38, 51.5, -10.9), (38, 29, 28.3), 20), "07_flare_rear_underside": ((-51, 48.3, -12.3), (-46.5, 28, 27.8), 20),
    "08_hood_cowl_from_above": ((75, 25, 75), (25, 0, 31), 30), "09_rear_ledge_from_behind": ((-115, 25, 60), (-55, 0, 30), 28), "10_fins_rear_quarter": ((-95, 70, 62), (-40, 18, 38), 24),
    "11_nose_front": ((120, 14, 27), (62, 6, 26), 24), "12_nose_from_front_low": ((98.7, 15, 2.5), (60.5, 15, 25), 20), "13_Bpillar_inner": ((-51.2, 56.8, 38.5), (-24.2, 23.0, 38.5), 12), "14_from_below": ((0, 0, -110), (0, 0.01, 30), 62)}
def pictures(ref, cand, out, cname):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    os.makedirs(out, exist_ok=True); done = []
    for k, (eye, tgt, fov) in VIEWS.items():
        fig, ax = plt.subplots(1, 2, figsize=(15.2, 5.7), dpi=100)
        for a_, m, lab in ((ax[0], ref, "REFERENCE"), (ax[1], cand, "CANDIDATE " + cname)): a_.imshow(render(m, eye, tgt, fov)); a_.set_title("%s  %s" % (lab, k), fontsize=10); a_.axis("off")
        fig.tight_layout(pad=0.3); p = os.path.join(out, k + ".png"); fig.savefig(p); plt.close(fig); done.append(p)
    print("%-34s %d PNGs (reference left, candidate right) in %s  INFO" % ("8 close-ups", len(done), os.path.abspath(out)))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("candidate"); ap.add_argument("--ref", default="scripts/data/shell_soma/soma.stl"); ap.add_argument("--out", default=None); a = ap.parse_args(); t0 = time.time()
    cand, ref = ld(a.candidate), ld(a.ref)
    print("candidate %s (%d faces, bounds X %.2f..%.2f Y %.2f..%.2f Z %.2f..%.2f) | reference %s (%d faces)" % (a.candidate, len(cand.faces), cand.bounds[0][0], cand.bounds[1][0], cand.bounds[0][1], cand.bounds[1][1], cand.bounds[0][2], cand.bounds[1][2], a.ref, len(ref.faces)))
    check_mesh(a.candidate, cand)
    sc, sr = survey(cand), survey(ref); check_wall(sc, sr); check_thin(sc, sr)
    fidelity(ref, cand, sr, "reference -> candidate", "a"); fidelity(cand, ref, sc, "candidate -> reference", "b")
    check_ragged(cand, ref); check_rest(cand); check_travel(cand); check_tilt(cand)
    if a.out: pictures(ref, cand, a.out, os.path.basename(a.candidate))
    nf = sum(1 for r in RES if not r); print("runtime %.0f s" % (time.time() - t0)); print("SHELL ACCEPT: PASS" if nf == 0 else "SHELL ACCEPT: FAIL (%d)" % nf); sys.exit(0 if nf == 0 else 1)
if __name__ == "__main__": main()
