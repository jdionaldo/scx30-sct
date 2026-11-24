"""v4 body shell WITHOUT the hood scoop -> stl/v4/Tacoma_Trophy_Body_Shell.stl (writes only stl/v4).
Base = stl/tacoma_trophy_shell.stl, widened like scripts/12 (side panels), then upgraded with Axial Yeti (AXIC9025)
tilt-open body mounting (2 front magnet bosses under the unbroken hood + 2 rear tilt-hinge tabs with 45-deg teardrop crown
and inner-wall fillet gusset) and fit & finish (smooth chamfered front wheel arches, +/-10 deg cross-axle roll rear wheel-arch
relief, recessed front grille surround + twin
headlight bucket pockets, recessed center hood contour + twin louvered hood pads, and clean open-rear tail framing around
Cage's AX80103 rear light bar / radiator).
usage: BODY_DIR=<per-body stl dir> python scripts/19_shell_v4.py   (run scripts/18_tub.py first; does not run 17_hood_scoop.py)"""
import os, sys, math, io, contextlib, runpy
from collections import defaultdict
import numpy as np, trimesh, manifold3d as m3d
from scipy.optimize import brentq
from scipy.spatial import ConvexHull
from scipy.interpolate import LinearNDInterpolator
from trimesh.collision import CollisionManager

BD = os.environ["BODY_DIR"]; V3 = os.environ.get("V3_DIR", "stl/v3/"); V4 = "stl/v4/"
sys.path.insert(0, "scripts"); import rearlib as R
SH = float(os.environ.get("REAR_SHIFT", 9.0)); R.shift_rear(SH); ROOF_LIFT = float(os.environ.get("ROOF_LIFT", 3.2))      # = 18_tub.py: rigid shell lift
def _ss(t): t = np.clip(t, 0, 1); return t*t*(3 - 2*t)
def lift(m, H=None):
    H = ROOF_LIFT if H is None else H; m = m.copy(); v = m.vertices.copy(); x, z = v[:, 0], v[:, 2]
    fx = _ss((x + 46.0)/10.0)*(1 - 0.85*_ss((x - 8.0)/16.0)); v[:, 2] += H*_ss((z - 17.0)/12.0)*fx; m.vertices = v; return m
SILL_Z = 6.7; SILL_BAND = 8.0     # original sill height; lower body stretched over this band so sill stays flat at Z=6.7
def lowlift(m, H=None):
    H = ROOF_LIFT if H is None else H; m = m.copy(); v = m.vertices.copy(); v[:, 2] += H*_ss((v[:, 2] - SILL_Z)/SILL_BAND); m.vertices = v; return m
def shift_tail(m):
    m = m.copy(); v = m.vertices.copy(); v[:, 0] -= SH*_ss((-30.0 - v[:, 0])/10.0); m.vertices = v; return m
PL = np.array([-26.0 - SH, float(os.environ.get("PL_Z", 2.4))]); PU = np.array([-24.53 - SH, float(os.environ.get("PU_Z", 12.0))]); R.PL = PL; R.PU = PU; R.A = np.array([-55.0 - SH, -0.5]); R.Bp = np.array([-57.5 - SH, 8.85])   # = 18_tub.py
RAIL_DY = float(os.environ.get("RAIL_DY", 4.0)); SHELL_MARGIN = float(os.environ.get("SHELL_MARGIN", 1.5)); WARP_DY = RAIL_DY + SHELL_MARGIN; CLR = float(os.environ.get("CLR", 0.3))
DELTA = 5.8      # Rev 42: tub stack raised (= 18_tub.py)
GBX_X, GBX_Y, GBX_Z0, GBX_Z1, SHAFT_Z = 6.0, 12.5, -4.9 + DELTA, 23.1 + DELTA, 4.1 + DELTA; MOTOR_R, MOTOR_L, MOTOR_Z = 8.5, 10.0, 18.1 + DELTA; MOTOR_X1 = -GBX_X; MOTOR_X0 = MOTOR_X1 - MOTOR_L
BAR_X, BAR_Z, BAR_Y, BAR_E_X = float(os.environ.get("PXH", -78.0)), 16.3 + 4.2, 19.1, -46.75 - SH; LINK_Y = 19.75; TAB_X, TAB_TOP_Z, LINK_INS = -46.75 - SH, 4.9, 0.8; WIRE_R, LINK_R = 0.4, 0.45

# ---- Axial Yeti (AXIC9025) tilt-open body mount coordinates (matched to 13_front_v3.py #23 & 18_tub.py #7/#17) ----
MAG_X, MAG_Y = float(os.environ.get("MAG_X", 57.3)), float(os.environ.get("MAG_Y", 7.6))      # front magnet seat centres (= 13_front_v3.py, final)
MAG_SEAT_Z = float(os.environ.get("MAG_SEAT_Z", 27.2))           # top Z of the chassis magnet seat (= 13_front_v3.py, final)
MAG_GAP = 0.40; MAG_T = 2.0                                      # air gap between the two magnet faces at rest; D3.0 x 2.0 disc
MAG_POCKET_D, MAG_POCKET_DEPTH, MAG_WALL = 3.20, 2.10, 1.20      # playbook: pocket = magnet dia + 0.2; blind pocket opening downward; >= 0.8 mm wall around
MAG_SKIN = 0.70                                                  # closed skin above the pocket (>= 0.6); a low bevelled hood pad is raised where the stock hood is too thin

TAIL_END_X = float(os.environ.get("TAIL_END_X", -67.0))            # rear end of the shell/bed: over the middle of the rear tyre (axle X -68.5) and exactly the end of the constant-slope zone
# Rear rail slope (the sloping roof-to-tail rail, |Y| 21.10..22.98). Reference = its top ridge on the inner face |Y| = REAR_RAIL_Y_IN (side-view silhouette):
#   Z_top(X) = REAR_SLOPE_Z0 + REAR_SLOPE*(X - REAR_SLOPE_X0),  underside on the same face: Z_bot(X) = REAR_UNDER_Z0 + REAR_UNDER_SLOPE*(X - REAR_SLOPE_X0)
# valid for REAR_SLOPE_XA >= X >= TAIL_END_X; measured on the built shell and re-checked at the end of this script.
REAR_RAIL_Y_IN = 21.100
REAR_SLOPE_X0, REAR_SLOPE_Z0, REAR_SLOPE = -64.0, 33.480, 0.28400      # top ridge: 15.855 deg
REAR_UNDER_Z0, REAR_UNDER_SLOPE = 30.998, 0.26150                      # underside: 14.655 deg (the rail tapers 2.48 -> 3.29 mm rear to front)
REAR_SLOPE_XA = -28.0
REAR_UNDER_XB = -55.0                                             # behind this X the hinge gusset fills the space under the rail
REAR_SLOPE_DEG, REAR_UNDER_DEG = math.degrees(math.atan(REAR_SLOPE)), math.degrees(math.atan(REAR_UNDER_SLOPE))
def rear_rail_z_top(x): return REAR_SLOPE_Z0 + REAR_SLOPE*(x - REAR_SLOPE_X0)
def rear_rail_z_under(x): return REAR_UNDER_Z0 + REAR_UNDER_SLOPE*(x - REAR_SLOPE_X0)
REAR_LEDGE_Z = 30.00                                             # top of the flat bed ledge the rail lands on
REAR_HINGE_X, REAR_HINGE_Z = TAIL_END_X + 3.0, 29.0                         # horizontal Y-axis rear tilt-open hinge axis on Cage (#7)
REAR_HINGE_YIN, REAR_HINGE_YOUT = 18.90, 21.30                   # 0.30 mm running gap to Cage hinge ear outer face (|Y|=18.60)
REAR_HINGE_BORE_D = float(os.environ.get("REAR_HINGE_BORE_D", 1.70))  # M2 pilot (screw enters from inboard through the Cage ear D2.2); horizontal bore with 45-deg teardrop crown (#16/#17)
HINGE_BLOCK_YOUT = 24.9                                          # outboard face of the screw block
REAR_HINGE_BOSS_R = 2.60                                         # OD 5.2 mm lug (>= 1.40 mm crown wall above teardrop apex)

def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
def U(ms): return trimesh.boolean.union(list(ms), engine="manifold")
def D(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def ld(n, d=V4): return trimesh.load(d + n + ".stl")
def cyl(axis, c, r, h, n=48):
    m = trimesh.creation.cylinder(radius=r, height=h, sections=n)
    if axis == "x": m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    elif axis == "y": m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    m.apply_translation(c); return m
def _ensure_ccw(poly):
    p = np.asarray(poly, float)
    area2 = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    return list(reversed(poly)) if area2 < 0 else list(poly)
def _fix_vol(m, label=""):
    m = m.copy()
    if not m.is_watertight:
        m.process(validate=True)
        parts = [p for p in m.split(only_watertight=False) if len(p.faces) >= 4 and abs(p.volume) > 1e-3]
        if len(parts) > 1: m = max(parts, key=lambda p: abs(p.volume))
    if m.volume < 0: m.invert()
    if not m.is_volume:
        trimesh.repair.fix_normals(m)
        if m.volume < 0: m.invert()
    assert m.is_volume and len(m.faces) > 0, f"{label} invalid volume! wt={m.is_watertight} vol={m.volume}"
    return m
def _bevel_convex(poly_2d, h_bot, h_sh, h_top, ch, label="bevel"):
    """Exact single-facet 45-deg beveled convex prism (zero stair-stepping)."""
    p0 = np.asarray(_ensure_ccw(poly_2d), float)
    polys = m3d.CrossSection([p0.tolist()]).offset(-ch, m3d.JoinType.Miter, 2.0).to_polygons()
    assert len(polys) >= 1, f"{label}: chamfer {ch} collapsed polygon"
    p1 = np.asarray(max(polys, key=lambda q: len(q)), float)
    pts = np.vstack([
        np.c_[p0, np.full(len(p0), h_bot)],
        np.c_[p0, np.full(len(p0), h_sh)],
        np.c_[p1, np.full(len(p1), h_top)],
    ])
    return _fix_vol(trimesh.convex.convex_hull(pts), label)
def _bevel_strip_ribbon(spine_outer, spine_inner, h_bot, h_sh, h_top, ch, label="ribbon"):
    """Single-facet beveled ribbon (zero stair-stepping) built from paired 2D boundary chains."""
    pieces = []
    for i in range(len(spine_outer) - 1):
        quad = [spine_outer[i], spine_outer[i + 1], spine_inner[i + 1], spine_inner[i]]
        pieces.append(_bevel_convex(quad, h_bot, h_sh, h_top, ch, f"{label}_{i}"))
    return U(pieces)

def _clean_mesh_for_stl(m, area_tol=5e-6, edge_tol=5e-4, fold_dot=-0.92, ladder=((0.03, 35.0), (0.08, 65.0), (0.25, 85.0), (1.2, 88.0)), flips=True):
    """Simplify via manifold3d, quantize to float32, and eliminate all degenerate faces (<area_tol) & folded fins."""
    mm = m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
    if mm.status() == m3d.Error.NoError and mm.num_tri() > 0:
        me = mm.simplify(2e-3).to_mesh()
        q = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3].astype(np.float32).astype(np.float64), np.asarray(me.tri_verts), process=True)
        parts = sorted(q.split(only_watertight=False), key=lambda t: -abs(t.volume))
        if parts and parts[0].is_watertight and abs(parts[0].volume - m.volume) < 2e-3 * abs(m.volume):
            m = parts[0]
    # Work directly on float32-quantized vertices so binary STL roundtrip has 0 rounding drift
    V = np.asarray(m.vertices, dtype=np.float32).astype(np.float64)
    q0 = trimesh.Trimesh(V, m.faces, process=True)
    V = np.asarray(q0.vertices, dtype=np.float32).astype(np.float64)
    F_list = [(int(f[0]), int(f[1]), int(f[2])) for f in q0.faces if f[0] != f[1] and f[1] != f[2] and f[2] != f[0]]
    alive = [True]*len(F_list)
    he = defaultdict(set); orient_map = defaultdict(set); vert_faces = defaultdict(set)
    areas = []; min_edges = []
    def _co(a, b, c):
        mn = min(a, b, c)
        return (a, b, c) if a == mn else ((b, c, a) if b == mn else (c, a, b))
    def _tn(a, b, c):
        cr = np.cross(V[b] - V[a], V[c] - V[a]); ln = float(np.linalg.norm(cr))
        return (cr/ln, 0.5*ln) if ln > 1e-18 else (np.zeros(3), 0.0)
    def _fm(f):
        a, b, c = f; _, ar = _tn(a, b, c)
        me = min(float(np.linalg.norm(V[b] - V[a])), float(np.linalg.norm(V[c] - V[b])), float(np.linalg.norm(V[a] - V[c])))
        return ar, me
    def is_bad(fi): return alive[fi] and (areas[fi] < area_tol or min_edges[fi] < edge_tol)
    def add_f(fi):
        a, b, c = F_list[fi]; alive[fi] = True
        he[(a, b)].add(fi); he[(b, c)].add(fi); he[(c, a)].add(fi)
        orient_map[_co(a, b, c)].add(fi)
        vert_faces[a].add(fi); vert_faces[b].add(fi); vert_faces[c].add(fi)
        ar, me = _fm(F_list[fi])
        if fi < len(areas): areas[fi] = ar; min_edges[fi] = me
        else: areas.append(ar); min_edges.append(me)
    def del_f(fi):
        if not alive[fi]: return
        a, b, c = F_list[fi]; alive[fi] = False
        he[(a, b)].discard(fi); he[(b, c)].discard(fi); he[(c, a)].discard(fi)
        orient_map[_co(a, b, c)].discard(fi)
        vert_faces[a].discard(fi); vert_faces[b].discard(fi); vert_faces[c].discard(fi)
    def app_f(f):
        fi = len(F_list); F_list.append(f); alive.append(True); add_f(fi); return fi
    def opp_v(fi, u, v):
        a, b, c = F_list[fi]
        return c if (a == u and b == v) else (a if (b == u and c == v) else (b if (c == u and a == v) else -1))
    def cancel_opp():
        for fi in range(len(F_list)):
            if not alive[fi]: continue
            a, b, c = F_list[fi]
            cs = [fj for fj in orient_map.get(_co(b, a, c), ()) if fj != fi and alive[fj]]
            if cs: del_f(fi); del_f(cs[0])
    for fi in range(len(F_list)): add_f(fi)
    cancel_opp()
    # Flip or collapse folded-fin edges (dot(n1, n2) < fold_dot) & degenerate triangles
    bad_set = {fi for fi in range(len(F_list)) if is_bad(fi)}
    for fi in range(len(F_list)):
        if not alive[fi]: continue
        a, b, c = F_list[fi]; n1, _ = _tn(a, b, c)
        for u, v in ((a, b), (b, c), (c, a)):
            for fj in he.get((v, u), ()):
                if alive[fj] and fj != fi:
                    n2, _ = _tn(*F_list[fj])
                    if float(np.dot(n1, n2)) < fold_dot:
                        bad_set.add(fi); bad_set.add(fj)
    for _ in range(4000 if flips else 0):
        if not bad_set: break
        did = False
        for mode in (0, 1, 2):
            for fi in sorted(bad_set):
                if not alive[fi]: bad_set.discard(fi); continue
                a, b, c = F_list[fi]
                n1, _ = _tn(a, b, c)
                # Find worst edge (folded edge or longest edge)
                es = [(a, b, c, float(np.linalg.norm(V[b] - V[a]))), (b, c, a, float(np.linalg.norm(V[c] - V[b]))), (c, a, b, float(np.linalg.norm(V[a] - V[c])))]
                es.sort(key=lambda x: -x[3])
                # Also prioritize any edge with folded dihedral
                for idx_e, (u_, v_, o_, l_) in enumerate(es):
                    tws = [fj for fj in he.get((v_, u_), ()) if fj != fi and alive[fj]]
                    if len(tws) == 1:
                        n2, _ = _tn(*F_list[tws[0]])
                        if float(np.dot(n1, n2)) < fold_dot:
                            es.insert(0, es.pop(idx_e)); break
                p, q, opp, pq_len = es[0]
                if pq_len < 1e-12: continue
                twins = [fj for fj in he.get((q, p), ()) if fj != fi and alive[fj]]
                if len(twins) != 1 or len(he.get((p, q), ())) != 1: continue
                fj = twins[0]; opp2 = opp_v(fj, q, p)
                if opp2 < 0 or opp2 == opp or float(np.linalg.norm(V[opp2] - V[opp])) < edge_tol: continue
                n2, _ = _tn(*F_list[fj])
                was_folded = float(np.dot(n1, n2)) < fold_dot
                if not is_bad(fi) and not was_folded:
                    bad_set.discard(fi); continue
                nf1 = (opp, opp2, q); nf2 = (opp2, opp, p)
                if any(alive[k] for k in orient_map.get(_co(*nf1), ())) or any(alive[k] for k in orient_map.get(_co(*nf2), ())): continue
                c1 = [k for k in orient_map.get(_co(nf1[1], nf1[0], nf1[2]), ()) if k != fi and k != fj and alive[k]]
                c2 = [k for k in orient_map.get(_co(nf2[1], nf2[0], nf2[2]), ()) if k != fi and k != fj and alive[k]]
                if not c1 and not c2 and (he.get((opp, opp2)) or he.get((opp2, opp))): continue
                a1, me1 = _fm(nf1); a2, me2 = _fm(nf2)
                nn1, _ = _tn(*nf1); nn2, _ = _tn(*nf2)
                now_folded = (not c1 and not c2 and float(np.dot(nn1, nn2)) < fold_dot)
                if now_folded: continue
                ok1 = bool(c1) or (a1 >= area_tol and me1 >= edge_tol)
                ok2 = bool(c2) or (a2 >= area_tol and me2 >= edge_tol)
                accept = False
                if mode == 0:
                    accept = (ok1 and ok2) or ((bool(c1) or bool(c2)) and (bool(c1) or a1 >= area_tol) and (bool(c2) or a2 >= area_tol))
                elif mode == 1:
                    accept = (is_bad(fj) or was_folded) and float(np.linalg.norm(V[opp2] - V[opp])) < pq_len - 1e-9 and a1 >= area_tol and a2 >= area_tol
                elif mode == 2 and not c1 and not c2:
                    accept = min(a1, a2) > min(areas[fi], areas[fj]) + 1e-10 and (a1 + a2) >= areas[fi] + areas[fj] - 1e-9
                if accept:
                    del_f(fi); del_f(fj); bad_set.discard(fi); bad_set.discard(fj)
                    if c1: del_f(c1[0]); bad_set.discard(c1[0])
                    else:
                        nfi = app_f(nf1)
                        if is_bad(nfi): bad_set.add(nfi)
                    if c2: del_f(c2[0]); bad_set.discard(c2[0])
                    else:
                        nfj = app_f(nf2)
                        if is_bad(nfj): bad_set.add(nfj)
                    did = True; break
            if did: break
        if not did: break
    def try_col(u, v, max_len, allow_deg):
        if float(np.linalg.norm(V[u] - V[v])) > max_len: return False
        if V[u, 2] < 6.75 and abs(V[u, 2] - V[v, 2]) > 1e-5: return False   # never pull sill vertices off Z=6.7
        f_uv = [fi for fi in he.get((u, v), ()) if alive[fi]]; f_vu = [fi for fi in he.get((v, u), ()) if alive[fi]]
        if len(f_uv) != 1 or len(f_vu) != 1: return False
        f1, f2 = f_uv[0], f_vu[0]; w1 = opp_v(f1, u, v); w2 = opp_v(f2, v, u)
        if w1 < 0 or w2 < 0 or w1 == w2: return False
        nu = {x for fi in vert_faces[u] if alive[fi] for x in F_list[fi] if x != u}
        nv = {x for fi in vert_faces[v] if alive[fi] for x in F_list[fi] if x != v}
        if (nu & nv) != {w1, w2}: return False
        inc = [fi for fi in vert_faces[u] if alive[fi] and fi != f1 and fi != f2]
        cos_l = math.cos(math.radians(allow_deg)); nfs = []
        for fi in inc:
            nf = tuple(v if x == u else x for x in F_list[fi])
            if len(set(nf)) < 3: return False
            ar_n, me_n = _fm(nf)
            if ar_n < area_tol or me_n < edge_tol: return False
            if areas[fi] >= area_tol:
                no, _ = _tn(*F_list[fi]); nn, _ = _tn(*nf)
                if float(np.dot(no, nn)) < cos_l: return False
            if any(alive[k] for k in orient_map.get(_co(*nf), ())): return False
            nfs.append((fi, nf))
        del_f(f1); del_f(f2); bad_set.discard(f1); bad_set.discard(f2)
        for fi, nf in nfs:
            del_f(fi); bad_set.discard(fi); nfi = app_f(nf)
            if is_bad(nfi): bad_set.add(nfi)
        return True
    # Also add any remaining folded faces to bad_set for collapse
    for fi in range(len(F_list)):
        if not alive[fi]: continue
        a, b, c = F_list[fi]; n1, _ = _tn(a, b, c)
        for u, v in ((a, b), (b, c), (c, a)):
            for fj in he.get((v, u), ()):
                if alive[fj] and fj != fi:
                    n2, _ = _tn(*F_list[fj])
                    if float(np.dot(n1, n2)) < fold_dot:
                        bad_set.add(fi); bad_set.add(fj)
    for ml, dg in ladder:
        for _ in range(800):
            if not bad_set: break
            did = False
            for fi in sorted(bad_set):
                if not alive[fi]: bad_set.discard(fi); continue
                a, b, c = F_list[fi]
                es = sorted([(a, b, float(np.linalg.norm(V[b] - V[a]))), (b, c, float(np.linalg.norm(V[c] - V[b]))), (c, a, float(np.linalg.norm(V[a] - V[c])))], key=lambda x: x[2])
                for u, v, _ in es:
                    if try_col(u, v, ml, dg) or try_col(v, u, ml, dg):
                        did = True; break
                if did: break
            if not did: break
    cancel_opp()
    F_out = np.array([F_list[fi] for fi in range(len(F_list)) if alive[fi]], dtype=np.int64)
    out = trimesh.Trimesh(V.astype(np.float32), F_out, process=True)
    if out.is_watertight and out.is_volume and len(out.split(only_watertight=False)) == 1:
        return out
    return m

CLEAN_TOL = 0.03      # mm: the cleanup may not move the surface further than this
def _surf_dev(a, b, n=40000):
    """Two-way surface deviation (mm) between meshes a and b."""
    pa = trimesh.sample.sample_surface(a, n, seed=3)[0]; pb = trimesh.sample.sample_surface(b, n, seed=4)[0]
    return max(float(trimesh.proximity.closest_point(b, pa)[1].max()), float(trimesh.proximity.closest_point(a, pb)[1].max()))
def _clean_checked(m):
    """_clean_mesh_for_stl with a geometry guard: its edge flips and long edge collapses (up to 1.2 mm) can fold a wall away, so each
    result is compared with the input and the next gentler setting is used when the surface moved more than CLEAN_TOL."""
    full = ((0.03, 35.0), (0.08, 65.0), (0.25, 85.0), (1.2, 88.0)); out = m
    for lad, fl in ((full, True), ((), True), (full[:2], False), (full[:1], False), ((), False)):
        out = _clean_mesh_for_stl(m, ladder=lad, flips=fl); d = _surf_dev(m, out)
        print("    cleanup (max collapse %.2f mm, flips %s): surface moved %.3f mm, vol %.0f -> %.0f, deg %d%s" % (lad[-1][0] if lad else 0.0, fl, d, m.volume, out.volume, int((out.area_faces < 1e-6).sum()), "" if d <= CLEAN_TOL else "  -> rejected"))
        if d <= CLEAN_TOL: return out
    return m

SIMPLIFY_TOL = float(os.environ.get("SIMPLIFY_TOL", 0.012))   # mm: manifold edge collapse that merges sliver/degenerate triangles (guarded like the cleanup)
def _simplify_checked(m):
    """Collapses degenerate and sliver triangles (edges shorter than SIMPLIFY_TOL); kept only if still one watertight shell within CLEAN_TOL."""
    if SIMPLIFY_TOL <= 0: return m
    try:
        o = m3d.Manifold(m3d.Mesh(np.asarray(m.vertices, dtype='float32'), np.asarray(m.faces, dtype='uint32'))).simplify(SIMPLIFY_TOL).to_mesh()
        t = trimesh.Trimesh(np.asarray(o.vert_properties)[:, :3], np.asarray(o.tri_verts), process=True)
        sh = sorted(t.split(only_watertight=False), key=lambda q: -abs(q.volume)); t = sh[0]
        ok = t.is_watertight and open_edges(t) == 0 and abs(t.volume - m.volume) < 5.0
        d = _surf_dev(m, t) if ok else 9.9
        print("    simplify %.3f: faces %d -> %d, deg(<1e-4) %d -> %d, surface moved %.3f mm%s" % (SIMPLIFY_TOL, len(m.faces), len(t.faces), int((m.area_faces < 1e-4).sum()), int((t.area_faces < 1e-4).sum()), d, "" if d <= CLEAN_TOL else "  -> rejected"))
        return t if d <= CLEAN_TOL else m
    except Exception as e:
        print("    simplify failed:", e); return m

def dil(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(3), c*b_/math.sqrt(3), c*e_/math.sqrt(3)] for a_ in (1, -1) for b_ in (1, -1) for e_ in (1, -1)):
        for f_ in (1.0, 0.5): k = m.copy(); k.apply_translation([q*f_ for q in d]); ms.append(k)
    return U(ms)
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
def vol(a, b):
    try: return abs(trimesh.boolean.intersection([a, b], engine="manifold").volume)
    except Exception: return float('nan')
def open_edges(m): _, c = np.unique(m.edges_sorted, axis=0, return_counts=True); return int((c == 1).sum())
def warp(m, dy, y0=8.0, y1=10.0):
    m = m.copy(); v = m.vertices.copy(); a = np.abs(v[:, 1]); v[:, 1] += np.sign(v[:, 1])*dy*np.clip((a - y0)/(y1 - y0), 0, 1); m.vertices = v; return m
def pocket(c):
    a = box(-GBX_X - c, GBX_X + c, -GBX_Y - c, GBX_Y + c, GBX_Z0 - c, GBX_Z1 + c)
    m = trimesh.creation.cylinder(radius=MOTOR_R + c, height=MOTOR_L + 2*c, sections=48); m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    m.apply_translation([(MOTOR_X0 + MOTOR_X1)/2, 0, MOTOR_Z])
    return U([a, m, box(-15.5 - c, 15.5 + c, -2.1, 2.1, SHAFT_Z - 2.1, SHAFT_Z + 2.1)])

def stretch_tail(m, xb=-48.0, ext=17.0):
    m = m.copy(); v = m.vertices.copy(); x0 = v[:, 0].min(); k = (xb - x0 + ext)/(xb - x0); sel = v[:, 0] < xb; v[sel, 0] = xb - (xb - v[sel, 0])*k; m.vertices = v; return m
TAIL_EXT = float(os.environ.get("TAIL_EXT", 18.0))
TAIL_E = float(os.environ.get("TAIL_E", 17.0))      # total rear lengthening (original tail end X -61.5 -> -78.5)
def lengthen(m, E=None, xa=-50.0, xb=-22.0):
    E = TAIL_E if E is None else E; m = m.copy(); v = m.vertices.copy(); x = v[:, 0]; k = 1.0 + E/(xb - xa)
    v[:, 0] = np.where(x >= xb, x, np.where(x >= xa, xb - (xb - x)*k, x - E)); m.vertices = v; return m
FRONT_EXT = float(os.environ.get("FRONT_EXT", 8.0))      # Rev 33: nose/hood lengthened forward (+8 mm)
def extend_nose(m, E, xa=47.0, xb=53.0):
    m = m.copy(); v = m.vertices.copy(); v[:, 0] += E*_ss((v[:, 0] - xa)/(xb - xa)); m.vertices = v; return m
BASE = lowlift(warp(extend_nose(lengthen(trimesh.load("stl/tacoma_trophy_shell.stl")), FRONT_EXT), WARP_DY))
SHELL_LIFT = float(os.environ.get("SHELL_LIFT", 11.0))     # Rev 29: rigid lift of the WHOLE shell; sill at Z=6.7 stays flat
def skirt_lift(m, H, band=1.0):
    m = m.copy(); v = m.vertices.copy(); v[:, 2] += H*_ss((v[:, 2] - SILL_Z)/band); m.vertices = v; return m
if SHELL_LIFT > 0: BASE = skirt_lift(BASE, SHELL_LIFT)
RAW_BASE = BASE.copy()     # clean reference hood/body prior to Yeti trim additions
print("base shell: vol %.0f bounds %s watertight %s" % (BASE.volume, BASE.bounds.round(1).tolist(), BASE.is_watertight))

# ---- static keepouts (v4 parts + v3 bumper) ----
STATIC = {n: ld(n) for n in ("Chassis_Tub", "Cage", "Cabin_Cage", "Ref_ESC_G40", "ESC_RX_Shelf_L", "ESC_RX_Shelf_R") if os.path.exists(V4 + n + ".stl")}
if os.path.exists(V3 + "Front_Bumper.stl"): STATIC["Front_Bumper"] = ld("Front_Bumper", V3)
STATIC["Ref_Servo_Envelope"] = ld("Ref_Servo_Envelope") if os.path.exists(V4 + "Ref_Servo_Envelope.stl") else U([box(13.4, 33.5, 1.8, 11.7, -4.9 + DELTA, 10.8 + DELTA), box(32.3, 33.5, 11.7, 14.65, -3.8 + DELTA, 9.4 + DELTA)])
STATIC["Ref_Battery_2S"] = box(-74.9, -29.6, -10.5, 10.5, 18.1, 29.3)
for n in ("Ref_Shock_RL", "Ref_Shock_RR", "Ref_Shock_Eye_L", "Ref_Shock_Eye_R"): STATIC[n] = ld(n)
if os.path.exists(V3 + "IFS_Bulkhead_Convertible.stl"): STATIC["IFS_Bulkhead_v3 (magnet seats)"] = ld("IFS_Bulkhead_Convertible", V3)

print("front magnet spec: X=%.2f Y=+-%.2f seat top Z=%.2f gap %.2f -> shell magnet face Z=%.2f" % (MAG_X, MAG_Y, MAG_SEAT_Z, MAG_GAP, MAG_SEAT_Z + MAG_GAP))

# ---- rear sweep poses (same maths as 18_tub.py) ----
LR = BAR_E_X - BAR_X; L0 = BAR_Z - (TAB_TOP_Z - LINK_INS)
def wcyl(p0, p1, r): return trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1], float), sections=20)
def bar_state(ta):
    T = R.rot(np.array([TAB_X, TAB_TOP_Z - LINK_INS]), math.radians(ta), PL)
    E = lambda th: np.array([BAR_X + LR*math.cos(th), BAR_Z - LR*math.sin(th)])
    f = lambda th: np.linalg.norm(E(th) - T) - L0
    xs = np.linspace(-1.2, 1.2, 241); fv = [f(x) for x in xs]; best, bd = 0.0, 9
    for i in range(len(xs) - 1):
        if fv[i]*fv[i + 1] <= 0:
            r = brentq(f, xs[i], xs[i + 1])
            if abs(r) < bd: best, bd = r, abs(r)
    return best, E(best), T
MOVE_ARM = {n: ld(n) for n in ("Rear_Trailing_Arm_L", "Rear_Trailing_Arm_R", "HW_Rear_Trailing_Pivot_Screw_L", "HW_Rear_Trailing_Pivot_Screw_R")}
MOVE_UL = {n: ld(n) for n in ("Rear_Upper_Link_L", "Rear_Upper_Link_R", "HW_Rear_Upper_Link_Pivot_Screw_L", "HW_Rear_Upper_Link_Pivot_Screw_R")}
AXG = {n: R.M[n] for n in ("Ref_Rear_Axle", "Ref_Wheel_RL", "Ref_Wheel_RR")}
for n in ("Rear_Axle_Top_Bracket", "HW_Rear_Axle_Top_Pin", "HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R"): AXG[n] = ld(n)
LEV = {sg: trimesh.util.concatenate([wcyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R), wcyl((BAR_E_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*LINK_Y, BAR_Z), WIRE_R)]) for sg in (1, -1)}
CROSS = wcyl((BAR_X, -BAR_Y - 0.4, BAR_Z), (BAR_X, BAR_Y + 0.4, BAR_Z), WIRE_R)
def rear_movers(ta):
    a, u, T = R.solve(ta); th, E_, T_ = bar_state(ta); sc = {}
    for n, m in MOVE_ARM.items(): sc[n] = posed(m, R.roty(a, PL[0], PL[1]))
    for n, m in MOVE_UL.items(): sc[n] = posed(m, R.roty(u, PU[0], PU[1]))
    for n, m in AXG.items(): sc[n] = posed(m, T)
    for n, sg in (("L", 1), ("R", -1)):
        sc["Bar_lever_" + n] = posed(LEV[sg], R.roty(th, BAR_X, BAR_Z)); sc["Bar_link_" + n] = wcyl((E_[0], sg*LINK_Y, E_[1]), (T_[0], sg*LINK_Y, T_[1]), LINK_R)
    sc["Bar_cross"] = CROSS; return sc
TA_MIN, TA_MAX = float(os.environ.get("TA_MIN", -16.0)), float(os.environ.get("TA_MAX", 8.0))      # rear stops: droop / bump (deg, = 18_tub.py)
TA = [float(t) for t in np.arange(TA_MIN, TA_MAX + 0.01, 1.0)]
touch = set(); POSES = {}
for ta in TA:
    sc = rear_movers(ta); POSES[ta] = sc
    cm = CollisionManager(); cm.add_object("shell", BASE)
    for n, m in sc.items():
        o = CollisionManager(); o.add_object(n, m)
        if cm.in_collision_other(o) or cm.min_distance_other(o) < CLR + 0.1: touch.add(n)
print("rear movers touching the widened base shell:", sorted(touch))
def hull_of(ms): return trimesh.convex.convex_hull(np.vstack([m.vertices for m in ms]))
SW = {n: [] for n in touch}
for ta in np.arange(TA_MIN, TA_MAX + 0.01, 0.5):
    sc = rear_movers(float(ta))
    for n in touch: SW[n].append(sc[n])
sweep = [hull_of(v) for v in SW.values()]

# ---- front wheel envelope (lock x travel), same idea as 08_relief.py / 10_verify_front_shell.py ----
sys.argv = ["07", BD]
with contextlib.redirect_stdout(io.StringIO()): ns = runpy.run_path("scripts/07_front_v2.py")
FW = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, 0.0, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: continue
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            if k in sc: FW.append(posed(*sc[k]))
cm = CollisionManager(); cm.add_object("shell", BASE); fw_touch = 0
for w in FW:
    o = CollisionManager(); o.add_object("w", w); fw_touch += bool(cm.in_collision_other(o))
print("front wheel poses touching the base shell: %d of %d" % (fw_touch, len(FW)))
RELIEF_FW = []
for nm in ("L", "R"):
    ws = []
    for w in FW:
        o = CollisionManager(); o.add_object("w", w)
        if cm.in_collision_other(o) and (w.centroid[1] > 0) == (nm == "L"): ws.append(w)
    if ws: RELIEF_FW.append(hull_of(ws))

_FE = trimesh.load(V3 + "Front_Wheel_Envelope.stl")
for _p in _FE.split(only_watertight=False):
    if len(_p.vertices) > 8: RELIEF_FW.append(trimesh.convex.convex_hull(_p.vertices))

# =====================================================================
# Axial Yeti (AXIC9025) Shell Fit & Finish + Tilt-Open Body Mounting
# =====================================================================
FENDER = os.environ.get("FENDER", "1") != "0"
FEN_CLR = float(os.environ.get("FEN_CLR", 1.5)) + float(os.environ.get("ARCH_X", 0.25)) + 0.10
FEN_XE = float(os.environ.get("FEN_XE", 49.5)); FEN_X0 = 14.5; FEN_Z0 = SILL_Z
def _wall_y(z): return 25.18 + 0.0757*(z - 10.0)      # inner face of cab side wall at X~10..16 (~4.3 deg slant)
def _side_outer_y(z):
    return np.where(z <= 22.0, 26.17 + (27.48 - 26.17)*(z - 7.5)/14.5, 27.48 - (27.48 - 26.38)*(z - 22.0)/6.0)
def _map_to_side(m_uv, y_fn, sg, label="side"):
    q = m_uv.vertices.copy()
    w = np.c_[q[:, 0], sg*(y_fn(q[:, 1]) + q[:, 2]), q[:, 1]]
    return _fix_vol(trimesh.Trimesh(w, m_uv.faces.copy(), process=False), label)

FEN_FILL_YIN, FEN_FILL_X1, FEN_FILL_Z1 = 23.90, 20.4, 25.45      # solid filler behind the front fender: swallows the stock 0.2..0.45 mm inner sheet and spur
FEN_IN = float(os.environ.get("FEN_IN", 0.15))     # extra inboard pan depth (outer face and arch-lip chamfer unchanged); 0.15 keeps Cabin_Cage >= 1.5 mm
def build_clean_fenders(base):
    """Smooth 2D-offset front fender side panels with single-facet 45-deg chamfered wheel-arch lip (no ragged boolean teeth)."""
    xs = [FEN_X0, 17.0, 19.5, 22.0, 24.0, 28.0, 32.0, 38.0, 44.0, FEN_XE]
    pans = []
    fe_all = [q for q in _FE.split(only_watertight=False) if len(q.vertices) > 4]
    for sg in (1, -1):
        top = []
        for x in xs:
            L_, _, _ = base.ray.intersects_location([[x, sg*26.0, 0.0]], [[0, 0, 1.0]], multiple_hits=True)
            zz = sorted(z for z in L_[:, 2] if z > 20.0); top.append(zz[0] + 0.5)
        cs_pan = m3d.CrossSection([_ensure_ccw([(FEN_X0, FEN_Z0), (FEN_XE, FEN_Z0)] + list(reversed(list(zip(xs, top)))))])
        wv = np.vstack([m.vertices for m in FW if (m.centroid[1] > 0) == (sg > 0)] +
                       [q.vertices for q in fe_all if (q.centroid[1] > 0) == (sg > 0)])
        w_near = wv[sg*wv[:, 1] <= _wall_y(wv[:, 2]) + 1.30 + FEN_CLR + 1.2]
        if len(w_near) < 10: w_near = wv
        xz = w_near[:, [0, 2]]; hull2 = ConvexHull(xz)
        cs_w = m3d.CrossSection([_ensure_ccw(xz[hull2.vertices].tolist())])
        cs_cut_in  = cs_w.offset(FEN_CLR + 0.35, m3d.JoinType.Round, circular_segments=24)
        cs_cut_mid = cs_w.offset(FEN_CLR + 0.35 - 0.05, m3d.JoinType.Round, circular_segments=24)
        cs_cut_out = cs_w.offset(FEN_CLR + 0.35 + 0.45, m3d.JoinType.Round, circular_segments=24)
        cs_front_trim = m3d.CrossSection([_ensure_ccw([(44.5, 0.0), (55.0, 0.0), (55.0, 23.0), (44.5, 23.0)])])
        cs_slab = (cs_pan - cs_cut_in - cs_front_trim)
        cs_slab = cs_slab.offset(-0.9, m3d.JoinType.Round, circular_segments=16).offset(0.9, m3d.JoinType.Round, circular_segments=16)
        mf_slab = m3d.Manifold.extrude(cs_slab, 1.55 + FEN_IN)
        p_in  = np.asarray(cs_cut_mid.to_polygons()[0], float)
        p_out = np.asarray(cs_cut_out.to_polygons()[0], float)
        ch_cut = _fix_vol(trimesh.convex.convex_hull(np.vstack([
            np.c_[p_in,  np.full(len(p_in),  1.10 + FEN_IN)],
            np.c_[p_out, np.full(len(p_out), 1.60 + FEN_IN)],
            np.c_[p_out, np.full(len(p_out), 2.10 + FEN_IN)],
        ])), f"fch_{sg}")
        mf_ch = m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(ch_cut.vertices, np.float32), tri_verts=np.asarray(ch_cut.faces, np.uint32)))
        me_pan = (mf_slab - mf_ch).to_mesh()
        pan_uv = trimesh.Trimesh(np.asarray(me_pan.vert_properties)[:, :3].astype(float), np.asarray(me_pan.tri_verts), process=False)
        pan = _map_to_side(pan_uv, lambda z: _wall_y(z) - 0.35 - FEN_IN, sg, f"fender_{sg}")
        pan = sorted(pan.split(only_watertight=False), key=lambda m_: -abs(m_.volume))[0]
        print("  clean fender %s: vol %.0f mm3, X %.1f..%.1f Z %.1f..%.1f" % ("L" if sg > 0 else "R", pan.volume, pan.bounds[0][0], pan.bounds[1][0], pan.bounds[0][2], pan.bounds[1][2]))
        pans.append(pan)
        # local filler: closes the 0.1-0.3 mm slot between the pan and the stock arch return flange (inboard face = stock flange face)
        pans.append(box(17.6, FEN_FILL_X1, min(sg*FEN_FILL_YIN, sg*25.75), max(sg*FEN_FILL_YIN, sg*25.75), 14.0, FEN_FILL_Z1))
    return pans

def build_side_and_arch_accents():
    """Rear wheel-arch upper lip reinforcement + bed-ledge pad. The rocker bar, beltline crease and front/rear arch eyebrow strips
    were removed on the owner's markup (plain body sides)."""
    adds = []
    for sg in (1, -1):
        # Rear wheel-arch upper lip reinforcement bar (supports raised +/-10 deg cross-axle roll arch opening at Z=26.6!)
        arch_cap_pts = [
            (-74.6, sg*24.3, 25.8), (-50.6, sg*25.3, 26.4),
            (-74.6, sg*27.2, 25.8), (-50.6, sg*27.7, 26.4),
            (-74.6, sg*24.0, 28.4), (-50.6, sg*25.0, 28.4),
            (-74.6, sg*26.9, 28.4), (-50.6, sg*27.5, 28.4),   # ends at the cut-out X -50.6 (the old sloped front end left a 0.4 mm blade)
        ]
        adds.append(_fix_vol(trimesh.convex.convex_hull(np.array(arch_cap_pts, float)), f"rear_arch_cap_{sg}"))
        # Underside pad for the stock bed ledge inner lip (wedge thins to 0.1-0.5 mm at |Y| 21.6..25): total >= 0.85 mm
        adds.append(_fix_vol(trimesh.convex.convex_hull(np.array([(x, sg*y, z) for (x, y) in ((-71.5, 20.8), (-71.5, 25.8), (-52.3, 20.9), (-52.3, 25.8)) for z in (BED_PAD_Z0, 29.95)], float)), f"bed_ledge_pad_{sg}"))
    return adds

BED_PAD_Z0 = 29.05         # underside of the bed-ledge pad (ledge top Z 29.85..30.0 -> >= 0.8 mm)
ROLL_CUT_YIN = 24.0      # inboard of the bed wall inner face (|Y| 24.4..25.2): the cut removes the full wall, no leftover skin
def build_rear_roll_arch_cuts():
    """Smooth chamfered rear wheel-arch relief clearing +/-10 deg cross-axle rolled rear tyres by >= 1.1 mm (#19)."""
    cuts = []
    for sg in (1, -1):
        # Raise outer rear wheel-arch opening (|Y|>=24.5, X=-76..-51) to Z=26.2..27.4 (sloping cleanly down to Z=20.2 at X=-45.2)
        pts = [
            (-76.5, sg*ROLL_CUT_YIN, 18.0), (-50.8, sg*ROLL_CUT_YIN, 18.0), (-45.2, sg*ROLL_CUT_YIN, 18.0),
            (-76.5, sg*35.0, 18.0), (-50.8, sg*35.0, 18.0), (-45.2, sg*35.0, 18.0),
            (-76.5, sg*ROLL_CUT_YIN, 26.00), (-50.8, sg*ROLL_CUT_YIN, 26.00), (-45.2, sg*ROLL_CUT_YIN, 20.2),
            (-76.5, sg*35.0, 27.60), (-50.8, sg*35.0, 27.60), (-45.2, sg*35.0, 21.0),
        ]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), f"rear_roll_cut_{sg}"))
    return cuts


# ---- owner markup 49.webp: fender cut-outs (side-view lines) ----
FA_CX, FA_CZ = 48.02, 1.5            # front arch centre = front wheel centre (X, Z)
FA_R = float(os.environ.get("FA_R", 21.74))     # 1.21 x tyre radius 18
CUT_YIN = 23.0                        # cuts start inboard of the side wall inner face (|Y| 24.4..26.0)
REAR_CUT_CH = float(os.environ.get("REAR_CUT_CH", 0.25))   # rear: thin bed lip above the cut, smaller chamfer
CUT_CH = 0.5                          # 45 deg outer-edge chamfer, like the existing arch lip
# rear opening: lower edge of the bed side wall, everything below removed (X, Z)
# rear arch = one circle read from the owner's markup 59.png (tyre centre X -68.5 Z 1.5 R 18 and arm pivot X -35 Z 2.4 give 10.0 px/mm)
REAR_ARCH_CX = float(os.environ.get("RA_CX", -50.6))
REAR_ARCH_CZ = float(os.environ.get("RA_CZ", 5.3))
REAR_ARCH_R = float(os.environ.get("RA_R", 21.1))
REAR_ARCH_N = 144                       # 2.5 deg per segment: sagitta 0.02 mm, no visible facets
def _y_out(z): return 26.1 + 0.09*(z - 7.0)       # outer face of the side wall (shell Y at Z)
def _ring(pts_xz, sg, y_fn, d_off=0.0, d_y=0.0):
    return [(x, sg*(y_fn(z) + d_y), z) for (x, z) in pts_xz]
def _cut_hull(poly_xz, sg, CUT_CH=CUT_CH):
    """convex XZ polygon -> prism along Y from CUT_YIN to beyond the outer face, with a 45 deg chamfer on the outer edge"""
    P = np.asarray(poly_xz, float)
    cs0 = m3d.CrossSection([_ensure_ccw(P.tolist())])
    p_c = np.asarray(cs0.offset(CUT_CH, m3d.JoinType.Miter, miter_limit=4.0).to_polygons()[0], float)
    p_f = np.asarray(cs0.offset(CUT_CH + 1.5, m3d.JoinType.Miter, miter_limit=4.0).to_polygons()[0], float)
    pts = []
    for (poly, dy, yfn) in ((P, 0.0, None), (P, -CUT_CH, _y_out), (p_c, 0.0, _y_out), (p_f, 1.5, _y_out)):
        for (x, z) in poly:
            y = CUT_YIN if yfn is None else yfn(z) + dy
            pts.append((x, sg*y, z))
    return pts
def build_fender_cutouts():
    cuts = []
    for sg in (1, -1):
        th = np.linspace(0.0, 2*np.pi, 73)[:-1]
        circ = np.c_[FA_CX + FA_R*np.cos(th), FA_CZ + FA_R*np.sin(th)]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(_cut_hull(circ, sg), float)), f"front_arch_cut_{sg}"))
        th = np.linspace(0.0, 2*np.pi, REAR_ARCH_N + 1)[:-1]
        rcirc = np.c_[REAR_ARCH_CX + REAR_ARCH_R*np.cos(th), REAR_ARCH_CZ + REAR_ARCH_R*np.sin(th)]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(_cut_hull(rcirc, sg, REAR_CUT_CH), float)), f"rear_arch_cut_{sg}"))
    return cuts


# ---- shell defect fixes (owner: "fix all shell defects"): hairline blades, a skirt-edge jog and a 0.3 mm roof-corner post ----
DEFECT_FIX_ON = os.environ.get("DEFECT_FIX", "1") == "1"
def build_defect_fixes():
    adds, cuts = [], []
    if not DEFECT_FIX_ON: return adds, cuts
    for sg in (1, -1):
        yy = lambda a, b: (min(sg*a, sg*b), max(sg*a, sg*b))
        # blade 0.03 mm thick sticking out of the bed-ledge top at the fin base (X -58..-52.8, |Y| 21.5..22.15, Z 29.96..30.1)
        y0, y1 = yy(21.5, 22.15); cuts.append(box(-58.0, -52.8, y0, y1, 29.96, 30.10))
        # blade under the roof-rail brace (paper fin) beyond the rail face |Y| 22.85
        y0, y1 = yy(22.85, 23.3); cuts.append(box(-36.9, -35.8, y0, y1, 38.1, 38.4))
        # 0.3 mm tab below the skirt chamfer line near X -25.3 (line Z = 8.24 - 0.43*(X+25))
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array([(x, sg*y, z) for (x, z) in ((-25.6, 8.50), (-24.9, 8.20), (-24.9, 7.4), (-25.6, 7.4)) for y in (24.5, 27.0)], float)), f"skirt_jog_{sg}"))
        # roof corner post X -21.8..-21.5 is 0.3 mm thick: thicken it forward to 1.65 mm (inside the cab, hangs under the roof)
        y0, y1 = yy(20.9, 22.0); adds.append(box(-21.55, -20.15, y0, y1, 42.2, 44.5))
        # rear bed-ledge pad under the fin: straighten the stepped outer edge (|Y| 24.9/24.7/24.34 -> one line 24.9), fill the inner notch/slit and the tail stairs
        y0, y1 = yy(22.19, 24.9); adds.append(box(-66.6, -51.5, y0, y1, 29.9, 30.4))
        y0, y1 = yy(21.5, 22.3); adds.append(box(-58.0, -56.7, y0, y1, 29.9, 30.4))
        y0, y1 = yy(21.1, 22.3); adds.append(box(-67.0, -66.37, y0, y1, 29.9, 30.4))
        # tail end face: fill the stair-stepped steps so the rear face is one flat plane at X -67 (|Y| 18.9..27.2, Z 26.9..30.4)
        y0, y1 = yy(18.9, 27.2); adds.append(box(-67.0, -66.3, y0, y1, 26.9, 30.4))
    # roof rear end: drop the 0.3 mm end chamfer (knife-thin bevel facets X -21.8..-21.5) by squaring the roof plate off at X -21.5
    cuts.append(box(-23.0, -21.5, -21.05, 21.05, 44.3, 47.0))
    return adds, cuts
_df_adds, _df_cuts = build_defect_fixes()


# ---- tail fins (stock Tacoma rear fins, refitted on the shortened bed) ----
# stock fin (measured on the old shell): X -63.6..-78.5, top Z 39.0 flat over X -70.5..-78.5, leading edge raked from (-63.6, 31.1) up to (-70.5, 39.0),
# slab 2.3..2.7 mm thick leaning outward with height. Here the whole fin is shifted forward so its rear edge sits on the bed end.
FIN_ON = os.environ.get("FIN_ON", "1") == "1"
FIN_SHIFT = float(os.environ.get("FIN_SHIFT", (TAIL_END_X + 0.6) - (-78.5)))     # +11.1: rear edge at X -67.6, trimmed flat by the tail cut at X -67.0
FIN_BASE_Z = 29.95                      # embedded 0.05 mm into the bed ledge (top Z 30.0)
FIN_CH = 0.4
def _fin_yi(z): return 22.33 + 0.4587*(z - 31.0)
def _fin_yo(z): return 25.03 + 0.4093*(z - 31.0)
def build_tail_fins():
    if not FIN_ON: return []
    sh = FIN_SHIFT
    P = [(-63.6 + sh, FIN_BASE_Z), (-63.6 + sh, 31.1), (-70.5 + sh, 39.0), (TAIL_END_X - 0.6, 39.0), (TAIL_END_X - 0.6, FIN_BASE_Z)]
    cs0 = m3d.CrossSection([_ensure_ccw(P)])
    Pin = np.asarray(cs0.offset(-FIN_CH, m3d.JoinType.Miter, miter_limit=4.0).to_polygons()[0], float)
    adds = []
    for sg in (1, -1):
        pts = []
        for (x, z) in P:
            pts += [(x, sg*(_fin_yi(z) + FIN_CH), z), (x, sg*(_fin_yo(z) - FIN_CH), z)]
        for (x, z) in Pin:
            pts += [(x, sg*_fin_yi(z), z), (x, sg*_fin_yo(z), z)]
        adds.append(_fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), f"tail_fin_{sg}"))
    return adds

GRILLE_XB = float(os.environ.get("GRILLE_XB", 61.7))     # pocket floor X: grille 3.3 mm / headlight buckets ~3 mm behind the fascia face
def build_front_grille_fascia():
    """Recessed Tacoma/Yeti front grille surround + twin headlight bucket pockets (AX31116 #3/#9/#11/#12, wall >= 1.0 mm)."""
    pts_outer = [
        (60.5, -20.5, 27.6), (60.5, 20.5, 27.6), (60.5, -20.5, 25.4), (60.5, 20.5, 25.4),
        (60.5, -12.0, 28.3), (60.5, 12.0, 28.3),
        (65.0, -10.2, 28.5), (65.0, 10.2, 28.5), (64.0, -20.6, 27.5), (64.0, 20.6, 27.5),
        (64.4, -10.0, 23.2), (64.4, 10.0, 23.2), (63.4, -18.2, 23.8), (63.4, 18.2, 23.8),
        (61.4, -10.0, 23.2), (61.4, 10.0, 23.2), (61.4, -17.4, 23.8), (61.4, 17.4, 23.8),
    ]
    fascia = _fix_vol(trimesh.convex.convex_hull(np.array(pts_outer, float)), "fascia_outer")
    def pocket_prism(pf, pb, xf, xb):
        return _fix_vol(trimesh.convex.convex_hull(np.array([(xf, y, z) for (y, z) in pf] + [(xb, y, z) for (y, z) in pb], float)), "pocket")
    # Center grille surround (4.0 x 16.8 x 2.0 mm recess, >= 1.1 mm solid backing wall)
    g_cut = pocket_prism([(-8.4, 27.8), (8.4, 27.8), (7.3, 23.8), (-7.3, 23.8)],
                         [(-7.5, 27.1), (7.5, 27.1), (6.5, 24.5), (-6.5, 24.5)], 66.0, GRILLE_XB)
    # Twin headlight bucket pockets (3.6 x 8.0 x 1.8 mm recess + Ø2.3 LED projector dish, >= 0.95 mm backing wall)
    hl_cuts = []
    for sg in (1, -1):
        hl_cuts.append(pocket_prism([(sg*10.1, 27.6), (sg*18.4, 27.1), (sg*17.0, 24.2), (sg*9.6, 24.0)],
                                    [(sg*10.8, 26.9), (sg*17.2, 26.5), (sg*16.0, 24.8), (sg*10.3, 24.6)], 66.0, GRILLE_XB))
        hl_cuts.append(cyl("x", [GRILLE_XB, sg*13.8, 25.8], 1.15, 0.60, 24))
    # pockets are cut from the fascia AND the base nose wall (after the union), slats are re-added last
    slat1 = box(GRILLE_XB - 0.3, 63.45, -8.2, 8.2, 26.05, 26.85)
    slat2 = box(GRILLE_XB - 0.3, 63.45, -7.8, 7.8, 24.65, 25.45)
    cbar  = box(GRILLE_XB - 0.3, 63.55, -1.0, 1.0, 23.9, 27.5)
    return fascia, [g_cut] + hl_cuts, [slat1, slat2, cbar]

def _hood_z_surface(base):
    xs, ys = np.linspace(24, 62.5, 20), np.linspace(0, 21.5, 16)
    pts, vals = [], []
    for x in xs:
        for y in ys:
            if 27.0 <= x <= 39.5 and y <= 6.2: continue
            locs, _, _ = base.ray.intersects_location([[x, y, 0.0]], [[0, 0, 1.0]], multiple_hits=True)
            zs = sorted([z for z in locs[:, 2] if z > 25.0]) if len(locs) else []
            if zs: pts.append([x, y]); vals.append(zs[-1])
    interp = LinearNDInterpolator(np.array(pts), np.array(vals), fill_value=np.nan)
    P, V = np.array(pts), np.array(vals)
    coef, *_ = np.linalg.lstsq(np.c_[np.ones(len(P)), P[:, 0], P[:, 1], P[:, 0]**2, P[:, 1]**2], V, rcond=None)
    def zfn(x, y):
        x, y = np.asarray(x, float), np.abs(np.asarray(y, float))
        z = interp(x, y); bad = np.isnan(z)
        if np.any(bad):
            xb, yb = x[bad], y[bad]
            z[bad] = coef[0] + coef[1]*xb + coef[2]*yb + coef[3]*xb**2 + coef[4]*yb**2
        return z
    return zfn

def build_hood_accents(base):
    """Twin louvered hood pad accents (AX31116 #13/#14, p41-42). The hood edge rails and cowl bar were removed on the owner's markup."""
    zfn = _hood_z_surface(base)
    def map_h(m_xy, lbl):
        q = m_xy.vertices.copy(); q[:, 2] += zfn(q[:, 0], q[:, 1])
        return _fix_vol(trimesh.Trimesh(q, m_xy.faces.copy(), process=False), lbl)
    adds = []
    y_in_front = max(MAG_Y + 3.4, 10.4); y_in_rear = max(y_in_front + 1.8, 12.2)
    for sg in (1, -1):
        pad_poly = [(41.5, sg*(y_in_rear + 2.6)), (53.0, sg*(y_in_front + 2.6)), (52.2, sg*19.0), (41.5, sg*20.2)]
        pad = map_h(_bevel_convex(pad_poly, -0.60, 0.55, 1.00, 0.45, f"hpad_{sg}"), f"hpad_m_{sg}")
        louvers = []
        for lx in (44.2, 46.8, 49.4):
            y0, y1 = min(sg*(y_in_front + 3.4), sg*18.2), max(sg*(y_in_front + 3.4), sg*18.2)
            zc = float(zfn(lx, sg*16.2))
            louvers.append(box(lx - 0.55, lx + 0.55, y0, y1, zc + 0.55, zc + 2.5))
        adds.append(_fix_vol(D(pad, louvers), f"hpad_louver_{sg}"))
    return adds

def _teardrop_bore_y(cx, cz, r, ym, length, n=28):
    """Horizontal Y-axis bore with 45-deg top teardrop crown relief (#16/#17) to prevent overhang droop."""
    th = np.linspace(0, 2*math.pi, n, endpoint=False)
    ring = [(cx + r*math.cos(a), cz + r*math.sin(a)) for a in th]
    # Gentle 45-deg crown apex at cz + 1.18*r (prevents top-arch sag while keeping circle-fit coaxiality <= 0.04 mm)
    ring.append((cx, cz + 1.18*r))
    y0, y1 = ym - 0.5*length, ym + 0.5*length
    pts = [(x, y, z) for (x, z) in ring for y in (y0, y1)]
    return _fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), "teardrop_y")

def build_rear_hinge_and_tail():
    """Twin rear tilt-hinge tabs matching Cage pivot ears (X=REAR_HINGE_X, Z=29.0, 45-deg teardrop bore, >=1.0 mm wall fillet) + open-rear tail framing."""
    adds, cuts, pre_cuts = [], [], []
    hx, hz, hy_in, hy_out, boss_r = REAR_HINGE_X, REAR_HINGE_Z, REAR_HINGE_YIN, REAR_HINGE_YOUT, REAR_HINGE_BOSS_R
    gx0, gx1 = hx + 1.9, hx + 7.3      # gusset stations (same offsets from the hinge axis as on the long tail)
    for sg in (1, -1):
        y0, y1 = min(sg*hy_in, sg*hy_out), max(sg*hy_in, sg*hy_out)
        ym, yt = 0.5*(y0 + y1), abs(y1 - y0)
        boss = cyl("y", [hx, ym, hz], boss_r, yt, 32)
        # Tapered structural bridge + >= 1.2 mm inner-wall fillet gusset rooting into shell rear inner wall (|Y|=21.1..24.2)
        bridge_pts = [
            (hx, sg*hy_in, hz + boss_r - 0.1), (hx, sg*hy_in, hz - boss_r + 0.1),
            (hx + 2.0, sg*hy_in, hz + boss_r - 0.1), (hx + 2.0, sg*hy_in, hz - boss_r + 0.1),
            (hx, sg*hy_out, hz + boss_r), (hx, sg*hy_out, hz - boss_r),
            # Inner-wall 45-deg fillet gusset at |Y|=19.8..21.1 -> widens footprint on inner bed wall
            (gx0, sg*19.8, 31.6), (gx0, sg*19.8, 26.4),
            (gx0, sg*21.1, 32.4), (gx0, sg*21.1, 25.6),
            (gx1, sg*21.1, 31.8), (gx1, sg*21.1, 27.6),
            (gx0, sg*24.7, 32.0), (gx0, sg*24.7, 26.0),
            (gx1, sg*24.7, 31.6), (gx1, sg*24.7, 27.6),
        ]
        adds.extend([boss, _fix_vol(trimesh.convex.convex_hull(np.array(bridge_pts, float)), f"hinge_bridge_{sg}")])
        cuts.append(_teardrop_bore_y(hx, hz, REAR_HINGE_BORE_D/2.0, ym + sg*0.85, yt + 3.7, 28))      # through the bridge root (|Y| 17.9..24.0, blind in the screw block)
        # stock fin root block and fin stub ahead of the new tail end (X -64..TAIL_END_X, outboard of the rail): removed down to the ledge
        pre_cuts.append(box(TAIL_END_X - 1.0, -63.4, min(sg*22.93, sg*33.5), max(sg*22.93, sg*33.5), REAR_LEDGE_Z - 0.02, 46.0))
        # outboard screw block: solid around the M2 pilot where it runs on past the lug through the bed ledge (tip at |Y| 23.7), >= 0.9 mm end wall
        adds.append(box(hx - boss_r, hx + boss_r, min(sg*21.2, sg*HINGE_BLOCK_YOUT), max(sg*21.2, sg*HINGE_BLOCK_YOUT), 27.0, 31.4))
    # shortened bed: everything behind TAIL_END_X is cut off (rear fins, old tail frame); the cut closes the ledge, wall and rail ends flat
    cuts.append(box(-200.0, TAIL_END_X, -60.0, 60.0, -50.0, 80.0))
    return adds, cuts, pre_cuts

def build_thin_trim_cuts():
    """Trims that remove unprintable knife edges of the stock mesh (inside / under the shell, no styling change)."""
    cuts = []
    for sg in (1, -1):
        # 45-deg chamfer along the acute lower-inboard edge of the under-hood side rib (edge tip Y 24.16 -> 23.07, Z 27.25 -> 27.80 over X 26..35)
        pts = []
        for x in (25.0, 36.0):
            ty, tz = 24.16 - 0.1215*(x - 26.0), 27.25 + 0.061*(x - 26.0)
            pts += [(x, sg*(ty + dy), tz + dz) for dy, dz in ((-0.6, -0.5), (1.3, -0.5), (1.3, -0.46), (-0.16, 1.0), (-0.6, 1.0))]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), f"hood_rib_chamfer_{sg}"))
    # stock windshield-header lip: a 0.1..0.2 mm thick blade ahead of the roof front edge (X 4.2..6.9, down to Z 39.7 in the corners) -> removed
    cuts.append(box(4.15, 7.1, -20.55, 20.55, 39.5, 43.3))
    for sg in (1, -1):
        # under-hood side rib ahead of the strut (X 44..54): hanging knife tip 0.3 mm -> cut back to where the rib is >= 1.2 mm wide (follows the hood slope)
        pts = [(x, sg*y, HOOD_RIB_ZCUT - 0.105*(x - 48.2) - dz) for x in (44.0, 54.0) for y in (20.0, 22.9) for dz in (0.0, 1.6)]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), f"hood_rib_tip_{sg}"))
        # stock roof-rail brace: front tip tapers to zero thickness (X -33.8..-31.6) -> blunt end
        # rear-arch rim: one fresh flat underside 0.05 mm above the stock rim edge, through the rim filler (which hangs 0.3 mm lower)
        pts = [(x, sg*y, 16.63 - 0.4225*(x + 45.0) + 0.05 - dz) for x in (-48.4, -25.5) for y in (24.6, 27.5) for dz in (0.0, 0.9)]
        cuts.append(_fix_vol(trimesh.convex.convex_hull(np.array(pts, float)), f"arch_rim_trim_{sg}"))
        cuts.append(box(-37.5, ROOF_BRACE_XCUT, min(sg*22.70, sg*24.9), max(sg*22.70, sg*24.9), 38.3, 40.4))      # also takes the 0.1 mm ledge at Z 38.5 and skims 0.08 mm off the wall face
    return cuts

FEN_PLUG_Z0, FEN_PLUG_Z1 = 25.6, 27.0      # fender-end plug: bottom 0.3 mm clear below the sloping hood underside (no near-coplanar faces), top inside the solid hood edge
HOOD_RIB_ZCUT = 27.35      # cut height of the under-hood rib tip at X 48.2
ROOF_BRACE_XCUT = -31.6    # roof-rail brace tip removed ahead of this X
def build_thin_fills():
    """Inboard / underside fillers that bury stock fins and close near-coincident slits (no outer styling change)."""
    adds = []
    for sg in (1, -1):
        # rear-arch leading rim: fill the stock inner chamfer (knife edge 0.16 mm at the rim) -> full-width flat rim (underside trimmed in build_thin_trim_cuts)
        tris = []
        for x in (-47.6, -37.0, -25.0):
            zb = 16.63 - 0.4225*(x + 45.0); ybi = 26.53 - 0.0678*(x + 45.0); yct = 25.55 - 0.0261*(x + 45.0); zct = 17.90 - 0.4767*(x + 45.0)
            tris.append([(x, sg*(yct - 0.064*(zct - zb + 0.3)), zb - 0.3), (x, sg*(ybi + 0.08), zb - 0.3), (x, sg*(ybi + 0.08), zb + 0.02), (x, sg*yct, zct + 0.10), (x, sg*(yct + 0.10), zct + 0.10)])
        for a, b in zip(tris[:-1], tris[1:]): adds.append(_fix_vol(trimesh.convex.convex_hull(np.array(a + b, float)), f"arch_rim_fill_{sg}"))
        # A-pillar base: block behind the stock 0.15..0.3 mm inner flange (joins the fender filler)
        adds.append(box(FEN_FILL_X1 - 0.05, 22.0, min(sg*24.3, sg*25.9), max(sg*24.3, sg*25.9), 24.3, 27.1))
        # fender inner fin at X 17.3..17.45 (0.16 mm sheet, 0.03 mm slit to the filler): buried
        adds.append(box(16.8, 17.65, min(sg*24.2, sg*25.1), max(sg*24.2, sg*25.1), 12.3, 19.2))
        # bed-ledge front end: 0.02..0.07 mm stock skin over the pad -> filled through
        adds.append(box(-54.4, -52.3, min(sg*24.2, sg*25.85), max(sg*24.2, sg*25.85), 29.2, 30.06))
        adds.append(box(-54.4, -52.3, min(sg*25.75, sg*27.0), max(sg*25.75, sg*27.0), 29.8, 30.06))
    for sg in (1, -1):
        # pocket under the hood edge where the front fender wall ends (X 49.5) short of the hood side bevel: closed with a faceted gusset
        pl = [(49.2, 26.0), (49.2, 27.40), (49.65, 27.40), (50.55, 26.40), (50.55, 26.0)]      # blunt 0.4 mm nose instead of a knife tip
        adds.append(_fix_vol(trimesh.convex.convex_hull(np.array([(x, sg*y, z) for (x, y) in pl for z in (FEN_PLUG_Z0, FEN_PLUG_Z1)], float)), f"fender_plug_{sg}"))
    # hood centre step: 0.02 mm air slit under a 0.09 mm skin (X 37.1..37.45, |Y| < 3.8) -> filled
    adds.append(box(37.0, 37.6, -4.0, 4.0, 31.4, 31.79))
    return adds

def _mirror_left_half(m):
    """Exact L/R symmetry: the left half (Y >= 0) is the master, the right half is its mirror (the stock mesh is not symmetric)."""
    h = D(m, [box(-200.0, 200.0, -200.0, 0.0, -200.0, 200.0)])
    v = h.vertices.copy(); v[np.abs(v[:, 1]) < 1e-6, 1] = 0.0
    f = h.faces[~np.all(v[h.faces][:, :, 1] == 0.0, axis=1)]      # drop the cap on the centre plane
    r = trimesh.Trimesh(v*np.array([1.0, -1.0, 1.0]), f[:, ::-1], process=False)
    o = trimesh.util.concatenate([trimesh.Trimesh(v, f, process=False), r]); o.merge_vertices()
    ok = o.is_watertight and len(o.split(only_watertight=False)) == 1
    print("  mirrored left half: watertight %s vol %.0f -> %.0f%s" % (o.is_watertight, m.volume, o.volume, "" if ok else "  (REJECTED, kept unsymmetrised)"))
    return o if ok else m

def build_front_magnet_mounts(base_shell):
    """Two magnet bosses on the hood UNDERSIDE (no hood holes): blind pocket for a D3.0 x 2.0 neodymium disc, opening downward.
    The disc seats against the pocket ceiling; its lower face sits MAG_GAP above the chassis magnet seat top (MAG_SEAT_Z)."""
    adds, cuts = [], []
    pr = MAG_POCKET_D/2.0; boss_r = pr + MAG_WALL
    th_arr = np.linspace(0, 2*math.pi, 48, endpoint=False)
    z_ceil = MAG_SEAT_Z + MAG_GAP + MAG_T; z_mouth = z_ceil - MAG_POCKET_DEPTH
    for sg in (1, -1):
        py = sg*MAG_Y
        locs, _, _ = base_shell.ray.intersects_location([[MAG_X, py, 0.0]], [[0, 0, 1.0]], multiple_hits=True)
        zs = sorted([z for z in locs[:, 2] if z > 24.0]); z_in, z_out = zs[-2], zs[-1]
        skin = z_out - z_ceil; z_top_embed = z_in + 0.80
        b_cyl = cyl("z", [MAG_X, py, 0.5*(z_mouth + z_top_embed)], boss_r, z_top_embed - z_mouth, 48)
        zf = max(z_mouth + 0.3, z_in - 1.0)      # 45-deg root fillet cone under the hood skin
        fillet = _fix_vol(trimesh.convex.convex_hull(np.vstack([
            np.c_[MAG_X + boss_r*np.cos(th_arr),                       py + boss_r*np.sin(th_arr),                       np.full(48, zf)],
            np.c_[MAG_X + (boss_r + z_top_embed - zf)*np.cos(th_arr), py + (boss_r + z_top_embed - zf)*np.sin(th_arr), np.full(48, z_top_embed)],
        ])), f"mag_fillet_{sg}")
        adds.extend([b_cyl, fillet])
        pad_h = z_ceil + MAG_SKIN - z_out      # > 0: stock hood skin cannot cover a 2 mm disc -> closed, bevelled hood pad (Yeti-style twin hood pads)
        if pad_h > 0:
            z_pt = z_ceil + MAG_SKIN; r_t = pr + 0.90; r_b = r_t + 0.85; z_pb = z_out - 0.45
            adds.append(_fix_vol(trimesh.convex.convex_hull(np.vstack([
                np.c_[MAG_X + r_b*np.cos(th_arr), py + r_b*np.sin(th_arr), np.full(48, z_pb)],
                np.c_[MAG_X + r_b*np.cos(th_arr), py + r_b*np.sin(th_arr), np.full(48, z_pt - 0.75)],
                np.c_[MAG_X + r_t*np.cos(th_arr), py + r_t*np.sin(th_arr), np.full(48, z_pt)],
            ])), f"mag_pad_{sg}"))
            skin = MAG_SKIN; print("  hood pad %s: raised %.2f mm above the stock hood (top Z %.2f, D %.2f base / %.2f top)" % ("L" if sg > 0 else "R", pad_h, z_pt, 2*r_b, 2*r_t))
        # underside relief over the chassis magnet seat (D4.8 boss, footprint +-2.4): 0.6 mm around, 0.3 mm above the seat top;
        # also clears the stock under-hood block whose front face (X 54.9) would otherwise touch the seat
        cuts.append(box(MAG_X - 3.0, MAG_X + 2.8, min(sg*(MAG_Y - 3.0), sg*(MAG_Y + 3.0)), max(sg*(MAG_Y - 3.0), sg*(MAG_Y + 3.0)), 20.0, z_mouth))
        cuts.append(cyl("z", [MAG_X, py, 0.5*(z_mouth - 0.5 + z_ceil)], pr, z_ceil - z_mouth + 0.5, 48))
        print("  front magnet boss %s at (%.2f, %+.2f): hood Z %.2f..%.2f, boss Z %.2f..%.2f OD %.2f, pocket D %.2f Z %.2f..%.2f, skin to outer hood %.2f mm%s" % ("L" if sg > 0 else "R", MAG_X, py, z_in, z_out, z_mouth, z_top_embed, 2*boss_r, MAG_POCKET_D, z_mouth, z_ceil, skin, "" if skin >= 0.6 else "   <-- SKIN < 0.6: lower MAG_SEAT_Z"))
    return adds, cuts

_yeti_adds = []
if FENDER: _yeti_adds += build_clean_fenders(RAW_BASE)
_yeti_adds += build_side_and_arch_accents()
_fascia, _grille_cuts, _grille_slats = build_front_grille_fascia(); _yeti_adds.append(_fascia)
_yeti_adds += build_hood_accents(RAW_BASE)
_yeti_adds += build_thin_fills()
_yeti_adds += build_tail_fins()
_yeti_adds += _df_adds
_rear_adds, _rear_cuts, _rear_pre_cuts = build_rear_hinge_and_tail()
_post_adds, _post_cuts = build_front_magnet_mounts(RAW_BASE)
_roll_cuts = build_rear_roll_arch_cuts()
_b0 = BASE.volume
BASE = D(BASE, _rear_pre_cuts)      # stock fin roots go before the hinge block is added
BASE = U([BASE] + _yeti_adds + _rear_adds + _post_adds)
BASE = D(BASE, _rear_cuts + _post_cuts + _roll_cuts + _grille_cuts + build_thin_trim_cuts() + build_fender_cutouts() + _df_cuts)
BASE = U([BASE] + _grille_slats)
BASE = sorted(BASE.split(only_watertight=False), key=lambda q: -abs(q.volume))[0]
print("  base shell + Yeti details & mounts: vol %.0f -> %.0f, watertight %s shells %d" % (_b0, BASE.volume, BASE.is_watertight, len(BASE.split(only_watertight=False))))

# ---- which static keepouts touch the base shell ----
use = []
for n, m in STATIC.items():
    v = vol(BASE, m); print("  base shell vs %-26s %.3f mm3" % (n, v))
    _o = CollisionManager(); _o.add_object('m', m); _cs = CollisionManager(); _cs.add_object('s', BASE)
    if os.environ.get("STATIC_CUTS") and (v > 0.01 or _cs.min_distance_other(_o) < CLR + 0.5): use.append(m)
print("  base shell vs gearbox+motor pocket: %.1f mm3" % vol(BASE, pocket(0.0)))

LEVEL_WALL = os.environ.get("LEVEL_WALL", "1") != "0"
def build(c):
    cuts = [dil(m, c + 0.9) for m in use] + ([pocket(0.4 + (c - 0.3)*0.5)] if use else [])
    ARCH_X = float(os.environ.get("ARCH_X", 0.25))
    cuts += [dil(m, c + ARCH_X) for m in sweep] + [dil(w, c + ARCH_X) for w in RELIEF_FW]
    out = D(BASE, [U(cuts)]) if (cuts and os.environ.get("ARCH_CUTS")) else BASE.copy()
    for _pass in range(3 if os.environ.get("PATCH") else 0):
        pts = trimesh.sample.sample_surface(out, 150000)[0]; balls = []
        for T in list(use) + list(sweep) + list(RELIEF_FW):
            lo, hi = T.bounds[0] - CLR - 0.2, T.bounds[1] + CLR + 0.2; sel = pts[np.all((pts > lo) & (pts < hi), axis=1)]
            if len(sel) == 0: continue
            cp_, dd_, _t = trimesh.proximity.closest_point(T, sel); bad = dd_ < CLR + 0.09
            if bad.any():
                q = np.unique(np.round(cp_[bad]/0.4).astype(int), axis=0)*0.4
                for c_ in q: b_ = trimesh.creation.icosphere(subdivisions=2, radius=CLR + 0.3); b_.apply_translation(c_); balls.append(b_)
        print("    patch pass %d: %d spheres" % (_pass, len(balls)))
        if not balls: break
        out = D(out, [U(balls)])
    sh = sorted(out.split(only_watertight=False), key=lambda q: -abs(q.volume)); big = sh[0]
    big = _mirror_left_half(_simplify_checked(_clean_checked(big)))      # after the cleanup: the cleanup itself is not L/R symmetric
    big = owner_fixes(big)
    if LEVEL_WALL:      # uniform 1.4 mm wall: inner surface = offset of every outer facet (scripts/shell_wall.py). On the cleaned, mirrored shell:
        import shell_wall      # sliver faces of the raw mesh give wrong offsets, and the Python cleanup above does not finish on the levelled mesh. level_wall returns a clean, L/R mirrored shell
        big, _ = shell_wall.level_wall(big, bodies={k: STATIC[k] for k in ("Cage", "Chassis_Tub", "IFS_Bulkhead_v3 (magnet seats)") if k in STATIC})
    print("  relief clr %.3f: vol %.0f -> %.0f, shells %d, faces %d, deg %d" % (c, BASE.volume, big.volume, len(sh), len(big.faces), int((big.area_faces < 1e-6).sum())))
    return big

# Owner markups from the SOMA session of 2026-11-21/22. They were one-off edits of the STL (lip-end fairings, rear ledge trims, filled holes, removed slivers,
# single-face fixes), lost on every rebuild. scripts/data/shell_soma/ holds soma.stl (the shell that session saved, from output/model.soma) and base.stl (the
# unlevelled builder shell it was edited from: LEVEL_WALL=0 OWNER_FIXES=0). What the session added (soma - base) and removed (base - soma) is kept by starting
# from soma.stl and applying to it whatever the builder has changed since base.stl, before the wall is levelled. scripts/shell_soma_patch.py lists the pieces.
OWNER_FIXES = os.environ.get("OWNER_FIXES", "1") != "0"; OWNER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "shell_soma")
def _to_m(t): return m3d.Manifold(m3d.Mesh(np.asarray(t.vertices, np.float32), np.asarray(t.faces, np.uint32)))
def owner_fixes(m):
    if not OWNER_FIXES: return m
    soma, base = (_to_m(trimesh.load(os.path.join(OWNER_DIR, f))) for f in ("soma.stl", "base.stl"))
    mm = _to_m(m); new, gone = [d for d in (mm - base).decompose() if d.volume() > 0.02], [d for d in (base - mm).decompose() if d.volume() > 0.02]      # what the builder changed since base.stl (nothing, if the builder is unchanged: the result is then soma.stl itself)
    r_ = soma
    for d in new: r_ = r_ + d
    for d in gone: r_ = r_ - d
    add, cut = soma - base, base - soma; q = r_.to_mesh()
    r = trimesh.Trimesh(np.asarray(q.vert_properties)[:, :3], np.asarray(q.tri_verts)); sh = sorted(r.split(only_watertight=False), key=lambda t: -abs(t.volume))
    print("  owner fixes (SOMA session patch): session added %.1f / removed %.1f mm3; builder changes since: %d added, %d removed pieces; volume %.1f -> %.1f, watertight %s, shells %d" % (add.volume(), cut.volume(), len(new), len(gone), m.volume, r.volume, r.is_watertight, len(sh)))
    if any(abs(t.volume) > 0.5 for t in sh[1:]): raise RuntimeError("owner_fixes cut a piece of %.1f mm3 loose" % max(abs(t.volume) for t in sh[1:]))
    return sh[0]
OUTF = os.environ.get("OUT_STL", V4 + "Tacoma_Trophy_Body_Shell.stl"); final = None
for c in (CLR, CLR + 0.033, CLR + 0.071, CLR + 0.12):
    o = build(c); o.export(OUTF); q = trimesh.load(OUTF)
    _sh = sorted(q.split(only_watertight=False), key=lambda t: -abs(t.volume))
    if len(_sh) > 1 and all(abs(t.volume) < 0.5 for t in _sh[1:]): _sh[0].export(OUTF); q = trimesh.load(OUTF); print("    dropped %d zero-volume sliver shells after the STL roundtrip" % (len(_sh) - 1))
    print("    roundtrip: watertight %s open %d shells %d deg %d" % (q.is_watertight, open_edges(q), len(q.split(only_watertight=False)), int((q.area_faces < 1e-6).sum())))
    if q.is_watertight and open_edges(q) == 0 and len(q.split(only_watertight=False)) == 1: final = q; break
print("SHELL: watertight %s open edges %d shells %d vol %.0f deg_faces %d bounds %s" % (q.is_watertight, open_edges(q), len(q.split(only_watertight=False)), q.volume, int((q.area_faces < 1e-6).sum()), q.bounds.round(1).tolist()))
# Route 2 (owner 2026-11-23): the shell that ships is built from planes by scripts/24_shell_planes.py (exact 1.4 mm wall per skin facet of the owner-approved
# scripts/data/shell_soma/soma.stl). The shell built above is kept only so the checks below have the same inputs; SHELL_PLANES=0 keeps it as the output.
# Owner 2026-11-24: the kit ships the SCT Cyber Trophy variant (scripts/25_shell_variant.py -> sct_shell_variant/trophy_shell.py).
# SHELL_SOURCE=planes ships the plane-built Tacoma (24_shell_planes.py) instead; SHELL_SOURCE=builder (or SHELL_PLANES=0) keeps the shell built above.
SHELL_SOURCE = os.environ.get("SHELL_SOURCE", "builder" if os.environ.get("SHELL_PLANES") == "0" else "variant")
if SHELL_SOURCE in ("variant", "planes"):
    import subprocess
    _scr = {"variant": "25_shell_variant.py", "planes": "24_shell_planes.py"}[SHELL_SOURCE]
    _r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), _scr)], env=dict(os.environ, OUT_STL=os.path.abspath(OUTF)), capture_output=True, text=True)
    print("  shipped shell (%s): exit %d; %s" % (_scr, _r.returncode, (_r.stdout.strip().splitlines() or [""])[-1]))
    if _r.returncode != 0: raise RuntimeError(_scr + " failed:\n" + _r.stderr[-2000:])
    q = trimesh.load(OUTF); print("SHELL (shipped): watertight %s shells %d vol %.0f faces %d bounds %s" % (q.is_watertight, len(q.split(only_watertight=False)), q.volume, len(q.faces), q.bounds.round(1).tolist()))
sh = trimesh.load(OUTF)

# ---- verification vs the finished shell ----
print("--- overlap of the v4 shell with:")
for n, m in STATIC.items(): print("  %-26s %.3f mm3" % (n, vol(sh, m)))
for n, m in (("Ref_Gearbox", ld("Ref_Gearbox")), ("Ref_Motor_GM1614", ld("Ref_Motor_GM1614"))): print("  %-26s %.3f mm3" % (n, vol(sh, m)))
cmx = CollisionManager(); cmx.add_object("shell", sh); worst = []
for ta in TA:
    for n, m in POSES[ta].items():
        o = CollisionManager(); o.add_object(n, m)
        if cmx.in_collision_other(o): worst.append((ta, n, round(vol(sh, m), 3)))
print("rear sweep contacts with the shell (ta, mover, mm3):", worst or "none")
bad = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, -1.4, 0.0, 1.4, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: continue
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            o = CollisionManager(); o.add_object(k, sc[k][0], transform=sc[k][1])
            if cmx.in_collision_other(o): bad.append((th, s, k))
print("front wheel/shell contacts over lock -8..+8 x rack +/-2.75:", bad or "none")

# ---- minimum clearance of every body to the finished shell (mm) ----
print("--- minimum clearance to the shell (mm; < 1.0 flagged; intentional: hinge ~0.30, front magnet boss to its chassis seat ~0.3) ---")
def mind(m):
    o = CollisionManager(); o.add_object("b", m); return cmx.min_distance_other(o)
names = [n for n in STATIC] + ["Ref_Gearbox", "Ref_Motor_GM1614", "Ref_Rear_Driveshaft", "Ref_Servo_Envelope"]
rows = []
for n in dict.fromkeys(names): rows.append((n, mind(STATIC[n] if n in STATIC else ld(n))))
for n in ("Rear_Trailing_Arm_L", "Rear_Upper_Link_L", "Rear_Axle_Top_Bracket", "Ref_Rear_Axle", "Ref_Wheel_RL", "Bar_lever_L", "Bar_cross"):
    rows.append((n + " (sweep %g..%+g)" % (TA_MIN, TA_MAX), min(mind(POSES[ta][n]) for ta in TA if n in POSES[ta])))
def _rolled(ta, deg):      # both rear wheels at trailing-arm angle ta, axle rolled deg about the X axis through the axle centre
    sc_ = POSES[ta] if ta in POSES else rear_movers(ta); wl, wr = sc_["Ref_Wheel_RL"], sc_["Ref_Wheel_RR"]; c = (wl.bounds.mean(axis=0) + wr.bounds.mean(axis=0))/2.0
    T_ = trimesh.transformations.rotation_matrix(math.radians(deg), [1, 0, 0], c); return [posed(wl, T_), posed(wr, T_)]
rows.append(("Rear wheels, roll +/-10 at ride (ta 0)", min(mind(w) for dg in (-10, 10) for w in _rolled(0.0, dg))))
_wz = lambda m: float(m.bounds[:, 2].mean()); _zlo, _zhi = _wz(POSES[TA[0]]["Ref_Wheel_RL"]), _wz(POSES[TA[-1]]["Ref_Wheel_RL"]); _zlo, _zhi = min(_zlo, _zhi), max(_zlo, _zhi)
_rr_in, _rr_all = [], []      # roll x travel; "within stops" = neither wheel centre above the bump stop or below the droop stop
for ta in [float(t) for t in np.arange(TA_MIN, TA_MAX + 0.01, 0.25)]:
    for dg in (-10, -7.5, -5, -2.5, 2.5, 5, 7.5, 10):
        ws = _rolled(ta, dg); d_ = min(mind(w) for w in ws); _rr_all.append((d_, ta, dg))
        if all(_zlo - 0.05 <= _wz(w) <= _zhi + 0.05 for w in ws): _rr_in.append((d_, ta, dg))
if _rr_in: rows.append(("Rear wheels, roll x travel within stops (worst ta %g, roll %+g)" % min(_rr_in)[1:], min(_rr_in)[0]))
rows.append(("Rear wheels, roll stacked beyond stops (worst ta %g, roll %+g)" % min(_rr_all)[1:], min(_rr_all)[0]))
print("  rear wheel centre Z: droop %.2f .. bump %.2f; roll poses within stops: %d of %d; +/-10 deg roll fits for ta %s" % (_zlo, _zhi, len(_rr_in), len(_rr_all), ("%g..%g" % (min(t for _, t, g in _rr_in if abs(g) == 10), max(t for _, t, g in _rr_in if abs(g) == 10))) if any(abs(g) == 10 for _, _, g in _rr_in) else "none"))
for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
    d = []
    for th in (-8, 0, 8):
        for s_ in (-2.75, 0.0, 2.75):
            sc, sol = ns["pose_scene"](th, s_)
            if sc is not None and k in sc: d.append(mind(posed(*sc[k])))
    rows.append((k + " (lock/bump)", min(d)))
for n, d in sorted(rows, key=lambda r: r[1]): print("  %-62s %7.2f%s" % (n, d, "   <-- within 1 mm" if d < 1.0 else ""))

# ---- Yeti tilt-open articulation check (0..45 deg nose-up around rear hinge axis) ----
# ---- rear rail slope: constants vs the built shell ----
_dev, _dev_u = [], []
for _x in np.arange(REAR_SLOPE_XA, TAIL_END_X + 0.4, -3.0):
    _sg = trimesh.intersections.mesh_plane(sh, [1, 0, 0], [_x, 0, 0]); _p = np.array(_sg).reshape(-1, 3)
    _p = _p[(np.abs(_p[:, 1] - REAR_RAIL_Y_IN) < 2e-3) & (_p[:, 2] > rear_rail_z_under(_x) - 0.05)]
    if len(_p): _dev.append(abs(_p[:, 2].max() - rear_rail_z_top(_x)))
    if len(_p) and _x >= REAR_UNDER_XB: _dev_u.append(abs(_p[:, 2].min() - rear_rail_z_under(_x)))
print("--- rear rail slope: top ridge %.3f deg (Z = %.3f + %.5f*(X %+.1f)), underside %.3f deg, inner face |Y| %.3f, tail end X %.1f; max deviation of the built shell: top ridge %.4f mm over X %.0f..%.1f (%d samples), underside %.4f mm over X %.0f..%.0f (%d samples)" % (REAR_SLOPE_DEG, REAR_SLOPE_Z0, REAR_SLOPE, -REAR_SLOPE_X0, REAR_UNDER_DEG, REAR_RAIL_Y_IN, TAIL_END_X, max(_dev) if _dev else float("nan"), REAR_SLOPE_XA, TAIL_END_X, len(_dev), max(_dev_u) if _dev_u else float("nan"), REAR_SLOPE_XA, REAR_UNDER_XB, len(_dev_u)))
print("--- Yeti tilt-open sweep (0..45 deg nose-up around X=%.1f, Z=%.1f) ---" % (REAR_HINGE_X, REAR_HINGE_Z))
fcm = CollisionManager()
for k_ in ("Chassis_Tub", "Cage", "Cabin_Cage", "Front_Bumper", "IFS_Bulkhead_v3 (magnet seats)"):
    if k_ in STATIC: fcm.add_object(k_, STATIC[k_])
tilt_bad = []
for deg in (0, 0.5, 1, 2, 3, 5, 10, 20, 35, 45):
    T_ = trimesh.transformations.rotation_matrix(-math.radians(deg), [0, 1, 0], [REAR_HINGE_X, 0.0, REAR_HINGE_Z])
    o_ = CollisionManager(); o_.add_object("sh", posed(sh, T_))
    d_, nm_ = fcm.min_distance_other(o_, return_names=True)
    hit_ = fcm.in_collision_other(o_)
    if hit_: tilt_bad.append((deg, nm_))
    print("  tilt %4.1f deg: hit=%-5s min_dist=%6.3f mm nearest=%s" % (deg, hit_, d_, nm_))
print("OVERALL: shell X %.1f..%.1f (L %.1f)  Y +-%.1f  Z %.1f..%.1f (roof)  tilt_contacts=%s" % (sh.bounds[0][0], sh.bounds[1][0], sh.bounds[1][0] - sh.bounds[0][0], sh.bounds[1][1], sh.bounds[0][2], sh.bounds[1][2], tilt_bad or "none"))
