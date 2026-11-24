"""Uniform shell wall for scripts/19_shell_v4.py: rebuild the inner surface as the 1.4 mm inward offset of every outer planar facet (along its own normal).
Exception zones (k_zones, rail_zone): tail hinge + bed ledge + fins, front magnet bosses, roof corner posts, rear rails, A-pillar base rib,
front grille. Inside a zone the shell is the old shell, nothing added. Whatever a viewer can see from outside stays where it was (classify,
peel). Added wall stays KEEP_CLR from the given bodies. Errors instead of silent fallbacks: lost part, clearance not reached, finish or peel
not converged, degenerate faces. Entry point: level_wall(shell, bodies=...)."""
import numpy as np, trimesh, manifold3d as m3d, time
from trimesh.ray.ray_pyembree import RayMeshIntersector

WALL_T = 1.4
FACING = 0.0
UNDERHOOD = [[17.0, 0.0], [56.0, 24.6], 29.2]     # X range, |Y| max (inboard of the fender wall inner face), Z max
CORE_LO = np.array([-50.0, 0.0, 12.0]); CORE_HI = np.array([50.0, 0.0, 22.0])

def to_m3d(m):
    return m3d.Manifold(m3d.Mesh64(vert_properties=np.asarray(m.vertices, np.float64), tri_verts=np.asarray(m.faces, np.uint64)))
def to_tm(mm):
    me = mm.to_mesh64(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
GROW = 2e-3
def hull(pts, g=None):
    """convex piece, grown GROW mm away from its centroid so that neighbouring pieces overlap instead of only touching"""
    p = np.asarray(pts, np.float64); g = GROW if g is None else g
    if g > 0:
        c = p.mean(0); d = p - c; L = np.linalg.norm(d, axis=1)[:, None]; p = p + g*d/np.maximum(L, 1e-12)
    return m3d.Manifold.hull_points(p.tolist())

def classify(m, blocker_boxes=()):
    """outer = faces of the exterior skin. core-axis sign test, ambiguous faces decided by a ray cone (interior blocker boxes)."""
    c = m.triangles_center; n = m.face_normals
    d = c - np.clip(c, CORE_LO, CORE_HI); dn = np.linalg.norm(d, axis=1) + 1e-9
    s = np.einsum('ij,ij->i', d, n)/dn
    outer = s > 0.25; amb = np.abs(s) <= 0.25
    if amb.any():
        scene = trimesh.util.concatenate([m] + [trimesh.creation.box(bounds=b) for b in blocker_boxes])
        I = RayMeshIntersector(scene); idx = np.where(amb)[0]
        rng = np.random.default_rng(0); K = 24; frac = np.zeros(len(idx))
        for k in range(K):
            v = rng.normal(size=(len(idx), 3)); v -= 0.35*np.einsum('ij,ij->i', v, n[idx])[:, None]*n[idx]
            v = n[idx] + 0.9*v/np.linalg.norm(v, axis=1)[:, None]; v /= np.linalg.norm(v, axis=1)[:, None]
            frac += ~I.intersects_any(c[idx] + n[idx]*2e-3, v)
        outer[idx] = frac/K > 0.3
    # a face that looks at another part of the shell from close by (< FACING mm) is under-structure, not exterior skin
    I0 = RayMeshIntersector(m); o = c + n*2e-3
    l, r, _ = I0.intersects_location(o, n, multiple_hits=False)
    near = np.zeros(len(c), bool); near[r] = np.linalg.norm(l - o[r], axis=1) < FACING
    outer &= ~near
    # under the hood: stock channels/blocks below the hood skin are under-structure (something of the shell is straight above them)
    ub = (c[:, 0] >= UNDERHOOD[0][0]) & (c[:, 0] <= UNDERHOOD[1][0]) & (np.abs(c[:, 1]) <= UNDERHOOD[1][1]) & (c[:, 2] <= UNDERHOOD[2])
    if ub.any():
        idx = np.where(ub)[0]; up = np.tile([0.0, 0.0, 1.0], (len(idx), 1))
        covered = I0.intersects_any(c[idx] + up*2e-3, up)
        outer[idx[covered]] = False
    # anything a viewer can see from outside the truck is exterior too, whichever way it faces (bed-side faces of rails and bed walls,
    # window frames seen through the openings): it stays where the old shell had it
    outer |= exposure(m) >= EXPO_MIN
    return outer, amb

EXPO_MIN = 0.3      # share of the sky directions on the face's side that are seen from the face (the ground under the sill counts as a blocker)
def expo_pts(m, p, n, nd=96):
    """per point p (outward normal n): fraction of nd sphere directions on the point's outer side along which a ray leaves the shell without hitting it or the ground"""
    z = m.bounds[0, 2] - 0.05; L = 500.0
    fl = trimesh.Trimesh([[-L, -L, z], [L, -L, z], [L, L, z], [-L, L, z]], [[0, 1, 2], [0, 2, 3]], process=False)
    I = RayMeshIntersector(trimesh.util.concatenate([m, fl]))
    i = np.arange(nd) + 0.5; ph = np.arccos(1 - 2*i/nd); th = np.pi*(1 + 5**0.5)*i; D = np.c_[np.cos(th)*np.sin(ph), np.sin(th)*np.sin(ph), np.cos(ph)]
    o = p + n*0.02; esc = np.zeros(len(p)); val = np.zeros(len(p))
    for d in D:
        k = np.where(n @ d > 0.05)[0]
        if len(k): val[k] += 1; esc[k] += ~I.intersects_any(o[k], np.tile(d, (len(k), 1)))
    return esc/np.maximum(val, 1)
def exposure(m):
    """per face: expo_pts at 4 points of the face (centroid and 3 points towards the corners): the mean, or the best point less (PEEL_VIS - EXPO_MIN)"""
    c = m.triangles_center; n = m.face_normals; T = m.triangles
    e = np.array([expo_pts(m, q, n) for q in [c] + [0.6*c + 0.4*T[:, k] for k in range(3)]])
    return np.maximum(e.mean(0), e.max(0) - (PEEL_VIS - EXPO_MIN))      # a face that is partly in view counts with its best point

MITER_MAX = 2.0     # x t: miter points of very sharp folds are clamped (keeps the corner solid without long spikes)
def miter(n1, n2, t):
    """offset vector of the line where the two offset planes meet (distance t from both faces)"""
    c = float(np.dot(n1, n2))
    if c < -0.95: return None
    m = t*(n1 + n2)/(1.0 + c); L = np.linalg.norm(m)
    return m if L <= MITER_MAX*t else m*(MITER_MAX*t/L)

NARROW = 0.3      # mm: mean width (area / longest extent) under which a planar outer facet is a step or a strip, not a wall face
def wide_faces(m, outer):
    """False for the faces of narrow planar facets (ledges of styling steps, strips, lone sliver triangles). Their own 1.4 mm prism would be a
    fin or a block hanging off the step, so they get no prism, wedge or fan; they stay exterior (clip, edge fill)."""
    A = m.area_faces; V = m.vertices; F = m.faces; grp = -np.ones(len(F), int)
    for i, f in enumerate(m.facets): grp[f] = i
    lone = np.where(grp < 0)[0]; grp[lone] = len(m.facets) + np.arange(len(lone)); wide = np.ones(len(F), bool)
    order = np.argsort(grp); cuts = np.r_[0, np.flatnonzero(np.diff(grp[order])) + 1, len(F)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        f = order[a:b]; f = f[outer[f]]
        if len(f) == 0: continue
        P = V[np.unique(F[f])]; L = np.linalg.norm(P.max(0) - P.min(0))
        if A[f].sum()/max(L, 1e-9) < NARROW: wide[f] = False
    return wide
def build_band(m, outer, t=WALL_T, log=print):
    """union of per-face prisms + edge wedges + vertex fans (valleys get a flat chamfer instead of a gap)."""
    t0 = time.time(); F = m.faces; V = m.vertices; N = m.face_normals; A = m.area_faces
    n_out = int(outer.sum()); outer = outer & wide_faces(m, outer); log("  narrow outer facets left without a prism: %d faces" % (n_out - int(outer.sum())))
    of = np.where(outer & (A > 1e-7))[0]; pieces = []
    for f in of:
        a, b, c = V[F[f]]; o = N[f]*t
        pieces.append(hull([a, b, c, a - o, b - o, c - o]))
    # edge wedges between two outer faces with different normals
    ea = m.face_adjacency; ee = m.face_adjacency_edges; ok = outer[ea[:, 0]] & outer[ea[:, 1]]
    nw = 0
    for (f1, f2), (u, v) in zip(ea[ok], ee[ok]):
        if np.linalg.norm(N[f1] - N[f2]) < 1e-3: continue
        P = [V[u], V[v], V[u] - N[f1]*t, V[v] - N[f1]*t, V[u] - N[f2]*t, V[v] - N[f2]*t]
        mv = miter(N[f1], N[f2], t)
        if mv is not None: P += [V[u] - mv, V[v] - mv]
        pieces.append(hull(P)); nw += 1
    # vertex fans
    vf = m.vertex_faces; nv = 0
    for vi in np.unique(F[of]):
        fs = [f for f in vf[vi] if f >= 0 and outer[f]]
        ns = np.unique(np.round(N[fs], 4), axis=0)
        if len(ns) < 3: continue
        pts = [V[vi]] + [V[vi] - q*t for q in ns]
        for i in range(len(ns)):
            for j in range(i + 1, len(ns)):
                mv = miter(ns[i], ns[j], t)
                if mv is not None: pts.append(V[vi] - mv)
        pieces.append(hull(pts)); nv += 1
    pieces = [p for p in pieces if not p.is_empty() and p.volume() > 1e-9]
    log("  band pieces: %d prisms, %d wedges, %d vertex fans (%.1fs)" % (len(of), nw, nv, time.time() - t0))
    U = m3d.Manifold.batch_boolean(pieces, m3d.OpType.Add); log("  band union %.1fs vol %.1f" % (time.time() - t0, U.volume()))
    return U

def build_edge_fill(m, outer, t=WALL_T):
    """Round fill along every edge of the outer skin: all points within t of the edge. Used only where the OLD shell was solid (level_wall
    intersects it with the old shell): where two outer faces meet without a wedge (a sliver or an inner face lies between them) the prisms
    leave a slit, up to a through-slit that cuts a wall loose from the skin above it."""
    F = m.faces; V = m.vertices; A = m.area_faces; of = np.where(outer & (A > 1e-7))[0]
    E = np.unique(np.sort(np.vstack([F[of][:, [0, 1]], F[of][:, [1, 2]], F[of][:, [2, 0]]]), axis=1), axis=0)
    D = trimesh.creation.icosphere(subdivisions=1).vertices*(t - 0.03)      # just inside the prisms: no near-coincident surfaces on flat facets
    pcs = [m3d.Manifold.hull_points(np.vstack([V[u] + D, V[v] + D]).tolist()) for u, v in E]
    return m3d.Manifold.batch_boolean(pcs, m3d.OpType.Add)

def build_out_band(m, outer, h=1.2):
    F = m.faces; V = m.vertices; N = m.face_normals; A = m.area_faces; pieces = []
    for f in np.where(outer & (A > 1e-7))[0]:
        a, b, c = V[F[f]]; o = N[f]*h
        pieces.append(hull([a, b, c, a + o, b + o, c + o]))
    return m3d.Manifold.batch_boolean([p for p in pieces if not p.is_empty()], m3d.OpType.Add)

def boxes_m3d(bxs):
    return m3d.Manifold.batch_boolean([m3d.Manifold.cube((b[1][0]-b[0][0], b[1][1]-b[0][1], b[1][2]-b[0][2])).translate(tuple(b[0])) for b in bxs], m3d.OpType.Add)

def sym_classify(m, blocker_boxes=()):
    outer, amb = classify(m, blocker_boxes)
    c = m.triangles_center; mc = c*np.array([1, -1, 1])
    from scipy.spatial import cKDTree
    d, j = cKDTree(c).query(mc)
    left = c[:, 1] >= 0
    o2 = outer.copy(); ok = d < 1e-3
    o2[ok & ~left] = outer[j[ok & ~left]]
    return o2, amb, ok

def k_zones():
    """exception zones (left half, mirrored): material of the old shell is kept inside them (union with the new 1.4 mm wall)."""
    Z = {}
    Z["tail hinge + bed ledge + fins"] = [[-67.6, 17.0, 24.0], [-50.0, 33.6, 40.2]]
    Z["front magnet bosses"] = [[51.5, 1.5, 20.0], [63.0, 13.6, 36.0]]
    Z["roof corner posts"] = [[-24.0, 19.5, 36.0], [-18.5, 24.0, 47.6]]
    Z["fender / A-pillar junction fillers"] = [[17.4, 23.4, 6.5], [20.6, 26.6, 27.6]]
    Z["front grille (bars and slots: exterior on both sides)"] = [[61.5, 0.0, 22.5], [64.8, 19.5, 28.0]]
    # Greenhouse (owner 2026-11-23: no ragged fringe at the window rims). Pillars, sills, cowl edge and the rim of the roof panel are seen through the
    # windows, so they stay the old shell; only the roof panel's core (X -19.5..2.0, |Y| < 17.5; odd decimals keep the zone faces off the shell's own planes) gets the added wall, ending in a straight step 2 mm in from its edges.
    Z["greenhouse below the roof (pillars, sills, cowl edge)"] = [[-23.63, 0.0, 30.47], [19.03, 26.5, 39.97]]
    Z["roof rim, rear"] = [[-23.63, 0.0, 39.96], [-19.53, 26.5, 47.6]]
    Z["roof rim, front + windshield header"] = [[2.03, 0.0, 39.96], [19.03, 26.5, 47.6]]
    Z["roof rim, sides"] = [[-19.54, 17.47, 39.96], [2.04, 26.5, 47.6]]
    return Z

def rail_zone(x0=-50.5, x1=-23.5, y0=20.6, y1=23.6):
    zt = lambda x: 33.48 + 0.284*(x + 64.0); zu = lambda x: 30.998 + 0.2615*(x + 64.0)
    pts = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (zu(x) - 0.5, zt(x) + 0.6)]
    return pts

def zones_m3d():
    pcs = []
    for nm, b in k_zones().items():
        for sg in (1, -1):
            lo, hi = np.array(b[0], float), np.array(b[1], float)
            if sg < 0: lo[1], hi[1] = -hi[1], -lo[1]
            pcs.append(m3d.Manifold.cube(tuple(hi - lo)).translate(tuple(lo)))
    for sg in (1, -1):
        pcs.append(hull([(x, sg*y, z) for (x, y, z) in rail_zone()]))
    return m3d.Manifold.batch_boolean(pcs, m3d.OpType.Add)

def in_zones(p):
    k = np.zeros(len(p), bool); q = p.copy(); q[:, 1] = np.abs(q[:, 1])
    for b in k_zones().values():
        k |= np.all((q >= b[0]) & (q <= b[1]), axis=1)
    from scipy.spatial import Delaunay
    k |= Delaunay(np.array(rail_zone())).find_simplex(q) >= 0
    return k

def mirror_left_half(m):
    """= scripts/19_shell_v4.py _mirror_left_half (exact L/R symmetry, left half Y>=0 is the master)."""
    mm = to_m3d(m).trim_by_plane((0, 1, 0), 0.0); h = to_tm(mm)
    v = h.vertices.copy(); v[np.abs(v[:, 1]) < 1e-6, 1] = 0.0
    f = h.faces[~np.all(v[h.faces][:, :, 1] == 0.0, axis=1)]
    r = trimesh.Trimesh(v*np.array([1.0, -1.0, 1.0]), f[:, ::-1], process=False)
    o = trimesh.util.concatenate([trimesh.Trimesh(v, f, process=False), r]); o.merge_vertices()
    return o

def mirror_m3d(W, eps=1e-4):
    L = W.trim_by_plane((0, 1, 0), 0.0).translate((0, -eps, 0)); R = L.mirror((0, 1, 0))
    return L + R

def drop_voids(t):
    sh = t.split(only_watertight=False); sh = sorted(sh, key=lambda q: -q.volume)
    return sh[0], [q.volume for q in sh[1:]]

BLOCKER = [[[-58, -19.5, 7.5], [56, 19.5, 26.0]]]
def level_wall(S, t=WALL_T, log=print, bodies=None):
    """S: watertight trimesh shell. Returns the levelled shell (trimesh) + the outer-face mask."""
    outer, amb, _ = sym_classify(S, BLOCKER)
    Sm = to_m3d(S)
    P = build_band(S, outer, t, log); Q = build_out_band(S, outer) - Sm; K = zones_m3d()
    W = (((P + (build_edge_fill(S, outer, t) ^ Sm)) - Q) - K) + (Sm ^ K)      # inside an exception zone the shell is the old shell, nothing added: added wall there
    # narrowed the hinge and ledge slots to 0.2 mm, filled the grille slots and bridged them (handles)
    if bodies: W = clearance_patch(W, Sm, bodies, log=log, S=S)
    w = finish(_largest(W, Sm, log, "levelled"), S, log=log)
    return peel(w, S, Sm, log=log), outer

DROP_MAX = 1.0      # mm3 of OLD shell material in one loose piece: more than this is an error, not a crumb
def _largest(W, Sm, log, lab):
    """largest component of W. Loose pieces are dropped: crumbs, and added material that the exterior clip cut loose (it lies outside the old shell).
    A loose piece that holds more than DROP_MAX mm3 of the old shell is a lost part: error."""
    ps = sorted(W.decompose(), key=lambda p: -p.volume()); lost = []
    log("  %s: %d components, kept %.1f mm3, dropped %.2f mm3" % (lab, len(ps), ps[0].volume(), sum(p.volume() for p in ps[1:])))
    for p in ps[1:]:
        if abs(p.volume()) <= DROP_MAX: continue
        vo = (p ^ Sm).volume(); bb = np.array(p.bounding_box())[[0, 3, 1, 4, 2, 5]]
        log("    dropped piece %.2f mm3 (%.2f mm3 of it old shell material), X %.1f..%.1f Y %.1f..%.1f Z %.1f..%.1f%s" % ((p.volume(), vo) + tuple(bb) + ("  <-- LOST PART" if vo > DROP_MAX else "  (added material cut loose outside the old shell)",)))
        if vo > DROP_MAX: lost.append(p)
    if lost: raise RuntimeError("shell_wall: %d loose piece(s) with more than %.1f mm3 of old shell material would be dropped (%s, see the log)" % (len(lost), DROP_MAX, lab))
    return ps[0]

PEEL_VIS = 0.37; PEEL_DEV = 0.04; FIN_T = 0.45; PEEL_OK = 0.3; PAD_VIS = 0.5; PAD_FIN = 0.3; REC_MAX = 1.0; FIN_AREA = 0.4; FIN_DONE = 0.8      # fin patches from FIN_AREA mm2 are cut; done when the largest is under FIN_DONE (a print defect is a patch over 1 mm2)      # exposure that counts as visible; distance from the old surface that counts as new; fin thickness; mm2 per side that may remain
def _dist(m, q):
    return np.concatenate([trimesh.proximity.closest_point(m, q[i:i + 20000])[1] for i in range(0, len(q), 20000)]) if len(q) else np.zeros(0)
def _cutters(q, pad, link=0.5, cell=2.5):
    """convex cutters around point groups: points closer than link form a group, a group is split on a cell grid, each part becomes the hull of its points +- pad"""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if len(q) == 0: return []
    pr = cKDTree(q).query_pairs(link, output_type="ndarray"); lab = connected_components(coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(q), len(q))), directed=False)[1]
    key = np.c_[lab, np.floor(q/cell).astype(int)]; _, g = np.unique(key, axis=0, return_inverse=True); g = g.ravel()
    off = np.vstack([np.eye(3), -np.eye(3)])*pad; out = []
    for i in range(g.max() + 1):
        c = (q[g == i][:, None, :] + off[None]).reshape(-1, 3)
        for sg in (1.0, -1.0): out.append(m3d.Manifold.hull_points((c*np.array([1.0, sg, 1.0])).tolist()))
    return out
def peel(o, S, Sm, log=print, passes=8):
    """Takes away added material that (a) a viewer can see from outside (new surface with exposure >= PEEL_VIS: there the old shell stays as it was)
    or (b) forms a fin: a sheet of added material thinner than FIN_T with new surface on both faces (face normals more than 120 deg apart, so a
    plain corner does not count). Convex cutters around the offending surface samples, never into the old shell; repeated until less than
    PEEL_OK mm2 per side is left. Left half is the master."""
    for it in range(passes + 1):
        ns = 300000; p, fi = trimesh.sample.sample_surface(o, ns, seed=21 + it); n = o.face_normals[fi]; a1 = o.area/ns
        k = p[:, 1] >= 0; p, n = p[k], n[k]; new = _dist(S, p) > PEEL_DEV; p, n = p[new], n[new]
        vis = expo_pts(o, p, n) >= PEEL_VIS
        rec = np.zeros(len(p), bool); rec[vis] = S.contains(p[vis] - n[vis]*0.01); vis &= ~rec      # a visible NEW surface inside the old shell is a recess (old material missing): cannot be cut away, reported
        org = p - n*1e-4; loc, r, tri = RayMeshIntersector(o).intersects_location(org, -n, multiple_hits=False)      # 1e-4: a paper-thin flap must not be skipped
        tt = np.full(len(p), np.inf); tt[r] = np.linalg.norm(loc - org[r], axis=1) + 1e-4; hit = np.zeros_like(p); hit[r] = loc
        hn = np.zeros_like(p); hn[r] = o.face_normals[tri]
        fin = np.zeros(len(p), bool); c = np.where((tt < FIN_T) & (np.einsum('ij,ij->i', hn, n) < -0.5))[0]; fin[c] = _dist(S, hit[c]) > PEEL_DEV
        fmax = 0.0
        if fin.any():      # only fins that form a patch of FIN_AREA or more count (and are cut); smaller specks are cut debris
            from scipy.spatial import cKDTree
            from scipy.sparse import coo_matrix
            from scipy.sparse.csgraph import connected_components
            q = p[fin]; pr = cKDTree(q).query_pairs(0.5, output_type="ndarray"); lab = connected_components(coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(q), len(q))), directed=False)[1]
            fmax = np.bincount(lab).max()*a1; fin[np.where(fin)[0][np.bincount(lab)[lab]*a1 < FIN_AREA]] = False
        av, af, ar = vis.sum()*a1, fin.sum()*a1, rec.sum()*a1
        peel.last = o
        log("  peel pass %d: new surface seen from outside %.2f mm2, fins under %.2f mm in patches of %.1f mm2 or more: %.2f mm2, largest patch %.2f mm2, visible recesses into the old shell %.2f mm2 (per side)" % (it, av, FIN_T, FIN_AREA, af, fmax, ar))
        if av < PEEL_OK and fmax < FIN_DONE:
            if ar > REC_MAX: raise RuntimeError("shell_wall.peel: %.2f mm2 per side of visible surface is recessed into the old shell (limit %.1f)" % (ar, REC_MAX))
            return o
        if it == passes: raise RuntimeError("shell_wall.peel: not converged after %d passes (visible new surface %.2f mm2, largest fin patch %.2f mm2 per side)" % (passes, av, fmax))
        cut = _cutters(p[vis], PAD_VIS) + _cutters(np.vstack([p[fin], hit[fin]]), PAD_FIN)
        o = finish(_largest(to_m3d(o) - (m3d.Manifold.batch_boolean(cut, m3d.OpType.Add) - Sm), Sm, log, "peel %d (%d cutters)" % (it, len(cut)//2)), S, log=log)
    return o


FIN_SIMPLIFY = 0.004; FIN_WELD = 0.02; FIN_ROUNDS = 10; PINCH_GAP = 4e-4; ASPECT_MAX = 500; NEEDLE_EDGE = 0.15
def _collapse_short(V, F, eps, ext, on, extra=(), skin=None, S=None):
    """Collapses every edge shorter than eps onto one of its ends (no vertex is created or moved, so nothing drifts further than eps).
    A collapse must keep the mesh a 2-manifold (link condition) and may not turn a face over. Vertices of the old exterior stay;
    centre-plane vertices only merge into centre-plane vertices. Returns the faces and the number of short edges that had to stay."""
    import heapq
    F = [list(map(int, f)) for f in F]; vf = {}
    for i, f in enumerate(F):
        for v in f: vf.setdefault(v, set()).add(i)
    alive = [True]*len(F)
    def elen(u, v): return float(np.linalg.norm(V[u] - V[v]))
    def nrm(a, b, c): return np.cross(V[b] - V[a], V[c] - V[a])
    h = [(elen(f[k], f[k - 2]), f[k], f[k - 2]) for f in F for k in range(3) if f[k] < f[k - 2]]
    h = [e for e in h if e[0] < eps] + [(elen(int(u), int(v)), int(min(u, v)), int(max(u, v))) for u, v in extra]; heapq.heapify(h); kept = set(); n = 0
    def try_col(r, k):      # remove r, keep k
        if ext[r] or (on[r] and not on[k]): return False
        if skin is not None and skin[r]:      # r lies on the old surface: it may only slide a hair (FIN_WELD) along that surface, never off it or across a corner
            if not skin[k] or elen(r, k) > FIN_WELD or trimesh.proximity.closest_point(S, [0.5*(V[r] + V[k])])[1][0] > 1e-3: return False
        sh = vf[r] & vf[k]
        if len(sh) != 2: return False
        w = {x for i in sh for x in F[i] if x != r and x != k}
        nr = {x for i in vf[r] for x in F[i]} - {r, k}; nk = {x for i in vf[k] for x in F[i]} - {r, k}
        if len(w) != 2 or (nr & nk) != w: return False
        for i in vf[r] - sh:
            a, b, c = F[i]; n0 = nrm(a, b, c); n1 = nrm(*[k if x == r else x for x in F[i]])
            if float(np.dot(n0, n0)) > 1e-16 and float(np.dot(n0, n1)) <= 0.0: return False
        for i in sh:
            alive[i] = False
            for x in F[i]: vf[x].discard(i)
        for i in list(vf[r]):
            F[i] = [k if x == r else x for x in F[i]]; vf[k].add(i)
        vf[r] = set()
        for x in {x for i in vf[k] for x in F[i]} - {k}:
            L = elen(k, x)
            if L < eps: heapq.heappush(h, (L, min(k, x), max(k, x)))
        return True
    while h:
        L, u, v = heapq.heappop(h)
        if not (vf.get(u) and vf.get(v)) or not (vf[u] & vf[v]): continue
        a, b = (u, v) if (on[v], ext[v]) >= (on[u], ext[u]) else (v, u)      # a goes, b stays
        if try_col(a, b) or try_col(b, a): n += 1; kept.discard((u, v))
        else: kept.add((u, v))
    left = sum(1 for (u, v) in kept if vf.get(u) and vf.get(v) and (vf[u] & vf[v]))
    return np.array([f for i, f in enumerate(F) if alive[i]], np.int64), n, left

def finish(Wm, S, log=print):
    """Levelled manifold -> clean, exactly L/R symmetric trimesh (left half Y >= 0 is the master, like _mirror_left_half in the builder).
    The union of thousands of prisms leaves ~10^5 micro-slivers that manifold's simplify keeps, and the builder's pure-Python cleanup
    does not finish on such a mesh. So: simplify as one original mesh, then collapse every edge shorter than FIN_WELD."""
    from scipy.spatial import cKDTree
    def arr(me, merge):      # float32 = what the STL keeps; merge: vertices that coincide there become one; the centre seam is put back on Y = 0 (simplify may pull it off by up to FIN_SIMPLIFY)
        v = np.asarray(me.vert_properties)[:, :3].astype(np.float32).astype(np.float64); v[np.abs(v[:, 1]) < FIN_SIMPLIFY + 1e-3, 1] = 0.0
        q = trimesh.Trimesh(v, np.asarray(me.tri_verts), process=merge); V = np.asarray(q.vertices).copy()
        return V, np.asarray(q.faces).astype(np.int64), V[:, 1] == 0.0, cKDTree(S.vertices).query(V)[0] < 2e-5, _dist(S, V) < 2e-4
    def man(V, F): return m3d.Manifold(m3d.Mesh(vert_properties=V.astype(np.float32), tri_verts=F.astype(np.uint32)))
    me = Wm.trim_by_plane((0, 1, 0), 0.0).as_original().simplify(FIN_SIMPLIFY).to_mesh64(); n0 = len(me.tri_verts); done = False
    for rnd in range(FIN_ROUNDS):
        # hair cracks between pieces (mouth < FIN_SIMPLIFY wide, up to ~0.7 mm deep into the wall) close here and are dropped as voids
        V, F, on, ext, skin = arr(me, True); F, nc, left = _collapse_short(V, F, FIN_WELD, ext, on, skin=skin, S=S); M = man(V, F); how = "merged"
        if M.status() != m3d.Error.NoError:      # the float32 merge pinched two sheets together: this round on the unmerged vertices
            V, F, on, ext, skin = arr(me, False); F, nc, left = _collapse_short(V, F, FIN_WELD, ext, on, skin=skin, S=S); M = man(V, F); how = "unmerged"
            if M.status() != m3d.Error.NoError: raise RuntimeError("shell_wall.finish: round %d left no valid manifold (%s)" % (rnd, M.status()))
        ps = sorted(M.simplify(FIN_SIMPLIFY).decompose(), key=lambda p: -p.volume()); me = ps[0].to_mesh()
        V, F, on, ext, skin = arr(me, True); q = trimesh.Trimesh(V, F, process=False)
        log("  finish round %d (%s): left half %d -> %d faces (%d edges < %.3f mm collapsed, %d could not be), kept %.1f mm3, dropped %d crumbs/voids (%.2e mm3), watertight %s" % (rnd, how, n0, len(F), nc, FIN_WELD, left, ps[0].volume(), len(ps) - 1, sum(abs(p.volume()) for p in ps[1:]), q.is_watertight))
        n0 = len(F)
        if nc == 0 and len(ps) == 1 and q.is_watertight: done = True; break
        if nc == 0 and len(ps) == 1:      # stalled on pinches: two sheets that touch in a vertex or along an edge once the vertices are merged.
            V, F, on, ext, skin = arr(me, False)      # keep manifold's own (unmerged) vertices and move the coincident ones PINCH_GAP apart, towards their own faces
            F, nc, left = _collapse_short(V, F, FIN_WELD, ext, on, skin=skin, S=S); used = np.zeros(len(V), bool); used[F.ravel()] = True
            _, inv, cnt = np.unique(V, axis=0, return_inverse=True, return_counts=True); dup = np.where((cnt[inv.ravel()] > 1) & used)[0]
            cen = V[F].mean(1); acc = np.zeros_like(V); k = np.zeros(len(V)); np.add.at(acc, F.ravel(), np.repeat(cen, 3, axis=0)); np.add.at(k, F.ravel(), 1)
            d = acc[dup]/np.maximum(k[dup], 1)[:, None] - V[dup]; d[on[dup], 1] = 0.0; V[dup] += PINCH_GAP*d/np.maximum(np.linalg.norm(d, axis=1), 1e-12)[:, None]
            V = V.astype(np.float32).astype(np.float64); V[on, 1] = 0.0
            log("  finish: %d pinch vertices moved %.4f mm apart" % (len(dup), PINCH_GAP)); done = True; break
    if not done: raise RuntimeError("shell_wall.finish: not converged after %d rounds (edges collapsed in the last round %d, components %d, watertight %s)" % (FIN_ROUNDS, nc, len(ps), q.is_watertight))
    F, nf = _flip_caps(V, F, on); nn = 0
    for _ in range(4):      # needle and cap faces (aspect over ASPECT_MAX): collapse the short edge of a needle, flip the long edge of a cap
        T = V[F]; L = np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2); A = 0.5*np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
        bad = np.where(L.max(1)**2/np.maximum(2*A, 1e-300) > ASPECT_MAX)[0]
        if len(bad) == 0: break
        k = L[bad].argmin(1); ok = (L[bad, k] < 0.25*L[bad].max(1)) & (L[bad, k] < NEEDLE_EDGE)      # L[:, k] is the edge from vertex k-1 to vertex k
        F2, nc, _ = _collapse_short(V, F, 0.0, ext, on, extra=[(F[f, j - 1], F[f, j]) for f, j in zip(bad[ok], k[ok])], skin=skin, S=S)
        F2, nf2 = _flip_caps(V, F2, on, amax=ASPECT_MAX); F = F2; nn += nc + nf2
        if nc + nf2 == 0: break
    F = F[~np.all(on[F], axis=1)]      # drop the cap on the centre plane
    o = trimesh.util.concatenate([trimesh.Trimesh(V, F, process=False), trimesh.Trimesh(V*np.array([1.0, -1.0, 1.0]), F[:, ::-1], process=False)]); o.merge_vertices()
    log("  finish: %d cap slivers flipped, %d needle/cap faces (aspect > %d) removed; shell %d faces, watertight %s, faces under 1e-6 mm2: %d, volume %.1f" % (nf, nn, ASPECT_MAX, len(o.faces), o.is_watertight, int((o.area_faces < 1e-6).sum()), o.volume))
    if not (o.is_watertight and o.is_volume): raise RuntimeError("shell_wall.finish: the levelled shell is not a watertight volume")
    if (o.area_faces < 1e-6).any(): raise RuntimeError("shell_wall.finish: %d degenerate faces (area < 1e-6 mm2) remain" % int((o.area_faces < 1e-6).sum()))
    return o

def _flip_caps(V, F, on, tol=2e-5, amax=None):
    """A cap sliver (area < tol mm2, no short edge: one vertex sits on the opposite edge) goes by flipping its longest edge."""
    F = F.copy(); n = 0
    def area(a, b, c): return 0.5*float(np.linalg.norm(np.cross(V[b] - V[a], V[c] - V[a])))
    for _ in range(8):
        T = V[F]; A = 0.5*np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
        asp = np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2).max(1)**2/np.maximum(2*A, 1e-300)
        bad = np.where((A < tol) | ((asp > amax) if amax else False))[0]
        if len(bad) == 0: break
        he = {(int(f[k]), int(f[(k + 1) % 3])): i for i, f in enumerate(F) for k in range(3)}; touched = set(); did = 0
        for i in bad:
            if i in touched: continue
            f = [int(x) for x in F[i]]; k = int(np.argmax([np.linalg.norm(V[f[(j + 1) % 3]] - V[f[j]]) for j in range(3)]))
            p, q, o = f[k], f[(k + 1) % 3], f[(k + 2) % 3]; j = he.get((q, p))
            if j is None or j in touched: continue
            o2 = [int(x) for x in F[j] if x != p and x != q]
            if len(o2) != 1 or (o, o2[0]) in he or (o2[0], o) in he: continue
            o2 = o2[0]
            if on[p] and on[q] and not (on[o] and on[o2]): continue      # the centre seam stays
            nj = np.cross(V[p] - V[q], V[o2] - V[q]); n1 = np.cross(V[o2] - V[p], V[o] - V[p]); n2 = np.cross(V[q] - V[o2], V[o] - V[o2])
            if min(area(p, o2, o), area(o2, q, o)) <= max(A[i], 1e-6) or float(np.dot(nj, n1)) <= 0 or float(np.dot(nj, n2)) <= 0: continue
            if A[i] >= tol:      # flipped for its aspect ratio only: both new faces must be better shaped than the worse of the two old ones
                def asp3(a_, b_, c_): return max(np.linalg.norm(V[a_] - V[b_]), np.linalg.norm(V[b_] - V[c_]), np.linalg.norm(V[c_] - V[a_]))**2/max(2*area(a_, b_, c_), 1e-300)
                if max(asp3(p, o2, o), asp3(o2, q, o)) >= 0.5*max(asp[i], asp[j]): continue
            F[i] = (p, o2, o); F[j] = (o2, q, o); touched.update((i, j)); did += 1
        n += did
        if did == 0: break
    return F, n

KEEP_CLR = 1.5
def dil_m3d(m, c, crop=None):
    """offset-by-translation dilation (sphere directions, 2 rings), like dil() in the builder"""
    mm = to_m3d(m)
    if crop is not None: mm = mm ^ m3d.Manifold.cube(tuple(np.subtract(crop[1], crop[0]))).translate(tuple(crop[0]))
    dirs = [np.array(v, float) for v in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1))]
    for a in (1, -1):
        for b in (1, -1):
            dirs += [np.array(v, float)/np.sqrt(2) for v in ((a,b,0),(a,0,b),(0,a,b))]
            for g in (1, -1): dirs.append(np.array((a, b, g), float)/np.sqrt(3))
    pcs = [mm] + [mm.translate(tuple(d*c*f)) for d in dirs for f in (1.0, 0.5)]
    return m3d.Manifold.batch_boolean(pcs, m3d.OpType.Add)

def clearance_cut(Wm, Sm, bodies, c=KEEP_CLR, log=print):
    """new (added) wall material may not come closer than c to the given bodies; old material is left as it was"""
    for nm, b in bodies.items():
        D = dil_m3d(b, c + 0.03) - Sm
        v0 = Wm.volume(); Wm = Wm - D; log("  clearance %.2f to %-24s removed %.3f mm3 of added wall" % (c, nm, v0 - Wm.volume()))
    return Wm

def clearance_patch(Wm, Sm, bodies, c=KEEP_CLR, log=print, passes=6, S=None):
    """remove added wall within c of each body: balls of radius c+0.05 at the body points nearest to offending wall samples"""
    for it in range(passes):
        A = Wm - Sm
        if A.is_empty(): break
        a = to_tm(A); pts = trimesh.sample.sample_surface(a, 300000, seed=11 + it)[0]; balls = []
        for nm, b in bodies.items():
            lo, hi = b.bounds[0] - c - 0.2, b.bounds[1] + c + 0.2; sel = pts[np.all((pts > lo) & (pts < hi), axis=1)]
            if len(sel) == 0: continue
            cp = np.zeros_like(sel); dd = np.zeros(len(sel))
            for i in range(0, len(sel), 15000):
                q = trimesh.proximity.closest_point(b, sel[i:i+15000]); cp[i:i+15000] = q[0]; dd[i:i+15000] = q[1]
            bad = dd < c + 0.03
            if bad.any():
                cc = cp[bad][np.unique(np.round(cp[bad]/0.15).astype(int), axis=0, return_index=True)[1]]      # one true body point per 0.15 mm cell (a rounded centre can sit 0.13 mm off the body: clearance 1.49)
                balls += [m3d.Manifold.sphere(c + 0.06, 24).translate(tuple(x)) for x in cc]
                log("  clearance pass %d: %-24s %d close samples (min %.3f) -> %d balls" % (it, nm, bad.sum(), dd.min(), len(cc)))
        if not balls: break
        Wm = Wm - (m3d.Manifold.batch_boolean(balls, m3d.OpType.Add) - Sm)
    A = Wm - Sm      # check: no NEW surface (further than 0.02 mm from the old surface) may be left within c of a body
    if S is not None and not A.is_empty():
        pts = trimesh.sample.sample_surface(to_tm(A), 300000, seed=97)[0]; left = []
        for nm, b in bodies.items():
            lo, hi = b.bounds[0] - c - 0.2, b.bounds[1] + c + 0.2; sel = pts[np.all((pts > lo) & (pts < hi), axis=1)]
            if len(sel) == 0: continue
            dd = _dist(b, sel); k = dd < c
            if k.any():
                k[k] = _dist(S, sel[k]) > 0.02
                if k.any(): left.append("%s: %d samples, min %.3f at %s" % (nm, k.sum(), dd[k].min(), np.round(sel[k][np.argmin(dd[k])], 1).tolist()))
        log("  clearance check: new surface within %.2f mm of a body: %s" % (c, "; ".join(left) or "none"))
        if left: raise RuntimeError("shell_wall.clearance_patch: added wall is still closer than %.2f mm to: %s" % (c, "; ".join(left)))
    return Wm
