"""Body shell rebuilt from planes (Project_Fugu method): every exterior facet of the owner-approved reference is a skin plane,
the wall is the slab between that plane and the same plane moved inward by WALL, the shell is the exact union of those slabs.
Run from the repo root:  python scripts/24_shell_planes.py
  OUT_STL=path            where the shell goes (default stl/v4/Tacoma_Trophy_Body_Shell.stl)
  SHELL_ONLY_CORE=1       wall cells only: no bars/fins, hinge lugs or magnet bosses from the reference (a look at the plane work alone)
See scripts/shell_planes_lib.py for how a cell, a crease wedge and the mirror weld are made."""
import os, sys, time
import numpy as np, trimesh, manifold3d as m3
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shell_planes_lib as L

# ---------------------------------------------------------------- constants (mm; X forward, Y left, Z up)
HERE = os.path.dirname(os.path.abspath(__file__))
REF_STL = os.path.join(HERE, "data", "shell_soma", "soma.stl")      # owner-approved exterior
OUT_STL = os.environ.get("OUT_STL", os.path.join(HERE, "..", "stl", "v4", "Tacoma_Trophy_Body_Shell.stl"))
ONLY_CORE = os.environ.get("SHELL_ONLY_CORE", "0") == "1"

WALL = 1.4                # printed wall: every skin plane is moved inward by exactly this
PLANE_TOL = 0.01          # a reference face joins a skin plane when its corners are within this of the plane ...
PLANE_ANGLE = 5.0         # ... and its normal within this many degrees
REF_ALT_MIN = 3e-3        # mm: reference triangles flatter than this (height over longest edge) are removed before planes are read: their normal is noise
REF_SHORT = 0.03          # mm: ... and an edge shorter than this on such a triangle is collapsed to a point
BEAD = 0.02               # mm: radius of the bead that fuses two parts of the body which would otherwise touch on a bare edge
MIN_PART = 0.05           # mm3: loose parts smaller than this are wafers of the union (not reported one by one); also the least volume of a piece after the weld
HALF_MARGIN = 3.0        # the +Y half is built from skin with Y > -HALF_MARGIN, then cut at Y = 0 and mirrored
CAVITY_FLOOR = ((-66.0, -24.5, 6.72), (61.5, 24.5, 6.72))   # the open underside of the cavity, closed by a flat sheet when asking what an inside face looks out on
LIP_OPEN_SHARE = 0.6       # a face the cavity can see, but which itself looks out on open air in more than this share of directions, is exterior (underside of a lip)
LIP_MAX_THICK = 8.0        # mm: ... and has skin facing the other way within this distance behind it
AWAY_DOT = 0.3             # a cell may grow into the cavity layer of a thin wall face only if that face is turned away from the cell's plane by more than this (cosine)
LEFTOVER_VOL = 0.5          # mm3: a piece outside the reference smaller than this ...
LEFTOVER_PLATE = 0.2        # mm: ... or thinner than this (volume / surface) ...
LEFTOVER_CONTACT = 0.25     # ... is a leftover when less than this share of its surface lies against the reference solid
OWN_PROBE = 0.05            # mm: depth of the sliver under a facet / along a crease used to tell which part of a cut cell is still its own wall
RIM_MAX_DOT = 0.9           # a neighbour turned more than this (cosine) away from a thin wall face ends that face's cavity layer
CORNER_MAX_DOT = 0.3        # ... and so does a neighbour that is a thin wall face itself but turned more than this (cosine) at a
                            # sharp corner of the solid (the inboard face of the A-pillar against its underside): it has its own layer
CAVITY_LAYER = WALL + 0.1  # depth of the layer in front of the reference's thin inside wall faces into which the wall may grow
# Openings whose rims follow the reference's OWN rim faces instead of standing square on the skin (+Y half, mirrored): a rim
# face of the reference that looks into one of these boxes (its centre moved RIM_LOOK along its normal lies in the box) makes
# the cell at that edge reach RIM_REACH past the edge over the opening; the envelope then cuts it to the reference's rim face.
#   name: (lo, hi, normal lo, normal hi)   the rim face's normal (Y as on the +Y half) must lie within normal lo..hi per axis
RIM_AS_REFERENCE = {
    "windscreen":      ((4.0, 0.0, 33.0), (17.6, 20.6, 43.6), (-1, -1, -1), (1, 1, 1)),   # roof front edge X 4.2, cowl edge X 17.1, A-pillar inner rims |Y| 20.5
    "side window top": ((-21.0, 20.8, 32.0), (17.0, 26.0, 43.0), (-1, -0.2, -1), (1, 0.2, -0.6)),  # roof-rail and A-pillar underside rims (not their inboard undersides)
}
RIM_LOOK = 0.5             # mm
RIM_REACH = 1.0            # mm: more than the reference's rim leans off square over the wall (0.67 at most)
RIM_DEEP = WALL + 0.2      # mm: how far down the rim face (below the reference's own wall) the space in front of it is kept clear

# Points inside the body cavity (under hood, cab and bed floor line). A reference face that one of them can see is the INSIDE
# of the wall and gets no skin plane. Grid: X, Y, Z stations.
CAVITY_X = np.arange(-64.0, 62.0, 3.0)
CAVITY_Y = (-14.0, -7.0, 0.0, 7.0, 14.0)
CAVITY_Z = (8.0, 13.0, 18.0)
HIDDEN_SHARE = float(os.environ.get("HIDDEN_SHARE", "0.7"))   # a face hidden from those points is still inside when this share of what it faces is inside (above 1: rule off)

# Regions (+Y half, mirrored) where the reference is exterior on BOTH sides: every face there is skin, whatever the cavity sees.
#   name: (lo, hi)
BOTH_SIDES = {
    "rear bars, fins, bed rails": ((-70.0, 0.0, 28.5), (-24.5, 40.0, 50.0)),
    "nose chin":                  ((59.5, 0.0, 21.0), (67.0, 22.5, 24.45)),
}

# Material taken straight from the reference inside these boxes (+Y half, mirrored), not rebuilt from planes.
#   name: (lo, hi, mode)   mode "replace": the plane-built shell is cleared inside the box first (ZONE_LAP short of its faces);
#                          "patch": cleared exactly to the box faces; "add": fused on top of it;
#                          "trim": nothing added, the shell is only cut back to the reference solid inside the box
AS_REFERENCE = {
    "rear bars, fins, bed rails": ((-70.0, 0.0, 28.5), (-24.5, 40.0, 50.0), "replace"),   # solid bars and rails, exterior all round: no wall to level
    "rear tilt-hinge lug":        ((-68.5, 18.0, 26.0), (-56.5, 27.5, 29.0), "replace"),  # pin axis along Y at X -64, Z 29; lug block runs to X -56.7
    "nose chin":                  ((59.5, 0.0, 21.0), (67.0, 22.5, 24.45), "replace"),    # solid bumper lip, its underside is exterior
    "front magnet boss":          ((51.0, 1.0, 20.0), (63.5, 14.0, 36.0), "add"),         # seats on the front bulkhead
    # small local fall-backs to the reference, where the plane build leaves a visible flaw ("patch": the box is cleared exactly
    # and the reference piece set in flush, no overlap band, because the two builds differ on the inside at the box faces):
    "rear flare forked end":      ((-46.5, 25.9, 22.2), (-35.5, 29.5, 28.6), "patch"),
    "nose corner":                ((60.0, 14.5, 24.4), (65.0, 21.0, 25.6), "patch"),
    "A-pillar base in arch":      ((16.4, 24.3, 21.3), (22.7, 26.2, 27.6), "patch"),
    "front flare front end":      ((49.5, 26.2, 26.3), (51.3, 28.0, 28.4), "patch"),
    "front fender edge at nose":  ((54.2, 19.0, 24.3), (60.0, 22.3, 28.4), "patch"),
    "under-hood film":            ((53.6, 20.8, 25.1), (55.2, 21.9, 26.5), "patch"),
    "arch ceiling junction":      ((35.5, 22.0, 27.9), (41.5, 24.2, 30.7), "patch"),
    "rear arch front lip":        ((-36.6, 25.5, 19.3), (-27.8, 32.2, 22.9), "patch"),
    "rear box lower face":        ((-70.0, 0.0, 28.44), (-24.4, 40.0, 28.56), "trim"),      # the overlap band of the rear-bars box: nothing outside the reference may stay in it
    "rear box front face":        ((-24.56, 0.0, 28.44), (-24.44, 40.0, 50.0), "trim"),
    "roof rear edge":             ((-22.9, 0.0, 43.3), (-21.3, 21.2, 46.3), "patch"),
    "B-pillar and top rail":      ((-25.5, 20.3, 30.0), (-20.2, 26.6, 46.3), "patch"),
}
ZONE_LAP = 0.03           # mm: a "replace" box is cleared this much short of its faces, so the two builds overlap instead of touching


def log(*a):
    print(*a); sys.stdout.flush()


# ---------------------------------------------------------------- 1. skin planes from the reference
def load_reference():
    """The reference, without its zero-area faces (they have no usable normal, so they cannot carry a skin plane)."""
    ref = trimesh.load(REF_STL, process=True)
    V, F = L.clean_slivers(ref.vertices, ref.faces, alt_min=REF_ALT_MIN, short=REF_SHORT)
    out = trimesh.Trimesh(V, F, process=False); out.remove_unreferenced_vertices()
    return out


def classify_skin(ref):
    """Per reference face: True = exterior skin (gets a skin plane). A face is INSIDE when a cavity point can see it, or (hidden
    ribs and pockets) when most of what it faces is inside; unless it looks out on open air (the underside of a lip, of the nose,
    of a flare: exterior although the cavity can see it through an arch) or lies in a both-sides box. The verdict is then made
    per flat facet, by area: one facet is never half skin and half inside."""
    src = np.array([(x, y, z) for x in CAVITY_X for y in CAVITY_Y for z in CAVITY_Z])
    src = src[~ref.contains(src)]
    inner = L.seen_from_inside(ref, src)
    if HIDDEN_SHARE <= 1.0: inner = L.looks_at_inside(ref, inner, share=HIDDEN_SHARE)
    # the underside of a lip: looks out on open air AND has the lip's other skin behind it. (The rim of an opening also looks out
    # on open air, but behind it the wall runs on lengthwise: it stays inside, so the rim is cut square to the skin.)
    idx = np.where(inner)[0]; fn = ref.face_normals; c = ref.triangles_center
    loc, ir, it = ref.ray.intersects_location(c[idx] - fn[idx] * L.EPS_START, -fn[idx], multiple_hits=False)
    backed = np.zeros(len(idx), bool)
    backed[ir] = ~inner[it] & (np.einsum("ij,ij->i", fn[it], -fn[idx][ir]) > AWAY_DOT) & (np.linalg.norm(loc - c[idx][ir], axis=1) < LIP_MAX_THICK)
    inner[idx[backed & (L.open_share(ref, idx, CAVITY_FLOOR) >= LIP_OPEN_SHARE)]] = False
    # a face from which no direction at all reaches open air (cavity floor closed) cannot be exterior: a hidden slot or pocket
    idx = np.where(~inner)[0]; inner[idx[L.open_share(ref, idx, CAVITY_FLOOR, count=96) <= 0.0]] = True
    fid, _, _ = L.facet_groups(ref, np.ones(len(ref.faces), bool), PLANE_TOL, PLANE_ANGLE, REF_ALT_MIN)
    ok = fid >= 0; A = ref.area_faces
    share = np.bincount(fid[ok], A[ok] * inner[ok]) / np.maximum(np.bincount(fid[ok], A[ok]), 1e-12)
    inner[ok] = share[fid[ok]] >= 0.5
    inner &= ~L.sym_box_mask(ref.triangles_center, list(BOTH_SIDES.values()))
    return ~inner


# ---------------------------------------------------------------- 2. wall cells
def own_part(body, env, normals, probe):
    """A cell or fill cut to the envelope, keeping only what is still joined to its own skin (probe = a sliver of the body at its
    facet, crease or corner). The envelope is the reference solid plus the cavity layers of the thin wall faces that face AWAY
    from the body's plane(s): a wall grows into the cavity behind its own skin, not sideways behind a neighbouring face.
    A cell that leaves the envelope through a gap and comes back into other reference material further on would leave a loose
    plate there: that part is dropped. Returns the kept body and the number of parts dropped."""
    solid, layers, ln, lo, hi = env
    bb = np.array(body.bounding_box()); near = np.all(lo <= bb[3:], axis=1) & np.all(hi >= bb[:3], axis=1)
    near &= np.min([ln @ n for n in normals], axis=0) < -AWAY_DOT
    r = body ^ solid
    if near.any(): r = r + (body ^ L.union([layers[i] for i in np.where(near)[0]]))
    parts = r.decompose()
    if len(parts) <= 1: return (None if r.is_empty() else r), 0
    keep = [q for q in parts if not (q ^ probe).is_empty()]
    return (L.union(keep) if keep else None), len(parts) - len(keep)


def wall_cells(ref, gid, nrm, use, env):
    """One prism per skin plane: the facet pushed inward by WALL, its sides square on the facet; cut to the envelope."""
    V = ref.vertices; faces = np.where(use)[0]; cells = []; n_tri = 0; n_drop = 0
    for g in np.unique(gid[faces]):
        fs = faces[gid[faces] == g]
        p = L.patch_prism(V, ref.faces[fs], nrm[g], WALL); top = L.patch_prism(V, ref.faces[fs], nrm[g], OWN_PROBE)
        if p is None or top is None:
            p = L.union(L.tri_prisms(V, ref.faces[fs], nrm[g], WALL)); top = L.union(L.tri_prisms(V, ref.faces[fs], nrm[g], OWN_PROBE)); n_tri += len(fs)
        p, d = own_part(p, env, [nrm[g]], top); n_drop += d
        if p is not None: cells.append(p)
    return cells, n_tri, n_drop


def crease_fills(ref, gid, nrm, use, env):
    """A wedge along every concave crease (valley) between two skin facets, and a small hull at every vertex such a crease runs into;
    cut to the envelope."""
    V = ref.vertices; adj = ref.face_adjacency; ed = ref.face_adjacency_edges; unsh = ref.face_adjacency_unshared
    out = []; vert_pts = {}; vert_n = {}; n_drop = 0
    sel = np.where(use[adj[:, 0]] & use[adj[:, 1]] & (gid[adj[:, 0]] != gid[adj[:, 1]]))[0]
    for k in sel:
        f1, f2 = adj[k]; a, b = V[ed[k]]; n1, n2 = nrm[gid[f1]], nrm[gid[f2]]; e = (b - a) / np.linalg.norm(b - a)
        t1 = np.cross(n1, e); t1 *= np.sign(t1 @ (V[unsh[k][0]] - a)) or 1.0
        if not t1 @ n2 > 0.0: continue                         # a ridge: the two cells overlap by themselves (exact mitre), nothing to add
        t2 = np.cross(n2, e); t2 *= np.sign(t2 @ (V[unsh[k][1]] - a)) or 1.0
        w = L.crease_wedge(a, b, n1, t1, n2, t2, WALL)         # a valley: facet 1 rises in front of plane 2
        if w is not None:
            w, d = own_part(w, env, [n1, n2], L.beads([(a, b)], OWN_PROBE)[0]); n_drop += d
            if w is not None: out.append(w)
        mid = L.valley_offsets(n1, n2, WALL)
        for v in ed[k]:
            vert_pts.setdefault(int(v), [V[v]]).extend([V[v] - n1 * WALL, V[v] - n2 * WALL] + [V[v] + m for m in mid])
            vert_n.setdefault(int(v), []).extend([n1, n2])
    n_edge = len(out)
    for v, pts in vert_pts.items():
        h = L.hull(pts)
        if h is None: continue
        h, d = own_part(h, env, vert_n[v], L.box(V[v] - OWN_PROBE, V[v] + OWN_PROBE)); n_drop += d
        if h is not None: out.append(h)
    return out, n_edge, len(vert_pts), n_drop


def rim_reaches(ref, gid, nrm, use, skin, env):
    """At the openings in RIM_AS_REFERENCE: along every edge where a skin facet meets a reference rim face that looks into the
    opening, a slab of the facet's wall reaching RIM_REACH past the edge (and a hull at each end of such an edge), cut to the
    reference solid alone (no cavity layers: they are swept round their edges and would reach out over the opening): the rim
    then lies on the reference's own rim face, whatever its angle to the skin, as deep as the reference wall goes."""
    V = ref.vertices; fn = ref.face_normals; adj = ref.face_adjacency; ed = ref.face_adjacency_edges; unsh = ref.face_adjacency_unshared
    env = (env[0], [], np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 3)))
    looks = np.zeros(len(fn), bool); nh = fn * np.where(ref.triangles_center[:, [1]] < 0, [1, -1, 1], 1)   # normal as on the +Y half
    for lo, hi, nlo, nhi in RIM_AS_REFERENCE.values():
        looks |= L.sym_box_mask(ref.triangles_center + fn * RIM_LOOK, [(lo, hi)]) & np.all((nh >= nlo) & (nh <= nhi), axis=1)
    out = []; ends = {}; n_edge = 0; n_drop = 0; mids = []; mid_n = []
    for k in np.where(ref.face_adjacency_convex)[0]:
        for i in (0, 1):
            f, r = adj[k][i], adj[k][1 - i]
            if not (use[f] and not skin[r] and looks[r]): continue
            n = nrm[gid[f]]
            if fn[r] @ n >= RIM_MAX_DOT: continue
            a, b = V[ed[k]]; e = (b - a) / np.linalg.norm(b - a)
            t = np.cross(n, e); t *= -(np.sign(t @ (V[unsh[k][i]] - a)) or 1.0)       # in the facet's plane, away from the facet
            s = [a, b, a + t * RIM_REACH, b + t * RIM_REACH]
            w = L.hull(s + [p - n * WALL for p in s])
            if w is not None:
                w, d = own_part(w, env, [n], L.beads([(a, b)], OWN_PROBE)[0]); n_drop += d
                if w is not None: out.append(w); n_edge += 1
            for v in ed[k]: ends.setdefault((int(v), int(gid[f])), [V[v]]).append(V[v] + t * RIM_REACH)
            mids.append((a + b) / 2); mid_n.append(n)
    for (v, g), pts in ends.items():
        n = nrm[g]; h = L.hull(pts + [p - n * WALL for p in pts])
        if h is None: continue
        h, d = own_part(h, env, [n], L.box(V[v] - OWN_PROBE, V[v] + OWN_PROBE)); n_drop += d
        if h is not None: out.append(h)
    # below the reference's wall the cells stand square on the skin and may lean out past the rim face: the space in front of
    # every such rim face (RIM_REACH deep, carried RIM_DEEP down the face, i.e. along minus the nearest rim edge's skin normal
    # laid into the rim face) is cleared of what is not reference material
    cut = []; mids = np.array(mids).reshape(-1, 3)
    for r in (np.where(looks & ~skin & (ref.triangles[:, :, 1].max(axis=1) > -HALF_MARGIN))[0] if len(mids) else []):
        ns = mid_n[int(np.argmin(np.linalg.norm(mids - ref.triangles_center[r], axis=1)))]
        down = -(ns - fn[r] * (ns @ fn[r])); ln = np.linalg.norm(down)
        if ln < 1e-6: continue
        tri = list(V[ref.faces[r]]); tri += [p + down / ln * RIM_DEEP for p in tri]
        cut.append(L.hull(tri + [p + fn[r] * RIM_REACH for p in tri]))
    return out, n_edge, n_drop, L.union(cut) - env[0]


def envelope(ref, skin):
    """Where wall material may be: the reference solid itself, plus a layer CAVITY_LAYER deep in front of the reference's inside
    wall faces where its wall is thinner than WALL (there the new wall grows into the cavity). Everything else is outside air:
    a cell that comes out through skin on the far side of a thin lip or bar is cut off there, so the lip keeps the reference
    thickness, and nothing can grow into an opening (a rim face is not a wall face: it gets no layer).
    Returns (solid, layers, their face normals, their box corners lo / hi), thin wall faces, inside faces."""
    V = ref.vertices; fn = ref.face_normals
    faces = np.where(~skin & (ref.triangles[:, :, 1].max(axis=1) > -HALF_MARGIN - CAVITY_LAYER))[0]
    tri = ref.triangles[faces]; c = tri.mean(axis=1)
    pts = np.concatenate([c[:, None, :], c[:, None, :] + 0.8 * (tri - c[:, None, :])], axis=1)            # 4 probes per face: centre and towards each corner
    o = (pts - fn[faces][:, None, :] * L.EPS_START).reshape(-1, 3); d = np.repeat(-fn[faces], 4, axis=0)
    loc, ir, it = ref.ray.intersects_location(o, d, multiple_hits=False)
    thin = (np.linalg.norm(loc - o[ir], axis=1) < WALL) & skin[it] & (np.einsum("ij,ij->i", fn[it], d[ir]) > 0.3)   # skin right behind, facing away
    wall_face = np.zeros(len(faces), bool); wall_face[ir[thin] // 4] = True
    # a layer is swept round the edges of its face. Where the face ends on a ridge of the reference solid against a face that is
    # not a thin wall face itself (the rim of an opening, the end of a lip), the layer stops at that neighbour's plane: the wall
    # grows into the cavity, never round a rim into the opening. At a sharp corner between two thin wall faces it stops too: each
    # has its own layer there, and a cell let into one must not wrap round the corner in front of the other
    is_wall = np.zeros(len(ref.faces), bool); is_wall[faces[wall_face]] = True
    stops = {}
    for (a, b), convex in zip(ref.face_adjacency, ref.face_adjacency_convex):
        if not convex: continue
        for f, r in ((a, b), (b, a)):
            if is_wall[f] and fn[f] @ fn[r] < (CORNER_MAX_DOT if is_wall[r] else RIM_MAX_DOT): stops.setdefault(int(f), []).append(int(r))
    layers = []; ln = []
    for f in faces[wall_face]:
        q = L.front_layer(V[ref.faces[f]], fn[f], CAVITY_LAYER)
        for r in stops.get(int(f), ()):
            if q is not None and not q.is_empty(): q = q.trim_by_plane(tuple(-fn[r]), -float(fn[r] @ V[ref.faces[r][0]]) - L.LAYER_LAP)
        if q is not None and not q.is_empty(): layers.append(q); ln.append(fn[f])
    bb = np.array([q.bounding_box() for q in layers]).reshape(-1, 6)
    return (L.to_manifold(ref), layers, np.array(ln).reshape(-1, 3), bb[:, :3], bb[:, 3:]), int(wall_face.sum()), len(faces)


def drop_leftovers(half, solid):
    """Thin leftovers. Take the plane-built material that lies OUTSIDE the reference solid and split it into connected pieces.
    A piece is real wall growth when it lies flat against the reference (a slab on the inside of a thin wall: a large share of
    its surface is contact). A piece that is small or plate-like AND only hangs on by an edge is a leftover of a cell that the
    envelope admitted in part: it is removed. Returns the body and the pieces removed as (volume, bounding box)."""
    gone = []; cut = []
    for q in (half - solid).as_original().simplify(L.SIMPLIFY_TOL).decompose():      # (merged first: the raw difference is full of zero-volume skins)
        v, a = q.volume(), q.surface_area()
        if v <= 0 or (v >= LEFTOVER_VOL and v / a >= LEFTOVER_PLATE): continue
        bb = np.array(q.bounding_box()); near = solid ^ L.box(bb[:3] - 0.2, bb[3:] + 0.2)
        contact = (a + near.surface_area() - (q + near).surface_area()) / 2.0
        if contact / a < LEFTOVER_CONTACT: cut.append(q); gone.append((v, np.round(bb, 1).tolist()))
    return (half - L.union(cut) if cut else half), gone


# ---------------------------------------------------------------- 3. functional features from the reference
def apply_reference_zones(half, ref):
    """Bars/fins, hinge lugs and magnet bosses: the reference material inside the named boxes, taken as it is."""
    rm = L.to_manifold(ref)
    for name, (lo, hi, mode) in AS_REFERENCE.items():
        if lo[1] <= 0.0: lo = (lo[0], -2 * HALF_MARGIN, lo[2])                    # a box that starts on the centre plane covers the margin too
        if mode == "trim":                                        # nothing is added: only what lies outside the reference solid is cut away
            cut = L.box(lo, hi) - rm; half = half - cut
            log("   as reference: %-24s %-7s -%.2f mm3 box volume outside the reference cleared" % (name, mode, cut.volume())); continue
        piece = rm ^ L.box(lo, hi)
        if mode == "replace": half = half - L.box(np.add(lo, ZONE_LAP), np.subtract(hi, ZONE_LAP))
        if mode == "patch": half = half - L.box(lo, hi)
        half = half + piece
        log("   as reference: %-24s %-7s %.1f mm3" % (name, mode, piece.volume()))
    return half


# ---------------------------------------------------------------- build
def build():
    t0 = time.time()
    ref = load_reference()
    skin = classify_skin(ref)
    gid, nrm, n_flat = L.facet_groups(ref, skin, PLANE_TOL, PLANE_ANGLE, REF_ALT_MIN); skin = skin & (gid >= 0)
    A = ref.area_faces
    big = np.bincount(gid[skin], A[skin]); log("skin: %d faces %.0f mm2 of %.0f; skin planes %d (%d over 5 mm2)  [%.1f s]" % (skin.sum(), A[skin].sum(), A.sum(), len(nrm), (big > 5).sum(), time.time() - t0))
    use = skin & (ref.triangles[:, :, 1].max(axis=1) > -HALF_MARGIN)
    env, n_layer, n_inside = envelope(ref, skin)
    cells, n_tri, n_far1 = wall_cells(ref, gid, nrm, use, env)
    fills, n_edge, n_vert, n_far2 = crease_fills(ref, gid, nrm, use, env)
    log("half: %d cells (%d faces built per triangle), %d valley wedges, %d vertex hulls, each cut to the envelope (reference solid + cavity layer on %d of %d inside faces); %d parts that had left their own wall dropped  [%.1f s]" % (len(cells), n_tri, n_edge, n_vert, n_layer, n_inside, n_far1 + n_far2, time.time() - t0))
    reach, n_rim, n_far3, rim_cut = rim_reaches(ref, gid, nrm, use, skin, env)
    half = L.union(cells + fills + reach); v0 = half.volume(); half = half - rim_cut
    log("half: %d rim edges follow the reference's own rim face (%s); %.3f mm3 in front of those rim faces cleared; %d parts dropped" % (n_rim, ", ".join(RIM_AS_REFERENCE), v0 - half.volume(), n_far3))
    half, left = drop_leftovers(half, L.to_manifold(ref))
    log("half: %d thin leftovers outside the reference removed (%.2f mm3 in all)" % (len(left), sum(v for v, _ in left)))
    for v, bb in left:
        if v >= 0.02 and bb[1] >= -0.5: log("   leftover %.3f mm3 at X %.1f..%.1f Y %.1f..%.1f Z %.1f..%.1f" % (v, bb[0], bb[3], bb[1], bb[4], bb[2], bb[5]))
    if not ONLY_CORE:
        half = apply_reference_zones(half, ref)
    half, n_bead = L.fuse_pinches(half, BEAD)
    half = half.trim_by_plane((0.0, 1.0, 0.0), 0.0)
    half, gone, n_pocket = L.main_part(half)
    log("half: %d edge contacts fused with a %.2f mm bead; %d closed pockets filled; %d loose parts dropped (%.3f mm3 in all)" % (n_bead, BEAD, n_pocket, len(gone), sum(v for v, _ in gone)))
    for v, bb in gone:
        if v >= MIN_PART: log("   DROPPED loose part %.2f mm3 at X %.1f..%.1f Y %.1f..%.1f Z %.1f..%.1f (hung on reference material that is not skin)" % (v, bb[0], bb[3], bb[1], bb[4], bb[2], bb[5]))
    for _ in range(10):                                     # the mesh clean-up can close a hair gap into an edge contact: fuse it and redo
        V, F = L.half_mesh(half); touch = L.overfull_edges(V, F)
        if not touch: break
        half = L.union([half] + L.beads(touch, BEAD)).trim_by_plane((0.0, 1.0, 0.0), 0.0); n_bead += len(touch)
        log("   %d edge contacts after clean-up, fused: %s" % (len(touch), "; ".join("(%.2f %.2f %.2f)-(%.2f %.2f %.2f)" % (*a, *b) for a, b in touch[:3])))
    full = L.weld_mirror(V, F)
    full, n_sheet, n_void, n_bad = L.drop_sheets_and_voids(full, MIN_PART)
    log("shell: %d faces, watertight %s, bodies %d, volume %.1f mm3; %d sheets and %d pockets removed after the weld; %d edges with more than two faces  [%.1f s]" % (len(full.faces), full.is_watertight, full.body_count, full.volume, n_sheet, n_void, n_bad, time.time() - t0))
    os.makedirs(os.path.dirname(os.path.abspath(OUT_STL)), exist_ok=True)
    full.export(OUT_STL)
    log("wrote", OUT_STL)


if __name__ == "__main__":
    build()
