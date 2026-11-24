"""Helpers for scripts/24_shell_planes.py: a body shell built from the planes of a reference exterior.

Idea (Project_Fugu method, done with numpy + manifold3d instead of CadQuery):
  * the exterior is a table of SKIN PLANES, one per flat facet of the reference exterior (extracted, not typed);
  * every skin plane gets a WALL CELL: the facet outline pushed inward by the wall, i.e. the slab between the plane and the
    same plane moved inward by WALL. A cell is a prism (a union of convex prisms for a non-convex facet);
  * creases: at a convex crease the two cells overlap (exact mitre); at a concave crease a convex wedge closes the gap up to
    the line where the two inner planes meet (exact mitre);
  * the shell is the exact boolean union of all cells. Openings are simply where there is no skin plane; because the side of
    a cell stands square on its plane, every opening rim is a planar band at 90 degrees to the skin, through the wall;
  * where the reference is exterior on both sides and thinner than the wall (lips, bars), a cell is cut by the opposite skin
    plane, so the lip keeps the reference thickness.
Nothing is offset after the fact and nothing is sampled into the geometry: rays are only used to decide which reference
faces are exterior skin and which opposite plane clips a thin lip.
"""
import numpy as np, trimesh, manifold3d as m3
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

EPS_START = 1e-4          # mm: ray start offset off a surface
SAME_POINT = 1e-4         # mm: corners nearer than this are one point
SIMPLIFY_TOL = 0.01       # mm: triangles left by the union are merged and edges shorter than this collapsed (surface moves by less than this)
LAYER_LAP = 0.02          # mm: overlap of a cavity layer with the solid behind it
MITRE_MAX_DEG = 100.0     # valleys up to this angle get a sharp inner mitre, sharper ones an arc (see valley_offsets)
ARC_STEP_DEG = 20.0
FILL_LAP = 0.05           # mm: how far a crease wedge reaches into the two cells it joins (overlap, never face-to-face)


# ---------------------------------------------------------------- small helpers
def to_manifold(mesh):
    return m3.Manifold(m3.Mesh64(vert_properties=np.asarray(mesh.vertices, np.float64), tri_verts=np.asarray(mesh.faces, np.uint64)))


def to_trimesh(man):
    q = man.to_mesh64()
    return trimesh.Trimesh(np.asarray(q.vert_properties)[:, :3], np.asarray(q.tri_verts), process=False)


def union(parts):
    parts = [p for p in parts if p is not None and not p.is_empty()]
    return m3.Manifold.batch_boolean(parts, m3.OpType.Add) if parts else m3.Manifold()


def hull(points):
    h = m3.Manifold.hull_points(np.asarray(points, np.float64))
    return None if h.is_empty() or h.volume() < 1e-9 else h


def box(lo, hi):
    return m3.Manifold.cube(np.subtract(hi, lo)).translate(lo)


def in_box(p, b):
    lo, hi = np.array(b[0]), np.array(b[1])
    return np.all((p >= lo) & (p <= hi), axis=1)


def sym_box_mask(p, boxes):
    """True for points inside any of the boxes, boxes given for the +Y half and applied to both halves."""
    q = p.copy(); q[:, 1] = np.abs(q[:, 1])
    out = np.zeros(len(p), bool)
    for b in boxes: out |= in_box(q, b)
    return out


# ---------------------------------------------------------------- which reference faces are exterior skin
def seen_from_inside(mesh, sources):
    """True for faces whose front can be seen from one of the points inside the body cavity: those are the INSIDE of the wall."""
    rt = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh)
    c, n = mesh.triangles_center, mesh.face_normals
    seen = np.zeros(len(c), bool)
    for s in sources:
        d = c - s; d /= np.linalg.norm(d, axis=1)[:, None]
        idx = np.where(((n * d).sum(1) < -0.05) & ~seen)[0]
        if len(idx):
            hit = rt.intersects_first(np.repeat(s[None], len(idx), 0), d[idx])
            seen[idx[hit == idx]] = True
    return seen


def cone_dirs(n, half_angle_deg, count):
    """count unit directions spread evenly inside the cone of half_angle_deg about each normal n (rows)."""
    i = np.arange(count) + 0.5; cz = 1 - i / count * (1 - np.cos(np.radians(half_angle_deg))); r = np.sqrt(1 - cz ** 2); ph = np.pi * (1 + 5 ** 0.5) * i
    loc = np.c_[r * np.cos(ph), r * np.sin(ph), cz]                                    # around +Z
    a = np.where(np.abs(n[:, [2]]) < 0.9, [[0.0, 0.0, 1.0]], [[1.0, 0.0, 0.0]]); u = np.cross(a, n); u /= np.linalg.norm(u, axis=1)[:, None]; v = np.cross(n, u)
    return loc[None, :, 0, None] * u[:, None, :] + loc[None, :, 1, None] * v[:, None, :] + loc[None, :, 2, None] * n[:, None, :]


def looks_at_inside(mesh, inner, half_angle_deg=70.0, count=24, share=0.5, rounds=4):
    """Faces hidden from the cavity points (ribs, pockets behind bosses) are still inside if what they face is the inside of the wall:
    a face becomes inner when at least `share` of the rays in a cone about its normal land on inner faces. Repeated until stable."""
    rt = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh); c, n = mesh.triangles_center, mesh.face_normals
    inner = inner.copy()
    for _ in range(rounds):
        idx = np.where(~inner)[0]
        d = cone_dirs(n[idx], half_angle_deg, count).reshape(-1, 3); o = np.repeat(c[idx] + n[idx] * 1e-3, count, axis=0)
        hit = rt.intersects_first(o, d); on_inner = (hit >= 0) & inner[np.maximum(hit, 0)]
        new = idx[on_inner.reshape(-1, count).mean(axis=1) >= share]
        if not len(new): break
        inner[new] = True
    return inner


def open_share(mesh, faces, floor, half_angle_deg=85.0, count=48):
    """Per face: the share of directions in front of it that reach open air, with the cavity's open underside closed by a flat sheet
    (floor = two opposite corners of the sheet, same Z)."""
    (x0, y0, z), (x1, y1, _) = floor
    sheet = trimesh.Trimesh([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]], [[0, 1, 2], [0, 2, 3]], process=False)
    both = trimesh.util.concatenate([mesh, sheet]); c, n = mesh.triangles_center[faces], mesh.face_normals[faces]
    hit = both.ray.intersects_any(np.repeat(c + n * 0.02, count, axis=0), cone_dirs(n, half_angle_deg, count).reshape(-1, 3))
    return 1.0 - hit.reshape(-1, count).mean(axis=1)


def facet_groups(mesh, skin, plane_tol, angle_tol_deg, alt_min=0.0):
    """Group skin faces into flat facets = skin planes. Largest reference facet first; a neighbouring skin face joins a plane when all
    its corners lie within plane_tol of it and its normal is within angle_tol_deg (a triangle flatter than alt_min has no usable normal:
    it joins on the distance test alone and never starts a plane). Returns gid per face (-1: not skin), unit normals, number of faces left out."""
    V, F, fn = mesh.vertices, mesh.faces, mesh.face_normals
    nb = [[] for _ in range(len(F))]
    for a, b in mesh.face_adjacency:
        if skin[a] and skin[b]: nb[a].append(b); nb[b].append(a)
    gid = np.full(len(F), -1); normals = []; cosmin = np.cos(np.radians(angle_tol_deg))
    tri = V[F]; longest = np.linalg.norm(tri - np.roll(tri, -1, axis=1), axis=2).max(axis=1)
    flat = 2 * mesh.area_faces < alt_min * longest
    order = np.argsort(-mesh.area_faces)
    for seed in order:
        if not skin[seed] or gid[seed] >= 0 or flat[seed]: continue
        n = fn[seed]; d = float(n @ V[F[seed]].mean(axis=0)); g = len(normals); gid[seed] = g; todo = [seed]
        while todo:
            f = todo.pop()
            for h in nb[f]:
                if gid[h] < 0 and (flat[h] or fn[h] @ n > cosmin) and np.abs(V[F[h]] @ n - d).max() <= plane_tol:
                    gid[h] = g; todo.append(h)
        normals.append(n)
    return gid, np.array(normals), int((skin & (gid < 0)).sum())


# ---------------------------------------------------------------- wall cells
def patch_prism(V, faces, n, depth, outward=False):
    """The facet (triangles `faces` into V) pushed inward by depth along -n: one closed prism, its sides square on the facet.
    With outward=True the facet is swept to the air side instead: n is then any direction that leaves through the facet's back
    (the sweep runs along -n), which gives the space a cell coming through this facet along -n would occupy."""
    ids, inv = np.unique(faces, return_inverse=True); F = inv.reshape(-1, 3); k = len(ids)
    top = V[ids]; bot = top - n * depth
    und = {}
    for a, b in F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2): und[(a, b)] = 1
    side = []
    for (a, b) in und:
        if (b, a) not in und: side += [(a, a + k, b + k), (a, b + k, b)]
    tris = np.vstack([F, F[:, ::-1] + k] + ([np.array(side)] if side else []))
    if outward: tris = tris[:, ::-1]
    man = m3.Manifold(m3.Mesh64(vert_properties=np.vstack([top, bot]), tri_verts=tris.astype(np.uint64)))
    return man if man.status() == m3.Error.NoError and not man.is_empty() else None


def tri_prisms(V, faces, n, depth, normals=None):
    """One prism per triangle, pushed by depth along -n (or along minus its own row of `normals`). Used when a facet cannot be
    pushed as one prism (its outline touches itself) and for loose faces."""
    out = [hull(np.vstack([V[f], V[f] - (n if normals is None else normals[i]) * depth])) for i, f in enumerate(faces)]
    return [q for q in out if q is not None]


_BALL = trimesh.creation.icosphere(subdivisions=1).vertices / 0.9      # 42 points; the hull contains the unit ball


def front_layer(tri, n, r):
    """Everything within r of the triangle on its front side (the side its normal n points to): hull of the triangle swept by a
    ball, cut at the triangle's own plane (LAYER_LAP behind it, so it overlaps what lies behind)."""
    h = hull((tri[:, None, :] + _BALL[None, :, :] * r).reshape(-1, 3))
    return None if h is None else h.trim_by_plane(tuple(n), float(n @ tri[0]) - LAYER_LAP)


def valley_offsets(n1, n2, depth):
    """Where the inner surface runs under a valley between planes with normals n1, n2: offsets from a point of the crease.
    Up to MITRE_MAX_DEG between the planes the two inner planes simply meet (one point: the mitre). A sharper valley would put that
    meeting line far below the skin and leave a knife-edged fin on the inside, so there the inner corner is the arc of radius
    `depth` about the crease, in steps of at most ARC_STEP_DEG (wall never under depth * cos(ARC_STEP_DEG / 2))."""
    c = float(np.clip(n1 @ n2, -1.0, 1.0)); phi = np.degrees(np.arccos(c))
    if phi <= MITRE_MAX_DEG: return [-(n1 + n2) * depth / (1.0 + c)]
    k = int(np.ceil(phi / ARC_STEP_DEG)); out = []
    for i in range(1, k):
        t = i / k; d = np.sin((1 - t) * np.radians(phi)) * n1 + np.sin(t * np.radians(phi)) * n2
        out.append(-d / np.linalg.norm(d) * depth)
    return out


def crease_wedge(a, b, n1, t1, n2, t2, depth):
    """Convex wedge along the valley a-b between two cells: from the crease line down to both inner planes and to where the inner
    surface runs between them (valley_offsets). It laps FILL_LAP into both cells, so nothing is joined face to face."""
    pts = [a, b]; mid = valley_offsets(n1, n2, depth)
    for p in (a, b):
        pts += [p - n1 * depth + t1 * FILL_LAP, p - n2 * depth + t2 * FILL_LAP] + [p + m for m in mid]
    return hull(pts)


def clean_slivers(V, F, area_min=1e-6, short=2e-3, passes=40, alt_min=0.0):
    """Remove the zero-area triangles an exact boolean leaves on coincident faces, without opening the mesh:
    a needle (an edge shorter than `short`) has that edge collapsed; a cap (a corner lying on the opposite edge) is removed by
    splitting the neighbour across that edge at the corner. No surface moves by more than `short`. Vertices on Y = 0 stay there."""
    V = V.copy(); F = F.copy()
    for _ in range(passes):
        tri = V[F]; e = np.linalg.norm(tri - np.roll(tri, -1, axis=1), axis=2)            # e[:, i] = edge i -> i+1
        area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        bad = np.where((area < area_min) | (2 * area < alt_min * e.max(axis=1)))[0]           # zero area, or lower than alt_min over its longest edge
        if not len(bad):
            key = np.sort(F, axis=1); _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
            F = F[cnt[inv.ravel()] == 1]; break
        edge_face = {}
        for i, f in enumerate(F):
            edge_face[(f[0], f[1])] = i; edge_face[(f[1], f[2])] = i; edge_face[(f[2], f[0])] = i
        ring = {}
        for f in F:
            for p_, q_ in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])): ring.setdefault(p_, set()).add(q_); ring.setdefault(q_, set()).add(p_)
        remap = np.arange(len(V)); locked = set(); touched = np.zeros(len(F), bool); drop = np.zeros(len(F), bool); new = []
        for i in bad:
            if touched[i]: continue
            k = int(np.argmin(e[i])); a, b = F[i][k], F[i][(k + 1) % 3]
            if e[i][k] < short:                                                            # needle: merge b into a (keep the one on Y = 0)
                # safe only if a and b share exactly the two corners opposite this edge (else the collapse pinches the surface)
                # (two corners at the same point are merged anyway; the flap that leaves is removed below)
                if a in locked or b in locked: continue
                if len(ring[a] & ring[b]) == 2 or e[i][k] <= SAME_POINT:
                    if abs(V[b][1]) < 1e-9 and abs(V[a][1]) >= 1e-9: a, b = b, a
                    remap[b] = a; touched[i] = True; locked |= ring[a] | ring[b]; continue
                # else: not safe to collapse here; split the neighbour instead, which moves the needle to where it can be
            k = int(np.argmax(e[i])); a, b, c = F[i][k], F[i][(k + 1) % 3], F[i][(k + 2) % 3]
            j = edge_face.get((b, a))
            if j is None or touched[j] or j == i or a in locked or b in locked or c in locked: continue
            d = [v for v in F[j] if v != a and v != b][0]
            if d == c or d in locked: continue
            touched[i] = touched[j] = True; drop[i] = drop[j] = True; new += [(c, a, d), (b, c, d)]
        F = np.vstack([F[~drop]] + ([np.array(new)] if new else []))
        F = remap[F]; F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
        key = np.sort(F, axis=1); _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        F = F[cnt[inv.ravel()] == 1]                                                       # a flap: two faces on the same three corners, back to back
    return V, F


def overfull_edges(V, F):
    """End points of edges that carry more than two faces: places where the body only touches itself along a line."""
    e = np.sort(F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1); u, cnt = np.unique(e, axis=0, return_counts=True)
    return [(V[a], V[b]) for a, b in u[cnt > 2]]


def half_mesh(half):
    """Closed +Y half (cut at Y = 0) as a clean triangle mesh: coplanar triangles merged, corners stored as the STL will store them
    (32-bit), corners at one point made one corner, zero-area triangles removed. Returns V, F."""
    t = to_trimesh(half.as_original().simplify(SIMPLIFY_TOL)); V = t.vertices.astype(np.float32).astype(np.float64); F = t.faces
    V[np.abs(V[:, 1]) < 1e-6, 1] = 0.0
    V, inv = np.unique(V, axis=0, return_inverse=True); F = inv.ravel()[F]
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    return clean_slivers(V, F)


def weld_mirror(V, F):
    """Half mesh -> full body, exactly mirrored: the faces on the cut are dropped and the mirrored copy is welded on the Y = 0 corners."""
    on = V[:, 1] == 0.0
    F = F[~on[F].all(axis=1)]
    idx = np.arange(len(V)); mir = np.where(on, idx, idx + len(V))
    full = trimesh.Trimesh(np.vstack([V, V * [1, -1, 1]]), np.vstack([F, mir[F][:, ::-1]]), process=False)
    full.remove_unreferenced_vertices()
    return full


def beads(edges, bead):
    """A thin 7-sided rod of radius `bead` along each edge (turned off-axis so that none of its faces lies in a body plane)."""
    out = []
    for a, b in edges:
        e = b - a; ln = np.linalg.norm(e)
        if ln < 1e-9: continue
        e /= ln; u = np.cross(e, [0.0, 0.0, 1.0] if abs(e[2]) < 0.9 else [1.0, 0.0, 0.0]); u /= np.linalg.norm(u); v = np.cross(e, u)
        ring = [np.cos(0.37 + 0.8976 * i) * u + np.sin(0.37 + 0.8976 * i) * v for i in range(7)]
        out.append(hull([a - e * bead] + [a + r * bead for r in ring] + [b + r * bead for r in ring] + [b + e * bead]))
    return [q for q in out if q is not None]


def pinch_edges(man):
    """Edges where the solid only touches itself (more than two faces on one line): list of (a, b) end points."""
    t = to_trimesh(man); V, inv = np.unique(np.round(t.vertices, 7), axis=0, return_inverse=True); F = inv.ravel()[t.faces]
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    return overfull_edges(V, F)


def fuse_pinches(man, bead, rounds=6):
    """Never leave two parts of the body touching on a bare edge: lay a bead of radius `bead` along every such edge and fuse it in.
    Returns the body and the number of beads."""
    total = 0
    for _ in range(rounds):
        man = man.as_original().simplify(SIMPLIFY_TOL)                 # merge what the union left coincident first: only real contacts remain
        edges = pinch_edges(man)
        if not edges: break
        bs = beads(edges, bead); man = union([man] + bs); total += len(bs)
    return man, total


def main_part(man):
    """Keep the one body: the largest solid part. Returns it, the other solid parts as (volume, bounding box) and the number of
    closed empty pockets filled (a pocket is a part with negative volume)."""
    parts = sorted(man.decompose(), key=lambda q: -q.volume())
    gone = [(q.volume(), np.round(q.bounding_box(), 1).tolist()) for q in parts[1:] if q.volume() > 0]
    return parts[0], gone, sum(1 for q in parts if q.volume() <= 0)


def drop_sheets_and_voids(mesh, min_volume):
    """Last hygiene on the welded body. Split it where it is joined properly (edges with exactly two faces); drop every piece whose
    volume is under min_volume: zero-volume sheets (two coincident faces back to back) and closed empty pockets (negative volume).
    Returns the mesh, sheets dropped, pockets dropped, edges that still carry more than two faces."""
    F = mesh.faces; V = mesh.vertices
    e = np.sort(F[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1); owner = np.repeat(np.arange(len(F)), 3)
    u, inv, cnt = np.unique(e, axis=0, return_inverse=True, return_counts=True); inv = inv.ravel()
    order = np.argsort(inv, kind="stable"); two = cnt[inv[order]] == 2; pair = owner[order][two].reshape(-1, 2)
    g = coo_matrix((np.ones(len(pair)), (pair[:, 0], pair[:, 1])), shape=(len(F),) * 2); _, lab = connected_components(g, directed=False)
    tri = V[F]; vol6 = np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])); vol = np.bincount(lab, vol6) / 6.0
    keep = vol[lab] >= min_volume
    out = trimesh.Trimesh(V, F[keep], process=False); out.remove_unreferenced_vertices()
    e2 = np.sort(out.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1); _, c2 = np.unique(e2, axis=0, return_counts=True)
    return out, int(((vol < min_volume) & (vol > -min_volume)).sum()), int((vol <= -min_volume).sum()), int((c2 != 2).sum())
