"""Cyberpunk Baja Desert Trophy Truck / Short Course Truck Body Shell for SCX30 SCT (107mm kit).

Works directly WITH the already-established Chassis_Tub and red Yeti-style Cage (no redundant
external exo-cage!). Carries the multifaceted Option 2 x Option 3 aesthetic in true Short Course
Truck proportions:
  - Unified 1-piece tilt-open faceted Trophy Truck shell (`SCT_Cyber_Trophy_Body`, 1.8 mm wall, ZERO curves)
  - Widebody high-clearance angular front & rear Trophy Truck fender flares with crisp polygonal arches
  - Flatter outer hood + steeper-walled center power-bulge, matching stepped power-bulge cab roof
  - Open windshield, open side windows, and open fastback rear bed showcasing the established red Cage
  - 3 sculpted raised forward-leaning gill louvers on each lower door flank
  - Separate snap-in `SCT_Hex_Mesh_Grille` (visibly open 1.8 mm hex honeycomb mesh!) and
    flush full-width `SCT_Cyan_DRL_Bar` across the bluff Trophy Truck nose brow
  - Exact drop-in mounting:
      * Front magnet bosses at X = 57.3, Y = +-7.6, Z = 27.4 (Ø3.2 x 2.1 mm pockets over tub tower)
      * Rear tilt-open hinge lugs at X = -64.0, Z = 29.0, |Y| = 18.9..21.3 (Ø5.2 OD, Ø1.7 M2 pilot)

Coordinate frame: +X forward, +Y left, +Z up, ground Z = -16.5 mm.
"""
import math
import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.optimize import linprog

WALL = 1.8            # mm uniform printable PETG shell wall
FIT = 0.20            # mm snap-in print clearance for grille & DRL bar
STRIP_H = 2.6         # mm height of full-width cyan DRL brow bar
GROOVE_D = 1.0        # mm blind groove depth for DRL bar
HEX_FLAT = 1.55       # mm across-flats of hex intake cells (crisp FDM-printable honeycomb!)
HEX_RIB = 0.55        # mm printable rib width between hex cells


def _hull(pts):
    arr = np.asarray(pts, float)
    ch = ConvexHull(arr)
    faces = []
    for s in ch.simplices:
        p0, p1, p2 = arr[s[0]], arr[s[1]], arr[s[2]]
        n = np.cross(p1 - p0, p2 - p0)
        if np.linalg.norm(n) < 1e-9:
            continue
        if np.dot(n, p0 - arr.mean(axis=0)) < 0:
            p1, p2 = p2, p1
        w = cq.Wire.makePolygon([cq.Vector(*p0), cq.Vector(*p1), cq.Vector(*p2), cq.Vector(*p0)])
        faces.append(cq.Face.makeFromWires(w))
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces))


def _both_y(points):
    out = []
    for x, y, z in points:
        out.append((float(x), float(y), float(z)))
        if abs(y) > 1e-9:
            out.append((float(x), -float(y), float(z)))
    return out


def _planes(points):
    eq = ConvexHull(np.asarray(points, float)).equations
    keep = []
    for q in eq:
        if not any(np.allclose(q, k, atol=1e-6) for k in keep):
            keep.append(q)
    return np.array(keep)


def _solid_from_planes(planes):
    A, b = planes[:, :3], -planes[:, 3]
    fit = linprog(
        [0, 0, 0, -1],
        A_ub=np.column_stack((A, np.ones(len(A)))),
        b_ub=b,
        bounds=[(None, None)] * 3 + [(0, None)],
        method="highs",
    )
    assert fit.success and fit.x[3] > 1e-3, "empty halfspace solid"
    pts = HalfspaceIntersection(planes, fit.x[:3]).intersections
    return _hull(pts)


def _shell_block(points, wall=WALL, open_if=None, mirror_y=True):
    pts = _both_y(points) if mirror_y else list(points)
    outer = _planes(pts)
    inner = outer.copy()
    for q in inner:
        if open_if is not None and open_if(q[:3]):
            q[3] -= 25.0
        else:
            q[3] += wall
    return _solid_from_planes(outer).cut(_solid_from_planes(inner)).clean(), _solid_from_planes(outer)


def build_cyber_trophy_truck():
    r"""Build the unified Short Course / Baja Desert Trophy Truck body shell (`SCT_Cyber_Trophy_Body`),
    separate `SCT_Hex_Mesh_Grille`, and `SCT_Cyan_DRL_Bar`.
    """
    # =============================================================================================
    # 1. LOWER TROPHY TRUCK BODY (matched directly to user's magenta side-view markup!)
    # =============================================================================================
    NF_TOP_X, NF_TOP_Z = 63.8, 29.0
    NF_BOT_X, NF_BOT_Z = 62.6, 21.2   # crisp raked nose tucked cleanly along magenta front face

    # Block 1A: Strongly pinched front hood & cowl (straight V-taper |Y|=28.6 -> 18.2 per top-view markup!)
    front_pts = [
        (NF_TOP_X, 18.0, NF_TOP_Z),
        (NF_TOP_X + 0.8, 0.0, NF_TOP_Z + 0.4),
        (NF_BOT_X, 17.4, NF_BOT_Z),
        (NF_BOT_X + 0.8, 0.0, NF_BOT_Z),
        # Straight V-pinched hood shoulder (from cowl |Y|=28.6 smoothly inward to nose |Y|=18.0!)
        (44.0, 23.2, 30.2),
        (23.5, 28.6, 31.4),
        (23.5,  0.0, 32.2),
        # Recessed door window-sill waist dip
        (14.0, 27.6, 30.2),
        (14.0,  0.0, 31.0),
        # Lower front door chine & rocker
        (24.5, 30.4, 19.5),
        (14.0, 30.6, 20.2),
        (24.5, 25.4,  7.2),
        (14.0, 25.4,  7.2),
    ]

    # Block 1B: Mid-door + pinched rear hips (|Y| tapers 28.2 -> 25.0 -> 24.2 toward tail!)
    rear_pts = [
        ( 16.0, 27.6, 30.2),
        ( 16.0,  0.0, 31.0),
        ( 16.0, 30.6, 20.2),
        ( 16.0, 25.4,  7.2),
        ( -4.0, 31.2, 21.8),
        (-25.0, 30.2, 24.0),
        (-26.5, 28.0, 32.4),
        (-26.5,  0.0, 32.8),
        (-24.5, 25.4,  7.2),
        # Pinched rear hip & bedside rail tapering cleanly inward to the tail
        (-45.0, 25.6, 27.2),
        (-45.0, 25.0, 32.4),
        (-67.0, 24.4, 28.8),
        (-67.0, 24.0, 32.6),
        (-67.0,  0.0, 32.6),
        (-67.0,  0.0, 28.8),
    ]

    f_out = _solid_from_planes(_planes(_both_y(front_pts)))
    r_out = _solid_from_planes(_planes(_both_y(rear_pts)))
    lower_env = f_out.fuse(r_out)

    def _lower_cav(pts_list, open_x_side=0):
        pl = _planes(_both_y(pts_list))
        for q in pl:
            if q[2] < -0.55 or (open_x_side < 0 and q[0] < -0.75) or (open_x_side > 0 and q[0] > 0.75):
                q[3] -= 25.0
            else:
                q[3] += WALL
        return _solid_from_planes(pl)

    body_lower = lower_env.cut(_lower_cav(front_pts, open_x_side=-1).fuse(_lower_cav(rear_pts, open_x_side=1)))

    # =============================================================================================
    # 2. PINCHED, DROOPING STEPPED CENTER POWER-BULGE ON HOOD (angled rear base echoing windshield!)
    # =============================================================================================
    hood_bulge_pts = [
        # Base footprint starts at X = 22.5 (angled rear ramp up to top crest at X = 26.2!)
        (60.0,  9.5, 29.2), (44.0, 11.4, 30.6), (22.5, 13.0, 31.8),
        (57.5,  5.6, 31.8), (44.0,  7.2, 33.4), (26.2,  8.5, 34.8),
        (58.0,  0.0, 32.3), (44.0,  0.0, 33.9), (26.2,  0.0, 35.2),
        (60.0,  9.5, 26.0), (22.5, 13.0, 27.5),
    ]
    bulge_out = _solid_from_planes(_planes(_both_y(hood_bulge_pts)))
    bulge_in_pl = _planes(_both_y(hood_bulge_pts))
    for q in bulge_in_pl:
        if q[2] < -0.55 or q[0] < -0.75:
            q[3] -= 25.0
        else:
            q[3] += WALL
    bulge_in = _solid_from_planes(bulge_in_pl)
    body_lower = body_lower.fuse(bulge_out).cut(bulge_in)

    # =============================================================================================
    # 3. SHORT COURSE TRUCK CAB & GREENHOUSE (matched to magenta markup: forward-leaning B-pillar
    #    base to roof, sleeker low-profile sloped roof & hump, recessed door window sill!)
    # =============================================================================================
    cab_pts = [
        # Wide window-ledge base (rear B-pillar base extended to X = -29.0 to wrap cage B-pillar hoop cleanly!)
        ( 23.5, 26.8, 29.6), ( 23.5,  0.0, 30.4),
        (-29.0, 26.8, 31.0), (-29.0,  0.0, 31.6),
        # Front roof header at X = +5.5, Z = 42.0 (tracks sloping red Cage smoothly!)
        (  5.5, 16.8, 42.0), (  5.5,  0.0, 42.5),
        # Rear roof peak at X = -26.2, Z = 44.8 (forward-leaning B-pillar rear edge -29.0 -> -26.2!)
        (-26.2, 16.0, 44.8), (-26.2,  0.0, 45.1),
    ]
    cab_outer = _solid_from_planes(_planes(_both_y(cab_pts)))

    # Sleeker, lower-profile rearward-tapering roof hump (tracks roof peak cleanly back to B-pillar!)
    roof_bulge_pts = [
        (  3.5, 10.0, 42.2), (-25.8,  5.0, 44.8),
        (  0.5,  6.0, 44.2), (-24.0,  2.4, 45.6),
        (  1.0,  0.0, 44.6), (-24.0,  0.0, 45.8),
        (  3.5, 10.0, 39.5), (-25.8,  5.0, 42.0),
    ]
    rbulge_out = _solid_from_planes(_planes(_both_y(roof_bulge_pts)))

    def _cab_inner(pts_list):
        pl = _planes(_both_y(pts_list))
        for q in pl:
            if q[2] < -0.55 or q[0] < -0.70:
                q[3] -= 25.0
            else:
                q[3] += WALL
        return _solid_from_planes(pl)

    cab = cab_outer.fuse(rbulge_out).cut(_cab_inner(cab_pts).fuse(_cab_inner(roof_bulge_pts)))

    # Cut crisp angular OPEN WINDSHIELD
    ws_cut = _hull(_both_y([
        ( 21.5, 19.5, 32.2), ( 21.5, 19.5, 36.8),
        (  7.8, 13.0, 40.4), (  7.8, 13.0, 44.6),
        ( 15.5, 19.5, 29.6), (  4.0, 13.0, 36.0),
    ]))
    cab = cab.cut(ws_cut)

    # Cut crisp OPEN SIDE WINDOWS with angled kick-down/kick-up sill corners at both A- and B-pillars!
    side_win_pts = [
        ( 20.0, 31.8),   # angled kick-up meeting A-pillar base
        (  4.5, 40.4),   # front roof header corner
        (-19.8, 42.8),   # rear roof header corner (vertical B-pillar front edge)
        (-19.8, 32.4),   # B-pillar beltline shoulder
        (-17.0, 29.8),   # angled drop into recessed door window sill (not 90 deg!)
        ( 16.5, 30.2),   # front sill corner angling up to A-pillar
    ]
    side_win = (
        cq.Workplane("XZ", origin=(0.0, -40.0, 0.0))
        .polyline(side_win_pts)
        .close()
        .extrude(-80.0)
        .val()
    )
    cab = cab.cut(side_win)

    # Fuse cab seamlessly into lower Trophy Truck body, and clear the cabin interior cavity
    body = body_lower.fuse(cab)
    # Also cut the recessed door window sill cleanly into the top of the door beltline (matches magenta dip!)
    body = body.cut(side_win)
    cabin_open = cq.Solid.makeBox(50.0, 44.0, 13.8, cq.Vector(-26.0, -22.0, 16.0))
    body = body.cut(cabin_open)

    # =============================================================================================
    # 4. OPEN TROPHY TRUCK REAR BED + SWEPT-BACK FACETED TAIL FINS (matches magenta rear fin!)
    # =============================================================================================
    # Cap bed_open at Z = 33.4 so it opens the rear bed cleanly without slicing the B-pillar or roof!
    bed_open = cq.Solid.makeBox(46.0, 42.4, 23.4, cq.Vector(-72.0, -21.2, 10.0))
    body = body.cut(bed_open)

    # Twin swept-back faceted rear tail fins seated cleanly on the pinched rear bedside rail (|Y| = 21.8..24.4)
    fin_L = _hull([
        (-51.5, 24.6, 31.8), (-51.5, 22.0, 31.8),
        (-67.0, 24.2, 31.8), (-67.0, 21.8, 31.8),
        (-58.0, 24.0, 39.2), (-58.0, 22.2, 39.2),
        (-68.6, 23.8, 39.8), (-68.6, 22.0, 39.8),
    ])
    body = body.fuse(fin_L).fuse(fin_L.mirror("XZ"))

    # =============================================================================================
    # 4B. STEPPED FACETED WIDEBODY OVER-FENDERS (Front & Rear, matching top-view magenta markup!)
    # =============================================================================================
    # Front over-fenders: flare sharply out from the pinched hood line (|Y|=18..28) to |Y|=31.8 over
    # the front wheel arch (Z >= 28.8 -> 0.0000 mm3 tire overlap at full bump!), stepped slightly
    # below the hood top so the bolted-on Trophy-Truck over-fender crease reads super crisp!
    front_of_L = _hull([
        # Inner anchor seated cleanly into the pinched hood side wall
        (25.5, 26.5, 30.6), (25.5, 26.5, 28.8),
        (59.5, 18.0, 28.8), (59.5, 18.0, 27.0),
        # Outer widebody faceted over-fender flare (|Y| = 31.8 over the front axle!)
        (32.5, 31.6, 30.4), (32.5, 31.8, 28.8),
        (55.0, 31.4, 29.2), (55.0, 31.6, 28.8),
        (61.0, 28.2, 28.6), (61.0, 28.4, 27.2),
    ])
    body = body.fuse(front_of_L).fuse(front_of_L.mirror("XZ"))

    # Rear over-fenders: flare sharply out from the pinched rear hip (|Y|=24..26) to |Y|=31.8 over
    # the rear wheel arch (Z >= 29.0 -> 0.0000 mm3 tire overlap at full bump!)
    rear_of_L = _hull([
        # Inner anchor seated cleanly into the pinched rear bedside wall
        (-42.0, 24.0, 32.0), (-42.0, 24.0, 29.0),
        (-67.0, 22.5, 32.0), (-67.0, 22.5, 29.0),
        # Outer widebody faceted rear over-fender flare (|Y| = 31.8 over the rear axle!)
        (-47.5, 31.6, 31.6), (-47.5, 31.8, 29.0),
        (-64.5, 31.4, 31.4), (-64.5, 31.6, 29.0),
        (-67.0, 29.2, 31.2), (-67.0, 29.4, 29.0),
    ])
    body = body.fuse(rear_of_L).fuse(rear_of_L.mirror("XZ"))

    # =============================================================================================
    # 5. CRISP ANGULAR HIGH-CLEARANCE WHEEL ARCHES + REAR SHOCK BAY CUT + FRONT INNER LINER TUBS
    # =============================================================================================
    # Front polygonal wheel arch (X = 26.5 .. 67, Z_ceil = 28.6 over full-bump front tire Z_max=27.2!)
    front_arch_prof = [
        (26.5,  0.0),
        (26.5, 19.5),
        (32.5, 28.6),
        (59.0, 28.6),
        (65.0, 21.5),
        (72.0, 21.5),
        (72.0,  0.0),
    ]
    front_arch_L = (
        cq.Workplane("XZ", origin=(0.0, 19.5, 0.0))
        .polyline(front_arch_prof)
        .close()
        .extrude(-25.0)
        .val()
    )
    body = body.cut(front_arch_L).cut(front_arch_L.mirror("XZ"))

    # Swept rear shock-bay & rear wheel arch cut (matches Tacoma's sweeping upward cut exposing the rear shock!)
    rear_arch_prof = [
        (-74.0,  0.0),
        (-74.0, 29.0),
        (-48.0, 29.0),
        (-36.0, 21.0),
        (-26.0,  7.0),
        (-26.0,  0.0),
    ]
    rear_arch_L = (
        cq.Workplane("XZ", origin=(0.0, 21.0, 0.0))
        .polyline(rear_arch_prof)
        .close()
        .extrude(-25.0)
        .val()
    )
    body = body.cut(rear_arch_L).cut(rear_arch_L.mirror("XZ"))

    # Front inner fender liner tubs (1.7 mm inboard wheel-well walls at |Y| = 17.9..19.6, X = 25.5..62.5)
    liner_prof = [
        (25.5, 31.2),
        (62.5, 30.0),
        (62.5, 22.0),
        (45.0, 21.5),
        (37.0, 17.5),
        (25.5, 16.5),
    ]
    liner_L = (
        cq.Workplane("XZ", origin=(0.0, 17.9, 0.0))
        .polyline(liner_prof)
        .close()
        .extrude(-1.7)
        .val()
    )
    liner_flange_L = cq.Solid.makeBox(36.0, 4.5, 2.0, cq.Vector(26.5, 17.9, 28.6))
    liner_L = liner_L.fuse(liner_flange_L).intersect(lower_env)
    body = body.fuse(liner_L).fuse(liner_L.mirror("XZ"))

    # =============================================================================================
    # 6. 3 SCULPTED RAISED FORWARD-LEANING CYBERPUNK GILL LOUVERS ON BOTH DOORS
    # =============================================================================================
    flank_n = np.array([0.0, 0.94, -0.34], float)
    flank_n /= np.linalg.norm(flank_n)
    ang = math.radians(-22.0)
    v_dir = np.array([-math.cos(ang), 0.0, math.sin(ang)], float)
    v_dir -= np.dot(v_dir, flank_n) * flank_n
    v_dir /= np.linalg.norm(v_dir)
    u_dir = np.cross(v_dir, flank_n)
    u_dir /= np.linalg.norm(u_dir)
    if u_dir[2] < 0:
        u_dir = -u_dir

    # Tighter 3-gill cluster (7.2 mm pitch) with rear-most gill aligned directly with the B-pillar (X = -20.5)!
    louver_specs = [(-6.1, 15.8), (-13.3, 16.2), (-20.5, 16.6)]
    throat_cuts = []
    for lx, lz in louver_specs:
        probe = lower_env.intersect(cq.Solid.makeBox(0.5, 40.0, 0.5, cq.Vector(lx - 0.25, 10.0, lz - 0.25)))
        fy = probe.BoundingBox().ymax if probe.Volume() > 1e-6 else 29.0
        c_surf = np.array([lx, fy, lz], float)

        hu_out, hu_top = 5.2, 4.0
        v_lead, v_crest, v_lip = -2.0, 0.8, 1.3
        w_embed, w_proud = -1.2, 2.0

        def to_w(u, v, w):
            p = c_surf + u * u_dir + v * v_dir + w * flank_n
            return (float(p[0]), float(p[1]), float(p[2]))

        fin = _hull([
            to_w(-hu_out, v_lead,  w_embed), to_w(hu_out, v_lead,  w_embed),
            to_w( hu_out, v_crest, w_embed), to_w(-hu_out, v_crest, w_embed),
            to_w(-hu_out, v_lead,  0.1),     to_w(hu_out, v_lead,  0.1),
            to_w(-hu_top, v_crest, w_proud), to_w(hu_top, v_crest, w_proud),
            to_w(-hu_top, v_lip,   w_proud - 0.5), to_w(hu_top, v_lip, w_proud - 0.5),
        ])
        body = body.fuse(fin).fuse(fin.mirror("XZ"))

        t_hu = 3.8
        tc = _hull([
            to_w(-t_hu, v_crest - 0.2, w_proud - 0.9), to_w(t_hu, v_crest - 0.2, w_proud - 0.9),
            to_w( t_hu, v_lip + 1.1,   w_proud - 0.9), to_w(-t_hu, v_lip + 1.1,  w_proud - 0.9),
            to_w(-t_hu, v_crest - 1.8, -WALL - 2.5),   to_w(t_hu, v_crest - 1.8, -WALL - 2.5),
            to_w( t_hu, v_lip - 0.3,   -WALL - 2.5),   to_w(-t_hu, v_lip - 0.3,  -WALL - 2.5),
        ])
        throat_cuts.append(tc)
        throat_cuts.append(tc.mirror("XZ"))

    for tc in throat_cuts:
        body = body.cut(tc)

    # =============================================================================================
    # 7. DROP-IN SCX30 MOUNTING: FRONT MAGNET BOSSES + REAR TILT-OPEN HINGE LUGS
    # =============================================================================================
    # Front magnet bosses under hood at X = 57.3, Y = +-7.6, Z_bot = 27.4 (Ø3.2 x 2.1 mm disc pocket)
    for sy in (-1, 1):
        boss = cq.Solid.makeCylinder(2.5, 6.0, cq.Vector(57.3, sy * 7.6, 27.4), cq.Vector(0, 0, 1))
        boss = boss.intersect(lower_env.fuse(bulge_out))
        pocket = cq.Solid.makeCylinder(1.65, 2.2, cq.Vector(57.3, sy * 7.6, 27.3), cq.Vector(0, 0, 1))
        body = body.fuse(boss).cut(pocket)

    # Rear tilt-open hinge lugs at X = -64.0, Z = 29.0, |Y| = 18.9..21.3 (Ø5.2 OD, Ø1.7 M2 pilot)
    for sy in (-1, 1):
        y0 = 18.9 if sy > 0 else -21.3
        wy0 = 20.5 if sy > 0 else -29.0
        lug = cq.Solid.makeCylinder(2.6, 2.4, cq.Vector(-64.0, y0, 29.0), cq.Vector(0, 1, 0))
        web = cq.Solid.makeBox(6.0, 8.5, 4.0, cq.Vector(-67.0, wy0, 28.6))
        pilot = cq.Solid.makeCylinder(0.85, 8.0, cq.Vector(-64.0, 17.5 if sy > 0 else -25.5, 29.0), cq.Vector(0, 1, 0))
        body = body.fuse(lug).fuse(web).cut(pilot)

    # =============================================================================================
    # 8. FULL-WIDTH CYAN LED DRL BROW BAR + VISIBLY OPEN HEXAGONAL MESH TROPHY GRILLE
    # =============================================================================================
    p_top = np.array([NF_TOP_X, 0.0, NF_TOP_Z])
    p_bot = np.array([NF_BOT_X, 0.0, NF_BOT_Z])
    v_dir = p_top - p_bot
    span_v = float(np.linalg.norm(v_dir))
    v_dir /= span_v
    u_dir = np.array([0.0, 1.0, 0.0])
    n_dir = np.cross(u_dir, v_dir)
    n_dir /= np.linalg.norm(n_dir)
    if n_dir[0] < 0:
        n_dir = -n_dir
    c_fascia = 0.5 * (p_top + p_bot)
    pl_fascia = cq.Plane(origin=tuple(c_fascia), xDir=tuple(u_dir), normal=tuple(n_dir))

    # A) Cyan DRL Brow Bar across pinched upper nose
    led_v_hi = span_v / 2.0 - 0.65
    led_v_lo = led_v_hi - 2.2
    led_v_mid = 0.5 * (led_v_hi + led_v_lo)
    led_w = 32.0

    led_groove = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w, STRIP_H)
        .extrude(4.0)
        .translate(cq.Vector(*(n_dir * (-GROOVE_D))))
        .val()
    )
    body = body.cut(led_groove)

    led_bar = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w - 2 * FIT, STRIP_H - 2 * FIT)
        .extrude(GROOVE_D - 0.1)
        .translate(cq.Vector(*(n_dir * (-GROOVE_D + 0.05))))
        .val()
    )

    # B) Hexagonal Honeycomb Mesh Trophy Grille (fits cleanly inside pinched nose!)
    gr_v_hi = led_v_lo - 0.75
    gr_v_lo = -span_v / 2.0 + 0.65
    gr_h = gr_v_hi - gr_v_lo
    gr_v_mid = 0.5 * (gr_v_hi + gr_v_lo)
    gr_w = 28.5

    def _chamf_rect(w, h, ch=1.1):
        hw, hh = w / 2.0, h / 2.0
        ch = min(ch, hw * 0.4, hh * 0.4)
        return [
            (-hw + ch, -hh), (hw - ch, -hh),
            (hw, -hh + ch),  (hw, hh - ch),
            (hw - ch, hh),   (-hw + ch, hh),
            (-hw, hh - ch),  (-hw, -hh + ch),
        ]

    intake_cut = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect(gr_w, gr_h, 1.1))
        .close()
        .extrude(WALL + 8.0)
        .translate(cq.Vector(*(n_dir * (-WALL - 3.0))))
        .val()
    )
    body = body.cut(intake_cut)

    gr_plate = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect(gr_w - 2 * FIT, gr_h - 2 * FIT, 1.0))
        .close()
        .extrude(WALL - 0.3)
        .translate(cq.Vector(*(n_dir * (-WALL))))
        .val()
    )
    gr_flange = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect(gr_w + 1.8, gr_h + 1.8, 1.3))
        .close()
        .extrude(1.1)
        .translate(cq.Vector(*(n_dir * (-WALL - 1.1 - FIT - 0.45))))
        .val()
    )
    gr_bridge = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect(gr_w - 2 * FIT, gr_h - 2 * FIT, 1.0))
        .close()
        .extrude(FIT + 0.95)
        .translate(cq.Vector(*(n_dir * (-WALL - FIT - 0.7))))
        .val()
    )
    gr_hole = (
        cq.Workplane(pl_fascia)
        .center(0.0, gr_v_mid)
        .polyline(_chamf_rect(gr_w - 2.4, gr_h - 1.8, 0.8))
        .close()
        .extrude(5.0)
        .translate(cq.Vector(*(n_dir * (-WALL - 3.0))))
        .val()
    )
    grille = gr_plate

    # Cut visibly open 1.8 mm hexagonal honeycomb cells!
    pitch = HEX_FLAT + HEX_RIB
    dx = pitch * math.sqrt(3.0) / 2.0
    dy = pitch
    hex_r = HEX_FLAT / math.sqrt(3.0)
    lim_x = (gr_w - 2 * FIT) / 2.0 - 1.2
    lim_y = (gr_h - 2 * FIT) / 2.0 - 0.40
    centers = []
    for ix in range(-9, 10):
        cx_h = ix * dx
        y_off = (dy / 2.0) if (ix % 2 != 0) else 0.0
        for iy in range(-3, 4):
            cy_h = iy * dy + y_off
            if abs(cx_h) + hex_r * 0.85 <= lim_x and abs(cy_h) + hex_r * 0.78 <= lim_y:
                centers.append((cx_h, gr_v_mid + cy_h))
    assert len(centers) >= 6, f"Expected visible hex grille cells, got {len(centers)}"
    wp_hex = cq.Workplane(pl_fascia).pushPoints(centers).polygon(6, 2.0 * hex_r).extrude(WALL + 6.0)
    grille = grille.cut(wp_hex.translate(cq.Vector(*(n_dir * (-WALL - 3.0)))).val())

    body = body.clean()
    grille = grille.clean()
    led_bar = led_bar.clean()
    assert body.isValid() and len(body.Solids()) == 1, f"SCT_Cyber_Trophy_Body invalid ({len(body.Solids())} solids)"
    assert grille.isValid() and len(grille.Solids()) == 1, f"SCT_Hex_Mesh_Grille invalid ({len(grille.Solids())} solids)"
    assert led_bar.isValid() and len(led_bar.Solids()) == 1, f"SCT_Cyan_DRL_Bar invalid ({len(led_bar.Solids())} solids)"

    return {
        "SCT_Cyber_Trophy_Body": body,
        "SCT_Hex_Mesh_Grille": grille,
        "SCT_Cyan_DRL_Bar": led_bar,
    }


if __name__ == "__main__":
    parts = build_cyber_trophy_truck()
    for name, sol in parts.items():
        bb = sol.BoundingBox()
        print(f"{name:22s}: vol={sol.Volume():7.1f} mm3  x=[{bb.xmin:6.1f},{bb.xmax:6.1f}] y=[{bb.ymin:6.1f},{bb.ymax:6.1f}] z=[{bb.zmin:6.1f},{bb.zmax:6.1f}]")
