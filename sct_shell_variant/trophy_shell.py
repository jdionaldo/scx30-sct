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

# Clearance to the SCX30 kit (Cage, Chassis_Tub, front bulkhead, shocks, ESC). Kit rule: >= 1.4 mm to cage / tub /
# bulkhead, magnet seat 0.3 mm, >= 2.0 mm to shocks. Numbers below are the kit's measured geometry (stl/v3, stl/v4).
ROCKER_Y = 26.4                                  # rocker bottom edge |Y| (was 25.4: inner face 0.46 mm from the tub skirt at |Y| 23.46)
CABIN_OPEN = ((-26.6, 22.7), 24.2, (16.0, 33.2))  # X range, |Y| max, Z range: the window-sill deck plate sat on the cage beltline and
                                                 # windshield-base tubes (top Z 31.6 / 30.4, |Y| up to 22.7) and the ESC
SILL_TRIM = ((-17.0, 16.5), 26.6, (16.0, 33.2))   # X (side-window span), |Y|, Z: the door-top ledge inboard of the door wall was a
                                                 # 0.2..0.4 mm sheet once the deck was gone (the window cut runs down to it)
BED_OPEN_Y = 22.0                                # bed opening |Y| (was 21.2: rear shock eye at |Y| 19.7 was 1.5 mm from the rail)
BED_NOTCH = ((-34.2, -26.0), 24.2)               # X range, |Y| max: bed rail notch over the cage beltline tie (|Y| 22.7, Z 31.6)
FRONT_HOOP_KEEPOUT = ((19.8, 28.7), 22.6, 29.75)  # X range, |Y| max, Z max: front hoop leg/top bar (X 26, Z 28.0) and hood rails
MAGNET_Z = 27.5                                  # boss underside: tub magnet seat top is Z 27.2 -> 0.3 mm, as the kit shell
MAGNET_SEAT_RELIEF = ((54.7, 60.0), (4.6, 10.6), 27.5)   # X, |Y|, Z max: hood-bulge skirt clear of the seat tower top
SHOCK_F_KEEPOUT = ((54.0, 61.5), (13.0, 19.7), 23.9)     # X, |Y|, Z max: nose side wall and liner clear of the front shock top
HOOD_REAR_DZ = 1.2                               # hood rear edge (X 23.5) and hood-bulge rear base raised: full 1.8 mm hood skin stays
                                                 # >= 1.4 mm above the cage hood rails (top Z 29.4 at X 22.7 -> 28.0 at X 26)
COWL_CLEAR = ((22.6, 23.7), 24.2, 30.5)          # X, |Y|, Z max: cab-base corner left under the hood rear (0.65 mm above the hood rails)
WINDSHIELD_FLOOR = ((14.0, 22.7), 19.5, (31.8, 35.0))  # X, |Y|, Z: hood skin behind its rear edge (X 22.7) under the windshield opening
FRONT_ARCH_CEIL_Z = 27.2                         # front arch ceiling (was 28.6) so the over-fender plate is a full 1.8 mm (it was 0.4..1.6);
                                                 # the front wheel envelope (steer + bump) tops out at Z 22.5
FLARE_T = WALL                                   # over-fender plate thickness, front and rear (rear was 2.2..3.0)
LINER_Y0 = 17.7                                  # front wheel-well liner inner face |Y| (was 17.9, 1.7 thick): 1.8 mm out to the arch cut at 19.5
LINER_FOOT_CLEAR = ((45.5, 54.3), (17.5, 19.7), 21.4)    # X, |Y|, Z max: wedge of the nose side wall left below the liner by the arch cut
DRL_BACKING = WALL                               # wall behind the LED groove floor (was 0.1..0.3 where the nose bows forward of the fascia plane)
WHEEL_WELL_BACK = (((23.0, 28.75), (21.0, 22.8), (9.5, 30.0)),     # X, |Y|, Z: back wall, 1.4 mm outboard of the hood-bay diagonal
                   ((28.65, 30.45), (17.7, 22.8), (16.0, 30.0)))    # cross wall to the liner (1.4 mm ahead of the front hoop leg)
PIVOT_R, PIVOT_Y1 = 3.2, 23.0                    # rear pivot barrel radius / outer |Y| (was a 2.6 x 2.4 mm lug, |Y| 18.9..21.3)
PIVOT_TOP = ((-68.4, -59.6), 24.4, 32.2)         # X range, |Y| max, Z of the block top, fused into the bed rail (top Z 32.6) and fin root
A_PILLAR_DX, A_PILLAR_DX_BASE = 2.5, 0.9        # side-window front edge moved back at the header / at the base (was X 4.5 / 20.0): the A-pillar
                                                 # was 1.3..2.5 mm wide in side view; now about 4 mm (base limited by the cage A-post node below it)
A_PILLAR_ZONE = (31.3, 42.0, 5.0, 17.0)          # Z range, forward reach from the side-window front edge, min |Y| of the A-pillar that is put back
                                                 # after the cabin cuts: the cabin opening and the squared window cut had cut it to a sliver
WIN_RIM_OUT, WIN_RIM_IN = 3.0, 4.0                # side-window cut reach outside / inside the cab wall along its normal
LED_TOP_MARGIN = 1.9                             # LED groove top below the nose top edge (was 0.65: the groove ate the top skin down to 0.7 mm)
ROOF_FRONT_DZ = 0.8                              # front roof header raised (was Z 42.0 / 42.5): roof underside was 0.70 mm above the
                                                 # cage roof rails / roof-front bar (X -22..8, |Y| <= 16)
ROOF_REAR_DZ = 1.3                               # rear roof peak raised (was Z 44.8 / 45.1): its underside was 0.16 mm above the
                                                 # cage roof-peak bar (X -25.5, |Y| <= 11.5, top Z 43.0)
NOSE_BOT_X = 63.5                                # nose bottom edge X (was 62.6): its inner face was 0.63 mm from the bulkhead /
                                                 # shock tower front and 0.06 mm from the front shock upper screw heads (X 60.8)


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
    NF_BOT_X, NF_BOT_Z = NOSE_BOT_X, 21.2   # crisp raked nose tucked cleanly along magenta front face

    # Block 1A: Strongly pinched front hood & cowl (straight V-taper |Y|=28.6 -> 18.2 per top-view markup!)
    front_pts = [
        (NF_TOP_X, 18.0, NF_TOP_Z),
        (NF_TOP_X + 0.8, 0.0, NF_TOP_Z + 0.4),
        (NF_BOT_X, 17.4, NF_BOT_Z),
        (NF_BOT_X + 0.8, 0.0, NF_BOT_Z),
        # Straight V-pinched hood shoulder (from cowl |Y|=28.6 smoothly inward to nose |Y|=18.0!)
        (44.0, 23.2, 30.2),
        (23.5, 28.6, 31.4 + HOOD_REAR_DZ),
        (23.5,  0.0, 32.2 + HOOD_REAR_DZ),
        # Recessed door window-sill waist dip
        (14.0, 27.6, 30.2),
        (14.0,  0.0, 31.0),
        # Lower front door chine & rocker
        (24.5, 30.4, 19.5),
        (14.0, 30.6, 20.2),
        (24.5, ROCKER_Y,  7.2),
        (14.0, ROCKER_Y,  7.2),
    ]

    # Block 1B: Mid-door + pinched rear hips (|Y| tapers 28.2 -> 25.0 -> 24.2 toward tail!)
    rear_pts = [
        ( 16.0, 27.6, 30.2),
        ( 16.0,  0.0, 31.0),
        ( 16.0, 30.6, 20.2),
        ( 16.0, ROCKER_Y,  7.2),
        ( -4.0, 31.2, 21.8),
        (-25.0, 30.2, 24.0),
        (-26.5, 28.0, 32.4),
        (-26.5,  0.0, 32.8),
        (-24.5, ROCKER_Y,  7.2),
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
        (60.0,  9.5, 29.2), (44.0, 11.4, 30.6), (22.5, 13.0, 31.8 + HOOD_REAR_DZ),
        (57.5,  5.6, 31.8), (44.0,  7.2, 33.4), (26.2,  8.5, 34.8),
        (58.0,  0.0, 32.3), (44.0,  0.0, 33.9), (26.2,  0.0, 35.2),
        (60.0,  9.5, 26.0), (22.5, 13.0, 27.5 + HOOD_REAR_DZ),
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
        (  5.5, 16.8, 42.0 + ROOF_FRONT_DZ), (  5.5,  0.0, 42.5 + ROOF_FRONT_DZ),
        # Rear roof peak at X = -26.2, Z = 44.8 (forward-leaning B-pillar rear edge -29.0 -> -26.2!)
        (-26.2, 16.0, 44.8 + ROOF_REAR_DZ), (-26.2,  0.0, 45.1 + ROOF_REAR_DZ),
    ]
    cab_outer = _solid_from_planes(_planes(_both_y(cab_pts)))

    # Sleeker, lower-profile rearward-tapering roof hump (tracks roof peak cleanly back to B-pillar!)
    roof_bulge_pts = [
        (  3.5, 10.0, 42.2 + ROOF_FRONT_DZ), (-25.8,  5.0, 44.8 + ROOF_REAR_DZ),
        (  0.5,  6.0, 44.2 + ROOF_FRONT_DZ), (-24.0,  2.4, 45.6 + ROOF_REAR_DZ),
        (  1.0,  0.0, 44.6 + ROOF_FRONT_DZ), (-24.0,  0.0, 45.8 + ROOF_REAR_DZ),
        (  3.5, 10.0, 39.5 + ROOF_FRONT_DZ), (-25.8,  5.0, 42.0 + ROOF_REAR_DZ),
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
    cab_ws = cab                                   # (kept: source of the A-pillar that is put back after the cabin cuts)

    # Cut crisp OPEN SIDE WINDOWS with angled kick-down/kick-up sill corners at both A- and B-pillars!
    side_win_pts = [
        ( 20.0 - A_PILLAR_DX_BASE, 31.8),   # angled kick-up meeting A-pillar base
        (  4.5 - A_PILLAR_DX, 40.4),   # front roof header corner
        (-19.8, 42.8),   # rear roof header corner (vertical B-pillar front edge)
        (-19.8, 32.4),   # B-pillar beltline shoulder
        (-17.0, 29.8),   # angled drop into recessed door window sill (not 90 deg!)
        ( 16.5, 30.2),   # front sill corner angling up to A-pillar
    ]
    # Window rims square to the cab side wall (owner 2026-11-24: consistent wall). A cut straight along Y left a
    # knife-edged header and pillar rims on the 40-degree side wall; here those corners are cut along the wall's own
    # normal, so the rim meets the skin at 90 degrees. The sill keeps the cut along Y. The A-pillar edge is moved back
    # A_PILLAR_DX so the pillar keeps a full-width section between the windshield and the side window.
    def _wall_y(x, z):
        bb = cab_outer.intersect(cq.Solid.makeBox(0.02, 60.0, 0.02, cq.Vector(x - 0.01, 0.0, z - 0.01))).BoundingBox()
        return bb.ymax

    def _window_cut(sy):
        pts = []
        for x, z in side_win_pts[:4]:                               # A-pillar, header and B-pillar corners: on the wall
            yw = _wall_y(x, z)
            dydz = (_wall_y(x, z + 0.4) - _wall_y(x, z - 0.4)) / 0.8
            n = np.array([0.0, 1.0, -dydz]); n /= np.linalg.norm(n)
            for t in (WIN_RIM_OUT, -WIN_RIM_IN):
                q = np.array([x, yw, z]) + n * t
                pts.append((float(q[0]), sy * float(q[1]), float(q[2])))
        for x, z in side_win_pts[4:]:                               # sill corners: straight across, as before
            for y in (0.0, 40.0):
                pts.append((x, sy * y, z))
        return _hull(pts)

    side_win = _window_cut(1).fuse(_window_cut(-1))
    cab = cab.cut(side_win)

    # Fuse cab seamlessly into lower Trophy Truck body, and clear the cabin interior cavity
    body = body_lower.fuse(cab)
    # Also cut the recessed door window sill cleanly into the top of the door beltline (matches magenta dip!)
    body = body.cut(side_win)
    (cx0, cx1), cy, (cz0, cz1) = CABIN_OPEN
    cabin_open = cq.Solid.makeBox(cx1 - cx0, 2 * cy, 31.8 - cz0, cq.Vector(cx0, -cy, cz0)).fuse(
        cq.Solid.makeBox(cx1 - cx0, 2 * cy, cz1 - 31.8, cq.Vector(cx0, -cy, 31.8)).intersect(_cab_inner(cab_pts)))
    (wx0, wx1), wy, (wz0, wz1) = WINDSHIELD_FLOOR
    (kx0, kx1), ky, kz = COWL_CLEAR
    cabin_open = cabin_open.fuse(cq.Solid.makeBox(wx1 - wx0, 2 * wy, wz1 - wz0, cq.Vector(wx0, -wy, wz0))).fuse(
        cq.Solid.makeBox(wx1 - wx0, 2 * cy, wz1 - wz0, cq.Vector(wx0, -cy, wz0)).intersect(_cab_inner(cab_pts))).fuse(
        cq.Solid.makeBox(kx1 - kx0, 2 * ky, kz - cz0, cq.Vector(kx0, -ky, cz0)))
    (sx0, sx1), sy_, (sz0, sz1) = SILL_TRIM
    cabin_open = cabin_open.fuse(cq.Solid.makeBox(sx1 - sx0, 2 * sy_, sz1 - sz0, cq.Vector(sx0, -sy_, sz0)))
    body = body.cut(cabin_open)

    # A-pillar, whole (owner 2026-11-24 markup): the cab wall between the windshield and the side window, cut only by
    # the windshield and the side window as drawn (front edge along Y), from the hood / sill up to the roof header.
    ex0, ez0 = side_win_pts[0]
    ex1, ez1 = side_win_pts[1]
    pz0, pz1, reach, py0 = A_PILLAR_ZONE
    def _edge_x(z):
        return ex0 + (ex1 - ex0) * (z - ez0) / (ez1 - ez0)
    zone = _hull([(x, y, z) for z in (pz0, pz1) for x in (_edge_x(z) - 0.02, _edge_x(z) + reach) for y in (py0, 30.0)])
    zone = zone.fuse(zone.mirror("XZ"))
    side_win_y = (
        cq.Workplane("XZ", origin=(0.0, -40.0, 0.0))
        .polyline(side_win_pts)
        .close()
        .extrude(-80.0)
        .val()
    )
    a_pillar = cab_ws.cut(side_win_y).intersect(zone)
    body = body.fuse(a_pillar)

    # =============================================================================================
    # 4. OPEN TROPHY TRUCK REAR BED + SWEPT-BACK FACETED TAIL FINS (matches magenta rear fin!)
    # =============================================================================================
    # Cap bed_open at Z = 33.4 so it opens the rear bed cleanly without slicing the B-pillar or roof!
    bed_open = cq.Solid.makeBox(46.0, 2 * BED_OPEN_Y, 23.4, cq.Vector(-72.0, -BED_OPEN_Y, 10.0))
    (nx0, nx1), ny = BED_NOTCH
    bed_open = bed_open.fuse(cq.Solid.makeBox(nx1 - nx0, 2 * ny, 23.4, cq.Vector(nx0, -ny, 10.0)))
    body = body.cut(bed_open)

    # Twin swept-back faceted rear tail fins seated cleanly on the pinched rear bedside rail (|Y| = 21.8..24.4)
    fin_L = _hull([
        (-51.5, 24.6, 31.8), (-51.5, 24.6 - WALL, 31.8),
        (-67.0, 24.2, 31.8), (-67.0, 24.2 - WALL, 31.8),
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
        (25.5, 28.0, 30.6), (25.5, 28.0, 30.6 - FLARE_T),
        (59.5, 18.9, 28.8), (59.5, 18.9, 28.8 - FLARE_T),
        # Outer widebody faceted over-fender flare (|Y| = 31.8 over the front axle!)
        (32.5, 31.6, 30.4), (32.5, 31.8, 30.4 - FLARE_T),
        (55.0, 31.4, 29.2), (55.0, 31.6, 29.2 - FLARE_T),
        (61.0, 28.2, 28.6), (61.0, 28.4, 28.6 - FLARE_T),
    ])
    body = body.fuse(front_of_L).fuse(front_of_L.mirror("XZ"))

    # Rear over-fenders: flare sharply out from the pinched rear hip (|Y|=24..26) to |Y|=31.8 over
    # the rear wheel arch (Z >= 29.0 -> 0.0000 mm3 tire overlap at full bump!)
    rear_of_L = _hull([
        # Inner anchor seated cleanly into the pinched rear bedside wall
        (-42.0, 26.3, 32.0), (-42.0, 26.3, 32.0 - FLARE_T),
        (-67.0, 23.8, 32.0), (-67.0, 23.8, 32.0 - FLARE_T),
        # Outer widebody faceted rear over-fender flare (|Y| = 31.8 over the rear axle!)
        (-47.5, 31.6, 31.6), (-47.5, 31.8, 31.6 - FLARE_T),
        (-64.5, 31.4, 31.4), (-64.5, 31.6, 31.4 - FLARE_T),
        (-67.0, 29.2, 31.2), (-67.0, 29.4, 31.2 - FLARE_T),
    ])
    body = body.fuse(rear_of_L).fuse(rear_of_L.mirror("XZ"))

    # =============================================================================================
    # 5. CRISP ANGULAR HIGH-CLEARANCE WHEEL ARCHES + REAR SHOCK BAY CUT + FRONT INNER LINER TUBS
    # =============================================================================================
    # Front polygonal wheel arch (X = 26.5 .. 67, Z_ceil = 28.6 over full-bump front tire Z_max=27.2!)
    front_arch_prof = [
        (26.5,  0.0),
        (26.5, 19.5),
        (32.5, FRONT_ARCH_CEIL_Z),
        (59.0, FRONT_ARCH_CEIL_Z),
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
        cq.Workplane("XZ", origin=(0.0, LINER_Y0, 0.0))
        .polyline(liner_prof)
        .close()
        .extrude(-(19.5 - LINER_Y0))
        .val()
    )
    liner_L = liner_L.intersect(lower_env)      # (the 2 mm flange box under the hood is gone: it doubled the hood skin)
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
    # Kit clearance cuts (constants at the top): front hoop and hood rails, magnet seat towers, front shock tops
    (hx0, hx1), hy, hz = FRONT_HOOP_KEEPOUT
    body = body.cut(cq.Solid.makeBox(hx1 - hx0, 2 * hy, hz - 5.0, cq.Vector(hx0, -hy, 5.0)))
    for sy in (-1, 1):
        for (x0, x1), (y0, y1), z1 in (MAGNET_SEAT_RELIEF, SHOCK_F_KEEPOUT, LINER_FOOT_CLEAR):
            ya, yb = (y0, y1) if sy > 0 else (-y1, -y0)
            body = body.cut(cq.Solid.makeBox(x1 - x0, yb - ya, z1 - 5.0, cq.Vector(x0, ya, 5.0)))
    # Front wheel-well back wall (owner 2026-11-24 markup): the liner is cut away round the cage front hoop leg and hood-bay
    # diagonal, which left the chassis in view behind the front wheel. A 1.8 mm wall outboard of the cage closes it, joined to
    # the liner by a short cross wall where the liner resumes.
    for sy in (-1, 1):
        for (x0, x1), (y0, y1), (z0, z1) in WHEEL_WELL_BACK:
            ya, yb = (y0, y1) if sy > 0 else (-y1, -y0)
            body = body.fuse(cq.Solid.makeBox(x1 - x0, yb - ya, z1 - z0, cq.Vector(x0, ya, z0)).intersect(lower_env))
    for sy in (-1, 1):
        boss = cq.Solid.makeCylinder(2.5, 6.0, cq.Vector(57.3, sy * 7.6, MAGNET_Z), cq.Vector(0, 0, 1))
        boss = boss.intersect(lower_env.fuse(bulge_out))
        pocket = cq.Solid.makeCylinder(1.65, 2.2, cq.Vector(57.3, sy * 7.6, MAGNET_Z - 0.1), cq.Vector(0, 0, 1))
        body = body.fuse(boss).cut(pocket)

    # Rear tilt-open pivot blocks under the tail fins (owner 2026-11-24: bolstered). Pin axis along Y at X -64.0, Z 29.0 (the
    # cage hinge ears end at |Y| 18.6, 0.3 mm inboard). Each block is one solid: a PIVOT_R barrel from |Y| 18.9 to PIVOT_Y1
    # hulled up into the bed rail and fin root (PIVOT_TOP), replacing the 5.2 mm lug and its 4 mm box web; M2 pilot 1.7 mm.
    px, pz = -64.0, 29.0
    (tx0, tx1), ty1, tz = PIVOT_TOP
    for sy in (-1, 1):
        ring = [(px + PIVOT_R * math.cos(a), y, pz + PIVOT_R * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False) for y in (18.9, PIVOT_Y1)]
        top = [(x, y, tz) for x in (tx0, tx1) for y in (18.9, ty1)]
        block = _hull([(x, sy * y, z) for x, y, z in ring + top])
        pilot = cq.Solid.makeCylinder(0.85, 9.0, cq.Vector(px, 17.5 if sy > 0 else -26.5, pz), cq.Vector(0, 1, 0))
        body = body.fuse(block).cut(pilot)

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
    led_v_hi = span_v / 2.0 - LED_TOP_MARGIN
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
    drl_pad = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w + 1.6, STRIP_H + 1.6)
        .extrude(WALL - 0.02)
        .translate(cq.Vector(*(n_dir * (-GROOVE_D - WALL))))
        .val()
    ).intersect(lower_env)
    body = body.fuse(drl_pad)

    led_bar = (
        cq.Workplane(pl_fascia)
        .center(0.0, led_v_mid)
        .rect(led_w - 2 * FIT, STRIP_H - 2 * FIT)
        .extrude(GROOVE_D - 0.1)
        .translate(cq.Vector(*(n_dir * (-GROOVE_D + 0.05))))
        .val()
    )

    # B) Hexagonal Honeycomb Mesh Trophy Grille (fits cleanly inside pinched nose!)
    gr_v_hi = led_v_lo - 0.5      # (was 0.75; LED bar lowered by LED_TOP_MARGIN)
    gr_v_lo = -span_v / 2.0 + 0.5      # (was 0.65)
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
