"""v4: one printed TUB replaces Chassis_Rail_L/R + Slider_L/R; rear links pivot on the tub tail -> stl/v4/.
usage (repo root): BODY_DIR=<per-body stl dir> python scripts/18_tub.py
Reads stl/v3 (cage, shell, arms ...) and BODY_DIR (axle, wheels, driveshaft). Never writes stl/v3, stl/v2 or front parts.
Axes: X forward, Y left, Z up, mm.  All dimensions are named constants (edit and re-run)."""
import os, sys, math, itertools, numpy as np, trimesh
from scipy.optimize import brentq
sys.path.insert(0, "scripts"); import rearlib as R
SH = float(os.environ.get("REAR_SHIFT", 9.0))      # wheelbase +SH (107.5 -> 116.5): everything behind the transmission zone moves back; front untouched
R.shift_rear(SH)
ROOF_LIFT = float(os.environ.get("ROOF_LIFT", 3.2))   # rigid shell lift (roof Z 31.9 -> 35.1); the cage top tubes follow it
def _ss(t): t = np.clip(t, 0, 1); return t*t*(3 - 2*t)
def lift(m, H=None):
    H = ROOF_LIFT if H is None else H; m = m.copy(); v = m.vertices.copy(); x, z = v[:, 0], v[:, 2]
    fx = _ss((x + 46.0)/10.0)*(1 - 0.85*_ss((x - 8.0)/16.0)); v[:, 2] += H*_ss((z - 17.0)/12.0)*fx; m.vertices = v; return m
BD = os.environ["BODY_DIR"]; V3 = os.environ.get("V3_DIR", "stl/v3/"); OUT = "stl/v4/"; os.makedirs(OUT, exist_ok=True)
def ld(n, src=V3): return trimesh.load(src + n + ".stl")
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
def ysp(sg, a_, b_): return (min(sg*a_, sg*b_), max(sg*a_, sg*b_))
def U(ms): return trimesh.boolean.union(list(ms), engine="manifold")
def D(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def I(a, b): return trimesh.boolean.intersection([a, b], engine="manifold")
def cylY(x, z, r, y0, y1, n=32):
    c = trimesh.creation.cylinder(radius=r, height=abs(y1 - y0), sections=n); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1)/2, z]); return c
def cylZ(x, y, r, z0, z1, n=32):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=n); c.apply_translation([x, y, (z0 + z1)/2]); return c
def mir(m): m = m.copy(); m.apply_transform(np.diag([1, -1, 1, 1.0])); return m
def open_edges(m): _, c = np.unique(m.edges_sorted, axis=0, return_counts=True); return int((c == 1).sum())
def vol(a, b):
    try: return abs(trimesh.boolean.intersection([a, b], engine="manifold").volume)
    except Exception: return float('nan')

# ================= spec (locked by the orchestrator) =================
DELTA = 5.8     # Rev 42 (Option A): the whole tub stack (floor, gearbox, motor, ESC/RX, servo pocket) is raised by DELTA; suspension, rear pads/deck/bump stops, cage feet stay at absolute Z
GBX_X, GBX_Y, GBX_Z0, GBX_Z1, GBX_SHAFT_Z = 6.0, 12.5, -4.9 + DELTA, 23.1 + DELTA, 4.1 + DELTA     # transmission 12 (X) x 25 (Y) x 28 (Z) on the shaft line; output stubs still end at X +-15.5
STUB_END = 15.5
MOTOR_R, MOTOR_L, MOTOR_Z = 8.5, 10.0, 18.1 + DELTA; MOTOR_X1 = -GBX_X; MOTOR_X0 = MOTOR_X1 - MOTOR_L; MOTOR_Y = 0.0   # GM1614 in line behind the transmission, axis along X on the input axis (Z 18.1)
CLR = 0.4
# ================= tub =================
FLOOR_TOP = GBX_Z0; FLOOR_T = float(os.environ.get("FLOOR_T", 1.8)); FLOOR_BOT = FLOOR_TOP - FLOOR_T            # Rev 33: 1.8 mm floor, underside -6.7 (was 2.6 / -7.5 + 1.2 mm skid boss under the bulkhead screws -> -8.7)
TUB_MID = 20.7; WIDE_X1 = 22.5             # wide middle zone (X GBX_ZONE_X0..WIDE_X1) outer half width; shell inner face is Y 22.3
TUB_OUT = 17.9                       # outer half width (arms start at 18.0; shell inner face is Y 22.3)
WALL = 1.5; GBX_IN = 13.5            # gearbox/front zone inner half width; tail inner = TUB_OUT-WALL
TAIL_IN = TUB_OUT - WALL
RIM_Z = 8.0                          # cage lower tubes start at Z 8.1 -> they sit on this rim
TUB_X0, TUB_X1 = float(os.environ.get("TUB_X0", -40.6)), 36.0         # floor/walls end (floor ends before the axle housing, X>-47)
GBX_ZONE_X0 = -31.0                  # walls are 'gearbox zone' (inner 13.5) from here forward
# gearbox foot holes: GUESS (4 x M1.4 pilots, 20 x 20 mm square about the gearbox centre) -> measure and edit
FOOT_HOLES = [(-4.8, -9.4), (4.8, 9.4)]; FOOT_THRU_D = 1.7; FOOT_CB_D = 3.2; FOOT_CB_DEPTH = 1.0   # Modrax photo: 2 feet on the case bottom, diagonal (one per case half); M1.4 from below
# front bulkhead interface (positions read from the v3 rails: holes at (27, Z 2.5) and (33, Z 5.0)); screws along Y into the tub walls
BULK_X0 = 24.0; BULK_SCREWS = [(27.0, 2.5), (33.0, 5.0)]; BULK_PILOT_D = 1.2
BULK_WALL_Y0, BULK_WALL_Y1 = 11.5, 13.0       # bulkhead side walls (holes dia 1.4..1.6 measured): M1.6 x 5 self-tapping from outside
BULK_THRU_D, BULK_CB_D, BULK_CB_DEPTH = 2.0, 3.3, 1.4; BULK_SCR_R, BULK_HEAD_R = 0.8, 1.5
SERVO_BAY = dict(x0=12.0, x1=36.0, y0=2.0, y1=13.45, z0=FLOOR_TOP, z1=FLOOR_TOP + 16.0)   # EDITABLE keep-out/cavity (documented; tub is open here)
SHAFT_TUNNEL_D = 3.2                 # front shaft tunnel is in the bulkhead (unchanged, front session)
# electronics (ASSUMED sizes, edit freely)
ESC_L, ESC_W, ESC_H = 30.0, 17.0, 6.0        # Rev 37: user-measured ESC 30 (L) x 17 (D) x 6 (H). Stands on its edge in the side bay: L vertical (Z), D along X, H across Y (the bay is 19 x 9 mm, so L=X flat does not fit)
RX_L, RX_W, RX_H = 20.0, 15.0, 7.0           # Rev 37: user-measured RX 20 (L) x 15 (D) x 7 (H); in the same side pocket it stands L vertical, D along X, H across Y
BATT_L_SPEC, BATT_L, BATT_W, BATT_H = 45.3, 45.3, 21.0, 11.2   # 50 mm does NOT fit lengthwise anywhere (see TUB_V4.md); BATT_L is the longest that fits the deck
NOSE_L, NOSE_RISE_L, NOSE_W = 19.5, 5.5, 8.0; BFS = [(44.5, 3.5), (44.5, -3.5), (52.0, 3.5), (52.0, -3.5)]   # bulkhead floor screw holes (read from stl/v3 IFS_Bulkhead_Convertible: 4 vertical D1.8 holes in the floor web, head seat Z ~ +0.5)
NOSE_TOP = -0.9; NOSE_BOT = NOSE_TOP - 1.8      # Rev 43: bulkhead web underside is Z -0.9 (FRONT_V3 UPDATE 14, web top +0.5, seats +0.8) -> skid top = web bottom
#      # Rev 42: bulkhead floor web (stl/v3) is Z -2.0..0.5, so the sunken skid under it tops out at -2.0 (underside -4.0) until the web is raised
BFS_PILOT_D = 1.2; BFS_HEAD_Z = 0.35; TAIL_KICK_L, TAIL_KICK_H = 7.0, 0.9
STRAP_SLOT_X = (-73.0, -54.5)      # 4 slots: X centres, |Y| 11.7..13.9
CHAMF = 3.2; BUMP_X0, BUMP_X1, BUMP_Z0 = -67.5, -61.0, 7.2       # lower-edge chamfer; bump pad: axle top Z 7.0 + 4.04 (ta +7) = 11.04, 0.36 gap
BAY_CLR = 0.4
# ESC / RX: two identical flat floor pockets (26.4 across Y x 18.4 along X, sunk 2 mm) under the shaft line, behind the transmission
# rear deck (battery + receiver) over the axle housing, bridge sits on it
X_FOOT, Y_FOOT = -43.0, 13.7
DECK_X1, DECK_X0 = -26.9, -35.0; DECK_Z0, DECK_Z1 = 14.7, 16.3   # raised so the axle-top bracket (top Z ~11 + 4 bump) clears
DECK_BOT_RIB = 0.0; BEAM_Y0 = 14.5; BEAM_Z0 = 11.2
BATT_X1 = -29.4; BATT_X0 = BATT_X1 - BATT_L - BAY_CLR; BATT_Y1 = (BATT_W + 0.8)/2; BATT_Y0 = -BATT_Y1     # 44.5 along X, 20 across Y, 10.5 tall, centred on the deck
# side bays (identical, left + right of the motor): ESC upright on the floor, shelf above it carries the receiver upright (either part fits either level)
EL_X1 = -(GBX_X + 0.4); EL_X0 = GBX_ZONE_X0 + 1.5; BAY_Y0, BAY_Y1 = 10.2, TUB_MID - WALL
SH_Z0 = FLOOR_TOP + ESC_L + 1.0; SH_T = 1.0; SH_Z1 = SH_Z0 + SH_T                 # shelf plate Z
EL_PX = ((EL_X0, EL_X0 + 2.0), (EL_X1 - 2.0, EL_X1)); EL_PY = ((10.8, 13.2), (BAY_Y1 - 2.4, BAY_Y1))     # four shelf pillars per side (X end, Y inner/outer)
EL_CX = (EL_PX[0][1] + EL_PX[1][0])/2                                           # centre X of the bay between the pillars
FENCE_H = 4.5
# ================= rear 4-link on the tub tail =================
PL = np.array([float(os.environ.get("PL_X", -26.0 - SH)), float(os.environ.get("PL_Z", 2.4))])      # trailing arm pivot (X,Z)
PU = np.array([float(os.environ.get("PU_X", -24.53 - SH)), float(os.environ.get("PU_Z", 12.0))])       # upper link pivot
A_ = np.array([-59.0 - SH, -0.5]); BP = np.array([-57.5 - SH, 8.85])       # axle-side points (unchanged)
LUG_X = 3.2; LUG_R = 3.4; PAD_Y0, PAD_Y1 = 14.4, TAIL_IN          # pad inside the tail wall, in which the pivots are tapped
PIV_PILOT_D = 1.25; PIV_CLEAR_D = 1.6
ARM_Y0, ARM_Y1 = 18.0, 23.4          # trailing arm Y span (v3)
# Rev 40: 3-hole anti-squat adjustment (Yeti p56): holes stacked at AS_PITCH on the arc of constant link length about the axle bracket pin, TOP hole = default (current geometry; the deck above leaves no room for holes higher), two lower holes
AS_PITCH = 2.0; _LL = float(np.linalg.norm(BP - PU))
PU_HOLES = [np.array([BP[0] + math.sqrt(_LL**2 - (PU[1] + k_*AS_PITCH - BP[1])**2), PU[1] + k_*AS_PITCH]) for k_ in (-2, -1, 0)]
UL_YB = PAD_Y0 - 1.35; BR_EYE_T = 1.1                # upper-link boss centre Y (boss 2.7 thick against the pad)
PRIDE_ARM_CUT = PL[0] + LUG_R + 0.1
TRAVEL = [-14, -10, -5, 0, 3, 7]
R.PL = PL; R.PU = PU; R.A = A_; R.Bp = BP

P = {}      # exported parts
# ---------------- reference boxes ----------------
def gbx_ref():
    b = box(-GBX_X, GBX_X, -GBX_Y, GBX_Y, GBX_Z0, GBX_Z1)
    st = []
    for s in (-1, 1):
        L_ = STUB_END - GBX_X
        c = trimesh.creation.cylinder(radius=1.5, height=L_ + 0.01, sections=24); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
        c.apply_translation([s*(GBX_X + L_/2 - 0.005), 0, GBX_SHAFT_Z]); st.append(c)
    return U([b] + st)
def cylX(y, z, r, x0, x1, n=32):
    c = trimesh.creation.cylinder(radius=r, height=abs(x1 - x0), sections=n); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    c.apply_translation([(x0 + x1)/2, y, z]); return c
def motor_ref(): return cylX(MOTOR_Y, MOTOR_Z, MOTOR_R, MOTOR_X0, MOTOR_X1, 48)
ESC_BOX = box(EL_CX - ESC_W/2, EL_CX + ESC_W/2, -((BAY_Y0 + BAY_Y1)/2 + ESC_H/2), -((BAY_Y0 + BAY_Y1)/2 - ESC_H/2), FLOOR_TOP, FLOOR_TOP + ESC_L)     # ESC upright in the right (-Y) bay
RX_BOX = box(0.0, RX_L, -RX_W/2, RX_W/2, 24.2, 24.2 + RX_H)      # receiver 22 x 14 x 7 lying FLAT above the centreline in front of the transmission (the shelf bay is only 7 mm wide); keep-out only, mount not modelled
PACK_Z0 = DECK_Z1 + 1.8                       # pack floor: rests on 2 deck ribs (front) and 3 cradle rungs (behind the deck end)
BATT_BOX = box(BATT_X0 + BAY_CLR/2, BATT_X1 - BAY_CLR/2, BATT_Y0 + 0.4, BATT_Y1 - 0.4, PACK_Z0, PACK_Z0 + BATT_H)
P["Ref_Gearbox"] = gbx_ref(); P["Ref_Motor_GM1614"] = motor_ref()
P["Ref_ESC_G40"] = ESC_BOX      # RX_BOX / BATT_BOX are internal keep-out volumes only (cage/deck/shell clearance), NOT exported as bodies

# ---------------- the tub ----------------
def build_tub():
    parts = [box(TUB_X0, TUB_X1, -TUB_OUT, TUB_OUT, FLOOR_BOT, FLOOR_TOP), box(GBX_ZONE_X0, WIDE_X1, -TUB_MID, TUB_MID, FLOOR_BOT, FLOOR_TOP)]   # local skid pad under the sunk ESC pocket
    for s in (1, -1):
        def yy(a, b): return (min(s*a, s*b), max(s*a, s*b))
        # gearbox/front zone walls
        y0, y1 = yy(GBX_IN, TUB_OUT); parts.append(box(WIDE_X1, TUB_X1, y0, y1, FLOOR_TOP - 0.01, RIM_Z))
        y0, y1 = yy(TUB_MID - WALL, TUB_MID); parts.append(box(GBX_ZONE_X0, WIDE_X1, y0, y1, FLOOR_TOP - 0.01, RIM_Z))                       # wide middle zone side wall
        y0, y1 = yy(GBX_IN, TUB_MID); parts.append(box(WIDE_X1 - 1.5, WIDE_X1 + 0.01, y0, y1, FLOOR_TOP - 0.01, RIM_Z))                      # front closing wall of the wide zone
        y0, y1 = yy(TAIL_IN, TUB_MID); parts.append(box(GBX_ZONE_X0, GBX_ZONE_X0 + 1.5, y0, y1, FLOOR_TOP - 0.01, RIM_Z))                     # rear closing wall
        # tail walls (thin), raised to the deck behind the motor
        y0, y1 = yy(TAIL_IN, TUB_OUT)
        parts.append(box(TUB_X0, GBX_ZONE_X0 + 0.01, y0, y1, FLOOR_TOP - 0.01, RIM_Z))
        parts.append(box(TUB_X0, DECK_X1, y0, y1, RIM_Z - 0.01, DECK_Z1))
        # pivot pads inside the tail wall (trailing lug + upper-link lug)
        y0, y1 = yy(PAD_Y0, TAIL_IN + 0.01)
        parts.append(box(PL[0] - LUG_X - 1.0, PU[0] + LUG_X + 0.5, y0, y1, min(FLOOR_TOP, PL[1] - LUG_R - 0.8) - 0.01, PU[1] + 2.6))   # corner link block: lower pivot low/outer, upper pivot above it
        y0, y1 = yy(PAD_Y0, TUB_OUT); parts.append(box(PL[0] - LUG_X - 1.0, PL[0] + LUG_X + 1.0, y0, y1, min(FLOOR_TOP, PL[1] - LUG_R - 0.8) - 0.01, FLOOR_TOP + 0.5))      # Rev 42: boss drops through the tail wall to the (fixed) trailing-arm pivot: wall + pad reach down behind the arm boss
        # step wall between gearbox zone and tail (closes the cavity behind the gearbox, keeps motor/shaft open above)
    parts.append(box(DECK_X0, DECK_X1, -TUB_OUT, TUB_OUT, DECK_Z0, DECK_Z1))             # deck (ends at the link pads: the rear cage is cantilevered behind it)
    for s_ in (1, -1):
        parts.append(box(DECK_X0 + 0.5, -30.5, *ysp(s_, 4.5, 8.5), DECK_Z1 - 0.01, PACK_Z0))                       # pack ribs on the deck
    for s_ in (1, -1): parts.append(box(BUMP_X0, BUMP_X1, *ysp(s_, 10.0, 14.0), BUMP_Z0, DECK_Z0 + 0.01))      # bump stops under the deck: axle top touches at ta +7
    for s_ in (1, -1):
        parts.append(box(BULK_X0 - 1.0, BULK_X0 - 0.1, min(s_*11.9, s_*GBX_IN + s_*0.3), max(s_*11.9, s_*GBX_IN + s_*0.3), FLOOR_TOP - 0.01, FLOOR_TOP + 3.9))       # bulkhead rear stop (X) beside each wall; floor is the Z seat
        if DECK_X0 < -50.0: parts.append(box(DECK_X0, -50.0, min(s_*BEAM_Y0, s_*TUB_OUT), max(s_*BEAM_Y0, s_*TUB_OUT), BEAM_Z0, DECK_Z0 + 0.01))       # side beam under the extended deck (cage feet screw into it)
    # skid nose under the bulkhead: sloped kick-up (profile in XZ, extruded across Y)
    nose = trimesh.convex.convex_hull(np.array([[x, y, z] for (x, z) in ((WIDE_X1 - 1.0, NOSE_BOT), (TUB_X1 + NOSE_L - NOSE_RISE_L, NOSE_BOT), (TUB_X1 + NOSE_L, NOSE_TOP - 0.4), (TUB_X1 + NOSE_L, NOSE_TOP), (WIDE_X1 - 1.0, NOSE_TOP)) for y in (-NOSE_W, NOSE_W)]))
    parts.append(box(WIDE_X1 - 1.0, WIDE_X1 + 0.5, -NOSE_W, NOSE_W, NOSE_BOT, FLOOR_TOP))      # riser joining the sunken skid to the main floor
    parts.append(nose)
    pass  # Rev 33: skid boss under the bulkhead floor screws removed (ground clearance)
    #parts.append(box(43.0, 54.0, -5.6, 5.6, FLOOR_BOT - 1.2, FLOOR_BOT + 0.01))   # local boss under the bulkhead floor screws (deeper pilots)
    m = U(parts)
    cuts = []
    for s_ in (1, -1):      # lightening windows (stadium, 7 mm high) in the wide-zone side walls between the axle zones; 2.5+ mm rim above, 3.4 mm below, 11 mm post between
        for (wx0, wx1) in ((-25.5, -6.5), (4.5, 18.5)):
            wz0, wz1 = FLOOR_TOP + 1.6, RIM_Z - 1.6; wr = (wz1 - wz0)/2; wy = ysp(s_, 18.9, 21.2)
            cuts.append(trimesh.convex.convex_hull(np.vstack([cylY(wx0 + wr, (wz0 + wz1)/2, wr, *wy, 32).vertices, cylY(wx1 - wr, (wz0 + wz1)/2, wr, *wy, 32).vertices])))     # stadium = hull of the two end cylinders: straight flats, no coplanar union seams
    for s_ in (1, -1):      # slimmer side profile: 45 deg chamfer along the lower outer edge of the rear/middle tub (less 'slab' under the shell sill)
        cuts.append(trimesh.convex.convex_hull(np.array([[TUB_X0 - 0.1 if x_ == 0 else WIDE_X1, s_*y_, z_] for x_ in (0, 1) for (y_, z_) in ((TUB_OUT - CHAMF, FLOOR_BOT - 0.1), (TUB_MID + 1.0, FLOOR_BOT - 0.1), (TUB_MID + 1.0, FLOOR_BOT - 0.1 + TUB_MID + 1.0 - TUB_OUT + CHAMF))])))
    CH_F = 1.6      # Rev 40: front-zone belly chamfer (smaller: the floor must still carry the walls, 1.1 mm left at the wall face)
    for s_ in (1, -1):
        cuts.append(trimesh.convex.convex_hull(np.array([[WIDE_X1 - 0.1 if x_ == 0 else TUB_X1 + 0.1, s_*y_, z_] for x_ in (0, 1) for (y_, z_) in ((TUB_OUT - CH_F, FLOOR_BOT - 0.1), (TUB_OUT + 1.0, FLOOR_BOT - 0.1), (TUB_OUT + 1.0, FLOOR_BOT - 0.1 + 1.0 + CH_F))])))
    # tail kick-up: wedge under the floor end (keeps >= 1.7 mm floor)
    cuts.append(trimesh.convex.convex_hull(np.array([[x, y, z] for (x, z) in ((TUB_X0 - 0.1, FLOOR_BOT - 0.1), (TUB_X0 + TAIL_KICK_L, FLOOR_BOT - 0.1), (TUB_X0 - 0.1, FLOOR_BOT + TAIL_KICK_H)) for y in (-TUB_OUT - 1, TUB_OUT + 1)])))
    # ESC floor pocket (sunk) + ESC wire notch through the gearbox-side wall of the cavity
    # gearbox foot pilots (blind, from the top)
    for fx, fy in FOOT_HOLES: cuts += [cylZ(fx, fy, FOOT_THRU_D/2, FLOOR_BOT - 0.01, FLOOR_TOP + 0.01, 16), cylZ(fx, fy, FOOT_CB_D/2, FLOOR_BOT - 0.01, FLOOR_BOT + FOOT_CB_DEPTH, 20)]
    # bulkhead pilots through each front wall (axis Y, from the outside)
    for s in (1, -1):
        for bx, bz in BULK_SCREWS:
            if s == 1 and bx == 33.0: continue      # +Y (33, 5) is the servo-ear position (ears X 32.3..33.5, Y 11.7..14.65): no bulkhead screw there
            cuts += [cylY(bx, bz, BULK_THRU_D/2, s*(GBX_IN - 0.3), s*(TUB_OUT + 0.01), 20), cylY(bx, bz, BULK_CB_D/2, s*(TUB_OUT - BULK_CB_DEPTH), s*(TUB_OUT + 0.01), 24)]   # through the whole wall + head counterbore (screws thread into the bulkhead)
        # trailing arm pivot pilot (from outside), upper link pivot pilot (from inside the pad -> through wall? both blind from outside)
        cuts.append(cylY(PL[0], PL[1], PIV_PILOT_D/2, s*(PAD_Y0 + 0.6), s*(TUB_OUT + 0.01), 16))
        for _h in PU_HOLES: cuts.append(cylY(_h[0], _h[1], 0.75, s*(PAD_Y0 - 0.01), s*(TUB_OUT + 0.01), 16))      # three D1.5 clearance holes (default = top)
        # deck bays: pack and receiver are retained by lips (below), wire slots through the deck
    for xs, ys in ((-30.0, -2.0), (-30.0, 2.0)): cuts.append(box(xs - 1.2, xs + 1.2, ys - 0.7, ys + 0.7, DECK_Z0 - 0.01, DECK_Z1 + 0.01))
    for s_ in (1, -1):
        for sx_ in STRAP_SLOT_X: cuts.append(box(sx_ - 4.0, sx_ + 4.0, *ysp(s_, 11.7, 13.9), DECK_Z0 - 0.1, DECK_Z1 + 0.1))      # 8 x 2.2 mm strap slots through the deck
    for s_ in (1, -1): cuts.append(box(EL_X0 - 0.01, DECK_X1 + 0.01, *ysp(s_, BAY_Y0, TUB_OUT + 0.01), RIM_Z - 0.02, DECK_Z1 + 0.01))    # the side bays reach under the deck front: notch the deck + raised tail wall there
    m = D(m, cuts)
    return m
SHK_UX, SHK_UZ = -38.2, 27.8                                          # rear shock upper pin (X,Z), M1.6 along Y
# Rev 39: REAL shock envelope (Injora 36 mm big bore class: 36 mm eye-to-eye, 7..9 mm travel assumed): nylon bottom end with brass eye bushing, D8 body, D10 spring, D2 rod, 3.0 mm wide eyes
SHK_LO = np.array([-44.0 - SH, 0.5]); SHK_UP = np.array([SHK_UX, SHK_UZ]); SHK_L0 = float(np.linalg.norm(SHK_UP - SHK_LO)); SHK_U = (SHK_UP - SHK_LO)/SHK_L0
SHK_YL, SHK_YU, SHK_EYE_T, SHK_EYE_R = 20.4, 18.2, 3.0, 2.2                # |Y| of the lower eye (arm clevis) / upper eye (tab on the tail post); eye width (assumed); eye boss R
SHK_BAR_R, SHK_SPR_R, SHK_ROD_R, SHK_BORE_R = 4.0, 4.75, 1.0, 0.85           # body D8, spring OD D9.5, rod D2, eye bushing bore D1.7
SHK_NECK_S, SHK_BAR_S0, SHK_SPR_S0, SHK_SPR_S1 = 7.5, 7.5, 10.5, SHK_L0 - 8.0    # lower end neck (between the clevis cheeks) -> body -> spring span (s from the lower pin)
SHK_TOP_S = SHK_L0 - 4.4
CG_YS = 11.5                                                          # |Y| of the cage tail section beside the shock (balls 12.95 -> 1.0 off the D9.5 spring)
def _axcyl(p0, p1, r): return trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1], float), sections=24)
def _spt(s_, sg=1): return np.array([SHK_LO[0] + SHK_U[0]*s_, sg*(SHK_YL + (SHK_YU - SHK_YL)*s_/SHK_L0), SHK_LO[1] + SHK_U[1]*s_])
def shock_T(a):
    lo_t = R.rot(SHK_LO, a, PL); d0 = SHK_UP - SHK_LO; dt = SHK_UP - lo_t
    dl = math.atan2(d0[1], d0[0]) - math.atan2(dt[1], dt[0]); M = R.roty(dl, SHK_LO[0], SHK_LO[1]); M[0, 3] += lo_t[0] - SHK_LO[0]; M[2, 3] += lo_t[1] - SHK_LO[1]; return M
def _posed_keep(m, ta): m = m.copy(); m.apply_transform(shock_T(R.solve(ta)[0])); return m
SPRING_KEEP = [_posed_keep(_axcyl(_spt(SHK_SPR_S0 - 1.0, sg), _spt(SHK_SPR_S1 + 1.0, sg), SHK_SPR_R + 1.0), ta_) for sg in (1, -1) for ta_ in (-14, -10, -5, 0, 4, 7.5)]      # 1.0 mm keep-out swept by the spring over the travel (relief in the tub lug and the cage tail foot)
SHELL = ld("Tacoma_Trophy_Body_Shell")
# ---- Rev 32: ONE new cage (scrapped Roll_Cage + Rear_Cage). Constants shared with the tub (lugs / rim bosses are part of the tub)
CG_ZR, CG_YT, CG_YM = 30.4, 15.7, 21.5            # roof rail height; tail rail |Y| (inside the tail walls); main rail |Y| (outside the ESC shelves, 1.15 mm off the shelf pillars)
CG_XM0, CG_XM1, CG_XT, CG_XC = -28.3, -7.6, -38.2, -3.1        # main posts (on the rim columns) and tail post X
CG_ZF, CG_ZTF = 9.6, 17.8                         # main foot node Z (rim top 8.0 + plate 1.3), tail foot node Z (lug top 16.3 + plate 1.1)
CG_SCREWS = {1: (CG_XM0 + 3.4, 21.5, 9.3, 8.0), 2: (CG_XM1 + 3.4, 21.5, 9.3, 8.0), 3: (-35.4, 16.1, 17.8, 16.3)}      # k: (x, |y|, head seat Z, tub seat Z); 3 x M1.6 per side
# Rev 42: Cabin_Cage (separate printed part: rear hoop, A-pillars, hood rails, front hoop). Post X, tube |Y|, roof/hood heights, bolt-on feet
CAB_X1, CAB_X2, CAB_XF = 1.2, 17.0, 26.0                 # rear hoop post / cowl post / front hoop post (X)
CAB_YC, CAB_YF = 20.4, 17.0                               # tube |Y| roof + pillars / front hoop legs
CAB_ZR, CAB_XA, CAB_ZH = 35.1, 5.0, 26.8                  # roof level, A-pillar top X, hood-rail level
CAB_SCREWS = {4: (CAB_X1 + 3.4, 21.5, 9.3, 8.0), 5: (CAB_X2 + 3.4, 21.5, 9.3, 8.0), 6: (CAB_XF + 2.7, CAB_YF, 9.3, 8.0)}     # M1.6 into the rim bosses (4, 5) and the front wall top (6)
ALL_SCREWS = {**CG_SCREWS, **CAB_SCREWS}
FEET = []
for sg in (1, -1):
    for k in (1, 2, 4, 5): bx = ALL_SCREWS[k][0]; FEET.append(box(bx - 5.0, bx + 1.6, *ysp(sg, 20.6, 23.0), 0.0, 8.0))      # boss on the outer face of the rim (screw lands in boss + rim)
    FEET.append(box(TUB_X0, -33.6, *ysp(sg, PAD_Y0, TUB_OUT), 14.5, 16.3))                              # tail lug block on the pad
def single(name, m, vmin=1.0):
    sh = sorted(m.split(only_watertight=False), key=lambda q: -abs(q.volume))
    for q in sh[1:]:
        if abs(q.volume) > vmin: print("  dropped fragment of", name, "vol %.1f" % abs(q.volume), "bounds", q.bounds.round(1).tolist())
    return sh[0]
TUB = build_tub(); EARS = box(32.3, 33.5, 11.7, 14.65, -3.8 + DELTA, 9.4 + DELTA); SERVO_CLR = 0.3      # Rev 31: servo case bottom is Z -5.0 (0.1 below the floor top): shallow planar pocket under the case footprint + ear slot opened to the screw heads
TUB = D(TUB, [box(31.9 - SERVO_CLR - 0.25, 33.9, 11.3, 15.05, -4.2 + DELTA, 9.8 + DELTA), box(13.37 - SERVO_CLR, 33.5 + SERVO_CLR, 1.8 - SERVO_CLR, 11.7 + SERVO_CLR, FLOOR_TOP - 0.4 - SERVO_CLR + 0.3, FLOOR_TOP + 0.01)]); TUB = U([TUB, box(33.65, 36.1, 11.85, 13.6, FLOOR_TOP - 0.01, 9.4 + DELTA)] + FEET); 
def hexribs(x0, x1, y0, y1, pitch=6.0, w=1.0, h=1.5):
    cx, cy = (x0 + x1)/2, (y0 + y1)/2; L = (x1 - x0) + (y1 - y0); ribs = []
    for ang in (0, 60, 120):
        a = math.radians(ang)
        for k in range(-int(L/pitch) - 1, int(L/pitch) + 2):
            b = trimesh.creation.box(extents=[2*L, w, h + 0.05]); b.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1]))
            b.apply_translation([cx - k*pitch*math.sin(a), cy + k*pitch*math.cos(a), FLOOR_TOP - 0.05 + (h + 0.05)/2]); ribs.append(b)
    return I(U(ribs), box(x0, x1, y0, y1, FLOOR_TOP - 0.05, FLOOR_TOP + h))
RIB_AREAS = [(GBX_X + 1.0, BULK_X0 - 0.8, -GBX_IN + 0.3, -2.2)]
TUB = U([TUB] + [hexribs(*r_) for r_ in RIB_AREAS])
# ---- equipment seats in the wide zone: (motor clamp and saddle removed, Rev 36) ESC towers + clips, Rx clips, battery hold-down bosses ----
def ysp(sg, a_, b_): return (min(sg*a_, sg*b_), max(sg*a_, sg*b_))
EL_PIL = [(sg, (x0_ + x1_)/2, (y0_ + y1_)/2) for sg in (1, -1) for (x0_, x1_) in EL_PX for (y0_, y1_) in EL_PY]       # pillar centres (side, X, |Y|)
EL_PILLARS = [box(x0_, x1_, *ysp(sg, y0_, y1_), FLOOR_TOP - 0.01, SH_Z0 + 0.01) for sg in (1, -1) for (x0_, x1_) in EL_PX for (y0_, y1_) in EL_PY]
RAIL_Y = 12.0; XF = -28.2; HOOK_Y = 6.5; HOOK_SX = -28.1; HOOK_SY = 9.2          # battery cradle rail line (|Y|) and its two deck feet
RAIL_BOSS = []      # no battery hooks any more (cradle removed)
TUB = U([TUB] + EL_PILLARS + RAIL_BOSS)
TUB = D(TUB, [cylZ(px_, sg_*py_, 0.6, SH_Z0 - 4.5, SH_Z0 + 0.02, 12) for sg_, px_, py_ in EL_PIL]
        + [cylZ(bx, by, BFS_PILOT_D/2, NOSE_BOT + 0.1, NOSE_TOP + 0.01, 12) for bx, by in BFS])
TUB = D(TUB, SPRING_KEEP)      # Rev 39: relief in the tail lug block around the D9.8 shock spring
TUB = single('Chassis_Tub', TUB); P["Chassis_Tub"] = TUB; print("cage feet columns added:", len(FEET))

# ---------------- kinematics (own copy of the rearlib maths, any pivots) ----------------
def kin(ta):
    a, u, T = R.solve(ta); return a, u, T
rows = []
def kin_table():
    out = []
    L_arm = np.linalg.norm(A_ - PL); L_link = np.linalg.norm(BP - PU)
    T0 = kin(0)[2]
    shock_lo0 = np.array([-44.0 - SH, 0.5]); shock_up = np.array([SHK_UX, SHK_UZ])
    G = np.array([-15.5, GBX_SHAFT_Z]); Q = np.array([-46.8 - SH, 3.5])      # driveshaft: gearbox stub end -> axle input (U-joint centre)
    for ta in [-14, -12, -10, -8, -5, -2, 0, 2, 4, 6, 7, 9]:
        a, u, T = kin(ta)
        Ax = (T @ np.array([A_[0], 0, A_[1], 1]))[[0, 2]]
        Qr = (T @ np.array([Q[0], 0, Q[1], 1]))[[0, 2]]
        # axle pitch: rotation of the axle body = angle of T about Y
        pitch = -math.degrees(math.atan2(T[2, 0], T[0, 0]))    # sign: + = nose up (same convention as arm angle)
        lo = R.rot(shock_lo0, a, PL); shock = np.linalg.norm(lo - shock_up)
        v = Qr - G; dl = np.linalg.norm(v); dang = math.degrees(math.atan2(v[1], v[0])) if v[0] > 0 else math.degrees(math.atan2(-v[1], -v[0]))
        out.append((ta, Ax[0] - A_[0], Ax[1] - A_[1], pitch, math.degrees(u), shock, dl, dang))
    return out
KT = kin_table()
# instant centre and anti-squat at ride
def ic_antisquat():
    a, u, T = kin(0); L = R.rot(A_, a, PL); U_ = R.rot(BP, u, PU)
    d1 = L - PL; d2 = U_ - PU                       # lower line (PL->axle), upper line (PU->axle)
    M = np.array([d1, -d2]).T; t = np.linalg.solve(M, PU - PL); ic = PL + t[0]*d1
    zg = -16.5; x_r = -59.0 - SH; wb = 48.0 - x_r; hcg = 21.5 - zg   # CG height above ground ~21.5 (CG Z 5.0), wheelbase 107 (assumed)
    slope = (ic[1] - zg)/(x_r - ic[0]) if ic[0] != x_r else float('inf')
    return ic, 100*abs(slope)/(hcg/wb), L_arm_len()
def L_arm_len(): return float(np.linalg.norm(A_ - PL)), float(np.linalg.norm(BP - PU))
IC, AS, LEN = ic_antisquat()
def _as_hole(h):
    a, u, T = kin(0); L = R.rot(A_, a, PL); U_ = BP.copy(); d1 = L - PL; d2 = U_ - h
    M = np.array([d1, -d2]).T; t = np.linalg.solve(M, h - PL); ic = PL + t[0]*d1
    zg = -16.5; x_r = -59.0 - SH; wb = 48.0 - x_r; hcg = 21.5 - zg
    return 100*(ic[1] - zg)/(ic[0] - x_r)/(hcg/wb), ic
print("--- anti-squat holes (upper-link tub pad, pitch %.1f; %% on the same assumed CG/wheelbase as the default) ---" % AS_PITCH)
for _k, _h in zip(("lowest", "middle", "TOP (default)"), PU_HOLES): print("   hole %-16s Z %.2f X %.2f  anti-squat %6.1f %%  IC (%.1f, %.1f)" % (_k, _h[1], _h[0], _as_hole(_h)[0], *_as_hole(_h)[1]))

# ---------------- trailing arm / upper link rebuilt for the tub-tail pivots ----------------
AXL_END = None
def _axl():
    ax = I(R.M['Ref_Rear_Axle'], box(-70 - SH, -45 - SH, -40, 40, -10, 10)); return dil_(ax, 0.1)
def dil_(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)):   # 18 directions (axes + edge diagonals): thin parts left diagonal gaps with only 6
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
AXL_END = _axl()
def arm(side):
    old = ld("Rear_Trailing_Arm_L"); keep = I(old, box(-70, PRIDE_ARM_CUT + SH, -40, 40, -20, 20)); keep.apply_translation([-SH, 0, 0])     # keep the rear part (axle end, shock clevis, tab)
    ya0, ya1 = ARM_Y0, ARM_Y1
    boss = cylY(PL[0], PL[1], LUG_R, ya0, ya1, 40); neck = trimesh.convex.convex_hull(np.vstack([boss.vertices, box(PRIDE_ARM_CUT, PRIDE_ARM_CUT + 0.3, ya0, ya1, A_[1] - 1.6, A_[1] + 1.6).vertices]))   # pitched neck: pivot boss (high) -> axle-side keep part (low)
    m = U([keep, boss, neck]); m = D(m, [cylY(PL[0], PL[1], PIV_CLEAR_D/2, ya0 - .1, ya1 + .1, 20), AXL_END])
    return m if side > 0 else mir(m)
def link(side):      # diagonal upper link: corner pivot (Y axis) -> shared axle-top bracket (vertical pin at Y 0); L eye above R eye (0.1 gap)
    zc = BP[1] + (0.6 if side > 0 else -0.6)
    bs = trimesh.convex.convex_hull(np.vstack([cylY(_h[0], _h[1], 1.75, side*(UL_YB - 1.35), side*(UL_YB + 1.35), 28).vertices for _h in PU_HOLES]))
    eye = cylZ(BP[0], 0.0, 1.75, zc - BR_EYE_T/2, zc + BR_EYE_T/2, 28)
    blade = trimesh.convex.convex_hull(np.vstack([cylY(PU[0], PU[1], 1.75, side*(UL_YB - 1.35), side*(UL_YB + 1.35), 28).vertices, eye.vertices]))      # flat tapered blade, thin at the eye
    m = U([blade, bs, eye]); m = D(m, [cylY(_h[0], _h[1], PIV_PILOT_D/2, side*(UL_YB - 2), side*(UL_YB + 2), 20) for _h in PU_HOLES] + [cylZ(BP[0], 0.0, 0.8, zc - 2, zc + 2, 16)])
    xcut = PU[0] - 6.0
    # eye stacking only matters where the two blades meet at the bracket (|Y| < 2): L keeps above BP+0.05, R keeps below BP-0.05
    if side > 0: m = D(m, [box(-140, xcut, -2.0, 2.0, -5, BP[1] + 0.05)])
    else: m = D(m, [box(-140, xcut, -2.0, 2.0, BP[1] - 0.05, 20), box(-140, xcut, -2.0, 2.0, -5, BP[1] - 0.6 - BR_EYE_T/2)])
    return m
for sg, n in ((1, "L"), (-1, "R")):
    P[f"Rear_Trailing_Arm_{n}"] = D(arm(sg), [box(-53.2, -47.8, *ysp(sg, 18.8, 19.1), -2.0, 8.0), box(-53.2, -47.8, *ysp(sg, 21.4, 22.1), -2.0, 8.0), cylY(SHK_LO[0], SHK_LO[1], SHK_EYE_R + 0.5, *ysp(sg, 18.8, 22.1), 24), box(-51.6, -46.5, *ysp(sg, 18.45, 18.9), 1.5, 8.0)]);   # shock clevis: eye side clearance 0.45 -> 0.75 / 1.0 (cheeks stay >= 0.8 thick)
    P[f"Rear_Upper_Link_{n}"] = link(sg)
    # hardware: M1.4 pivot screws (tapped into the tub pads, no nuts), screw heads outside the arm / inside the link
    sh = U([cylY(PL[0], PL[1], 0.7, sg*(PAD_Y0 + 0.8), sg*(ARM_Y1), 12), cylY(PL[0], PL[1], 1.35, sg*ARM_Y1, sg*(ARM_Y1 + 0.9), 16)])
    P[f"HW_Rear_Trailing_Pivot_Screw_{n}"] = sh
    sh2 = U([cylY(PU[0], PU[1], 0.7, sg*(UL_YB - 0.95), sg*(TUB_OUT + 0.05), 12), cylY(PU[0], PU[1], 1.35, sg*TUB_OUT, sg*(TUB_OUT + 0.9), 16)])
    P[f"HW_Rear_Upper_Link_Pivot_Screw_{n}"] = sh2
# unchanged v3 parts we keep using (arms' hardware at the axle, shocks, axle-side link HW)
KEEP = ["HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R",
        "Ref_Shock_RL", "Ref_Shock_RR", "HW_Rear_Shock_Lower_Screw_L", "HW_Rear_Shock_Lower_Screw_R", "HW_Rear_Shock_Upper_Screw_L", "HW_Rear_Shock_Upper_Screw_R",
        "Ref_Rear_Driveshaft"]
def _stretch_ds(m):
    m = m.copy(); v = m.vertices.copy(); v[:, 0] -= SH*np.clip((-v[:, 0] - 22.0)/16.0, 0, 1); m.vertices = v; return m
for k in KEEP:
    P[k] = ld(k)
    if k == "Ref_Rear_Driveshaft": P[k] = _stretch_ds(P[k])
    else: P[k].apply_translation([-SH, 0, 0])
# Rev 39 shock bodies (constants above): lower nylon end (eye in the clevis, neck, offset body axis), D8 body, D10 spring, D2 rod; upper eye is the separate Ref_Shock_Eye part
for sg, nm in ((1, "L"), (-1, "R")):
    sh_ = U([cylY(SHK_LO[0], SHK_LO[1], SHK_EYE_R, *ysp(sg, SHK_YL - SHK_EYE_T/2, SHK_YL + SHK_EYE_T/2), 28),
             _axcyl(_spt(1.0, sg), _spt(SHK_NECK_S, sg), 1.4), _axcyl(_spt(SHK_NECK_S - 0.5, sg), _spt(SHK_SPR_S0 + 3.0, sg), SHK_BAR_R),
             _axcyl(_spt(SHK_SPR_S0, sg), _spt(SHK_SPR_S1, sg), SHK_SPR_R), _axcyl(_spt(SHK_SPR_S1 - 0.5, sg), _spt(SHK_TOP_S, sg), SHK_ROD_R)])
    P["Ref_Shock_R" + nm] = D(sh_, [cylY(SHK_LO[0], SHK_LO[1], SHK_BORE_R, *ysp(sg, SHK_YL - 2.5, SHK_YL + 2.5), 14)])
    _yi, _yo = SHK_YU - SHK_EYE_T/2, SHK_YU + SHK_EYE_T/2      # upper eye faces
    P[f"HW_Rear_Shock_Upper_Screw_{nm}"] = U([cylY(SHK_UX, SHK_UZ, 0.72, *ysp(sg, 12.4, _yo + 0.15), 12), cylY(SHK_UX, SHK_UZ, 1.3, *ysp(sg, _yi - 0.8, _yi), 14), cylY(SHK_UX, SHK_UZ, 1.1, *ysp(sg, _yo + 0.1, _yo + 0.9), 16)])
# rear unsprung bodies that used to be picked up from the per-body folder at the OLD wheelbase: export the shifted copies (R.shift_rear(SH) already applied to R.M), rest pose = identity
REAR_UNSPRUNG = ["Ref_Rear_Axle", "Ref_Wheel_RL", "Ref_Wheel_RR", "HW_Rear_Wheel_Nut_L", "HW_Rear_Wheel_Nut_R"]      # Rev 34: Upper_Link_Axle screw/nut (orphans: the link rides on HW_Rear_Axle_Top_Pin) and Shock_Lower nut (the lower shock screw is tapped into the arm's outer cheek) removed
for k in REAR_UNSPRUNG: P[k] = R.M[k].copy()
# Rev 34 fastener fixes: trailing-arm axle bolt = M2 through arm cheeks + axle ear, head outside, nut seated flush on the arm's inner cheek (Y 18.1) with the screw tip inside it;
# shock lower screw = M1.6 tapped into the arm's outer cheek (pilot through Y 21.5..23.4), tip lengthened to 23.3 (1.8 mm engagement)
ARM_IN_Y, AXB_NUT_T = 18.1, 1.3
# ---------------- Rev 43: rear droop stop (driveshaft angle limit) ----------------
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
DRV_MAX = 13.0      # deg: stock single U-joint limit at the axle
def drv_ang(ta_):
    a_, u_, T_ = kin(ta_); Qr = (T_ @ np.array([-46.8 - SH, 0, 3.5, 1]))[[0, 2]]; v_ = Qr - np.array([-15.5, GBX_SHAFT_Z]); return math.degrees(math.atan2(v_[1], v_[0])) if v_[0] > 0 else math.degrees(math.atan2(-v_[1], -v_[0]))
_lo, _hi = -14.0, 0.0
for _ in range(40):
    _m = (_lo + _hi)/2
    if drv_ang(_m) > DRV_MAX: _lo = _m
    else: _hi = _m
TA_D = round(_hi, 2)      # droop limit: arm angle where the rear shaft reaches DRV_MAX
STOP_BOX = lambda sg_: box(-39.6, -37.4, *ysp(sg_, 11.9, 14.2), FLOOR_TOP - 0.01, 14.0)
_stops = []
for sg_, n_ in ((1, "L"), (-1, "R")):
    _lk = posed(P[f"Rear_Upper_Link_{n_}"], R.roty(R.solve(TA_D)[1], PU[0], PU[1])); _in = I(_lk, STOP_BOX(sg_))
    _zt = float(_in.bounds[0][2]); _stops.append(box(-39.6, -37.4, *ysp(sg_, 11.9, 14.2), FLOOR_TOP - 0.01, _zt))      # clean post on the tub floor under the link blade: touches the link underside exactly at TA_D
    _lk0 = posed(P[f"Rear_Upper_Link_{n_}"], R.roty(R.solve(0)[1], PU[0], PU[1])); print("  droop stop %s: ta %.2f (shaft %.2f deg), post top Z %.2f, link underside at rest %.2f" % (n_, TA_D, drv_ang(TA_D), _zt, float(I(_lk0, STOP_BOX(sg_)).bounds[0][2])))
P["Chassis_Tub"] = single("Chassis_Tub(stop)", U([P["Chassis_Tub"]] + _stops))

for sg, n in ((1, "L"), (-1, "R")):
    _b = P[f"HW_Rear_Trailing_Axle_Screw_{n}"].bounds; _cx, _cz = (_b[0][0] + _b[1][0])/2, (_b[0][2] + _b[1][2])/2
    P[f"HW_Rear_Trailing_Axle_Screw_{n}"] = U([cylY(_cx, _cz, 0.9, *ysp(sg, ARM_IN_Y - AXB_NUT_T + 0.5, 21.65), 14), cylY(_cx, _cz, 1.4, *ysp(sg, 21.6, 22.7), 16)])      # M2 x ~5.7 under head
    P[f"HW_Rear_Trailing_Axle_Nut_{n}"] = D(cylY(_cx, _cz, 1.4, *ysp(sg, ARM_IN_Y - AXB_NUT_T - 0.15, ARM_IN_Y - 0.15), 6), [cylY(_cx, _cz, 0.9, *ysp(sg, ARM_IN_Y - AXB_NUT_T - 0.1, ARM_IN_Y + 0.1), 14)])
    _b = P[f"HW_Rear_Shock_Lower_Screw_{n}"].bounds; _cx, _cz = (_b[0][0] + _b[1][0])/2, (_b[0][2] + _b[1][2])/2
    P[f"HW_Rear_Shock_Lower_Screw_{n}"] = U([cylY(_cx, _cz, 1.1, *ysp(sg, 18.05, 18.8), 16), cylY(_cx, _cz, 0.72, *ysp(sg, 18.75, 23.3), 12)])      # M1.6 x ~4.5
# ---- axle-top bracket (printed, sits on the pumpkin) + vertical pin; links ride on the pin ----
BR_PLATE_Z0, BR_PLATE_Z1 = 7.0, BP[1] - 0.6 - BR_EYE_T/2 - 0.2      # plate sits on the two axle-top ridges (|Y| 1.7..3+, Z 7.0), 0.1 under the lower eye
BR_KEY = (BP[0] - 1.0, BP[0] + 1.0, -1.55, 1.55, 3.2, BR_PLATE_Z0)      # key drops into the 3.5 mm groove between the ridges (locates in Y, pin goes through it)
P["Rear_Axle_Top_Bracket"] = U([box(BP[0] - 3.4, BP[0] + 1.9, -4.5, 4.5, BR_PLATE_Z0, BR_PLATE_Z1), box(*BR_KEY[:4], BR_KEY[4], BR_KEY[5] + 0.01)])
P["Rear_Axle_Top_Bracket"] = D(P["Rear_Axle_Top_Bracket"], [cylZ(BP[0], 0.0, 0.6, 3.0, BR_PLATE_Z1 + 0.5, 14)])
_pt = BP[1] + 0.6 + BR_EYE_T/2
P["HW_Rear_Axle_Top_Pin"] = U([cylZ(BP[0], 0.0, 0.7, 3.4, _pt + 0.4, 14), cylZ(BP[0], 0.0, 1.3, _pt, _pt + 0.4, 16)])
# ---- rear cage (Rev 13, Yeti style): hoop + two side frames (upper rail, drop diagonals, post, battery cradle rails, half straps) ----
TUBE_R = 1.15                                         # measured on Roll_Cage (section 2.3 x 2.5)
PXH = float(os.environ.get("PXH", -78.0)); HOOP_Y = 15.5; HOOP_TOP = DECK_Z1 + 13.7             # hoop behind the pack, now stands above the pack top (27.5)
CRUN_Z = DECK_Z1 + 13.3                               # upper side rails / strap bars (bar bottom 1 mm above the pack top)
BAR_X, BAR_Z, BAR_Y, BAR_E_X = PXH, DECK_Z1 + 4.2, 19.1, -46.75 - SH; LINK_Y = 19.75      # lever wire runs at |Y| 19.1 (1.6 mm off the shell); a 0.65 mm jog at its front end puts the drop link back at 19.75 (clear of the cage plate and the arm cheek)
TAB_X, TAB_TOP_Z, LINK_INS = -46.75 - SH, 4.9, 0.8; WIRE_R, LINK_R, TUN_R = 0.4, 0.45, 0.9
PXP = -65.0; FOOT_SCREW_D = 0.6                       # side post X
XA, YF = PXH + 6.0, 15.7                              # hoop joint plate centre (X) and frame line (Y)
XK = -58.5                                            # end of the horizontal upper rail, drop diagonal starts
XN, YN, ZN = -56.0, 16.2, 21.3                        # frame end over the main-cage lug (horizontal lap, vertical screws)
JT_Z0, JT_Z1 = 12.5, 22.5           # hoop joint plate Z
HJ_SCREWS = [(XA - 1.2, 15.5), (XA + 1.2, 19.5)]; NJ_SCREWS = [(-52.4, 16.4), (-49.8, 16.4), (-47.2, 16.4)]      # 3 x M1.6 per side
RAIL_Z = DECK_Z1 + 1.25; CRAIL_Z = 24.4; RAIL_X0 = -75.5       # RAIL_Z: deck rails; CRAIL_Z: battery cradle rails (mid pack height, above the main-cage rail tops at Z 20.7) with front hooks down to the deck
BUMP_POST_FOOT = float(os.environ.get("BUMP_FOOT", 6.7)); BUMP_POST_X, BUMP_POST_Y, BUMP_POST_Z = -71.0, 11.8, BUMP_POST_FOOT + 1.15      # bump stops hang from the cradle rung over the axle (the deck that carried the old pads is gone)
STRAP_X = (-69.0, -61.5); STRAP_SX = 1.8; STRAP_BARS = False     # open top deck: no cage tubes across the pack; the pack is strapped through deck slots              # strap bar X positions; lap screws at X +-STRAP_SX
def tube(p0, p1, r=TUBE_R):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    ms = [trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1]), sections=20)]
    for q in (p0, p1): ms.append(trimesh.creation.icosphere(subdivisions=2, radius=r*1.03)); ms[-1].apply_translation(q)
    return U(ms)
def ysp(sg, a_, b_): return (min(sg*a_, sg*b_), max(sg*a_, sg*b_))
# ---- Rev 32/40: the cage = ONE printed part, single chamfered square tube size (CG_S), mitred/lapped corner joints, straight continuous tubes, one flat foot-plate style,
# 3 x M1.6 per side into tub lugs. Roof rails sit above the ESC shelves / motor / gearbox; hoop + tail frame cantilever behind the tail lugs over the rear links.
BALL_R = 1.45; BAR_BALL_R = 1.25; HZ0 = 12.0
CG_S, CG_CH = 2.4, 0.5                                # Rev 40: chamfered SQUARE tube, side 2.4, 0.5 chamfers (solid when printed); 1.2 end extension fills the 90 deg corner joints (no ball nodes)
def ctube(p0, p1, r=None, b0=None, b1=None, e0=None, e1=None, s=None):
    p0, p1 = np.array(p0, float), np.array(p1, float); a = (p1 - p0)/np.linalg.norm(p1 - p0)
    ref = np.array([0., 0., 1.]) if abs(a[2]) < 0.9 else np.array([1., 0., 0.])
    u = ref - a*np.dot(ref, a); u /= np.linalg.norm(u); v = np.cross(a, u); h, c = (CG_S if s is None else s)/2, CG_CH
    ring = [(sx*h, sy*(h - c)) for sx in (1, -1) for sy in (1, -1)] + [(sx*(h - c), sy*h) for sx in (1, -1) for sy in (1, -1)]
    e0 = CG_S/2 if e0 is None else e0; e1 = CG_S/2 if e1 is None else e1
    pts = [q + o[0]*u + o[1]*v for q, o in ((p0 - a*e0, o_) for o_ in ring)] + [q + o[0]*u + o[1]*v for q, o in ((p1 + a*e1, o_) for o_ in ring)]
    return trimesh.convex.convex_hull(np.array(pts))
def ball(c, r): return box(c[0] - r, c[0] + r, c[1] - r, c[1] + r, c[2] - r, c[2] + r)      # square bearing boss on the hoop leg (sway-bar wire)
BUMP_X = -71.0
def zlow(x): return float(np.interp(x, [PXH, CG_XT], [HZ0, CG_ZTF]))      # tail lower tube height along X (hoop foot -> tail foot)
RUNG2_X, RUNG2_Z = -45.0, 16.9
YF = CG_YT; RUNG_Z = 16.6                                  # pack-support rungs: >= 1.5 mm above the axle top bracket at full bump, 0.35 mm under the pack (Z 18.1)
parts = [ctube((PXH, -YF, CG_ZR), (PXH, YF, CG_ZR)), ctube((PXH, -YF, HZ0), (PXH, YF, HZ0)), ctube((BUMP_X, -YF, RUNG_Z), (BUMP_X, YF, RUNG_Z)), ctube((RUNG2_X, -CG_YS, RUNG2_Z), (RUNG2_X, CG_YS, RUNG2_Z)),    # hoop top / bottom bar, bump-post rung
         ctube((PXH, -YF, CG_ZR), (-34.0, CG_YS, CG_ZR)), ctube((PXH, YF, CG_ZR), (-34.0, -CG_YS, CG_ZR))]      # Yeti-style X brace in the roof plane between the tail rails (crosses at Y 0)                                                                                             # roof cross tube over the gearbox
for sg in (1, -1):
    Yt, Ym = sg*CG_YT, sg*CG_YM
    parts += [ctube((PXH, Yt, HZ0), (PXH, Yt, CG_ZR)),                                  # hoop leg
              ctube((PXH, Yt, CG_ZR), (-60.0, Yt, CG_ZR)), ctube((-60.0, Yt, CG_ZR), (-55.0, sg*CG_YS, CG_ZR)), ctube((-55.0, sg*CG_YS, CG_ZR), (-34.0, sg*CG_YS, CG_ZR)),     # tail roof rail, jogged inboard beside the shock spring
              ctube((PXH, Yt, HZ0), (-60.0, Yt, zlow(-60.0))), ctube((-60.0, Yt, zlow(-60.0)), (-55.0, sg*CG_YS, zlow(-55.0))), ctube((-55.0, sg*CG_YS, zlow(-55.0)), (-35.4, sg*CG_YS, CG_ZTF)),   # tail lower tube, jogged inboard, to the tail post
              ctube((-35.4, sg*CG_YS, CG_ZTF), (-35.4, sg*CG_YS, CG_ZR)), ctube((-35.4, sg*CG_YS, CG_ZTF), (-35.4, Yt + sg*0.4, CG_ZTF)),   # tail post (inboard) + foot link out to the lug screw
              box(SHK_UX - 2.5, SHK_UX + 2.5, *ysp(sg, CG_YS, SHK_YU - SHK_EYE_T/2 - 0.8), SHK_UZ - 2.1, SHK_UZ + 2.1),   # shock hanger tab out from the inboard roof rail / post (pin pilot below)
              ctube((-34.0, sg*CG_YS, CG_ZR), (CG_XM0, Ym, CG_ZR)),                           # jog out to the main rail
              ctube((CG_XM0, Ym, CG_ZR), (CG_XC, Ym, CG_ZR)),                           # main roof rail (ends past the motor, joins the cross tube)
              ctube((CG_XM0, Ym, CG_ZF), (CG_XM0, Ym, CG_ZR)), ctube((CG_XM1, Ym, CG_ZF), (CG_XM1, Ym, CG_ZR)),     # posts
              ctube((CG_XM0, Ym, CG_ZF), (CG_XM1, Ym, CG_ZR)),                          # diagonal
              ctube((BUMP_X, Yt, RUNG_Z), (BUMP_X, Yt, zlow(BUMP_X))),                  # rung end drops to the tail lower tube
              ctube((BUMP_X, sg*BUMP_POST_Y, BUMP_POST_Z), (BUMP_X, sg*BUMP_POST_Y, RUNG_Z), e0=0.7),   # bump-stop post hanging from the rung (rounded foot)
              ctube((PXH, sg*(CG_YT - 1.4), BAR_Z), (PXH, sg*(CG_YT + 1.4), BAR_Z), s=3.6, e0=0.0, e1=0.0),      # Rev 41: sway-bar bearing saddle on the hoop leg: 3.6 square chamfered block, 2.8 long, D1.4 bore along Y                                       # bearing ball on the hoop leg for the sway-bar wire
              box(CG_XM0 - 1.6, CG_XM0 + 4.9, *ysp(sg, 20.0, 23.0), 8.02, 9.3), box(CG_XM1 - 1.6, CG_XM1 + 4.9, *ysp(sg, 20.0, 23.0), 8.02, 9.3),     # foot plates on the rim bosses
              box(-37.4, -33.8, *ysp(sg, 14.6, 17.8), 16.32, 17.8)]                     # tail foot plate on the lug block
CAGE_NEW = D(U(parts), [cylY(PXH, BAR_Z, 0.7, sg*(YF - 3.0), sg*(YF + 3.0), 20) for sg in (1, -1)]
             + [cylY(SHK_UX, SHK_UZ, PIV_PILOT_D/2, sg*(CG_YS + 0.4), sg*(SHK_YU - SHK_EYE_T/2 - 0.7), 14) for sg in (1, -1)]
             + [cylZ(x_, sg*y_, 1.45, zs_ + 0.001, zs_ + 1.3, 18) for sg in (1, -1) for (x_, y_, zs_, zt_) in CG_SCREWS.values()] + [cylZ(x_, sg*y_, 0.8, zs_ - 1.5, zs_ + 1.2, 14) for sg in (1, -1) for (x_, y_, zs_, zt_) in CG_SCREWS.values()])
CAGE_NEW = D(CAGE_NEW, [box(-120, 20, -40, 40, CG_ZR + CG_S/2, CG_ZR + 5.0)])      # Rev 40: flat roof plane (diagonal / post end extensions trimmed to the rail top)
CG_ZA = 35.0      # Rev 42: roof cross tube arches over the raised gearbox (top Z 28.9): stubs from the rail ends up to Z 35.0 (0.8 below... shell roof there is >= 43)
CAGE_NEW = U([CAGE_NEW, ctube((CG_XC, -CG_YM, CG_ZA), (CG_XC, CG_YM, CG_ZA))] + [ctube((CG_XC, sg*CG_YM, CG_ZR), (CG_XC, sg*CG_YM, CG_ZA)) for sg in (1, -1)])
CAGE_NEW = D(CAGE_NEW, [TUB])      # seat trim: nothing of the cage extends into the tub
CAGE_NEW = D(CAGE_NEW, SPRING_KEEP)      # Rev 39: 1.0 mm relief around the shock spring (trims the tail foot plate corner)
P["Cage"] = CAGE_NEW
# ---------------- Rev 42: Cabin_Cage (rear hoop, A-pillars, hood rails, front hoop; bolts to rim bosses / front wall tops) ----------------
_cab = [ctube((CAB_X1, -CAB_YC, CAB_ZR), (CAB_X1, CAB_YC, CAB_ZR)), ctube((CAB_XA, -CAB_YC, CAB_ZR), (CAB_XA, CAB_YC, CAB_ZR)),      # rear hoop + A-pillar-top cross tube (roof)
        ctube((CAB_X2, -CAB_YC, CAB_ZH), (CAB_X2, CAB_YC, CAB_ZH)), ctube((CAB_XF, -CAB_YF, CAB_ZH), (CAB_XF, CAB_YF, CAB_ZH)),                  # cowl cross tube, front hoop top
        ctube((CAB_X2, -CAB_YC, CAB_ZH), (CAB_XF, CAB_YF, CAB_ZH)), ctube((CAB_X2, CAB_YC, CAB_ZH), (CAB_XF, -CAB_YF, CAB_ZH))]             # X brace in the hood plane (crosses at Y 0)
for sg in (1, -1):
    Yc, Yf = sg*CAB_YC, sg*CAB_YF
    _cab += [ctube((CAB_X1, Yc, CG_ZF), (CAB_X1, Yc, CAB_ZR)), ctube((CAB_X1, Yc, CAB_ZR), (CAB_XA, Yc, CAB_ZR)),         # rear post, roof rail
             ctube((CAB_XA, Yc, CAB_ZR), (CAB_X2, Yc, CAB_ZH)),                                                          # A-pillar (follows the windshield frame)
             ctube((CAB_X2, Yc, CG_ZF), (CAB_X2, Yc, CAB_ZH)),                                                           # cowl post
             ctube((CAB_X2, Yc, CAB_ZH), (CAB_XF, Yf, CAB_ZH)),   # hood rail, diagonal inboard (Rev 43: no straight run next to the fender panel)
             ctube((CAB_XF, Yf, CG_ZF), (CAB_XF, Yf, CAB_ZH)),                                                           # front hoop leg
             box(CAB_X1 - 1.6, CAB_X1 + 4.9, *ysp(sg, 19.4, 23.0), 8.02, 9.3), box(CAB_X2 - 1.6, CAB_X2 + 4.9, *ysp(sg, 19.4, 23.0), 8.02, 9.3),     # foot plates on the rim bosses
             box(CAB_XF - 2.6, CAB_XF + 4.6, *ysp(sg, 15.4, 18.4), 8.02, 9.3)]                                           # front foot plate on the wall top
CABIN = U(_cab)
CABIN = D(CABIN, [box(-120, 200, -40, 40, 0, 8.0)])
CABIN = D(CABIN, [cylZ(x_, sg*y_, 1.45, zs_ + 0.001, zs_ + 1.3, 18) for sg in (1, -1) for k_, (x_, y_, zs_, zt_) in CAB_SCREWS.items()] + [cylZ(x_, sg*y_, 0.8, zs_ - 1.5, zs_ + 1.2, 14) for sg in (1, -1) for k_, (x_, y_, zs_, zt_) in CAB_SCREWS.items()])
CABIN = D(CABIN, [TUB])
P["Cabin_Cage"] = single("Cabin_Cage", CABIN)
CAGE_PILOTS = [cylZ(x_, sg*y_, 0.6, zt_ - 5.0, zt_ + 0.02, 12) for sg in (1, -1) for (x_, y_, zs_, zt_) in ALL_SCREWS.values()]
CAGE_T = P["Cage"]     # keep-out only, not exported
for sg, nm in ((1, "L"), (-1, "R")):
    _top = _spt(SHK_TOP_S, sg); _yi, _yo = SHK_YU - SHK_EYE_T/2, SHK_YU + SHK_EYE_T/2
    _eye = D(trimesh.convex.convex_hull(np.vstack([cylY(SHK_UX, SHK_UZ, SHK_EYE_R, *ysp(sg, _yi, _yo), 28).vertices, cylY(_top[0], _top[2], SHK_ROD_R + 0.4, *ysp(sg, _yi, _yo), 16).vertices])), [cylY(SHK_UX, SHK_UZ, SHK_BORE_R, *ysp(sg, SHK_YU - 2.5, SHK_YU + 2.5), 14)])
    P[f"Ref_Shock_Eye_{nm}"] = single(f"Ref_Shock_Eye_{nm}", _eye)      # eye wraps the shock top: zero overlap (clean seat)    # rod end: stem from the shock top to the pin eye
    for k_, (x_, y_, zs_, zt_) in ALL_SCREWS.items(): P[f"HW_Cage_Screw_{nm}{k_}"] = U([cylZ(x_, sg*y_, 1.3, zs_, zs_ + 0.9, 16), cylZ(x_, sg*y_, 0.7, zt_ - 5.0, zs_ + 0.1, 12)])      # M1.6 x 6, head seated on the foot plate
for k_, (bx, by) in enumerate(BFS, 1): P[f"HW_Bulkhead_Floor_Screw_{k_}"] = U([cylZ(bx, by, 0.7, NOSE_BOT + 0.2, BFS_HEAD_Z + 0.1, 12), cylZ(bx, by, 1.3, BFS_HEAD_Z, BFS_HEAD_Z + 0.8, 16)])   # M1.6 x ~8, head on the bulkhead floor-web seat
LR = BAR_E_X - BAR_X                                   # lever length
CROSS = U([tube((BAR_X, -BAR_Y, BAR_Z), (BAR_X, BAR_Y, BAR_Z), WIRE_R)])
def lever(sg): return U([tube((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R), tube((BAR_E_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*LINK_Y, BAR_Z), WIRE_R)])
L0 = BAR_Z - (TAB_TOP_Z - LINK_INS)
def bar_state(ta):
    a = math.radians(ta); T = R.rot(np.array([TAB_X, TAB_TOP_Z - LINK_INS]), a, PL)
    E = lambda th: np.array([BAR_X + LR*math.cos(th), BAR_Z - LR*math.sin(th)])
    f = lambda th: np.linalg.norm(E(th) - T) - L0
    xs = np.linspace(-1.2, 1.2, 241); fv = [f(x) for x in xs]; best = 0.0; bd = 9
    for i in range(len(xs) - 1):
        if fv[i]*fv[i + 1] <= 0:
            r = brentq(f, xs[i], xs[i + 1])
            if abs(r) < bd: best, bd = r, abs(r)
    return best, E(best), T
def drop_link(Epos, T, sg):
    return tube((Epos[0], sg*LINK_Y, Epos[1]), (T[0], sg*LINK_Y, T[1]), LINK_R)
def wcyl(p0, p1): return trimesh.creation.cylinder(radius=WIRE_R, segment=np.array([p0, p1], float), sections=20)
P["Ref_Rear_Sway_Bar"] = U([wcyl((BAR_X, -BAR_Y - 0.4, BAR_Z), (BAR_X, BAR_Y + 0.4, BAR_Z))] + [wcyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z)) for sg in (1, -1)] + [wcyl((BAR_E_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*LINK_Y, BAR_Z)) for sg in (1, -1)])
th0, E0, T0 = bar_state(0)
for n, sg in (("L", 1), ("R", -1)): P[f"Ref_Sway_Drop_Link_{n}"] = drop_link(E0, T0, sg)
BAR_TILT = []
for ta in (TA_D, -2, 0, 3, 6, 7):
    th, E_, T_ = bar_state(ta); BAR_TILT.append((ta, math.degrees(th), math.degrees(math.atan2(E_[0] - T_[0], E_[1] - T_[1])), float(np.linalg.norm(E_ - T_))))
# servo envelope (front session): case on its side, ears, pilot wall
P["Ref_Servo_Envelope"] = U([box(13.4, 33.5, 1.8, 11.7, FLOOR_TOP, 10.8 + DELTA), box(32.3, 33.5, 11.7, 14.65, -3.8 + DELTA, 9.4 + DELTA)])
# ---------------- cage / shell relief against the new bodies ----------------
# ---- printed retainers + their screws ----
# shelves (one per side, identical): plate on four pillars, fence for an upright receiver, wire slots + finger notch
def csk(x, y, rt, rb, zt, d):      # countersink cone: radius rt at the top face zt, rb at depth d
    return trimesh.convex.convex_hull(np.vstack([cylZ(x, y, rt, zt - 0.001, zt, 24).vertices, cylZ(x, y, rb, zt - d, zt - d + 0.001, 24).vertices]))
CSK_RT, CSK_RB, CSK_D = 1.3, 0.7, 0.6                 # Rev 40: flat-head M1.4-class screws, head flush with the shelf top
for nm, sg in (("L", 1), ("R", -1)):
    pl = box(EL_X0, EL_X1, *ysp(sg, BAY_Y0 + 0.3, BAY_Y1), SH_Z0, SH_Z1)
    fx0, fx1 = EL_CX - RX_W/2 - 0.4, EL_CX + RX_W/2 + 0.4; fy0, fy1 = BAY_Y0 + 0.2, BAY_Y0 + 0.6 + RX_H + 0.4
    ring = D(box(fx0 - 1.2, fx1 + 1.2, *ysp(sg, fy0 - 1.2, fy1 + 1.2), SH_Z1 - 0.01, SH_Z1 + FENCE_H), [box(fx0, fx1, *ysp(sg, fy0, fy1), SH_Z1 - 0.02, SH_Z1 + FENCE_H + 0.1)])
    m = pl      # no receiver fence any more (receiver lives above the centreline, see RX_BOX)
    cs = [cylZ(px_, sg*py_, 0.8, SH_Z0 - 1, SH_Z1 + 1, 12) for sg_, px_, py_ in EL_PIL if sg_ == sg]
    cs += [box(EL_X0 + 0.6, EL_X0 + 3.4, *ysp(sg, 13.9, 16.9), SH_Z0 - 1, SH_Z1 + 1)]     # wire slot (between the rear pillars)
    cs += [box(EL_CX - 4.0, EL_CX + 4.0, *ysp(sg, 18.0, BAY_Y1 + 0.5), SH_Z0 - 1, SH_Z1 + 1)]    # finger notch at the outer edge
    cs += [box(EL_X1 - 3.4, EL_X1 - 0.6, *ysp(sg, 13.9, 16.9), SH_Z0 - 1, SH_Z1 + 1)]             # Rev 40: receiver antenna / wire exit slot at the front end (mirror of the rear wire slot)
    cs += [csk(px_, sg*py_, CSK_RT + 0.1, CSK_RB + 0.1, SH_Z1 + 0.002, CSK_D + 0.05) for sg_, px_, py_ in EL_PIL if sg_ == sg]      # countersinks
    P[f"ESC_RX_Shelf_{nm}"] = D(m, cs)
for sg, px_, py_ in EL_PIL:
    nm = "L" if sg == 1 else "R"; k_ = [q for q in EL_PIL if q[0] == sg].index((sg, px_, py_)) + 1
    P[f"HW_Shelf_Screw_{nm}{k_}"] = U([cylZ(px_, sg*py_, 0.7, SH_Z0 - 4.2, SH_Z1 - CSK_D + 0.1, 12), csk(px_, sg*py_, CSK_RT, CSK_RB, SH_Z1, CSK_D)])
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
SW_AX = U([posed(U([P["Rear_Axle_Top_Bracket"], P["HW_Rear_Axle_Top_Pin"]]), R.solve(t)[2]) for t in (-14, 0, 7)])
SW_BAR = []
for t in (-14, -10, 0, 7):
    th_, E_, T_ = bar_state(t)
    for sg in (1, -1): SW_BAR += [posed(lever(sg), R.roty(th_, BAR_X, BAR_Z)), drop_link(E_, T_, sg)]
SW_BAR = U(SW_BAR)
KEEPOUT = {"Axle_top_sweep": SW_AX, "Bar_sweep": SW_BAR, "Links_sweep": U([posed(P[f"Rear_Upper_Link_{n_}"], R.roty(R.solve(t)[1], PU[0], PU[1])) for t in (-14, 0, 7) for n_ in "LR"]), "Chassis_Tub": P["Chassis_Tub"], "Ref_Battery_2S": BATT_BOX, "Ref_ESC_G40": ESC_BOX, "Ref_Gearbox": P["Ref_Gearbox"], "Ref_Motor_GM1614": P["Ref_Motor_GM1614"], "ESC_RX_Shelf_L": P["ESC_RX_Shelf_L"], "ESC_RX_Shelf_R": P["ESC_RX_Shelf_R"],
           "Cage": CAGE_T}
def dil(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)):   # 18 directions (axes + edge diagonals): thin parts left diagonal gaps with only 6
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
BULK = ld("IFS_Bulkhead_Convertible"); _b = I(BULK, box(BULK_X0 - 1, 70.0, -40, 40, -20, 40)); _v0 = P["Chassis_Tub"].volume
SW_DROOP = U([posed(R.M["Ref_Rear_Axle"], R.solve(t)[2]) for t in range(-14, 1, 2)] + [posed(P[f"Rear_Trailing_Arm_{n_}"], R.roty(R.solve(t)[0], PL[0], PL[1])) for t in range(-14, 1, 2) for n_ in "LR"])   # droop envelope of axle + arms: tub tail is relieved around it
NOTCH_C = float(os.environ.get("NOTCH_C", 0.3))
_tub0 = P["Chassis_Tub"]
_old = I(_tub0, dil(SW_DROOP, 0.2))    # what the swept envelope removes (ragged); re-cut below as per-side convex wedges = a few clean planes
_wedges = []
for sg_ in (1, -1):
    _part = I(_old, box(-200, 200, 0, sg_*40, -50, 50) if sg_ > 0 else box(-200, 200, -40, 0, -50, 50))
    _v = _part.vertices.copy(); _v[:, 1] = sg_*np.maximum(np.abs(_v[:, 1]), TUB_OUT + 0.05); _w = trimesh.convex.convex_hull(np.vstack([_v, _v + [-30.0, 0, 0]])); _wedges.append(_w)     # extruded backwards: leaves the tail end face cleanly, no sheet slivers
    _t = I(_tub0, _w); print("wedge side", sg_, "removes %.1f mm3 (ragged cut removed %.1f)" % (_t.volume, _part.volume), "bounds", _t.bounds.round(1).tolist())
P["Chassis_Tub"] = single("Chassis_Tub(notch)", D(P["Chassis_Tub"], [dil(_b, 0.1)] + _wedges)); KEEPOUT["Chassis_Tub"] = P["Chassis_Tub"]
_rm = D(_tub0, [P["Chassis_Tub"]]); print("notch removed bounds", _rm.bounds.round(1).tolist() if len(_rm.faces) else None)
for n_, sg_ in (("L", 1), ("R", -1)):
    for k_, (bx, bz) in enumerate(BULK_SCREWS, 1):
        if sg_ == 1 and bx == 33.0: continue
        P[f"HW_Bulkhead_Screw_{n_}{k_}"] = U([cylY(bx, bz, BULK_HEAD_R, sg_*(TUB_OUT - BULK_CB_DEPTH - 0.05), sg_*(TUB_OUT - 0.2), 20), cylY(bx, bz, BULK_SCR_R, sg_*(BULK_WALL_Y0 + 0.35), sg_*(TUB_OUT - BULK_CB_DEPTH + 0.3), 14)])
print("tub notched for the bulkhead: %.1f mm3 removed (bulkhead mounting faces untouched)" % (_v0 - P["Chassis_Tub"].volume))
CUT_CLR = float(os.environ.get("CUT_CLR", 0.3)); DIL = {k: dil(m, CUT_CLR) for k, m in KEEPOUT.items()}
CLR_KEY = {"Ref_ESC_G40": 1.45, "ESC_RX_Shelf_L": 1.45, "ESC_RX_Shelf_R": 1.45}     # cage tubes keep >= 1.0 mm off the ESC and the ESC/RX shelves (was 0.2)
def relief(name, target, keys, vmin=40.0):
    use = [k for k in keys if vol(target, KEEPOUT[k]) > 0.05]
    print('  relief', name, 'cut by:', [(k, round(vol(target, KEEPOUT[k]), 2)) for k in use])
    for c in (CUT_CLR, CUT_CLR + 0.033, CUT_CLR + 0.071, CUT_CLR + 0.12):
        out = D(target, [dil(KEEPOUT[k], max(c, CLR_KEY.get(k, 0.0))) for k in use]) if use else target
        sh = sorted(out.split(only_watertight=False), key=lambda q: -abs(q.volume))
        if name != "Shell": break
        out.export(OUT + "_tmp.stl"); q_ = trimesh.load(OUT + "_tmp.stl")
        if len(sh) == 1 and q_.is_watertight and len(q_.split(only_watertight=False)) == 1: break
    keep = [q for q in sh if abs(q.volume) >= vmin]; res = U(keep) if len(keep) > 1 else sh[0]
    print("relief", name, "clr %.3f" % c, "removed %.0f mm3" % (target.volume - res.volume), "shells", len(sh), "dropped:", [(round(abs(q.volume), 1), q.bounds.round(0).tolist()) for q in sh[1:] if abs(q.volume) > 5]); return res
_cg = single("Cage", relief("Cage", P["Cage"], [k for k in KEEPOUT if k not in ("Cage", "Chassis_Tub")], 1e9))
P["Cage"] = _cg
P["Cage"].export(OUT + "_tmp.stl"); P["Cage"] = single("Cage", trimesh.load(OUT + "_tmp.stl"))
P["Chassis_Tub"] = D(P["Chassis_Tub"], CAGE_PILOTS)      # screw pilots in the tub
# shell: owned by scripts/19_shell_v4.py (no hood scoop, full relief); 18 only reads stl/v4/Tacoma_Trophy_Body_Shell.stl for the sweep

# ---------------- export + watertight roundtrip ----------------
rep = []
for n, m in sorted(P.items()):
    if n.startswith("Ref_") and n in ("Ref_Rear_Driveshaft",): pass
    m.export(OUT + n + ".stl"); q = trimesh.load(OUT + n + ".stl")
    sh = len(q.split(only_watertight=False)); rep.append((n, q.is_watertight, open_edges(q), sh, round(q.volume, 1)))
bad = [r for r in rep if not (r[1] and r[2] == 0 and r[3] == 1)]
print("exported", len(rep), "parts; not clean (watertight/open edges/shells):", bad)

# ---------------- sweep: moving rear bodies vs everything ----------------
R.PL = PL; R.PU = PU
FIX = {n: ld(n, OUT) for n in ("Chassis_Tub", "Cage", "Tacoma_Trophy_Body_Shell", "Ref_Servo_Envelope", 
                              "Ref_Gearbox", "Ref_Motor_GM1614", "Ref_ESC_G40", "Ref_Shock_RL", "Ref_Shock_RR")}
MOVE_ARM = {n: ld(n, OUT) for n in ("Rear_Trailing_Arm_L", "Rear_Trailing_Arm_R", "HW_Rear_Trailing_Pivot_Screw_L", "HW_Rear_Trailing_Pivot_Screw_R")}
MOVE_UL = {n: ld(n, OUT) for n in ("Rear_Upper_Link_L", "Rear_Upper_Link_R", "HW_Rear_Upper_Link_Pivot_Screw_L", "HW_Rear_Upper_Link_Pivot_Screw_R")}
AXG = {n: R.M[n] for n in ("Ref_Rear_Axle", "Ref_Wheel_RL", "Ref_Wheel_RR") if n in R.M}; AXG["Rear_Axle_Top_Bracket"] = ld("Rear_Axle_Top_Bracket", OUT); AXG["HW_Rear_Axle_Top_Pin"] = ld("HW_Rear_Axle_Top_Pin", OUT)
for n in ("HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R"): AXG[n] = ld(n, OUT)
FIX["IFS_Bulkhead_Convertible"] = ld("IFS_Bulkhead_Convertible")
FIX["Ref_Battery_2S"] = BATT_BOX      # keep-out volumes for the overlap/sweep checks only
for n_ in P:
    if n_.startswith(("HW_Battery_Hook", "HW_Bulkhead_Floor", "HW_Cage_Screw", "HW_Pocket", "Pocket_Clip")) and n_ not in FIX: FIX[n_] = ld(n_, OUT)
for n_ in ("IFS_Lower_Arm_L", "IFS_Lower_Arm_R", "IFS_Upper_Arm_L", "IFS_Upper_Arm_R", "Steering_Rack", "Servo_Horn", "Front_Carrier_L", "Front_Carrier_R", "Front_Spindle_L", "Front_Spindle_R"): FIX[n_] = ld(n_)
FIX["Ref_Front_Center_Driveshaft"] = trimesh.load(BD + "/Ref_Front_Center_Driveshaft.stl")
FIX["Ref_Rear_Driveshaft"] = ld("Ref_Rear_Driveshaft", OUT)
def scene(ta):
    a, u, T = R.solve(ta); sc = {}
    for n, m in FIX.items(): sc[n] = (m, np.eye(4))
    for n, m in MOVE_ARM.items(): sc[n] = (m, R.roty(a, PL[0], PL[1]))
    for n in ("Ref_Shock_RL", "Ref_Shock_RR"): sc[n] = (FIX[n], shock_T(a))      # Rev 38: shock follows the arm (lower eye on the clevis, top toward the upper pin; telescopes)
    for n, m in MOVE_UL.items(): sc[n] = (m, R.roty(u, PU[0], PU[1]))
    for n, m in AXG.items(): sc[n] = (m, T)
    th, E_, T_ = bar_state(ta); sc["Bar_cross"] = (CROSS, np.eye(4))
    for n, sg in (("L", 1), ("R", -1)):
        sc["Bar_lever_" + n] = (lever(sg), R.roty(th, BAR_X, BAR_Z)); sc["Bar_link_" + n] = (drop_link(E_, T_, sg), np.eye(4))
    return sc
def hits_vol(ta):
    sc = scene(ta); h = R.hits(sc); out = {}
    for p in h:
        a, b = p
        try:
            ma = sc[a][0].copy(); ma.apply_transform(sc[a][1]); mb = sc[b][0].copy(); mb.apply_transform(sc[b][1]); out[p] = vol(ma, mb)
        except Exception: out[p] = float('nan')
    return out
SW = {ta: hits_vol(ta) for ta in (TA_D, -2, 0, 7)}
print("--- rest overlaps (mm3, only > 0.01) ---")
for p, v in sorted(SW[0].items(), key=lambda kv: -kv[1]):
    if not v < 0.01: print("  ", p, round(v, 3))
print("--- gearbox / motor / ESC / receiver / battery boxes vs everything (rest, >0.01 mm3):", [(p, round(v, 2)) for p, v in SW[0].items() if any(k in p[0] + p[1] for k in ("Ref_Gearbox", "Ref_Motor", "Ref_ESC", "Ref_Receiver", "Ref_Battery")) and not v < 0.01] or "none")
print("--- pairs that appear in the sweep but not at rest ---")
for ta in (TA_D, -2, 7):
    for p, v in sorted(SW[ta].items()):
        if p not in SW[0] and not v < 0.01: print("  ta", ta, p, round(v, 3))
print("--- kinematics ---   ta | axle dX dZ | pitch | link ang | shock eye-eye | driveshaft L, angle")
for r_ in KT: print("  %4d | %6.2f %6.2f | %5.2f | %6.2f | %5.2f | %6.2f %5.2f" % r_)
print("--- sway bar: ta | lever deg | link tilt from vertical deg | link length (L0 %.2f)" % L0)
for r_ in BAR_TILT: print("  %4d | %6.2f | %6.2f | %5.2f" % r_)
print("arm len %.2f link len %.2f  IC (%.1f, %.1f)  anti-squat ~%.0f%%" % (LEN[0], LEN[1], IC[0], IC[1], AS))
open(OUT + "_report.txt", "w").write(repr({"rep": rep, "KT": KT, "IC": IC.tolist(), "AS": AS}))

# ---------------- dense rear sweep: min clearance of every mover to every fixed body, every 1 deg from full droop to full bump ----------------
from trimesh.collision import CollisionManager
def dense_sweep(t0=None, t1=7):
    t0 = TA_D if t0 is None else t0
    cm = CollisionManager()
    for n, (m, T) in scene(0).items():
        if n in MOVE_ARM or n in MOVE_UL or n in AXG or n.startswith(("Bar_lever", "Bar_link")) or n.startswith("HW_"): continue
        cm.add_object(n, m)
    worst = {}
    for ta in range(int(math.ceil(t0)), t1 + 1):
        for n, (m, T) in scene(ta).items():
            if n.startswith("HW_") or not (n in MOVE_ARM or n in MOVE_UL or n in AXG or n.startswith(("Bar_lever", "Bar_link"))): continue
            mm = m.copy(); mm.apply_transform(T)
            d, nm = cm.min_distance_single(mm, return_name=True)
            if n in ("Ref_Rear_Axle", "Bar_link_L", "Rear_Axle_Top_Bracket", "Rear_Upper_Link_L", "Rear_Upper_Link_R") and ta in (-14, -7, 0, 7):
                _d, _nm, _dat = cm.min_distance_single(mm, return_name=True, return_data=True)
                try: print("   diag", n, ta, round(_d, 3), _nm, [round(float(x), 2) for x in _dat.point(_nm)], [round(float(x), 2) for x in _dat.point("__mesh__")] if False else "")
                except Exception as e_: print("   diag", n, ta, round(_d, 3), _nm, "(no point:", e_, ")")
            if (n not in worst) or d < worst[n][0]: worst[n] = (d, nm, ta)
    print("--- DENSE rear sweep ta %d..%d every 1 deg: min clearance mover -> nearest fixed body (mm; <1 flagged) ---" % (t0, t1))
    for n, (d, nm, ta) in sorted(worst.items(), key=lambda kv: kv[1][0]):
        print("  %-30s %-28s %6.2f at ta %3d %s" % (n, nm, d, ta, "  <-- within 1 mm" if d < 1.0 else ""))
dense_sweep()
_cmc = CollisionManager()
_cmc.add_object("Cage", P["Cage"])
_shc = {_t: CollisionManager() for _t in range(-4, 8)}
for _t, _c in _shc.items():
    for _n in ("Ref_Shock_RL", "Ref_Shock_RR"): _x = FIX[_n].copy(); _x.apply_transform(shock_T(R.solve(_t)[0])); _c.add_object(_n, _x)
    for _n in ("Ref_Shock_Eye_L", "Ref_Shock_Eye_R"): _c.add_object(_n, P[_n])
_wc = {}
for _ta in range(-4, 8):
    for _n, (_m, _T) in scene(_ta).items():
        if _n.startswith("HW_") or not (_n in MOVE_ARM or _n in MOVE_UL or _n in AXG or _n.startswith(("Bar_lever", "Bar_link"))): continue
        _mm = _m.copy(); _mm.apply_transform(_T); _d, _nm = _cmc.min_distance_single(_mm, return_name=True)
        _d2, _nm2 = _shc[_ta].min_distance_single(_mm, return_name=True)      # moving shock (posed with the arm) vs this mover
        if _d2 < _d: _d, _nm = _d2, _nm2
        if _n not in _wc or _d < _wc[_n][0]: _wc[_n] = (_d, _nm, _ta)
print("--- rear cage / shock clearance of every mover over ta -14..+7 (nearest cage/shock body; mm) ---")
for _n, (_d, _nm, _ta) in sorted(_wc.items(), key=lambda kv: kv[1][0]): print("  %-28s %-22s %6.2f at ta %3d%s" % (_n, _nm, _d, _ta, "  <-- under 1.5" if _d < 1.5 else ""))
for _ta in (0, 3, 5, 6, 7):
    _m = posed(R.M["Ref_Rear_Axle"], R.solve(_ta)[2]); _o = CollisionManager(); _o.add_object("ax", _m)
    _po = CollisionManager()
    for _sg in (1, -1): _po.add_object("post%d" % _sg, tube((BUMP_POST_X, _sg*BUMP_POST_Y, BUMP_POST_Z), (BUMP_POST_X, _sg*BUMP_POST_Y, BUMP_POST_Z + 3.0)))
    print("bump stop: axle vs rear cage (all) at ta %d = %.2f mm; vs the bump post foot only = %.2f mm" % (_ta, _cmc.min_distance_other(_o), _po.min_distance_other(_o)))
