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
FLOOR_TOP = GBX_Z0; FLOOR_T = float(os.environ.get("FLOOR_T", 1.8)); FLOOR_BOT = FLOOR_TOP - FLOOR_T            # Rev 33: 1.8 mm floor, underside -0.9
TUB_OUT = 17.9                       # outer half width (arms start at 18.0; shell inner face is Y 24.6)
CHEEK_TIP_W = float(os.environ.get('CHEEK_TIP_W', 0.1))      # flat end of the front cheek (was a 0.1 mm feather the servo-side wall tab hid on one side)
TUB_MID = 20.7; TUB_WIDE = 23.0; WIDE_X0 = -29.8; WIDE_X1 = 22.0; SH1_X1 = WIDE_X1 + (TUB_WIDE - TUB_OUT); CHEEK_X1 = 31.1; SIDE_CH_Z = 2.8
WALL = 1.5; GBX_IN = 13.5            # gearbox/front zone inner half width; tail inner = TUB_OUT-WALL
TAIL_IN = TUB_OUT - WALL
RIM_Z = 8.0                          # cage lower tubes start at Z 8.1 -> they sit on this rim
TUB_X0, TUB_X1 = float(os.environ.get("TUB_X0", -40.6)), 36.0         # floor/walls end (floor ends before the axle housing, X>-47)
GBX_ZONE_X0 = -31.0                  # walls are 'gearbox zone' (inner 13.5) from here forward
# gearbox foot holes: GUESS (4 x M1.4 pilots, 20 x 20 mm square about the gearbox centre) -> measure and edit
FOOT_HOLES = [(-4.8, -9.4), (4.8, 9.4)]; FOOT_THRU_D = 1.7; FOOT_CB_D = 3.2; FOOT_CB_DEPTH = 1.0   # Modrax photo: 2 feet on the case bottom, diagonal (one per case half); M1.4 from below
# front bulkhead interface (now fused into Chassis_Tub)
BULK_X0 = 24.0
SERVO_BAY = dict(x0=12.0, x1=36.0, y0=2.0, y1=13.45, z0=FLOOR_TOP, z1=FLOOR_TOP + 16.0)   # EDITABLE keep-out/cavity (documented; tub is open here)
SHAFT_TUNNEL_D = 3.2                 # front shaft tunnel is in the bulkhead (unchanged, front session)
# electronics (ASSUMED sizes, edit freely)
ESC_L, ESC_W, ESC_H = 30.0, 17.0, 6.0        # Rev 37: user-measured ESC 30 (L) x 17 (D) x 6 (H). Stands on its edge in the side bay: L vertical (Z), D along X, H across Y (the bay is 19 x 9 mm, so L=X flat does not fit)
RX_L, RX_W, RX_H = 20.0, 15.0, 7.0           # Rev 37: user-measured RX 20 (L) x 15 (D) x 7 (H); in the same side pocket it stands L vertical, D along X, H across Y
BATT_L_SPEC, BATT_L, BATT_W, BATT_H = 45.3, 45.3, 21.0, 11.2   # 50 mm does NOT fit lengthwise anywhere (see TUB_V4.md); BATT_L is the longest that fits the deck
BFS = [(44.5, 3.5), (44.5, -3.5), (52.0, 3.5), (52.0, -3.5)]   # obsolete bulkhead floor screw holes (plugged when fusing IFS_Bulkhead_Convertible)
NOSE_TOP = -0.9; NOSE_BOT = NOSE_TOP - 1.8
DIFF_X0_TOP, DIFF_X0_BOT = 19.7, 21.5; DIFF_X1_BOT, DIFF_X1_TOP = 51.0, 56.5; DIFF_Y_TOP, DIFF_Y_BOT = 8.2, 6.4   # 45 deg chamfered diff drop (1.8 mm step + 5.5 mm front skid ramp)
TAIL_KICK_L, TAIL_KICK_H = 7.0, 0.9
STRAP_SLOT_X = (-73.0, -54.5)      # 4 slots: X centres, |Y| 11.7..13.9
BUMP_X0, BUMP_X1, BUMP_Z0 = -67.5, -61.0, 7.2       # bump pad: axle top Z 7.0 + 4.04 (ta +7) = 11.04, 0.36 gap
BAY_CLR = 0.4
# ESC / RX: two identical flat floor pockets (26.4 across Y x 18.4 along X, sunk 2 mm) under the shaft line, behind the transmission
# rear deck (battery + receiver) over the axle housing, bridge sits on it
X_FOOT, Y_FOOT = -43.0, 13.7
DECK_X1, DECK_X0 = -26.9, -35.0; DECK_Z0, DECK_Z1 = 14.7, 16.3   # raised so the axle-top bracket (top Z ~11 + 4 bump) clears
DECK_BOT_RIB = 0.0; BEAM_Y0 = 14.5; BEAM_Z0 = 11.2
BATT_X1 = -29.4; BATT_X0 = BATT_X1 - BATT_L - BAY_CLR; BATT_Y1 = (BATT_W + 0.8)/2; BATT_Y0 = -BATT_Y1     # 44.5 along X, 20 across Y, 10.5 tall, centred on the deck
# side bays (identical, left + right of the motor): ESC upright on the floor, shelf above it carries the receiver upright (either part fits either level)
EL_X1 = -(GBX_X + 0.4); EL_X0 = GBX_ZONE_X0 + 1.5; BAY_Y0, BAY_Y1 = 10.2, TUB_MID - WALL
SH_Z0 = FLOOR_TOP + ESC_L + 1.8; SH_T = 1.4; SH_Z1 = SH_Z0 + SH_T                 # Yeti deck lid Z (1.4 mm thick plate: 0.8 mm under 0.6 mm countersinks; 1.1 mm above Cage roof rail 31.6)
EL_PX = ((EL_X0, EL_X0 + 2.0), (EL_X1 - 2.0, EL_X1)); EL_PY = ((10.8, 13.2), (BAY_Y1 - 2.4, BAY_Y1))     # four shelf pillars per side (X end, Y inner/outer)
EL_CX = (EL_PX[0][1] + EL_PX[1][0])/2                                           # centre X of the bay between the pillars
FENCE_H = 4.5
# ================= rear 4-link on the tub tail =================
PL = np.array([float(os.environ.get("PL_X", -26.0 - SH)), float(os.environ.get("PL_Z", 2.4))])      # trailing arm pivot (X,Z)
PU = np.array([float(os.environ.get("PU_X", -24.53 - SH)), float(os.environ.get("PU_Z", 12.0))])       # upper link pivot
A_ = np.array([-55.0 - SH, -0.5]); BP = np.array([-57.5 - SH, 8.85])       # axle-side points: A_ = the M2 bolt through the arm cheeks and the axle ear (the real arm/axle joint; was the axle-tube centre X -59-SH, which let the bolt hole drift 1.0 mm from the ear hole at droop)
UL_EYE_GAP = 0.30      # gap between the two stacked upper-link eyes / blades at the axle-top pin (was 0.10)
LUG_X = 3.2; LUG_R = 3.4; PAD_Y0, PAD_Y1 = 14.4, TAIL_IN          # pad inside the tail wall, in which the pivots are tapped
PIV_PILOT_D = 1.35; PIV_CLEAR_D = 1.6; M2_CLR_D, M2_PILOT_D, M2_HEAD_R, M2_HEAD_H = 2.2, 1.7, 1.9, 1.3      # M2 for printed-to-printed joints (cage / cabin feet, trailing-arm pivots): clearance 2.2, self-tap pilot 1.7 (playbook, untested)
#                            # Ø1.35 mm PETG-CF pilot (prevents Z-layer splitting)
ARM_Y0, ARM_Y1 = 18.0, 23.4          # trailing arm Y span (v3)
# Rev 40: 3-hole anti-squat adjustment (Yeti p56): holes stacked at AS_PITCH on the arc of constant link length about the axle bracket pin, TOP hole = default (current geometry; the deck above leaves no room for holes higher), two lower holes
AS_PITCH = 2.0; _LL = float(np.linalg.norm(BP - PU))
PU_HOLES = [np.array([BP[0] + math.sqrt(_LL**2 - (PU[1] + k_*AS_PITCH - BP[1])**2), PU[1] + k_*AS_PITCH]) for k_ in (-2, -1, 0)]
UL_YB = PAD_Y0 - 1.35; BR_EYE_T = 1.1                # upper-link boss centre Y (boss 2.7 thick against the pad)
PRIDE_ARM_CUT = PL[0] + LUG_R + 0.1
TA_MIN = float(os.environ.get("TA_MIN", -16.0)); TA_BUMP = int(os.environ.get("TA_BUMP", 8))      # rear travel limits (arm angle, deg): droop stop search floor (the DRV_MAX shaft limit may stop it earlier) and the bump stop pose; every sweep below uses them
TRAVEL = [TA_MIN, -10, -5, 0, 3, TA_BUMP]
R.PL = PL; R.PU = PU; R.A = A_; R.Bp = BP

REAR_DY = float(os.environ.get('REAR_DY', 1.05))      # user decision: the rear trailing arms, axle ends, wheels and their hardware sit this far further out (rear track +2 x REAR_DY); arm inner face Y 18.0 -> 19.05 = 1.15 mm off the tub side (17.9; measured >= 1.01 on the built meshes)
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
CRK_PAD = (32.55, -8.15, 1.6, -1.9)      # X, Y, radius, bottom Z: 1.0 mm pad under the steering-crank pivot pin bore (front script's bore broke through the 45 deg diff-drop chamfer: 0.05..0.3 floor)
VG_X0, VG_Y1, VG_ZB = 39.85, 8.55, -2.0; VG_X1 = DIFF_X1_BOT + (VG_ZB - NOSE_BOT)*(DIFF_X1_TOP - DIFF_X1_BOT)/(NOSE_TOP - NOSE_BOT)      # groove fill: start X, outer |Y| (0.05 into the skirt), bottom Z; X where the front skid ramp reaches that depth
def build_tub():
    def hull_pts(pts): return trimesh.convex.convex_hull(np.array(pts, float))
    parts = [box(TUB_X0, BULK_X0 + 0.55, -TUB_OUT, TUB_OUT, FLOOR_BOT, FLOOR_TOP)]
    for s in (1, -1):
        def yy(a, b): return (min(s*a, s*b), max(s*a, s*b))
        # tail walls (thin), raised to the deck behind the motor
        y0, y1 = yy(TAIL_IN, TUB_OUT)
        parts.append(box(TUB_X0, WIDE_X0 + 0.1, y0, y1, FLOOR_TOP - 0.01, RIM_Z))
        parts.append(box(TUB_X0, DECK_X1, y0, y1, RIM_Z - 0.01, DECK_Z1))
        # pivot pads inside the tail wall (trailing lug + upper-link lug)
        y0, y1 = yy(PAD_Y0, TAIL_IN + 0.01)
        parts.append(box(PL[0] - LUG_X - 1.0, PU[0] + LUG_X + 0.5, y0, y1, FLOOR_BOT, PU[1] + 2.6))   # corner link block: bottom flush with the tub floor underside (owner markup: it hung 0.9 mm below); lower pivot low/outer, upper pivot above it
        y0, y1 = yy(PAD_Y0, TUB_OUT); parts.append(box(PL[0] - LUG_X - 1.0, PL[0] + LUG_X + 1.0, y0, y1, FLOOR_BOT, FLOOR_TOP + 0.5))      # Rev 42: boss drops through the tail wall to the (fixed) trailing-arm pivot: wall + pad reach down behind the arm boss
        # Yeti AX31104/AX31165 raised sculpted bracket detailing framing the 3-hole anti-squat column (outer face flush pocket for screw head)
        _as_x0, _as_x1 = PU[0] - 4.0, PU[0] + 3.6
        _as_s0, _as_s1 = PU[0] - 2.0, PU[0] + 2.0
        _as_z0, _as_z1 = 6.5, PU[1] + 2.5
        parts.append(hull_pts([(x, s*y, z) for x in (_as_x0, _as_s0) for (y, z) in ((TUB_OUT - 0.05, _as_z0), (TUB_OUT - 0.05, _as_z1), (TUB_OUT + 0.7, _as_z0 + 0.6), (TUB_OUT + 0.7, _as_z1 - 0.6))]))
        parts.append(hull_pts([(x, s*y, z) for x in (_as_s1, _as_x1) for (y, z) in ((TUB_OUT - 0.05, _as_z0), (TUB_OUT - 0.05, _as_z1), (TUB_OUT + 0.7, _as_z0 + 0.6), (TUB_OUT + 0.7, _as_z1 - 0.6))]))
        parts.append(hull_pts([(x, s*y, z) for x in (_as_x0, _as_x1) for (y, z) in ((TUB_OUT - 0.05, _as_z1 - 0.9), (TUB_OUT - 0.05, _as_z1), (TUB_OUT + 0.7, _as_z1 - 0.5), (TUB_OUT + 0.7, _as_z1 - 0.8))]))
        parts.append(hull_pts([(x, s*y, z) for x in (_as_x0, _as_x1) for (y, z) in ((TUB_OUT - 0.05, _as_z0), (TUB_OUT - 0.05, _as_z0 + 0.9), (TUB_OUT + 0.7, _as_z0 + 0.2), (TUB_OUT + 0.7, _as_z0 + 0.5))]))
        # wide side wings (cyan |Y|=17.9 @ Z=-0.9 -> magenta |Y|=23.0 @ Z=2.8..8.0) + 2-stage 45 deg front taper
        SKIRT_RAMP = float(os.environ.get("SKIRT_RAMP", 2.3))      # plan-view taper of the wide skirt's rear end (was a square step at X0); 2.3 = reaches full width 0.3 ahead of the B-post foot plate (X -27.0) that sits on it
        parts.append(hull_pts([(WIDE_X0, s*y, z) for (y, z) in ((TUB_OUT, FLOOR_BOT), (TUB_OUT, RIM_Z))] + [(WIDE_X0 + SKIRT_RAMP, s*y, z) for (y, z) in ((TUB_WIDE, RIM_Z), (TUB_WIDE, SIDE_CH_Z))] + [(WIDE_X1, s*y, z) for (y, z) in ((TUB_OUT, FLOOR_BOT), (TUB_OUT, RIM_Z), (TUB_WIDE, RIM_Z), (TUB_WIDE, SIDE_CH_Z))]))
        parts.append(hull_pts([(WIDE_X1, s*y, z) for (y, z) in ((TUB_OUT, FLOOR_BOT), (TUB_OUT, RIM_Z), (TUB_WIDE, RIM_Z), (TUB_WIDE, SIDE_CH_Z))] + [(SH1_X1, s*TUB_OUT, FLOOR_BOT), (SH1_X1, s*TUB_OUT, RIM_Z)]))
        parts.append(box(WIDE_X1 - 0.1, CHEEK_X1, *yy(12.9, TUB_OUT), FLOOR_BOT, RIM_Z))
        parts.append(hull_pts([(CHEEK_X1, s*y, z) for y in (12.9, TUB_OUT) for z in (FLOOR_BOT, RIM_Z)] + [(TUB_X1, s*y, z) for y in (12.9, 12.9 + CHEEK_TIP_W) for z in (FLOOR_BOT, RIM_Z)]))
    parts.append(box(DECK_X0, DECK_X1, -TUB_OUT, TUB_OUT, DECK_Z0, DECK_Z1))             # deck (ends at the link pads: the rear cage is cantilevered behind it)
    for s_ in (1, -1):
        parts.append(box(DECK_X0 + 0.5, -30.5, *ysp(s_, 4.5, 8.5), DECK_Z1 - 0.01, PACK_Z0))                       # pack ribs on the deck
    for s_ in (1, -1): parts.append(box(BUMP_X0, BUMP_X1, *ysp(s_, 10.0, 14.0), BUMP_Z0, DECK_Z0 + 0.01))      # bump stops under the deck: axle top touches at ta +7
    for s_ in (1, -1):
        if DECK_X0 < -50.0: parts.append(box(DECK_X0, -50.0, min(s_*BEAM_Y0, s_*TUB_OUT), max(s_*BEAM_Y0, s_*TUB_OUT), BEAM_Z0, DECK_Z0 + 0.01))       # side beam under the extended deck (cage feet screw into it)
    # 4-sided chamfered diff drop (top Z=-0.8 overlaps floor/web by 0.1 mm, bottom flat Z=-2.7); unioned AFTER cuts so line 148 never cuts an undercut groove
    diff_drop = hull_pts([[x, y, NOSE_TOP + 0.1] for x in (DIFF_X0_TOP, DIFF_X1_TOP) for y in (-DIFF_Y_TOP, DIFF_Y_TOP)]
                         + [[x, y, NOSE_BOT] for x in (DIFF_X0_BOT, DIFF_X1_BOT) for y in (-DIFF_Y_BOT, DIFF_Y_BOT)])
    m = U(parts)
    cuts = []
    # wide-zone interior bay (keeps solid 3.8 mm outer wall |Y|=19.2..23.0; rear wall at EL_X0=-29.5), front gbx cavity, and bulkhead bay (|Y|<12.9)
    for s_ in (1, -1):
        cuts.append(box(EL_X0 - 0.05, WIDE_X1 - 1.5, *ysp(s_, 0.0, BAY_Y1), FLOOR_TOP, RIM_Z + 1.0))
        cuts.append(box(WIDE_X1 - 1.6, BULK_X0, *ysp(s_, 0.0, GBX_IN), FLOOR_TOP, RIM_Z + 1.0))
    # bulkhead bay cut: keep floor extended to BULK_X0+0.5 below Z=0.50 (closes X=24 air gap + bulkhead rear bevel; 0.30 mm clear below Servo_Ref at Z>=0.80)
    cuts.append(box(BULK_X0 + 0.5, TUB_X1 + 0.1, -12.9, 12.9, FLOOR_BOT - 0.1, 0.50 + 0.01))
    cuts.append(box(BULK_X0, TUB_X1 + 0.1, -12.9, 12.9, 0.50, RIM_Z + 1.0))
    # rear driveshaft droop clearance U-slot in tub tail floor: X=-40.6..-24.0, |Y|<=5.0, 45 deg chamfered front corners + beveled top lip
    _ds_slot_pts = [(TUB_X0 - 0.5, -5.0), (TUB_X0 - 0.5, 5.0), (-26.0, 5.0), (-24.0, 3.0), (-24.0, -3.0), (-26.0, -5.0)]
    cuts.append(hull_pts([[x, y, z] for (x, y) in _ds_slot_pts for z in (FLOOR_BOT - 0.5, FLOOR_TOP + 0.5)]))
    cuts.append(hull_pts([[x, y, z] for (x, z) in ((-24.2, FLOOR_TOP + 0.5), (-23.1, FLOOR_TOP + 0.5), (-24.2, FLOOR_TOP - 0.9)) for y in (-3.2, 3.2)]))
    # tail kick-up: wedge under the floor end (keeps >= 1.7 mm floor)
    cuts.append(hull_pts([[x, y, z] for (x, z) in ((TUB_X0 - 0.1, FLOOR_BOT - 0.1), (TUB_X0 + TAIL_KICK_L, FLOOR_BOT - 0.1), (TUB_X0 - 0.1, FLOOR_BOT + TAIL_KICK_H)) for y in (-TUB_OUT - 1, TUB_OUT + 1)]))
    # gearbox foot pilots (blind, from the top)
    for fx, fy in FOOT_HOLES: cuts += [cylZ(fx, fy, FOOT_THRU_D/2, FLOOR_BOT - 0.01, FLOOR_TOP + 0.01, 24), cylZ(fx, fy, FOOT_CB_D/2, FLOOR_BOT - 0.01, FLOOR_BOT + FOOT_CB_DEPTH, 24)]
    for s in (1, -1):
        # trailing arm pivot pilot (from outside), upper link pivot pilot (from inside the pad)
        cuts.append(cylY(PL[0], PL[1], M2_PILOT_D/2, s*(PAD_Y0 - 0.01), s*(TUB_OUT + 0.01), 24))      # M2 pilot right through the 3.5 mm pad
        for _h in PU_HOLES: cuts.append(cylY(_h[0], _h[1], 0.75, s*(PAD_Y0 - 0.01), s*(TUB_OUT + 0.01), 24))      # three D1.5 clearance holes (default = top)
        # deck bays: wire slots through the deck
    for xs, ys in ((-30.0, -2.0), (-30.0, 2.0)): cuts.append(box(xs - 1.2, xs + 1.2, ys - 0.7, ys + 0.7, DECK_Z0 - 0.01, DECK_Z1 + 0.01))
    for s_ in (1, -1):
        for sx_ in STRAP_SLOT_X: cuts.append(box(sx_ - 4.0, sx_ + 4.0, *ysp(s_, 11.7, 13.9), DECK_Z0 - 0.1, DECK_Z1 + 0.1))      # 8 x 2.2 mm strap slots through the deck
    for s_ in (1, -1): cuts.append(box(EL_X0 - 0.01, DECK_X1 + 0.01, *ysp(s_, BAY_Y0, TUB_OUT + 0.01), RIM_Z - 0.02, DECK_Z1 + 0.01))    # the side bays reach under the deck front: notch the deck + raised tail wall there
    m = D(m, cuts)
    _cp = trimesh.convex.convex_hull(np.vstack([cylZ(CRK_PAD[0], CRK_PAD[1], CRK_PAD[2], CRK_PAD[3], CRK_PAD[3] + 0.01, 48).vertices, cylZ(CRK_PAD[0], CRK_PAD[1], CRK_PAD[2] + (FLOOR_BOT - CRK_PAD[3]), FLOOR_BOT + 0.01, FLOOR_BOT + 0.02, 48).vertices]))      # crank pad as a 45 deg frustum (same chamfer as the diff drop): no 1.0 mm vertical step under the floor
    # V-groove fill beside the front hinge-boss skirts (|Y| 8.5): the 45 deg diff-drop flank left a see-through groove X 39.9..57, |Y| 7.5..8.5, Z -2.0..-0.9
    _vg = []
    for s_ in (1, -1):
        _sec = lambda x_, zb_: [[x_, s_*VG_Y1, FLOOR_BOT + 0.05], [x_, s_*VG_Y1, zb_], [x_, s_*(DIFF_Y_TOP - (NOSE_TOP + 0.1 - zb_)), zb_], [x_, s_*(DIFF_Y_TOP - 0.05), FLOOR_BOT + 0.05]]
        _vg.append(trimesh.convex.convex_hull(np.array(_sec(VG_X0, VG_ZB) + _sec(VG_X1, VG_ZB) + [[DIFF_X1_TOP - 0.1, s_*VG_Y1, FLOOR_BOT + 0.05], [DIFF_X1_TOP - 0.1, s_*(DIFF_Y_TOP - 0.05), FLOOR_BOT + 0.05]], float)))
    m = U([m, diff_drop, _cp] + _vg + [ cylZ(CRK_PAD[0], CRK_PAD[1], 1.2, FLOOR_BOT - 0.5, FLOOR_BOT + 0.1, 24)])
    return m
SHK_UX, SHK_UZ = -40.2, 28.8                                          # rear shock upper pin (X,Z), M1.6 along Y (default Yeti position: 24.3 deg lean, L0 31.06)
SHK_UX2, SHK_UZ2 = -44.0, 29.2                                        # Yeti AX31102 twin-position shock mount plate: secondary upright tuning hole (17.4 deg lean)
# Rev 39: REAL shock envelope (Injora 36 mm big bore class: 36 mm eye-to-eye, 7..9 mm travel assumed): nylon bottom end with brass eye bushing, D8 body, D10 spring, D2 rod, 3.0 mm wide eyes
SHK_LO = np.array([-44.0 - SH, 0.5]); SHK_UP = np.array([SHK_UX, SHK_UZ]); SHK_L0 = float(np.linalg.norm(SHK_UP - SHK_LO)); SHK_U = (SHK_UP - SHK_LO)/SHK_L0
SHK_DY = float(os.environ.get('SHK_DY', 0.85))      # shock lower eye moved out with the thickened clevis cheeks (1.6 inner/outer, slot 3.3)
SHK_YL, SHK_YU, SHK_EYE_T, SHK_EYE_R = 20.4 + REAR_DY + SHK_DY, 18.2, 3.0, 2.2                # |Y| of the lower eye (arm clevis) / upper eye (tab on the tail post); eye width (assumed); eye boss R
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
SPRING_KEEP = [_posed_keep(_axcyl(_spt(SHK_SPR_S0 - 1.0, sg), _spt(SHK_SPR_S1 + 1.0, sg), SHK_SPR_R + 1.0), ta_) for sg in (1, -1) for ta_ in (TA_MIN, -10, -5, 0, 4, TA_BUMP + 0.5)]      # 1.0 mm keep-out swept by the spring over the travel (relief in the tub lug and the cage tail foot)
def _keep_cage(sg):
    _cs = [_posed_keep(_axcyl(_spt(SHK_SPR_S0 - 1.08, sg), _spt(SHK_SPR_S1 + 1.08, sg), SHK_SPR_R + 1.08), float(ta_)) for ta_ in range(int(TA_MIN), int(TA_BUMP) + 2)]
    return [trimesh.convex.convex_hull(np.vstack([a_.vertices, b_.vertices])) for a_, b_ in zip(_cs[:-1], _cs[1:])]      # hull of consecutive 1 deg poses: a smooth swept keep-out (separate cylinders left 0.06 mm ridges/flakes on the screw-3 pad top)
SPRING_KEEP_CAGE = _keep_cage(1) + _keep_cage(-1)
SHELL = ld("Tacoma_Trophy_Body_Shell")
# ---- Rev 32: ONE new cage (scrapped Roll_Cage + Rear_Cage). Constants shared with the tub (lugs / rim bosses are part of the tub)
CG_ZR, CG_YT, CG_YM = 30.4, 15.7, 21.5            # roof rail height; tail rail |Y| (inside the tail walls); main rail |Y| (outside the ESC shelves, 1.15 mm off the shelf pillars)
CG_XM0, CG_XM1, CG_XT, CG_XC = -25.5, -7.6, -38.2, -3.1      # Rev 49: CG_XM0 is the B-post (X -25.5, in the open bed just behind the roof edge); the post at CG_XM1 is gone
#        # main posts (on the rim columns) and tail post X
CG_ZF, CG_ZTF = 9.6, 17.8                         # main foot node Z (rim top 8.0 + plate 1.3), tail foot node Z (lug top 16.3 + plate 1.1)
CG_SCREWS = {1: (CG_XM0 + 3.4, 20.9, 9.3, 8.0), 3: (-36.0, 16.5, 17.8, 16.3)}      # k: (x, |y|, head seat Z, tub seat Z); 3 x M1.6 per side
# Rev 42: Cabin_Cage (separate printed part: rear hoop, A-pillars, hood rails, front hoop). Post X, tube |Y|, roof/hood heights, bolt-on feet
CAB_X1, CAB_X2, CAB_XF = 1.2, 17.0, 26.0                 # rear hoop post / cowl post / front hoop post (X)
CAB_YC, CAB_YF = 20.4, 17.0                               # tube |Y| roof + pillars / front hoop legs
CAB_ZR, CAB_XA, CAB_ZH = 35.1, 5.0, 26.8                  # roof level, A-pillar top X, hood-rail level
CAB_FOOT_Y1 = 22.7; CAB_SCREWS = {5: (CAB_X2 + 3.4, CAB_FOOT_Y1 - 1.85, 9.3, 8.0), 6: (CAB_XF + 3.0, CAB_YF - 1.25, 9.3, 8.0)}     # M1.6 into the rim bosses (4, 5) and the front wall top (6)
ALL_SCREWS = {**CG_SCREWS, **CAB_SCREWS}
FEET = []
for sg in (1, -1):
    FEET.append(box(TUB_X0, -33.6, *ysp(sg, PAD_Y0, TUB_OUT), 14.5, 16.3))                              # tail lug block on the pad
    FEET.append(trimesh.convex.convex_hull(np.array([(x_, sg*y_, z_) for x_ in (TUB_X0, -33.6) for (y_, z_) in ((TUB_OUT - 0.05, 13.85), (TUB_OUT - 0.05, 16.3), (18.55, 16.3), (18.55, 14.5))], float)))      # lug shelf: outer face 18.55 (45 deg underside) so the screw-3 pilot (Y 16.5) keeps a 1.2 mm wall outboard and inboard (the upper link eye limits the inboard side)
def single(name, m, vmin=1.0):
    sh = sorted(m.split(only_watertight=False), key=lambda q: -abs(q.volume))
    for q in sh[1:]:
        if abs(q.volume) > vmin: print("  dropped fragment of", name, "vol %.1f" % abs(q.volume), "bounds", q.bounds.round(1).tolist())
    return sh[0]
TUB = build_tub(); SERVO_CLR = 0.3; SERVO_DY = 0.15      # shallow planar pocket under servo bottom + ear clearance in front cheek before fusing bulkhead
SV_EAR_X = (32.15, 34.7); SV_CH = dict(x=(33.55, 41.0), y=(11.95, 14.55), z=((12.4, 15.0), (2.4, 5.0)))      # front script's numbers: ear recess + screw heads (ear wall now BEHIND the tabs at X 29.75..32.15); two D2.6 driver channels in front of the ears
TUB = D(TUB, [box(SV_EAR_X[0], SV_EAR_X[1], 11.3, 15.05 + SERVO_DY, -4.2 + DELTA, RIM_Z + 1.0)] + [box(SV_CH['x'][0], SV_CH['x'][1], SV_CH['y'][0], SV_CH['y'][1], z0_, z1_) for (z0_, z1_) in SV_CH['z']] + [ box(13.37 - SERVO_CLR, 33.5 + SERVO_CLR, 1.8 + SERVO_DY - SERVO_CLR, 11.7 + SERVO_DY + SERVO_CLR, FLOOR_TOP - 0.4 - SERVO_CLR + 0.3, FLOOR_TOP + 0.01)])
SV_WALL = (31.0, 34.5, 15.2, 16.4)      # left side wall carried forward past the servo ears and screw heads (the front ear plate moved behind the tabs): squares off the cheek's feathered taper tip, 1.2 mm thick, 0.4 off the ears
FOOT_LIP_LAND = 0.8
def corbel(x0, x1, y_face, y_out, z_top, sg):      # 45 deg support lip under a cage foot plate that reaches past the tub face (y_face -> y_out): 0.8 mm vertical land (no feather edge), 0.05 mm into the wall
    d_ = 1 if y_out > y_face else -1; yf = y_face - d_*0.05
    return trimesh.convex.convex_hull(np.array([[x_, sg*y_, z_] for x_ in (x0, x1) for (y_, z_) in ((yf, z_top), (y_out, z_top), (y_out, z_top - FOOT_LIP_LAND), (yf, z_top - FOOT_LIP_LAND - abs(y_out - yf)))], float))
FOOT_LIPS = []      # every cage / cabin foot plate sits fully on tub material (owner: no overhangs on the tub)
for sg_ in (1, -1):
    FOOT_LIPS += [corbel(x_ - 1.6, x_ + 5.4, TUB_WIDE, 23.35, RIM_Z, sg_) for x_ in (CG_XM0,)]                    # Cage main feet reach |Y| 23.3 (tub side 23.0)
    FOOT_LIPS += [corbel(x_ - 1.6, x_ + 5.4, TUB_MID - WALL, 18.9, RIM_Z, sg_) for x_ in (CAB_X2,)]               # Cabin_Cage feet 4, 5 reach in to |Y| 18.95 (wall inner face 19.2)
    FOOT_LIPS += [corbel(CAB_XF - 2.6, CHEEK_X1, TUB_OUT, 18.45, RIM_Z, sg_)]                                             # Cabin_Cage front foot reaches |Y| 18.4 (cheek outer face 17.9)
    FOOT_LIPS += [corbel(-37.9, -33.8, PAD_Y0, 14.15, 16.3, sg_)]                                                         # Cage tail foot reaches in to |Y| 14.2 (lug block 14.4)
PILOT_BOSS_R = M2_PILOT_D/2 + 1.2      # pilot wall >= 1.2 mm: round boss round every cage-screw pilot, clipped to the tub side (the shell sits 1.6 mm outside it)
PILOT_BOSSES = [trimesh.boolean.intersection([cylZ(x_, sg_*y_, PILOT_BOSS_R, zt_ - 5.0, zt_ + 0.02, 32), box(x_ - 4, x_ + 4, *ysp(sg_, 0.0, ymax_), zt_ - 5.0, zt_ + 0.02)], engine="manifold")
                for sg_ in (1, -1) for k_, (x_, y_, zs_, zt_) in ALL_SCREWS.items() if k_ in (1, 5) for ymax_ in [{1: 23.0, 3: TUB_OUT, 5: 22.7}[k_]]]
TUB = U([TUB] + FEET + FOOT_LIPS + PILOT_BOSSES + ([box(SV_WALL[0], SV_WALL[1], s_*SV_WALL[2] if s_ > 0 else -SV_WALL[3], s_*SV_WALL[3] if s_ > 0 else -SV_WALL[2], FLOOR_BOT, RIM_Z) for s_ in (1, -1)] if os.environ.get('SV_WALL_ON', '1') == '1' else []))      # the servo-side tab (image 58): removing it brings back a thin feather on the +Y cheek (front script's ear wall) -> now mirrored to -Y so the tub is symmetric here
RIB_MIN_L = 1.5; RIB_MIN_W = 0.6      # mean width (mm) a clipped rib must keep; full ribs are 0.9
RIB_TIP_W = 0.45
def _trim_rib_tips(q, a, t0, t1, h):
    ax = np.array([math.cos(a), math.sin(a)]); pe = np.array([-math.sin(a), math.cos(a)]); zc = FLOOR_TOP - 0.05 + (h + 0.05)/2
    def wid(tt):
        p_ = q.section(plane_origin=[tt*ax[0], tt*ax[1], zc], plane_normal=[ax[0], ax[1], 0.0])
        return 0.0 if p_ is None else float(np.ptp(p_.vertices[:, :2] @ pe))
    ts = t0
    while ts < t1 and wid(ts + 0.02) < RIB_TIP_W: ts += 0.05
    te = t1
    while te > ts and wid(te - 0.02) < RIB_TIP_W: te -= 0.05
    if te - ts < RIB_MIN_L: return None
    if ts <= t0 + 1e-6 and te >= t1 - 1e-6: return q
    c = (ts + te)/2; b = trimesh.creation.box(extents=[te - ts, 4.0, 4.0]); b.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1])); b.apply_translation([c*ax[0], c*ax[1], zc])
    r = I(q, b); return r if len(r.faces) else None
def hexribs(x0, x1, y0, y1, pitch=6.0, w=0.9, h=1.2):
    cx, cy = 0.0, 0.0; L = max(abs(x0), abs(x1)) + max(abs(y0), abs(y1)) + (x1 - x0) + (y1 - y0); ribs = []      # one lattice for every area (origin 0,0): the halves line up across the gaps, no per-area shift
    for ang in (0, 60, 120):
        a = math.radians(ang)
        for k in range(-int(L/pitch) - 1, int(L/pitch) + 2):
            b = trimesh.creation.box(extents=[2*L, w, h + 0.05]); b.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1]))
            b.apply_translation([cx - k*pitch*math.sin(a), cy + k*pitch*math.cos(a), FLOOR_TOP - 0.05 + (h + 0.05)/2])
            q = I(b, box(x0, x1, y0, y1, FLOOR_TOP - 0.05, FLOOR_TOP + h))      # each rib clipped on its own: a rib the area edge slices lengthwise (or a corner graze) is dropped, not left as a 0.25 mm strip
            if len(q.faces) == 0: continue
            t_ = q.vertices[:, :2] @ np.array([math.cos(a), math.sin(a)])
            if not (abs(q.volume)/((h + 0.05)*max(float(np.ptp(t_)), 1e-6)) >= RIB_MIN_W and float(np.ptp(t_)) >= RIB_MIN_L): continue
            q = _trim_rib_tips(q, a, float(t_.min()), float(t_.max()), h)      # blunt both ends: any end section narrower than RIB_TIP_W is cut back (the acute wedge tips an area edge leaves were 0.2-0.3 mm stubs)
            if q is not None: ribs.append(q)      # no rib piece shorter than RIB_MIN_L: those are the wedge tips an area edge cuts off a rib
    return U(ribs)
# Yeti AX31103 isogrid stiffening ribs across open inner tub floor zones (clear of gearbox, motor, ESC, servo, pillars, droop stops, and rear driveshaft slot)
RIB_AREAS = [
    (GBX_X + 0.8, BULK_X0 - 0.8, -GBX_IN + 0.3, -2.2),                # front-right (beside servo)
    (GBX_X + 0.8, 12.6, 2.2, GBX_IN - 0.3),                           # front-left (ahead of gearbox, behind servo)
    (-5.8, WIDE_X1 - 1.8, -BAY_Y1 + 0.3, -GBX_Y - 0.7),               # mid-right wide wing
    (-5.8, 12.6, GBX_Y + 0.7, BAY_Y1 - 0.3),                          # mid-left wide wing (behind servo)
    (-28.8, -GBX_X - 0.8, 5.6, BAY_Y0 - 0.3),                         # rear-center left (flanking driveshaft)
    (-28.8, -GBX_X - 0.8, -BAY_Y0 + 0.3, -5.6),                       # rear-center right (flanking driveshaft, clear of ESC)
    (-18.0, -GBX_X - 0.8, -5.8, 5.8),                                 # center behind gearbox (ahead of driveshaft droop slot)
    (-36.8, -29.8, 5.8, PAD_Y0 - 0.5),                                # tail floor left (flanking U-slot, ahead of droop stop)
    (-36.8, -29.8, -PAD_Y0 + 0.5, -5.8),                              # tail floor right (flanking U-slot, ahead of droop stop)
]
TUB = U([TUB] + [hexribs(*r_) for r_ in RIB_AREAS])
# ---- equipment seats in the wide zone: (motor clamp and saddle removed, Rev 36) ESC towers + clips, Rx clips, battery hold-down bosses ----
def ysp(sg, a_, b_): return (min(sg*a_, sg*b_), max(sg*a_, sg*b_))
RAIL_Y = 12.0; XF = -28.2; HOOK_Y = 6.5; HOOK_SX = -28.1; HOOK_SY = 9.2          # battery cradle rail line (|Y|) and its two deck feet
RAIL_BOSS = []      # no battery hooks any more (cradle removed)
BAY_BACK = [box(WIDE_X0 + 0.05, EL_X0 + 1.2, *ysp(sg_, TAIL_IN, BAY_Y1 + 0.05), FLOOR_TOP - 0.01, RIM_Z) for sg_ in (1, -1)]      # rear end wall of each side bay (X -29.75..-28.3, 1.45 thick): joins the tail wall end to the wing's rear skin where the shelf pillar used to (no 0.25 skin / slit / slot)
TUB = U([TUB] + BAY_BACK + RAIL_BOSS)
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
    for ta in [-16, -14, -12, -10, -8, -5, -2, 0, 2, 4, 6, 7, 8, 9]:
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
AXB_X, AXB_Z, AXB_PL_Y0, AXB_PL_Y1 = -55.0 - SH, -0.5, 21.1, 22.1      # trailing-arm axle bolt centre; outer cheek plate |Y| span (the ear sweep trims its inner face to 21.25 -> 0.85 thick)
def _axl():
    ax0 = dil_(I(R.M['Ref_Rear_Axle'], box(-70 - SH, -45 - SH, -40, 40, -10, 10)), 0.24)
    # Sweep axle ear relative to trailing arm over full droop..bump (-18..+18 deg) under both R.solve(ta) and rotation about true axle bolt (-64.0, -0.5)
    sw = []
    for ta_ in (-18, -12, -6, 0, 6, 12, 18):
        a_, _, Tax_ = R.solve(max(-18, min(9, ta_)))
        m1 = ax0.copy(); m1.apply_transform(R.roty(-a_, PL[0], PL[1]) @ Tax_); sw.append(m1)
        m2 = ax0.copy(); m2.apply_transform(R.roty(math.radians(ta_), -55.0 - SH, -0.5)); sw.append(m2)
    return U(sw)
def dil_(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)):   # 18 directions (axes + edge diagonals): thin parts left diagonal gaps with only 6
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
def simp(m, tol=0.01):      # collapse sliver/degenerate facets left by stacked booleans (manifold3d simplify; keeps the solid watertight, moves the surface < tol)
    import manifold3d as _m3
    s_ = _m3.Manifold(_m3.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32))).simplify(tol).to_mesh()
    return trimesh.Trimesh(np.asarray(s_.vert_properties)[:, :3], np.asarray(s_.tri_verts), process=False)
AXL_END = simp(_axl(), 0.01)      # the 14-pose x 19-copy sweep has 339k facets, 18k of them degenerate: they were the 3.4k sliver faces of each arm
AXB_SLOT_Y0, AXB_SLOT_Y1, AXB_SLOT_R = 19.5, 21.3, 1.9      # inside the ear slot the bolt runs free: removes the 0.2 mm sleeve the ear's own hole (r 1.44 - 0.24) left round the D2.0 bolt hole
CLV_RAIL = (-51.7, -46.7, 18.0, 18.8, 0.5, 1.5)             # inner top rail beside the shock eye: 0.7 x 1.0 (was a 0.3 mm strip)
CLV_SLIT = (-57.0, -56.2, 19.05, 20.45, 0.9, 1.9)           # plugs the 0.25 mm slit between the rear-window beam and the tab block
TAB_BACK = (-57.7, -56.7, 18.0, 21.5, 1.9, 4.8)             # sway-link clevis back wall 0.5 -> 1.1 (grows rearward, away from the link eye)
WIN_FILL = (-60.0, -58.0, 19.05, 20.45, -2.6, -1.1)         # plugs the 0.2..0.8 mm sliver left of the rear window between the ear slot wall and the strut
PL_SKIRT = (-65.0, -57.6, 21.1, 22.1, -3.3, -2.0)           # outer cheek plate carried down to the bottom flange (the old outer skin below it was 0.24)
PL_WIN = (-59.6, -57.6, 21.1, 22.1, -1.6, 0.7)              # outer cheek plate closed over the rear window (left a 0.1 fin at X -57.7)
STRUT_PAD = (-47.0, -45.7, 19.05, 20.45, 1.0, 2.0)          # square end pad where truss diagonal 1 meets the top flange (was a 0.2 spike beside a notch)
PL_CAP = (-65.6, -57.6, 21.1, 22.1, 1.0, 1.9)               # ... and up to the top flange (0.2 skin above the plate)
SHK_BOSS_R, SHK_BOSS_Y1, SHK_LSCR_L = 1.7, 26.9, 8.0          # outboard boss on the shock clevis outer cheek: M1.6 x 8 threads 4.6 mm (was x4.5: 1.1 mm); 2.1 mm to the wheel
CLV_TIE = (-47.8, -45.7, 18.0, 20.45, 0.5, 1.5)             # verifier FE: ties the clevis rail into the strut pad (the 0.36 x 0.25 ligament at X -47 had SF 0.8)
CLV_WEB = (-49.7, -46.6, 19.05, 20.45, -3.3, 0.4)           # verifier FE: web from the bottom flange up to the rail ahead of the shock eye
FIN_PAD = (-47.0, -45.7, 20.4, 21.05, 1.0, 2.0)             # joins the strut pad to the outer flange rib (a 0.13 lip lay between them)
ARM_FILLS = (CLV_RAIL, CLV_SLIT, TAB_BACK, WIN_FILL, PL_SKIRT, PL_WIN, PL_CAP, STRUT_PAD, CLV_TIE, CLV_WEB, FIN_PAD)      # all added BEFORE the axle-ear sweep is cut, so none can enter the ear's swept space
def arm(side):
    old = ld("Rear_Trailing_Arm_L"); keep = I(old, box(-70, PRIDE_ARM_CUT + SH, -40, 40, -20, 20)); keep.apply_translation([-SH, 0, 0])     # keep the rear part (axle end, shock clevis, tab)
    ya0, ya1 = ARM_Y0, ARM_Y1
    boss = cylY(PL[0], PL[1], LUG_R, ya0, ya1, 40); neck = trimesh.convex.convex_hull(np.vstack([boss.vertices, box(PRIDE_ARM_CUT, PRIDE_ARM_CUT + 0.3, ya0, ya1, A_[1] - 1.6, A_[1] + 1.6).vertices]))   # pitched neck: pivot boss (high) -> axle-side keep part (low)
    # Yeti AX31109 recessed central diagonal X-truss web (|Y|=19.05..20.45, 1.40 mm thick, recessed 1.05 mm inside both side faces)
    def _strut2d(p0, p1, w=1.1, y0_=19.05, y1_=20.45):
        p0, p1 = np.array(p0, float), np.array(p1, float); d = p1 - p0; n = np.array([-d[1], d[0]])/np.linalg.norm(d)*(w/2)
        pts = [[q[0] + s_*n[0], y_, q[1] + s_*n[1]] for q in (p0, p1) for s_ in (-1, 1) for y_ in (y0_, y1_)]
        return trimesh.convex.convex_hull(np.array(pts, float))
    _truss_adds = [
        _strut2d((-46.8, -3.4), (-39.3, 1.5), w=1.15),     # main window X-truss diagonal 1
        _strut2d((-46.8, 1.5), (-39.3, -3.4), w=1.15),     # main window X-truss diagonal 2
        _strut2d((-60.6, -2.7), (-57.4, 1.3), w=1.15),     # rear window diagonal truss strut
        _strut2d((-39.3, -3.4), (-36.4, 0.3), w=1.15),     # verifier FE: closes the truss from the main window's lower front corner up to the pivot boss
        box(-46.8, -39.2, 21.00, 21.80, 1.15, 2.00),       # Yeti AX31245 outer top link-plate flange rib
        box(-46.8, -39.2, 21.00, 21.80, -4.05, -3.15),     # Yeti AX31245 outer bottom link-plate flange rib (0.8 wide, overlaps the beam flange: no 0.1 gap / 0.4 fin)
        trimesh.convex.convex_hull(np.vstack([cylY(AXB_X, AXB_Z, 2.2, AXB_PL_Y0, AXB_PL_Y1, 28).vertices, box(-58.6, -57.6, AXB_PL_Y0, AXB_PL_Y1, -3.0, 1.8).vertices])),   # AX31245-style outer side plate over the axle ear: the ear slot (ear Y 19.8..21.0 + 0.24) left a 0.25 mm outer cheek
    ]
    m = U([keep, boss, neck] + _truss_adds + [box(*_b) for _b in ARM_FILLS] + [cylY(SHK_LO[0], SHK_LO[1], SHK_BOSS_R, ARM_Y1 - 0.15, SHK_BOSS_Y1, 28)]); m = D(m, [cylY(PL[0], PL[1], M2_CLR_D/2, ya0 - .1, ya1 + .1, 24), cylY(AXB_X, AXB_Z, M2_CLR_D/2, AXB_PL_Y0 - 0.2, AXB_PL_Y1 + 0.1, 24), AXL_END, cylY(AXB_X, AXB_Z, AXB_SLOT_R, AXB_SLOT_Y0, AXB_SLOT_Y1, 24), cylY(SHK_LO[0], SHK_LO[1], PIV_PILOT_D/2, 22.0, SHK_BOSS_Y1 + 0.1, 16)])
    return m if side > 0 else mir(m)
# ---- Rev 45 trailing arm, drawn as one part (no v3 remnant): straight bottom rail, straight top line, one X truss, full-width post at the truss / clevis join, hulled ends ----
AR_Y0, AR_Y1 = 18.0, 21.5                 # rail / post / spine |Y| span (inner face 18.0)
AR_RH = 1.3                               # rail height (constant)
AR_WEB = (19.05, 20.45)                   # truss web |Y| span (1.4 thick, centred between the rail faces)
AR_XR, AR_XF = -61.0, -39.5               # rails run from the axle-end spine to the pivot blend
AR_POST = (-49.3, -48.1)                  # full-width post: front wall of the shock-eye pocket and rear end of the truss (the old 0.36 x 0.25 ligament at X -47)
AR_SPINE_X1 = -55.75; AR_TAB_Z = 4.9      # AR_TAB_Z = TAB_TOP_Z (defined later with the sway bar)
#                      # solid spine block between the axle ear and the eye pocket
AR_EAR_R, AR_EARCH_Y = 2.6, ((18.0, 19.5), (21.3, 22.1))      # axle-ear cheek radius round the bolt; inner / outer cheek |Y| (ear slot 19.56..21.24 between them)
AR_CLV_X = (-56.6, -49.3); AR_CHK_IN, AR_CHK_OUT = (18.0, 18.8), (22.2, 23.0)     # shock clevis: X span, inner / outer cheek |Y| (eye 18.9..21.9)
AR_DIAG_W, AR_GUS = 1.2, 1.2              # truss diagonal width; corner gusset leg
def _zt(x): return 2.0 + 0.045*(x + 64.0)             # top edge of the top line (1.0 mm under the swept shock body at X -50)
def _zb(x): return -2.3 + 0.0735*(x + 53.0)           # top edge of the bottom rail (passes 0.1 under the eye pocket, meets the pivot boss underside)
def _prism(xz, y0, y1): return trimesh.convex.convex_hull(np.array([[x_, y_, z_] for (x_, z_) in xz for y_ in (y0, y1)], float))
def _band(x0, x1, top, y0, y1, h=None): h = AR_RH if h is None else h; return _prism([(x0, top(x0)), (x1, top(x1)), (x1, top(x1) - h), (x0, top(x0) - h)], y0, y1)
def _full(x0, x1, y0, y1): return _prism([(x0, _zt(x0)), (x1, _zt(x1)), (x1, _zb(x1) - AR_RH), (x0, _zb(x0) - AR_RH)], y0, y1)      # full depth between the top line and the rail underside
def arm_v10(side):      # Rev 45 one-piece redesign, NOT used: the verifier's FE gave SF 0.5 (v9 arm: 1.7). Kept for reference only
    xp0, xp1 = AR_POST; wy0, wy1 = AR_WEB
    boss = cylY(PL[0], PL[1], LUG_R, ARM_Y0, ARM_Y1, 48)
    blend = trimesh.convex.convex_hull(np.vstack([cylY(PL[0], PL[1], LUG_R, AR_Y0, AR_Y1, 48).vertices, _full(AR_XF - 0.4, AR_XF, AR_Y0, AR_Y1).vertices]))      # rails run tangentially into the pivot boss
    add = [boss, blend,
           _band(AR_XR, AR_XF, _zb, AR_Y0, AR_Y1),                                   # bottom rail, one straight bar
           _band(xp0, AR_XF, _zt, AR_Y0, AR_Y1),                                     # top rail ahead of the shock
           _band(AR_XR, xp0 + 0.1, _zt, *AR_CHK_IN),                                 # ... carried past the shock eye by the inner cheek (same top line)
           _full(xp0, xp1, AR_Y0, AR_Y1), _full(AR_XF - 1.2, AR_XF, AR_Y0, AR_Y1),   # posts at both ends of the truss
           _full(AR_XR, AR_SPINE_X1, AR_Y0, AR_Y1),                                  # spine block behind the eye pocket
           _full(AR_CLV_X[0], AR_CLV_X[1] + 0.1, *AR_CHK_IN),                        # clevis inner cheek (0.8)
           _band(AR_CLV_X[0], xp0 + 0.1, _zb, AR_Y1 - 0.05, AR_CHK_OUT[1]),          # clevis floor out to the outer cheek
           _full(AR_CLV_X[0], AR_CLV_X[1], *AR_CHK_OUT),                             # clevis outer cheek (0.8), carries the M1.6 boss
           cylY(SHK_LO[0], SHK_LO[1], SHK_BOSS_R, AR_CHK_OUT[1] - 0.05, SHK_BOSS_Y1, 28)]
    for (y0, y1) in AR_EARCH_Y:                                                       # axle-ear cheeks: bolt circle hulled into the full depth of the spine (teardrop, no shelf)
        add.append(trimesh.convex.convex_hull(np.vstack([cylY(AXB_X, AXB_Z, AR_EAR_R, y0, y1, 40).vertices, _full(AR_XR, AR_XR + 0.6, y0, y1).vertices])))
    add.append(_full(AR_XR - 0.05, AR_SPINE_X1, AR_Y1 - 0.05, AR_EARCH_Y[1][1]))      # spine carried out under the outer ear cheek
    # one X truss between the posts, in the web plane, with four equal corner gussets
    xa, xb = xp1, AR_XF - 1.2; za0, za1, zb0, zb1 = _zb(xa), _zt(xa) - AR_RH, _zb(xb), _zt(xb) - AR_RH
    def _diag(p0, p1):
        p0, p1 = np.array(p0), np.array(p1); n_ = np.array([-(p1 - p0)[1], (p1 - p0)[0]]); n_ = n_/np.linalg.norm(n_)*AR_DIAG_W/2
        return _prism([tuple(p0 + n_), tuple(p0 - n_), tuple(p1 - n_), tuple(p1 + n_)], wy0, wy1)
    g = AR_GUS
    add += [_diag((xa - 0.3, za0 - 0.3), (xb + 0.3, zb1 + 0.3)), _diag((xa - 0.3, za1 + 0.3), (xb + 0.3, zb0 - 0.3)),
            _prism([(xa - 0.1, za0 - 0.1), (xa + g, za0 - 0.1), (xa - 0.1, za0 + g)], wy0, wy1), _prism([(xa - 0.1, za1 + 0.1), (xa + g, za1 + 0.1), (xa - 0.1, za1 - g)], wy0, wy1),
            _prism([(xb + 0.1, zb0 - 0.1), (xb - g, zb0 - 0.1), (xb + 0.1, zb0 + g)], wy0, wy1), _prism([(xb + 0.1, zb1 + 0.1), (xb - g, zb1 + 0.1), (xb + 0.1, zb1 - g)], wy0, wy1)]
    # sway-bar drop-link tab on the spine (wire at TAB_X, |Y| LINK_Y): two cheeks + back wall, rounded top
    tz = AR_TAB_Z
    add += [_prism([(-57.7, _zt(-57.7) - 0.3), (-57.7, tz - 0.4), (-57.3, tz), (-56.0, tz), (-55.7, tz - 0.3), (-55.7, _zt(-55.7) - 0.3)], AR_Y0, AR_Y1)]
    m = U(add)
    cuts = [cylY(PL[0], PL[1], M2_CLR_D/2, ARM_Y0 - .1, ARM_Y1 + .1, 24),                                                    # pivot bore (M2)
            cylY(AXB_X, AXB_Z, M2_CLR_D/2, AR_EARCH_Y[0][0] - 0.2, AR_EARCH_Y[1][1] + 0.1, 24), AXL_END,                      # axle bolt bore, swept axle ear
            box(-70.0, AR_XR + 0.3, 19.5 + 0.02, 21.3 - 0.02, -8.0, 8.0),                                                     # ear slot squared off between the cheeks
            cylY(SHK_LO[0], SHK_LO[1], SHK_EYE_R + 0.5, AR_CHK_IN[1], AR_CHK_OUT[0], 32),                                     # shock-eye pocket ...
            box(SHK_LO[0] - SHK_EYE_R - 0.5, xp0, AR_CHK_IN[1], AR_CHK_OUT[0], SHK_LO[1], 9.0),                               # ... open upward for the neck, up to the post
            box(-56.5, -55.6, 18.8, 20.6, 3.25, 9.0),                                                                         # drop-link slot in the tab (back wall stays)
            cylY(SHK_LO[0], SHK_LO[1], 1.25, AR_CHK_IN[0] - 0.1, AR_CHK_IN[1] + 0.05, 24),                                    # screw-head passage through the inner cheek
            cylY(SHK_LO[0], SHK_LO[1], PIV_PILOT_D/2, AR_CHK_OUT[0] - 0.2, SHK_BOSS_Y1 + 0.1, 16)]                            # M1.6 pilot through the outer cheek and boss
    m = D(m, cuts)
    return m if side > 0 else mir(m)
UL_FLAT, UL_HOLE_IN, UL_FIN_X1, UL_FIN_Y = 0.0, 5.0, -63.6, 2.9      # flat eye extension; pivot holes run right through the blade (a 0.15 cap closed their inner end); the R blade's wedge left outside the stacking cut (0.03..0.4 fin) is trimmed
def link(side):      # diagonal upper link: corner pivot (Y axis) -> shared axle-top bracket (vertical pin at Y 0); L eye above R eye (0.1 gap)
    zc = BP[1] + (1 if side > 0 else -1)*(BR_EYE_T + UL_EYE_GAP)/2
    bs = trimesh.convex.convex_hull(np.vstack([cylY(_h[0], _h[1], 1.75, side*(UL_YB - 1.35), side*(UL_YB + 1.35), 28).vertices for _h in PU_HOLES]))
    eye = cylZ(BP[0], 0.0, 1.75, zc - BR_EYE_T/2, zc + BR_EYE_T/2, 28)
    _ud = np.array([PU[0] - BP[0], side*UL_YB]); _ud = _ud/np.linalg.norm(_ud)*UL_FLAT      # the eye's flat 1.1 section runs UL_FLAT along the blade before it tapers: the stacked zone (|Y| < 2) was 0.69 thick at its edge
    blade = trimesh.convex.convex_hull(np.vstack([cylY(PU[0], PU[1], 1.75, side*(UL_YB - 1.35), side*(UL_YB + 1.35), 28).vertices, eye.vertices, cylZ(BP[0] + _ud[0], _ud[1], 1.75, zc - BR_EYE_T/2, zc + BR_EYE_T/2, 28).vertices]))      # flat tapered blade, thin at the eye
    m = U([blade, bs, eye]); m = D(m, [cylY(_h[0], _h[1], 1.28/2, side*(UL_YB - UL_HOLE_IN), side*(UL_YB + 2), 24) for _h in PU_HOLES] + [cylZ(BP[0], 0.0, 0.8, zc - 2, zc + 2, 24)])
    xcut = PU[0] - 6.0
    # eye stacking only matters where the two blades meet at the bracket (|Y| < 2): L keeps above BP+0.05, R keeps below BP-0.05
    if side > 0: m = D(m, [box(-140, xcut, -2.0, 2.0, -5, BP[1] + UL_EYE_GAP/2)])
    else: m = D(m, [box(-140, xcut, -2.0, 2.0, BP[1] - UL_EYE_GAP/2, 20), box(-140, UL_FIN_X1, -UL_FIN_Y, -1.9, BP[1] - UL_EYE_GAP/2, 20), box(-140, xcut, -2.0, 2.0, -5, BP[1] - UL_EYE_GAP/2 - BR_EYE_T)])
    return m
CLV_CUT_Y0, CLV_CUT_Y1, CLV_ROOF_Z = 17.9, 19.1, 3.25      # shock-neck relief takes the whole inner ridge (was 18.45..18.9: left a 0.35 ridge and a 0.1 fin); flat roof over the eye pocket instead of the arc (no feather edges)
SLOT_LIP = (-62.4, -61.15, 19.5, 21.3, 0.9, 2.2)            # removes the 0.26 mm lip the ear sweep left over the slot mouth
ARM_C3 = os.environ.get("ARM_C3", "1") == "1"; ARM_DIR = os.environ.get("ARM_DIR", "arm_d4")
for sg, n in ((1, "L"), (-1, "R")):
    _arm_m = D(arm(sg), [box(-53.2, -47.8, *ysp(sg, 18.8, 19.1), -2.0, 8.0), box(-53.2, -47.8, *ysp(sg, 21.4, 22.1), -2.0, 8.0), box(-49.2, -46.85, *ysp(sg, 21.06, 21.85), 1.05, 2.2), box(-55.8, -53.1, *ysp(sg, 21.42, 22.6), 2.12, 5.0), box(-57.55, -54.5, *ysp(sg, 21.22, 21.9), -4.3, -3.1), cylY(SHK_LO[0], SHK_LO[1], SHK_EYE_R + 0.5, *ysp(sg, 18.8, 22.1), 24), box(-51.6, -46.5, *ysp(sg, CLV_CUT_Y0, CLV_CUT_Y1), 1.5, 8.0), box(SHK_LO[0] - SHK_EYE_R - 0.5, SHK_LO[0] + SHK_EYE_R + 0.5, *ysp(sg, 18.8, 22.1), SHK_LO[1], CLV_ROOF_Z), box(SLOT_LIP[0], SLOT_LIP[1], *ysp(sg, SLOT_LIP[2], SLOT_LIP[3]), SLOT_LIP[4], SLOT_LIP[5])])   # shock clevis: eye side clearance 0.45 -> 0.75 / 1.0 (cheeks stay >= 0.8 thick)
    P[f"Rear_Trailing_Arm_{n}"] = simp(single(f"Rear_Trailing_Arm_{n}", _arm_m), 0.002)
    if ARM_C3:      # Rev 50: plate-beam arm "C3" (built by the arm study from arm_v10, FE-verified SF 2.4 at 200 g, droop clearance 0.610 mm): the verified mesh is loaded as is
        _c3 = trimesh.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", ARM_DIR, f"Rear_Trailing_Arm_{n}.stl")); assert _c3.is_watertight
        P[f"Rear_Trailing_Arm_{n}"] = _c3
    P[f"Rear_Upper_Link_{n}"] = link(sg)
    # hardware: M1.4 pivot screws (tapped into the tub pads, no nuts), screw heads outside the arm / inside the link
    sh = U([cylY(PL[0], PL[1], 1.0, sg*(ARM_Y1 - 10.0), sg*(ARM_Y1 + REAR_DY), 12), cylY(PL[0], PL[1], M2_HEAD_R, sg*(ARM_Y1 + REAR_DY), sg*(ARM_Y1 + REAR_DY + 1.1), 16)])      # M2 x 10 button head: 3.5 mm in the pad (whole pad), tip 1.0 proud inside
    P[f"HW_Rear_Trailing_Pivot_Screw_{n}"] = sh
    sh2 = U([cylY(PU[0], PU[1], 0.7, sg*(UL_YB - 0.95), sg*(TUB_OUT + 0.05), 12), cylY(PU[0], PU[1], 1.35, sg*TUB_OUT, sg*(TUB_OUT + 0.9), 16)])
    P[f"HW_Rear_Upper_Link_Pivot_Screw_{n}"] = sh2
# unchanged v3 parts we keep using (arms' hardware at the axle, shocks, axle-side link HW)
KEEP = ["HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R",
        "Ref_Shock_RL", "Ref_Shock_RR", "HW_Rear_Shock_Lower_Screw_L", "HW_Rear_Shock_Lower_Screw_R", "HW_Rear_Shock_Upper_Screw_L", "HW_Rear_Shock_Upper_Screw_R",
        "Ref_Rear_Driveshaft"]
def _stretch_ds(m):
    # Realign visual rear driveshaft from old v3 endpoints (-15.5, 0, 4.89)->(-46.84, 0, 1.60)
    # onto raised gearbox rear stub (-15.55, 0, GBX_SHAFT_Z=9.90) -> shifted axle pinion (-46.8-SH, 0, 3.50)
    m = m.copy(); v = m.vertices.copy()
    p0_o = np.array([-15.50, 0.0, 4.89]); p1_o = np.array([-46.84, 0.0, 1.60])
    L_o = float(np.linalg.norm(p1_o - p0_o)); u_o = (p1_o - p0_o)/L_o
    v_ax = np.array([0.0, 1.0, 0.0]); w_o = np.cross(u_o, v_ax)
    p0_n = np.array([-15.55, 0.0, GBX_SHAFT_Z]); p1_n = np.array([-46.8 - SH, 0.0, 3.50])
    L_n = float(np.linalg.norm(p1_n - p0_n)); u_n = (p1_n - p0_n)/L_n; w_n = np.cross(u_n, v_ax)
    d = v - p0_o
    s = np.clip((d @ u_o)/L_o, 0.0, 1.0)
    ty = d @ v_ax; tz = d @ w_o
    m.vertices = p0_n + np.outer(s*L_n, u_n) + np.outer(ty, v_ax) + np.outer(tz, w_n)
    return m
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
DRV_MAX = float(os.environ.get("DRV_MAX", 21.0))      # deg: shaft angle limit at the axle; 21 = Injora-type double-joint rear shaft (single stock U-joint was 13)
def drv_ang(ta_):
    a_, u_, T_ = kin(ta_); Qr = (T_ @ np.array([-46.8 - SH, 0, 3.5, 1]))[[0, 2]]; v_ = Qr - np.array([-15.5, GBX_SHAFT_Z]); return math.degrees(math.atan2(v_[1], v_[0])) if v_[0] > 0 else math.degrees(math.atan2(-v_[1], -v_[0]))
_lo, _hi = TA_MIN, 0.0
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
    P[f"HW_Rear_Trailing_Axle_Screw_{n}"] = U([cylY(_cx, _cz, 0.9, *ysp(sg, ARM_IN_Y - AXB_NUT_T + 0.5, AXB_PL_Y1 + 0.05), 14), cylY(_cx, _cz, 1.4, *ysp(sg, AXB_PL_Y1, AXB_PL_Y1 + 1.1), 16)])      # M2 x ~5.7 under head
    P[f"HW_Rear_Trailing_Axle_Nut_{n}"] = D(cylY(_cx, _cz, 1.4, *ysp(sg, ARM_IN_Y - AXB_NUT_T - 0.15, ARM_IN_Y - 0.15), 6), [cylY(_cx, _cz, 0.9, *ysp(sg, ARM_IN_Y - AXB_NUT_T - 0.1, ARM_IN_Y + 0.1), 14)])
    _b = P[f"HW_Rear_Shock_Lower_Screw_{n}"].bounds; _cx, _cz = (_b[0][0] + _b[1][0])/2, (_b[0][2] + _b[1][2])/2
    P[f"HW_Rear_Shock_Lower_Screw_{n}"] = U([cylY(_cx, _cz, 1.1, *ysp(sg, 18.05, 18.8), 16), cylY(_cx, _cz, 0.72, *ysp(sg, 18.75, 18.8 + SHK_LSCR_L), 12)])      # M1.6 x 8
# ---- axle-top bracket (printed saddle clamp on the pumpkin, walls >= 0.95 mm, beveled wings clear of +-10 deg roll twist) + vertical pin ----
BR_PLATE_Z0, BR_PLATE_Z1 = 5.4, 7.10      # top lowered 0.2 (was 7.30): 0.50 under the lower link eye, so the saddle clears it at +-10 deg axle roll
_br_raw = trimesh.convex.convex_hull(np.array([
    [x, y, z] for x in (BP[0] - 3.4, BP[0] + 1.9) for (y, z) in ((-4.5, BR_PLATE_Z0), (4.5, BR_PLATE_Z0), (-4.5, 6.85), (4.5, 6.85), (-1.7, BR_PLATE_Z1), (1.7, BR_PLATE_Z1))
], float))
_br_key = box(BP[0] - 2.0, BP[0] + 1.9, -1.55, 1.55, 3.2, BR_PLATE_Z1)
_ax_cut = dil_(I(R.M['Ref_Rear_Axle'], box(BP[0] - 4.0, BP[0] + 3.0, -6.0, 6.0, 2.0, 8.0)), 0.15)
BR_EAR_X0, BR_EAR_Y0, BR_EAR_Y1 = -57.5 - SH, 1.75, 2.80      # the two link-mount ears on the axle housing (measured: X >= -66.5, |Y| 1.75..2.8, top Z 7.0): open slots in the saddle instead of a 0.04..0.10 mm skin over them
P["Rear_Axle_Top_Bracket"] = single("Rear_Axle_Top_Bracket", D(U([_br_raw, _br_key]), [_ax_cut, cylZ(BP[0], 0.0, 0.625, 3.0, BR_PLATE_Z1 + 0.5, 24)] + [box(BR_EAR_X0 - 0.15, BP[0] + 2.0, *ysp(sg_, BR_EAR_Y0 - 0.15, BR_EAR_Y1 + 0.15), BR_PLATE_Z0 - 0.1, BR_PLATE_Z1 + 0.2) for sg_ in (1, -1)]))
_pt = BP[1] + UL_EYE_GAP/2 + BR_EYE_T
P["HW_Rear_Axle_Top_Pin"] = U([cylZ(BP[0], 0.0, 0.7, 3.4, _pt + 0.4, 16), cylZ(BP[0], 0.0, 1.3, _pt, _pt + 0.4, 18)])
# ---- rear track widened by 2 x REAR_DY (user decision): arms, axle ears/ends, wheels, wheel nuts, axle screws/nuts and the shock lower screw move out; the axle centre, upper links, bracket and driveshaft stay.
def _shy(m, dy): m = m.copy(); m.apply_translation([0, dy, 0]); return m
def _stretch_y(m, y0, dy):
    m = m.copy(); v = m.vertices.copy(); mk = np.abs(v[:, 1]) > y0; v[mk, 1] += np.sign(v[mk, 1])*dy; m.vertices = v; return m
if REAR_DY:
    R.M["Ref_Rear_Axle"] = _stretch_y(R.M["Ref_Rear_Axle"], 14.5, REAR_DY); P["Ref_Rear_Axle"] = R.M["Ref_Rear_Axle"].copy()      # prismatic tube between the housing and the ear: stretched, not translated
    for _n, _sg in (("L", 1), ("R", -1)):
        for _k in (f"Rear_Trailing_Arm_{_n}", f"HW_Rear_Trailing_Axle_Screw_{_n}", f"HW_Rear_Trailing_Axle_Nut_{_n}", f"HW_Rear_Shock_Lower_Screw_{_n}"): P[_k] = _shy(P[_k], _sg*REAR_DY)
        for _k in (f"Ref_Wheel_R{_n}", f"HW_Rear_Wheel_Nut_{_n}"):
            R.M[_k] = _shy(R.M[_k], _sg*REAR_DY); P[_k] = R.M[_k].copy()
    P["Chassis_Tub"] = single("Chassis_Tub(pivot boss)", U([P["Chassis_Tub"]] + [D(cylY(PL[0], PL[1], 1.9, *ysp(sg_, TUB_OUT - 0.3, TUB_OUT + REAR_DY - 0.1), 28), [cylY(PL[0], PL[1], M2_PILOT_D/2, *ysp(sg_, TUB_OUT - 0.4, TUB_OUT + REAR_DY), 16)]) for sg_ in (1, -1)]))      # pivot spacer boss on the tub side wall, 0.1 mm short of the arm face: the arm turns on it
# ---- rear cage (Rev 13, Yeti style): hoop + two side frames (upper rail, drop diagonals, post, battery cradle rails, half straps) ----
TUBE_R = 1.15                                         # measured on Roll_Cage (section 2.3 x 2.5)
PXH = float(os.environ.get("PXH", -65.5)); HOOP_Y = 15.5; HOOP_TOP = DECK_Z1 + 13.7             # hoop behind the pack, now stands above the pack top (27.5)
CRUN_Z = DECK_Z1 + 13.3                               # upper side rails / strap bars (bar bottom 1 mm above the pack top)
TAB_DX = float(os.environ.get('TAB_DX', -1.2))      # sway-link tab moved rearward (arm C4)
BAR_X, BAR_Z, BAR_Y, BAR_E_X = PXH, float(os.environ.get("BAR_Z", 17.2)), 19.1, -46.75 - SH + TAB_DX; LINK_Y = 19.8 + REAR_DY      # lever wire runs at |Y| 19.1 (1.6 mm off the shell); a 0.65 mm jog at its front end puts the drop link back at 19.75 (clear of the cage plate and the arm cheek)
TAB_X, TAB_TOP_Z, LINK_INS = -46.75 - SH + TAB_DX, 4.0, 0.8; WIRE_R, LINK_R, TUN_R = 0.4, 0.45, 0.9
PXP = -65.0; FOOT_SCREW_D = 0.6                       # side post X
XA, YF = PXH + 6.0, 15.7                              # hoop joint plate centre (X) and frame line (Y)
XK = -58.5                                            # end of the horizontal upper rail, drop diagonal starts
XN, YN, ZN = -56.0, 16.2, 21.3                        # frame end over the main-cage lug (horizontal lap, vertical screws)
JT_Z0, JT_Z1 = 12.5, 22.5           # hoop joint plate Z
HJ_SCREWS = [(XA - 1.2, 15.5), (XA + 1.2, 19.5)]; NJ_SCREWS = [(-52.4, 16.4), (-49.8, 16.4), (-47.2, 16.4)]      # 3 x M1.6 per side
RAIL_Z = DECK_Z1 + 1.25; CRAIL_Z = 24.4; RAIL_X0 = -75.5       # RAIL_Z: deck rails; CRAIL_Z: battery cradle rails (mid pack height, above the main-cage rail tops at Z 20.7) with front hooks down to the deck
BUMP_POST_FOOT = float(os.environ.get("BUMP_FOOT", 6.8)); BUMP_POST_X, BUMP_POST_Y, BUMP_POST_Z = -71.0, 11.8, BUMP_POST_FOOT + 1.15      # bump stops hang from the cradle rung over the axle (the deck that carried the old pads is gone)
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
_ctube0 = ctube; TUBE_REG = {}
def ctube(p0, p1, r=None, b0=None, b1=None, e0=None, e1=None, s=None):
    m = _ctube0(p0, p1, r, b0, b1, e0, e1, s); TUBE_REG[id(m)] = (m, np.array(p0, float), np.array(p1, float), e0, e1, s); return m
NODE_TOL = 0.35
def fix_tubes(lst, hulls=True):
    """tube-tube nodes without shards: an end that meets another tube (end to end or on its side) stops at the node point (no 1.2 mm extension poking out at odd angles);
    the end rings of every tube meeting at one point are hulled into one clean convex node."""
    recs = [(i, TUBE_REG[id(m)]) for i, m in enumerate(lst) if id(m) in TUBE_REG and TUBE_REG[id(m)][0] is m]
    def near(pt, j):
        _, (_, q0, q1, _, _, _) = recs[j]; d = q1 - q0; t = float(np.dot(pt - q0, d)/np.dot(d, d)); return t, float(np.linalg.norm(q0 + np.clip(t, 0, 1)*d - pt))
    out = list(lst); clusters = []
    for k, (i, (m, q0, q1, e0, e1, s)) in enumerate(recs):
        ne = [e0, e1]
        for end, pt in ((0, q0), (1, q1)):
            if (e0, e1)[end] is not None: continue
            hit = False
            for j, (i2, (m2, r0, r1, f0, f1, s2)) in enumerate(recs):
                if j == k: continue
                t, dist = near(pt, j)
                if dist < NODE_TOL and -0.02 < t < 1.02: hit = True; break
            if hit: ne[end] = 0.0
        if ne != [e0, e1]: out[i] = _ctube0(q0, q1, None, None, None, ne[0], ne[1], s)
    if hulls:
        pts_ = []
        for k, (i, (m, q0, q1, e0, e1, s)) in enumerate(recs):
            for end, pt in ((0, q0), (1, q1)): pts_.append((k, end, pt))
        used = set()
        for a_ in range(len(pts_)):
            if a_ in used: continue
            grp = [a_] + [b_ for b_ in range(a_ + 1, len(pts_)) if b_ not in used and np.linalg.norm(pts_[a_][2] - pts_[b_][2]) < NODE_TOL and pts_[b_][0] != pts_[a_][0]]
            if len(grp) < 2: continue
            used.update(grp); c = np.mean([pts_[g][2] for g in grp], axis=0); vs = []
            for g in grp:
                k, end, pt = pts_[g]; i = recs[k][0]; mm = out[i]; ax_ = (recs[k][1][2] - recs[k][1][1]); ax_ = ax_/np.linalg.norm(ax_)*(1 if end == 1 else -1)
                v_ = mm.vertices[np.abs((mm.vertices - pt) @ ax_) < 1e-6]; vs.append(v_)
            if all(len(v_) >= 8 for v_ in vs): out.append(trimesh.convex.convex_hull(np.vstack(vs)))
    return out
def ball(c, r): return box(c[0] - r, c[0] + r, c[1] - r, c[1] + r, c[2] - r, c[2] + r)      # square bearing boss on the hoop leg (sway-bar wire)
BUMP_X = -71.0
# ---- Rev 47 short tail: the shell ends at X -67.0 and its rear rail top ridge is Z = 33.480 + 0.28400*(X + 64.0) at |Y| 21.1 (SHELL_REAR_SLOPE.md, 15.855 deg)
SHELL_RIDGE_Z0, SHELL_RIDGE_M, SHELL_END_X = 33.480, 0.28400, -67.0
TAIL_DX = PXH + 78.0                                   # how far the rear hoop (and the radiator / light bar on it) moved from the long-tail position
FB_OFF = 1.56 + (CG_S/2)*math.sqrt(1 + SHELL_RIDGE_M**2)      # fastback rail centre line below the shell ridge: 1.5 mm clear above the tube's top face, constant along X
def zfb(x): return SHELL_RIDGE_Z0 + SHELL_RIDGE_M*(x + 64.0) - FB_OFF      # fastback rail centre height: EXACTLY the shell slope
TIE_X = -31.0; A1_Y, A1_Z = 19.6, 29.2      # beltline tie corner X; A-post top / beltline node (|Y|, Z): 0.8 inboard and 1.2 lower than the B-post end, for 1.5 mm to the shell cowl
FB_KX0 = -46.0; FB_KX1 = FB_KX0 - (CG_YT - CG_YS)      # kink: starts at X -46.0 on |Y| 11.5, ends 4.2 further back on |Y| 15.7 (45 deg in plan)
REAR_BAR = (SHELL_END_X - 0.2, PXH + CG_S/2, CG_ZR - CG_S/2, CG_ZR + CG_S/2 + 1.0)      # rear cross tube X0, X1, Z0, Z1: 1.0 taller than a tube so 0.95 / 1.55 mm stay round the hinge-screw groove now the light bar is gone
def zlr(x): return 12.0 + 5.8*(x + 78.0)/42.6          # lower rail line (unchanged from the long tail: same heights over the axle)
def zlow(x): return float(np.interp(x, [PXH, CG_XT], [HZ0, CG_ZTF]))      # tail lower tube height along X (hoop foot -> tail foot)
RUNG2_X, RUNG2_Z = -45.0, 16.9
CG_XSN = -42.0      # rear shock node on the tail roof rail: roof X, front strut and fastback strut all meet here beside the shock pins (cage research P1)
YF = CG_YT; RUNG_Z = 16.6                                  # pack-support rungs: >= 1.5 mm above the axle top bracket at full bump, 0.35 mm under the pack (Z 18.1)
CG_FOOT_Y1 = 23.3      # main foot plates reach 0.3 past the tub side: 0.8 mm round the M2 clearance hole (was 0.5)
CG_XTP = -35.4; CG_ZN = SHELL_RIDGE_Z0 + SHELL_RIDGE_M*(CG_XTP + 64.0) - (1.56 + (CG_S/2)*math.sqrt(1 + SHELL_RIDGE_M**2))      # tail hoop (the two tail posts + top bar): X, top height. The fastback rails run straight from its top corners down to the rear hoop corners (10.8 deg; the shell sail line is ~15..17 deg)
_rb = [(REAR_BAR[0] + CG_CH, REAR_BAR[2]), (REAR_BAR[1] - CG_CH, REAR_BAR[2]), (REAR_BAR[1], REAR_BAR[2] + CG_CH), (REAR_BAR[1], REAR_BAR[3] - CG_CH), (REAR_BAR[1] - CG_CH, REAR_BAR[3]), (REAR_BAR[0] + CG_CH, REAR_BAR[3]), (REAR_BAR[0], REAR_BAR[3] - CG_CH), (REAR_BAR[0], REAR_BAR[2] + CG_CH)]
parts = [trimesh.convex.convex_hull(np.array([[x_, y_, z_] for (x_, z_) in _rb for y_ in (-YF - CG_S/2, YF + CG_S/2)], float)), ctube((PXH, -YF, zlr(PXH)), (PXH, -CG_YS, zlr(PXH))), ctube((PXH, CG_YS, zlr(PXH)), (PXH, YF, zlr(PXH))), ctube((BUMP_X, -CG_YS, RUNG_Z), (BUMP_X, CG_YS, RUNG_Z)), ctube((RUNG2_X, -CG_YS, RUNG2_Z), (RUNG2_X, CG_YS, RUNG2_Z)),    # rear hoop top / bottom bar, two pack rungs between the lower rails
         ctube((CG_XM0, -CG_YS, zfb(CG_XM0)), (CG_XM0, CG_YS, zfb(CG_XM0)))]      # roof-peak bar on the B-posts: the fastback and roof rails meet here
# Yeti rear body tilt-hinge coordinates (p46 — shared with @shell): horizontal Y-axis pin bore on twin rear hoop ears (Ø1.75 mm + 45 deg teardrop for upside-down print)
HINGE_X, HINGE_Z, HINGE_Y0, HINGE_Y1, HINGE_R, HINGE_BORE_D = PXH + 1.5, 29.0, CG_YT, 18.6, 2.5, M2_CLR_D      # M2 hinge screw: D2.2 clearance in the ear (1.4 wall), D1.7 pilot in the shell lug
SHK_TAB_YO = SHK_YU - SHK_EYE_T/2 - 0.8    # outer face of shock mount plate (15.9)
for sg in (1, -1):
    Yt, Ym = sg*CG_YT, sg*CG_YM
    # Yeti AX31102/AX31166 twin-position rear shock mount plate (default hole SHK_UX/UZ + upright tuning hole SHK_UX2/UZ2)
    _shk_plate_base = trimesh.convex.convex_hull(np.vstack([
        cylY(SHK_UX, SHK_UZ, 2.3, *ysp(sg, CG_YS, SHK_TAB_YO), 24).vertices,
        cylY(SHK_UX2, SHK_UZ2, 2.3, *ysp(sg, CG_YS, SHK_TAB_YO), 24).vertices,
        box(SHK_UX2 - 2.0, SHK_UX + 2.2, *ysp(sg, CG_YS, SHK_TAB_YO - 0.6), 29.2, 31.4).vertices]))
    _shk_plate_cheek = trimesh.convex.convex_hull(np.vstack([
        cylY(SHK_UX, SHK_UZ, 2.0, *ysp(sg, SHK_TAB_YO - 0.7, SHK_TAB_YO), 24).vertices,
        cylY(SHK_UX2, SHK_UZ2, 2.0, *ysp(sg, SHK_TAB_YO - 0.7, SHK_TAB_YO), 24).vertices]))
    # Yeti p46 rear tilt-open body hinge ear fused to rear hoop top corner (|Y|=15.7..18.6, bore Ø1.75 at X=-76.5, Z=29.0)
    _hinge_ear = trimesh.convex.convex_hull(np.vstack([
        cylY(HINGE_X, HINGE_Z, HINGE_R, *ysp(sg, HINGE_Y0, HINGE_Y1), 28).vertices,
        box(PXH - 1.1, PXH + 1.1, *ysp(sg, HINGE_Y0 - 0.8, HINGE_Y0 + 1.1), HINGE_Z - 2.3, CG_ZR + 1.0).vertices]))
    parts += [ctube((PXH, Yt, zlr(PXH)), (PXH, Yt, CG_ZR)),                                  # rear hoop leg
              ctube((PXH, Yt, zfb(PXH)), (FB_KX1, Yt, zfb(FB_KX1))), ctube((FB_KX1, Yt, zfb(FB_KX1)), (FB_KX0, sg*CG_YS, zfb(FB_KX0))), ctube((FB_KX0, sg*CG_YS, zfb(FB_KX0)), (CG_XM0, sg*CG_YS, zfb(CG_XM0))),      # fastback rail (owner markup): narrow from the tail hoop, one 45 deg kink outward, wide to the rear corner; every segment on the shell slope                       # fastback rail: rear hoop corner (hinge ear) -> tail hoop top corner, one straight tube
              ctube((PXH, Yt, CG_ZR), (FB_KX1, Yt, CG_ZR)), ctube((FB_KX1, Yt, CG_ZR), (FB_KX0, sg*CG_YS, CG_ZR)), ctube((FB_KX0, sg*CG_YS, CG_ZR), (CG_XTP, sg*CG_YS, CG_ZR)),      # shock rail (level): same plan as the fastback rail above it (narrow, one 45 deg kink, wide), so the top view shows one line per side                       # shock rail (level): rear hoop corner -> tail post, carries the shock plate
              ctube((BUMP_X, sg*CG_YS, zlr(BUMP_X)), (CG_XTP, sg*CG_YS, CG_ZTF)),                  # lower rail: rear hoop bottom bar -> tail foot, straight (1.0 mm inboard of the spring)
              ctube((CG_XTP, sg*CG_YS, CG_ZTF), (CG_XTP, sg*CG_YS, CG_ZR)), ctube((CG_XTP, sg*CG_YS, CG_ZTF), (CG_XTP, Yt + sg*0.4, CG_ZTF)),   # tail post (tail hoop leg) + foot link out to the lug screw
              _shk_plate_base, _shk_plate_cheek, _hinge_ear,
              ctube((CG_XTP, sg*CG_YS, CG_ZR), (TIE_X, Ym, CG_ZR)), ctube((TIE_X, Ym, CG_ZR), (CG_XM0, Ym, CG_ZR)),      # beltline tie: out from the tail post to |Y| 21.5 behind the ESC corner (2.6 mm off it), then straight to the B-post
              ctube((CG_XM0, Ym, CG_ZR), (CAB_X2, sg*A1_Y, A1_Z)),                                                     # beltline rail B-post -> A-post (owner side-view markup)
              ctube((CG_XM0, Ym, CG_ZF), (CG_XM0, Ym, CG_ZR)), ctube((CG_XM0, Ym, CG_ZR), (CG_XM0, sg*CG_YS, zfb(CG_XM0))),      # B-post: vertical to the beltline, then leaning in to the roof peak (inboard of the shell's rear rail)
              ctube((CAB_X2, sg*CAB_YC, CG_ZF), (CG_XM0, Ym, CG_ZR)), ctube((CG_XM0, Ym, CG_ZF), (CAB_X2, sg*A1_Y, A1_Z)),      # door bay X brace
              ctube((CG_XTP, sg*CG_YS, CG_ZR), (FB_KX0, sg*CG_YS, zfb(FB_KX0))),      # tail bay diagonal: tail post at the beltline -> fastback kink (the second leg of a V would lie on the fastback rail and lift it into the shell gap)
              ctube((BUMP_X, sg*BUMP_POST_Y, BUMP_POST_Z), (BUMP_X, sg*BUMP_POST_Y, RUNG_Z), e0=0.7),   # bump-stop post hanging from the rung (rounded foot)
              ctube((PXH, sg*(CG_YT - 1.4), BAR_Z), (PXH, sg*(CG_YT + 1.4), BAR_Z), s=3.6, e0=0.0, e1=0.0),      # Rev 41: sway-bar bearing saddle on the hoop leg: 3.6 square chamfered block, 2.8 long, D1.4 bore along Y                                       # bearing ball on the hoop leg for the sway-bar wire
              box(CG_XM0 - 1.5, CG_XM0 + 5.4, *ysp(sg, 18.5, CG_FOOT_Y1), 8.02, 9.3),     # B-post foot plate on the rim
              box(-37.9, -33.8, *ysp(sg, 14.2, 18.55), 16.32, 17.8)]                     # tail foot plate on the lug block
# Yeti AX80103 recessed radiator + dual-fan shroud framed inside rear hoop (Z=22.4..29.2, clear of battery X>=-74.9 and sway bar Z<=20.9; all walls >= 0.80 mm)
RAD_FAN_Y = 3.2      # fan centres (were +-5.8): both fans sit inboard of the hinge-screw notches
_rad_body = U([box(-78.8, -76.4, -13.2, 13.2, 22.4, 29.4), box(-78.8, -76.8, -15.0, 15.0, 24.5, 29.4)])
_rad_body = D(_rad_body, [box(-79.0, -77.6, -11.2, 11.2, 23.2, 28.4)])      # recessed fin core cavity (1.2 mm solid back wall)
_rad_details = [box(-78.2, -77.4, -11.4, 11.4, z_ - 0.40, z_ + 0.40) for z_ in (24.3, 25.8, 27.3)]      # horizontal radiator louver ribs (0.8 mm thick)
for _fy in (-RAD_FAN_Y, RAD_FAN_Y):
    _shroud = D(cylX(_fy, 25.8, 2.7, -78.85, -77.5, 24), [cylX(_fy, 25.8, 1.9, -79.0, -77.4, 24)])     # 0.8 mm ring wall
    _hub = cylX(_fy, 25.8, 0.90, -78.85, -77.5, 16)
    _sp1 = box(-78.6, -77.5, _fy - 2.3, _fy + 2.3, 25.8 - 0.40, 25.8 + 0.40)
    _sp2 = box(-78.6, -77.5, _fy - 0.40, _fy + 0.40, 25.8 - 2.3, 25.8 + 2.3)
    _rad_details += [_shroud, _hub, _sp1, _sp2]
RADIATOR = False and PXH < BATT_X0 - 2.0      # the radiator only fits when the rear hoop stands behind the battery pack; with the short tail the hoop is inside the pack's length
if RADIATOR:
    _rad = U([_rad_body] + _rad_details); _rad.apply_translation([TAIL_DX, 0, 0]); parts.append(_rad)
def _teardropY(x, z, r, y0, y1):
    c = cylY(x, z, r, y0, y1, 28)
    tip = [[x, y_, z - r*math.sqrt(2)] for y_ in (min(y0, y1), max(y0, y1))]
    return trimesh.convex.convex_hull(np.vstack([c.vertices, tip]))
CG_PAD_T = 0.9      # screw-3 head bore sat in the corner of the tail-post flare and left a 0.12 mm skin: a square pad with 0.9 mm wall round the bore (plate level up to the bore top) backs it up
CG_PAD_T2 = 0.5      # shock-facing quadrant (-X, outboard of the screw): no leg there, only the head bore to back up; the full 0.9 ring came to 0.945 mm of the rear shock spring at ta -2
def _cg_pad(x_, y_, zs_, zt_, sg):
    zb, zt2 = zt_ + 0.02, zs_ + M2_HEAD_H + 0.3
    big, small = M2_HEAD_R + 0.15 + CG_PAD_T, M2_HEAD_R + 0.15 + CG_PAD_T2
    _sm = cylZ(x_, sg*y_, small, zb, zt2, 40); _tip = [[x_ - big, sg*y_, zb], [x_ - big, sg*y_, zt2]]      # tangent flank from the full ring's tip to the thin ring, no step
    return U([trimesh.convex.convex_hull(np.vstack([_sm.vertices, _tip])),
              trimesh.boolean.intersection([cylZ(x_, sg*y_, big, zb, zt2, 40), box(x_, x_ + 4.0, *ysp(sg, y_ - 4.0, y_ + 4.0), zb, zt2)], engine="manifold"),
              trimesh.boolean.intersection([cylZ(x_, sg*y_, big, zb, zt2, 40), box(x_ - 4.0, x_ + 4.0, *ysp(sg, y_ - 4.0, y_), zb, zt2)], engine="manifold")])
CG_CUPS = [_cg_pad(x_, y_, zs_, zt_, sg) for sg in (1, -1) for k_, (x_, y_, zs_, zt_) in CG_SCREWS.items() if k_ == 3]
if os.environ.get('CAGE_NODES', '1') == '1': parts = fix_tubes(parts)
CAGE_NEW = D(U(parts + CG_CUPS), [cylY(PXH, BAR_Z, 0.7, sg*(YF - 3.0), sg*(YF + 3.0), 24) for sg in (1, -1)]
             + [_teardropY(HINGE_X, HINGE_Z, HINGE_BORE_D/2, sg*(HINGE_Y0 - 1.5), sg*(HINGE_Y1 + 0.5)) for sg in (1, -1)]
             + [cylY(SHK_UX, SHK_UZ, PIV_PILOT_D/2, sg*(CG_YS - 1.3), sg*(SHK_TAB_YO + 0.1), 24) for sg in (1, -1)]
             + [cylY(SHK_UX2, SHK_UZ2, PIV_PILOT_D/2, sg*(CG_YS - 1.3), sg*(SHK_TAB_YO + 0.1), 24) for sg in (1, -1)]
             + [cylZ(x_, sg*y_, M2_HEAD_R + 0.15, zs_ + 0.001, zs_ + M2_HEAD_H + 0.3, 24) for sg in (1, -1) for (x_, y_, zs_, zt_) in CG_SCREWS.values()] + [box(CG_SCREWS[1][0] - M2_HEAD_R - 0.15, CG_SCREWS[1][0] + M2_HEAD_R + 0.15, *ysp(sg, 19.0, 23.4), CG_SCREWS[1][2] + 0.001, CG_SCREWS[1][2] + M2_HEAD_H + 0.3) for sg in (1, -1)]      # B-post head pocket is a straight-walled box over the whole post tube (a round bore 0.5 mm off the tube axis crossed the chamfered tube at shallow angles: paper edges)
             + [cylZ(x_, sg*y_, M2_CLR_D/2, zs_ - 1.5, zs_ + 1.2, 24) for sg in (1, -1) for (x_, y_, zs_, zt_) in CG_SCREWS.values()])
# (Rev 49: no flat-top trim; the B-posts, roof-peak bar and rails rise above the beltline)
HINGE_NOTCH_Y0, HINGE_NOTCH_R, HINGE_NOTCH_ZF = 5.95, M2_HEAD_R + 0.15, 26.85      # hinge-screw insertion notch: |Y| it starts at (M2 x 8 fully withdrawn: head at 6.4..7.7), radius (head dia + 0.3), floor Z in the radiator
HINGE_POCKET_R, HINGE_CUP_T, HINGE_POCKET_L = M2_HEAD_R + 0.3, 0.9, 2.25      # screw-head pocket inboard of the ear (flat seat at |Y| 15.7) inside a cup with 0.9 mm walls: the bare pocket left 0.13..0.5 mm crusts of the hoop leg / top bar / radiator flange
CAGE_NEW = D(U([CAGE_NEW] + [cylY(HINGE_X, HINGE_Z, HINGE_POCKET_R + HINGE_CUP_T, *ysp(sg, HINGE_Y0 - HINGE_POCKET_L, HINGE_Y0 + 0.05), 32) for sg in (1, -1)]),
             [cylY(HINGE_X, HINGE_Z, HINGE_POCKET_R, *ysp(sg, HINGE_Y0 - HINGE_POCKET_L - 0.01, HINGE_Y0), 32) for sg in (1, -1)] + [_teardropY(HINGE_X, HINGE_Z, HINGE_BORE_D/2, sg*(HINGE_Y0 - 1.5), sg*(HINGE_Y1 + 0.5)) for sg in (1, -1)])
# Yeti AX80103 #1 rear light-bar bezel on rear hoop top cross-bar (4 recessed lamp buckets, clear of shell/battery, below arch top 36.2)
_lb_pts = [[-80.4, -13.5, 29.4], [-80.4, 13.5, 29.4], [-80.0, -13.0, 33.4], [-80.0, 13.0, 33.4],
           [-76.8, -13.5, 29.2], [-76.8, 13.5, 29.2], [-76.8, -13.0, 33.0], [-76.8, 13.0, 33.0]]
_lb_bar = trimesh.convex.convex_hull(np.array(_lb_pts, float))
_lb_x1 = lambda ly_: -78.6      # the outer lamp buckets are shallower: 0.85 mm between their floor and the hinge-screw notch
LB_LAMPS_Y = (-4.6, -1.6, 1.6, 4.6)      # all four lamps inboard of the hinge-screw notches
_lb_cuts = [cylX(ly_, 31.4, 1.15, -80.6, _lb_x1(ly_), 20) for ly_ in LB_LAMPS_Y]
_lb_emit = [cylX(ly_, 31.4, 0.5, _lb_x1(ly_) - 0.6, _lb_x1(ly_) + 0.2, 12) for ly_ in LB_LAMPS_Y]
_lb_bar = U([D(_lb_bar, _lb_cuts)] + _lb_emit); _lb_bar.apply_translation([TAIL_DX, 0, 0])
CG_ZA = 35.0      # Rev 42: roof cross tube arches over the raised gearbox (top Z 28.9): stubs from the rail ends up to Z 35.0 (0.8 below... shell roof there is >= 43)
LIGHT_BAR = False      # owner markup: the rear light bar block (and the radiator under it) is deleted; one straight cross tube joins the rear corners
if LIGHT_BAR: CAGE_NEW = U([CAGE_NEW, _lb_bar])      # Rev 48: the rear cage's own front arch is gone; the cabin's rear hoop (X 1.2) is the shared hoop
# hinge-screw insertion notch (owner decision): a quarter-round groove along the screw axis in the front-lower corner of the light bar / hoop top bar, and the radiator's top outer corners cut down to HINGE_NOTCH_ZF.
# Open to the front (+X): the M2 x 8 is laid in from the front at |Y| 6.4..15.7 and slid outboard into the ear; a ball-end driver reaches it at ~20 deg.
CAGE_NEW = D(CAGE_NEW, [cylY(HINGE_X, HINGE_Z, HINGE_NOTCH_R, *ysp(sg, HINGE_NOTCH_Y0, HINGE_Y0 - HINGE_POCKET_L + 0.05), 32) for sg in (1, -1)]
             + [box(HINGE_X - 2.5, HINGE_X + 0.2, *ysp(sg, HINGE_NOTCH_Y0, HINGE_Y0 - HINGE_POCKET_L + 0.05), HINGE_NOTCH_ZF, 29.2) for sg in (1, -1)])
CAGE_NEW = D(CAGE_NEW, [box(-200.0, SHELL_END_X, -40, 40, 17.9, 60.0)])      # the upper cage ends with the shell (X -67.0); only the low axle frame (lower rails, bump rung and posts, below Z 19) runs on to X -72.2
CAGE_NEW = D(CAGE_NEW, [TUB])      # seat trim: nothing of the cage extends into the tub
CAGE_NEW = D(CAGE_NEW, SPRING_KEEP_CAGE)      # Rev 39: 1.0 mm relief around the shock spring (trims the tail foot plate corner)
P["Cage"] = CAGE_NEW
# ---------------- Rev 42 + Yeti Y-380 / Rock-Bouncer / Desert-Runner: Cabin_Cage ----------------
CAB_ZB, CAB_ZFL = 18.5, 19.4                              # Yeti side beltline height / front hoop lower cross-bar height (1.7 mm clear above Servo_Ref)
RF_X, RF_Y, RF_Z = 3.0, 14.0, 38.6      # roof-front nodes: top of the windshield rails (roof is narrower than the body so the roof / fastback line stays inboard of the shell rails)
_cab = [ctube((RF_X, -RF_Y, RF_Z), (RF_X, RF_Y, RF_Z)), ctube((CAB_X2, -A1_Y, A1_Z), (CAB_X2, A1_Y, A1_Z)), ctube((CAB_XF, -CAB_YF, CAB_ZH), (CAB_XF, CAB_YF, CAB_ZH)),      # roof-front bar, windshield-base bar (beltline), front hoop top
        ctube((RF_X, RF_Y, RF_Z), (CG_XM0, -CG_YS, zfb(CG_XM0))), ctube((RF_X, -RF_Y, RF_Z), (CG_XM0, CG_YS, zfb(CG_XM0)))]      # roof bay X brace (plan view)
for sg in (1, -1):
    Yc, Yf = sg*CAB_YC, sg*CAB_YF
    _cab += [ctube((CAB_X2, Yc, CG_ZF), (CAB_X2, sg*A1_Y, A1_Z), e1=0.3),                                                            # A-post, foot to beltline
             ctube((CAB_X2, sg*A1_Y, A1_Z), (RF_X, sg*RF_Y, RF_Z)),                                                           # windshield rail
             ctube((RF_X, sg*RF_Y, RF_Z), (CG_XM0, sg*CG_YS, zfb(CG_XM0))),                                              # roof rail, runs straight into the fastback rail at the roof peak
             ctube((CAB_X2, sg*A1_Y, A1_Z), (CAB_XF, Yf, CAB_ZH)),                                                            # hood rail
             ctube((CAB_X2, Yc, CG_ZF), (CAB_XF, Yf, CAB_ZH)),                                                            # hood bay diagonal
             ctube((CAB_XF, Yf, CG_ZF), (CAB_XF, Yf, CAB_ZH)),                                                            # front hoop leg
             box(CAB_X2 - 1.6, CAB_X2 + 5.4, *ysp(sg, 18.95, CAB_FOOT_Y1), 8.02, 9.3),                                    # A-post foot plate
             box(CAB_XF - 2.6, 29.7, *ysp(sg, 14.15, 18.4), 8.02, 9.3), box(29.7 - 0.02, CAB_XF + 5.0, *ysp(sg, 14.45, 18.4), 8.02, 9.3)]      # inner edge 14.15 up to X 29.7 keeps a >= 0.5 mm web beside the screw-6 clearance hole; beyond it the tub ear wall needs 14.45                                           # front foot plate on the wall top
if os.environ.get('CAGE_NODES', '1') == '1': _cab = fix_tubes(_cab)
CABIN = U(_cab)
CABIN = D(CABIN, [box(-120, 200, -40, 40, 0, 8.0)])
CABIN = D(CABIN, [cylZ(x_, sg*y_, M2_HEAD_R + 0.15, zs_ + 0.001, zs_ + M2_HEAD_H + 0.3, 24) for sg in (1, -1) for k_, (x_, y_, zs_, zt_) in CAB_SCREWS.items()] + [cylZ(x_, sg*y_, M2_CLR_D/2, zs_ - 1.5, zs_ + 1.2, 24) for sg in (1, -1) for k_, (x_, y_, zs_, zt_) in CAB_SCREWS.items()])
CABIN = D(CABIN, [TUB])
P["Cage"] = single("Cage", U([P["Cage"], CABIN]))      # Rev 48: ONE cage (rear frame + cabin), one printed part, six screws per side
CAGE_PILOTS = [cylZ(x_, sg*y_, M2_PILOT_D/2, zt_ - 5.0, zt_ + 0.02, 24) for sg in (1, -1) for (x_, y_, zs_, zt_) in ALL_SCREWS.values()]
CAGE_T = P["Cage"]     # keep-out only, not exported
for sg, nm in ((1, "L"), (-1, "R")):
    _top = _spt(SHK_TOP_S, sg); _yi, _yo = SHK_YU - SHK_EYE_T/2, SHK_YU + SHK_EYE_T/2
    _eye = D(trimesh.convex.convex_hull(np.vstack([cylY(SHK_UX, SHK_UZ, SHK_EYE_R, *ysp(sg, _yi, _yo), 28).vertices, cylY(_top[0], _top[2], SHK_ROD_R + 0.4, *ysp(sg, _yi, _yo), 16).vertices])), [cylY(SHK_UX, SHK_UZ, SHK_BORE_R, *ysp(sg, SHK_YU - 2.5, SHK_YU + 2.5), 14)])
    P[f"Ref_Shock_Eye_{nm}"] = single(f"Ref_Shock_Eye_{nm}", _eye)      # eye wraps the shock top: zero overlap (clean seat)    # rod end: stem from the shock top to the pin eye
    for k_, (x_, y_, zs_, zt_) in ALL_SCREWS.items(): P[f"HW_Cage_Screw_{nm}{k_}"] = U([cylZ(x_, sg*y_, M2_HEAD_R, zs_, zs_ + M2_HEAD_H, 16), cylZ(x_, sg*y_, 1.0, zs_ - 6.0, zs_ + 0.1, 12)])      # M2 x 6 pan head on the foot plate: 4.7 mm (4.5 at the tail) in the tub pilot
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
for n_, sg_ in (("L", 1), ("R", -1)): P[f"HW_Sway_Link_Pin_{n_}"] = cylY(TAB_X, TAB_TOP_Z - LINK_INS, 0.6, *ysp(sg_, 18.1, 21.5), 16)      # D1.2 x 3.4 steel pin through the tab clevis (cheeks 1.2, slot 1.2) and the link eye, retained with CA
P["Ref_Rear_Sway_Bar"] = U([wcyl((BAR_X, -BAR_Y - 0.4, BAR_Z), (BAR_X, BAR_Y + 0.4, BAR_Z))] + [wcyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z)) for sg in (1, -1)] + [wcyl((BAR_E_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*LINK_Y, BAR_Z)) for sg in (1, -1)])
th0, E0, T0 = bar_state(0)
for n, sg in (("L", 1), ("R", -1)): P[f"Ref_Sway_Drop_Link_{n}"] = drop_link(E0, T0, sg)
BAR_TILT = []
for ta in (TA_D, -2, 0, 3, 6, TA_BUMP):
    th, E_, T_ = bar_state(ta); BAR_TILT.append((ta, math.degrees(th), math.degrees(math.atan2(E_[0] - T_[0], E_[1] - T_[1])), float(np.linalg.norm(E_ - T_))))
# servo envelope (front session): case on its side, ears, pilot wall
P["Ref_Servo_Envelope"] = U([box(13.4, 33.5, 1.8 + SERVO_DY, 11.7 + SERVO_DY, FLOOR_TOP, 10.8 + DELTA), box(32.3, 33.5, 11.7 + SERVO_DY, 14.65 + SERVO_DY, -3.8 + DELTA, 9.4 + DELTA)])      # follows the servo moved +0.15 in Y by the front script (coupler clearance 0.20)
# ---------------- cage / shell relief against the new bodies ----------------
# ---- printed retainers + their screws ----
# Rev 46: ESC/RX shelves, their 8 screws and the four pillars per side are removed (owner request); the ESC / RX stay in the side bays on the floor
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
for sg, nm in ((1, "L"), (-1, "R")): P[f"HW_Rear_Hinge_Screw_{nm}"] = U([cylY(HINGE_X, HINGE_Z, 1.0, *ysp(sg, HINGE_Y0 - 0.05, HINGE_Y0 + 8.0), 12), cylY(HINGE_X, HINGE_Z, M2_HEAD_R, *ysp(sg, HINGE_Y0 - M2_HEAD_H, HINGE_Y0), 16)])      # M2 x 8 from inboard, head on the ear's inner face, threads in the shell lug
SW_AX = U([posed(U([P["Rear_Axle_Top_Bracket"], P["HW_Rear_Axle_Top_Pin"]]), R.solve(t)[2]) for t in (TA_MIN, 0, TA_BUMP)])
SW_BAR = []
for t in (TA_MIN, -10, 0, TA_BUMP):
    th_, E_, T_ = bar_state(t)
    for sg in (1, -1): SW_BAR += [posed(lever(sg), R.roty(th_, BAR_X, BAR_Z)), drop_link(E_, T_, sg)]
SW_BAR = U(SW_BAR)
KEEPOUT = {"Axle_top_sweep": SW_AX, "Bar_sweep": SW_BAR, "Links_sweep": U([posed(P[f"Rear_Upper_Link_{n_}"], R.roty(R.solve(t)[1], PU[0], PU[1])) for t in (TA_MIN, 0, TA_BUMP) for n_ in "LR"]), "Chassis_Tub": P["Chassis_Tub"], "Ref_Battery_2S": BATT_BOX, "Ref_ESC_G40": ESC_BOX, "Ref_Gearbox": P["Ref_Gearbox"], "Ref_Motor_GM1614": P["Ref_Motor_GM1614"],
           "Cage": CAGE_T}
def dil(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)):   # 18 directions (axes + edge diagonals): thin parts left diagonal gaps with only 6
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
BULK = ld("IFS_Bulkhead_Convertible"); _v0 = P["Chassis_Tub"].volume
SW_DROOP = U([posed(R.M["Ref_Rear_Axle"], R.solve(t)[2]) for t in range(int(TA_MIN), 1, 2)] + [posed(P[f"Rear_Trailing_Arm_{n_}"], R.roty(R.solve(t)[0], PL[0], PL[1])) for t in range(int(TA_MIN), 1, 2) for n_ in "LR"])   # droop envelope of axle + arms: tub tail is relieved around it
NOTCH_C = float(os.environ.get("NOTCH_C", 0.3))
_tub0 = P["Chassis_Tub"]
_old = I(_tub0, dil(SW_DROOP, 0.2)) if vol(_tub0, dil(SW_DROOP, 0.2)) > 0.01 else trimesh.Trimesh()    # what the swept envelope removes (ragged); re-cut below as per-side convex wedges = a few clean planes
_wedges = []
if len(_old.faces):
    for sg_ in (1, -1):
        _part = I(_old, box(-200, 200, 0, sg_*40, -50, 50) if sg_ > 0 else box(-200, 200, -40, 0, -50, 50))
        if len(_part.faces) and len(_part.vertices) >= 4:
            _v = _part.vertices.copy(); _v[:, 1] = sg_*np.maximum(np.abs(_v[:, 1]), TUB_OUT + 0.05); _w = trimesh.convex.convex_hull(np.vstack([_v, _v + [-30.0, 0, 0]]))
            if _w.is_volume:
                _wedges.append(_w)     # extruded backwards: leaves the tail end face cleanly, no sheet slivers
                _t = I(_tub0, _w); print("wedge side", sg_, "removes %.1f mm3 (ragged cut removed %.1f)" % (_t.volume, _part.volume), "bounds", _t.bounds.round(1).tolist())
_bfs_plugs = [cylZ(bx, by, 1.75, NOSE_TOP - 0.05, 0.50, 20) for bx, by in BFS]     # seal obsolete bulkhead floor screw holes (<= Z 0.50, 0.35 mm below diff ring)
_unified = U([P["Chassis_Tub"], BULK] + _bfs_plugs)
P["Chassis_Tub"] = single("Chassis_Tub(unified)", D(_unified, _wedges) if _wedges else _unified); KEEPOUT["Chassis_Tub"] = P["Chassis_Tub"]
_rm = D(_tub0, [P["Chassis_Tub"]]); print("notch removed bounds", _rm.bounds.round(1).tolist() if len(_rm.faces) else None)
print("tub fused with bulkhead + diff drop: %.1f -> %.1f mm3" % (_v0, P["Chassis_Tub"].volume))
CUT_CLR = float(os.environ.get("CUT_CLR", 0.3)); DIL = {k: dil(m, CUT_CLR) for k, m in KEEPOUT.items()}
CLR_KEY = {"Ref_ESC_G40": 1.45}     # cage tubes keep >= 1.0 mm off the ESC and the ESC/RX shelves (was 0.2)
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
if os.path.exists(OUT + "_tmp.stl"): os.remove(OUT + "_tmp.stl")
for _f in os.listdir(OUT):      # Rev 46: drop the removed shelf lids and their screws from stl/v4
    if (_f.startswith(("ESC_RX_Shelf_", "HW_Shelf_Screw_", "Cabin_Cage")) or (_f.startswith("HW_Cage_Screw_") and int(_f[len("HW_Cage_Screw_") + 1:-4]) not in ALL_SCREWS)) and _f.endswith(".stl"): os.remove(OUT + _f); print("  removed obsolete", _f)
CAB_FOOT6_SEAT = (29.7, 31.05, 14.4, 14.95, 8.0, 9.35)      # the front script's ear wall (X 29.75..32.15, Y up to 14.9, Z up to 15.7) rose through the Cabin_Cage front-left foot plate: 0.72 mm3 overlap in the 17:49 build
SV_PILOT = dict(y=13.25, z=(13.7, 3.7), d=0.9, x=(29.93, 32.2))      # front script's M1.0 servo ear-screw pilots in the ear wall behind the tabs: my cheek (Y 12.9+, Z up to 8) had filled the lower one
P["Chassis_Tub"] = D(P["Chassis_Tub"], CAGE_PILOTS + [cylX(SV_PILOT['y'], z_, SV_PILOT['d']/2, SV_PILOT['x'][0], SV_PILOT['x'][1], 20) for z_ in SV_PILOT['z']] + [box(*CAB_FOOT6_SEAT)])      # screw pilots in the tub; seat notch for the Cabin_Cage front-left foot
# Volume-guarded manifold simplification on Chassis_Tub to collapse degenerate sliver triangles (<1e-6 mm2) before STL/STEP export
try:
    import manifold3d as _mf
    _mt = P["Chassis_Tub"]
    _mm = _mf.Manifold(_mf.Mesh(vert_properties=_mt.vertices.astype(np.float32), tri_verts=_mt.faces.astype(np.uint32)))
    _mo = _mm.simplify(0.002).to_mesh()
    _mq = single("Chassis_Tub(simp)", trimesh.Trimesh(np.array(_mo.vert_properties)[:, :3].astype(np.float64), np.array(_mo.tri_verts), process=False))
    if _mq.is_watertight and open_edges(_mq) == 0 and abs(_mq.volume - _mt.volume) < 0.002 * _mt.volume:
        print("  simplified Chassis_Tub: faces %d -> %d, deg(<1e-6) %d -> %d, dVol %.4f%%" % (len(_mt.faces), len(_mq.faces), (_mt.area_faces < 1e-6).sum(), (_mq.area_faces < 1e-6).sum(), (_mq.volume - _mt.volume)/_mt.volume*100))
        P["Chassis_Tub"] = _mq
except Exception as _e:
    print("  Chassis_Tub simplify skipped:", _e)
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
SW = {ta: hits_vol(ta) for ta in (TA_D, -2, 0, TA_BUMP)}
print("--- rest overlaps (mm3, only > 0.01) ---")
for p, v in sorted(SW[0].items(), key=lambda kv: -kv[1]):
    if not v < 0.01: print("  ", p, round(v, 3))
print("--- gearbox / motor / ESC / receiver / battery boxes vs everything (rest, >0.01 mm3):", [(p, round(v, 2)) for p, v in SW[0].items() if any(k in p[0] + p[1] for k in ("Ref_Gearbox", "Ref_Motor", "Ref_ESC", "Ref_Receiver", "Ref_Battery")) and not v < 0.01] or "none")
print("--- pairs that appear in the sweep but not at rest ---")
for ta in (TA_D, -2, TA_BUMP):
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
def dense_sweep(t0=None, t1=TA_BUMP):
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
            if n in ("Ref_Rear_Axle", "Bar_link_L", "Rear_Axle_Top_Bracket", "Rear_Upper_Link_L", "Rear_Upper_Link_R") and ta in (int(TA_MIN), -7, 0, TA_BUMP):
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
_shc = {_t: CollisionManager() for _t in range(int(TA_MIN), TA_BUMP + 1)}
for _t, _c in _shc.items():
    for _n in ("Ref_Shock_RL", "Ref_Shock_RR"): _x = FIX[_n].copy(); _x.apply_transform(shock_T(R.solve(_t)[0])); _c.add_object(_n, _x)
    for _n in ("Ref_Shock_Eye_L", "Ref_Shock_Eye_R"): _c.add_object(_n, P[_n])
_wc = {}
for _ta in range(int(TA_MIN), TA_BUMP + 1):
    for _n, (_m, _T) in scene(_ta).items():
        if _n.startswith("HW_") or not (_n in MOVE_ARM or _n in MOVE_UL or _n in AXG or _n.startswith(("Bar_lever", "Bar_link"))): continue
        _mm = _m.copy(); _mm.apply_transform(_T); _d, _nm = _cmc.min_distance_single(_mm, return_name=True)
        _d2, _nm2 = _shc[_ta].min_distance_single(_mm, return_name=True)      # moving shock (posed with the arm) vs this mover
        if _d2 < _d: _d, _nm = _d2, _nm2
        if _n not in _wc or _d < _wc[_n][0]: _wc[_n] = (_d, _nm, _ta)
print("--- rear cage / shock clearance of every mover over ta %d..+%d (nearest cage/shock body; mm) ---" % (TA_MIN, TA_BUMP))
for _n, (_d, _nm, _ta) in sorted(_wc.items(), key=lambda kv: kv[1][0]): print("  %-28s %-22s %6.2f at ta %3d%s" % (_n, _nm, _d, _ta, "  <-- under 1.5" if _d < 1.5 else ""))
for _ta in (0, 3, 5, 6, 7, TA_BUMP):
    _m = posed(R.M["Ref_Rear_Axle"], R.solve(_ta)[2]); _o = CollisionManager(); _o.add_object("ax", _m)
    _po = CollisionManager()
    for _sg in (1, -1): _po.add_object("post%d" % _sg, tube((BUMP_POST_X, _sg*BUMP_POST_Y, BUMP_POST_Z), (BUMP_POST_X, _sg*BUMP_POST_Y, BUMP_POST_Z + 3.0)))
    print("bump stop: axle vs rear cage (all) at ta %d = %.2f mm; vs the bump post foot only = %.2f mm" % (_ta, _cmc.min_distance_other(_o), _po.min_distance_other(_o)))
