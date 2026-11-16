"""v4: one printed TUB replaces Chassis_Rail_L/R + Slider_L/R; rear links pivot on the tub tail -> stl/v4/.
usage (repo root): BODY_DIR=<per-body stl dir> python scripts/18_tub.py
Reads stl/v3 (cage, shell, arms ...) and BODY_DIR (axle, wheels, driveshaft). Never writes stl/v3, stl/v2 or front parts.
Axes: X forward, Y left, Z up, mm.  All dimensions are named constants (edit and re-run)."""
import os, sys, math, itertools, numpy as np, trimesh
from scipy.optimize import brentq
sys.path.insert(0, "scripts"); import rearlib as R
BD = os.environ["BODY_DIR"]; V3 = "stl/v3/"; OUT = "stl/v4/"; os.makedirs(OUT, exist_ok=True)
def ld(n, src=V3): return trimesh.load(src + n + ".stl")
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
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
GBX_X, GBX_Y, GBX_Z0, GBX_Z1, GBX_SHAFT_Z = 11.5, 13.0, -4.9, 23.1, 4.1     # gearbox box, base at the tub floor top
MOTOR_R, MOTOR_L, MOTOR_Z = 8.5, 10.0, 18.1                                  # GM1614, motor on the REAR side (-X)
CLR = 0.4
# ================= tub =================
FLOOR_TOP = GBX_Z0; FLOOR_T = 2.6; FLOOR_BOT = FLOOR_TOP - FLOOR_T            # -7.5 skid line
TUB_OUT = 17.9                       # outer half width (arms start at 18.0; shell inner face is Y 22.3)
WALL = 1.5; GBX_IN = 13.5            # gearbox/front zone inner half width; tail inner = TUB_OUT-WALL
TAIL_IN = TUB_OUT - WALL
RIM_Z = 8.0                          # cage lower tubes start at Z 8.1 -> they sit on this rim
TUB_X0, TUB_X1 = -46.0, 36.0         # floor/walls end (floor ends before the axle housing, X>-47)
GBX_ZONE_X0 = -12.0                  # walls are 'gearbox zone' (inner 13.5) from here forward
# gearbox foot holes: GUESS (4 x M1.4 pilots, 20 x 20 mm square about the gearbox centre) -> measure and edit
FOOT_HOLES = [(-10.0, -11.0), (10.0, 11.0)]; FOOT_THRU_D = 1.7; FOOT_CB_D = 3.2; FOOT_CB_DEPTH = 1.0   # Modrax photo: 2 feet on the case bottom, diagonal (one per case half); M1.4 from below
# front bulkhead interface (positions read from the v3 rails: holes at (27, Z 2.5) and (33, Z 5.0)); screws along Y into the tub walls
BULK_X0 = 24.0; BULK_SCREWS = [(27.0, 2.5), (33.0, 5.0)]; BULK_PILOT_D = 1.2
BULK_WALL_Y0, BULK_WALL_Y1 = 11.5, 13.0       # bulkhead side walls (holes dia 1.4..1.6 measured): M1.6 x 5 self-tapping from outside
BULK_THRU_D, BULK_CB_D, BULK_CB_DEPTH = 2.0, 3.3, 1.4; BULK_SCR_R, BULK_HEAD_R = 0.8, 1.5
SERVO_BAY = dict(x0=12.0, x1=36.0, y0=2.0, y1=13.45, z0=FLOOR_TOP, z1=FLOOR_TOP + 16.0)   # EDITABLE keep-out/cavity (documented; tub is open here)
SHAFT_TUNNEL_D = 3.2                 # front shaft tunnel is in the bulkhead (unchanged, front session)
# electronics (ASSUMED sizes, edit freely)
ESC_L, ESC_W, ESC_H = 24.0, 18.0, 8.0        # 26 across Y, 18 along X, under the motor
RX_L, RX_W, RX_H = 22.0, 14.0, 7.0           # on the rear deck, 22 along X
BATT_L_SPEC, BATT_L, BATT_W, BATT_H = 44.5, 44.5, 20.0, 10.5   # 50 mm does NOT fit lengthwise anywhere (see TUB_V4.md); BATT_L is the longest that fits the deck
NOSE_L, NOSE_RISE_L, NOSE_W = 16.0, 7.0, 13.4; TAIL_KICK_L, TAIL_KICK_H = 7.0, 0.9
BAY_CLR = 0.4; LIP_T = 0.9; LIP_H = 3.0; CLIP_X = (-45.0, -62.0)
ESC_X1 = -(GBX_X + 0.9)                      # ESC front face (just behind the gearbox)
ESC_X0 = ESC_X1 - ESC_W - BAY_CLR
ESC_SINK = 2.0                               # floor pocket sink so ESC top (Z 2.1) is below the driveshaft
# rear deck (battery + receiver) over the axle housing, bridge sits on it
DECK_X1, DECK_X0 = -26.9, -81.5; DECK_Z0, DECK_Z1 = 14.7, 16.3   # raised so the axle-top bracket (top Z ~11 + 4 bump) clears
DECK_BOT_RIB = 0.0; BEAM_Y0 = 14.5; BEAM_Z0 = 11.2
BATT_X1 = -29.4; BATT_X0 = BATT_X1 - BATT_L - BAY_CLR; BATT_Y1 = (BATT_W + 0.8)/2; BATT_Y0 = -BATT_Y1     # 44.5 along X, 20 across Y, 10.5 tall, centred on the deck
RX_SINK = 2.0; RX_Z0 = FLOOR_TOP - RX_SINK; RX_X1 = -31.8; RX_X0 = RX_X1 - RX_W - BAY_CLR; RX_Y1 = (RX_L + BAY_CLR)/2; RX_Y0 = -RX_Y1             # receiver 22 across Y x 14 along X, in a floor pocket sunk 2 mm (top under the rear driveshaft)
# ================= rear 4-link on the tub tail =================
PL = np.array([float(os.environ.get("PL_X", -26.0)), -0.5])      # trailing arm pivot (X,Z)
PU = np.array([float(os.environ.get("PU_X", -24.53)), float(os.environ.get("PU_Z", 7.35))])       # upper link pivot
A_ = np.array([-59.0, -0.5]); BP = np.array([-57.5, 8.85])       # axle-side points (unchanged)
LUG_X = 3.2; LUG_R = 3.4; PAD_Y0, PAD_Y1 = 14.4, TAIL_IN          # pad inside the tail wall, in which the pivots are tapped
PIV_PILOT_D = 1.25; PIV_CLEAR_D = 1.6
ARM_Y0, ARM_Y1 = 18.0, 23.4          # trailing arm Y span (v3)
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
        c = trimesh.creation.cylinder(radius=1.5, height=4.0, sections=24); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
        c.apply_translation([s*(GBX_X + 2.0 - 0.01), 0, GBX_SHAFT_Z]); st.append(c)
    return U([b] + st)
def motor_ref():
    c = trimesh.creation.cylinder(radius=MOTOR_R, height=MOTOR_L, sections=48); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    c.apply_translation([-(GBX_X + MOTOR_L/2), 0, MOTOR_Z]); return c
ESC_BOX = box(ESC_X0 + BAY_CLR/2, ESC_X1 - BAY_CLR/2, -ESC_L/2, ESC_L/2, FLOOR_TOP - ESC_SINK, FLOOR_TOP - ESC_SINK + ESC_H)
BATT_BOX = box(BATT_X0 + BAY_CLR/2, BATT_X1 - BAY_CLR/2, BATT_Y0 + 0.4, BATT_Y1 - 0.4, DECK_Z1, DECK_Z1 + BATT_H)
RX_BOX = box(RX_X0 + BAY_CLR/2, RX_X1 - BAY_CLR/2, RX_Y0 + BAY_CLR/2, RX_Y1 - BAY_CLR/2, RX_Z0, RX_Z0 + RX_H)
P["Ref_Gearbox"] = gbx_ref(); P["Ref_Motor_GM1614"] = motor_ref()
P["Ref_ESC_G40"] = ESC_BOX; P["Ref_Receiver"] = RX_BOX; P["Ref_Battery_2S"] = BATT_BOX

# ---------------- the tub ----------------
def build_tub():
    parts = [box(TUB_X0, TUB_X1, -TUB_OUT, TUB_OUT, FLOOR_BOT, FLOOR_TOP), box(ESC_X0 - 0.5, ESC_X1 + 0.5, -ESC_L/2 - 1.5, ESC_L/2 + 1.5, FLOOR_BOT - 1.0, FLOOR_BOT + 0.01)]   # local skid pad under the sunk ESC pocket
    for s in (1, -1):
        def yy(a, b): return (min(s*a, s*b), max(s*a, s*b))
        # gearbox/front zone walls
        y0, y1 = yy(GBX_IN, TUB_OUT); parts.append(box(GBX_ZONE_X0, TUB_X1, y0, y1, FLOOR_TOP - 0.01, RIM_Z))
        # tail walls (thin), raised to the deck behind the motor
        y0, y1 = yy(TAIL_IN, TUB_OUT)
        parts.append(box(TUB_X0, GBX_ZONE_X0 + 0.01, y0, y1, FLOOR_TOP - 0.01, RIM_Z))
        parts.append(box(TUB_X0, DECK_X1, y0, y1, RIM_Z - 0.01, DECK_Z1))
        # pivot pads inside the tail wall (trailing lug + upper-link lug)
        y0, y1 = yy(PAD_Y0, TAIL_IN + 0.01)
        parts.append(box(PL[0] - LUG_X - 1.0, PU[0] + LUG_X + 0.5, y0, y1, FLOOR_TOP - 0.01, PU[1] + 2.6))   # corner link block: lower pivot low/outer, upper pivot above it
        # step wall between gearbox zone and tail (closes the cavity behind the gearbox, keeps motor/shaft open above)
    parts.append(box(DECK_X0, DECK_X1, -TUB_OUT, TUB_OUT, DECK_Z0, DECK_Z1))             # deck
    for s_ in (1, -1):
        parts.append(box(BULK_X0 - 1.0, BULK_X0 - 0.1, min(s_*11.9, s_*GBX_IN + s_*0.3), max(s_*11.9, s_*GBX_IN + s_*0.3), FLOOR_TOP - 0.01, -1.0))       # bulkhead rear stop (X) beside each wall; floor is the Z seat
        parts.append(box(DECK_X0, -50.0, min(s_*BEAM_Y0, s_*TUB_OUT), max(s_*BEAM_Y0, s_*TUB_OUT), BEAM_Z0, DECK_Z0 + 0.01))       # side beam under the extended deck (cage feet screw into it)
    parts.append(box(TUB_X0, RX_X1 + 0.5, RX_Y0 - 1.0, RX_Y1 + 1.0, FLOOR_BOT - 1.0, FLOOR_BOT + 0.01))   # local skid pad under the sunk receiver pocket
    # skid nose under the bulkhead: sloped kick-up (profile in XZ, extruded across Y)
    nose = trimesh.convex.convex_hull(np.array([[x, y, z] for (x, z) in ((TUB_X1 - 0.5, FLOOR_BOT), (TUB_X1 + NOSE_L - NOSE_RISE_L, FLOOR_BOT), (TUB_X1 + NOSE_L, FLOOR_TOP - 0.4), (TUB_X1 + NOSE_L, FLOOR_TOP), (TUB_X1 - 0.5, FLOOR_TOP)) for y in (-NOSE_W, NOSE_W)]))
    parts.append(nose)
    m = U(parts)
    cuts = []
    # tail kick-up: wedge under the floor end (keeps >= 1.7 mm floor)
    cuts.append(trimesh.convex.convex_hull(np.array([[x, y, z] for (x, z) in ((TUB_X0 - 0.1, FLOOR_BOT - 0.1), (TUB_X0 + TAIL_KICK_L, FLOOR_BOT - 0.1), (TUB_X0 - 0.1, FLOOR_BOT + TAIL_KICK_H)) for y in (-TUB_OUT - 1, TUB_OUT + 1)])))
    # ESC floor pocket (sunk) + ESC wire notch through the gearbox-side wall of the cavity
    cuts.append(box(TUB_X0 - 0.5, RX_X1, RX_Y0, RX_Y1, RX_Z0, FLOOR_TOP + 0.01))   # receiver floor pocket, open at the rear (slides in from behind; 1.0 wall to the ESC pocket)
    cuts.append(box(ESC_X0, ESC_X1, -ESC_L/2 - BAY_CLR/2, ESC_L/2 + BAY_CLR/2, FLOOR_TOP - ESC_SINK, FLOOR_TOP + 0.01))
    # gearbox foot pilots (blind, from the top)
    for fx, fy in FOOT_HOLES: cuts += [cylZ(fx, fy, FOOT_THRU_D/2, FLOOR_BOT - 0.01, FLOOR_TOP + 0.01, 16), cylZ(fx, fy, FOOT_CB_D/2, FLOOR_BOT - 0.01, FLOOR_BOT + FOOT_CB_DEPTH, 20)]
    # bulkhead pilots through each front wall (axis Y, from the outside)
    for s in (1, -1):
        for bx, bz in BULK_SCREWS:
            if s == 1 and bx == 33.0: continue      # +Y (33, 5) is the servo-ear position (ears X 32.3..33.5, Y 11.7..14.65): no bulkhead screw there
            cuts += [cylY(bx, bz, BULK_THRU_D/2, s*(GBX_IN - 0.3), s*(TUB_OUT + 0.01), 20), cylY(bx, bz, BULK_CB_D/2, s*(TUB_OUT - BULK_CB_DEPTH), s*(TUB_OUT + 0.01), 24)]   # through the whole wall + head counterbore (screws thread into the bulkhead)
        # trailing arm pivot pilot (from outside), upper link pivot pilot (from inside the pad -> through wall? both blind from outside)
        cuts.append(cylY(PL[0], PL[1], PIV_PILOT_D/2, s*(PAD_Y0 + 0.6), s*(TUB_OUT + 0.01), 16))
        cuts.append(cylY(PU[0], PU[1], PIV_CLEAR_D/2, s*(PAD_Y0 - 0.01), s*(TUB_OUT + 0.01), 16))
        # deck bays: pack and receiver are retained by lips (below), wire slots through the deck
    for xs, ys in ((-30.0, -2.0), (-30.0, 2.0)): cuts.append(box(xs - 1.2, xs + 1.2, ys - 0.7, ys + 0.7, DECK_Z0 - 0.01, DECK_Z1 + 0.01))
    cuts.append(box(BATT_X1 - 3.6, BATT_X1, BATT_Y1 + 1.0, BATT_Y1 + 3.2, DECK_Z0 - 0.01, DECK_Z1 + 0.01))      # battery lead slot (deck, beside the pack's front-right corner)
    m = D(m, cuts)
    # retaining lips on the deck
    lips = []
    def frame(x0, x1, y0, y1):
        t = LIP_T; z0, z1 = DECK_Z1 - 0.01, DECK_Z1 + LIP_H
        return [box(x0 - t, x1 + t, y0 - t, y0, z0, z1), box(x0 - t, x1 + t, y1, y1 + t, z0, z1), box(x0 - t, x0, y0, y1, z0, z1), box(x1, x1 + t, y0, y1, z0, z1)]
    for fr in (frame(BATT_X0, BATT_X1, BATT_Y0, BATT_Y1),):
        for f in fr: lips.append(f)
    m = U([m] + lips)
    # lip wire notches (3 mm) on the inner (rear) end of each bay
    m = D(m, [box(BATT_X1 - 3.8, BATT_X1 - 0.2, BATT_Y1 - .01, BATT_Y1 + LIP_T + .01, DECK_Z1 + 0.6, DECK_Z1 + LIP_H + .1)])   # lead notch in the side lip at the front-right corner
    for cx in CLIP_X:     # snap clips: 0.9 thick flex arms on both long lips, hook 0.4 mm over the pack top edge
        for s_ in (1, -1):
            yy_ = lambda a_, b_: (min(s_*a_, s_*b_), max(s_*a_, s_*b_))
            ya, yb = yy_(BATT_Y1, BATT_Y1 + LIP_T); m = U([m, box(cx - 2.5, cx + 2.5, ya, yb, DECK_Z1 + LIP_H - 0.01, DECK_Z1 + BATT_H + 0.1)])
            ya, yb = yy_(BATT_Y1 - 0.4 - 0.4, BATT_Y1 + LIP_T); m = U([m, box(cx - 2.5, cx + 2.5, ya, yb, DECK_Z1 + BATT_H + 0.05, DECK_Z1 + BATT_H + 0.45)])
    return m
CAGE = ld("Roll_Cage"); SHELL = ld("Tacoma_Trophy_Body_Shell")
def feet_columns():
    v = CAGE.vertices; low = v[(v[:, 2] < CAGE.bounds[0][2] + 0.3) & (v[:, 0] > TUB_X0 + 1) & (v[:, 0] < GBX_ZONE_X0)]
    cols = []
    if len(low) == 0: return cols
    from scipy.cluster.hierarchy import fcluster, linkage
    c = fcluster(linkage(low[:, :2], 'single'), 2.5, 'distance')
    for k in set(c):
        q = low[c == k]; x0, x1 = q[:, 0].min(), q[:, 0].max(); ay = np.abs(q[:, 1]); y0, y1 = ay.min(), ay.max(); s = 1 if q[:, 1].mean() > 0 else -1
        if y1 < TAIL_IN and y1 > PAD_Y0:   # foot floats inside the tail cavity: column from the floor to the wall, never inboard of the pad face
            lo, hi = sorted([s*max(y0, PAD_Y0), s*(TAIL_IN + 0.01)])
            cols.append(box(x0, x1, lo, hi, FLOOR_TOP - 0.01, CAGE.bounds[0][2] + 0.05))
    return cols
FEET = feet_columns()
def single(name, m, vmin=1.0):
    sh = sorted(m.split(only_watertight=False), key=lambda q: -abs(q.volume))
    for q in sh[1:]:
        if abs(q.volume) > vmin: print("  dropped fragment of", name, "vol %.1f" % abs(q.volume), "bounds", q.bounds.round(1).tolist())
    return sh[0]
TUB = build_tub(); EARS = box(32.3, 33.5, 11.7, 14.65, -3.8, 9.4); TUB = D(TUB, [box(31.9, 33.9, 11.3, 15.05, -4.2, 9.8)]); TUB = U([TUB, box(33.65, 36.1, 11.85, 13.6, FLOOR_TOP - 0.01, 9.4)] + FEET); 
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
TUB = single('Chassis_Tub', TUB); P["Chassis_Tub"] = TUB; print("cage feet columns added:", len(FEET))

# ---------------- kinematics (own copy of the rearlib maths, any pivots) ----------------
def kin(ta):
    a, u, T = R.solve(ta); return a, u, T
rows = []
def kin_table():
    out = []
    L_arm = np.linalg.norm(A_ - PL); L_link = np.linalg.norm(BP - PU)
    T0 = kin(0)[2]
    shock_lo0 = np.array([-44.0, 0.5]); shock_up = np.array([-26.0, 21.5])
    G = np.array([-15.5, GBX_SHAFT_Z]); Q = np.array([-46.8, 3.5])      # driveshaft: gearbox stub end -> axle input (U-joint centre)
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
    zg = -16.5; x_r = -59.0; wb = 48.0 - x_r; hcg = 21.5 - zg   # CG height above ground ~21.5 (CG Z 5.0), wheelbase 107 (assumed)
    slope = (ic[1] - zg)/(x_r - ic[0]) if ic[0] != x_r else float('inf')
    return ic, 100*abs(slope)/(hcg/wb), L_arm_len()
def L_arm_len(): return float(np.linalg.norm(A_ - PL)), float(np.linalg.norm(BP - PU))
IC, AS, LEN = ic_antisquat()

# ---------------- trailing arm / upper link rebuilt for the tub-tail pivots ----------------
AXL_END = None
def _axl():
    ax = I(R.M['Ref_Rear_Axle'], box(-70, -45, -40, 40, -10, 10)); return dil_(ax, 0.1)
def dil_(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]):
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
AXL_END = _axl()
def arm(side):
    old = ld("Rear_Trailing_Arm_L"); keep = I(old, box(-70, PRIDE_ARM_CUT, -40, 40, -20, 20))     # keep the rear part (axle end, shock clevis, tab)
    ya0, ya1 = ARM_Y0, ARM_Y1
    boss = cylY(PL[0], PL[1], LUG_R, ya0, ya1, 40); neck = box(PL[0], PRIDE_ARM_CUT + 0.3, ya0, ya1, PL[1] - 1.6, PL[1] + 1.6)
    m = U([keep, boss, neck]); m = D(m, [cylY(PL[0], PL[1], PIV_CLEAR_D/2, ya0 - .1, ya1 + .1, 20), AXL_END])
    return m if side > 0 else mir(m)
def link(side):      # diagonal upper link: corner pivot (Y axis) -> shared axle-top bracket (vertical pin at Y 0); L eye above R eye (0.1 gap)
    zc = BP[1] + (0.6 if side > 0 else -0.6)
    bs = cylY(PU[0], PU[1], 1.75, side*(UL_YB - 1.35), side*(UL_YB + 1.35), 28)
    eye = cylZ(BP[0], 0.0, 1.75, zc - BR_EYE_T/2, zc + BR_EYE_T/2, 28)
    blade = trimesh.convex.convex_hull(np.vstack([bs.vertices, eye.vertices]))      # flat tapered blade, thin at the eye
    m = U([blade, bs, eye]); m = D(m, [cylY(PU[0], PU[1], PIV_PILOT_D/2, side*(UL_YB - 2), side*(UL_YB + 2), 20), cylZ(BP[0], 0.0, 0.8, zc - 2, zc + 2, 16)])
    xcut = PU[0] - 6.0
    if side > 0: m = D(m, [box(-80, xcut, -30, 30, -5, BP[1] + 0.05)])
    else: m = D(m, [box(-80, xcut, -30, 30, BP[1] - 0.05, 20), box(-80, xcut, -30, 30, -5, BP[1] - 0.6 - BR_EYE_T/2)])
    return m
for sg, n in ((1, "L"), (-1, "R")):
    P[f"Rear_Trailing_Arm_{n}"] = arm(sg); P[f"Rear_Upper_Link_{n}"] = link(sg)
    # hardware: M1.4 pivot screws (tapped into the tub pads, no nuts), screw heads outside the arm / inside the link
    sh = U([cylY(PL[0], PL[1], 0.7, sg*(PAD_Y0 + 0.8), sg*(ARM_Y1), 12), cylY(PL[0], PL[1], 1.35, sg*ARM_Y1, sg*(ARM_Y1 + 0.9), 16)])
    P[f"HW_Rear_Trailing_Pivot_Screw_{n}"] = sh
    sh2 = U([cylY(PU[0], PU[1], 0.7, sg*(UL_YB - 0.95), sg*(TUB_OUT + 0.05), 12), cylY(PU[0], PU[1], 1.35, sg*TUB_OUT, sg*(TUB_OUT + 0.9), 16)])
    P[f"HW_Rear_Upper_Link_Pivot_Screw_{n}"] = sh2
# unchanged v3 parts we keep using (arms' hardware at the axle, shocks, axle-side link HW)
KEEP = ["HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R",
        "Ref_Shock_RL", "Ref_Shock_RR", "HW_Rear_Shock_Lower_Screw_L", "HW_Rear_Shock_Lower_Screw_R", "HW_Rear_Shock_Upper_Screw_L", "HW_Rear_Shock_Upper_Screw_R",
        "Ref_Rear_Driveshaft"]
for k in KEEP: P[k] = ld(k)
# ---- axle-top bracket (printed, sits on the pumpkin) + vertical pin; links ride on the pin ----
BR_PLATE_Z0, BR_PLATE_Z1 = 7.0, BP[1] - 0.6 - BR_EYE_T/2 - 0.2      # plate sits on the two axle-top ridges (|Y| 1.7..3+, Z 7.0), 0.1 under the lower eye
BR_KEY = (BP[0] - 1.0, BP[0] + 1.0, -1.55, 1.55, 3.2, BR_PLATE_Z0)      # key drops into the 3.5 mm groove between the ridges (locates in Y, pin goes through it)
P["Rear_Axle_Top_Bracket"] = U([box(BP[0] - 3.4, BP[0] + 1.9, -4.5, 4.5, BR_PLATE_Z0, BR_PLATE_Z1), box(*BR_KEY[:4], BR_KEY[4], BR_KEY[5] + 0.01)])
P["Rear_Axle_Top_Bracket"] = D(P["Rear_Axle_Top_Bracket"], [cylZ(BP[0], 0.0, 0.6, 3.0, BR_PLATE_Z1 + 0.5, 14)])
_pt = BP[1] + 0.6 + BR_EYE_T/2
P["HW_Rear_Axle_Top_Pin"] = U([cylZ(BP[0], 0.0, 0.7, 3.4, _pt + 0.4, 14), cylZ(BP[0], 0.0, 1.3, _pt, _pt + 0.4, 16)])
# ---- rear cage (tail hoop) + sway bar ----
TUBE_R = 1.15                                         # measured on Roll_Cage (section 2.3 x 2.5)
PXH = -77.5; HOOP_Y = 15.5; HOOP_TOP = DECK_Z1 + 8.5             # hoop moved behind the (longer) pack
BAR_X, BAR_Z, BAR_Y, BAR_E_X = PXH, DECK_Z1 + 4.2, 19.75, -46.75
TAB_X, TAB_TOP_Z, LINK_INS = -46.75, 4.9, 0.8; WIRE_R, LINK_R, TUN_R = 0.4, 0.45, 0.9
POST_X = -62.0; FOOT_SCREW_D = 0.6
XA, YF = PXH + 6.0, 16.4                     # hoop joint plate centre (X) and frame line (Y)
XN, YN, ZN = -54.0, 16.9, 20.7               # frame end over the main-cage lug (horizontal lap, vertical screws; clear of the sway levers at Y 19.35+)
JT_Z0, JT_Z1 = DECK_Z1 + 0.4, DECK_Z1 + 8.7 # hoop joint plate Z
HJ_SCREWS = [(XA - 1.2, 19.5), (XA + 1.2, 23.0)]; NJ_SCREWS = [(-50.6, 16.9), (-48.4, 16.9)]
def tube(p0, p1, r=TUBE_R):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    ms = [trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1]), sections=20)]
    for q in (p0, p1): ms.append(trimesh.creation.icosphere(subdivisions=2, radius=r*1.03)); ms[-1].apply_translation(q)
    return U(ms)
def ysp(sg, a_, b_): return (min(sg*a_, sg*b_), max(sg*a_, sg*b_))
TOPT = np.array([XA, YF, HOOP_TOP - 0.5]); ENDN = np.array([XN, YN, ZN])
def stay_pt(sg, x):
    t = (x - TOPT[0])/(ENDN[0] - TOPT[0]); q = TOPT + (ENDN - TOPT)*t; return np.array([q[0], sg*q[1], q[2]])
# ---- piece 1: hoop (legs, top bar, collars, cross member, bumper bar, rear stays, feet, joint tabs for the stay frames)
parts = []
for sg in (1, -1):
    parts.append(tube((PXH, sg*HOOP_Y, DECK_Z1 + 1.3), (PXH, sg*HOOP_Y, HOOP_TOP)))
    parts.append(box(PXH - 3.5, PXH + 1.4, *ysp(sg, 13.2, 17.8), DECK_Z1, DECK_Z1 + 1.2))                                      # hoop foot pad on the deck
    parts.append(cylZ(PXH, sg*HOOP_Y, 2.4, BAR_Z - 2.4, BAR_Z + 2.4, 28))                                                      # bar collar on the leg
    parts.append(box(PXH + 0.5, XA + 2.5, *ysp(sg, 13.7, 15.2), JT_Z0, JT_Z1))                                                 # joint tab (stay frame screws on from outside)
parts.append(tube((PXH, -HOOP_Y, HOOP_TOP), (PXH, HOOP_Y, HOOP_TOP)))
parts.append(tube((PXH, -HOOP_Y, BAR_Z - 2.0), (PXH, HOOP_Y, BAR_Z - 2.0)))
parts.append(box(PXH - 1.8, PXH + 1.8, -2.0, 2.0, BAR_Z - 3.0, BAR_Z + 1.6))
BUMP_X, BUMP_Z = PXH - 6.0, DECK_Z1 + 4.0
for sg in (1, -1):
    parts.append(tube((PXH, sg*HOOP_Y, BAR_Z + 2.4), (PXH, 0.0, HOOP_TOP)))
    parts.append(tube((PXH, sg*HOOP_Y, HOOP_TOP - 0.5), (BUMP_X, sg*HOOP_Y, BUMP_Z)))
parts.append(tube((PXH, -HOOP_Y, DECK_Z1 + 1.25), (PXH, HOOP_Y, DECK_Z1 + 1.25)))
parts.append(tube((BUMP_X, -HOOP_Y, BUMP_Z), (BUMP_X, HOOP_Y, BUMP_Z)))
HOOP = U(parts)
HOOP = D(HOOP, [cylY(PXH, BAR_Z, TUN_R, -HOOP_Y - 4, HOOP_Y + 4, 24)] + [cylZ(PXH - 2.6, sg*HOOP_Y, 0.8, DECK_Z1 - 0.5, DECK_Z1 + 1.3, 12) for sg in (1, -1)]
            + [cylY(x_, z_, 0.6, sg*13.8, sg*15.9, 12) for sg in (1, -1) for x_, z_ in HJ_SCREWS])
P["Rear_Cage_Hoop"] = HOOP
# ---- pieces 2/3: side stay frames (hoop joint plate -> stay -> main-cage lug plate; post + deck rail + diagonal; screw to the deck beams)
for n, sg in (("L", 1), ("R", -1)):
    S = stay_pt(sg, POST_X); YP = S[1]; fr = []
    fr.append(box(XA - 2.5, XA + 2.5, *ysp(sg, 15.8, 17.0), JT_Z0 - 0.0, JT_Z1))                                              # hoop joint plate (outer)
    fr.append(tube((XA, sg*YF, HOOP_TOP - 0.5), (XN, sg*YN, ZN)))                                                              # forward stay
    fr.append(tube((POST_X, YP, DECK_Z1 + 1.25), (POST_X, YP, S[2])))                                                          # post
    fr.append(box(POST_X - 2.0, POST_X + 4.2, *sorted((sg*(abs(YP) - 2.2), sg*(abs(YP) + 1.7))), DECK_Z1, DECK_Z1 + 1.2))      # post foot pad
    fr.append(tube((XA, sg*YF, DECK_Z1 + 1.25), (POST_X, YP, DECK_Z1 + 1.25)))                                                 # deck rail
    fr.append(tube((XA, sg*YF, HOOP_TOP - 0.5), (POST_X, YP, DECK_Z1 + 1.25)))                                                 # diagonal
    fr.append(box(-54.0, -46.8, *ysp(sg, 15.6, 18.2), 20.1, 21.3))                                                            # main-cage joint plate (lies on the cage lug)
    m = U(fr)
    m = D(m, [cylZ(POST_X + 2.8, YP, 0.8, DECK_Z1 - 0.5, DECK_Z1 + 1.3, 12)] + [cylY(x_, z_, 0.8, sg*15.5, sg*17.5, 14) for x_, z_ in HJ_SCREWS] + [cylZ(x_, sg*y_, 0.8, 19.9, 21.5, 14) for x_, y_ in NJ_SCREWS])
    P[f"Rear_Cage_Stay_{n}"] = m
# lugs welded to the main Roll_Cage rear nodes (applied after the cage relief below)
CAGE_LUGS = [box(-52.4, -44.4, *ysp(sg, 15.4, 18.4), 17.8, 20.0) for sg in (1, -1)]; CAGE_CUT_X = -46.2    # main cage is trimmed to X >= -46.2 (thin leftovers of the old rear ring removed) and gets these lugs
CAGE_T = U([P["Rear_Cage_Hoop"], P["Rear_Cage_Stay_L"], P["Rear_Cage_Stay_R"]])     # keep-out only, not exported
FOOT_SCREWS = {"L": (1, PXH - 2.6, HOOP_Y), "R": (-1, PXH - 2.6, HOOP_Y), "Post_L": (1, POST_X + 2.8, abs(stay_pt(1, POST_X)[1])), "Post_R": (-1, POST_X + 2.8, abs(stay_pt(-1, POST_X)[1]))}
for n, (sg, fx, fy) in FOOT_SCREWS.items():
    P[f"HW_Rear_Cage_Screw_{n}"] = U([cylZ(fx, sg*fy, 0.7, DECK_Z1 - 3.5, DECK_Z1 + 1.3, 12), cylZ(fx, sg*fy, 1.3, DECK_Z1 + 1.2, DECK_Z1 + 1.9, 16)])
P["Chassis_Tub"] = D(P["Chassis_Tub"], [cylZ(fx, sg*fy, FOOT_SCREW_D, DECK_Z1 - 3.4, DECK_Z1 + 0.01, 12) for (sg, fx, fy) in FOOT_SCREWS.values()])
for sg, nm in ((1, "L"), (-1, "R")):
    for k_, (x_, z_) in enumerate(HJ_SCREWS, 1): P[f"HW_Rear_Cage_Joint_Hoop_{nm}{k_}"] = U([cylY(x_, z_, 1.3, sg*17.0, sg*18.0, 16), cylY(x_, z_, 0.7, sg*14.3, sg*17.1, 12)])
    for k_, (x_, y_) in enumerate(NJ_SCREWS, 1): P[f"HW_Rear_Cage_Joint_Node_{nm}{k_}"] = U([cylZ(x_, sg*y_, 1.3, 21.3, 22.3, 16), cylZ(x_, sg*y_, 0.7, 18.6, 21.4, 12)])
LR = BAR_E_X - BAR_X                                   # lever length
CROSS = U([tube((BAR_X, -BAR_Y, BAR_Z), (BAR_X, BAR_Y, BAR_Z), WIRE_R)])
def lever(sg): return tube((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R)
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
    return tube((Epos[0], sg*BAR_Y, Epos[1]), (T[0], sg*BAR_Y, T[1]), LINK_R)
def wcyl(p0, p1): return trimesh.creation.cylinder(radius=WIRE_R, segment=np.array([p0, p1], float), sections=20)
P["Ref_Rear_Sway_Bar"] = U([wcyl((BAR_X, -BAR_Y - 0.4, BAR_Z), (BAR_X, BAR_Y + 0.4, BAR_Z))] + [wcyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z)) for sg in (1, -1)])
th0, E0, T0 = bar_state(0)
for n, sg in (("L", 1), ("R", -1)): P[f"Ref_Sway_Drop_Link_{n}"] = drop_link(E0, T0, sg)
BAR_TILT = []
for ta in (-14, -10, -5, 0, 3, 6, 7):
    th, E_, T_ = bar_state(ta); BAR_TILT.append((ta, math.degrees(th), math.degrees(math.atan2(E_[0] - T_[0], E_[1] - T_[1])), float(np.linalg.norm(E_ - T_))))
# servo envelope (front session): case on its side, ears, pilot wall
P["Ref_Servo_Envelope"] = U([box(13.4, 33.5, 1.8, 11.7, FLOOR_TOP, 10.8), box(32.3, 33.5, 11.7, 14.65, -3.8, 9.4)])
# ---------------- cage / shell relief against the new bodies ----------------
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
SW_AX = U([posed(U([P["Rear_Axle_Top_Bracket"], P["HW_Rear_Axle_Top_Pin"]]), R.solve(t)[2]) for t in (-14, 0, 7)])
SW_BAR = []
for t in (-14, -10, 0, 7):
    th_, E_, T_ = bar_state(t)
    for sg in (1, -1): SW_BAR += [posed(lever(sg), R.roty(th_, BAR_X, BAR_Z)), drop_link(E_, T_, sg)]
SW_BAR = U(SW_BAR)
KEEPOUT = {"Axle_top_sweep": SW_AX, "Bar_sweep": SW_BAR, "Links_sweep": U([posed(P[f"Rear_Upper_Link_{n_}"], R.roty(R.solve(t)[1], PU[0], PU[1])) for t in (-14, 0, 7) for n_ in "LR"]), "Chassis_Tub": P["Chassis_Tub"], "Ref_Battery_2S": BATT_BOX, "Ref_Receiver": RX_BOX, "Ref_ESC_G40": ESC_BOX,
           "Rear_Cage_Tail": CAGE_T, "HW_Rear_Cage_Screw_L": P["HW_Rear_Cage_Screw_L"], "HW_Rear_Cage_Screw_R": P["HW_Rear_Cage_Screw_R"], "HW_Rear_Cage_Screw_Post_L": P["HW_Rear_Cage_Screw_Post_L"], "HW_Rear_Cage_Screw_Post_R": P["HW_Rear_Cage_Screw_Post_R"]}
def dil(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]):
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
BULK = ld("IFS_Bulkhead_Convertible"); _b = I(BULK, box(BULK_X0 - 1, TUB_X1 + 1, -40, 40, -20, 40)); _v0 = P["Chassis_Tub"].volume
P["Chassis_Tub"] = single("Chassis_Tub(notch)", D(P["Chassis_Tub"], [dil(_b, 0.1)])); KEEPOUT["Chassis_Tub"] = P["Chassis_Tub"]
for n_, sg_ in (("L", 1), ("R", -1)):
    for k_, (bx, bz) in enumerate(BULK_SCREWS, 1):
        if sg_ == 1 and bx == 33.0: continue
        P[f"HW_Bulkhead_Screw_{n_}{k_}"] = U([cylY(bx, bz, BULK_HEAD_R, sg_*(TUB_OUT - BULK_CB_DEPTH + 0.2), sg_*(TUB_OUT - 0.2), 20), cylY(bx, bz, BULK_SCR_R, sg_*(BULK_WALL_Y0 + 0.35), sg_*(TUB_OUT - BULK_CB_DEPTH + 0.3), 14)])
print("tub notched for the bulkhead: %.1f mm3 removed (bulkhead mounting faces untouched)" % (_v0 - P["Chassis_Tub"].volume))
CUT_CLR = float(os.environ.get("CUT_CLR", 0.3)); DIL = {k: dil(m, CUT_CLR) for k, m in KEEPOUT.items()}
def relief(name, target, keys, vmin=40.0):
    use = [k for k in keys if vol(target, KEEPOUT[k]) > 0.05]
    for c in (CUT_CLR, CUT_CLR + 0.033, CUT_CLR + 0.071, CUT_CLR + 0.12):
        out = D(target, [dil(KEEPOUT[k], c) for k in use]) if use else target
        sh = sorted(out.split(only_watertight=False), key=lambda q: -abs(q.volume))
        if name != "Shell": break
        out.export(OUT + "_tmp.stl"); q_ = trimesh.load(OUT + "_tmp.stl")
        if len(sh) == 1 and q_.is_watertight and len(q_.split(only_watertight=False)) == 1: break
    keep = [q for q in sh if abs(q.volume) >= vmin]; res = U(keep) if len(keep) > 1 else sh[0]
    print("relief", name, "clr %.3f" % c, "removed %.0f mm3" % (target.volume - res.volume), "shells", len(sh), "dropped:", [(round(abs(q.volume), 1), q.bounds.round(0).tolist()) for q in sh[1:] if abs(q.volume) > 5]); return res
P["Roll_Cage"] = single("Roll_Cage", relief("Roll_Cage", CAGE, [k for k in KEEPOUT if "Cage" not in k], 1e9))
P["Roll_Cage"] = single("Roll_Cage", D(P["Roll_Cage"], [box(-80, CAGE_CUT_X, -40, 40, -20, 60)]))
P["Roll_Cage"] = D(U([P["Roll_Cage"]] + CAGE_LUGS), [cylZ(x_, sg*y_, 0.6, 18.6, 20.01, 12) for sg in (1, -1) for x_, y_ in NJ_SCREWS])
P["Roll_Cage"].export(OUT + "_tmp.stl"); P["Roll_Cage"] = single("Roll_Cage", trimesh.load(OUT + "_tmp.stl"))   # drop zero-volume slivers that only appear after the STL roundtrip
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
FIX = {n: ld(n, OUT) for n in ("Chassis_Tub", "Roll_Cage", "Tacoma_Trophy_Body_Shell", "Rear_Cage_Hoop", "Rear_Cage_Stay_L", "Rear_Cage_Stay_R", "Ref_Servo_Envelope", 
                              "Ref_Gearbox", "Ref_Motor_GM1614", "Ref_ESC_G40", "Ref_Receiver", "Ref_Battery_2S", "Ref_Shock_RL", "Ref_Shock_RR", "HW_Rear_Cage_Screw_L", "HW_Rear_Cage_Screw_R", "HW_Rear_Cage_Screw_Post_L", "HW_Rear_Cage_Screw_Post_R")}
MOVE_ARM = {n: ld(n, OUT) for n in ("Rear_Trailing_Arm_L", "Rear_Trailing_Arm_R", "HW_Rear_Trailing_Pivot_Screw_L", "HW_Rear_Trailing_Pivot_Screw_R")}
MOVE_UL = {n: ld(n, OUT) for n in ("Rear_Upper_Link_L", "Rear_Upper_Link_R", "HW_Rear_Upper_Link_Pivot_Screw_L", "HW_Rear_Upper_Link_Pivot_Screw_R")}
AXG = {n: R.M[n] for n in ("Ref_Rear_Axle", "Ref_Wheel_RL", "Ref_Wheel_RR") if n in R.M}; AXG["Rear_Axle_Top_Bracket"] = ld("Rear_Axle_Top_Bracket", OUT); AXG["HW_Rear_Axle_Top_Pin"] = ld("HW_Rear_Axle_Top_Pin", OUT)
for n in ("HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R"): AXG[n] = ld(n, OUT)
FIX["IFS_Bulkhead_Convertible"] = ld("IFS_Bulkhead_Convertible")
for n_ in ("IFS_Lower_Arm_L", "IFS_Lower_Arm_R", "IFS_Upper_Arm_L", "IFS_Upper_Arm_R", "Steering_Rack", "Servo_Horn", "Front_Carrier_L", "Front_Carrier_R", "Front_Spindle_L", "Front_Spindle_R"): FIX[n_] = ld(n_)
FIX["Ref_Front_Center_Driveshaft"] = trimesh.load(BD + "/Ref_Front_Center_Driveshaft.stl")
FIX["Ref_Rear_Driveshaft"] = ld("Ref_Rear_Driveshaft", OUT)
def scene(ta):
    a, u, T = R.solve(ta); sc = {}
    for n, m in FIX.items(): sc[n] = (m, np.eye(4))
    for n, m in MOVE_ARM.items(): sc[n] = (m, R.roty(a, PL[0], PL[1]))
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
SW = {ta: hits_vol(ta) for ta in (-14, -10, 0, 7)}
print("--- rest overlaps (mm3, only > 0.01) ---")
for p, v in sorted(SW[0].items(), key=lambda kv: -kv[1]):
    if not v < 0.01: print("  ", p, round(v, 3))
print("--- gearbox / motor / ESC / receiver / battery boxes vs everything (rest, >0.01 mm3):", [(p, round(v, 2)) for p, v in SW[0].items() if any(k in p[0] + p[1] for k in ("Ref_Gearbox", "Ref_Motor", "Ref_ESC", "Ref_Receiver", "Ref_Battery")) and not v < 0.01] or "none")
print("--- pairs that appear in the sweep but not at rest ---")
for ta in (-14, -10, 7):
    for p, v in sorted(SW[ta].items()):
        if p not in SW[0] and not v < 0.01: print("  ta", ta, p, round(v, 3))
print("--- kinematics ---   ta | axle dX dZ | pitch | link ang | shock eye-eye | driveshaft L, angle")
for r_ in KT: print("  %4d | %6.2f %6.2f | %5.2f | %6.2f | %5.2f | %6.2f %5.2f" % r_)
print("--- sway bar: ta | lever deg | link tilt from vertical deg | link length (L0 %.2f)" % L0)
for r_ in BAR_TILT: print("  %4d | %6.2f | %6.2f | %5.2f" % r_)
print("arm len %.2f link len %.2f  IC (%.1f, %.1f)  anti-squat ~%.0f%%" % (LEN[0], LEN[1], IC[0], IC[1], AS))
open(OUT + "_report.txt", "w").write(repr({"rep": rep, "KT": KT, "IC": IC.tolist(), "AS": AS}))
