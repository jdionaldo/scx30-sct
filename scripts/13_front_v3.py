"""Front-end v2: real steering knuckles (carrier + spindle on vertical kingpins), servo mounts,
4WD-capable front driveline, rack steering.  Builds watertight STLs (manifold3d) and runs
kinematic collision sweeps (python-fcl).

Inputs : per-body STL exports of scx30_sct_107mm_kit.sc (BODY_DIR) -- made with solidcraft-cli
         `file.export` per body (see AUDIT.md).  Coordinates: X fwd, Y left, Z up, mm.
Outputs: stl/v2/*.stl  (new/changed parts, LEFT+RIGHT),  report on stdout.
Usage  : python scripts/07_front_v2.py BODY_DIR
"""
import sys, os, glob, math, json
import numpy as np, trimesh
from scipy.optimize import brentq

BODY_DIR = sys.argv[1]
OUT = os.path.join(os.path.dirname(__file__), "..", "stl", "v3")
os.makedirs(OUT, exist_ok=True)
ENG = "manifold"

# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return m

def cyl(axis, c, r, l, n=40):
    m = trimesh.creation.cylinder(radius=r, height=l, sections=n)
    if axis == "x": m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    if axis == "y": m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    m.apply_translation(c); return m

def U(*a):
    a = [x for x in a if x is not None]
    return trimesh.boolean.union(a, engine=ENG)

def D(a, *b):
    return trimesh.boolean.difference([a] + list(b), engine=ENG)

def mirror(m):
    m = m.copy(); m.apply_transform(np.diag([1, -1, 1, 1.0])); m.faces = m.faces[:, ::-1]; m.fix_normals(); return m

def load(n): return trimesh.load(os.path.join(BODY_DIR, n + ".stl"))

def rotx(a, y0, z0):
    """rotation about X-parallel axis through (y0,z0), a in rad"""
    T = trimesh.transformations.rotation_matrix(a, [1, 0, 0], [0, y0, z0]); return T

def rotz(a, x0, y0):
    return trimesh.transformations.rotation_matrix(a, [0, 0, 1], [x0, y0, 0])

def rn(n): return n[:-1] + "R" if n.endswith("L") else n

def tr(v): return trimesh.transformations.translation_matrix(v)

# ---------------------------------------------------------------- constants (left side)
KX = 48.0                      # wheel / steering axis X
ZH = 3.0                       # hub centre Z
AL = (10.5, 0.0)               # lower arm inner pivot (Y,Z)
AU = (11.6, 6.8)               # upper arm inner pivot
PL = (32.5, 0.0)               # lower arm outer pivot (carrier)
PU = (30.9, 6.8)               # upper arm outer pivot
YS = 36.1                      # steering (kingpin) axis Y
WHEEL_REST_YMIN = 28.99          # rest-pose inner face of the front wheel before WHEEL_SHIFT (gives ride Y 32.89..46.49, Z -16.52..19.47 as at the v9 commit)
SHOCK_SCREW_REST_Y = 24.0       # lower shock screw centre Y before the move to SHOCK_LO0 (old body-dir position)
WHEEL_SHIFT = 4.0              # front wheels move outboard (track 85 -> 93 mm)
PIN_HINGE_D, BORE_HINGE_D = 1.5, 1.7
PIN_KP_D, BORE_KP_D = 1.2, 1.4
# ---- REAL servo: Injora N30 Pro (datasheet; items marked ? are pixel estimates -> correct after measuring)
SERVO_L, SERVO_W, SERVO_H = 15.8, 9.9, 20.13     # main case: L along Z (vertical), W along Y, H along X (spline axis = X, pointing forward) -- servo ON ITS SIDE
EAR_OUT = 2.95                  # ear flange projects past ONE long side (+Y side here): flange total 12.85
EAR_T = 1.2                     # ? ear thickness (ears sit at the case's spline-end face)
HOLE_SP, HOLE_V = 10.0, 11.3    # mount-hole spacing (centred on L) and distance from the non-ear long face
SPL_V, SPL_U = 4.3, 4.4         # spline centre: from the non-ear long face, from the (top) short end
SPL_D, BOSS_H = 3.93, 2.8       # spline boss dia / height past the case front face
EAR_HOLE_D = 1.2
SERVO_DZ = 5.8                  # UPDATE 13: tub + drivetrain stack (gearbox, motor, servo, wire clip) raised by 5.8 mm; wheels/arms/diff/bulkhead pivots stay
XC1, Y_N, ZC1 = 33.5, 1.95, 10.8 + SERVO_DZ   # (was 10.8) case front (spline-end) face X, non-ear (inner) face Y, case top Z (cage tube X 26.9..29.2 is at Z >= 12.1; tube X 34..36.1 at Z >= 10.4)
XC0 = XC1 - SERVO_H; Y_E = Y_N + SERVO_W; ZC0 = ZC1 - SERVO_L; ZCM = (ZC0 + ZC1) / 2
YSP, ZSP = Y_N + SPL_V, ZC1 - SPL_U                      # spline axis (Y, Z); axis along X
PILOTS = [(Y_N + HOLE_V, ZCM + g * HOLE_SP / 2) for g in (1, -1)]   # ear-screw axes (Y, Z); screws run +X through the ears into the front wall
HORN_T = 1.12; HORN_X0 = XC1 + 0.03; HORN_X1 = HORN_X0 + HORN_T
LINK_T = 1.03; LINK_X0 = HORN_X1 + 0.42; LINK_X1 = LINK_X0 + LINK_T
XA = (LINK_X0 + LINK_X1) / 2   # horn-pin line X
# ---- bell crank (vertical pivot pin on the bulkhead floor, XY plane): servo horn -> drag link -> crank input arm (-X); output arm (+X) pin rides an X-slot in the rack plate
XP, YPC, AO, AI = 32.55, -8.15, 5.4, 3.4    # pivot (X,Y), output arm, input arm lengths (output at X=37.95, keeps >=0.30 mm off tub cheek |Y|=12.9 X<=36.0)
CK_Z0, CK_Z1 = 0.25, 1.30                   # crank plane (leaves 0.85 mm solid floor above Z=-0.9 after -0.30 mm relief)
PL_Z0, PL_Z1 = 1.60, 2.65                   # rack slot plate / drag-link plate plane (0.30 mm clear above crank & pivot pin)
ZI = (PL_Z0 + PL_Z1) / 2
ZPIN = ZI                                   # horn pin height coplanar with ZI (minimises drag-link tilt & keeps floor thick)
HORN_R = ZSP - ZPIN                         # horn crank length (pin at ZPIN straight below the spline at centre)
I0 = np.array([XP - AI, YPC, ZI]); A0 = np.array([XA, YSP, ZPIN]); LD = float(np.linalg.norm(A0 - I0))   # drag-link pin-to-pin length
# ---- front axle chain (stock/Injora layout): diff axle shaft (slotted end) -> dogbone (joint at BOTH ends) -> stub axle in the knuckle (2 bearings). ESTIMATES: edit after measuring.
INJORA_PN = "SCX30-34BKFR"      # Injora +4.5 mm Planet Axle set (front 27.8 g, 2/13T front gear, stainless double-joint shafts, retaining spring on each joint pin)
DOG_RATING_DEG = 52.0           # listed steering rating of the Injora double-joint shaft (listing value)
DOG_LEN = None                  # joint-to-joint length: derived = YS - DOG_J1_Y (+DOG_J2_DY); lengthen by lowering DOG_J1_Y (longer shaft/plunge) if the diff output spacing is wider
WORM_RING_CD = 3.85             # ASSUMPTION: worm/ring centre distance (stock). The Injora 13T ring may differ; keep parametric and re-check the worm mesh
DOG_J1_Y = 10.3                 # diff-side joint centre Y (slotted end of the axle shaft, at the spool end) = CVD_S
DOG_J2_DY = 0.0                 # knuckle-side joint centre offset along the stub axle from the kingpin axis (0 = joint centre ON the kingpin axis)
DOG_SLOT = 2.5                  # assumed slot engagement depth per end (plunge capacity each way = this minus 0.5 retention)
BRG_IN_Y, BRG_OUT_Y = 40.5, 42.1  # stub-axle bearings in the knuckle (Ø4 x 1.6, back to back): one in the wall, one in the hub sleeve
# wire clip (pad on the bulkhead + lid): wall >= 0.8 mm between groove, screw holes and edges (widened to Y=14.35 per #16 printability audit)
CLIP_X0, CLIP_X1, CLIP_GX, CLIP_GR, CLIP_ZT = 22.9, 29.9, 26.4, 0.7, 12.2 + SERVO_DZ   # lid roof over groove = 1.0 mm
CLIP_Y1, CLIP_SY = 14.35, 13.175   # pad/lid outer Y=14.35 (2.50 mm wide), screw line Y=13.10 (>=0.75 mm wall each side of screw holes)
CLIP_SX = (24.3, 28.5)            # screw x: groove wall = 2.1-0.55-0.7 = 0.85 ; edge wall 24.3-0.55-22.9 = 0.85
GBX_SHAFT_Z = 4.1 + SERVO_DZ      # (was 4.1) gearbox output shaft height, raised with the stack; UNVERIFIED gearbox output-shaft height (spec update 3)
GBX_STUB_X = 15.5               # end of the gearbox output stub (Ref_Gearbox mesh: Ø3 stub on Y0 / Z4.1, case face X6, stub end X15.5)
GBX_STUB_R = 1.5                # stub radius (mesh)
CPL_LEN, CPL_OD, CPL_BORE_R, CPL_CAP = 3.5, 3.5, 1.55, 1.0   # coupler sleeve: slides over the stub (bore Ø3.1), 1.0 mm cap; the shaft starts on the cap face
GBX_X = GBX_STUB_X + CPL_CAP      # front shaft start (coupler cap face), X16.5
WORM_IN = (39.8, 0.0, 6.85)     # front worm input (existing Ref_Front_Input_Worm_Gear)
TUN_R = 1.80                    # shaft tunnel radius (Ø2.2 shaft + 0.70 radial clearance)
GUIDE_X = 38.35                 # rack guide screw / guide slot centre X (slot 1.24 wide inside a frame X 36.9..39.8: 0.83 mm walls both sides)
RACK_X, ROD_Y0, ROD_Z, ROD_T = 37.95, 6.0, 3.95, 1.1  # rack cross-bar ahead of horn/link stack (>=0.30 mm clear of Pin_A & Drag_Link, tie rods clear upper arms & tub)
ACK_A = 6.5
ACK_AIM = 90.0   # steering arm aims at X = -ACK_AIM on the centre line. Owner decision (Ackermann variant B): 59.0 -> 90 (59 was the rear axle at the old 107 mm wheelbase and gave 129 % at rack 3.0; 90 gives 58..92 %, over-centre limit 3.22 -> 3.65)
th_a = math.atan2(YS, KX + ACK_AIM)
ARM_PT = (KX - ACK_A * math.cos(th_a), YS - ACK_A * math.sin(th_a))   # tie-rod outer pivot (X,Y)

parts = {}                     # name -> mesh (LEFT unless noted)

# ---------------------------------------------------------------- arms
SHOCK_UP = np.array([58.6, 12.0, 20.5]); SHOCK_LO0 = np.array([58.6, 22.5, 2.5])   # eye centres (upper on bulkhead fork tower, lower on lower arm at arm angle 0; Item 1: moved LO inboard 24->22.5 and UP to (12.0,20.5) for >=1.0 mm wheel & bulkhead clearance across full travel)
def _bevel_pocket(poly_xy, z_bot, z_top=1.75, bev=0.25):
    """Chamfered top-face pocket cutter: flat floor at z_bot, 45 deg bevel expanding outward at top."""
    P_ = np.array(poly_xy, float); cen = P_.mean(0)
    Q_ = []
    for i_ in range(len(P_)):
        p_prev, p_cur, p_next = P_[(i_ - 1) % len(P_)], P_[i_], P_[(i_ + 1) % len(P_)]
        v1 = p_cur - p_prev; v1 = v1 / np.linalg.norm(v1)
        v2 = p_next - p_cur; v2 = v2 / np.linalg.norm(v2)
        # outward normal (assuming CCW or auto-orient away from centroid)
        n1 = np.array([v1[1], -v1[0]]); n2 = np.array([v2[1], -v2[0]])
        if np.dot(n1, p_cur - cen) < 0: n1 = -n1
        if np.dot(n2, p_cur - cen) < 0: n2 = -n2
        bis = n1 + n2; bis = bis / max(1e-6, np.linalg.norm(bis))
        denom = max(0.35, float(np.dot(bis, n1)))
        Q_.append(p_cur + bis * (bev / denom))
    pts = [(x, y, z_bot) for (x, y) in P_] + [(x, y, z_top - bev) for (x, y) in P_] + [(x, y, z_top) for (x, y) in Q_]
    return trimesh.convex.convex_hull(np.array(pts))

la = load("IFS_Lower_Arm_L")
la = D(la, box(40, 58, 27.0, 40, -5, 6), box(54.8, 58.2, 22.4, 25.8, 1.8, 4.2),
       box(51.8, 53.0, 26.0, 27.1, -5, 6),        # wall fix 2: feathered tip of the diagonal rib against the Y 27.0 cut (0.06-0.28 mm) cut back to X 53.0, where the rib is 0.85 mm wide
       box(53.2, 55.0, 25.05, 25.8, 1.8, 4.2))     # wall fix: removes the 0.38 mm neck left between the raised block (Y<=25.05) and the diagonal rib (Y>=25.8)
tip = box(44.2, 51.8, 26.0, 34.4, -1.5, 1.5)
tip = D(tip, box(46.3, 49.7, 29.0, 36, -5, 5))
LUG_SOLID_X0 = 50.0  # the solid lug block reaches back to X 50 (owner: solid all the way to the arm); X 48 would come 0.5 mm from the inner CVD at rest
LUG_R = 1.75          # lower shock lug eye radius (was 1.35: 0.57 mm of ring round the D1.56 pilot; now about 1.0 mm)
lug = U(box(55.4, 57.8, SHOCK_LO0[1] - LUG_R, SHOCK_LO0[1] + LUG_R, 0.8, SHOCK_LO0[2]),
        cyl("x", [56.6, SHOCK_LO0[1], SHOCK_LO0[2]], LUG_R, 2.4, 28))
la = U(la, tip, lug)
# Solid lug gusset (owner 2026-11-24): the shock lug stood on a post and two slender struts with an open triangle under the eye.
# The struts, post, legs and eye are filled to their convex hull (one solid block from the arm bottom to the eye, X LUG_SOLID_X0..57.8).
_lg = trimesh.boolean.intersection([la, box(LUG_SOLID_X0, 57.8, 15.5, 27.5, -3.3, SHOCK_LO0[2] + LUG_R + 0.1)], engine=ENG)   # full depth, back into the arm body
la = U(la, _lg.convex_hull)
# Axial Yeti (AX80111) recessed top-face truss pockets (arm plate Z=-1.5..1.5; bottom stays flat). Measured on the built arm: every pocket wall >= 0.93 mm to the nearest side face / window / hole, floor web >= 1.95 mm.
#  two wedge pockets flank the triangular window (between it and the outer edges), two tip pockets sit between the small through-holes (Y<=24.7) and the carrier slot (Y>=29.0)
_LA_POCKETS = [([(45.4, 25.8), (47.1, 25.8), (47.1, 27.9), (46.8, 27.9), (45.4, 26.9)], 0.45),      # rear tip pocket (corner cut clear of the spindle droop relief at X44.3..45.3 / Y28.3..29.4)
               ([(48.9, 25.8), (50.6, 25.8), (50.6, 27.9), (48.9, 27.9)], 0.45),                    # front tip pocket (1.8 mm centre rib between the two)
               ([(43.25, 15.6), (44.15, 15.6), (46.0, 19.8), (44.9, 21.6), (44.6, 22.0)], 0.50),    # rear flank wedge
               ([(52.75, 15.6), (51.85, 15.6), (50.0, 19.8), (51.1, 21.6), (51.4, 22.0)], 0.50)]    # front flank wedge (mirror about X=48)
la = D(la, *[_bevel_pocket(p_, z_bot=zb_, z_top=1.70, bev=0.20) for (p_, zb_) in _LA_POCKETS],
       cyl("x", [KX, PL[0], PL[1]], BORE_HINGE_D / 2, 12, 28),
       cyl("x", [56.6, SHOCK_LO0[1], SHOCK_LO0[2]], 0.78, 3.6, 28))
la = max(la.split(only_watertight=False), key=lambda s: s.volume)
parts["IFS_Lower_Arm_L"] = la

ua = load("IFS_Upper_Arm_L")
ua = D(ua, box(40, 58, 27.0, 40, 3, 12))
tip = box(44.2, 51.8, 26.0, 32.8, 5.3, 8.3)
tip = D(tip, box(46.3, 49.7, 28.5, 36, 3, 11))
ua = U(ua, tip)
ua = D(ua, cyl("x", [KX, PU[0], PU[1]], BORE_HINGE_D / 2, 12, 28))
parts["IFS_Upper_Arm_L"] = ua

# ---------------------------------------------------------------- carrier
def tongue(py, pz, zlo, zhi):
    return U(box(46.5, 49.5, py, 35.0, zlo, zhi), cyl("x", [KX, py, pz], (zhi - zlo) / 2, 3.0, 28))
low_t = tongue(PL[0], PL[1], -1.4, 1.4)
up_t = tongue(PU[0], PU[1], 5.3, 8.3)
bars = [box(KX - 3.5, KX - 2.3, 34.7, 37.5, -1.5, 8.3), box(KX + 2.3, KX + 3.5, 34.7, 37.5, -1.5, 8.3)]
webs = [box(KX - 3.5, KX + 3.5, 34.7, 37.5, -1.5, 0.5), box(KX - 3.5, KX + 3.5, 34.7, 37.5, 5.5, 8.3)]
car = U(low_t, up_t, *bars, *webs)
car = D(car, cyl("x", [KX, PL[0], PL[1]], BORE_HINGE_D / 2, 12, 28), cyl("x", [KX, PU[0], PU[1]], BORE_HINGE_D / 2, 12, 28),
        cyl("z", [KX, YS, 7.0], BORE_KP_D / 2, 14, 24), cyl("z", [KX, YS, -0.5], BORE_KP_D / 2, 6, 24))
_cvs = [cyl("y", [KX, (YS - 0.25 + 43.3) / 2, ZH], 0.98 + 0.25, 43.3 - YS + 0.25, 24) for _ in range(1)]
_cvs = U(*[_cvs[0].copy().apply_transform(rotz(math.radians(a_), KX, YS)) for a_ in np.arange(-42.5, 42.6, 2.5)])
_car0 = car; car = D(car, _cvs); car = max(car.split(only_watertight=False), key=lambda s: s.volume)
print(" carrier CVD relief: removed %.3f mm3 (outer CVD stub r 0.98 + 0.25 swept +/-42.5 deg about the kingpin; bars notched at their outboard inner corners)" % (_car0.volume - car.volume))
parts["Front_Carrier_L"] = car

# ---------------------------------------------------------------- spindle (steering knuckle)
wall = box(44.8, 51.2, 39.7, 41.7, -3.6, 10.4)
ear_u = U(box(45.4, 50.6, YS, 39.8, 8.6, 10.4), cyl("z", [KX, YS, 9.5], 2.5, 1.8, 28))
ear_l = U(box(45.4, 50.6, YS, 39.8, -3.6, -1.8), cyl("z", [KX, YS, -2.7], 2.5, 1.8, 28))
hub = cyl("y", [KX, 42.5, ZH], 3.2, 1.6, 32)
arm_path = U(box(44.8, 46.5, 39.7, 41.7, 2.0, 3.1),
             trimesh.convex.convex_hull(np.array([[44.8, 39.7, 2.0], [44.8, 41.7, 2.0], [42.8, 41.7, 2.0], [42.8, 39.7, 2.0],
                                                  [44.8, 39.7, 3.1], [44.8, 41.7, 3.1], [42.8, 41.7, 3.1], [42.8, 39.7, 3.1]])),
             box(41.9, 43.7, 33.0, 41.7, 2.0, 3.1),
             cyl("z", [ARM_PT[0], ARM_PT[1], 2.55], 1.9, 1.1, 28))
sp = U(wall, ear_u, ear_l, arm_path)
sp = D(sp, cyl("y", [KX, 40.7, ZH], 2.05, 6.0, 40),                         # bearing seat 4.1
       cyl("z", [KX, YS, 9.5], BORE_KP_D / 2, 4, 28), cyl("z", [KX, YS, -2.7], BORE_KP_D / 2, 4, 28),
       cyl("z", [ARM_PT[0], ARM_PT[1], 2.55], 0.6, 4, 28))      # Ø1.2 self-tapping pilot for the M1.4 tie-rod screw (was a Ø1.6 clearance hole: nothing held the screw)
parts["Front_Spindle_L"] = sp

# ---------------------------------------------------------------- bulkhead: servo cradle (servo ON ITS SIDE), shaft tunnel, rack deck
bk = load("IFS_Bulkhead_Convertible")
V_ORIG = bk.volume
bk = D(bk, box(38.0, 59.0, 13.6, 20.0, 8.5, 20.0), box(38.0, 59.0, -14.0, 14.0, 12.5, 20.0),   # strip obsolete v2 side-servo wing/tower if BODY_DIR came from v2 export
       box(24, 36.1, 13.45, 15, -3, 13), box(24, 36.1, -15, -13.45, -3, 13))                   # side walls flush with rail inner faces
EARW_T = 2.4                    # owner request: the plate that carries the ear-screw pilots sits BEHIND the servo tabs (was in front, X 33.65..36.1); screws enter from the front and thread rearward
EARW_X1 = XC1 - EAR_T - 0.15; EARW_X0 = EARW_X1 - EARW_T
EARS_ACC_R, EARS_ACC_X1 = 1.3, 41.0   # driver / head channel radius (head dia 2.0 + 0.3) and front end X
front_wall = box(EARW_X0, EARW_X1, Y_E + 0.15, Y_E + EAR_OUT + 0.10, ZC0 + 0.2, ZC1 - 0.9)   # ear wall behind the tabs: X 29.75..32.15, Y 12.0..14.9, Z 1.0..15.7 (name kept)
cage_ = trimesh.load(next(f_ for f_ in (os.path.join(os.path.dirname(__file__), "..", "stl", d_, n_ + ".stl") for n_ in ("Cage", "Roll_Cage") for d_ in ("v4", "v3")) if os.path.exists(f_)))   # read-only: keep the pilot wall off the cage tubes (+0.3)
cc_ = trimesh.boolean.intersection([cage_, box(30, 40, 6, 17, -6, 14)], engine=ENG)
if len(cc_.faces): front_wall = D(front_wall, U(*[cc_.copy().apply_translation(d_) for d_ in ([0, 0, 0], [.3, 0, 0], [-.3, 0, 0], [0, .3, 0], [0, -.3, 0], [0, 0, .3], [0, 0, -.3])]))
HINGE_FILL_X = (38.3, 40.0)     # wall fix: lower hinge-pin bore tail behind the pin end (PIN_TRIM_X0 40.2) filled solid: the floor under it was 0.10-0.15 mm
UHB_BOSS = (46.4, 49.4, 1.6, 0.82)   # wall fix 2: X0, X1, outer r, inner r of a ring coaxial with the upper hinge pin on the boss between the arm prongs (outboard wall 0.40 -> 0.8; same X extent as the boss, so the axial gap to the prongs is unchanged)
bk = U(bk, *[D(cyl("x", [sum(UHB_BOSS[:2]) / 2, s_ * AU[0], AU[1]], UHB_BOSS[2], UHB_BOSS[1] - UHB_BOSS[0], 48), cyl("x", [sum(UHB_BOSS[:2]) / 2, s_ * AU[0], AU[1]], UHB_BOSS[3], 4.0, 32)) for s_ in (1, -1)])
EARS_DBL = (XC1 + 0.15, 36.4, 10.8, 11.7, 2.3, 5.1)   # doubler on the inboard face of the cheek post in front of the lower ear screw: the driver channel leaves 0.35 mm of the post (Y 11.6..11.95) without it
bk = U(bk, box(*EARS_DBL))
bk = U(bk,  front_wall, box(33.5, 40.2, -4.2, 4.2, -1.6, 1.80),*[cyl("x", [sum(HINGE_FILL_X) / 2, s_ * AL[0], AL[1]], 0.9, HINGE_FILL_X[1] - HINGE_FILL_X[0], 28) for s_ in (1, -1)])                         # + rack deck (guide-screw boss at RACK_X=38.4)
bk = D(bk, box(XC0 - 0.15, XC1 + 0.15, Y_N - 0.15, Y_E + 0.15, ZC0 - 0.15, 40),                      # case pocket (open top: servo drops in)
       box(XC1 - EAR_T - 0.15, XC1 + 0.15, Y_E, Y_E + EAR_OUT + 0.3, ZC0 - 0.15, 40),                 # ear-tab recess
       *[cyl("x", [(XC1 + 0.05 + EARS_ACC_X1) / 2, py_, pz_], EARS_ACC_R, EARS_ACC_X1 - XC1 - 0.05, 28) for (py_, pz_) in PILOTS],   # screw-head / driver access IN FRONT of the tabs (dia 2.6 channels along X to X 41)
       cyl("x", [XC1 + 1.4, YSP, ZSP], SPL_D / 2 + 0.6, 3.2, 28))                                     # boss / spline relief
for (py_, pz_) in PILOTS:
    bk = D(bk, cyl("x", [EARW_X1 - 1.1, py_, pz_], 0.45, 2.25, 14))                                    # dia 0.9 pilot for M1.0 (self tapping), 2.2 deep into the ear wall from its front face (blind: 0.18 mm floor)
SH0 = np.array([GBX_X + 0.5, 0.0, GBX_SHAFT_Z + (WORM_IN[2] - GBX_SHAFT_Z) * 0.5 / (WORM_IN[0] - GBX_X)])
SH1 = np.array(WORM_IN)
def seg_cyl(p0, p1, r, n=24):
    p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0)
    m = trimesh.creation.cylinder(radius=r, height=L, sections=n)
    m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (p1 - p0) / L)); m.apply_translation((p0 + p1) / 2); return m
bk = D(bk, seg_cyl(SH0, SH1 + np.array([4.0, 0, 0.55]), TUN_R))
bk = D(bk, seg_cyl(SH0 + (SH1 - SH0) * ((35.4 - SH0[0]) / (SH1[0] - SH0[0])), SH1 + np.array([0.5, 0, 0.05]), 2.15))     # Ø4.3 seat for the worm-end dogbone/U-joint coupler
# ---- front shock: Injora SCX30-08 (26 mm extended / 18.3 compressed, 7.7 travel, 1.0 g; body Ø8-9 with spring, eye centre to end ~2 mm). SHOCK_UP / SHOCK_LO0 defined above.
SHOCK_LMIN, SHOCK_LMAX = 18.3, 26.0       # eye-to-eye compressed / extended (listing, 7.7 mm travel)
SHOCK_BODY_R, SHOCK_BODY_LEN, SHOCK_ROD_R, SHOCK_EYE_R, SHOCK_EYE_W, SHOCK_BODY_OFF = 4.0, 13.0, 0.7, 1.5, 1.4, 3.0   # eye 1.4 wide: fits between the arm lug (X<=57.8) and the lower screw head (X>=59.3)   # body Ø8 spans OFF..LEN from the upper eye (eye tab/neck before it)   # body Ø8 (listing: Ø8-9 with spring: ESTIMATE), body length from the upper eye, rod Ø3, eye boss
def make_shock(U3, P3, grow=0.0):
    U3 = np.array(U3, float); P3 = np.array(P3, float); d = (P3 - U3) / np.linalg.norm(P3 - U3)
    eye = lambda c: D(cyl("x", list(c), SHOCK_EYE_R + grow, SHOCK_EYE_W + 2 * grow, 24), cyl("x", list(c), 0.85, 4.0, 16)) if grow == 0 else cyl("x", list(c), SHOCK_EYE_R + grow, SHOCK_EYE_W + 2 * grow, 24)
    m_ = U(eye(U3), seg_cyl(U3, U3 + d * (SHOCK_BODY_OFF + 0.5), SHOCK_EYE_W / 2 + grow), seg_cyl(U3 + d * SHOCK_BODY_OFF, U3 + d * SHOCK_BODY_LEN, SHOCK_BODY_R + grow), seg_cyl(U3 + d * (SHOCK_BODY_LEN - 0.5), P3, SHOCK_ROD_R + grow), eye(P3))
    return m_ if grow else D(m_, cyl("x", list(U3), 0.85, 4.0, 20), cyl("x", list(P3), 0.85, 4.0, 20))      # pin bores re-opened after the union
def shock_lo(th_deg):     # lower eye centre when the lower arm is at angle th_deg (rotation about the inner pivot AL, X axis)
    a_ = math.radians(th_deg); d_ = SHOCK_LO0[1:] - np.array(AL)
    return np.array([58.6, AL[0] + d_[0] * math.cos(a_) - d_[1] * math.sin(a_), AL[1] + d_[0] * math.sin(a_) + d_[1] * math.cos(a_)])
sw = [make_shock(SHOCK_UP, shock_lo(th_), grow=0.15) for th_ in np.arange(-14, 12.1, 2)]
relief = U(*sw)
bk = D(bk, relief, mirror(relief))
print("bulkhead (before steering relief): shells %d, vol %.0f -> %.0f mm3" % (len(bk.split(only_watertight=False)), V_ORIG, bk.volume))
parts["IFS_Bulkhead_Convertible"] = bk

# ---------------------------------------------------------------- steering parts: horn (Y-Z plane) -> link (along Y) -> centre rack
RACK_PILOT_R, RACK_REAR_X = 0.60, 36.72
rack = U(
    box(36.90, 39.40, -7.4, 7.4, 2.05, 3.05),                                   # main cross-bar (2.50 wide x 1.00 thick)
    box(36.50, 39.40, -4.6, 0.4, 2.05, 3.05),                                   # 2.90 wide guide-slot frame (>=0.83 mm walls on both sides of slot, fixes #16 0.35 mm hotspot!)
    cyl("z", [RACK_X, ROD_Y0, 2.425], 1.50, 1.25, 28),                          # left tie-rod boss Z 1.80..3.05
    cyl("z", [RACK_X, -ROD_Y0, 2.425], 1.50, 1.25, 28),                         # right tie-rod boss Z 1.80..3.05
    box(36.15, 39.40, YPC - 1.45, -6.8, PL_Z0, PL_Z1),                          # slot plate ahead of tub cheek X<=36.0
    box(36.90, 39.80, -4.6, 4.6, 2.05, 3.05),                                   # guide frame widened forward to X 39.8 (0.83 mm wall ahead of the guide slot)
    box(35.65, 36.20, YPC - 0.95, YPC + 0.95, PL_Z0, PL_Z1),                    # rear rib behind the crank-pin slot: slot end wall 0.30 -> 0.80 mm (stays 0.8 mm inside the tub cheek |Y|=12.9 at lock)
)
rack = D(
    rack,
    box(35.0, RACK_REAR_X, 0.4, 9.0, 1.0, 4.0),                                  # +Y rear face (wall fix: 36.90 -> 36.72, Pin_A head X 36.30 keeps 0.42 mm = above the pair minimum 0.409)
    box(35.0, 36.60, -7.1, -6.0, 1.0, 4.0),                                     # chamfers slot-plate rear +Y corner for >=0.38 mm drag-link clearance
    box(GUIDE_X - 0.62, GUIDE_X + 0.62, -3.8, 3.8, 1.5, 6.0),                     # guide slot (1.24 wide for Ø1.10 guide screw shank)
    cyl("z", [RACK_X, ROD_Y0, 2.5], RACK_PILOT_R, 6, 28),                       # wall fix: dia 1.2 self-tapping pilot for the M1.4 inner tie-rod screw (was a dia 1.56 clearance hole: 0.27 mm rear wall, nothing held the screw)
    cyl("z", [RACK_X, -ROD_Y0, 2.5], RACK_PILOT_R, 6, 28),
    box(36.45, 38.55, YPC - 0.55, YPC + 0.55, 1.0, 4.0),                        # X-elongated slot for crank output pin
)
parts["Steering_Rack"] = rack

HXC = (HORN_X0 + HORN_X1) / 2
horn = U(cyl("x", [HXC, YSP, ZSP], 3.3, HORN_T, 36), box(HORN_X0, HORN_X1, YSP - 1.5, YSP + 1.5, ZPIN, ZSP), cyl("x", [HXC, YSP, ZPIN], 1.55, HORN_T, 28))
horn = D(horn, cyl("x", [HXC, YSP, ZSP], 1.95, 3, 32), cyl("x", [HXC, YSP, ZPIN], 0.65, 3, 24))     # Ø3.9 spline bore; Ø1.3 pin hole
parts["Servo_Horn"] = horn
def seg_plate(p0, p1, w, z0, z1):
    p0, p1 = np.array(p0[:2], float), np.array(p1[:2], float); Lh = float(np.linalg.norm(p1 - p0))
    m = box(0, Lh, -w / 2, w / 2, z0, z1); m.apply_transform(rotz(math.atan2(*(p1 - p0)[::-1]), 0, 0)); m.apply_translation([p0[0], p0[1], 0]); return m
KE = (34.95, -1.0)     # elbow: keeps >=1.0 mm off servo case, >=0.26 mm off servo horn, and >=0.38 mm off rack
link = U(cyl("x", [XA, YSP, ZPIN], 1.45, LINK_T, 28), seg_plate((XA + 0.05, YSP), KE, 1.15, PL_Z0, PL_Z1), seg_plate(KE, (I0[0], I0[1]), 1.50, PL_Z0, PL_Z1),
         cyl("z", [KE[0], KE[1], ZI], 0.75, PL_Z1 - PL_Z0, 24), cyl("z", [I0[0], I0[1], ZI], 1.55, PL_Z1 - PL_Z0, 28))
link = D(link, cyl("x", [XA, YSP, ZPIN], 0.65, 3, 24), cyl("z", [I0[0], I0[1], ZI], 0.55, 4, 24))
parts["Steering_Drag_Link"] = link
CZ = (CK_Z0 + CK_Z1) / 2; CH = CK_Z1 - CK_Z0
crank = U(cyl("z", [XP, YPC, CZ], 2.2, CH, 32), box(XP, XP + AO, YPC - 1.15, YPC + 1.15, CK_Z0, CK_Z1), cyl("z", [XP + AO, YPC, CZ], 1.50, CH, 28),
          box(XP - AI, XP, YPC - 1.15, YPC + 1.15, CK_Z0, CK_Z1), cyl("z", [XP - AI, YPC, CZ], 1.55, CH, 28))
crank = D(crank, cyl("z", [XP, YPC, CZ], 0.65, 4, 28), cyl("z", [XP + AO, YPC, CZ], 0.55, 4, 24), cyl("z", [XP - AI, YPC, CZ], 0.55, 4, 24))
parts["Steering_BellCrank"] = crank

def rod_mesh(p0, p1, w=2.0, t=ROD_T, eye=1.55):
    p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0)
    m = U(box(0, L, -w / 2, w / 2, -t / 2, t / 2), cyl("z", [0, 0, 0], eye, t, 28), cyl("z", [L, 0, 0], eye, t, 28))
    m = D(m, cyl("z", [0, 0, 0], 0.78, 3, 24), cyl("z", [L, 0, 0], 0.78, 3, 24))
    d = (p1 - p0) / L; ang = math.atan2(d[1], d[0])
    m.apply_transform(rotz(ang, 0, 0)); m.apply_translation(p0); return m
ROD_IN = np.array([RACK_X, ROD_Y0, ROD_Z]); ROD_OUT = np.array([ARM_PT[0], ARM_PT[1], ROD_Z])
ROD_L0 = np.linalg.norm(ROD_OUT - ROD_IN)
ROD_SEAT_DROP = 0.10   # outer eye spot-faced 0.10 from the top (eye 1.0 thick): lowers the outer screw head, which passed 0.155 mm under the upper-arm prong at droop + lock after the ACK_AIM change
parts["Tie_Rod_L"] = D(rod_mesh(ROD_IN, ROD_OUT), cyl("z", [ROD_OUT[0], ROD_OUT[1], ROD_Z + ROD_T / 2 - ROD_SEAT_DROP + 0.5], 1.75, 1.0, 40))
_rod_fill = U(parts["Tie_Rod_L"], cyl("z", list(ROD_IN), 0.85, ROD_T, 24), cyl("z", list(ROD_OUT), 0.85, ROD_T, 24))   # wall fix: rod with closed eyes, used only as the cutter of the rack / spindle seat reliefs

# ---------------------------------------------------------------- reference / hardware (not printed)
refs = {}
sv = U(box(XC0, XC1, Y_N, Y_E, ZC0, ZC1), cyl("x", [XC1 + BOSS_H / 2, YSP, ZSP], SPL_D / 2, BOSS_H, 24),
       box(XC1 - EAR_T, XC1, Y_E, Y_E + EAR_OUT, ZCM - HOLE_SP / 2 - 1.8, ZCM + HOLE_SP / 2 + 1.8))
sv = D(sv, *[cyl("x", [XC1, py_, pz_], EAR_HOLE_D / 2, 3, 24) for py_, pz_ in PILOTS])
refs["Servo_Ref"] = sv
for i, (py_, pz_) in enumerate(PILOTS):
    refs[f"HW_Servo_Mount_Screw_{'LR'[i]}"] = U(cyl("x", [XC1 - 1.75, py_, pz_], 0.5, 3.5, 20), cyl("x", [XC1 + 0.30, py_, pz_], 1.0, 0.6, 20))   # M1.0 x 3.5 from the FRONT: head X 33.5..34.1 on the ear, shank back to X 30.0 (2.15 mm in the ear wall)
refs["HW_Rack_Guide_Screw"] = U(cyl("z", [GUIDE_X, 0, 1.4], 0.55, 3.8, 24), cyl("z", [GUIDE_X, 0, 3.30], 0.9, 0.5, 24))
refs["HW_Horn_Screw"] = U(cyl("x", [XC1 + 2.4, YSP, ZSP], 0.6, 2.0, 20), cyl("x", [XC1 + BOSS_H + 0.25, YSP, ZSP], 1.2, 0.5, 20))
refs["HW_Link_Pin_A"] = U(cyl("x", [(HORN_X0 + 0.15 + LINK_X1) / 2, YSP, ZPIN], 0.6, LINK_X1 - HORN_X0 - 0.15, 24), cyl("x", [LINK_X1 + 0.10, YSP, ZPIN], 0.85, 0.20, 24))
refs["HW_Crank_Pivot_Pin"] = cyl("z", [XP, YPC, 0.225], 0.6, 2.05, 24)                                                               # Ø1.2 steel dowel pin Z=-0.80..+1.25 (flush below crank top 1.30, 0.35 mm clear of drag link, stays inside floor >=-0.90)
refs["HW_Crank_Out_Pin"] = cyl("z", [XP + AO, YPC, (0.30 + PL_Z1) / 2], 0.5, PL_Z1 - 0.30, 20)   # Ø1.0 HEADLESS dowel Z=0.30..2.65 (press fit in the crank, flush with the rack slot-plate top: Tie_Rod_R passes over it at full droop)
refs["HW_Crank_In_Pin"] = U(cyl("z", [I0[0], I0[1], 1.50], 0.5, 2.40, 20), cyl("z", [I0[0], I0[1], PL_Z1 + 0.15], 0.85, 0.30, 20))   # Ø1.0 pin Z=0.30..2.95: crank hole -> drag-link eye
# front driveshaft dogbone/U-joint couplers (Ø3.0 x 4) at both ends; gearbox-end seat is the gearbox output bore
_sd = (np.array(WORM_IN) - np.array([GBX_X, 0, GBX_SHAFT_Z])); _sd = _sd / np.linalg.norm(_sd)
refs["HW_Front_Driveshaft_UJ_Gbx"] = D(seg_cyl([GBX_STUB_X - CPL_LEN + CPL_CAP, 0, GBX_SHAFT_Z], [GBX_X, 0, GBX_SHAFT_Z], CPL_OD / 2), seg_cyl([GBX_STUB_X - CPL_LEN + CPL_CAP - 0.5, 0, GBX_SHAFT_Z], [GBX_STUB_X, 0, GBX_SHAFT_Z], CPL_BORE_R))   # sleeve over the Ø3 gearbox stub (seat), cap end at X16.5
refs["HW_Front_Driveshaft_UJ_Worm"] = seg_cyl(np.array(WORM_IN) - _sd * 4.0, WORM_IN, 1.5)
refs["Ref_Front_Driveshaft"] = seg_cyl([GBX_X, 0, GBX_SHAFT_Z], WORM_IN, 1.1)
refs["Ref_Gearbox_Spec_Box"] = box(-11.5, 11.5, -13.0, 13.0, -4.9, 23.1)
refs["Rail_Box_L"] = box(-64.0, 36.0, 13.5, 18.5, -2.8, 23.1)
refs["Rail_Box_R"] = box(-64.0, 36.0, -18.5, -13.5, -2.8, 23.1)

CVD_S = np.array([KX, DOG_J1_Y, ZH]); CVD_J0 = np.array([KX, YS, ZH])
cvd_in = U(cyl("y", [KX, (DOG_J1_Y + YS) / 2, ZH], 1.0, YS - DOG_J1_Y, 20), trimesh.creation.icosphere(radius=1.7, subdivisions=2).apply_translation(CVD_J0))
cvd_out = U(cyl("y", [KX, (YS + 43.3) / 2, ZH], 0.98, 43.3 - YS, 20))
brg = D(cyl("y", [KX, BRG_OUT_Y, ZH], 2.0, 1.6, 32), cyl("y", [KX, BRG_OUT_Y, ZH], 0.99, 3, 20))
brg_in = D(cyl("y", [KX, BRG_IN_Y, ZH], 2.0, 1.6, 32), cyl("y", [KX, BRG_IN_Y, ZH], 0.99, 3, 20))
hw_hinge = {  # X-axis hinge pins at arm outer ends
    "HW_Front_Lower_Outer_Pin_L": cyl("x", [KX, PL[0], PL[1]], PIN_HINGE_D / 2, 8.0, 14),
    "HW_Front_Upper_Outer_Pin_L": cyl("x", [KX, PU[0], PU[1]], PIN_HINGE_D / 2, 8.0, 14)}
hw_kp = {"HW_Front_Upper_Kingpin_L": cyl("z", [KX, YS, 7.95], PIN_KP_D / 2, 4.7, 14),
         "HW_Front_Lower_Kingpin_L": cyl("z", [KX, YS, -1.55], PIN_KP_D / 2, 3.9, 14)}
# tie-rod screws (M1.4) L side: inner screw bottom Z=1.80 (0.50 mm above bellcrank top 1.30), 1.25 mm engagement in rack boss (1.80..3.05), head Z=4.55..5.05 on tie rod top (4.50)
hw_tr = {"HW_TieRod_Outer_Screw_L": U(cyl("z", [ARM_PT[0], ARM_PT[1], (2.05 + 4.55) / 2 - ROD_SEAT_DROP], 0.7, 2.50, 14), cyl("z", [ARM_PT[0], ARM_PT[1], 4.65 - ROD_SEAT_DROP], 1.0, 0.30, 20)),   # M1.4 x 2.5 wafer head (Ø2.0 x 0.3): shank Z 2.05..4.55 (ends inside the steering arm, was 0.2 proud below and hit the lower arm at droop+lock), head top Z 4.8 (was 5.2: hit the upper arm tip at droop+lock)
         "HW_TieRod_Inner_Screw_L": U(cyl("z", [RACK_X, ROD_Y0, 3.20], 0.7, 2.80, 14), cyl("z", [RACK_X, ROD_Y0, 4.80], 1.0, 0.50, 16))}

# ---------------------------------------------------------------- assemble LEFT/RIGHT sets
L_names = ["IFS_Lower_Arm_L", "IFS_Upper_Arm_L", "Front_Carrier_L", "Front_Spindle_L", "Tie_Rod_L"]
printable = {}
for n in L_names:
    printable[n] = parts[n]; printable[rn(n)] = mirror(parts[n])
for n in ["IFS_Bulkhead_Convertible", "Steering_Rack", "Servo_Horn", "Steering_Drag_Link", "Steering_BellCrank"]: printable[n] = parts[n]

# ---------------------------------------------------------------- static scene
replaced = ("IFS_Lower_Arm_", "IFS_Upper_Arm_", "IFS_Knuckle_", "IFS_Bulkhead", "Servo_Ref", "Servo_Horn", "Drag_Link", "HW_Drag",
            "Steering_", "Tie_Rod_", "HW_Front_Upper_Kingpin", "HW_Front_Lower_Kingpin", "HW_Front_TieRod", "HW_Steering_",
            "HW_Drag_Link", "HW_Servo_Mount", "Ref_Front_CVD", "Ref_Front_Center_Driveshaft", "Ref_Wheel_F", "HW_Front_Wheel_Nut",
            "HW_Front_Lower_Hinge_Pin", "HW_Front_Upper_Hinge_Pin",
            "Chassis_Rail", "Roll_Cage", "Cage", "Tacoma", "Slider", "Spare", "Rear_", "Ref_Rear", "Ref_Shock_R", "Ref_Wheel_R", "HW_Rear", "Ref_Gearbox", "Front_Pin_Retainer", "HW_Front_Shock_Upper_Screw", "HW_Front_Shock_Lower_Screw", "Ref_Shock_F")
allb = {os.path.basename(f)[:-4]: trimesh.load(f) for f in glob.glob(os.path.join(BODY_DIR, "*.stl"))}
static = {k: v for k, v in allb.items() if not k.startswith(replaced)}
# keep inner hinge pins (inboard end unchanged) but trim: reuse originals clipped to Y<=14 -> simply reuse
PIN_TRIM_X0 = 40.2               # lower hinge pins: inboard tip trimmed to X>=40.2 so it clears the bell-crank hub (>=0.2 mm)
for k in ("HW_Front_Lower_Hinge_Pin_L", "HW_Front_Lower_Hinge_Pin_R", "HW_Front_Upper_Hinge_Pin_L", "HW_Front_Upper_Hinge_Pin_R"):
    static[k] = allb[k] if "Upper" in k else D(allb[k], box(30.0, PIN_TRIM_X0, -30.0, 30.0, -10.0, 10.0))
static["IFS_Bulkhead_Convertible"] = bk
static["Servo_Ref"] = refs["Servo_Ref"]
for s in ("L", "R"):
    pass
static["HW_Servo_Mount_Screw_L"] = refs["HW_Servo_Mount_Screw_L"]; static["HW_Servo_Mount_Screw_R"] = refs["HW_Servo_Mount_Screw_R"]
static["HW_Rack_Guide_Screw"] = refs["HW_Rack_Guide_Screw"]
static["Ref_Front_Driveshaft"] = refs["Ref_Front_Driveshaft"]
SHOCK_UP_SHIFT = SHOCK_UP - np.array([58.6, 9.2, 10.4])      # upper eye moved from the old mount
_xl, _xr = SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1 - 0.8 - 0.4, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1 + 0.8        # screw through tower wall - eye - wall: shank Ø1.36, head Ø2.4 x 0.6 outside the +X wall
static["HW_Front_Shock_Upper_Screw_L"] = U(cyl("x", [(_xl + _xr) / 2, SHOCK_UP[1], SHOCK_UP[2]], 0.68, _xr - _xl, 20), cyl("x", [_xr + 0.3, SHOCK_UP[1], SHOCK_UP[2]], 1.2, 0.6, 24))
static["HW_Front_Shock_Upper_Screw_R"] = mirror(static["HW_Front_Shock_Upper_Screw_L"])
for k_ in ("Ref_Gearbox_Spec_Box", "Rail_Box_L", "Rail_Box_R", "HW_Horn_Screw"): static[k_] = refs[k_]

def sided(name, m):   # returns {name_L: m, name_R: mirror}
    return {name.replace("_L", "") + "_L": m, name.replace("_L", "") + "_R": mirror(m)}

# Body-dir drift guard (2026-11-21): the BODY_DIR export is regenerated from the .sc and can hold the wheel, nut and lower shock screw ALREADY at the ride pose / shifted track
# (wheel Y 36.89.., Z centre 1.48 instead of the rest pose Y 28.99.., Z centre ZH). Canonicalise them here so the result no longer depends on what the export contains.
_wb = allb["Ref_Wheel_FL"].bounds
_WD = np.array([0.0, WHEEL_REST_YMIN - _wb[0][1], ZH - (_wb[0][2] + _wb[1][2]) / 2])
_sb = allb["HW_Front_Shock_Lower_Screw_L"].bounds
_SD = np.array([0.0, SHOCK_SCREW_REST_Y - (_sb[0][1] + _sb[1][1]) / 2, SHOCK_LO0[2] - (_sb[0][2] + _sb[1][2]) / 2])
print(f"  body-dir drift guard: wheel/nut moved by {_WD.round(2).tolist()}, lower shock screw by {_SD.round(2).tolist()} before the pose")
# moving-group definitions: each entry (mesh_L, group) ; groups: 'arm_l','arm_u','car','spin','cvd_in', ...
moving = {"IFS_Lower_Arm_L": ("arm_l", parts["IFS_Lower_Arm_L"]), "IFS_Upper_Arm_L": ("arm_u", parts["IFS_Upper_Arm_L"]),
          "Front_Carrier_L": ("car", parts["Front_Carrier_L"]), "Front_Spindle_L": ("spin", parts["Front_Spindle_L"]),
          "Ref_Wheel_FL": ("spin", allb["Ref_Wheel_FL"].copy().apply_translation(_WD + [0, WHEEL_SHIFT, 0])),
          "HW_Front_Wheel_Nut_L": ("spin", allb["HW_Front_Wheel_Nut_L"].copy().apply_translation(_WD + [0, WHEEL_SHIFT, 0])),
          "CVD_Outer_L": ("spin", cvd_out), "Front_Bearing_L": ("spin", brg), "Front_Bearing_Inner_L": ("spin", brg_in),
          "HW_Front_Upper_Kingpin_L": ("car", hw_kp["HW_Front_Upper_Kingpin_L"]), "HW_Front_Lower_Kingpin_L": ("car", hw_kp["HW_Front_Lower_Kingpin_L"]),
          "HW_Front_Lower_Outer_Pin_L": ("car", hw_hinge["HW_Front_Lower_Outer_Pin_L"]), "HW_Front_Upper_Outer_Pin_L": ("car", hw_hinge["HW_Front_Upper_Outer_Pin_L"]),
          "HW_TieRod_Outer_Screw_L": ("spin", hw_tr["HW_TieRod_Outer_Screw_L"]),
          "CVD_Inner_L": ("cvd", cvd_in), "Tie_Rod_L": ("rod", parts["Tie_Rod_L"]),
          "HW_TieRod_Inner_Screw_L": ("rack", hw_tr["HW_TieRod_Inner_Screw_L"]),
          "Ref_Shock_FL": ("shock", make_shock(SHOCK_UP, SHOCK_LO0)), "HW_Front_Shock_Lower_Screw_L": ("arm_l", allb["HW_Front_Shock_Lower_Screw_L"].copy().apply_translation(_SD + [0.0, SHOCK_LO0[1] - SHOCK_SCREW_REST_Y, 0.0]))}
for k in ("Ref_Wheel_FL", "HW_Front_Wheel_Nut_L", "Ref_Shock_FL"): pass
# right side moving (rest only; mirrored)
rest_all = dict(static)
for n, (g, m) in moving.items():
    rest_all[n] = m; rest_all[rn(n)] = mirror(m)
rest_all["Steering_Rack"] = parts["Steering_Rack"]; rest_all["Servo_Horn"] = parts["Servo_Horn"]; rest_all["Steering_Drag_Link"] = parts["Steering_Drag_Link"]; rest_all["Steering_BellCrank"] = parts["Steering_BellCrank"]

from trimesh.collision import CollisionManager
def check(scene, label, ignore=()):
    cm = CollisionManager()
    for n, (m, T) in scene.items(): cm.add_object(n, m, transform=T)
    hit, names = cm.in_collision_internal(return_names=True)
    pairs = sorted({tuple(sorted(p)) for p in names if tuple(sorted(p)) not in ignore})
    return pairs

I = np.eye(4)
rest_scene = {n: (m, I) for n, m in rest_all.items()}
pairs = check(rest_scene, "rest")
print("\nREST interference pairs (v2 front end, all", len(rest_scene), "bodies):", len(pairs))
for p in pairs: print("  ", p)

# ---------------------------------------------------------------- kinematics
def solve_front(theta_deg, delta_rad=0.0):
    th = math.radians(theta_deg)
    def pl(a): return np.array([AL[0] + math.cos(a) * (PL[0] - AL[0]) - math.sin(a) * (PL[1] - AL[1]), AL[1] + math.sin(a) * (PL[0] - AL[0]) + math.cos(a) * (PL[1] - AL[1])])
    def pu(a): return np.array([AU[0] + math.cos(a) * (PU[0] - AU[0]) - math.sin(a) * (PU[1] - AU[1]), AU[1] + math.sin(a) * (PU[0] - AU[0]) + math.cos(a) * (PU[1] - AU[1])])
    d0 = np.linalg.norm(np.array(PU) - np.array(PL)); P = pl(th)
    f = lambda a: np.linalg.norm(pu(a) - P) - d0
    xs = np.radians(np.linspace(-60, 60, 241)); fv = [f(x) for x in xs]; best = None
    for i in range(len(xs) - 1):
        if fv[i] * fv[i + 1] <= 0:
            r = brentq(f, xs[i], xs[i + 1])
            if best is None or abs(r - th) < abs(best - th): best = r
    phi = best; Pu = pu(phi)
    v0 = np.array(PU) - np.array(PL); v1 = Pu - P
    psi = math.atan2(v1[1], v1[0]) - math.atan2(v0[1], v0[0])
    Tc = rotx(psi, PL[0], PL[1]); Tc = tr([0, P[0] - PL[0], P[1] - PL[1]]) @ Tc
    return dict(th=th, phi=phi, psi=psi, Tc=Tc)

def rod_len_err(sol, delta, s, sgn=1):
    """tie-rod length error for left (sgn=1) side at steering angle delta (rad, + = turn left), rack offset s"""
    Tz = rotz(delta, KX, YS)
    o = np.array([ARM_PT[0], ARM_PT[1], ROD_Z, 1.0]); O = (sol["Tc"] @ Tz @ o)[:3]
    Iy = ROD_IN + np.array([0, s, 0])
    return np.linalg.norm(O - Iy) - ROD_L0, O

def steer_from_rack(sol, s, side="L"):
    """steering angle (deg) of left wheel for rack offset s; right wheel uses mirrored geometry with -s"""
    ss = s if side == "L" else -s
    f = lambda d: rod_len_err(sol, d, ss)[0]
    xs = np.radians(np.linspace(-50, 50, 201)); fv = [f(x) for x in xs]
    roots = []
    for i in range(len(xs) - 1):
        if fv[i] * fv[i + 1] <= 0: roots.append(brentq(f, xs[i], xs[i + 1]))
    return min(roots, key=abs) if roots else None

print("\n--- STEERING (rack offset s -> wheel angles at ride height) ---")
sol0 = solve_front(0.0)
for s in (-3, -2.75, -2.5, -2, -1, 0, 1, 2, 2.5, 2.75, 3):
    dl = steer_from_rack(sol0, s, "L"); dr = steer_from_rack(sol0, s, "R")
    print(" s=%+.1f  left=%s  right=%s" % (s, None if dl is None else round(math.degrees(dl), 1), None if dr is None else round(-math.degrees(dr), 1)))
print(" (positive left angle = wheel turns left; right shown with same sign convention)")
# horn angle to deliver s
def crank_psi(s): return math.asin(max(-1.0, min(1.0, s / AO)))     # rack offset s = AO sin(psi) (output pin follows the rack slot)
def I_of(psi): return np.array([XP - AI * math.cos(psi), YPC - AI * math.sin(psi), ZI])
def A_of(a): return np.array([XA, YSP + HORN_R * math.sin(a), ZSP - HORN_R * math.cos(a)])
def horn_angle(s):
    """horn angle (rad, about +X through the spline) for rack offset s: |A(a) - I(psi)| = drag-link length"""
    I_ = I_of(crank_psi(s)); f = lambda a: float(np.linalg.norm(A_of(a) - I_)) - LD
    xs = np.linspace(-1.2, 1.2, 241); fv = [f(x) for x in xs]; roots = []
    for i in range(len(xs) - 1):
        if fv[i] * fv[i + 1] <= 0: roots.append(brentq(f, xs[i], xs[i + 1]))
    return min(roots, key=abs)
def link_T(s):
    a = horn_angle(s); A = A_of(a); I_ = I_of(crank_psi(s)); d0 = I0 - A0; d = I_ - A
    R = trimesh.geometry.align_vectors(d0 / np.linalg.norm(d0), d / np.linalg.norm(d)); return tr(A) @ R @ tr(-A0)
def crank_T(s): return rotz(crank_psi(s), XP, YPC)
print(" servo horn angle for s=+/-2.6 mm: %.1f deg ; s=+/-3.0: %.1f deg" % (math.degrees(horn_angle(2.6)), math.degrees(horn_angle(3.0))))

# bump steer: toe change over travel with rack centred
print("\n--- BUMP STEER (rack s=0): left wheel toe vs lower-arm angle ---")
for th in (-23, -12, -4, 0, 8, 15, 22):
    sol = solve_front(th)
    ds = steer_from_rack(sol, 0.0, "L")
    wz = (sol["Tc"] @ np.array([KX, 0, ZH, 1]))[2] if False else None
    hub = (sol["Tc"] @ np.array([KX, YS, ZH, 1]))[:3]
    print(f" arm {th:+4d} deg: hub Z={hub[2]:+6.2f}  Y={hub[1]:6.2f}  toe={None if ds is None else round(math.degrees(ds),2)} deg  camber={math.degrees(sol['psi']):+.1f}")

# ---------------------------------------------------------------- pose collision sweep
def pose_scene(theta_deg, s):
    sol = solve_front(theta_deg); sc = {n: (m, I) for n, m in static.items()}
    for n in ("Steering_Rack",): pass
    Th_ = rotx(horn_angle(s), YSP, ZSP); Tl_ = link_T(s); Tc_ = crank_T(s); Tr_ = tr([0, s, 0])
    sc["Servo_Horn"] = (parts["Servo_Horn"], Th_)
    sc["HW_Link_Pin_A"] = (refs["HW_Link_Pin_A"], Th_)
    sc["Steering_Drag_Link"] = (parts["Steering_Drag_Link"], Tl_)
    sc["Steering_BellCrank"] = (parts["Steering_BellCrank"], Tc_)
    sc["HW_Crank_In_Pin"] = (refs["HW_Crank_In_Pin"], Tc_)
    sc["HW_Crank_Out_Pin"] = (refs["HW_Crank_Out_Pin"], Tc_)
    sc["Steering_Rack"] = (parts["Steering_Rack"], Tr_)
    sc["HW_TieRod_Inner_Screw_L"] = (hw_tr["HW_TieRod_Inner_Screw_L"], Tr_)
    sc["HW_TieRod_Inner_Screw_R"] = (mirror(hw_tr["HW_TieRod_Inner_Screw_L"]), Tr_)
    for side, sgn in (("L", 1), ("R", -1)):
        ss = s if side == "L" else -s
        d = steer_from_rack(sol, ss, "L")           # mirrored geometry: solve with mirrored rack offset
        if d is None: return None, None
        Tz = rotz(d, KX, YS); Tspin = sol["Tc"] @ Tz
        M = (lambda m: m) if side == "L" else mirror
        def put(name, mesh, T):
            if side == "R":
                Tm = np.diag([1, -1, 1, 1.0]); T2 = Tm @ T @ Tm; mm = mirror(mesh)  # mirror mesh then conjugate transform
                sc[rn(name)] = (mm, T2)
            else: sc[name] = (mesh, T)
        for n, (g, mesh) in moving.items():
            if n.startswith("Ref_Shock") or n.startswith("HW_TieRod_Inner") or g in ("rod", "cvd"): continue
            T = {"arm_l": None, "arm_u": None}.get(g)
            if g == "arm_l": T = rotx(sol["th"], *AL)
            elif g == "arm_u": T = rotx(sol["phi"], *AU)
            elif g == "car": T = sol["Tc"]
            elif g == "spin": T = Tspin
            put(n, mesh, T)
        # CVD inner: from fixed S to joint J
        J = (sol["Tc"] @ np.append(CVD_J0, 1))[:3]; S = CVD_S; v = J - S
        L0 = YS - DOG_J1_Y; Tcv = tr(S - CVD_S) @ trimesh.geometry.align_vectors([0, 1, 0], v / np.linalg.norm(v))
        Tcv = tr(S) @ trimesh.geometry.align_vectors([0, 1, 0], v / np.linalg.norm(v)) @ tr(-S)
        put("CVD_Inner_L", cvd_in, Tcv)
        sc["_plunge_" + side] = (trimesh.creation.box(extents=[0.01] * 3), tr([0, np.linalg.norm(v) - L0, 0]))
        # tie rod rigid between rack end and outer point
        I_ = ROD_IN + np.array([0, ss, 0]); O = (Tspin @ np.append(ROD_OUT, 1))[:3]
        rod_dir = (O - I_); Trod = tr(I_ - ROD_IN) @ trimesh.geometry.align_vectors(ROD_OUT - ROD_IN, rod_dir) @ tr(-ROD_IN)
        # align_vectors is 3D rotation about origin: rotate about ROD_IN
        Trod = tr(I_) @ trimesh.geometry.align_vectors(ROD_OUT - ROD_IN, rod_dir) @ tr(-ROD_IN)
        put("Tie_Rod_L", parts["Tie_Rod_L"], Trod)
        # shock: rotate about upper mount to point at lower-arm mount
        put("Ref_Shock_FL", make_shock(SHOCK_UP, (rotx(sol["th"], *AL) @ np.append(SHOCK_LO0, 1))[:3]), np.eye(4))      # exact per pose (rod slides in the body)
    return sc, sol

# ---------- rack relief: tie rods swept over travel (-12..+8) x lock (+/-3.4) in the RACK frame, cut from the rack top (0.20 clearance; rods tilt down ~9.6 deg at full droop)
_rk = []
for th_ in (-12, -11, -10, -8, -6, -4, 0, 4, 8):
    for s_ in np.linspace(-3.4, 3.4, 18):
        sc_, sol_ = pose_scene(th_, float(s_))
        if sc_ is None: continue
        for k_ in ("Tie_Rod_L", "Tie_Rod_R"):
            m_, T_ = sc_[k_]; mm_ = (_rod_fill if k_.endswith("L") else mirror(_rod_fill)).copy(); mm_.apply_transform(tr([0, -float(s_), 0]) @ T_); _rk.append(mm_)
_rk = U(*_rk); _ok = 0.20
_rk = U(*[_rk.copy().apply_translation(d_) for d_ in ([0, 0, 0], [_ok, 0, 0], [-_ok, 0, 0], [0, _ok, 0], [0, -_ok, 0], [0, 0, -_ok], [_ok * .6, _ok * .6, -_ok * .6], [-_ok * .6, _ok * .6, -_ok * .6], [_ok * .6, -_ok * .6, -_ok * .6], [-_ok * .6, -_ok * .6, -_ok * .6])])
RACK_CH = (38.1, 3.05, 39.6, 2.75, 4.95, 7.6)   # wall fix 2: flat chamfer (X0, Z0) -> (X1, Z1) over |Y| 4.95..7.6 that contains the rod-sweep relief on the bar-end top front edge (the sweep left a 0.09 mm skin at X 39.3..39.4)
_rk = U(_rk, *[trimesh.convex.convex_hull(np.array([p_ for y_ in (s_ * RACK_CH[4], s_ * RACK_CH[5])
                                                    for p_ in ((RACK_CH[0], y_, RACK_CH[1]), (RACK_CH[2], y_, RACK_CH[3]), (RACK_CH[2], y_, 3.4), (RACK_CH[0], y_, 3.4))])) for s_ in (1, -1)])
_rack0 = parts["Steering_Rack"]; _rack1 = D(_rack0, _rk); _rack1 = max(_rack1.split(only_watertight=False), key=lambda s: s.volume)
print(" rack tie-rod relief: removed %.3f mm3 (rod sweep -12..+8 x +/-3.4, clearance %.2f); rack top now Z %.2f at the bosses" % (_rack0.volume - _rack1.volume, _ok, _rack1.bounds[1][2]))
parts["Steering_Rack"] = _rack1; printable["Steering_Rack"] = _rack1; rest_all["Steering_Rack"] = _rack1
# ---------- swept steering relief in the bulkhead (ALL moving steering parts + pins + tie rods over steering x travel), then final rest check
sweep = []
for th_ in (-12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8):
    for s_ in np.linspace(-3.4, 3.4, 23):
        sc_, sol_ = pose_scene(th_, float(s_))
        if sc_ is None: continue
        for k_ in ("Tie_Rod_L", "Tie_Rod_R"):
            m_, T_ = sc_[k_]; mm_ = m_.copy(); mm_.apply_transform(T_); sweep.append(mm_)
_hw_tr_L = hw_tr["HW_TieRod_Inner_Screw_L"]; _hw_tr_R = mirror(_hw_tr_L)
for s_ in np.linspace(-3.4, 3.4, 35):
    Th_ = rotx(horn_angle(s_), YSP, ZSP); Tl_ = link_T(s_); Tc_ = crank_T(s_); Tr_ = tr([0, float(s_), 0])
    for m_, T_ in (
        (parts["Servo_Horn"], Th_),
        (refs["HW_Link_Pin_A"], Th_),
        (refs["HW_Link_Pin_A"], Tl_),
        (parts["Steering_Drag_Link"], Tl_),
        (parts["Steering_BellCrank"], Tc_),
        (refs["HW_Crank_In_Pin"], Tc_),
        (refs["HW_Crank_Out_Pin"], Tc_),
        (parts["Steering_Rack"], Tr_),
        (_hw_tr_L, Tr_),
        (_hw_tr_R, Tr_),
    ):
        mm_ = m_.copy(); mm_.apply_transform(T_); sweep.append(mm_)
# upper arm (+ bump to +12 deg, droop to -4) sweeps through the cradle floor: cut it so the floor can sit low
for th_ in np.arange(-4, 12.1, 2):
    sol_ = solve_front(float(th_)); mm_ = parts["IFS_Upper_Arm_L"].copy(); mm_.apply_transform(rotx(sol_["phi"], *AU)); sweep.append(mm_)
cutv = U(*sweep)
# 26-direction anisotropic dilation (axes, edge diagonals, space diagonals): real gap >=0.25 mm in EVERY direction incl. 3D corners; 0.40 mm down (floor under the bell crank Z=-0.15, 0.75 mm solid above the Z=-0.9 tub interface)
_cxy, _czp, _czm = 0.46, 0.46, 0.40
_s2 = 1.0 / math.sqrt(2.0)
_dirs = [
    (0.0, 0.0, 0.0),
    (_cxy, 0, 0), (-_cxy, 0, 0), (0, _cxy, 0), (0, -_cxy, 0), (0, 0, _czp), (0, 0, -_czm),
    (_cxy * _s2, _cxy * _s2, 0), (_cxy * _s2, -_cxy * _s2, 0), (-_cxy * _s2, _cxy * _s2, 0), (-_cxy * _s2, -_cxy * _s2, 0),
    (_cxy * _s2, 0, _czp * _s2), (-_cxy * _s2, 0, _czp * _s2), (_cxy * _s2, 0, -_czm * _s2), (-_cxy * _s2, 0, -_czm * _s2),
    (0, _cxy * _s2, _czp * _s2), (0, -_cxy * _s2, _czp * _s2), (0, _cxy * _s2, -_czm * _s2), (0, -_cxy * _s2, -_czm * _s2),
] + [(sx_ * _cxy / math.sqrt(3.0), sy_ * _cxy / math.sqrt(3.0), (_czp if sz_ > 0 else -_czm) / math.sqrt(3.0)) for sx_ in (1, -1) for sy_ in (1, -1) for sz_ in (1, -1)]   # + 8 space diagonals: the 18-direction set left only ~0.21 mm at 3D corners
infl = U(*[cutv.copy().apply_translation(d_) for d_ in _dirs])
# no-step cleanup of the bell-crank pocket (owner): the 26-direction dilation left terraces at the foot of every wall (floor -0.15 / -0.03 / 0.02 / 0.25, shelf 1.20 .. 1.40).
# A thin slab of the relief at the height where its outline is full width is stretched straight down to the floor level: vertical walls, one floor, material removed only.
def _zslab(m_, x0_, x1_, y0_, y1_, za_, zb_, z0_):
    s_ = trimesh.boolean.intersection([m_, box(x0_, x1_, y0_, y1_, za_, zb_)], engine=ENG)
    if not len(s_.faces): return None
    k_ = (zb_ - z0_) / (zb_ - za_); M_ = np.eye(4); M_[2, 2] = k_; M_[2, 3] = zb_ * (1 - k_); s_.apply_transform(M_); return s_
CRK_WIN = (26.0, 41.0, -13.3, 0.0)      # X0, X1, Y0, Y1 of the cleaned area
_flat = [p_ for p_ in (_zslab(infl, *CRK_WIN, CK_Z0 + 0.01, CK_Z0 + 0.06, CK_Z0 - _czm), _zslab(infl, *CRK_WIN, PL_Z0 + 0.01, PL_Z0 + 0.04, PL_Z0 - _czm)) if p_ is not None]
if _flat: infl = U(infl, *_flat)
bk = D(parts["IFS_Bulkhead_Convertible"], infl)
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)   # drop zero-volume slivers
bk = D(bk, cyl("z", [GUIDE_X, 0.0, 0.3], 0.55, 4.8, 16))   # rack guide-screw pilot (opened after the rack channel is cut)
# clear crank hub disc cleanly above floor boss Z=0.10 (eliminates tangent slivers between rotated sweep copies)
bk = D(bk, cyl("z", [XP, YPC, 1.05], 2.8, 1.9, 40))
bk = D(bk, D(cyl("z", [XP, YPC, (CK_Z0 - _czm + 0.1) / 2], 2.8, 0.1 - (CK_Z0 - _czm), 48), cyl("z", [XP, YPC, 0.0], 2.19, 1.0, 48)))   # no-step: ring r 2.2..2.8 around the pivot boss down to the pocket floor, so the boss is one clean dia 4.4 cylinder (it had ragged 0.25 mm shelves where the crank sweep did not reach)
# upper shock fork tower (Item 1 preserved: SHOCK_UP=[58.6,12.0,20.5], >=1.0 mm shock clearances) + Axial Yeti (AX31111) top bridge & adjustable front body posts
TW_W, TW_R, TW_Y0, TW_Y1, TW_Z0, TW_ZG = 0.8, 1.9, 5.0, 7.2, 11.0, 17.0
for _xs in (SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1 - TW_W, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1):
    _stem = box(_xs, _xs + TW_W, TW_Y0, TW_Y1, TW_Z0, SHOCK_UP[2] + TW_R)
    _arm = trimesh.convex.convex_hull(np.vstack([cyl("x", [_xs + TW_W / 2, SHOCK_UP[1], SHOCK_UP[2]], TW_R, TW_W, 28).vertices, box(_xs, _xs + TW_W, TW_Y0, TW_Y1, TW_ZG, SHOCK_UP[2] + TW_R).vertices]))
    _w = U(_stem, _arm)
    bk = U(bk, _w, mirror(_w))
# Yeti AX31111 shock-tower bridge + arched underside gussets + two MAGNET SEATS for the shell (user change: no body posts, no clip holes; the hood carries matching magnet bosses, no through-hole)
# seat = boss dia 4.8 with an upward-open pocket dia 3.15 x 2.1 for a dia 3.0 x 2.0 neodymium disc (wall 0.825, solid floor), flat horizontal top 0.4 mm below the hood inner surface, dia 1.0 push-out hole from below
MAG_D, MAG_H = 3.0, 2.0                       # magnet disc
MAG_PD, MAG_PDEPTH, MAG_BOSS_D = 3.2, 2.1, 4.8      # pocket = magnet dia + 0.2 (playbook); wall (4.8 - 3.2) / 2 = 0.80
MAG_HOOD_Z = 27.6                             # hood inner surface at the seat XY (stl/v4/Tacoma_Trophy_Body_Shell.stl, measured; re-measured and printed in the FINAL AUDIT)
MAG_X, MAG_Y, MAG_TOP = 57.3, 7.6, MAG_HOOD_Z - 0.4   # seat centre (X, +-Y) and flat top Z; boss front edge X 59.7 stays behind the shell nose inner wall (X >= ~60.3)
MAG_PUSH = (58.3, 7.85)                       # push-out hole axis: inside the pocket floor, exits the bridge underside (Z 20.0) in the open fork gap X 57.8..59.4, outboard of the arch gusset (Y <= 7.2)
_tx0, _tx1 = SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1 - TW_W, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1 + TW_W   # 57.0 .. 60.2
BRIDGE_Z1 = 22.4
_bar = trimesh.convex.convex_hull(np.array(
    [(x_, y_, z_) for x_ in (_tx0 + 0.3, _tx1 - 0.3) for y_ in (-8.2, 8.2) for z_ in (20.0, BRIDGE_Z1)] +
    [(x_, y_, z_) for x_ in (_tx0, _tx1) for y_ in (-8.5, 8.5) for z_ in (20.3, BRIDGE_Z1 - 0.3)]
))
_arch_l = trimesh.convex.convex_hull(np.array(
    [(x_, 5.0, z_) for x_ in (_tx0 + 0.1, _tx1 - 0.1) for z_ in (16.5, 20.3)] +
    [(x_, 7.2, z_) for x_ in (_tx0 + 0.1, _tx1 - 0.1) for z_ in (16.5, 20.3)] +
    [(x_, 1.5, z_) for x_ in (_tx0 + 0.1, _tx1 - 0.1) for z_ in (20.0, 20.3)]
))
_mag_seat_l = U(
    cyl("z", [MAG_X, MAG_Y, (24.0 + MAG_TOP) / 2], MAG_BOSS_D / 2, MAG_TOP - 24.0, 40),                                               # boss dia 4.8, Z 24.0..top
    trimesh.convex.convex_hull(np.vstack([cyl("z", [MAG_X, MAG_Y, 24.0], MAG_BOSS_D / 2, 0.02, 40).vertices,
                                          box(_tx0, MAG_X + MAG_BOSS_D / 2, 5.2, 8.5, 20.3, BRIDGE_Z1 - 0.3).vertices])),             # gusset from the bridge body up to the boss (rear overhang <= 30 deg from vertical)
)
_mag_cut = [m_ for s_ in (1, -1) for m_ in (cyl("z", [MAG_X, s_ * MAG_Y, MAG_TOP - MAG_PDEPTH / 2 + 0.5], MAG_PD / 2, MAG_PDEPTH + 1.0, 40),     # pocket, open upward
                                             cyl("z", [MAG_PUSH[0], s_ * MAG_PUSH[1], (19.0 + MAG_TOP - MAG_PDEPTH) / 2 + 0.05], 0.5, MAG_TOP - MAG_PDEPTH - 19.0 + 0.1, 20))]   # dia 1.0 push-out hole
bk = D(bk, *[box(56.3, 57.9, min(s_ * 8.15, s_ * 10.6), max(s_ * 8.15, s_ * 10.6), 8.65, 11.8) for s_ in (1, -1)])   # wall fix: remains of the old upper shock mount (block X 56.6..57.5, |Y| 8.2..10.3, Z 8.9..11.5 hollowed to 0.3 mm shells by the shock relief) removed
# wall fix 2 (audit final2), leftovers of the steering relief, material removed only:
UHB_SLOT = (38.3, 41.0, 0.72, 5.4, 6.6)   # X0, X1, half width, Z0, Z1: floor under the upper hinge-pin bore tail (0.4 -> 0.05 mm feather) opened into a slot narrower than the pin
bk = D(bk, *[box(UHB_SLOT[0], UHB_SLOT[1], s_ * AU[0] - UHB_SLOT[2], s_ * AU[0] + UHB_SLOT[2], UHB_SLOT[3], UHB_SLOT[4]) for s_ in (1, -1)],
       box(23.9, 25.6, 0.9, 1.9, 7.75, 8.8),        # knife-edge fin beside the servo case (0.02-0.2 mm above Z 7.75) truncated to a 0.65 mm flat
       box(35.0, 37.25, -2.75, 2.95, 1.16, 2.0), box(37.2, 38.0, 0.7, 2.0, 1.36, 2.0))    # 0.1 mm relief ridges on the solid rack deck behind the rack flattened to Z 1.16
RET_POCKET = (57.05, 58.25, 9.2, 13.5, -2.2, 5.3)   # plate envelope (X 57.05..58.05, |Y| 9.4..13.3, Z -2.0..5.1) + 0.2: a nose corner at (57.5..58.05, 10.0..10.5, 4.0..4.9) overlapped the thickened plate by 0.023 mm3
bk = D(bk, *[m_ for s_ in (1, -1) for m_ in (box(RET_POCKET[0], RET_POCKET[1], min(s_ * RET_POCKET[2], s_ * RET_POCKET[3]), max(s_ * RET_POCKET[2], s_ * RET_POCKET[3]), RET_POCKET[4], RET_POCKET[5]),
                                             cyl("x", [58.5, s_ * 11.6, 3.4], 1.4, 1.0, 32))])   # + screw head (dia 2.4 x 0.7 at X 58.05..58.75) with 0.2 around it
bk = U(bk, _bar, _arch_l, mirror(_arch_l), _mag_seat_l, mirror(_mag_seat_l))
bk = D(bk, cyl("x", [SHOCK_UP[0], SHOCK_UP[1], SHOCK_UP[2]], 0.85, 8.0, 28), cyl("x", [SHOCK_UP[0], -SHOCK_UP[1], SHOCK_UP[2]], 0.85, 8.0, 28), *_mag_cut)
print(" shock fork tower + magnet seats: centres X=%.1f, Y=+-%.1f, flat top Z=%.2f (hood inner %.2f - 0.4); magnet dia %.1f x %.1f; pocket dia %.2f x %.1f deep (floor Z %.2f), boss dia %.1f (wall %.3f), push-out dia 1.0 at (%.2f, +-%.2f)" % (MAG_X, MAG_Y, MAG_TOP, MAG_HOOD_Z, MAG_D, MAG_H, MAG_PD, MAG_PDEPTH, MAG_TOP - MAG_PDEPTH, MAG_BOSS_D, (MAG_BOSS_D - MAG_PD) / 2, MAG_PUSH[0], MAG_PUSH[1]))
# bell-crank floor pivot boss (Z=-0.90..+0.10, 0.15 mm below bellcrank bottom 0.25) with clean through-pilot Ø1.10; wire-clip pad; rear bottom bevel fill at X=24.0..24.6 flat on Z=-0.90
WEB_Z0, WEB_TOP, SEAT_TOP = -0.9, 0.5, 0.8     # UPDATE 14: floor web underside Z=-0.9 (meets the raised tub); web top 0.5 (diff ring bottom Z=0.85); screw seats up to 0.8
bk = U(bk, cyl("z", [XP, YPC, -0.40], 2.2, 1.0, 32),                                               # owner request: Servo_Wire_Clip, its pad and its two M1.0 pilots removed
       box(24.0, 24.6, -13.0, 13.0, WEB_Z0, 0.2))                                                                                    # eliminates 0.10 mm rear bottom bevel at X=24.0..24.2 (flush Z=-0.90 with tub!)
bk = D(bk, cyl("z", [XP, YPC, -0.35], 0.55, 1.6, 24))
# ---- UPDATE 12 (Yeti-style bumper + Y-brace): mount pad under the nose floor + two screw posts floor->top skin
BMX, BMY = 61.8, 3.0                    # bumper through-bolt axes (X, +-Y): vertical, through plate, floor, post, top skin and the Y-brace ear
BPAD_Z0 = 2.9                           # flat underside of the pad = flat top of the bumper centre plate
# HARDWARE UPSIZE (owner): bumper through-bolts M1.4 -> M2 x 14 cheese head (ISO 1207: head dia 3.8 x 1.3) + M2 hex nut (4.0 AF x 1.6) on the Y-brace
BM_CLR_R, BM_HEAD_R, BM_HEAD_H, BM_CB_R, BM_POST_R = 1.1, 1.9, 1.3, 2.0, 1.9     # dia 2.2 clearance, head, dia 4.0 counterbore, post dia 3.8 (0.8 wall around the bore; front X 63.70, 0.2 behind the bumper foot strip)
BM_NUT_R, BM_NUT_H = 2.31, 1.6          # M2 nut: 4.0 across flats = 4.62 across corners
BM_POST_Z0 = BPAD_Z0   # wall fix 2: posts start at the pad underside (were Z 5.25: between pad top 4.3 and 5.25 the bore crossed an old slot X 61.0..61.5 with a 0.06 mm skin; pad front wall at the bore 0.70 -> 0.80)
NOSE_FILL = (55.8, 56.5, 4.3, 0.4, 1.2)   # wall fix 2: X0, X1, |Y|, Z0, Z1 of the fill behind the 0.23 mm sloped nose skin (closes the 0.29 mm slot between it and the front wall)
bk = U(bk, box(60.2, 63.6, -8.5, 8.5, BPAD_Z0, 4.3), *[cyl("z", [BMX, s_ * BMY, (BM_POST_Z0 + 11.15) / 2], BM_POST_R, 11.15 - BM_POST_Z0, 64) for s_ in (1, -1)],
       box(NOSE_FILL[0], NOSE_FILL[1], -NOSE_FILL[2], NOSE_FILL[2], NOSE_FILL[3], NOSE_FILL[4]))
bk = D(bk, *[cyl("z", [BMX, s_ * BMY, 7.8], BM_CLR_R, 11.8, 48) for s_ in (1, -1)])
BH_HOLES = [(44.5, 3.5), (44.5, -3.5), (52.0, 3.5), (52.0, -3.5)]
BOSS_REAR_X = 39.9   # wall fix: rear wall of the lower hinge bosses (below Z -0.9) 0.28 -> 0.88 mm, and solid under the hinge-pin bore tail X 40.0..40.5
bk = D(bk, box(24.0, BOSS_REAR_X, -14.0, 14.0, -6.0, WEB_Z0), box(BOSS_REAR_X, 64.0, -8.5, 8.5, -6.0, WEB_Z0), box(BOSS_REAR_X, 40.5, 13.0, 14.0, -6.0, WEB_Z0), box(BOSS_REAR_X, 40.5, -14.0, -13.0, -6.0, WEB_Z0))      # trim the underside (hinge-pin bosses at |Y|>8.5 untouched)
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)
BH_SEAT_R = 2.4   # wall fix 3: seat boss dia 4.2 -> 4.8, ring around the dia 3.2 counterbore 0.49 -> 0.80 mm
bk = U(bk, *[cyl("z", [hx_, hy_, (0.2 + SEAT_TOP) / 2], BH_SEAT_R, SEAT_TOP - 0.2, 28) for (hx_, hy_) in BH_HOLES])
BH_FLOOR_Z0, BH_FLOOR_Z1 = WEB_Z0, SEAT_TOP
for (hx_, hy_) in BH_HOLES:
    ok_ = bool(bk.contains([[hx_ + 1.3, hy_, 0.0], [hx_ - 1.3, hy_, 0.0], [hx_, hy_ + 1.3, 0.0], [hx_, hy_ - 1.3, 0.0]]).all())
    print("   tub screw hole (%.1f, %.1f): floor solid around it at Z 0? %s" % (hx_, hy_, ok_))
    bk = D(bk, cyl("z", [hx_, hy_, 0.0], 0.9, 3.4, 24), cyl("z", [hx_, hy_, SEAT_TOP - 0.45 + 0.9], 1.6, 1.8, 28))   # Ø1.8 through, Ø3.2 counterbore 0.45 deep from the seat top
for (py_, pz_) in PILOTS:   # re-open the ear-screw pilots (clean through-pilot along X, no 0.05 mm membrane)
    bk = D(bk, cyl("x", [EARW_X1 - 1.1, py_, pz_], 0.45, 2.25, 24))
    print("   ear pilot (Y %.1f, Z %.1f): wall solid 0.6 mm beside it? %s" % (py_, pz_, bool(bk.contains([[EARW_X1 - 1.1, py_ + 0.95, pz_]])[0])))
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)   # drop slivers again
# front final (4): the swept relief left a terraced dish (Z 0.18/0.30/0.40/0.28 steps, a 0.05 mm wide rib at X 35.0). Flat floor Z 0.10 for X 34.30..36.55 (starts inside the continuous 0.18 floor, ends on the
# rising wall past X 36.45), floor stays 1.0 mm over the tub interface (Z -0.9): removes material only, so link/horn/crank clearance can only grow
DISH_CUT = box(34.30, 36.55, 3.9, 9.1, 0.10, 1.0)
_bk_d = D(bk, DISH_CUT, mirror(DISH_CUT)); print(' dish flat floor: removed %.3f mm3 from the bulkhead' % (bk.volume - _bk_d.volume)); bk = max(_bk_d.split(only_watertight=False), key=lambda s: s.volume)
parts["IFS_Bulkhead_Convertible"] = bk; static["IFS_Bulkhead_Convertible"] = bk; rest_all["IFS_Bulkhead_Convertible"] = bk
# ---------- spindle steering-arm seat relief: the flat tie rod rocks ~9.6 deg on its outer pivot at full droop; cut its sweep (spindle frame, 0.10 clearance) from the steering-arm pad top
def _dil(m_, o_, dirs_=None):
    dirs_ = dirs_ or [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] + [(sx_ * .7, sy_ * .7, 0) for sx_ in (1, -1) for sy_ in (1, -1)] + [(sx_ * .58, sy_ * .58, sz_ * .58) for sx_ in (1, -1) for sy_ in (1, -1) for sz_ in (1, -1)]
    return U(m_, *[m_.copy().apply_translation([o_ * d_[0] + 0.0007 * (i_ % 3 - 1), o_ * d_[1] + 0.0005 * (i_ % 2), o_ * d_[2] + 0.0003 * (i_ % 4 - 1.5)]) for i_, d_ in enumerate(dirs_)])
_rs = []
for th_ in (-12, -10, -8, -4, 0, 4, 8):
    for s_ in np.linspace(-3.0, 3.0, 13):
        sc_, sol_ = pose_scene(th_, float(s_))
        if sc_ is None: continue
        m_ = _rod_fill.copy(); m_.apply_transform(np.linalg.inv(sc_["Front_Spindle_L"][1]) @ sc_["Tie_Rod_L"][1]); _rs.append(m_)
_cut = trimesh.boolean.intersection([_dil(U(*_rs), 0.22), box(39.5, 46.6, 30.0, 39.6, 2.5, 3.2)], engine=ENG)   # window from X 39.5 (was 40.0: left a 0.06 mm rim on the boss edge at X 39.94)
# front final (5): the dilated rod sweep left a ragged crescent rim (0.1..0.3 mm walls) on the steering-arm boss and on the bar edge. Level it: the footprint of the cut at the pad top is stretched straight
# down to one flat floor (SP_LEVEL_Z = lowest point of the dilated sweep, 2.59 -> 2.60, so the rod clearance is unchanged) and the boss + bar corner are cleared as one pocket (X 39.5..43.8, Y 32.4..36.6)
SP_LEVEL_Z = 2.60
_sp_sl = _zslab(_cut, 39.0, 46.0, 30.0, 39.6, 3.15, 3.19, SP_LEVEL_Z)
_cut = U(*[q_ for q_ in (_cut, _sp_sl, box(39.5, 43.8, 32.4, 36.6, SP_LEVEL_Z, 3.4)) if q_ is not None]) if len(_cut.faces) else _cut
_sp0 = parts["Front_Spindle_L"]; _sp1 = D(_sp0, _cut) if len(_cut.faces) else _sp0; _sp1 = max(_sp1.split(only_watertight=False), key=lambda s: s.volume)
print(" spindle tie-rod seat relief: removed %.3f mm3 (rod sweep arm -12..+8 x lock, clearance 0.22)" % (_sp0.volume - _sp1.volume))
parts["Front_Spindle_L"] = _sp1; moving["Front_Spindle_L"] = ("spin", _sp1); printable["Front_Spindle_L"] = _sp1; printable["Front_Spindle_R"] = mirror(_sp1); rest_all["Front_Spindle_L"] = _sp1; rest_all["Front_Spindle_R"] = mirror(_sp1)
# ---------- lower-arm relief: spindle + tie-rod outer screw swept over droop (-12..-7) x steering lock, cut from the lower arm (window around the contact only)
ARM_RELIEF_CLR = 0.38   # was 0.28: the outer tie-rod screw sits 0.10 lower now (ROD_SEAT_DROP) and its tip is 0.05 proud of the pad; keeps screw to lower arm at the earlier level
_sp = []
for th_ in (-12, -11, -10, -9, -8, -7):
    for s_ in (-3.0, -2.8, -2.4, -2.0, -1.5, -1.0, 0.0, 1.0, 1.5, 2.0, 2.4, 2.8, 3.0):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        Tr_ = np.linalg.inv(sc_["IFS_Lower_Arm_L"][1]) @ sc_["Front_Spindle_L"][1]
        for m0_ in (parts["Front_Spindle_L"], hw_tr["HW_TieRod_Outer_Screw_L"]):
            m_ = m0_.copy(); m_.apply_transform(Tr_); _sp.append(m_)
_o = ARM_RELIEF_CLR; _sw = _dil(U(*_sp), _o)
_win = box(43.0, 47.5, 26.5, 31.3, -1.0, 4.5)   # arm-local frame (arm at rest; stops at Y=31.3 clear of outer hinge bore Y>=31.65)
_cut = trimesh.boolean.intersection([_sw, _win], engine=ENG)
# second window: the spindle's lower kingpin ear passes under the two prong ends (beyond the hinge bore, Y>=33.4) with only 0.17 mm at ride / 0.09 at -8 deg: relieve the prong-end undersides over arm -12..0
_sp2 = []
for th_ in (-12, -10, -8, -6, -4, -2, 0):
    for s_ in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        m_ = parts["Front_Spindle_L"].copy(); m_.apply_transform(np.linalg.inv(sc_["IFS_Lower_Arm_L"][1]) @ sc_["Front_Spindle_L"][1]); _sp2.append(m_)
_cut2 = trimesh.boolean.intersection([_dil(U(*_sp2), _o), U(box(44.1, 46.4, 33.4, 34.5, -1.6, -0.9), box(49.6, 51.9, 33.4, 34.5, -1.6, -0.9))], engine=ENG)
print("  prong-end relief cut: vol %.3f" % (_cut2.volume if len(_cut2.faces) else 0))
if len(_cut2.faces): _cut = U(_cut, _cut2) if len(_cut.faces) else _cut2
print("  relief cut: %d faces, vol %.3f" % (len(_cut.faces), _cut.volume if len(_cut.faces) else 0))
_la = D(parts["IFS_Lower_Arm_L"], _cut) if len(_cut.faces) else parts["IFS_Lower_Arm_L"]; _la = max(_la.split(only_watertight=False), key=lambda s: s.volume)
print(" lower-arm relief: removed %.2f mm3 (spindle + outer screw sweep, droop -12..-7 x lock +/-3, clearance %.2f)" % (parts["IFS_Lower_Arm_L"].volume - _la.volume, _o))
parts["IFS_Lower_Arm_L"] = _la; moving["IFS_Lower_Arm_L"] = ("arm_l", _la); printable["IFS_Lower_Arm_L"] = _la; printable["IFS_Lower_Arm_R"] = mirror(_la)
rest_all["IFS_Lower_Arm_L"] = _la; rest_all["IFS_Lower_Arm_R"] = mirror(_la)
# ---------- droop stop: the lower arm's inboard bottom edge lands on the bulkhead hinge boss. Cut the bulkhead with the arm posed at -12.0 / -12.25 / -12.5 deg: a clean conforming stop face, contact at -12.5 deg, 0 overlap at -12
DROOP_STOP = -12.5
_ds = U(*[_la.copy().apply_transform(rotx(math.radians(t_), *AL)) for t_ in (-12.0, -12.25, DROOP_STOP)])
_bk0 = parts["IFS_Bulkhead_Convertible"]; bk = D(_bk0, _ds, mirror(_ds)); bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)
print(" droop-stop face: removed %.3f mm3 from the bulkhead (lower arm at %.2f deg); hub Z at the stop %.2f" % (_bk0.volume - bk.volume, DROOP_STOP, 3.0 + math.sin(math.radians(DROOP_STOP)) * (PL[0] - AL[0])))
parts["IFS_Bulkhead_Convertible"] = bk; static["IFS_Bulkhead_Convertible"] = bk; rest_all["IFS_Bulkhead_Convertible"] = bk
# ---------- wall fix 3 (wall scan < 0.78 mm, 45 clusters): thin sheets thickened to >= 0.8 or removed; added material is kept out of the steering relief (infl) and stays inside the X faces of the existing bosses
_ybox = lambda x0_, x1_, s_, y0_, y1_, z0_, z1_: box(x0_, x1_, min(s_ * y0_, s_ * y1_), max(s_ * y0_, s_ * y1_), z0_, z1_)
_bk3 = bk
TUN_ROOF = (39.5, 42.98, 2.0, 9.9, 10.4)          # X0, X1, |Y|, Z0, Z1: pad on the web over the shaft tunnel (roof 0.41-0.64 -> 0.86-1.09)
UHF_DBL = (55.2, 57.0, 12.55, 12.8, (0.25, 0.45)) # X0, X1, Y0, Y1 of the solid outboard skin of the front hinge boss, copied outboard by these steps: wall outside the upper pin bore 0.40-0.48 -> 0.80, outside the retainer-screw pilot 0.50-0.55 -> 1.0
UHR_DBL = (38.3, 40.8, 12.75, 13.0, (0.25,))      # same for the rear upper hinge boss (pin tail X 38.6..40.5): 0.52-0.68 -> 0.80
MAG_WEB = (57.75, 59.45, 8.3, 9.2, 20.0, BRIDGE_Z1)   # web between the fork arms outboard of the dia 1.0 push-out hole (wall 0.08-0.43 -> 0.85); 1.3 mm inboard of the shock eye sweep
bk = U(bk, box(TUN_ROOF[0], TUN_ROOF[1], -TUN_ROOF[2], TUN_ROOF[2], TUN_ROOF[3], TUN_ROOF[4]), *[_ybox(MAG_WEB[0], MAG_WEB[1], s_, *MAG_WEB[2:]) for s_ in (1, -1)],
       *[D(U(*[trimesh.boolean.intersection([_bk3, _ybox(d_[0], d_[1], s_, d_[2], d_[3], z0_, z1_)], engine=ENG).apply_translation([0, s_ * dy_, 0]) for dy_ in d_[4]]), infl)
         for s_ in (1, -1) for (d_, z0_, z1_) in ((UHF_DBL, 5.9, 7.7), (UHF_DBL, 2.6, 4.3), (UHR_DBL, 5.9, 7.7))])
LHB_SLOT = (45.75, 50.25, 0.8, 0.0, 2.1)          # X0, X1, half width, Z0, Z1: feathered roof (0.02-0.3) of the lower hinge-pin bore on the centre boss opened to a slot; ends on a 1.0 mm high face
bk = D(bk, box(59.2, 60.25, -4.0, 4.0, 1.9, BPAD_Z0),                                                              # nose: wedge hanging under the pad (0.0-0.8, median 0.43) removed, pad underside flat from X 59.3
       *[_ybox(49.45, 53.0, s_, 5.7, 6.02, 10.45, 11.0) for s_ in (1, -1)],                                         # 0.2 wide x 0.5 thick lip at |Y| 5.8..6.0 removed
       *[_ybox(RET_POCKET[0], RET_POCKET[1], s_, RET_POCKET[2], 10.6, RET_POCKET[5], 6.1) for s_ in (1, -1)],      # 0.43-0.70 roof over the retainer-plate pocket removed (its top is the shock relief, its underside the plate + 0.2)
       *[cyl("x", [EARW_X0 + 0.15, py_, pz_], 0.45, 0.5, 24) for (py_, pz_) in PILOTS],                             # ear pilots: 0.17 mm blind floor removed (through pilots)
       box(29.7, 30.05, 14.1, 15.0, 7.65, 9.55),                                                                    # ear wall: 0.25 mm skin behind the cage-tube keep-out removed
       *[_ybox(LHB_SLOT[0], LHB_SLOT[1], s_, AL[0] - LHB_SLOT[2], AL[0] + LHB_SLOT[2], LHB_SLOT[3], LHB_SLOT[4]) for s_ in (1, -1)])
bk = D(bk, *[D(_ybox(46.35, 46.75, s_, 8.9, 12.9, 5.0, 5.4), cyl("x", [46.55, s_ * AU[0], AU[1]], UHB_BOSS[2], 1.0, 48)) for s_ in (1, -1)],   # wall fix 3, knife edges of the steering relief (0.01-0.35), material removed only: rib under the upper hinge centre boss (ring kept)
       box(46.35, 46.70, 9.3, 12.0, 8.15, 8.75), box(49.08, 49.45, 9.3, 12.0, 8.15, 8.75), box(55.15, 55.50, 9.3, 11.4, 7.9, 8.75),    # overhanging lips at Z 8.4..8.7 on the boss end faces squared off
       *[_ybox(55.95, 56.35, s_, 8.1, 9.0, 8.7, 9.3) for s_ in (1, -1)],                                           # 0.2 mm flake left of the old upper shock mount
       box(33.3, 35.25, -11.5, -9.2, 1.25, 2.05), box(34.28, 35.6, -5.3, -2.7, 0.9, 1.25),                           # bell-crank pocket: 0.2 mm shelf (4.75 mm2) and 0.06-0.2 mm ledge
       box(34.95, 36.2, -13.1, -11.45, -0.14, 3.0), box(38.0, 38.75, -13.1, -12.15, -0.14, 1.2),                    # bell-crank pocket: 0.18-0.3 mm curtains on the outer wall, cut back to where the wall is >= 0.7
       box(33.65, 34.4, 0.2, 2.7, 1.3, 2.0), box(37.2, 39.0, -0.95, 1.15, 1.16, 2.0),                               # rack deck: 0.5 mm lip in front of the servo pocket; 0.2-0.5 mm rim around the guide pilot flattened to the deck (Z 1.16)
       box(36.25, 36.85, 2.9, 4.25, 1.31, 2.0), box(36.35, 36.8, 9.15, 10.3, 1.38, 1.8), box(34.6, 35.15, 9.1, 10.5, 0.8, 2.1), box(34.4, 35.15, 2.95, 3.95, 0.74, 1.4),   # 0.05-0.2 mm fins around the dish
       box(40.5, 40.85, -10.4, -8.9, 5.6, 5.95), box(40.45, 40.85, 9.0, 10.9, 1.62, 2.1), *[_ybox(39.85, 40.0, s_, 2.2, 4.3, 3.3, 3.7) for s_ in (1, -1)],   # teeth / fin / flakes at X 40..40.8
       box(40.5, 40.85, 8.9, 10.4, 5.6, 5.95), box(40.6, 41.3, 12.3, 13.05, 8.5, 9.2))   # wall fix 4: +Y twin of the tooth (0.1 wide, median 0.15) and the feathered lower edge of the overhang at X 40.7, Y 12.6..13 (0.09-0.37) squared off at Z 9.2
CRK_FLOOR_UP = 0.05   # wall fix 4: bell-crank pocket floor is a free skin in the fused tub (18_tub.py removes the tub floor under the bulkhead): 0.75 -> 0.80 mm by raising its top Z -0.15 -> -0.10 (crank / pin clearance to the floor 0.40 / 0.45 -> 0.35 / 0.40, audit limit 0.25)
_fl = trimesh.boolean.intersection([bk, box(CRK_WIN[0], 39.95, CRK_WIN[2], -2.8, WEB_Z0 - 0.1, CK_Z0 - _czm)], engine=ENG)   # everything below the floor top (floor + wall feet, pivot pilot stays open), stops behind the lower hinge-pin bore (X 40.0)
bk = U(bk, _fl.apply_translation([0, 0, CRK_FLOOR_UP]))
print(" wall fix 3: bulkhead vol %.2f -> %.2f mm3, shells %d" % (_bk3.volume, bk.volume, len(bk.split(only_watertight=False))))
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)
parts["IFS_Bulkhead_Convertible"] = bk; static["IFS_Bulkhead_Convertible"] = bk; rest_all["IFS_Bulkhead_Convertible"] = bk
# ---------- upper-arm relief: tie rod swept over high bump (+4..+8) x steering lock (+/-3.0), cut from upper arm rear-inner corner (keeps >=0.25 mm clearance at th=+8, s=+-3.0)
_tr_u = []
for th_ in (4, 5, 6, 7, 8):
    for s_ in (-3.0, -2.5, -2.0, 2.0, 2.5, 3.0):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        Tr_ = np.linalg.inv(sc_["IFS_Upper_Arm_L"][1]) @ sc_["Tie_Rod_L"][1]; m_ = parts["Tie_Rod_L"].copy(); m_.apply_transform(Tr_); _tr_u.append(m_)
_sw_u = U(*_tr_u); _ou = 0.28
_sw_u = U(*[_sw_u.copy().apply_translation(d_) for d_ in ([_ou, 0, 0], [-_ou, 0, 0], [0, _ou, 0], [0, -_ou, 0], [0, 0, _ou], [0, 0, -_ou], [0, 0, 0])])
_win_u = box(40.5, 43.0, 9.5, 16.0, 4.5, 5.88)
_cut_u = trimesh.boolean.intersection([_sw_u, _win_u], engine=ENG)
# second cut: the tie-rod outer screw head passes under the rear prong's bottom-rear edge (X 44.2) at full droop + lock (0.06 mm): chamfer-like relief limited to Z <= 5.75 (hinge bore bottom is 5.95)
_su2 = []
for th_ in (-12, -11, -10, -8, -6, -4):
    for s_ in (-3.0, -2.8, -2.4, -2.0, -1.5, -1.0, 0.0, 1.0, 2.0, 3.0):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        Tr_ = np.linalg.inv(sc_["IFS_Upper_Arm_L"][1]) @ sc_["HW_TieRod_Outer_Screw_L"][1]; m_ = hw_tr["HW_TieRod_Outer_Screw_L"].copy(); m_.apply_transform(Tr_); _su2.append(m_)
        m_ = parts["Tie_Rod_L"].copy(); m_.apply_transform(np.linalg.inv(sc_["IFS_Upper_Arm_L"][1]) @ sc_["Tie_Rod_L"][1]); _su2.append(m_)
_cut_u2 = trimesh.boolean.intersection([_dil(U(*_su2), 0.34), box(44.1, 46.4, 28.4, 32.9, 5.2, 5.75)], engine=ENG)
print("  upper-arm prong relief (outer screw head + rod at droop): vol %.3f" % (_cut_u2.volume if len(_cut_u2.faces) else 0))
if len(_cut_u2.faces): _cut_u = U(_cut_u, _cut_u2) if len(_cut_u.faces) else _cut_u2
_ua = D(parts["IFS_Upper_Arm_L"], _cut_u) if len(_cut_u.faces) else parts["IFS_Upper_Arm_L"]; _ua = max(_ua.split(only_watertight=False), key=lambda s: s.volume)
print(" upper-arm tie-rod relief: removed %.2f mm3 (bump +4..+8 x lock +/-3, clearance %.2f)" % (parts["IFS_Upper_Arm_L"].volume - _ua.volume, _ou))
parts["IFS_Upper_Arm_L"] = _ua; moving["IFS_Upper_Arm_L"] = ("arm_u", _ua); printable["IFS_Upper_Arm_L"] = _ua; printable["IFS_Upper_Arm_R"] = mirror(_ua)
rest_all["IFS_Upper_Arm_L"] = _ua; rest_all["IFS_Upper_Arm_R"] = mirror(_ua)
print("bulkhead after steering relief: shells %d, vol %.0f mm3 (original %.0f)" % (len(bk.split(only_watertight=False)), bk.volume, V_ORIG))
for i_, s_ in enumerate(bk.split(only_watertight=False)): print("   shell", i_, "vol %.1f" % s_.volume, s_.bounds.round(1).tolist())
# ================================================================ UPDATE 12 + YETI OVERHAUL: AXIAL YETI FRONT BUMPER (AX31112: bull-bar hoop + swept wings + recessed bash plate + skid oval) + Y-BRACE (AX31104 #1)
BS, BCH = 2.6, 0.5                      # chamfered-square tube: 2.6 mm side, 0.5 mm chamfer (printable >= 2.2 mm, same language as cage tubes)
BXF, BXR = 65.2, 59.4                   # tube centre lines: hoop + oval front (X), oval rear (X)
BLZ = 0.2                               # oval tube centre Z (tube Z -1.1..1.5; flat bed contact Z=-1.1, top overlaps plate underside 1.3 by 0.2)
BP_Z0, BP_Z1 = 1.3, BPAD_Z0             # centre plate Z (top = flat underside of the bulkhead pad)
HY, HR, HZT, HZB = 8.0, 2.8, 11.4, 1.9  # hoop: leg foot Y, top corner radius, top-bar centre Z, leg foot Z (inside plate)
HY_TOP = 6.4                            # trapezoidal Yeti bull-bar top half-width (tapers from HY=8.0 at foot to 6.4 at top corners)
YZ0, YZ1 = 12.7, 14.3                   # Y-brace plate Z (bottom rests on hoop top bar at Z=12.7, top Z=14.3 flat on bed when printed upside-down)
def _oct(h, c): return [(h - c, h), (-(h - c), h), (-h, h - c), (-h, -(h - c)), (-(h - c), -h), (h - c, -h), (h, -(h - c)), (h, h - c)]
def sq_seg(p0, p1, nrm, ext=0.0, bs=None, bch=None):
    bs = BS if bs is None else bs; bch = BCH if bch is None else bch
    p0 = np.array(p0, float); p1 = np.array(p1, float); L_ = np.linalg.norm(p1 - p0); u_ = (p1 - p0) / L_
    a_ = np.array(nrm, float); a_ = a_ - u_ * np.dot(a_, u_)
    if np.linalg.norm(a_) < 1e-6: a_ = np.array([0.0, 0.0, 1.0]) - u_ * u_[2]
    a_ /= np.linalg.norm(a_); b_ = np.cross(u_, a_)
    return trimesh.convex.convex_hull(np.array([q_ + a_ * x_ + b_ * y_ for q_ in (p0 - u_ * ext, p1 + u_ * ext) for (x_, y_) in _oct(bs / 2, bch)]))
def arc2(c, R, a0, a1, n): return [(c[0] + R * math.cos(math.radians(t)), c[1] + R * math.sin(math.radians(t))) for t in np.linspace(a0, a1, n + 1)]
def sq_path(P, nrm, closed=False, ext=0.35, bs=None, bch=None):
    n_ = len(P); segs = []
    for i_ in range(n_ if closed else n_ - 1):
        p0, p1 = np.array(P[i_], float), np.array(P[(i_ + 1) % n_], float)
        if np.linalg.norm(p1 - p0) < 1e-6: continue
        segs.append(sq_seg(p0, p1, nrm, ext=ext, bs=bs, bch=bch))
    return segs
def cprism(poly, z0, z1, c=0.35):
    """convex polygon prism with the top and bottom perimeter chamfered c"""
    P_ = np.array(poly, float); cen = P_.mean(0)
    e_ = [abs((P_[(i + 1) % len(P_)][0] - P_[i][0]) * (cen[1] - P_[i][1]) - (P_[(i + 1) % len(P_)][1] - P_[i][1]) * (cen[0] - P_[i][0])) / np.linalg.norm(P_[(i + 1) % len(P_)] - P_[i]) for i in range(len(P_))]
    k_ = 1 - min(c, 0.45 * min(e_)) / min(e_); Q_ = cen + (P_ - cen) * k_
    pts = [(x, y, z) for (x, y) in Q_ for z in (z0, z1)] + [(x, y, z) for (x, y) in P_ for z in (z0 + c, z1 - c)]
    return trimesh.convex.convex_hull(np.array(pts))
def rect(x0, x1, y0, y1): return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
# --- FRONT GUARD redrawn to the owner's side profile (2026-11-20): leaning hoop + 45 deg skid ramp under the nose -----------------
# side profile (X, Z): bar outer face (69.2, 20.3) -> kink (67.9, 3.65); ramp 45 deg down and rearward to (63.15, -1.1); flat skid back to X 60.2.
# The ramp is the sketch's line lowered 2.3 mm: as drawn it ran through the bulkhead nose pad (Z 2.9..4.3) and the M2 bolt posts.
GD_T = 2.4                               # bar / ramp thickness
GD_TOP, GD_KO, GD_KI = (69.2, 20.3), (67.9, 3.65), (65.55, 4.55)   # top outer face, kink outer, kink inner (X, Z)
GD_ZB, GD_X0 = BLZ - BS / 2, 60.2        # skid underside Z (-1.1, same as the old oval) and rear end X of the skid block
GD_YF, GD_YB = HY + GD_T / 2, 8.6        # half width of the ramp at the hoop feet (9.2) and of the skid block under the nose (8.6: 1.4 mm inside the shocks at |Y| 10.0)
GD_NOTCH_Z = 0.8                          # ceiling of the bolt access notch (head sits Z 0.9..2.2)
GD_ZT = GD_TOP[1] - GD_T / 2             # top bar centre Z
_gk = ((GD_KO[0] + GD_KI[0]) / 2, (GD_KO[1] + GD_KI[1]) / 2)        # hoop centre line at the kink
_gdx = ((GD_TOP[0] - GD_T / 2) - _gk[0]) / (GD_ZT - _gk[1])          # forward lean per mm of height
def _gx(z_): return _gk[0] + _gdx * (z_ - _gk[1])
_hp = [(-HY, _gk[1])] + arc2((-HY_TOP + HR, GD_ZT - HR), HR, 170, 90, 6) + arc2((HY_TOP - HR, GD_ZT - HR), HR, 90, 10, 6) + [(HY, _gk[1])]
hoop_tubes = sq_path([(_gx(z_), y_, z_) for (y_, z_) in _hp], [1, 0, 0], ext=0.38, bs=GD_T, bch=0.45)
# mid rung: swept back to the old hoop top-bar line (X = BXF, top Z 12.7), it carries the Y-brace front screw pilot
_yu = HY - (HY - HY_TOP) * (HZT - _gk[1]) / (GD_ZT - HR - _gk[1])    # hoop leg Y at the rung height
rung_tubes = sq_path([(_gx(HZT), -_yu, HZT), (BXF, -4.6, HZT), (BXF, 4.6, HZT), (_gx(HZT), _yu, HZT)], [0, 0, 1], ext=0.38)
def _xz_prism(pts_, y_):                  # convex prism of an (X, Z) polygon across -y_..+y_
    return trimesh.convex.convex_hull(np.array([(x_, s_ * y_, z_) for (x_, z_) in pts_ for s_ in (1, -1)]))
_rx = GD_KO[0] - (GD_KO[1] - GD_ZB)       # where the ramp's outer face reaches the skid underside (X 63.15)
_ri = GD_KI[0] - (GD_KI[1] - BP_Z1)       # where the ramp's inner face reaches the plate top Z 2.9 (X 63.9, 0.2 in front of the nose)
_ramp = _xz_prism([GD_KO, GD_KI, (_ri, BP_Z1), (_rx - 1.0, GD_ZB), (_rx, GD_ZB)], GD_YF)
_skid = cprism(rect(GD_X0, _ri + 0.05, -GD_YB, GD_YB), GD_ZB, BP_Z1)
_R = (BXF - BXR) / 2; _XC = (BXF + BXR) / 2; _YC = 6.7     # (old oval numbers, still printed in the report line)
bump = U(*hoop_tubes, *rung_tubes, _ramp, _skid)
bump = D(bump,
         *[cyl("z", [BMX, s_ * BMY, 1.0], BM_CLR_R, 6.0, 28) for s_ in (1, -1)],                                               # M2 clearance dia 2.2 through the skid block
         *[cyl("z", [BMX, s_ * BMY, (GD_ZB - 0.2 + BP_Z0 + 0.9) / 2], BM_CB_R, BP_Z0 + 0.9 - (GD_ZB - 0.2), 32) for s_ in (1, -1)],   # head counterbore dia 4.0 from the underside up to the head seat Z 2.2 (0.7 mm of plate above it, as before)
         *[box(GD_X0 - 0.2, BMX, s_ * BMY - BM_CLR_R, s_ * BMY + BM_CLR_R, BP_Z0 + 0.8, BP_Z1 + 0.2) for s_ in (1, -1)],           # bolt holes stay rear-open slots above the head seat (hole centre is 1.6 mm from the rear edge)
         *[box(GD_X0 - 0.2, BMX, s_ * BMY - BM_CB_R, s_ * BMY + BM_CB_R, GD_ZB - 0.2, BP_Z0 + 0.9) for s_ in (1, -1)],           # counterbore open to the rear up to the head seat: the dia 4.0 circle only grazed the rear edge X 60.2 and left 0.1 mm paper fins at Z 1.5
         *[box(GD_X0 - 0.2, 65.1, s_ * BMY - BM_CB_R, s_ * BMY + BM_CB_R, GD_ZB - 0.2, GD_NOTCH_Z) for s_ in (1, -1)],           # access notch under each bolt head: removes the knife-edge sliver (0.05 mm) between the dia 4.0 counterbore and the 45 deg ramp face; wall above it >= 0.8
         cyl("z", [BXF, 0.0, HZT], 0.6, 4.0, 24))                                         # dia 1.2 pilot through the mid rung for the Y-brace front screw
_loc = trimesh.boolean.intersection([bk, box(55.0, 60.05, -10.0, 10.0, -3.0, 2.75)], engine=ENG)   # rear nose wall only (stays off Z=2.90 bolt pad)
_dil = U(*[_loc.copy().apply_translation(d_) for d_ in ([0, 0, 0], [.15, 0, 0], [-.15, 0, 0], [0, .15, 0], [0, -.15, 0], [0, 0, .15], [0, 0, -.15])])
bump = D(bump, _dil)
bump = max(bump.split(only_watertight=False), key=lambda s: s.volume)
def _rtok2(m):
    f_ = os.path.join(OUT, "_rtchk2.stl"); m.export(f_); ok_ = trimesh.load(f_).is_watertight; os.remove(f_); return ok_
if not _rtok2(bump):
    for dec_ in (3, 2, 1):   # quantise vertices (float32-STL safe) and drop degenerate faces
        q_ = trimesh.Trimesh(np.round(bump.vertices, dec_), bump.faces, process=True); q_.update_faces(q_.nondegenerate_faces()); q_.remove_unreferenced_vertices()
        q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume)
        ok_ = q_.is_watertight and _rtok2(q_); print("  bumper roundtrip repair: quantise %d dec -> watertight after roundtrip %s, vol %.1f" % (dec_, ok_, q_.volume))
        if ok_: bump = q_; break
# --- Yeti AX31104 #1 sculpted Y-brace plate over nose (flat top Z=YZ1 for zero-support upside-down print) -
def _yplate(poly, z0=YZ0, z1=YZ1, c=0.30):
    """Y-brace convex prism: flat top at z1 (100% print bed contact upside-down), chamfered bottom edge c."""
    P_ = np.array(poly, float); cen = P_.mean(0)
    e_ = [abs((P_[(i + 1) % len(P_)][0] - P_[i][0]) * (cen[1] - P_[i][1]) - (P_[(i + 1) % len(P_)][1] - P_[i][1]) * (cen[0] - P_[i][0])) / np.linalg.norm(P_[(i + 1) % len(P_)] - P_[i]) for i in range(len(P_))]
    k_ = 1 - min(c, 0.4 * min(e_)) / min(e_); Q_ = cen + (P_ - cen) * k_
    pts = [(x, y, z0) for (x, y) in Q_] + [(x, y, z0 + c) for (x, y) in P_] + [(x, y, z1) for (x, y) in P_]
    return trimesh.convex.convex_hull(np.array(pts))
YB_TONGUE_X1 = 66.85   # wall fix 2: tongue front edge 66.5 -> 66.85 (0.8 mm in front of the dia 1.7 screw hole, was 0.15-0.45; bumper front face is X 66.9)
YB_EAR_R = 2.6    # bolt-ear radius around the dia 2.2 M2 hole: 1.5 mm wall (1.2 at the 0.3 bottom chamfer); M2 nut across corners 4.62 sits inside
_yp = [
    _yplate(rect(63.2, YB_TONGUE_X1, -2.4, 2.4)),                                                                # front bumper-hoop tongue
    _yplate([(63.2, -2.4), (63.2, 2.4), (61.8, 4.4), (61.8, -4.4)]),                                       # nose flare to bolt bosses
    _yplate(rect(56.6, 61.8, -4.4, 4.4)),                                                                  # central bridge (passes 0.6 mm clear of shock tower stems |Y|>=5.0)
]
for s_ in (1, -1):
    # Flared Yeti Y-arm sweeping cleanly rearward behind shock tower onto bulkhead deck & upper shoulder
    _yp.append(_yplate([(BMX + YB_EAR_R * math.cos(a_), s_ * BMY + YB_EAR_R * math.sin(a_)) for a_ in np.linspace(0, 2 * math.pi, 32, endpoint=False)]))   # wall fix: round bolt ear (the dia 1.6 bolt hole broke out of the slanted flare edge: 0.00-0.08 mm)
    _yp.append(_yplate([(57.2, s_ * 4.4), (57.2, s_ * 1.5), (53.8, s_ * 2.4), (53.8, s_ * 5.2)]))          # inner Y-arm transition
    _yp.append(_yplate([(54.2, s_ * 5.2), (54.2, s_ * 2.8), (51.2, s_ * 5.4), (51.2, s_ * 7.6)]))          # flared outer Yeti tail leg
    # Underside spine stiffening flange (prints upward when inverted -> zero overhang)
    _yp.append(trimesh.convex.convex_hull(np.array([
        (56.4, s_ * 3.1, 12.1), (56.4, s_ * 4.1, 12.1), (56.4, s_ * 3.1, YZ0 + 0.2), (56.4, s_ * 4.1, YZ0 + 0.2),
        (51.8, s_ * 5.8, 12.1), (51.8, s_ * 6.8, 12.1), (51.8, s_ * 5.8, YZ0 + 0.2), (51.8, s_ * 6.8, YZ0 + 0.2),
    ])))
    _yp.append(box(53.8, 55.6, min(s_ * 2.6, s_ * 4.8), max(s_ * 2.6, s_ * 4.8), 9.65, YZ0 + 0.3))          # inner foot seating 0.15 mm above bulkhead deck (Z=9.5)
    _yp.append(box(51.2, 53.0, min(s_ * 5.7, s_ * 7.5), max(s_ * 5.7, s_ * 7.5), 11.20, YZ0 + 0.3))         # outer foot seating 0.20 mm above bulkhead upper shoulder (Z=11.0)
ybr = U(*_yp)
# Yeti AX31104 #1 central triangular truss lightening window (keeps >=1.2 mm ribs around all bolt holes & perimeter)
_ycut_tri = trimesh.convex.convex_hull(np.array([
    (59.9, 0.0, YZ0 - 0.5), (57.8, 1.5, YZ0 - 0.5), (57.8, -1.5, YZ0 - 0.5),
    (59.9, 0.0, YZ1 + 0.5), (57.8, 1.5, YZ1 + 0.5), (57.8, -1.5, YZ1 + 0.5),
]))
_yloc = trimesh.boolean.intersection([bk, box(50.5, 64.0, -9.0, 9.0, 8.5, 13.5)], engine=ENG)
_ydil = U(*[_yloc.copy().apply_translation(d_) for d_ in ([0, 0, 0], [.15, 0, 0], [-.15, 0, 0], [0, .15, 0], [0, -.15, 0], [0, 0, .15], [0, 0, -.15])])
ybr = D(ybr, _ycut_tri, _ydil, *[box(56.0, 60.5, min(s_ * 4.4, s_ * 6.5), max(s_ * 4.4, s_ * 6.5), 12.0, 15.0) for s_ in (1, -1)],   # bolt ears trimmed flat beside the shock tower stems (no thin lip; 0.81 mm wall left at the corner)
       
        *[cyl("z", [BMX, s_ * BMY, 13.5], BM_CLR_R, 3.0, 28) for s_ in (1, -1)],
        cyl("z", [BXF, 0.0, 13.5], 0.85, 3.0, 24),                                       # dia 1.7 clearance (playbook) for the M1.4 Y-brace front screw (kept M1.4: the hoop bar is 2.6 mm square)
        cyl("z", [BXF, 0.0, YZ1 - 0.45], 1.4, 0.9, 28),
        box(BXF, YB_TONGUE_X1 + 0.5, -1.4, 1.4, YZ1 - 0.9, YZ1 + 0.1))                   # wall fix 2: head counterbore open to the front (it left 0.10-0.25 mm fins at the tongue front edge)
ybr = max(ybr.split(only_watertight=False), key=lambda s: s.volume)
# --- hardware (reference bodies, M1.4): bolts + nuts through the nose, front + rear Y-brace screws ---------
def _scr(x_, y_, z_head_top, z_tip, hd=2.6, hh=0.9, r=0.7):
    return U(cyl("z", [x_, y_, (z_head_top + z_tip) / 2], r, abs(z_head_top - z_tip), 16), cyl("z", [x_, y_, z_head_top - hh / 2], hd / 2, hh, 20))
hw = {}
for side_, s_ in (("L", 1), ("R", -1)):
    hw["HW_Front_Bumper_Bolt_" + side_] = _scr(BMX, s_ * BMY, BP_Z0 + 0.9, BP_Z0 + 0.9 + 14.0, hd=2 * BM_HEAD_R, hh=BM_HEAD_H, r=1.0)   # M2 x 14: head seat Z 2.2 (head Z 0.9..2.2), tip Z 16.2 (0.3 past the nut)
    hw["HW_Front_Bumper_Nut_" + side_] = trimesh.creation.cylinder(radius=BM_NUT_R, height=BM_NUT_H, sections=6); hw["HW_Front_Bumper_Nut_" + side_].apply_translation([BMX, s_ * BMY, YZ1 + BM_NUT_H / 2])
hw["HW_Front_YBrace_Front_Screw"] = _scr(BXF, 0.0, YZ1, 10.3)
parts["Front_Bumper"] = bump; parts["Front_Y_Brace"] = ybr
for n_, m_ in (("Front_Bumper", bump), ("Front_Y_Brace", ybr)): printable[n_] = m_; static[n_] = m_; rest_all[n_] = m_
for n_, m_ in hw.items(): static[n_] = m_; rest_all[n_] = m_
print("bumper: vol %.0f mm3 ext %s watertight %s ; Y-brace: vol %.0f ext %s watertight %s" % (bump.volume, np.round(bump.extents, 1), bump.is_watertight, ybr.volume, np.round(ybr.extents, 1), ybr.is_watertight))


rest_scene = {n: (m, I) for n, m in rest_all.items()}
pairs = check(rest_scene, "rest")
print("REST interference pairs after relief:", len(pairs))
for p_ in pairs:
    if all(n_ in rest_all for n_ in p_) and not any(n_.startswith(("HW_", "CVD", "Ref_", "Rail")) for n_ in p_) and p_ != ("Steering_Rack", "Tie_Rod_L"):
        x_ = trimesh.boolean.intersection([rest_all[p_[0]], rest_all[p_[1]]], engine=ENG)
        print("   overlap %s x %s: %.2f mm3 bounds %s" % (p_[0], p_[1], x_.volume if len(x_.faces) else 0, np.round(x_.bounds, 1).tolist() if len(x_.faces) else ""))
for p in pairs: print("  ", p)
rest_pairs = set(tuple(sorted(p)) for p in pairs)
def real(pr):
    out = []
    for p in pr:
        if p in rest_pairs: continue
        if any(n.startswith("HW_") for n in p): continue          # hardware is placed static in poses: artefact
        if any(n.startswith("Ref_Shock") for n in p) and "IFS_Bulkhead_Convertible" not in p and not any(n.startswith("Ref_Wheel") for n in p): continue
        out.append(p)
    return out
def touching_only(sc, pair, tol=0.05):
    """FCL reports face/point contact as collision: accept a pair whose true boolean overlap is < tol mm3"""
    ms = []
    for n_ in pair:
        m_, T_ = sc[n_]; m2 = m_.copy(); m2.apply_transform(T_); ms.append(m2)
    try:
        x_ = trimesh.boolean.intersection(ms, engine=ENG); return (x_.volume if len(x_.faces) else 0.0) < tol
    except Exception: return False
print("\n--- TRAVEL LIMITS (first real contact scanning arm angle; ride pose = -4 deg) ---")
import time
def free_at(th, s):
    sc, sol = pose_scene(th, s)
    if sc is None: return False, "steer solve fails"
    sc = {k: v for k, v in sc.items() if not k.startswith("_plunge")}
    pr = [q for q in real(check(sc, "x")) if not touching_only(sc, q)]; return (not pr), pr[:3]
print("\n--- STEERING LOCK at ride pose (arm -4 deg) ---")
SLOCK = 0.0
for s in np.arange(0.5, 3.21, 0.1):
    ok, why = free_at(-4, float(s))
    if not ok: print(f" first contact at rack s={s:.2f} mm: {why}"); break
    SLOCK = float(s)
dl = steer_from_rack(solve_front(-4), SLOCK, "L"); dr = steer_from_rack(solve_front(-4), SLOCK, "R")
print(f" usable rack travel +/-{SLOCK:.2f} mm -> wheel angles {math.degrees(dl):.1f} / {math.degrees(-dr):.1f} deg; servo horn +/-{math.degrees(horn_angle(SLOCK)):.1f} deg")
PL_ = []
for s in (0.0, SLOCK):
    lim = {}
    for direction, rng in (("bump", range(-4, 24, 2)), ("droop", range(-4, -26, -2))):
        last = -4; why = None
        for th in rng:
            sc, sol = pose_scene(th, s)
            if sc is None: why = "steer solve fails"; break
            plung = max(abs(v[1][1, 3]) for k, v in sc.items() if k.startswith("_plunge"))
            sc = {k: v for k, v in sc.items() if not k.startswith("_plunge")}
            pr = [q for q in real(check(sc, "x")) if not touching_only(sc, q)]
            if pr: why = pr[:4]; break
            hub = (sol["Tc"] @ np.array([KX, YS, ZH, 1]))[2]; last = th; lastz = hub
            PL_.append(plung)
        lim[direction] = (last, why)
        print(f" steer s={s:+.1f}: {direction:5s} free to arm {last:+d} deg (hub Z {lastz if last != -4 else 1.47:+.1f}); blocked by: {why}")

print(" max CVD plunge/extension needed over free travel: %.2f mm" % max(PL_ or [0]))

# ---------------------------------------------------------------- write STLs + roundtrip watertight check
import manifold3d as _m3d
printable["IFS_Bulkhead_Convertible"] = parts["IFS_Bulkhead_Convertible"]   # the RELIEVED bulkhead (was exported unrelieved)
def _rt_ok(m, tag="_rtchk"):
    f_ = os.path.join(OUT, tag + ".stl"); m.export(f_); r_ = trimesh.load(f_); os.remove(f_)
    return bool(r_.is_watertight and len(r_.split(only_watertight=False)) == 1)
_DEGEN_CLEAN = ("IFS_Lower_Arm_L", "Steering_Rack", "Front_Spindle_L", "Front_Pin_Retainer_Plate_L", "Front_Pin_Retainer_Plate_R", "Front_Bumper")   # wall fix 2: near-coincident boolean slivers (faces under 1e-6 mm2) left by the dilated sweep cutters
def _dedegen(name, m):
    nb_ = int((m.area_faces < 1e-6).sum()); best_, n0_ = m, nb_
    for tol_ in (1e-3, 2e-3, 3e-3):
        if nb_ == 0: break
        try:
            mm_ = _m3d.Manifold(_m3d.Mesh(vert_properties=m.vertices.astype(np.float32), tri_verts=m.faces.astype(np.uint32)))
            if mm_.status() != _m3d.Error.NoError: break
            o_ = mm_.simplify(tol_).to_mesh(); q_ = trimesh.Trimesh(np.array(o_.vert_properties)[:, :3].astype(np.float64), np.array(o_.tri_verts), process=False)
            n_ = int((q_.area_faces < 1e-6).sum())
            if n_ < nb_ and abs(q_.volume - m.volume) < 0.01 and _rt_ok(q_): best_, nb_ = q_, n_
        except Exception:
            pass
    if nb_ < n0_: print(f"  {name} degenerate faces {n0_} -> {nb_} (manifold simplify, surface moved <= 0.003 mm)")
    return best_
def _repair_f32(name, m):
    if name in _DEGEN_CLEAN: m = _dedegen(name, m)
    if _rt_ok(m): return m
    for tol_ in (5e-4, 1.5e-3, 3e-3, 5e-3):
        try:
            mm_ = _m3d.Manifold(_m3d.Mesh(vert_properties=m.vertices.astype(np.float32), tri_verts=m.faces.astype(np.uint32)))
            if mm_.status() == _m3d.Error.NoError and mm_.num_tri() > 0:
                o_ = mm_.simplify(tol_).to_mesh()
                q_ = trimesh.Trimesh(np.array(o_.vert_properties)[:, :3].astype(np.float64), np.array(o_.tri_verts), process=False)
                q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume) if len(q_.faces) else q_
                if _rt_ok(q_):
                    print(f"  {name} roundtrip repair: manifold simplify({tol_}) -> watertight True, vol {q_.volume:.1f}")
                    return q_
        except Exception:
            pass
    for dec_ in (3, 2):
        q_ = trimesh.Trimesh(np.round(m.vertices, dec_), m.faces, process=True); q_.update_faces(q_.nondegenerate_faces()); q_.remove_unreferenced_vertices()
        q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume) if len(q_.faces) else q_
        if _rt_ok(q_):
            print(f"  {name} roundtrip repair: quantise {dec_} dec -> watertight True, vol {q_.volume:.1f}")
            return q_
    q_ = m
    for it_ in range(4):
        f_ = os.path.join(OUT, "_rtchk3.stl"); q_.export(f_); r_ = trimesh.load(f_); os.remove(f_)
        u_, c_ = np.unique(r_.edges_sorted, axis=0, return_counts=True); bad_ = u_[c_ != 2]
        if not len(bad_): break
        locs_ = np.unique(np.round(r_.vertices[bad_.ravel()].reshape(-1, 2, 3).mean(1), 2), axis=0)
        print(f"  {name} roundtrip: {len(bad_)} non-manifold edge(s) near {locs_.tolist()} -> 0.3 mm relief")
        q_ = D(q_, *[box(l_[0] - 0.15, l_[0] + 0.15, l_[1] - 0.15, l_[1] + 0.15, l_[2] - 0.15, l_[2] + 0.15) for l_ in locs_]); q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume)
        if _rt_ok(q_): return q_
    return q_
for _n in list(printable.keys()):
    if _n.endswith("_R") and (_n[:-2] + "_L") in printable: continue
    printable[_n] = _repair_f32(_n, printable[_n])
    parts[_n] = printable[_n]
    if _n in static: static[_n] = printable[_n]
    if _n in rest_all: rest_all[_n] = printable[_n]
    if _n in moving: moving[_n] = (moving[_n][0], printable[_n])
    if _n.endswith("_L") and rn(_n) in printable:
        printable[rn(_n)] = mirror(printable[_n])
        if rn(_n) in rest_all: rest_all[rn(_n)] = printable[rn(_n)]
for n, m in printable.items(): m.export(os.path.join(OUT, n + ".stl"))
rt = {n: trimesh.load(os.path.join(OUT, n + ".stl")) for n in printable}
print("\nROUNDTRIP watertight:", {n: bool(m.is_watertight) for n, m in rt.items() if not m.is_watertight} or "all printed parts OK", " (", len(rt), "parts )")
print("Servo envelope bbox (hood pocket):", np.round(refs["Servo_Ref"].bounds, 1).tolist())

printable["IFS_Bulkhead_Convertible"] = parts["IFS_Bulkhead_Convertible"]
bad = [n for n, m in printable.items() if not (m.is_watertight and m.volume > 0)]
print("printable parts:", len(printable), " non-watertight:", bad)
for n in sorted(printable):
    m = printable[n]; print(f"  {n:28s} vol {m.volume:7.1f}  ext {np.round(m.extents,1)}")


# ---------------------------------------------------------------- envelope check vs the CURRENT cage / shell / rails (stl/v3, read-only) + moving-part sweeps
print("\n--- ENVELOPE CHECK vs stl/v3 cage, shell, rails, gearbox ---")
envd = os.path.join(os.path.dirname(__file__), "..", "stl", "v3")
def _envf(n): return next((f_ for f_ in (os.path.join(envd, "..", d_, n + ".stl") for d_ in ("v4", "v3")) if os.path.exists(f_)), None)
env = {n: trimesh.load(_envf(n)) for n in ("Cage", "Tacoma_Trophy_Body_Shell", "Chassis_Rail_L", "Chassis_Rail_R", "Ref_Gearbox") if _envf(n)}
sw_ = {"horn": [], "link": [], "crank": [], "rack": []}
for s_ in np.linspace(-3.0, 3.0, 13):
    for k_, m_, T_ in (("horn", parts["Servo_Horn"], rotx(horn_angle(s_), YSP, ZSP)), ("link", parts["Steering_Drag_Link"], link_T(s_)), ("crank", parts["Steering_BellCrank"], crank_T(s_)), ("rack", parts["Steering_Rack"], tr([0, s_, 0]))):
        mm_ = m_.copy(); mm_.apply_transform(T_); sw_[k_].append(mm_)
mine = {"Servo_Ref": refs["Servo_Ref"], "HW_Servo_Mount_Screw_L": refs["HW_Servo_Mount_Screw_L"], "HW_Servo_Mount_Screw_R": refs["HW_Servo_Mount_Screw_R"],
        "Bulkhead": parts["IFS_Bulkhead_Convertible"], "horn_sweep": U(*sw_["horn"]), "link_sweep": U(*sw_["link"]), "crank_sweep": U(*sw_["crank"]), "rack_sweep": U(*sw_["rack"]), "Tie_Rod_L": parts["Tie_Rod_L"]}
for en, em in env.items():
    for mn, mm in mine.items():
        try: x_ = trimesh.boolean.intersection([em, mm], engine=ENG); v_ = x_.volume if len(x_.faces) else 0.0
        except Exception: v_ = -1
        if v_ > 0.02 or v_ < 0: print("   %-22s x %-24s %.2f mm3  bounds %s" % (en, mn, v_, np.round(x_.bounds, 1).tolist() if v_ > 0 else ""))
print(" (no line above = no overlap)  servo case X %.1f..%.1f Y %.1f..%.1f Z %.1f..%.1f ; ears to Y %.2f ; boss tip X %.1f" % (XC0, XC1, Y_N, Y_E, ZC0, ZC1, Y_E + EAR_OUT, XC1 + BOSS_H))

# ---------------------------------------------------------------- tub screws: hardware + head/driver clearance vs static bodies
print("\n--- TUB SCREW HOLES (floor Z %.1f..%.1f; Ø1.8 through, Ø3.2 counterbore 1.0 deep from the top) ---" % (BH_FLOOR_Z0, BH_FLOOR_Z1))
for (hx_, hy_) in BH_HOLES:
    col = cyl("z", [hx_, hy_, 5.0], 1.7, 11.0, 24)     # driver/head column from the floor top up to Z 10.5
    hits = []
    for n_, m_ in static.items():
        if n_.startswith(("IFS_Bulkhead", "HW_")): continue
        try: x_ = trimesh.boolean.intersection([m_, col], engine=ENG); v_ = x_.volume if len(x_.faces) else 0.0
        except Exception: v_ = -1
        if v_ > 0.01 or v_ < 0: hits.append((n_, round(v_, 2)))
    print("   hole (%.1f, %.1f): driver column hits: %s" % (hx_, hy_, hits or "none"))
bkc = parts["IFS_Bulkhead_Convertible"]
print(" bulkhead final: shells %d, watertight %s, volume %.0f" % (len(bkc.split(only_watertight=False)), bkc.is_watertight, bkc.volume))

# ================================================================ FULL-TRAVEL CHECK (shock stroke, hard stops, env contacts, wheel envelope)
print("\n=== FULL TRAVEL: shock stroke, stops, env contacts ===")
UP_ = SHOCK_UP.copy(); LO0_ = SHOCK_LO0.copy()
def shock_L(th_deg): return float(np.linalg.norm((rotx(math.radians(th_deg), *AL) @ np.append(LO0_, 1))[:3] - UP_))
SHOCK_STROKE = SHOCK_LMAX - SHOCK_LMIN      # 7.7 (Injora SCX30-08 listing)
SHOCK_BODY_FACE = SHOCK_BODY_LEN   # axis distance from the upper eye to the end face of the shock body
TH_STOP = 4.0           # end of clear travel (bump stop contact), arm angle
TH_N = -4.0             # neutral (mid-stroke, spring neutral) arm angle: lower arm 4 deg down, hub Z 1.47 = rear 1.5
Lmid = shock_L(TH_N)
print(" shock eye-eye at ride %.2f mm (mid-stroke %.2f); stroke %.1f -> bottom-out L=%.2f, full extension L=%.2f ; bump stop (arm %+.0f) L=%.2f (%.2f left), droop (-12) L=%.2f (%.2f left)" % (Lmid, (SHOCK_LMIN + SHOCK_LMAX) / 2, SHOCK_STROKE, SHOCK_LMIN, SHOCK_LMAX, TH_STOP, shock_L(TH_STOP), shock_L(TH_STOP) - SHOCK_LMIN, shock_L(-12), SHOCK_LMAX - shock_L(-12)))
for th in (-14, -12, -10, -8, -4, 0, 4, 6, 8, 12):
    hz = 3.0 + (AL[0] * 0 + math.sin(math.radians(th)) * (PL[0] - AL[0]))
    print("   arm %+3d deg: hub Z %+5.2f  shock L %.2f (%+.2f from mid)" % (th, hz, shock_L(th), shock_L(th) - Lmid))
th_b = brentq(lambda t: shock_L(t) - SHOCK_LMIN, TH_N, 40); th_d = brentq(lambda t: shock_L(t) - SHOCK_LMAX, -80, TH_N)
print(" shock bottoms at arm %+.1f deg (hub Z %.2f), fully extends at arm %+.1f deg (hub Z %.2f)" % (th_b, 3 + math.sin(math.radians(th_b)) * (PL[0] - AL[0]), th_d, 3 + math.sin(math.radians(th_d)) * (PL[0] - AL[0])))

envf = {}
for n_ in ("Chassis_Tub", "Cage", "Tacoma_Trophy_Body_Shell"):
    for d_ in ("v4", "v3"):
        f_ = os.path.join(os.path.dirname(__file__), "..", "stl", d_, n_ + ".stl")
        if os.path.exists(f_): envf[n_] = trimesh.load(f_); print("  env %s from stl/%s" % (n_, d_)); break
env_scene = {("Env_" + k): (v, I) for k, v in envf.items()}
def scan(th, s):
    sc, sol = pose_scene(th, s)
    if sc is None: return None
    sc = {k: v for k, v in sc.items() if not k.startswith("_plunge")}
    sc.update(env_scene)
    return sc
base = set(tuple(sorted(p)) for p in check(scan(TH_N, 0.0), "base"))
print(" baseline (neutral, straight) pairs incl. env: %d" % len(base))
print(" matrix: first NEW real contacts (not in baseline, boolean overlap >= 0.05 mm3). rows arm angle, cols rack s")
for th in (-14, -12, -10, -8, -6, -4, 0, 4, 6, 8):
    row = []
    for s in (0.0, 2.8, 3.0):
        sc = scan(th, s)
        if sc is None: row.append("s=%.1f: steer-solve fails" % s); continue
        pr = [q for q in check(sc, "m") if q not in base and not any(n_.startswith("HW_") for n_ in q) and not touching_only(sc, q)]
        pr = [q for q in pr if not (any(n_.startswith("Ref_Shock") for n_ in q) and not any(n_.startswith(("Env_", "IFS_Bulkhead")) for n_ in q))]
        row.append("s=%.1f: %s" % (s, "clear" if not pr else str([(a.replace("Tacoma_Trophy_Body_Shell", "Shell"), b.replace("Tacoma_Trophy_Body_Shell", "Shell")) for a, b in pr[:3]])))
    print("  arm %+3d | " % th + " | ".join(row))

# min clearances at the stops
print(" min clearances (mm) wheel/arms/tie rod vs env + bulkhead at selected poses")
def mind(sc, a, b):
    cm = trimesh.collision.CollisionManager(); m_, T_ = sc[b]; cm.add_object(b, m_, transform=T_)
    m2, T2 = sc[a]; return cm.min_distance_single(m2, transform=T2)
for th, s in ((-4.0, 0.0), (-12, 0.0), (4, 0.0), (-8, 3.0), (4, 3.0)):
    sc = scan(th, s)
    if sc is None: continue
    out = []
    for a in ("Ref_Wheel_FL", "Ref_Wheel_FR", "IFS_Lower_Arm_L", "IFS_Upper_Arm_L", "Tie_Rod_L"):
        for b in ("Env_Chassis_Tub", "Env_Tacoma_Trophy_Body_Shell", "Env_Cage", "IFS_Bulkhead_Convertible"):
            if a in sc and b in sc: out.append("%s/%s %.2f" % (a.replace("Ref_Wheel_", "W").replace("IFS_", "").replace("Tie_Rod_L", "Rod"), b.replace("Env_Tacoma_Trophy_Body_Shell", "Shell").replace("Env_", "").replace("IFS_Bulkhead_Convertible", "Bulk"), mind(sc, a, b)))
    print("  arm %+.0f s=%.1f: %s" % (th, s, "; ".join(out)))

# wheel envelope over the usable range: union of per-pose convex hulls, both wheels
env_meshes = []
for th in (-12, -10, -8, -6, -4, -2, 0, 2, 4):
    for s in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0):
        sc, _ = pose_scene(th, s)
        if sc is None: continue
        for k_ in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            m_, T_ = sc[k_]; mm_ = m_.copy(); mm_.apply_transform(T_); env_meshes.append(mm_.convex_hull)
wenv = U(*env_meshes)
_we = [_repair_f32("Front_Wheel_Envelope", c_) for c_ in sorted(wenv.split(only_watertight=False), key=lambda c: -c.volume)[:2]]   # reference keep-out body: one closed shell per side (L, R), each made float32-STL safe
wenv = trimesh.util.concatenate(_we)
wenv.export(os.path.join(OUT, "Front_Wheel_Envelope.stl"))
_wr = trimesh.load(os.path.join(OUT, "Front_Wheel_Envelope.stl")); print(" Front_Wheel_Envelope roundtrip: watertight %s, shells %d (one per side)" % (_wr.is_watertight, len(_wr.split(only_watertight=False))))
print(" Front_Wheel_Envelope.stl (stl/v3): arm -12..+4 deg x rack +/-3.0 mm, both wheels, union of per-pose convex hulls; vol %.0f, watertight %s, bounds %s" % (wenv.volume, wenv.is_watertight, np.round(wenv.bounds, 1).tolist()))
for n_, m_ in envf.items():
    try: x_ = trimesh.boolean.intersection([m_, wenv], engine=ENG); print("   envelope x %s: %.1f mm3" % (n_, x_.volume if len(x_.faces) else 0))
    except Exception as e_: print("   envelope x %s: boolean not possible (%s; watertight %s)" % (n_, e_, m_.is_watertight))

# ================================================================ BUMPER / Y-BRACE REPORT (UPDATE 12)
print("\n=== FRONT BUMPER + Y-BRACE: extents, overhang, clearances ===")
for n_ in ("Front_Bumper", "Front_Y_Brace"):
    m_ = parts[n_]; print("  %-14s X %.1f..%.1f  Y %.1f..%.1f  Z %.1f..%.1f  vol %.0f mm3" % ((n_,) + tuple(np.round(np.array([m_.bounds[0][0], m_.bounds[1][0], m_.bounds[0][1], m_.bounds[1][1], m_.bounds[0][2], m_.bounds[1][2]]), 1)) + (m_.volume,)))
_bx = parts["Front_Bumper"].bounds[1][0]; _shx = envf["Tacoma_Trophy_Body_Shell"].bounds[1][0]; _tyx = wenv.bounds[1][0]; _bkx = parts["IFS_Bulkhead_Convertible"].bounds[1][0]
print("  bumper front face X %.1f : +%.1f mm past the bulkhead nose (X%.1f), +%.1f past the shell nose (X%.1f), %.1f mm BEHIND the tyre front plane (X%.1f) -> front overhang change %+.1f mm" % (_bx, _bx - _bkx, _bkx, _bx - _shx, _shx, _tyx - _bx, _tyx, max(0.0, _bx - _tyx)))
_cm = trimesh.collision.CollisionManager()
def _dist(m_, o_):
    c_ = trimesh.collision.CollisionManager(); c_.add_object("o", o_); return c_.min_distance_single(m_)
_tg = {"wheel envelope (full lock x arm -12..+4)": wenv, "shell": envf["Tacoma_Trophy_Body_Shell"], "tub": envf["Chassis_Tub"], "cage": envf["Cage"], "Ref_Shock_FL": rest_all.get("Ref_Shock_FL"), "Ref_Shock_FR": rest_all.get("Ref_Shock_FR"),
       "Servo_Ref": refs["Servo_Ref"], "Steering_BellCrank": parts["Steering_BellCrank"], "Lower_Arm_L (rest)": rest_all["IFS_Lower_Arm_L"], "Lower hinge pin L": trimesh.load(os.path.join(OUT, "HW_Front_Lower_Hinge_Pin_L.stl")), "Retainer plate L": trimesh.load(os.path.join(OUT, "Front_Pin_Retainer_Plate_L.stl")),
       "CVD_Outer_L": rest_all.get("CVD_Outer_L"), "CVD_Inner_L": rest_all.get("CVD_Inner_L"), "Bulkhead": parts["IFS_Bulkhead_Convertible"]}
for n_ in ("Front_Bumper", "Front_Y_Brace"):
    print("  %s min distance to:" % n_, "; ".join("%s %.2f" % (k_, _dist(parts[n_], v_)) for k_, v_ in _tg.items() if v_ is not None and (k_ != "Bulkhead" or n_ == "Front_Bumper")))
print("  bumper vs bulkhead (touching faces only: plate top = pad underside Z%.1f, nut/screw pads), Y-brace vs bulkhead: min distance %.2f ; Y-brace rear feet vs deck top (Z9.5): %.2f" % (BPAD_Z0, _dist(parts["Front_Y_Brace"], parts["IFS_Bulkhead_Convertible"]), _dist(box(53.9, 55.8, 2.4, 4.8, 9.6, 9.7), parts["IFS_Bulkhead_Convertible"])))
print("  hoop span outer Y +-%.1f (%.1f), tube %.1f sq / %.1f chamfer ; oval outer %.1f x %.1f ; ground clearance of the oval underside: %.1f mm (tyre ground Z %.1f)" % (HY + BS / 2, 2 * (HY + BS / 2), BS, BCH, 2 * (_YC + _R + BS / 2), BXF - BXR + BS, (BLZ - BS / 2) - wenv.bounds[0][2], wenv.bounds[0][2]))

# ================================================================ bump stops (shock rod rings), wire-clip lid, fastener list
print("\n=== BUMP STOPS / CLIP / HARDWARE ===")
ax_ = (LO0_ - UP_) / np.linalg.norm(LO0_ - UP_)
RING_OFF = SHOCK_BODY_FACE + (shock_L(0.0) - shock_L(TH_STOP))      # ring face on the rod touches the body face when the shock has shortened from L(0) to L(TH_STOP)
print(" bump stop: ring face %.2f mm from the upper eye; touches the shock body (face at %.1f) at arm %+.1f deg = shock L %.2f (%.2f above bottom-out at L %.2f)" % (RING_OFF, SHOCK_BODY_FACE, TH_STOP, shock_L(TH_STOP), shock_L(TH_STOP) - SHOCK_LMIN, SHOCK_LMIN))
ring = D(seg_cyl(UP_ + ax_ * RING_OFF, UP_ + ax_ * (RING_OFF + 2.0), SHOCK_ROD_R + 1.5), seg_cyl(UP_ + ax_ * (RING_OFF - 1.0), UP_ + ax_ * (RING_OFF + 3.0), SHOCK_ROD_R + 0.05))     # on the exposed shock rod: stops compression at arm +4 deg
_oldclip = os.path.join(OUT, "Servo_Wire_Clip.stl")
if os.path.exists(_oldclip): os.remove(_oldclip); print("  removed stl/v3/Servo_Wire_Clip.stl (owner: no servo brace)")
extra = {"Front_Bump_Stop_L": ring, "Front_Bump_Stop_R": mirror(ring)}
for n_, m_ in extra.items(): m_.export(os.path.join(OUT, n_ + ".stl"))
for n_, m_ in extra.items():
    rt_ = trimesh.load(os.path.join(OUT, n_ + ".stl")); print("  %-20s watertight %s vol %.1f" % (n_, rt_.is_watertight, rt_.volume))
old_ = os.path.join(OUT, "Steering_Link.stl")
if os.path.exists(old_): os.remove(old_); print("  removed stl/v3/Steering_Link.stl (replaced by Steering_Drag_Link + Steering_BellCrank)")
print(" servo horn angle (rack +/-2.8): %.1f deg ; crank psi %.1f deg ; drag-link length %.2f" % (math.degrees(horn_angle(2.8)), math.degrees(crank_psi(2.8)), LD))
for nme, m_ in (("HW_Front_Driveshaft_UJ_Gbx", refs["HW_Front_Driveshaft_UJ_Gbx"]), ("HW_Front_Driveshaft_UJ_Worm", refs["HW_Front_Driveshaft_UJ_Worm"])):
    x_ = trimesh.boolean.intersection([m_, refs["Servo_Ref"]], engine=ENG); print("  %s x servo case: %.2f mm3" % (nme, x_.volume if len(x_.faces) else 0))
_gb = trimesh.load(os.path.join(os.path.dirname(__file__), "..", "stl", "v4", "Ref_Gearbox.stl")); _wm = allb["Ref_Front_Input_Worm_Gear"]
print(" front shaft: gearbox output stub axis Y0 Z%.2f, stub end X%.1f -> coupler cap X%.1f ; shaft (%.1f,%.1f) -> worm (%.1f,%.1f): length %.2f, angle %.1f deg ; UJ at each end (stock/Injora UJ ok to ~20 deg)" % (GBX_SHAFT_Z, GBX_STUB_X, GBX_X, GBX_X, GBX_SHAFT_Z, WORM_IN[0], WORM_IN[2], float(np.linalg.norm(np.array(WORM_IN) - np.array([GBX_X, 0, GBX_SHAFT_Z]))), math.degrees(math.atan2(WORM_IN[2] - GBX_SHAFT_Z, WORM_IN[0] - GBX_X))))
for nme, m_ in (("Ref_Front_Driveshaft", refs["Ref_Front_Driveshaft"]), ("HW_Front_Driveshaft_UJ_Gbx", refs["HW_Front_Driveshaft_UJ_Gbx"]), ("HW_Front_Driveshaft_UJ_Worm", refs["HW_Front_Driveshaft_UJ_Worm"])):
    for on_, om_ in (("Ref_Gearbox", _gb), ("Ref_Front_Input_Worm_Gear", _wm), ("Servo_Ref", refs["Servo_Ref"])):
        x_ = trimesh.boolean.intersection([m_, om_], engine=ENG); cm_ = trimesh.collision.CollisionManager(); cm_.add_object("o", om_)
        print("  %-28s x %-26s overlap %.3f mm3, min distance %.2f mm" % (nme, on_, x_.volume if len(x_.faces) else 0, cm_.min_distance_single(m_)))


# ================================================================ DOGBONE ANGLES (joint at BOTH ends) and plunge
print("\n=== DOGBONE: per-end joint angle over steering lock x hub travel ===")
def dog_state(th, s, j1y=None, dy=None):
    j1y = DOG_J1_Y if j1y is None else j1y; dy = DOG_J2_DY if dy is None else dy
    sol = solve_front(th); d = steer_from_rack(sol, s, "L")
    if d is None: return None
    Tsp = sol["Tc"] @ rotz(d, KX, YS)
    J2 = (Tsp @ np.array([KX, YS + dy, ZH, 1.0]))[:3]; u = Tsp[:3, :3] @ np.array([0, 1.0, 0]); J1 = np.array([KX, j1y, ZH]); v = J2 - J1; L = np.linalg.norm(v); v = v / L
    ang = lambda a, b: math.degrees(math.acos(max(-1, min(1, float(np.dot(a, b))))))
    return ang(v, np.array([0, 1.0, 0])), ang(v, u), L, math.degrees(d)
def dog_report(j1y, dy, ths, ss):
    r_ = [(th, s, dog_state(th, s, j1y, dy)) for th in ths for s in ss]; r_ = [x for x in r_ if x[2]]
    L0_ = dog_state(TH_N, 0.0, j1y, dy)[2]
    m1 = max(r_, key=lambda x: x[2][0]); m2 = max(r_, key=lambda x: x[2][1])
    pl = [x[2][2] - L0_ for x in r_]
    return m1, m2, L0_, min(pl), max(pl)
RIDE = [TH_N]; TRAVEL = [-12, -8, -4, 0, 4]; BUMPX = [-12, -8, -4, 0, 4, 8, 12]
SS = [-2.8, -2.0, -1.0, 0.0, 1.0, 2.0, 2.8]
for j1y, dy in ((DOG_J1_Y, DOG_J2_DY), (10.3, 2.0), (14.0, 0.0), (18.0, 0.0)):
    for lab, ths in (("travel -12..+4", TRAVEL), ("travel -12..+12", BUMPX)):
        m1, m2, L0_, pmin, pmax = dog_report(j1y, dy, ths, SS)
        print(" J1_Y=%.1f J2_dy=%.1f  %-15s: dogbone L(neutral)=%.2f ; max diff-end %.1f deg (arm %+d, s %+.1f) ; max knuckle-end %.1f deg (arm %+d, s %+.1f, wheel %.1f deg) ; plunge %+.2f..%+.2f mm"
              % (j1y, dy, lab, L0_, m1[2][0], m1[0], m1[1], m2[2][1], m2[0], m2[1], m2[2][3], pmin, pmax))
print(" slot engagement DOG_SLOT=%.1f mm per end -> plunge capacity about +/-%.1f mm; joint centre on the kingpin axis keeps steering from changing the length" % (DOG_SLOT, DOG_SLOT - 0.5))

_m = max(dog_report(DOG_J1_Y, DOG_J2_DY, BUMPX, SS)[i][2][k] for i, k in ((0, 0), (1, 1)))
print(" Injora %s: max per-end angle %.1f deg vs rating %.0f deg -> %s, margin %.1f deg" % (INJORA_PN, _m, DOG_RATING_DEG, "PASS" if _m < DOG_RATING_DEG else "FAIL", DOG_RATING_DEG - _m))

# ================================================================ export the NON-printed front bodies I changed/added (rest pose) so downstream (integration / .sc) overrides the stale per-body ones
print("\n=== EXPORT front reference/hardware bodies to stl/v3 (rest pose) ===")
exported = []
_skip = ("Rail_Box", "Ref_Gearbox_Spec_Box", "Env_")
cand = dict(rest_all); cand["Ref_Front_Center_Driveshaft"] = refs["Ref_Front_Driveshaft"]      # same name as the stale body: now the fixed shaft (the only front shaft body; Ref_Front_Driveshaft is NOT exported = would duplicate it)
cand.pop("Ref_Front_Driveshaft", None)
KEEP_FROM_FOLDER = ("Ref_Front_Diff_Ring_Gear", "Ref_Front_Input_Worm_Gear", "HW_Front_Lower_Hinge_Pin_L", "HW_Front_Lower_Hinge_Pin_R", "HW_Front_Upper_Hinge_Pin_L", "HW_Front_Upper_Hinge_Pin_R",
                    "HW_Front_Shock_Lower_Screw_L", "HW_Front_Shock_Lower_Screw_R", "Front_Pin_Retainer_Plate_L", "Front_Pin_Retainer_Plate_R", "HW_Front_Retainer_Screw_L", "HW_Front_Retainer_Screw_R")   # retainer plate+screw keep the inboard hinge pins (X57, touching bulkhead/pin, no overlap): still valid, printed plate   # retainer plate+screw keep the inboard hinge pins (X57, touching bulkhead/pin, no overlap): still valid, printed plate
RET_SCREW, RET_RING_R, RET_HOLE_R, RET_THICKEN = (11.6, 3.4), 1.7, 0.85, 0.30      # wall fix: retainer plate screw hole (Y, Z); 0.85 mm ring around a dia 1.7 clearance hole (was dia 1.5 with 0.32-0.45 to the outer edge and to the notch); plate 0.70 -> 1.00 thick (forward)
_plate_fix = {}
for s_, k_ in ((1, "Front_Pin_Retainer_Plate_L"), (-1, "Front_Pin_Retainer_Plate_R")):
    if k_ in allb:
        x0_, x1_ = allb[k_].bounds[0][0], allb[k_].bounds[1][0] + RET_THICKEN
        _pl = U(allb[k_], allb[k_].copy().apply_translation([RET_THICKEN, 0, 0]), cyl("x", [(x0_ + x1_) / 2, s_ * RET_SCREW[0], RET_SCREW[1]], RET_RING_R, x1_ - x0_, 40))
        _plate_fix[k_] = D(_pl, cyl("x", [(x0_ + x1_) / 2, s_ * RET_SCREW[0], RET_SCREW[1]], RET_HOLE_R, x1_ - x0_ + 2.0, 28))
        _plate_fix[k_] = _dedegen(k_, _plate_fix[k_])      # paper triangles from the body-dir plate (7 on the L plate)
        print(" retainer plate %s: thickened %.2f, ring r %.2f around a dia %.1f hole, vol %.2f -> %.2f, bounds %s, shells %d" % (k_, RET_THICKEN, RET_RING_R, 2 * RET_HOLE_R, allb[k_].volume, _plate_fix[k_].volume, _plate_fix[k_].bounds.round(2).tolist(), len(_plate_fix[k_].split(only_watertight=False))))
        _ks = k_.replace("Front_Pin_Retainer_Plate", "HW_Front_Retainer_Screw")      # M1.4 x 3 (kept M1.4: the bulkhead wall outboard of its pilot is only 0.5 mm): tip X 55.05, head dia 2.4 x 0.7 on the thicker plate
        _plate_fix[_ks] = U(cyl("x", [(x1_ - 3.0 + x1_) / 2, s_ * RET_SCREW[0], RET_SCREW[1]], 0.7, 3.0, 20), cyl("x", [x1_ + 0.35, s_ * RET_SCREW[0], RET_SCREW[1]], 1.2, 0.7, 24))
if "Front_Pin_Retainer_Plate_L" in _plate_fix and "Front_Pin_Retainer_Plate_R" in _plate_fix: _plate_fix["Front_Pin_Retainer_Plate_R"] = mirror(_plate_fix["Front_Pin_Retainer_Plate_L"])   # hygiene: R is the exact mirror of the cleaned L plate (its own boolean left 36 sliver faces between 1e-6 and 1e-4 mm2 that _dedegen does not count)
for k_ in KEEP_FROM_FOLDER:
    if k_ in allb: _plate_fix.get(k_, static.get(k_, allb[k_])).export(os.path.join(OUT, k_ + ".stl")); exported.append(k_ + (" (ring added)" if k_ in _plate_fix else " (unchanged copy from per-body folder)"))
for k_, m_ in refs.items():
    if k_.startswith(("HW_", "Ref_Front", "Servo_Ref")) and k_ != "Ref_Front_Driveshaft": cand[k_] = m_
for n_, m_ in cand.items():
    if n_ in printable or n_.startswith(_skip) or n_ in ("Ref_Gearbox",): continue
    if n_ in allb and abs(allb[n_].volume - m_.volume) < 1e-3 and np.allclose(allb[n_].bounds, m_.bounds, atol=1e-3): continue
    if not len(m_.faces): continue
    m_.export(os.path.join(OUT, n_ + ".stl")); exported.append(n_)
print(" exported %d: %s" % (len(exported), ", ".join(sorted(exported))))
_g = trimesh.load(os.path.join(os.path.dirname(__file__), "..", "stl", "v4", "Ref_Gearbox.stl")); _c = trimesh.load(os.path.join(OUT, "Ref_Front_Center_Driveshaft.stl")); _u = trimesh.load(os.path.join(OUT, "HW_Front_Driveshaft_UJ_Gbx.stl"))
for nm_, mm_ in (("Ref_Front_Center_Driveshaft", _c), ("HW_Front_Driveshaft_UJ_Gbx", _u)):
    x_ = trimesh.boolean.intersection([_g, mm_], engine=ENG); print("  exported %s x stl/v4 Ref_Gearbox: %.3f mm3" % (nm_, x_.volume if len(x_.faces) else 0))

_cr = trimesh.load(os.path.join(OUT, "Steering_BellCrank.stl")); _dmin = 9.0
for s_ in np.linspace(-3.0, 3.0, 13):
    for k_ in ("HW_Front_Lower_Hinge_Pin_L", "HW_Front_Lower_Hinge_Pin_R"):
        cm_ = trimesh.collision.CollisionManager(); cm_.add_object("p", static[k_]); mc_ = _cr.copy(); mc_.apply_transform(crank_T(s_)); _dmin = min(_dmin, cm_.min_distance_single(mc_))
print(" lower hinge pin tip (trimmed at X%.1f) vs bell crank over lock: min distance %.2f mm" % (PIN_TRIM_X0, _dmin))
OBSOLETE = ("Ref_Front_Driveshaft", "Ref_Front_CVD_", "HW_Drag_Link_", "Kingpin_Nut", "HW_Front_TieRod_Screw", "HW_Steering_TieRod_Inner_Screw", "Steering_Link", "IFS_Knuckle", "Ref_Servo_Envelope")
_bad = [f for f in os.listdir(OUT) if f.endswith(".stl") and any(o in f for o in OBSOLETE)]
print(" obsolete names present in stl/v3: %s" % (_bad or "none"))

# ================================================================ UPDATE 13: STATIC RIDE POSE EXPORT (arm -4 deg) + raised-stack steering/driveline report
print("\n=== UPDATE 13: raised stack (+%.1f mm) and static ride pose (arm %+.0f deg) ===" % (SERVO_DZ, TH_N))
sc_r, sol_r = pose_scene(TH_N, 0.0)
_ride = set(moving) | {rn(n_) for n_ in moving} | {n_ for n_ in L_names} | {rn(n_) for n_ in L_names}
_rn = []
for n_ in sorted(_ride):
    if n_.startswith("HW_TieRod_Inner") or n_ not in sc_r: continue
    if n_.endswith("_R") and (n_[:-2] + "_L") in _ride: continue
    m_, T_ = sc_r[n_]; mm_ = m_.copy(); mm_.apply_transform(T_)
    if n_ in printable: mm_ = _repair_f32(n_, mm_)
    mm_.export(os.path.join(OUT, n_ + ".stl")); _rn.append(n_)
    if n_.endswith("_L") and rn(n_) in _ride:
        mm_r = mirror(mm_); mm_r.export(os.path.join(OUT, rn(n_) + ".stl")); _rn.append(rn(n_))
_ax0 = (SHOCK_LO0 - SHOCK_UP) / np.linalg.norm(SHOCK_LO0 - SHOCK_UP); _axr = (shock_lo(TH_N) - SHOCK_UP) / np.linalg.norm(shock_lo(TH_N) - SHOCK_UP)
_Tb = tr(SHOCK_UP) @ trimesh.geometry.align_vectors(_ax0, _axr) @ tr(-SHOCK_UP)
for n_, m_ in (("Front_Bump_Stop_L", extra["Front_Bump_Stop_L"]), ("Front_Bump_Stop_R", extra["Front_Bump_Stop_R"])):
    mm_ = m_.copy(); mm_.apply_transform(_Tb if n_.endswith("L") else np.diag([1, -1, 1, 1.0]) @ _Tb @ np.diag([1, -1, 1, 1.0])); mm_.export(os.path.join(OUT, n_ + ".stl")); _rn.append(n_)
print(" exported at ride pose (%d bodies): %s" % (len(_rn), ", ".join(_rn)))
_wl = trimesh.load(os.path.join(OUT, "Ref_Wheel_FL.stl")); _wr = trimesh.load(os.path.join(OUT, "Ref_Wheel_FR.stl"))
_rw = trimesh.load(os.path.join(os.path.dirname(__file__), "..", "stl", "v4", "Ref_Rear_Axle.stl")) if os.path.exists(os.path.join(os.path.dirname(__file__), "..", "stl", "v4", "Ref_Rear_Axle.stl")) else None
print(" front wheels at ride: FL Z %.2f..%.2f (centre %.2f), FR Z %.2f..%.2f ; hub Z %.2f ; ground plane target Z -16.5 ; arm %+.1f deg, shock eye-eye %.2f (mid %.2f)" % (_wl.bounds[0][2], _wl.bounds[1][2], _wl.centroid[2], _wr.bounds[0][2], _wr.bounds[1][2], (sol_r["Tc"] @ np.append(CVD_J0, 1))[2], TH_N, shock_L(TH_N), (SHOCK_LMIN + SHOCK_LMAX) / 2))
# steering with the raised servo
_ha = {s_: horn_angle(s_) for s_ in (-3.0, -2.8, 0.0, 2.8, 3.0)}
_tilt = []
for s_ in np.linspace(-3.0, 3.0, 25):
    v_ = A_of(horn_angle(s_)) - I_of(crank_psi(s_)); _tilt.append(math.degrees(math.asin(v_[2] / np.linalg.norm(v_))))
print(" servo raised %.1f: spline Z %.2f ; horn arm %.2f mm (pin Z %.2f stays on the bell-crank/link plane Z %.2f) ; horn angle at s=+-2.8: %.1f / %.1f deg, s=+-3.0: %.1f / %.1f deg" % (SERVO_DZ, ZSP, HORN_R, ZPIN, ZI, math.degrees(_ha[2.8]), math.degrees(_ha[-2.8]), math.degrees(_ha[3.0]), math.degrees(_ha[-3.0])))
print(" drag link %.2f mm pin-to-pin, planar (tilt over +-3.0 mm rack: %.2f..%.2f deg, max |%.2f|) ; crank psi at s=2.8: %.1f deg ; rack travel +-2.8 (hard +-3.0)" % (LD, min(_tilt), max(_tilt), max(abs(t_) for t_ in _tilt), math.degrees(crank_psi(2.8))))
for s_ in (-2.8, 2.8):
    dl_ = steer_from_rack(sol_r, s_, "L"); dr_ = steer_from_rack(sol_r, s_, "R"); print("   ride pose rack %+.1f: left wheel %.1f deg, right %.1f deg" % (s_, math.degrees(dl_), -math.degrees(dr_)))
_dz = GBX_SHAFT_Z - WORM_IN[2]; _dx = WORM_IN[0] - GBX_X
print(" front shaft: gearbox end (X%.1f, Z%.2f) -> worm (X%.1f, Z%.2f): length %.2f mm, angle %+.2f deg at BOTH ends (gearbox axis and worm axis horizontal; was %+.2f with gearbox Z %.1f); UJ limit ~20 deg" % (GBX_X, GBX_SHAFT_Z, WORM_IN[0], WORM_IN[2], math.hypot(_dx, _dz), -math.degrees(math.atan2(_dz, _dx)), math.degrees(math.atan2(WORM_IN[2] - 4.1, _dx)), 4.1))
# under-hood front hoop tie: free space above the bulkhead
_sh = envf["Tacoma_Trophy_Body_Shell"]; _V = _sh.vertices
for x0_, x1_ in ((30, 40), (40, 50), (50, 58), (58, 63.6)):
    m_ = (_V[:, 0] >= x0_) & (_V[:, 0] < x1_) & (abs(_V[:, 1]) < 16) & (_V[:, 2] > 12)
    print("  shell underside (|Y|<16, Z>12) X%.0f..%.0f: lowest Z %s" % (x0_, x1_, round(float(_V[m_, 2].min()), 1) if m_.any() else None))
_bt = parts["IFS_Bulkhead_Convertible"]; _sv = refs["Servo_Ref"]
print("  servo case top Z %.1f (X%.1f..%.1f, Y%.1f..%.1f); Y-brace top Z %.1f, nuts top %.1f; fork towers top Z %.1f (Y+-%.1f..%.1f, X57.0..60.2); shock top Z %.1f (Y>=%.1f)" % (_sv.bounds[1][2], _sv.bounds[0][0], _sv.bounds[1][0], _sv.bounds[0][1], _sv.bounds[1][1], parts["Front_Y_Brace"].bounds[1][2], static["HW_Front_Bumper_Nut_L"].bounds[1][2] if "HW_Front_Bumper_Nut_L" in static else 0, SHOCK_UP[2] + TW_R, TW_Y0, SHOCK_UP[1] + TW_R, rest_all["Ref_Shock_FL"].bounds[1][2], SHOCK_UP[1] - SHOCK_EYE_R))

# ---- UPDATE 14: floor web report
_b = parts["IFS_Bulkhead_Convertible"]; _zs = np.arange(-4.0, 2.0, 0.05)
for (hx_, hy_) in BH_HOLES:
    for dy_ in (1.5,):
        c_ = _b.contains(np.array([[hx_ + 2.6 * 0.0, hy_ + (dy_ if hy_ > 0 else -dy_), z_] for z_ in _zs])); zz_ = _zs[c_]
        print(" web at (%.1f,%.1f)+1.5 Y beside the hole: solid Z %.2f..%.2f" % (hx_, hy_, zz_.min(), zz_.max()))
print(" web: underside Z%.1f, top Z%.1f (%.1f thick), screw seat pads top Z%.1f; counterbore floor Z%.2f -> web under the head %.2f mm; bulkhead volume %.0f mm3; min Z of bulkhead %.2f" % (WEB_Z0, WEB_TOP, WEB_TOP - WEB_Z0, SEAT_TOP, SEAT_TOP - 0.45, SEAT_TOP - 0.45 - WEB_Z0, _b.volume, _b.bounds[0][2]))

# ================================================================ FINAL AUDIT: steering lock x suspension travel, true min distances on the FINAL meshes (incl. pins/screws)
print("\n=== FINAL AUDIT: rack s=-3..+3 x arm -12..+8 : steering parts + pins vs bulkhead / Chassis_Tub / each other ===")
AUD_MIN = 0.25
_bkF = parts["IFS_Bulkhead_Convertible"]; _tubF = envf.get("Chassis_Tub")
def _cmo(m_):
    c_ = trimesh.collision.CollisionManager(); c_.add_object("o", m_); return c_
_tgt = {"Bulkhead": _cmo(_bkF)}
if _tubF is not None: _tgt["Chassis_Tub(v4 file)"] = _cmo(_tubF)
_tm = {"Bulkhead": _bkF, "Chassis_Tub(v4 file)": _tubF}
_bossX = cyl("z", [XP, YPC, -0.4], 2.45, 1.2, 32)     # crank pivot boss (top Z 0.10): the crank's thrust face runs on it with 0.15 axial play = intended bearing, excluded for the crank only
_tgtC = {k_: _cmo(D(v_, _bossX)) for k_, v_ in _tm.items() if v_ is not None}
def _steer_group(s_):
    Th_ = rotx(horn_angle(s_), YSP, ZSP); Tl_ = link_T(s_); Tc_ = crank_T(s_); Tr_ = tr([0, float(s_), 0])
    return {"Servo_Horn": (parts["Servo_Horn"], Th_), "HW_Link_Pin_A": (refs["HW_Link_Pin_A"], Th_), "Steering_Drag_Link": (parts["Steering_Drag_Link"], Tl_),
            "Steering_BellCrank": (parts["Steering_BellCrank"], Tc_), "HW_Crank_In_Pin": (refs["HW_Crank_In_Pin"], Tc_), "HW_Crank_Out_Pin": (refs["HW_Crank_Out_Pin"], Tc_),
            "Steering_Rack": (parts["Steering_Rack"], Tr_), "HW_TieRod_Inner_Screw_L": (_hw_tr_L, Tr_), "HW_TieRod_Inner_Screw_R": (_hw_tr_R, Tr_),
            "HW_Crank_Pivot_Pin": (refs["HW_Crank_Pivot_Pin"], I), "HW_Rack_Guide_Screw": (refs["HW_Rack_Guide_Screw"], I)}
_SEATED = {"HW_Crank_Pivot_Pin", "HW_Rack_Guide_Screw"}     # pressed / screwed into the floor: intended contact with bulkhead/tub
_JOINTS = {frozenset(p_) for p_ in (("Servo_Horn", "HW_Link_Pin_A"), ("Steering_Drag_Link", "HW_Link_Pin_A"), ("Steering_Drag_Link", "HW_Crank_In_Pin"), ("Steering_BellCrank", "HW_Crank_In_Pin"),
          ("Steering_BellCrank", "HW_Crank_Out_Pin"), ("Steering_Rack", "HW_Crank_Out_Pin"), ("Steering_BellCrank", "HW_Crank_Pivot_Pin"), ("Steering_Rack", "HW_TieRod_Inner_Screw_L"),
          ("Steering_Rack", "HW_TieRod_Inner_Screw_R"), ("Steering_Rack", "HW_Rack_Guide_Screw"), ("Tie_Rod_L", "HW_TieRod_Inner_Screw_L"), ("Tie_Rod_R", "HW_TieRod_Inner_Screw_R"))}
_PT = {}
def _d(c_, m_, T_):
    if c_.in_collision_single(m_, transform=T_): return 0.0
    d_, dd_ = c_.min_distance_single(m_, transform=T_, return_data=True); _PT["p"] = np.round(dd_.point("o"), 2).tolist(); return float(d_)
def _ovl(ma_, Ta_, mb_, Tb_=None):
    a_ = ma_.copy(); a_.apply_transform(Ta_); b_ = mb_.copy()
    if Tb_ is not None: b_.apply_transform(Tb_)
    try: x_ = trimesh.boolean.intersection([a_, b_], engine=ENG); return x_.volume if len(x_.faces) else 0.0
    except Exception: return -1.0
_res = {}       # (part, target) -> (min dist, s, th, overlap mm3 at the min)
def _upd(k_, d_, s_, th_, ov_=0.0):
    if k_ not in _res or d_ < _res[k_][0] or (d_ == 0.0 and ov_ > _res[k_][3]): _res[k_] = (d_, s_, th_, ov_, _PT.get("p"))
for s_ in np.linspace(-3.0, 3.0, 25):
    g_ = _steer_group(float(s_))
    for n_, (m_, T_) in g_.items():
        if n_ in _SEATED: continue
        for tn_, c_ in (_tgtC if n_ == "Steering_BellCrank" else _tgt).items():
            d_ = _d(c_, m_, T_); _upd((n_, tn_), d_, s_, None, _ovl(m_, T_, _tm[tn_]) if d_ == 0.0 else 0.0)
    ns_ = list(g_)
    for i_ in range(len(ns_)):
        for j_ in range(i_ + 1, len(ns_)):
            if frozenset((ns_[i_], ns_[j_])) in _JOINTS: continue
            ma_, Ta_ = g_[ns_[i_]]; mb_, Tb_ = g_[ns_[j_]]; c_ = trimesh.collision.CollisionManager(); c_.add_object("o", mb_, transform=Tb_)
            d_ = _d(c_, ma_, Ta_); _upd((ns_[i_], ns_[j_]), d_, s_, None, _ovl(ma_, Ta_, mb_, Tb_) if d_ == 0.0 else 0.0)
_ARMS = ("IFS_Upper_Arm_L", "IFS_Upper_Arm_R", "IFS_Lower_Arm_L", "IFS_Lower_Arm_R", "Front_Carrier_L", "Front_Carrier_R", "CVD_Inner_L", "CVD_Inner_R", "Ref_Shock_FL", "Ref_Shock_FR", "Front_Bumper", "Front_Y_Brace", "Servo_Ref", "Ref_Front_Driveshaft",
         "HW_Front_Lower_Hinge_Pin_L", "HW_Front_Lower_Hinge_Pin_R", "HW_Front_Upper_Hinge_Pin_L", "HW_Front_Upper_Hinge_Pin_R")
for th_ in (-12, -10, -8, -4, 0, 4, 6, 8):
    for s_ in np.linspace(-3.0, 3.0, 13):
        sc_, _sol = pose_scene(th_, float(s_))
        if sc_ is None: print("   steer solve fails at arm %+d s %+.2f" % (th_, s_)); continue
        g_ = _steer_group(float(s_))
        for rn_ in ("Tie_Rod_L", "Tie_Rod_R"):
            m_, T_ = sc_[rn_]
            for tn_, c_ in _tgt.items():
                d_ = _d(c_, m_, T_); _upd((rn_, tn_), d_, s_, th_, _ovl(m_, T_, _tm[tn_]) if d_ == 0.0 else 0.0)
            for on_ in list(g_) + [a_ for a_ in _ARMS if a_ in sc_]:
                if frozenset((rn_, on_)) in _JOINTS: continue
                mb_, Tb_ = g_[on_] if on_ in g_ else sc_[on_]; c_ = trimesh.collision.CollisionManager(); c_.add_object("o", mb_, transform=Tb_)
                d_ = _d(c_, m_, T_); _upd((rn_, on_), d_, s_, th_, _ovl(m_, T_, mb_, Tb_) if d_ == 0.0 else 0.0)
_fail = 0
for k_ in sorted(_res, key=lambda k: _res[k][0]):
    d_, s_, th_, ov_, pt_ = _res[k_]
    lim_ = AUD_MIN if (k_[1].startswith(("Bulkhead", "Chassis_Tub"))) else 0.10
    bad_ = (d_ < lim_) if lim_ == AUD_MIN else (d_ == 0.0); _fail += bad_
    if bad_ or d_ < 1.0: print("  %-26s x %-26s min %.3f mm at s=%+.2f%s%s  %s" % (k_[0], k_[1], d_, s_, "" if th_ is None else " arm %+d" % th_, (" OVERLAP %.3f mm3" % ov_) if d_ == 0.0 else (" near %s" % pt_), "FAIL" if bad_ else ("TIGHT" if d_ < lim_ else "ok")))
print(" audit pairs %d ; failures (<%.2f mm to bulkhead/tub, or any overlap): %d" % (len(_res), AUD_MIN, _fail))
# tie rod vs upper arm inside the usable envelope (bump stop +4 deg, lock +/-2.8) and just outside it
for th_, s_ in ((4, -2.8), (4, -3.0), (6, -3.0), (8, -2.8), (8, -3.0)):
    sc_, _sol = pose_scene(th_, s_); c_ = trimesh.collision.CollisionManager(); c_.add_object("o", sc_["IFS_Upper_Arm_L"][0], transform=sc_["IFS_Upper_Arm_L"][1])
    print("   Tie_Rod_L x IFS_Upper_Arm_L at arm %+d, s %+.1f: %.3f mm" % (th_, s_, _d(c_, *sc_["Tie_Rod_L"])))
# ---- Item 1 re-check: front shock clearances over travel -12..+4 (>=1.0 mm): Ø8 body vs bulkhead (eye/neck sit inside the fork tower by design), whole shock + lower screw vs wheel envelope, bumper, Y-brace, upper arm
_cW = _cmo(wenv); _cBu = _cmo(parts["Front_Bumper"]); _cY = _cmo(parts["Front_Y_Brace"]); _cBk = _tgt["Bulkhead"]
_sh = {"body->bulkhead": 9.0, "shock->wheel envelope": 9.0, "lower screw->wheel envelope": 9.0, "shock->bumper": 9.0, "shock->Y-brace": 9.0, "shock->upper arm": 9.0, "lower eye->bulkhead": 9.0}
for th_ in np.arange(-12, 4.01, 1.0):
    lo_ = shock_lo(float(th_)); dv_ = (lo_ - SHOCK_UP) / np.linalg.norm(lo_ - SHOCK_UP)
    body_ = seg_cyl(SHOCK_UP + dv_ * SHOCK_BODY_OFF, SHOCK_UP + dv_ * SHOCK_BODY_LEN, SHOCK_BODY_R); full_ = make_shock(SHOCK_UP, lo_)
    low_ = U(seg_cyl(SHOCK_UP + dv_ * (SHOCK_BODY_LEN - 0.5), lo_, SHOCK_ROD_R), cyl("x", list(lo_), SHOCK_EYE_R, SHOCK_EYE_W, 24))
    sol_ = solve_front(float(th_)); scr_ = moving["HW_Front_Shock_Lower_Screw_L"][1]; Ta_ = rotx(sol_["th"], *AL)
    cu_ = trimesh.collision.CollisionManager(); cu_.add_object("o", parts["IFS_Upper_Arm_L"], transform=rotx(sol_["phi"], *AU))
    for k_, v_ in (("body->bulkhead", _d(_cBk, body_, I)), ("lower eye->bulkhead", _d(_cBk, low_, I)), ("shock->wheel envelope", _d(_cW, full_, I)), ("lower screw->wheel envelope", _d(_cW, scr_, Ta_)),
                   ("shock->bumper", _d(_cBu, full_, I)), ("shock->Y-brace", _d(_cY, full_, I)), ("shock->upper arm", _d(cu_, full_, I))):
        _sh[k_] = min(_sh[k_], v_)
print(" front shock min clearances over arm -12..+4 (target >=1.0): " + "; ".join("%s %.2f" % kv_ for kv_ in _sh.items()))
# ---- magnet seats vs the current shell file, shocks and arms
_shl = envf.get("Tacoma_Trophy_Body_Shell")
_seatL = trimesh.boolean.intersection([_bkF, box(MAG_X - 3.2, MAG_X + 3.2, MAG_Y - 3.2, MAG_Y + 3.2, BRIDGE_Z1 + 0.05, MAG_TOP + 0.1)], engine=ENG)
print(" magnet seat L: bounds X %.2f..%.2f Y %.2f..%.2f top Z %.2f ; pocket floor solid %s ; wall solid at r 2.0 %s" % (_seatL.bounds[0][0], _seatL.bounds[1][0], _seatL.bounds[0][1], _seatL.bounds[1][1], _seatL.bounds[1][2],
      bool(_bkF.contains([[MAG_X - 0.6, MAG_Y - 0.6, MAG_TOP - MAG_PDEPTH - 0.3]])[0]), bool(_bkF.contains([[MAG_X + 2.0 * math.cos(a_), MAG_Y + 2.0 * math.sin(a_), MAG_TOP - 1.0] for a_ in np.linspace(0, 6.2, 12)]).all())))
_ms = 9.0; _ma = 9.0
for th_ in np.arange(-12, 8.01, 2.0):
    sol_ = solve_front(float(th_)); cs_ = _cmo(_seatL)
    _ms = min(_ms, _d(cs_, make_shock(SHOCK_UP, shock_lo(float(th_))), I)); _ma = min(_ma, _d(cs_, parts["IFS_Upper_Arm_L"], rotx(sol_["phi"], *AU)))
print(" magnet seat min distance over arm -12..+8: shock %.2f mm, upper arm %.2f mm (steering parts are all below Z 6)" % (_ms, _ma))
if _shl is not None:
    _cS = _cmo(_shl); _hz = [min([z_ for z_ in np.atleast_2d(_shl.ray.intersects_location([[MAG_X + dx_, MAG_Y + dy_, 15.0]], [[0, 0, 1]])[0])[:, 2]] or [0.0]) for dx_, dy_ in ((0, 0), (1.2, 0), (-1.2, 0), (0, 1.2), (0, -1.2))]
    print(" current stl/v4 shell above seat L: underside Z at centre/+-1.2 mm %s ; seat vs shell min distance %.2f mm ; whole bulkhead vs shell %.2f mm" % (np.round(_hz, 2).tolist(), _d(_cS, _seatL, I), _d(_cS, _bkF, I)))

# ================================================================ TRAVEL AUDIT (collision-checker round): non-mated moving pairs incl. hardware, real travel arm -12..+4 x rack +/-2.8 (and the +/-3.0 hard limit)
print("\n=== TRAVEL AUDIT: arm -12..+4 x rack 0/+-2.8/+-3.0 : knuckle-side pairs incl. hardware (min distance, overlap) ===")
_TP = [("HW_TieRod_Outer_Screw_L", "IFS_Lower_Arm_L"), ("Front_Spindle_L", "IFS_Lower_Arm_L"), ("Front_Spindle_L", "IFS_Upper_Arm_L"), ("CVD_Outer_L", "Front_Carrier_L"), ("CVD_Inner_L", "Front_Carrier_L"),
       ("IFS_Lower_Arm_L", "IFS_Bulkhead_Convertible"), ("IFS_Upper_Arm_L", "IFS_Bulkhead_Convertible"), ("Front_Spindle_L", "Tie_Rod_L"), ("HW_TieRod_Outer_Screw_L", "IFS_Upper_Arm_L"), ("Tie_Rod_L", "IFS_Lower_Arm_L"),
       ("HW_Front_Lower_Kingpin_L", "IFS_Lower_Arm_L"), ("HW_Front_Upper_Kingpin_L", "IFS_Upper_Arm_L"), ("Front_Spindle_L", "Front_Carrier_L")]
_tr28 = {}; _tr30 = {}
for th_ in (-12, -10, -8, -4, 0, 4):
    for s_ in (-3.0, -2.8, -1.4, 0.0, 1.4, 2.8, 3.0):
        sc_, _sol = pose_scene(th_, s_)
        if sc_ is None: continue
        for a_, b_ in _TP:
            c_ = trimesh.collision.CollisionManager(); c_.add_object("o", sc_[b_][0], transform=sc_[b_][1]); d_ = _d(c_, *sc_[a_])
            ov_ = _ovl(sc_[a_][0], sc_[a_][1], sc_[b_][0], sc_[b_][1]) if d_ == 0.0 else 0.0
            for R_ in ((_tr28, _tr30) if abs(s_) <= 2.8 else (_tr30,)):
                if (a_, b_) not in R_ or (d_, -ov_) < (R_[(a_, b_)][0], -R_[(a_, b_)][1]): R_[(a_, b_)] = (d_, ov_, th_, s_, _PT.get("p"))
for a_, b_ in _TP:
    r1_ = _tr28[(a_, b_)]; r2_ = _tr30[(a_, b_)]
    print("  %-24s x %-24s | lock 2.8: %.3f mm (ov %.3f) at arm %+d s %+.1f near %s | incl. 3.0: %.3f mm (ov %.3f) at arm %+d s %+.1f" % (a_, b_, r1_[0], r1_[1], r1_[2], r1_[3], r1_[4], r2_[0], r2_[1], r2_[2], r2_[3]))
_cg = _cmo(refs["Servo_Ref"]); print("  HW_Front_Driveshaft_UJ_Gbx x Servo_Ref (static, coupler rotates): %.3f mm ; Ref_Front_Driveshaft x Servo_Ref %.3f mm" % (_d(_cg, refs["HW_Front_Driveshaft_UJ_Gbx"], I), _d(_cg, refs["Ref_Front_Driveshaft"], I)))
