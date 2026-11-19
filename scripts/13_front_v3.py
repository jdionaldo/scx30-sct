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
XC1, Y_N, ZC1 = 33.5, 1.8, 10.8 + SERVO_DZ   # (was 10.8) case front (spline-end) face X, non-ear (inner) face Y, case top Z (cage tube X 26.9..29.2 is at Z >= 12.1; tube X 34..36.1 at Z >= 10.4)
XC0 = XC1 - SERVO_H; Y_E = Y_N + SERVO_W; ZC0 = ZC1 - SERVO_L; ZCM = (ZC0 + ZC1) / 2
YSP, ZSP = Y_N + SPL_V, ZC1 - SPL_U                      # spline axis (Y, Z); axis along X
PILOTS = [(Y_N + HOLE_V, ZCM + g * HOLE_SP / 2) for g in (1, -1)]   # ear-screw axes (Y, Z); screws run +X through the ears into the front wall
HORN_T = 1.5; HORN_X0 = XC1 + 0.05; HORN_X1 = HORN_X0 + HORN_T
ZPIN = 1.5
HORN_R = ZSP - ZPIN             # horn crank length (pin at ZPIN straight below the spline at centre)           # link pin height (Z) and rack-lug pin Y at centre
LINK_T = 1.2; LINK_X0 = HORN_X1 + 0.1; LINK_X1 = LINK_X0 + LINK_T
XA = (LINK_X0 + LINK_X1) / 2   # horn-pin line X
# ---- bell crank (vertical pivot pin on the bulkhead floor, XY plane): servo horn -> drag link -> crank input arm (-X); output arm (+X) pin rides an X-slot in the rack plate
XP, YPC, AO, AI = 32.4, -8.4, 5.4, 3.4      # pivot (X,Y), output arm, input arm lengths
CK_Z0, CK_Z1 = 0.1, 1.3                     # crank plane
PL_Z0, PL_Z1 = 1.4, 2.5                     # rack slot plate / drag-link plate plane (just above the crank)
ZI = (PL_Z0 + PL_Z1) / 2
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
# wire clip (pad on the bulkhead + lid): wall >= 0.8 mm between groove, screw holes and edges
CLIP_X0, CLIP_X1, CLIP_GX, CLIP_GR, CLIP_ZT = 22.9, 29.9, 26.4, 0.7, 12.0 + SERVO_DZ   # lid roof over the groove = CLIP_ZT-10.5-CLIP_GR = 0.8
CLIP_SX = (24.3, 28.5)            # screw x: groove wall = 2.1-0.55-0.7 = 0.85 ; edge wall 24.3-0.55-22.9 = 0.85
GBX_SHAFT_Z = 4.1 + SERVO_DZ      # (was 4.1) gearbox output shaft height, raised with the stack; UNVERIFIED gearbox output-shaft height (spec update 3)
GBX_STUB_X = 15.5               # end of the gearbox output stub (Ref_Gearbox mesh: Ø3 stub on Y0 / Z4.1, case face X6, stub end X15.5)
GBX_STUB_R = 1.5                # stub radius (mesh)
CPL_LEN, CPL_OD, CPL_BORE_R, CPL_CAP = 3.5, 3.5, 1.55, 1.0   # coupler sleeve: slides over the stub (bore Ø3.1), 1.0 mm cap; the shaft starts on the cap face
GBX_X = GBX_STUB_X + CPL_CAP      # front shaft start (coupler cap face), X16.5
WORM_IN = (39.8, 0.0, 6.85)     # front worm input (existing Ref_Front_Input_Worm_Gear)
TUN_R = 1.6                     # shaft tunnel radius (Ø2.2 shaft + 0.5)
RACK_X, ROD_Y0, ROD_Z, ROD_T = 37.8, 6.0, 3.95, 1.1   # rack cross-bar BEHIND the horn pin (stem reaches forward): tie rods stay behind the arms (X<41) like v2
ACK_A = 6.5
th_a = math.atan2(YS, KX + 59.0)
ARM_PT = (KX - ACK_A * math.cos(th_a), YS - ACK_A * math.sin(th_a))   # tie-rod outer pivot (X,Y)

parts = {}                     # name -> mesh (LEFT unless noted)

# ---------------------------------------------------------------- arms
la = load("IFS_Lower_Arm_L")
la = D(la, box(40, 58, 27.0, 40, -5, 6))
tip = box(44.2, 51.8, 26.0, 34.4, -1.5, 1.5)
tip = D(tip, box(46.3, 49.7, 29.0, 36, -5, 5))
la = U(la, tip)
la = D(la, cyl("x", [KX, PL[0], PL[1]], BORE_HINGE_D / 2, 12))
parts["IFS_Lower_Arm_L"] = la

ua = load("IFS_Upper_Arm_L")
ua = D(ua, box(40, 58, 27.0, 40, 3, 12))
tip = box(44.2, 51.8, 26.0, 32.8, 5.3, 8.3)
tip = D(tip, box(46.3, 49.7, 28.5, 36, 3, 11))
ua = U(ua, tip)
ua = D(ua, cyl("x", [KX, PU[0], PU[1]], BORE_HINGE_D / 2, 12))
parts["IFS_Upper_Arm_L"] = ua

# ---------------------------------------------------------------- carrier
def tongue(py, pz, zlo, zhi):
    return U(box(46.5, 49.5, py, 35.0, zlo, zhi), cyl("x", [KX, py, pz], (zhi - zlo) / 2, 3.0, 28))
low_t = tongue(PL[0], PL[1], -1.4, 1.4)
up_t = tongue(PU[0], PU[1], 5.3, 8.3)
bars = [box(KX - 3.5, KX - 2.3, 34.7, 37.5, -1.5, 8.3), box(KX + 2.3, KX + 3.5, 34.7, 37.5, -1.5, 8.3)]
webs = [box(KX - 3.5, KX + 3.5, 34.7, 37.5, -1.5, 0.5), box(KX - 3.5, KX + 3.5, 34.7, 37.5, 5.5, 8.3)]
car = U(low_t, up_t, *bars, *webs)
car = D(car, cyl("x", [KX, PL[0], PL[1]], BORE_HINGE_D / 2, 12), cyl("x", [KX, PU[0], PU[1]], BORE_HINGE_D / 2, 12),
        cyl("z", [KX, YS, 7.0], BORE_KP_D / 2, 14, 12), cyl("z", [KX, YS, -0.5], BORE_KP_D / 2, 6, 12))
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
             cyl("z", [ARM_PT[0], ARM_PT[1], 2.55], 1.9, 1.1, 24))
sp = U(wall, ear_u, ear_l, arm_path)
sp = D(sp, cyl("y", [KX, 40.7, ZH], 2.05, 6.0, 40),                         # bearing seat 4.1
       cyl("z", [KX, YS, 9.5], BORE_KP_D / 2, 4, 12), cyl("z", [KX, YS, -2.7], BORE_KP_D / 2, 4, 12),
       cyl("z", [ARM_PT[0], ARM_PT[1], 2.55], 0.8, 4, 16))
parts["Front_Spindle_L"] = sp

# ---------------------------------------------------------------- bulkhead: servo cradle (servo ON ITS SIDE), shaft tunnel, rack deck
bk = load("IFS_Bulkhead_Convertible")
V_ORIG = bk.volume
bk = D(bk, box(24, 36.1, 13.45, 15, -3, 13), box(24, 36.1, -15, -13.45, -3, 13))     # side walls flush with rail inner faces
front_wall = box(XC1 + 0.15, XC1 + 2.6, Y_E + 0.15, Y_E + EAR_OUT + 0.25, ZC0 + 0.2, ZC1 - 0.9)   # carries the two ear-screw pilots (screws go forward through the ears)
cage_ = trimesh.load(next(f_ for f_ in (os.path.join(os.path.dirname(__file__), "..", "stl", d_, n_ + ".stl") for n_ in ("Cage", "Roll_Cage") for d_ in ("v4", "v3")) if os.path.exists(f_)))   # read-only: keep the pilot wall off the cage tubes (+0.3)
cc_ = trimesh.boolean.intersection([cage_, box(30, 40, 6, 17, -6, 14)], engine=ENG)
if len(cc_.faces): front_wall = D(front_wall, U(*[cc_.copy().apply_translation(d_) for d_ in ([0, 0, 0], [.3, 0, 0], [-.3, 0, 0], [0, .3, 0], [0, -.3, 0], [0, 0, .3], [0, 0, -.3])]))
bk = U(bk,  front_wall, box(33.5, 38.5, -4.2, 4.2, -1.6, 1.8))                          # + rack deck (guide-screw boss)
bk = D(bk, box(XC0 - 0.15, XC1 + 0.15, Y_N - 0.15, Y_E + 0.15, ZC0 - 0.15, 40),                      # case pocket (open top: servo drops in)
       box(XC1 - EAR_T - 0.15, XC1 + 0.15, Y_E, Y_E + EAR_OUT + 0.3, ZC0 - 0.15, 40),                 # ear-tab recess
       box(30.8, XC1 - EAR_T - 0.1, Y_E + 0.15, Y_E + EAR_OUT + 0.6, ZC0 - 0.15, 40),                 # screw-head / driver access behind the tabs
       cyl("x", [XC1 + 1.4, YSP, ZSP], SPL_D / 2 + 0.6, 3.2, 28))                                     # boss / spline relief
for (py_, pz_) in PILOTS:
    bk = D(bk, cyl("x", [XC1 + 1.35, py_, pz_], 0.45, 2.4, 14))                                       # Ø0.9 pilot for M1.0 x 4 (self tapping)
SH0 = np.array([GBX_X + 0.5, 0.0, GBX_SHAFT_Z + (WORM_IN[2] - GBX_SHAFT_Z) * 0.5 / (WORM_IN[0] - GBX_X)])
SH1 = np.array(WORM_IN)
def seg_cyl(p0, p1, r, n=24):
    p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0)
    m = trimesh.creation.cylinder(radius=r, height=L, sections=n)
    m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (p1 - p0) / L)); m.apply_translation((p0 + p1) / 2); return m
bk = D(bk, seg_cyl(SH0, SH1 + np.array([4.0, 0, 0.55]), TUN_R))
bk = D(bk, seg_cyl(SH0 + (SH1 - SH0) * ((35.6 - SH0[0]) / (SH1[0] - SH0[0])), SH1 + np.array([0.5, 0, 0.05]), 2.0))      # Ø4.0 seat for the worm-end dogbone/U-joint coupler
# ---- front shock: Injora SCX30-08 (26 mm extended / 18.3 compressed, 7.7 travel, 1.0 g; body Ø8-9 with spring, eye centre to end ~2 mm). Shock plane X58.6, upper eye raised from (Y9.2,Z10.4) to (Y9.0,Z17.0) on a bulkhead fork tower so ride sits near mid-stroke and the Ø8 body clears the hinge pins/retainer.
SHOCK_UP = np.array([58.6, 9.0, 17.0]); SHOCK_LO0 = np.array([58.6, 24.0, 2.5])   # eye centres (upper on the bulkhead, lower on the lower arm at arm angle 0)
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
rack = U(box(RACK_X - 1.2, RACK_X + 1.2, -7.4, 7.4, 2.1, 3.0), box(35.4, 39.3, YPC - 1.7, -6.8, PL_Z0, PL_Z1))   # bar + slot plate (below the right tie rod, Z < 2.7)
rack = D(rack, box(RACK_X - 0.85, RACK_X + 0.85, -3.8, 3.8, 2.0, 6.0),      # guide slot (7.6 long)
         cyl("z", [RACK_X, ROD_Y0, 2.5], 0.8, 6, 16), cyl("z", [RACK_X, -ROD_Y0, 2.5], 0.8, 6, 16),
         box(36.2, 38.45, YPC - 0.55, YPC + 0.55, 1.0, 4.0))                 # X-elongated slot for the crank output pin
parts["Steering_Rack"] = rack

HXC = (HORN_X0 + HORN_X1) / 2
horn = U(cyl("x", [HXC, YSP, ZSP], 3.3, HORN_T, 36), box(HORN_X0, HORN_X1, YSP - 1.5, YSP + 1.5, ZPIN, ZSP), cyl("x", [HXC, YSP, ZPIN], 1.85, HORN_T, 24))
horn = D(horn, cyl("x", [HXC, YSP, ZSP], 1.95, 3, 24), cyl("x", [HXC, YSP, ZPIN], 0.65, 3, 16))     # Ø3.9 spline bore; Ø1.3 pin hole
parts["Servo_Horn"] = horn
def seg_plate(p0, p1, w, z0, z1):
    p0, p1 = np.array(p0[:2], float), np.array(p1[:2], float); Lh = float(np.linalg.norm(p1 - p0))
    m = box(0, Lh, -w / 2, w / 2, z0, z1); m.apply_transform(rotz(math.atan2(*(p1 - p0)[::-1]), 0, 0)); m.apply_translation([p0[0], p0[1], 0]); return m
KE = (34.8, -1.5)      # elbow: keeps the link 1.8 mm clear of the servo case corner
link = U(cyl("x", [XA, YSP, ZPIN], 1.9, LINK_T, 24), seg_plate((XA + 0.3, YSP), KE, 1.2, PL_Z0, PL_Z1), seg_plate(KE, (I0[0], I0[1]), 1.6, PL_Z0, PL_Z1),
         cyl("z", [KE[0], KE[1], ZI], 0.8, PL_Z1 - PL_Z0, 16), cyl("z", [I0[0], I0[1], ZI], 1.8, PL_Z1 - PL_Z0, 24))
link = D(link, cyl("x", [XA, YSP, ZPIN], 0.65, 3, 16), cyl("z", [I0[0], I0[1], ZI], 0.55, 4, 16))
parts["Steering_Drag_Link"] = link
CZ = (CK_Z0 + CK_Z1) / 2; CH = CK_Z1 - CK_Z0
crank = U(cyl("z", [XP, YPC, CZ], 2.4, CH, 32), box(XP, XP + AO, YPC - 1.2, YPC + 1.2, CK_Z0, CK_Z1), cyl("z", [XP + AO, YPC, CZ], 1.8, CH, 24),
          box(XP - AI, XP, YPC - 1.2, YPC + 1.2, CK_Z0, CK_Z1), cyl("z", [XP - AI, YPC, CZ], 1.8, CH, 24))
crank = D(crank, cyl("z", [XP, YPC, CZ], 0.65, 4, 20), cyl("z", [XP + AO, YPC, CZ], 0.55, 4, 16), cyl("z", [XP - AI, YPC, CZ], 0.55, 4, 16))
parts["Steering_BellCrank"] = crank

def rod_mesh(p0, p1, w=2.0, t=ROD_T, eye=1.4):
    p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0)
    m = U(box(0, L, -w / 2, w / 2, -t / 2, t / 2), cyl("z", [0, 0, 0], eye, t, 20), cyl("z", [L, 0, 0], eye, t, 20))
    m = D(m, cyl("z", [0, 0, 0], 0.8, 3, 16), cyl("z", [L, 0, 0], 0.8, 3, 16))
    d = (p1 - p0) / L; ang = math.atan2(d[1], d[0])
    m.apply_transform(rotz(ang, 0, 0)); m.apply_translation(p0); return m
ROD_IN = np.array([RACK_X, ROD_Y0, ROD_Z]); ROD_OUT = np.array([ARM_PT[0], ARM_PT[1], ROD_Z])
ROD_L0 = np.linalg.norm(ROD_OUT - ROD_IN)
parts["Tie_Rod_L"] = rod_mesh(ROD_IN, ROD_OUT)

# ---------------------------------------------------------------- reference / hardware (not printed)
refs = {}
sv = U(box(XC0, XC1, Y_N, Y_E, ZC0, ZC1), cyl("x", [XC1 + BOSS_H / 2, YSP, ZSP], SPL_D / 2, BOSS_H, 20),
       box(XC1 - EAR_T, XC1, Y_E, Y_E + EAR_OUT, ZCM - HOLE_SP / 2 - 1.8, ZCM + HOLE_SP / 2 + 1.8))
sv = D(sv, *[cyl("x", [XC1, py_, pz_], EAR_HOLE_D / 2, 3, 12) for py_, pz_ in PILOTS])
refs["Servo_Ref"] = sv
for i, (py_, pz_) in enumerate(PILOTS):
    refs[f"HW_Servo_Mount_Screw_{'LR'[i]}"] = U(cyl("x", [XC1 + 0.5, py_, pz_], 0.5, 3.4, 14), cyl("x", [XC1 - EAR_T - 0.35, py_, pz_], 1.0, 0.6, 14))
refs["HW_Rack_Guide_Screw"] = U(cyl("z", [RACK_X, 0, 1.4], 0.55, 4.0, 14), cyl("z", [RACK_X, 0, 3.25], 0.9, 0.5, 14))
refs["HW_Horn_Screw"] = U(cyl("x", [XC1 + 2.4, YSP, ZSP], 0.6, 2.0, 12), cyl("x", [XC1 + BOSS_H + 0.25, YSP, ZSP], 1.2, 0.5, 14))
refs["HW_Link_Pin_A"] = U(cyl("x", [(HORN_X0 + 0.4 + LINK_X1) / 2, YSP, ZPIN], 0.6, LINK_X1 - HORN_X0 - 0.4, 12), cyl("x", [LINK_X1 + 0.2, YSP, ZPIN], 1.0, 0.4, 14))
refs["HW_Crank_Pivot_Pin"] = U(cyl("z", [XP, YPC, -0.05], 0.6, 2.9, 14), cyl("z", [XP, YPC, CK_Z1 + 0.15], 1.1, 0.3, 16))      # Ø1.2 steel pin, pressed into the floor boss
refs["HW_Crank_Out_Pin"] = U(cyl("z", [XP + AO, YPC, 1.5], 0.5, 3.0, 12), cyl("z", [XP + AO, YPC, PL_Z1 + 0.15], 0.9, 0.3, 14))     # Ø1.0 pin: crank hole -> rack slot
refs["HW_Crank_In_Pin"] = U(cyl("z", [I0[0], I0[1], 1.5], 0.5, 3.0, 12), cyl("z", [I0[0], I0[1], PL_Z1 + 0.15], 0.9, 0.3, 14))      # Ø1.0 pin: crank hole -> drag-link eye
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
# tie-rod screws (M1.4) L side
hw_tr = {"HW_TieRod_Outer_Screw_L": cyl("z", [ARM_PT[0], ARM_PT[1], 3.5], 0.7, 3.4, 14),
         "HW_TieRod_Inner_Screw_L": cyl("z", [RACK_X, ROD_Y0, 3.2], 0.7, 4.0, 14)}

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

# moving-group definitions: each entry (mesh_L, group) ; groups: 'arm_l','arm_u','car','spin','cvd_in', ...
moving = {"IFS_Lower_Arm_L": ("arm_l", parts["IFS_Lower_Arm_L"]), "IFS_Upper_Arm_L": ("arm_u", parts["IFS_Upper_Arm_L"]),
          "Front_Carrier_L": ("car", parts["Front_Carrier_L"]), "Front_Spindle_L": ("spin", parts["Front_Spindle_L"]),
          "Ref_Wheel_FL": ("spin", allb["Ref_Wheel_FL"].copy().apply_translation([0, WHEEL_SHIFT, 0])),
          "HW_Front_Wheel_Nut_L": ("spin", allb["HW_Front_Wheel_Nut_L"].copy().apply_translation([0, WHEEL_SHIFT, 0])),
          "CVD_Outer_L": ("spin", cvd_out), "Front_Bearing_L": ("spin", brg), "Front_Bearing_Inner_L": ("spin", brg_in),
          "HW_Front_Upper_Kingpin_L": ("car", hw_kp["HW_Front_Upper_Kingpin_L"]), "HW_Front_Lower_Kingpin_L": ("car", hw_kp["HW_Front_Lower_Kingpin_L"]),
          "HW_Front_Lower_Outer_Pin_L": ("car", hw_hinge["HW_Front_Lower_Outer_Pin_L"]), "HW_Front_Upper_Outer_Pin_L": ("car", hw_hinge["HW_Front_Upper_Outer_Pin_L"]),
          "HW_TieRod_Outer_Screw_L": ("spin", hw_tr["HW_TieRod_Outer_Screw_L"]),
          "CVD_Inner_L": ("cvd", cvd_in), "Tie_Rod_L": ("rod", parts["Tie_Rod_L"]),
          "HW_TieRod_Inner_Screw_L": ("rack", hw_tr["HW_TieRod_Inner_Screw_L"]),
          "Ref_Shock_FL": ("shock", make_shock(SHOCK_UP, SHOCK_LO0)), "HW_Front_Shock_Lower_Screw_L": ("arm_l", allb["HW_Front_Shock_Lower_Screw_L"])}
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
    sc["Servo_Horn"] = (parts["Servo_Horn"], rotx(horn_angle(s), YSP, ZSP))
    sc["Steering_Drag_Link"] = (parts["Steering_Drag_Link"], link_T(s)); sc["Steering_BellCrank"] = (parts["Steering_BellCrank"], crank_T(s))
    sc["Steering_Rack"] = (parts["Steering_Rack"], tr([0, s, 0]))
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

# ---------- swept steering relief in the bulkhead (rack, horn+pin, tie rods over steering x travel), then final rest check
sweep = []
for th_ in (-12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8):
    for s_ in np.linspace(-3.0, 3.0, 7):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        for k_ in ("Steering_Rack", "Servo_Horn", "Steering_Drag_Link", "Steering_BellCrank", "Tie_Rod_L", "Tie_Rod_R"):
            m_, T_ = sc_[k_]; mm_ = m_.copy(); mm_.apply_transform(T_); sweep.append(mm_)
for s_ in np.linspace(-3.4, 3.4, 35):
    for k_, m_, T_ in (('h', parts['Servo_Horn'], rotx(horn_angle(s_), YSP, ZSP)), ('l', parts['Steering_Drag_Link'], link_T(s_)), ('c', parts['Steering_BellCrank'], crank_T(s_)), ('r', parts['Steering_Rack'], tr([0, s_, 0]))):
        mm_ = m_.copy(); mm_.apply_transform(T_); sweep.append(mm_)
# upper arm (+ bump to +12 deg, droop to -4) sweeps through the cradle floor: cut it so the floor can sit low
for th_ in np.arange(-4, 12.1, 2):
    sol_ = solve_front(float(th_)); mm_ = parts["IFS_Upper_Arm_L"].copy(); mm_.apply_transform(rotx(sol_["phi"], *AU)); sweep.append(mm_)
cutv = U(*sweep)
# inflate by 0.25 mm: union of slightly shifted copies
infl = U(*[cutv.copy().apply_translation(d_) for d_ in ([0.3517, 0.0013, 0.0007], [-0.3483, -0.0011, 0.0009], [0.0006, 0.3521, -0.0008], [-0.0004, -0.3479, 0.0012], [0.0010, 0.0005, 0.3513], [-0.0007, 0.0003, -0.3487], [0, 0, 0])])
bk = D(parts["IFS_Bulkhead_Convertible"], infl)
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)   # drop zero-volume slivers
bk = D(bk, cyl("z", [RACK_X, 0.0, 0.3], 0.55, 4.8, 16))   # rack guide-screw pilot (opened after the rack channel is cut)
# --- tub mounting: 4 vertical M1.6 screws through the floor (Ø1.8 hole, Ø3.2 x 1.0 counterbore from the top), in the solid floor outside all sweeps
# bell-crank pivot boss (re-added after the relief; the crank sweep would otherwise shave it) with Ø1.1 pin pilot; wire-clip pad with two pilots and a cable groove
bk = D(bk, box(35.0, 37.0, 3.5, 4.8, -0.5, 0.4))   # removes a tangent sliver left between the horn-boss and link sweeps (non-manifold edge after STL roundtrip)
bk = D(bk, cyl("z", [XP, YPC, 1.05], 3.0, 2.0, 40))   # clears the crank hub disc (tangent slivers between the rotated sweep copies)
# upper shock fork tower: two 0.8 mm walls either side of the eye (eye 2.4 wide), hull of the eye boss and a base block on the bulkhead top; Ø1.7 pin bore along X
TW_W, TW_R, TW_Y0, TW_Y1, TW_Z0 = 0.8, 2.2, 5.0, 7.6, 11.5
for _xs in (SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1 - TW_W, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1):
    _w = trimesh.convex.convex_hull(np.vstack([cyl("x", [_xs + TW_W / 2, SHOCK_UP[1], SHOCK_UP[2]], TW_R, TW_W, 28).vertices, box(_xs, _xs + TW_W, TW_Y0, TW_Y1, TW_Z0, SHOCK_UP[2] + 1.0).vertices]))
    bk = U(bk, _w, mirror(_w))
bk = D(bk, cyl("x", [SHOCK_UP[0], SHOCK_UP[1], SHOCK_UP[2]], 0.85, 8.0, 20), cyl("x", [SHOCK_UP[0], -SHOCK_UP[1], SHOCK_UP[2]], 0.85, 8.0, 20))
print(" shock fork tower: walls X%.1f..%.1f / %.1f..%.1f, base solid under it? %s" % (SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1 - TW_W, SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.1, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1, SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.1 + TW_W, bool(bk.contains([[SHOCK_UP[0] - SHOCK_EYE_W / 2 - 0.5 - TW_W / 2, (TW_Y0 + TW_Y1) / 2, TW_Z0 - 0.5], [SHOCK_UP[0] + SHOCK_EYE_W / 2 + 0.5 + TW_W / 2, (TW_Y0 + TW_Y1) / 2, TW_Z0 - 0.5]]).all())))
bk = U(bk, cyl("z", [XP, YPC, -0.95], 2.4, 1.9, 32), box(CLIP_X0, CLIP_X1, Y_E + 0.15, 13.4, 8.0, 10.5 + SERVO_DZ))
bk = D(bk, cyl("z", [XP, YPC, -0.65], 0.55, 1.3, 16), cyl("z", [CLIP_SX[0], 12.65, 9.3 + SERVO_DZ], 0.45, 2.4, 14), cyl("z", [CLIP_SX[1], 12.65, 9.3 + SERVO_DZ], 0.45, 2.4, 14), cyl("y", [CLIP_GX, 12.65, 10.5 + SERVO_DZ], CLIP_GR, 3.0, 16))
# ---- UPDATE 12 (Yeti-style bumper + Y-brace): mount pad under the nose floor + two screw posts floor->top skin
BMX, BMY = 61.8, 3.0                    # bumper through-bolt axes (X, +-Y): vertical, through plate, floor, post, top skin and the Y-brace ear
BPAD_Z0 = 2.9                           # flat underside of the pad = flat top of the bumper centre plate
bk = U(bk, box(60.2, 63.6, -8.5, 8.5, BPAD_Z0, 4.3), *[cyl("z", [BMX, s_ * BMY, 8.2], 1.7, 5.9, 24) for s_ in (1, -1)])
bk = D(bk, *[cyl("z", [BMX, s_ * BMY, 7.8], 0.8, 11.8, 16) for s_ in (1, -1)])
BH_HOLES = [(44.5, 3.5), (44.5, -3.5), (52.0, 3.5), (52.0, -3.5)]
WEB_Z0, WEB_TOP, SEAT_TOP = -0.9, 0.5, 0.8     # UPDATE 14: floor web underside raised -1.9 -> -0.9 (meets the raised tub); web top stays 0.5 (diff ring bottom is Z0.85); screw seats are pads up to 0.8
bk = D(bk, box(24.0, 40.5, -14.0, 14.0, -6.0, WEB_Z0), box(40.5, 64.0, -8.5, 8.5, -6.0, WEB_Z0))      # trim the underside (hinge-pin bosses at |Y|>8.5 untouched)
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)
bk = U(bk, *[cyl("z", [hx_, hy_, (0.2 + SEAT_TOP) / 2], 2.1, SEAT_TOP - 0.2, 28) for (hx_, hy_) in BH_HOLES])
BH_FLOOR_Z0, BH_FLOOR_Z1 = WEB_Z0, SEAT_TOP
for (hx_, hy_) in BH_HOLES:
    ok_ = bool(bk.contains([[hx_ + 1.3, hy_, 0.0], [hx_ - 1.3, hy_, 0.0], [hx_, hy_ + 1.3, 0.0], [hx_, hy_ - 1.3, 0.0]]).all())
    print("   tub screw hole (%.1f, %.1f): floor solid around it at Z 0? %s" % (hx_, hy_, ok_))
    bk = D(bk, cyl("z", [hx_, hy_, 0.0], 0.9, 3.4, 20), cyl("z", [hx_, hy_, SEAT_TOP - 0.45 + 0.9], 1.6, 1.8, 24))   # Ø1.8 through, Ø3.2 counterbore 0.45 deep from the seat top
for (py_, pz_) in PILOTS:   # re-open the ear-screw pilots (along X) and verify the front wall still carries them
    bk = D(bk, cyl("x", [XC1 + 1.35, py_, pz_], 0.45, 2.4, 14))
    print("   ear pilot (Y %.1f, Z %.1f): wall solid 0.6 mm beside it? %s" % (py_, pz_, bool(bk.contains([[XC1 + 1.35, py_ + 0.95, pz_]])[0])))
bk = D(bk, box(35.4, 37.2, -1.8, 2.8, 1.0, 1.6))   # UPDATE 13: levels the deck under the link/horn sweep (a 0.05 mm sliver sheet at Z~1.1 made 4-face edges after the STL roundtrip)
bk = max(bk.split(only_watertight=False), key=lambda s: s.volume)   # drop slivers again
parts["IFS_Bulkhead_Convertible"] = bk; static["IFS_Bulkhead_Convertible"] = bk; rest_all["IFS_Bulkhead_Convertible"] = bk
# ---------- lower-arm relief: spindle swept over droop (-12..-8) x steering lock, cut from the lower arm (window around the contact only)
ARM_RELIEF_CLR = 0.15
_sp = []
for th_ in (-12, -11, -10, -9, -8):
    for s_ in (-3.0, -2.8, -2.0, -1.0, 0.0, 1.0, 2.0, 2.8, 3.0):
        sc_, sol_ = pose_scene(th_, s_)
        if sc_ is None: continue
        Tr_ = np.linalg.inv(sc_["IFS_Lower_Arm_L"][1]) @ sc_["Front_Spindle_L"][1]; m_ = parts["Front_Spindle_L"].copy(); m_.apply_transform(Tr_); _sp.append(m_)
_sw = U(*_sp); _o = ARM_RELIEF_CLR
_sw = U(*[_sw.copy().apply_translation(d_) for d_ in ([_o, 0.0011, 0.0007], [-_o, -0.0011, 0.0009], [0.0006, _o, -0.0008], [-0.0004, -_o, 0.0012], [0.0010, 0.0005, _o], [-0.0007, 0.0003, -_o])])
_win = box(43.0, 47.5, 26.5, 32.0, -1.0, 4.5)   # arm-local frame (arm at rest)
_cut = trimesh.boolean.intersection([_sw, _win], engine=ENG)
print("  relief cut: %d faces, vol %.3f" % (len(_cut.faces), _cut.volume if len(_cut.faces) else 0))
_la = D(parts["IFS_Lower_Arm_L"], _cut) if len(_cut.faces) else parts["IFS_Lower_Arm_L"]; _la = max(_la.split(only_watertight=False), key=lambda s: s.volume)
print(" lower-arm relief: removed %.2f mm3 (spindle sweep, droop -12..-8 x lock +/-3, clearance %.2f)" % (parts["IFS_Lower_Arm_L"].volume - _la.volume, _o))
parts["IFS_Lower_Arm_L"] = _la; moving["IFS_Lower_Arm_L"] = ("arm_l", _la); printable["IFS_Lower_Arm_L"] = _la; printable["IFS_Lower_Arm_R"] = mirror(_la)
rest_all["IFS_Lower_Arm_L"] = _la; rest_all["IFS_Lower_Arm_R"] = mirror(_la)
print("bulkhead after steering relief: shells %d, vol %.0f mm3 (original %.0f)" % (len(bk.split(only_watertight=False)), bk.volume, V_ORIG))
for i_, s_ in enumerate(bk.split(only_watertight=False)): print("   shell", i_, "vol %.1f" % s_.volume, s_.bounds.round(1).tolist())
# ================================================================ UPDATE 12: YETI-STYLE FRONT BUMPER (bull-bar hoop + skid oval + centre plate, ONE printed part) + Y-BRACE PLATE
BS, BCH = 2.6, 0.5                      # chamfered-square tube: 2.6 mm side, 0.5 mm chamfer (same language as the cage tubes)
BXF, BXR = 65.2, 59.4                   # tube centre lines: hoop + oval front (X), oval rear (X)
BLZ = 0.2                               # oval tube centre Z (tube Z -1.1..1.5; its top overlaps the plate underside 1.3 by 0.2)
BP_Z0, BP_Z1 = 1.3, BPAD_Z0             # centre plate Z (top = flat underside of the bulkhead pad)
HY, HR, HZT, HZB = 8.0, 3.5, 11.4, 1.9  # hoop: leg Y, corner radius, top-bar centre Z, leg foot Z (inside the plate)
YZ0, YZ1 = 12.7, 14.3                   # Y-brace plate Z (bottom rests on the hoop top bar, top-bar top = 11.4+1.3)
def _oct(h, c): return [(h - c, h), (-(h - c), h), (-h, h - c), (-h, -(h - c)), (-(h - c), -h), (h - c, -h), (h, -(h - c)), (h, h - c)]
def sq_seg(p0, p1, nrm, ext=0.0):
    p0 = np.array(p0, float); p1 = np.array(p1, float); L_ = np.linalg.norm(p1 - p0); u_ = (p1 - p0) / L_
    a_ = np.array(nrm, float); a_ = a_ - u_ * np.dot(a_, u_); a_ /= np.linalg.norm(a_); b_ = np.cross(u_, a_)
    return trimesh.convex.convex_hull(np.array([q_ + a_ * x_ + b_ * y_ for q_ in (p0 - u_ * ext, p1 + u_ * ext) for (x_, y_) in _oct(BS / 2, BCH)]))
def arc2(c, R, a0, a1, n): return [(c[0] + R * math.cos(math.radians(t)), c[1] + R * math.sin(math.radians(t))) for t in np.linspace(a0, a1, n + 1)]
def sq_path(P, nrm, closed=False, ext=0.3):
    n_ = len(P); segs = []
    for i_ in range(n_ if closed else n_ - 1):
        p0, p1 = np.array(P[i_], float), np.array(P[(i_ + 1) % n_], float)
        if np.linalg.norm(p1 - p0) < 1e-6: continue
        segs.append(sq_seg(p0, p1, nrm, ext=ext))
    return segs
def cprism(poly, z0, z1, c=0.4):
    """convex polygon prism with the top and bottom perimeter chamfered c"""
    P_ = np.array(poly, float); cen = P_.mean(0)
    e_ = [np.linalg.norm(np.cross(P_[(i + 1) % len(P_)] - P_[i], cen - P_[i])) / np.linalg.norm(P_[(i + 1) % len(P_)] - P_[i]) for i in range(len(P_))]
    k_ = 1 - c / min(e_); Q_ = cen + (P_ - cen) * k_
    pts = [(x, y, z) for (x, y) in Q_ for z in (z0, z1)] + [(x, y, z) for (x, y) in P_ for z in (z0 + c, z1 - c)]
    return trimesh.convex.convex_hull(np.array(pts))
def rect(x0, x1, y0, y1): return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
# --- hoop (frontal arch in plane X = BXF) -------------------------------------------------------------
_hp = [(-HY, HZB)] + arc2((-HY + HR, HZT - HR), HR, 180, 90, 6) + arc2((HY - HR, HZT - HR), HR, 90, 0, 6) + [(HY, HZB)]
_hp3 = [(BXF, y_, z_) for (y_, z_) in _hp]
hoop_tubes = sq_path(_hp3, [1, 0, 0])
# --- skid oval (stadium in plane Z = BLZ) -------------------------------------------------------------
_R = (BXF - BXR) / 2; _XC = (BXF + BXR) / 2; _YC = 6.7
_lp = arc2((_XC, _YC), _R, 0, 180, 12) + arc2((_XC, -_YC), _R, 180, 360, 12)
oval_tubes = sq_path([(x_, y_, BLZ) for (x_, y_) in _lp], [0, 0, 1], closed=True)
# --- centre plate with tabs (bolts to the nose pad) and front foot strip carrying the hoop legs --------
bump = U(*hoop_tubes, *oval_tubes, cprism(rect(60.2, 66.5, -5.4, 5.4), BP_Z0, BP_Z1), cprism(rect(63.9, 66.5, -9.4, 9.4), BP_Z0, BP_Z1))
bump = D(bump, *[cyl("z", [BMX, s_ * BMY, 2.0], 0.8, 4.0, 16) for s_ in (1, -1)], *[cyl("z", [BMX, s_ * BMY, BP_Z0 + 0.15], 1.4, 1.5, 20) for s_ in (1, -1)],   # M1.4 bolt holes + head counterbores (heads sit inside the oval)
         cyl("z", [BXF, 0.0, HZT + 0.3 + 0.1], 0.6, 2.6, 14))                                                                                            # Ø1.2 pilot in the hoop top bar for the Y-brace screw
_loc = trimesh.boolean.intersection([bk, box(55.0, 64.5, -10.0, 10.0, -3.0, 3.2)], engine=ENG)                 # bulkhead nose underside only (small piece: cheap to dilate)
_dil = U(*[_loc.copy().apply_translation(d_) for d_ in ([0, 0, 0], [.15, 0, 0], [-.15, 0, 0], [0, .15, 0], [0, -.15, 0], [0, 0, .15], [0, 0, -.15])])
bump = D(bump, _dil)                                                                                       # 0.15 mm relief where the oval rear corners graze the nose wall
bump = max(bump.split(only_watertight=False), key=lambda s: s.volume)
def _rtok2(m):
    f_ = os.path.join(OUT, "_rtchk2.stl"); m.export(f_); ok_ = trimesh.load(f_).is_watertight; os.remove(f_); return ok_
if not _rtok2(bump):
    for dec_ in (3, 2, 1):   # quantise vertices (float32-STL safe) and drop degenerate faces
        q_ = trimesh.Trimesh(np.round(bump.vertices, dec_), bump.faces, process=True); q_.update_faces(q_.nondegenerate_faces()); q_.remove_unreferenced_vertices()
        q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume)
        ok_ = q_.is_watertight and _rtok2(q_); print("  bumper roundtrip repair: quantise %d dec -> watertight after roundtrip %s, vol %.1f" % (dec_, ok_, q_.volume))
        if ok_: bump = q_; break
# --- Y-brace plate over the nose ------------------------------------------------------------------------
_yp = [cprism(rect(63.4, 66.5, -2.4, 2.4), YZ0, YZ1), cprism([(63.4, -2.4), (63.4, 2.4), (62.0, 4.4), (62.0, -4.4)], YZ0, YZ1), cprism(rect(58.6, 62.0, -4.4, 4.4), YZ0, YZ1)]
for s_ in (1, -1):
    _yp.append(cprism([(58.6, s_ * 4.4), (58.6, s_ * 1.4), (53.9, s_ * 2.2), (53.9, s_ * 5.2)], YZ0, YZ1))
    _yp.append(box(53.9, 55.8, min(s_ * 2.4, s_ * 4.8), max(s_ * 2.4, s_ * 4.8), 9.6, YZ0 + 0.3))              # drop foot onto the deck block (deck top Z 9.5)
ybr = U(*_yp)
ybr = D(ybr, *[cyl("z", [BMX, s_ * BMY, 13.5], 0.8, 3.0, 16) for s_ in (1, -1)], cyl("z", [BXF, 0.0, 13.5], 0.8, 3.0, 16),
         cyl("z", [BXF, 0.0, YZ1 - 0.45], 1.4, 0.9, 20))
ybr = max(ybr.split(only_watertight=False), key=lambda s: s.volume)
# --- hardware (reference bodies, M1.4): bolts + nuts through the nose, front + rear Y-brace screws ---------
def _scr(x_, y_, z_head_top, z_tip, hd=2.6, hh=0.9):
    return U(cyl("z", [x_, y_, (z_head_top + z_tip) / 2], 0.7, abs(z_head_top - z_tip), 16), cyl("z", [x_, y_, z_head_top - hh / 2], hd / 2, hh, 20))
hw = {}
for side_, s_ in (("L", 1), ("R", -1)):
    hw["HW_Front_Bumper_Bolt_" + side_] = _scr(BMX, s_ * BMY, BP_Z0 + 0.9, YZ1 + 1.2 + 0.8)        # head up inside the counterbore? (head seats at Z 1.3..2.2, shank up through everything)
    hw["HW_Front_Bumper_Nut_" + side_] = trimesh.creation.cylinder(radius=1.45, height=1.2, sections=6); hw["HW_Front_Bumper_Nut_" + side_].apply_translation([BMX, s_ * BMY, YZ1 + 0.6])
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
printable["IFS_Bulkhead_Convertible"] = parts["IFS_Bulkhead_Convertible"]   # the RELIEVED bulkhead (was exported unrelieved)
def _rt_ok(m, tag="_rtchk"):
    f_ = os.path.join(OUT, tag + ".stl"); m.export(f_); ok_ = trimesh.load(f_).is_watertight; os.remove(f_); return ok_
_bk = printable["IFS_Bulkhead_Convertible"]
if not _rt_ok(_bk):
    for dec_ in (3, 2):   # quantise vertices (float32-STL safe) and drop degenerate faces
        q_ = trimesh.Trimesh(np.round(_bk.vertices, dec_), _bk.faces, process=True); q_.update_faces(q_.nondegenerate_faces()); q_.remove_unreferenced_vertices()
        q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume) if len(q_.faces) else q_
        print("  bulkhead roundtrip repair: quantise %d dec -> watertight after roundtrip %s, vol %.1f" % (dec_, _rt_ok(q_), q_.volume))
        if _rt_ok(q_): printable["IFS_Bulkhead_Convertible"] = q_; break
if not _rt_ok(printable["IFS_Bulkhead_Convertible"]):      # float32 roundtrip merges two solids that touch along a line (4-face edge): cut a 0.4 mm cube there
    for it_ in range(4):
        m_ = printable["IFS_Bulkhead_Convertible"]; m_.export(os.path.join(OUT, "_rtchk3.stl")); r_ = trimesh.load(os.path.join(OUT, "_rtchk3.stl")); os.remove(os.path.join(OUT, "_rtchk3.stl"))
        u_, c_ = np.unique(r_.edges_sorted, axis=0, return_counts=True); bad_ = u_[c_ != 2]
        if not len(bad_): break
        locs_ = np.unique(np.round(r_.vertices[bad_.ravel()].reshape(-1, 2, 3).mean(1), 1), axis=0)
        print("  bulkhead roundtrip: %d non-manifold edge(s) near %s -> 0.4 mm relief" % (len(bad_), locs_.tolist()))
        q_ = D(m_, *[box(l_[0] - 0.2, l_[0] + 0.2, l_[1] - 0.2, l_[1] + 0.2, l_[2] - 0.2, l_[2] + 0.2) for l_ in locs_]); q_ = max(q_.split(only_watertight=False), key=lambda s: s.volume)
        printable["IFS_Bulkhead_Convertible"] = q_
        if _rt_ok(q_): break
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
wenv.export(os.path.join(OUT, "Front_Wheel_Envelope.stl"))
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
lid = D(box(CLIP_X0, CLIP_X1, Y_E + 0.15, 13.4, 10.5 + SERVO_DZ, CLIP_ZT), cyl("y", [CLIP_GX, 12.65, 10.5 + SERVO_DZ], CLIP_GR, 3.0, 16), cyl("z", [CLIP_SX[0], 12.65, 11.0 + SERVO_DZ], 0.55, 3.0, 14), cyl("z", [CLIP_SX[1], 12.65, 11.0 + SERVO_DZ], 0.55, 3.0, 14))
extra = {"Front_Bump_Stop_L": ring, "Front_Bump_Stop_R": mirror(ring), "Servo_Wire_Clip": lid}
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
for k_ in KEEP_FROM_FOLDER:
    if k_ in allb: static.get(k_, allb[k_]).export(os.path.join(OUT, k_ + ".stl")); exported.append(k_ + " (unchanged copy from per-body folder)")
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
    m_, T_ = sc_r[n_]; mm_ = m_.copy(); mm_.apply_transform(T_); mm_.export(os.path.join(OUT, n_ + ".stl")); _rn.append(n_)
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
print("  servo case top Z %.1f (X%.1f..%.1f, Y%.1f..%.1f); Y-brace top Z %.1f, nuts top %.1f; fork towers top Z %.1f (Y+-5.2..8.8, X57.5..60.2); shock top Z %.1f (Y>=7.5)" % (_sv.bounds[1][2], _sv.bounds[0][0], _sv.bounds[1][0], _sv.bounds[0][1], _sv.bounds[1][1], parts["Front_Y_Brace"].bounds[1][2], static["HW_Front_Bumper_Nut_L"].bounds[1][2] if "HW_Front_Bumper_Nut_L" in static else 0, 19.4, rest_all["Ref_Shock_FL"].bounds[1][2]))

# ---- UPDATE 14: floor web report
_b = parts["IFS_Bulkhead_Convertible"]; _zs = np.arange(-4.0, 2.0, 0.05)
for (hx_, hy_) in BH_HOLES:
    for dy_ in (1.5,):
        c_ = _b.contains(np.array([[hx_ + 2.6 * 0.0, hy_ + (dy_ if hy_ > 0 else -dy_), z_] for z_ in _zs])); zz_ = _zs[c_]
        print(" web at (%.1f,%.1f)+1.5 Y beside the hole: solid Z %.2f..%.2f" % (hx_, hy_, zz_.min(), zz_.max()))
print(" web: underside Z%.1f, top Z%.1f (%.1f thick), screw seat pads top Z%.1f; counterbore floor Z%.2f -> web under the head %.2f mm; bulkhead volume %.0f mm3; min Z of bulkhead %.2f" % (WEB_Z0, WEB_TOP, WEB_TOP - WEB_Z0, SEAT_TOP, SEAT_TOP - 0.45, SEAT_TOP - 0.45 - WEB_Z0, _b.volume, _b.bounds[0][2]))
