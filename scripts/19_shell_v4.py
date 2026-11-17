"""v4 body shell WITHOUT the hood scoop -> stl/v4/Tacoma_Trophy_Body_Shell.stl (writes only stl/v4).
Base = stl/tacoma_trophy_shell.stl, widened like scripts/12 (side panels), then relief against the v4 bodies:
tub, rear cage hoop, deck bays (ESC/RX/battery), servo envelope, bulkhead, gearbox+motor pocket, rear shocks,
rear travel sweeps (arms, links, axle, wheels, bracket, sway bar), front wheel lock/travel envelope (07_front_v2 pose_scene).
usage: BODY_DIR=<per-body stl dir> python scripts/19_shell_v4.py   (run scripts/18_tub.py first; does not run 17_hood_scoop.py)"""
import os, sys, math, io, contextlib, runpy, numpy as np, trimesh
from scipy.optimize import brentq
from trimesh.collision import CollisionManager
BD = os.environ["BODY_DIR"]; V3 = "stl/v3/"; V4 = "stl/v4/"
sys.path.insert(0, "scripts"); import rearlib as R
SH = float(os.environ.get("REAR_SHIFT", 9.0)); R.shift_rear(SH); ROOF_LIFT = float(os.environ.get("ROOF_LIFT", 28.1))      # = 18_tub.py
def _ss(t): t = np.clip(t, 0, 1); return t*t*(3 - 2*t)
def lift(m, H=None):
    H = ROOF_LIFT if H is None else H; m = m.copy(); v = m.vertices.copy(); x, z = v[:, 0], v[:, 2]
    fx = _ss((x + 46.0)/10.0)*(1 - 0.85*_ss((x - 8.0)/16.0)); v[:, 2] += H*_ss((z - 17.0)/12.0)*fx; m.vertices = v; return m
def shift_tail(m):
    m = m.copy(); v = m.vertices.copy(); v[:, 0] -= SH*_ss((-30.0 - v[:, 0])/10.0); m.vertices = v; return m
PL = np.array([-26.0 - SH, float(os.environ.get("PL_Z", 2.4))]); PU = np.array([-24.53 - SH, float(os.environ.get("PU_Z", 12.0))]); R.PL = PL; R.PU = PU; R.A = np.array([-59.0 - SH, -0.5]); R.Bp = np.array([-57.5 - SH, 8.85])   # = 18_tub.py
RAIL_DY = float(os.environ.get("RAIL_DY", 4.0)); WARP_DY = RAIL_DY + 0.1; CLR = float(os.environ.get("CLR", 0.3))
GBX_X, GBX_Y, GBX_Z0, GBX_Z1, SHAFT_Z = 6.0, 12.5, -4.9, 23.1, 4.1; MOTOR_R, MOTOR_L, MOTOR_Z = 8.5, 10.0, 18.1; MOTOR_X1 = -(GBX_X + 0.1); MOTOR_X0 = MOTOR_X1 - MOTOR_L; MC_PX = (-14.5, -10.0)
BAR_X, BAR_Z, BAR_Y, BAR_E_X = -78.5 - SH, 16.3 + 4.2, 19.75, -46.75 - SH; TAB_X, TAB_TOP_Z, LINK_INS = -46.75 - SH, 4.9, 0.8; WIRE_R, LINK_R = 0.4, 0.45
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
def U(ms): return trimesh.boolean.union(list(ms), engine="manifold")
def D(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def ld(n, d=V4): return trimesh.load(d + n + ".stl")
def dil(m, c):
    ms = [m]
    for d in ([c, 0, 0], [-c, 0, 0], [0, c, 0], [0, -c, 0], [0, 0, c], [0, 0, -c]) + tuple([c*a_/math.sqrt(2), c*b_/math.sqrt(2), 0] for a_ in (1, -1) for b_ in (1, -1)) + tuple([c*a_/math.sqrt(2), 0, c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)) + tuple([0, c*a_/math.sqrt(2), c*b_/math.sqrt(2)] for a_ in (1, -1) for b_ in (1, -1)):   # 18 directions (axes + edge diagonals): thin parts left diagonal gaps with only 6
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
def posed(m, T): m = m.copy(); m.apply_transform(T); return m
def vol(a, b):
    try: return abs(trimesh.boolean.intersection([a, b], engine="manifold").volume)
    except Exception: return float('nan')
def open_edges(m): _, c = np.unique(m.edges_sorted, axis=0, return_counts=True); return int((c == 1).sum())
def warp(m, dy, y0=8.0, y1=10.0):
    m = m.copy(); v = m.vertices.copy(); a = np.abs(v[:, 1]); v[:, 1] += np.sign(v[:, 1])*dy*np.clip((a - y0)/(y1 - y0), 0, 1); m.vertices = v; return m
def pocket(c):
    a = box(-GBX_X - c, GBX_X + c, -GBX_Y - c, GBX_Y + c, GBX_Z0 - c, GBX_Z1 + c)
    m = trimesh.creation.cylinder(radius=MOTOR_R + c, height=MOTOR_L + 2*c, sections=48); m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    m.apply_translation([(MOTOR_X0 + MOTOR_X1)/2, 0, MOTOR_Z])
    cl = box(MC_PX[0] - 1.6 - c, MC_PX[1] + 1.6 + c, -10.4 - c, 10.4 + c, MOTOR_Z, MOTOR_Z + 3.2 + c)      # clamp ears + screw heads
    cl2 = box(MC_PX[0] - 1.6 - c, MC_PX[1] + 1.6 + c, -MOTOR_R - 2.0 - c, MOTOR_R + 2.0 + c, MOTOR_Z, MOTOR_Z + MOTOR_R + 2.0 + c)
    return U([a, m, cl, cl2, box(-15.5 - c, 15.5 + c, -2.1, 2.1, SHAFT_Z - 2.1, SHAFT_Z + 2.1)])

def stretch_tail(m, xb=-48.0, ext=17.0):
    """lengthen the rear of the body: x < xb is stretched linearly so the tail end moves back by ext (planes of constant Y/Z stay planar)"""
    m = m.copy(); v = m.vertices.copy(); x0 = v[:, 0].min(); k = (xb - x0 + ext)/(xb - x0); sel = v[:, 0] < xb; v[sel, 0] = xb - (xb - v[sel, 0])*k; m.vertices = v; return m
TAIL_EXT = float(os.environ.get("TAIL_EXT", 18.0))
BASE = lift(warp(stretch_tail(shift_tail(trimesh.load("stl/tacoma_trophy_shell.stl")), xb=-48.0 - SH, ext=TAIL_EXT), WARP_DY))         # no hood scoop, no rails: only the widened original body
print("base shell: vol %.0f bounds %s watertight %s" % (BASE.volume, BASE.bounds.round(1).tolist(), BASE.is_watertight))

# ---- static keepouts (v4 parts) ----
STATIC = {n: ld(n) for n in ("Chassis_Tub", "Rear_Cage_Hoop", "Rear_Cage_Stay_L", "Rear_Cage_Stay_R", "Ref_ESC_G40", "Ref_Receiver", "Ref_Battery_2S", "Ref_Servo_Envelope", "Roll_Cage", "ESC_RX_Shelf_L", "ESC_RX_Shelf_R")}
STATIC["IFS_Bulkhead_Convertible"] = ld("IFS_Bulkhead_Convertible", V3)
for n in ("Ref_Shock_RL", "Ref_Shock_RR", "Ref_Shock_Eye_L", "Ref_Shock_Eye_R"): STATIC[n] = ld(n)

# ---- rear sweep poses (same maths as 18_tub.py) ----
LR = BAR_E_X - BAR_X; L0 = BAR_Z - (TAB_TOP_Z - LINK_INS)
def wcyl(p0, p1, r): return trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1], float), sections=20)
def bar_state(ta):
    T = R.rot(np.array([TAB_X, TAB_TOP_Z - LINK_INS]), math.radians(ta), PL)
    E = lambda th: np.array([BAR_X + LR*math.cos(th), BAR_Z - LR*math.sin(th)])
    f = lambda th: np.linalg.norm(E(th) - T) - L0
    xs = np.linspace(-1.2, 1.2, 241); fv = [f(x) for x in xs]; best, bd = 0.0, 9
    for i in range(len(xs) - 1):
        if fv[i]*fv[i + 1] <= 0:
            r = brentq(f, xs[i], xs[i + 1])
            if abs(r) < bd: best, bd = r, abs(r)
    return best, E(best), T
MOVE_ARM = {n: ld(n) for n in ("Rear_Trailing_Arm_L", "Rear_Trailing_Arm_R", "HW_Rear_Trailing_Pivot_Screw_L", "HW_Rear_Trailing_Pivot_Screw_R")}
MOVE_UL = {n: ld(n) for n in ("Rear_Upper_Link_L", "Rear_Upper_Link_R", "HW_Rear_Upper_Link_Pivot_Screw_L", "HW_Rear_Upper_Link_Pivot_Screw_R")}
AXG = {n: R.M[n] for n in ("Ref_Rear_Axle", "Ref_Wheel_RL", "Ref_Wheel_RR")}
for n in ("Rear_Axle_Top_Bracket", "HW_Rear_Axle_Top_Pin", "HW_Rear_Trailing_Axle_Nut_L", "HW_Rear_Trailing_Axle_Nut_R", "HW_Rear_Trailing_Axle_Screw_L", "HW_Rear_Trailing_Axle_Screw_R"): AXG[n] = ld(n)
LEV = {sg: wcyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R) for sg in (1, -1)}
CROSS = wcyl((BAR_X, -BAR_Y - 0.4, BAR_Z), (BAR_X, BAR_Y + 0.4, BAR_Z), WIRE_R)
def rear_movers(ta):
    a, u, T = R.solve(ta); th, E_, T_ = bar_state(ta); sc = {}
    for n, m in MOVE_ARM.items(): sc[n] = posed(m, R.roty(a, PL[0], PL[1]))
    for n, m in MOVE_UL.items(): sc[n] = posed(m, R.roty(u, PL[0] if False else PU[0], PU[1]))
    for n, m in AXG.items(): sc[n] = posed(m, T)
    for n, sg in (("L", 1), ("R", -1)):
        sc["Bar_lever_" + n] = posed(LEV[sg], R.roty(th, BAR_X, BAR_Z)); sc["Bar_link_" + n] = wcyl((E_[0], sg*BAR_Y, E_[1]), (T_[0], sg*BAR_Y, T_[1]), LINK_R)
    sc["Bar_cross"] = CROSS; return sc
TA = [float(t) for t in np.arange(-14, 7.01, 1.0)]
touch = set(); POSES = {}
for ta in TA:
    sc = rear_movers(ta); POSES[ta] = sc
    cm = CollisionManager(); cm.add_object("shell", BASE)
    for n, m in sc.items():
        o = CollisionManager(); o.add_object(n, m)
        if cm.in_collision_other(o) or cm.min_distance_other(o) < CLR + 0.1: touch.add(n)     # also movers that only graze the shell
print("rear movers touching the widened base shell:", sorted(touch))
def hull_of(ms): return trimesh.convex.convex_hull(np.vstack([m.vertices for m in ms]))
SW = {n: [] for n in touch}
for ta in np.arange(-14, 7.01, 0.5):
    sc = rear_movers(float(ta))
    for n in touch: SW[n].append(sc[n])
sweep = [hull_of(v) for v in SW.values()]          # one convex hull per touching mover (fewer sliver faces than a union of poses)

# ---- front wheel envelope (lock x travel), same idea as 08_relief.py / 10_verify_front_shell.py ----
sys.argv = ["07", BD]
with contextlib.redirect_stdout(io.StringIO()): ns = runpy.run_path("scripts/07_front_v2.py")
FW = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, 0.0, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: continue
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            if k in sc: FW.append(posed(*sc[k]))
cm = CollisionManager(); cm.add_object("shell", BASE); fw_touch = 0
for w in FW:
    o = CollisionManager(); o.add_object("w", w); fw_touch += bool(cm.in_collision_other(o))
print("front wheel poses touching the base shell: %d of %d" % (fw_touch, len(FW)))
RELIEF_FW = []
for nm in ("L", "R"):      # one hull per front wheel over every pose that touches the base shell
    ws = []
    for w in FW:
        o = CollisionManager(); o.add_object("w", w)
        if cm.in_collision_other(o) and (w.centroid[1] > 0) == (nm == "L"): ws.append(w)
    if ws: RELIEF_FW.append(hull_of(ws))

_FE = trimesh.load("stl/v3/Front_Wheel_Envelope.stl")
for _p in _FE.split(only_watertight=False):      # front-v3 envelope (lock x travel), non-watertight halves -> convex hull each, relieved with the same clearance CLR (>= 0.5)
    if len(_p.vertices) > 8: RELIEF_FW.append(trimesh.convex.convex_hull(_p.vertices))
# ---- which static keepouts touch the base shell ----
use = []
for n, m in STATIC.items():
    v = vol(BASE, m); print("  base shell vs %-26s %.2f mm3" % (n, v))
    _o = CollisionManager(); _o.add_object('m', m); _cs = CollisionManager(); _cs.add_object('s', BASE)
    if v > 0.01 or _cs.min_distance_other(_o) < CLR + 0.5: use.append(m)      # also parts that merely graze the shell      # incl. the cage lugs (the v3 cage itself never touched the shell)
print("  base shell vs gearbox+motor pocket: %.1f mm3" % vol(BASE, pocket(0.0)))

def build(c):
    cuts = [dil(m, c + 0.9) for m in use] + [pocket(0.4 + (c - 0.3)*0.5)]        # static parts (cage, bulkhead, ...) get +0.3 extra: >= 0.8 mm opening
    cuts += [dil(m, c) for m in sweep] + [dil(w, c) for w in RELIEF_FW]
    out = D(BASE, [U(cuts)])
    sh = sorted(out.split(only_watertight=False), key=lambda q: -abs(q.volume)); big = sh[0]
    print("  relief clr %.3f: vol %.0f -> %.0f, shells %d, dropped %s" % (c, BASE.volume, big.volume, len(sh), [(round(abs(q.volume), 1), q.bounds.round(1).tolist()) for q in sh[1:5]]))
    return big
OUTF = V4 + "Tacoma_Trophy_Body_Shell.stl"; final = None
for c in (CLR, CLR + 0.033, CLR + 0.071, CLR + 0.12):
    o = build(c); o.export(OUTF); q = trimesh.load(OUTF)
    print("    roundtrip: watertight %s open %d shells %d" % (q.is_watertight, open_edges(q), len(q.split(only_watertight=False))))
    if q.is_watertight and open_edges(q) == 0 and len(q.split(only_watertight=False)) == 1: final = q; break
print("SHELL: watertight %s open edges %d shells %d vol %.0f bounds %s" % (q.is_watertight, open_edges(q), len(q.split(only_watertight=False)), q.volume, q.bounds.round(1).tolist()))
sh = trimesh.load(OUTF)
# ---- verification vs the finished shell ----
print("--- overlap of the v4 shell with:")
for n, m in STATIC.items(): print("  %-26s %.3f mm3" % (n, vol(sh, m)))
for n, m in (("Ref_Gearbox", ld("Ref_Gearbox")), ("Ref_Motor_GM1614", ld("Ref_Motor_GM1614"))): print("  %-26s %.3f mm3" % (n, vol(sh, m)))
cmx = CollisionManager(); cmx.add_object("shell", sh); worst = []
for ta in TA:
    for n, m in POSES[ta].items():
        o = CollisionManager(); o.add_object(n, m)
        if cmx.in_collision_other(o): worst.append((ta, n, round(vol(sh, m), 3)))
print("rear sweep contacts with the shell (ta, mover, mm3):", worst or "none")
bad = []
for th in (-8, -4, 0, 4, 8):
    for s in (-2.75, -1.4, 0.0, 1.4, 2.75):
        sc, sol = ns["pose_scene"](th, s)
        if sc is None: continue
        for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
            o = CollisionManager(); o.add_object(k, sc[k][0], transform=sc[k][1])
            if cmx.in_collision_other(o): bad.append((th, s, k))
print("front wheel/shell contacts over lock -8..+8 x rack +/-2.75:", bad or "none")

# ---- minimum clearance of every body to the finished shell (mm) ----
print("--- minimum clearance to the shell (mm; < 1.0 flagged) ---")
def mind(m):
    o = CollisionManager(); o.add_object("b", m); return cmx.min_distance_other(o)
names = [n for n in STATIC] + ["Ref_Gearbox", "Ref_Motor_GM1614", "Motor_Clamp", "Ref_Rear_Driveshaft", "Ref_Servo_Envelope"]
rows = []
for n in dict.fromkeys(names): rows.append((n, mind(STATIC[n] if n in STATIC else ld(n))))
for n in ("Rear_Trailing_Arm_L", "Rear_Upper_Link_L", "Rear_Axle_Top_Bracket", "Ref_Rear_Axle", "Ref_Wheel_RL", "Bar_lever_L", "Bar_cross"):
    rows.append((n + " (sweep -14..+7)", min(mind(POSES[ta][n]) for ta in TA if n in POSES[ta])))
for k in ("Ref_Wheel_FL", "Ref_Wheel_FR"):
    d = []
    for th in (-8, 0, 8):
        for s_ in (-2.75, 0.0, 2.75):
            sc, sol = ns["pose_scene"](th, s_)
            if sc is not None and k in sc: d.append(mind(posed(*sc[k])))
    rows.append((k + " (lock/bump)", min(d)))
for n, d in sorted(rows, key=lambda r: r[1]): print("  %-40s %7.2f%s" % (n, d, "   <-- within 1 mm" if d < 1.0 else ""))
print("OVERALL: shell X %.1f..%.1f (L %.1f)  Y +-%.1f  Z %.1f..%.1f (roof)" % (sh.bounds[0][0], sh.bounds[1][0], sh.bounds[1][0] - sh.bounds[0][0], sh.bounds[1][1], sh.bounds[0][2], sh.bounds[1][2]))
