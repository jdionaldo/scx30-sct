"""Chassis/cage/shell/rear-pivot redesign around the real Modrax gearbox -> stl/v3/.
usage (repo root): BODY_DIR=<per-body stl dir> python scripts/12_chassis_gbx.py
Does not touch front parts, stl/v2 or earlier scripts."""
import os, sys, math, numpy as np, trimesh
from scipy.optimize import brentq
sys.path.insert(0, "scripts"); import rearlib as R   # needs BODY_DIR
BD = os.environ["BODY_DIR"]; OUT = "stl/v3/"; os.makedirs(OUT, exist_ok=True)

# ---------------- locked gearbox spec ----------------
GBX_X, GBX_Y = 11.5, 13.0                 # half sizes (rev 4: rotated 90 deg, 23 mm along the shafts)
GBX_Z0, GBX_Z1 = -4.9, 23.1
GBX_SHAFT_Z = 4.1                         # unverified (base+9)
MOTOR_DIR = -1                            # -1 = motor can toward rear (-X), +1 = front
MOTOR_R, MOTOR_L, MOTOR_Z = 8.2, 14.5, 18.1  # GAC GM1614 can: O16.4 x 14.5, axis along X, centre Z = base+23
GBX_CLR = 0.4
RAIL_IN, RAIL_T = 13.5, 5.0               # rail inner face / thickness
RAIL_DY = RAIL_IN - 11.5                  # +2.0 vs the original rails
# ---------------- new rear upper-link front pivot ----------------
PU_X = float(os.environ.get("PU_X", -15.5)); PU_Z = float(os.environ.get("PU_Z", 4.5))   # was (-12.5, 5.25)
RELIEF_CLR = 0.5; BRIDGE_LIFT = 10.0
# ---------------- rear sway bar / spare tire (rev 5) ----------------
BAR_X, BAR_Z, BAR_Y = -63.5, 16.0, 19.75   # bar axis through the bridge, arms at Y = +-BAR_Y (centre of the trailing arm)
BAR_E_X = -46.75                           # lever end X (above the arm tab)
TAB_X, TAB_TOP_Z, LINK_INS = -46.75, 4.9, 0.8   # tab on the trailing arm; link end sinks LINK_INS into the tab
WIRE_R, LINK_R, TUN_R = 0.4, 0.45, 0.9     # wire 0.8 dia, drop link 0.9 dia, bridge tunnel 1.8 dia
SPARE_DZ = 6.0                             # spare tire + carrier lifted to clear the raised bridge

def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
def U(ms): return trimesh.boolean.union(ms, engine="manifold")
def D(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def biggest(m, vmin=20):
    p = [q for q in m.split(only_watertight=False) if abs(q.volume) > vmin]
    return max(p, key=lambda q: q.volume)
def dil(m, c=RELIEF_CLR):
    ms = [m]
    for d in ([c,0,0],[-c,0,0],[0,c,0],[0,-c,0],[0,0,c],[0,0,-c]):
        k = m.copy(); k.apply_translation(d); ms.append(k)
    return U(ms)
def rt(m, path):                          # STL roundtrip
    m.export(path); return trimesh.load(path)
def open_edges(m):
    _, c = np.unique(m.edges_sorted, axis=0, return_counts=True); return int((c == 1).sum())

# ---------------- reference gearbox + pocket ----------------
def gbx_ref():
    b = box(-GBX_X, GBX_X, -GBX_Y, GBX_Y, GBX_Z0, GBX_Z1)
    stubs = []
    for s in (-1, 1):
        c = trimesh.creation.cylinder(radius=1.5, height=4.0, sections=24)
        c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
        c.apply_translation([s*(GBX_X + 2.0 - 0.01), 0, GBX_SHAFT_Z]); stubs.append(c)
    return U([b] + stubs)
def motor_ref():
    c = trimesh.creation.cylinder(radius=MOTOR_R, height=MOTOR_L, sections=48)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    c.apply_translation([MOTOR_DIR*(GBX_X + MOTOR_L/2), 0, MOTOR_Z]); return c
def pocket(c=GBX_CLR):
    a = box(-GBX_X - c, GBX_X + c, -GBX_Y - c, GBX_Y + c, GBX_Z0 - c, GBX_Z1 + c)
    m = trimesh.creation.cylinder(radius=MOTOR_R + c, height=MOTOR_L + 2*c, sections=48)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [0, 1, 0]))
    m.apply_translation([MOTOR_DIR*(GBX_X + MOTOR_L/2), 0, MOTOR_Z])
    stub = box(-GBX_X - 5, GBX_X + 5, -2.1, 2.1, GBX_SHAFT_Z - 2.1, GBX_SHAFT_Z + 2.1)
    return U([a, m, stub])

# ---------------- rear upper link rebuilt for the new pivot ----------------
def link(side):                            # side +1 = L, -1 = R ; rest pose, pivot at (PU_X, PU_Z)
    old = trimesh.load(BD + "/Rear_Upper_Link_L.stl")
    keep = trimesh.boolean.intersection([old, box(-70, -45, -20, 30, -10, 30)], engine="manifold")  # keep X <= -45
    yc = lambda x: 10.85 + 0.1467*(x + 12.5)          # old centre line in Y
    YB = RAIL_IN - 1.4                                  # boss centre Y (boss sits against the rail inner face)
    p0 = np.array([-45.0, yc(-45.0), 5.25]); p1 = np.array([PU_X, YB, PU_Z])
    rod = trimesh.creation.cylinder(radius=1.3, segment=np.array([p0, p1]), sections=20)
    bs = trimesh.creation.cylinder(radius=1.75, height=2.7, sections=28)
    bs.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    bs.apply_translation([PU_X, p1[1], PU_Z])
    m = U([keep, rod, bs])
    if side < 0: m.apply_transform(np.diag([1, -1, 1, 1])); m.invert()
    return m

# ---------------- load / build parts ----------------
shell = trimesh.load("stl/tacoma_trophy_shell.stl")
P = {}
BASE0 = R.hits(R.scene(0))                # pre-change rest-pose contact pairs (original parts)
OLD = {n: trimesh.load(f"{BD}/{n}.stl") for n in ("Roll_Cage", "Chassis_Rail_L", "Tacoma_Trophy_Body_Shell")} if False else {}
NUT_Y0 = RAIL_IN + 0.1                    # recessed trailing-pivot nut: inner face sits 0.1 inside the rail
def shift_y(m, dy): m = m.copy(); m.apply_translation([0, dy, 0]); return m
def warp_cage(m, dy=RAIL_DY, y0=8.0, y1=10.0):   # outboard shift of everything at |y|>=y1, ramped between y0..y1
    m = m.copy(); v = m.vertices.copy(); a = np.abs(v[:, 1])
    v[:, 1] += np.sign(v[:, 1])*dy*np.clip((a - y0)/(y1 - y0), 0, 1); m.vertices = v; return m
AXL = dil(trimesh.boolean.intersection([R.M["Ref_Rear_Axle"], box(-70, -40, -40, 40, -8, 10)], engine="manifold"), 0.3)  # axle end, +0.1 clearance
for n in "LR":
    s = 1 if n == "L" else -1
    r = trimesh.load(f"{BD}/Chassis_Rail_{n}.stl"); r.apply_translation([0, s*RAIL_DY, 0])
    # plug the old upper-link pivot hole (full rail thickness; pocket cut and new hole follow)
    plug = trimesh.creation.cylinder(radius=2.0, height=RAIL_T, sections=32)
    plug.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    plug.apply_translation([-12.5, s*(RAIL_IN + RAIL_T/2), 5.25]); r = U([r, plug])
    # screw hole for the moved upper-link pivot (axis Y, through rail)
    h = trimesh.creation.cylinder(radius=1.5, height=8, sections=24)
    h.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
    h.apply_translation([PU_X, s*14.5, PU_Z])
    # trailing arm pivot hardware: arm and HW +0.5 outboard, nut recessed into the rail, screw shortened
    nut = shift_y(trimesh.load(f"{BD}/HW_Rear_Trailing_Pivot_Nut_{n}.stl"), s*RAIL_DY)
    ny = nut.bounds[0][1] if s > 0 else -nut.bounds[1][1]
    nut.apply_translation([0, s*(NUT_Y0 - ny), 0]); P[f"HW_Rear_Trailing_Pivot_Nut_{n}"] = nut
    scr = shift_y(trimesh.load(f"{BD}/HW_Rear_Trailing_Pivot_Screw_{n}.stl"), s*RAIL_DY)
    ys = sorted([s*NUT_Y0, s*30]); P[f"HW_Rear_Trailing_Pivot_Screw_{n}"] = trimesh.boolean.intersection([scr, box(-30, 30, ys[0], ys[1], -30, 30)], engine="manifold")
    nb = nut.bounds; ya, yb = sorted([s*(RAIL_IN - 0.1), nb[1][1] + 0.2 if s > 0 else nb[0][1] - 0.2])
    pk = box(nb[0][0] - 0.25, nb[1][0] + 0.25, ya, yb, nb[0][2] - 0.25, nb[1][2] + 0.25)
    P[f"Chassis_Rail_{n}"] = D(r, [h, pk])
    P[f"Rear_Trailing_Arm_{n}"] = biggest(D(shift_y(trimesh.load(f"{BD}/Rear_Trailing_Arm_{n}.stl"), s*RAIL_DY), [AXL]))   # notch where the +0.5 shift meets the axle end
    for hw in ("Screw", "Nut"):
        P[f"HW_Rear_Trailing_Axle_{hw}_{n}"] = shift_y(trimesh.load(f"{BD}/HW_Rear_Trailing_Axle_{hw}_{n}.stl"), s*RAIL_DY)
    sl = trimesh.load(f"{BD}/Slider_{n}.stl"); sl.apply_translation([0, s*RAIL_DY, 0]); P[f"Slider_{n}"] = sl
    P[f"Rear_Upper_Link_{n}"] = link(s)
    for hw in ("Screw", "Nut"):
        m = trimesh.load(f"{BD}/HW_Rear_Upper_Link_Pivot_{hw}_{n}.stl"); m.apply_translation([PU_X + 12.5, s*((RAIL_IN - 1.4) - 10.85), PU_Z - 5.25])
        P[f"HW_Rear_Upper_Link_Pivot_{hw}_{n}"] = m
b = trimesh.load(BD + "/Rear_Sway_Bridge.stl"); b.apply_translation([0, 0, BRIDGE_LIFT]); P["Rear_Sway_Bridge"] = warp_cage(b, y0=8.0, y1=10.0)   # widened to meet the wider rails
P["Rear_Sway_Bridge"] = D(U([P["Rear_Sway_Bridge"], box(-63.2, -59.1, -13.5, 13.5, 13.5, 16.2)]), [trimesh.creation.cylinder(radius=TUN_R, segment=np.array([[BAR_X, -17, BAR_Z], [BAR_X, 17, BAR_Z]]), sections=24)])   # centre web down to Z 13.5 + bar tunnel
for n in "LR":
    P[f"HW_Rear_Sway_Bridge_Screw_{n}"] = shift_y(trimesh.load(f"{BD}/HW_Rear_Sway_Bridge_Screw_{n}.stl"), (1 if n == "L" else -1)*RAIL_DY)
CAGE0 = trimesh.load(BD + "/Roll_Cage.stl"); P["Roll_Cage"] = warp_cage(CAGE0)
_ca = trimesh.load(BD + "/Spare_Tire_Carrier.stl")
_slab = trimesh.boolean.intersection([_ca, box(-60, -35, -15, 15, 19, 21.0)], engine="manifold"); _v = _slab.vertices.copy()
_v[:, 2] += SPARE_DZ*np.clip((_v[:, 2] - _ca.bounds[0][2])/(21.0 - _ca.bounds[0][2]), 0, 1); _slab.vertices = _v   # feet stretched down to the cage
_cu = _ca.copy(); _cu.apply_translation([0, 0, SPARE_DZ]); P["Spare_Tire_Carrier"] = U([_cu, _slab])
for _n in ("Ref_Wheel_Spare", "HW_Spare_Wheel_Nut"):
    _m = trimesh.load(f"{BD}/{_n}.stl"); _m.apply_translation([0, 0, SPARE_DZ]); P[_n] = _m
shell0 = shell; shell = warp_cage(shell0, dy=RAIL_DY + 0.1, y0=8.0, y1=10.0)   # side panels +0.5 outboard
P["Tacoma_Trophy_Body_Shell"] = shell
P["Ref_Rear_Driveshaft"] = trimesh.boolean.intersection([trimesh.load(BD + "/Ref_Rear_Driveshaft.stl"), box(-80, -(GBX_X + 4.0), -10, 10, -10, 30)], engine="manifold")  # starts at the shaft stub tip
REF = gbx_ref(); POCK = pocket(); MOT = motor_ref()
for _n in "LR":      # use the arm redesign's exports (tab geometry) when present; verify only
    for _k in (f"Rear_Trailing_Arm_{_n}", f"HW_Rear_Trailing_Axle_Screw_{_n}", f"HW_Rear_Trailing_Axle_Nut_{_n}"):
        if os.path.exists(OUT + _k + ".stl"): P[_k] = trimesh.load(OUT + _k + ".stl")
def cyl(p, q, r, sec=16): return trimesh.creation.cylinder(radius=r, segment=np.array([p, q], float), sections=sec)
def sph(p, r):
    m = trimesh.creation.icosphere(subdivisions=2, radius=r); m.apply_translation(p); return m
CROSS = U([cyl((BAR_X, -BAR_Y, BAR_Z), (BAR_X, BAR_Y, BAR_Z), WIRE_R), sph((BAR_X, -BAR_Y, BAR_Z), WIRE_R), sph((BAR_X, BAR_Y, BAR_Z), WIRE_R)])
LEV = {sg: U([cyl((BAR_X, sg*BAR_Y, BAR_Z), (BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R), sph((BAR_E_X, sg*BAR_Y, BAR_Z), WIRE_R)]) for sg in (1, -1)}
P["Ref_Rear_Sway_Bar"] = U([CROSS, LEV[1], LEV[-1]])
for sg, n in ((1, "L"), (-1, "R")): P[f"Ref_Sway_Drop_Link_{n}"] = cyl((BAR_E_X, sg*BAR_Y, BAR_Z), (TAB_X, sg*BAR_Y, TAB_TOP_Z - LINK_INS), LINK_R)
E0 = np.array([BAR_E_X, BAR_Z]); AX = np.array([BAR_X, BAR_Z]); T0 = np.array([TAB_X, TAB_TOP_Z - LINK_INS]); L0 = float(np.linalg.norm(E0 - T0))
def bar_state(ta):
    a = R.solve(ta)[0]; Tp = R.rot(T0, a, R.PL)
    f = lambda th: np.linalg.norm(R.rot(E0, th, AX) - Tp) - L0
    xs = np.radians(np.linspace(-70, 70, 281)); fv = [f(x) for x in xs]; best = None
    for i in range(len(xs) - 1):
        if fv[i]*fv[i+1] <= 0:
            r_ = brentq(f, xs[i], xs[i+1])
            if best is None or abs(r_) < abs(best): best = r_
    return best, R.rot(E0, best, AX), Tp
def SC(ta):
    sc = R.scene(float(ta)); th, E, Tp = bar_state(ta)
    sc["Ref_Rear_Sway_Bar_cross"] = (CROSS, np.eye(4))
    for sg, n in ((1, "L"), (-1, "R")):
        sc[f"Ref_Sway_Lever_{n}"] = (LEV[sg], R.roty(th, BAR_X, BAR_Z))
        sc[f"Ref_Sway_Drop_Link_{n}"] = (cyl((E[0], sg*BAR_Y, E[1]), (Tp[0], sg*BAR_Y, Tp[1]), LINK_R), np.eye(4))
    return sc

# kinematics with new pivot
R.PU = np.array([PU_X, PU_Z])
for k, v in P.items():
    if k in R.M or k == "Tacoma_Trophy_Body_Shell": R.M[k] = v
R.M["Tacoma_Trophy_Body_Shell"] = shell; R.M["Ref_Gearbox"] = REF; R.M["Ref_Motor_GM1614"] = MOT
GN = ("Ref_Gearbox", "Ref_Motor_GM1614")
isbar = lambda n: n.startswith(("Ref_Sway", "Ref_Rear_Sway"))
def keep(p):   # rest-pose pairs to ignore: attachments, and the intentional bar/link/tab insertions
    if any(g in p for g in GN): return False
    if not any(isbar(n) for n in p): return True
    return all(isbar(n) for n in p) or (any(n.startswith("Ref_Sway_Drop") for n in p) and any(n.startswith("Rear_Trailing_Arm") for n in p))
rest0 = {p for p in R.hits(SC(0)) if keep(p)}

# ---------------- travel relief (bump 0..+7) ----------------
TARGETS = ["Chassis_Rail_L", "Chassis_Rail_R", "Rear_Sway_Bridge", "Roll_Cage", "Spare_Tire_Carrier",
           "Tacoma_Trophy_Body_Shell", "Slider_L", "Slider_R"]
touch = {}
for ta in np.arange(-10, 7.01, 1.0):
    for p in R.hits(SC(float(ta))):
        if p in rest0 or any(n.startswith("HW_") or n in GN for n in p): continue
        for a, c in (p, p[::-1]):
            if a in TARGETS and c not in TARGETS: touch.setdefault(a, set()).add(c)
print("relief contacts:", {k: sorted(v) for k, v in touch.items()})
RL = {}
for t, movs in touch.items():
    sw = []
    for ta in np.arange(-10, 7.01, 0.5):
        sc = SC(float(ta))
        for n in movs:
            if n in sc:
                m, T = sc[n]; c = m.copy(); c.apply_transform(T); sw.append(c)
    RL[t] = sw   # raw (undilated) swept poses; dilated per attempt in finish()
FINAL = {}
def finish(t, clr_try):
    m = P[t]
    cuts = []
    if t in RL: cuts.append(U([dil(c, RELIEF_CLR + 0.03*clr_try) for c in RL[t]]))
    if t in ("Roll_Cage", "Tacoma_Trophy_Body_Shell", "Spare_Tire_Carrier") or True:
        cuts.append(pocket(GBX_CLR + (0.0, 0.013, 0.027)[clr_try]))
    out = biggest(D(m, cuts))
    return out
for t in TARGETS:
    for att in range(3):
        o = finish(t, att); r = rt(o, OUT + t + ".stl"); oe = open_edges(r)
        print(f"{t}: attempt {att} vol {P[t].volume:.0f}->{r.volume:.0f} watertight {r.is_watertight} open_edges {oe} shells {len(r.split(only_watertight=False))}")
        if r.is_watertight and oe == 0 and len(r.split(only_watertight=False)) == 1: FINAL[t] = r; break
    else: FINAL[t] = r
for t in P:
    if t in TARGETS: continue
    if t.startswith(("Rear_Trailing_Arm", "HW_Rear_Trailing_Axle")):   # owned by the arm redesign (14_shock_slot.py): verify only, do not export
        FINAL[t] = P[t]; continue
    r = rt(P[t], OUT + t + ".stl"); FINAL[t] = r
    print(f"{t}: watertight {r.is_watertight} open_edges {open_edges(r)} shells {len(r.split(only_watertight=False))}")
mt = rt(MOT, OUT + "Ref_Motor_GM1614.stl"); print("Ref_Motor watertight", mt.is_watertight, mt.bounds.round(2).tolist()); r = rt(REF, OUT + "Ref_Gearbox.stl"); print("Ref_Gearbox watertight", r.is_watertight, "bounds", r.bounds.round(2).tolist())

# ---------------- verification ----------------
for t, m in FINAL.items():
    if t in R.M or t == "Tacoma_Trophy_Body_Shell": R.M[t] = m
R.M["Ref_Gearbox"] = r; R.M["Ref_Motor_GM1614"] = mt
from trimesh.collision import CollisionManager
allb = dict(R.M); allb.update(FINAL)
SKIPN = ("Front", "IFS", "Steering", "Servo", "Tie_Rod", "Tie", "Ref_Front", "Ref_Wheel_F", "Ref_Shock_F", "HW_Front", "Drag", "HW_Drag", "Ref_Shock_R", "HW_Rear_Shock", "Rear_Trailing_Arm", "HW_Rear_Trailing_Axle")
for lab, ref in (("GEARBOX", r), ("MOTOR", mt)):
    cm = CollisionManager(); cm.add_object("G", ref); bad = []
    for n, m in allb.items():
        if n.startswith(SKIPN) or n in GN: continue
        c2 = CollisionManager(); c2.add_object(n, m)
        if cm.in_collision_other(c2): bad.append(n)
    print(lab, "COLLISIONS (non-front, non-arm/shock bodies):", bad or "none")
rest = R.hits(SC(0))
def real(p): return sorted(x for x in p if x not in rest0 and not any(n.startswith("HW_") for n in x) and not all(n in GN for n in x))
res = {}
for ta in range(-10, 7):
    h = real(R.hits(SC(float(ta))))
    if h: res[ta] = h[:6]
print("REAR SWEEP -10..+6 contacts:", res or "none")
for ta in (-10, 0, 6):
    a, u, T = R.solve(ta); print("ta %+d  link angle %.2f deg  axle dZ %+.2f dX %+.2f pitch %.2f" % (ta, math.degrees(u), T[2, 3], T[0, 3], math.degrees(math.atan2(-T[2, 0], T[0, 0]))))

# ---------------- extra checks: new rest-pose contacts, cage/rail/shell overlap, HW outboard of Y=12 ----------------
now0 = R.hits(SC(0))
print("NEW rest-pose contact pairs vs original:", sorted(p for p in now0 - BASE0 if "Ref_Gearbox" not in p) or "none")
print("gone vs original:", sorted(BASE0 - now0) or "none")
def ivol(a, b):
    try: return round(abs(trimesh.boolean.intersection([a, b], engine="manifold").volume), 1)
    except Exception: return 0.0
rl0 = trimesh.load(f"{BD}/Chassis_Rail_L.stl")
print("cage∩rail_L vol: old %s new %s" % (ivol(CAGE0, rl0), ivol(P["Roll_Cage"], FINAL["Chassis_Rail_L"])))
print("cage∩shell vol: old %s new %s" % (ivol(CAGE0, shell0), ivol(FINAL["Roll_Cage"], FINAL["Tacoma_Trophy_Body_Shell"])))
for n in "LR":
    for k in ("Nut", "Screw"):
        m = FINAL[f"HW_Rear_Trailing_Pivot_{k}_{n}"]; print(f"HW_Rear_Trailing_Pivot_{k}_{n} |Y| min %.2f max %.2f" % (np.abs(m.bounds[:, 1]).min(), np.abs(m.bounds[:, 1]).max()))
print("arm L Y", FINAL["Rear_Trailing_Arm_L"].bounds[:, 1].round(2).tolist(), "rail L Y", FINAL["Chassis_Rail_L"].bounds[:, 1].round(2).tolist())

print("SWAY LINK over travel (ta, lever angle deg, link angle from vertical deg, link length):")
for ta in range(-10, 7, 2):
    th, E, Tp = bar_state(ta); print("  ta %+3d  lever %+6.1f  link %+5.1f  len %.2f  E z %.1f" % (ta, math.degrees(th), math.degrees(math.atan2(E[0] - Tp[0], E[1] - Tp[1])), np.linalg.norm(E - Tp), E[1]))
print("rest link length %.2f (wire centre to tab insertion point); tab top Z %.1f" % (L0, TAB_TOP_Z))
chk = ["Ref_Rear_Sway_Bar", "Ref_Sway_Drop_Link_L", "Ref_Sway_Drop_Link_R", "Rear_Sway_Bridge", "Spare_Tire_Carrier", "Ref_Wheel_Spare"]
for k in chk[:3]: FINAL[k] = rt(P[k], OUT + k + ".stl")
FINAL["Ref_Wheel_Spare"] = rt(P["Ref_Wheel_Spare"], OUT + "Ref_Wheel_Spare.stl"); FINAL["HW_Spare_Wheel_Nut"] = rt(P["HW_Spare_Wheel_Nut"], OUT + "HW_Spare_Wheel_Nut.stl")
for k in ("Ref_Rear_Sway_Bar", "Ref_Sway_Drop_Link_L", "Ref_Sway_Drop_Link_R", "Ref_Wheel_Spare", "Spare_Tire_Carrier", "Rear_Sway_Bridge"):
    print(k, "watertight", FINAL[k].is_watertight, "open_edges", open_edges(FINAL[k]), "shells", len(FINAL[k].split(only_watertight=False)))
allx = dict(R.M); allx.update(FINAL)
for k in ("Rear_Sway_Bridge", "Spare_Tire_Carrier", "Ref_Wheel_Spare", "HW_Spare_Wheel_Nut"): R.M[k] = FINAL[k]
for k in chk[:3]: allx[k] = FINAL[k]
for k in chk:
    cm = CollisionManager(); cm.add_object(k, allx[k]); hit = []
    for n, m in allx.items():
        if n == k or n.startswith(SKIPN[:11]) or n.startswith("Ref_Front") or n.startswith("HW_Front"): continue
        c2 = CollisionManager(); c2.add_object(n, m)
        if cm.in_collision_other(c2): hit.append(n)
    print("OVERLAP", k, "vs", hit or "none")
c1 = CollisionManager(); c1.add_object("a", FINAL["Rear_Sway_Bridge"]); c2 = CollisionManager(); c2.add_object("b", FINAL["Ref_Wheel_Spare"]); print("bridge-spare min distance %.2f" % c1.min_distance_other(c2))
