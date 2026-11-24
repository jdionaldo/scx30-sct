"""arm_d5 = arm_d4 with a neck relief on the rear shock clevis inner cheek -> scripts/data/arm_d5/.
usage (repo root): python scripts/arm_d5_ear_relief.py   then rebuild with ARM_DIR=arm_d5 (18_tub.py) or set its default.
Why: the lower shock end's neck (R1.4, leaning inboard toward the upper eye at |Y| 18.2) ran into the outboard face of the
arm's thickened inner cheek (|Y| 20.65) over the whole travel, worst at full droop (ta -16: ~0.017 mm3 at X -51.4..-50.2, Z -2.6..-1.8).
Fix: (1) cut the neck keep-out (R1.4 + CLR) swept relative to the arm over ta -17..+9 out of the inner cheek only (|Y| < eye face 20.8),
(2) make the inner cheek taller over that footprint (cap to Z CAP_Z1, full 1.6 mm thick, flush with the inner face) so the ligament above
the bore gains more than the relief takes. The inner face is the bed face in print/make_plates.py, so nothing is added proud of it.
Frame: X fwd, Y left, Z up, mm; same pivots/shock points as 18_tub.py (REAR_SHIFT 9, PL (-35, 2.4), shock lower pin (-53, 0.5),
upper pin (-40.2, 28.8)). arm_d4/arm_d5 meshes are stored before the REAR_DY track shift, which 18_tub.py applies."""
import os, math, numpy as np, trimesh, manifold3d as mf
SH = 9.0; REAR_DY = 1.05; SHK_DY = 0.85
PL = np.array([-26.0 - SH, 2.4]); SHK_LO = np.array([-44.0 - SH, 0.5]); SHK_UP = np.array([-40.2, 28.8])
SHK_L0 = float(np.linalg.norm(SHK_UP - SHK_LO)); SHK_U = (SHK_UP - SHK_LO)/SHK_L0
SHK_YL, SHK_YU, SHK_EYE_T = 20.4 + REAR_DY + SHK_DY, 18.2, 3.0
NECK_R = 1.4; CLR = float(os.environ.get("NECK_CLR", 0.25))      # target >= 0.2 mm neck-to-cheek over the full travel
TA0, TA1, DTA = -17.0, 9.0, 0.25                                  # sweep 1 deg past the droop (-16) and bump (+8) limits
CHEEK_IN, CHEEK_OUT = 19.05, 20.65                                # inner cheek |Y| (after REAR_DY)
CAP_X0, CAP_X1, CAP_Z1 = -53.6, -50.3, 3.2                        # cheek top raised from ~2.5 to 3.2 over the relief footprint
SRC, DST = "scripts/data/arm_d4/", "scripts/data/arm_d5/"
def roty(a, x0, z0):
    c, s = math.cos(a), math.sin(a); R = np.eye(4); R[0, 0] = c; R[0, 2] = s; R[2, 0] = -s; R[2, 2] = c
    R[0, 3] = x0 - (c*x0 + s*z0); R[2, 3] = z0 - (-s*x0 + c*z0); return R
def rot(p, a, o): d = p - o; return o + np.array([d[0]*math.cos(a) + d[1]*math.sin(a), -d[0]*math.sin(a) + d[1]*math.cos(a)])
def shock_T(a):      # same as 18_tub.py: lower eye rides on the arm, shock points at the upper pin
    lo_t = rot(SHK_LO, a, PL); d0 = SHK_UP - SHK_LO; dt = SHK_UP - lo_t
    dl = math.atan2(d0[1], d0[0]) - math.atan2(dt[1], dt[0]); M = roty(dl, SHK_LO[0], SHK_LO[1]); M[0, 3] += lo_t[0] - SHK_LO[0]; M[2, 3] += lo_t[1] - SHK_LO[1]; return M
def spt(s, sg): return np.array([SHK_LO[0] + SHK_U[0]*s, sg*(SHK_YL + (SHK_YU - SHK_YL)*s/SHK_L0), SHK_LO[1] + SHK_U[1]*s])
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])
def bo(op, ms): return getattr(trimesh.boolean, op)(ms, engine="manifold")
def f32safe(m):      # STL is float32: weld in float32 through manifold and confirm the binary round trip is one watertight body
    for _ in range(4):
        g = mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32))).simplify(0.0005).to_mesh()
        m = trimesh.Trimesh(np.asarray(g.vert_properties[:, :3], np.float64), np.asarray(g.tri_verts))
        rt = trimesh.load(trimesh.util.wrap_as_stream(m.export(file_type="stl")), file_type="stl")
        if rt.is_watertight and rt.is_volume and len(rt.split(only_watertight=False)) == 1: return rt
    raise SystemExit("float32 STL round trip failed")
os.makedirs(DST, exist_ok=True)
for sg, n in ((1, "L"), (-1, "R")):
    arm = trimesh.load(SRC + f"Rear_Trailing_Arm_{n}.stl"); assert arm.is_watertight; arm.apply_translation([0, sg*REAR_DY, 0])
    neck = trimesh.creation.cylinder(radius=NECK_R + CLR, segment=np.array([spt(0.6, sg), spt(7.6, sg)]), sections=48)
    sweep = bo("union", [neck.copy().apply_transform(np.linalg.inv(roty(math.radians(t), *PL)) @ shock_T(math.radians(t))) for t in np.arange(TA0, TA1 + 1e-6, DTA)])
    yin = SHK_YL - SHK_EYE_T/2      # eye inner face: the eye keeps its 0.15 mm bushing gap, only the inner cheek is touched
    cut = bo("intersection", [sweep, box(-60, -44, *((10, yin) if sg > 0 else (-yin, -10)), -8, 10)])
    cap = bo("difference", [box(CAP_X0, CAP_X1, *((CHEEK_IN, CHEEK_OUT) if sg > 0 else (-CHEEK_OUT, -CHEEK_IN)), 2.0, CAP_Z1), cut])
    new = f32safe(bo("union", [bo("difference", [arm, cut]), cap]))
    new.apply_translation([0, -sg*REAR_DY, 0]); new.export(DST + f"Rear_Trailing_Arm_{n}.stl")
    print(f"Rear_Trailing_Arm_{n}: {arm.volume:.2f} -> {new.volume:.2f} mm3, watertight {new.is_watertight}")
