"""Full-assembly battery: source/leftover check, mesh quality of print/parts, pairwise collision (FCL + manifold volume), hardware overlaps. Run from repo root: python scripts/checks/battery.py"""
import os,sys,numpy as np,trimesh,manifold3d as m3
from trimesh.collision import CollisionManager
B=os.environ.get("BODY_DIR")  # optional: extra low-precedence folder of per-body STLs; leave unset
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); from make_viewer_3mf import DROP   # the ONE active-set list (scripts/make_viewer_3mf.py)
src={}
for d in ("stl/v3","stl/v4"):   # BODY_DIR is no longer a source: it only holds superseded bodies
    for f in os.listdir(d):
        if f.endswith(".stl"): src[f[:-4]]=(d,os.path.join(d,f))
names=sorted(n for n in src if n not in DROP and not n.startswith("Ref_Wheel_Spare"))
print("== A. SOURCE / LEFTOVERS")
fromold=[n for n in names if src[n][0] in (B,"stl/v2")]
print("bodies loaded from stale old folders (b/ or v2):",fromold or "none")
v3only=[n for n in names if src[n][0]=="stl/v3"]
print("from v3:",len(v3only),"from v4:",sum(src[n][0]=="stl/v4" for n in names))
pp=sorted(f[:-4] for f in os.listdir("print/parts") if f.endswith(".stl"))
print("print/parts not in current assembly:",[p for p in pp if p not in names] or "none")
printed_expected=[n for n in names if not n.startswith(("HW_","Ref_","CVD_","Front_Bearing","Servo_Ref"))]
print("printable bodies without print/parts file:",[n for n in printed_expected if n not in pp] or "none")
meshes={n:trimesh.load(src[n][1],process=False) for n in names}
print("\n== B. MESH QUALITY (printed parts)")
bad=0
for n in pp:
    if n not in meshes: continue
    m=meshes[n]; m=trimesh.load(f"print/parts/{n}.stl")
    parts=m.split(only_watertight=False)
    vols=sorted([abs(p.volume) for p in parts],reverse=True)
    ar=m.area_faces; deg=int((ar<1e-4).sum())
    gen=1-m.euler_number/2 if m.is_watertight else -1
    flags=[]
    if not m.is_watertight: flags.append("NOT WATERTIGHT")
    if not m.is_winding_consistent: flags.append("winding")
    if m.volume<=0: flags.append("neg vol")
    if len(parts)>1 and vols[-1]<1.0: flags.append(f"{len(parts)} shells, tiny floater {vols[-1]:.2f}")
    if deg: flags.append(f"{deg} degenerate faces")
    if flags: bad+=1
    print(f"{n:32s} shells {len(parts)} vol {m.volume:8.1f} genus~{gen:.0f} {'; '.join(flags) or 'ok'}")
print("parts with flags:",bad)
print("\n== C. COLLISIONS (rest pose, volume > 0.5 mm3, non-HW)")
M=lambda t:m3.Manifold(m3.Mesh(np.asarray(t.vertices,dtype='float32'),np.asarray(t.faces,dtype='uint32')))
cm=CollisionManager()
for n in names: cm.add_object(n,meshes[n])
h,pr=cm.in_collision_internal(return_names=True)
pairs=sorted({tuple(sorted(x)) for x in pr})
res=[]
for a,b in pairs:
    try: v=m3.Manifold.batch_boolean([M(meshes[a]),M(meshes[b])],m3.OpType.Intersect).volume()
    except Exception as e: v=-1
    res.append((v,a,b))
for v,a,b in sorted(res,reverse=True):
    hw=a.startswith("HW_") or b.startswith("HW_")
    if v>0.5 and not hw: print(f"{v:9.1f}  {a} | {b}")
print("HW pairs:",sum(1 for v,a,b in res if a.startswith('HW_') or b.startswith('HW_')),"; total pairs",len(res))
print("\n== D. HARDWARE FIT (screw/nut: overlap with each touching part, mm3)")
for v,a,b in sorted(res,key=lambda r:r[1]):
    if (a.startswith("HW_") or b.startswith("HW_")) and v>0.05: print(f"{v:7.2f} {a} | {b}")
print("\n== E. boolean errors:",[ (a,b) for v,a,b in res if v<0])
print("non-HW pairs by volume (any):")
for v,a,b in sorted(res,reverse=True):
    if not(a.startswith("HW_") or b.startswith("HW_")): print(f"{v:9.2f} {a} | {b}")
