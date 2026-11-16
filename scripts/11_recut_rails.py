import os,sys,numpy as np,trimesh
sys.path.insert(0,"scripts"); import rearlib as R
CLR=0.52
def dil(m):
    ms=[m]
    for d in ([CLR,0,0],[-CLR,0,0],[0,CLR,0],[0,-CLR,0],[0,0,CLR],[0,0,-CLR]):
        c=m.copy(); c.apply_translation(d); ms.append(c)
    return trimesh.boolean.union(ms,engine="manifold")
sw=[]
for ta in np.arange(0,7.01,0.5):
    sc=R.scene(float(ta)); m,T=sc["Ref_Rear_Axle"]; c=m.copy(); c.apply_transform(T); sw.append(dil(c))
swept=trimesh.boolean.union(sw,engine="manifold")
for n in "LR":
    base=trimesh.load(os.environ["BODY_DIR"]+f"/Chassis_Rail_{n}.stl")
    out=trimesh.boolean.difference([base,swept],engine="manifold")
    parts=[p for p in out.split(only_watertight=False) if abs(p.volume)>20]; out=parts[0]
    f=f"stl/v2/Chassis_Rail_{n}.stl"; out.export(f); r=trimesh.load(f)
    e=r.edges_sorted; u,c=np.unique(e,axis=0,return_counts=True)
    print(n,"watertight after STL roundtrip",r.is_watertight,"open edges",(c==1).sum(),round(r.volume,1))
