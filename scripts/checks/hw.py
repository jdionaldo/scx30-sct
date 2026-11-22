"""Fastener audit: HW_ bodies with <2 neighbours within 0.35 mm (possible floating hardware). Run from repo root."""
import os,trimesh,numpy as np
from trimesh.collision import CollisionManager
src={}
for d in ("stl/v3","stl/v4"):
    for f in os.listdir(d):
        if f.endswith(".stl"): src[f[:-4]]=os.path.join(d,f)
M={n:trimesh.load(p) for n,p in src.items() if n not in("Front_Wheel_Envelope","IFS_Bulkhead_Convertible")}
hw=[n for n in M if n.startswith("HW_")]
oth=[n for n in M if not n.startswith("HW_") and n not in("Tacoma_Trophy_Body_Shell",)]
cm=CollisionManager()
for n in oth: cm.add_object(n,M[n])
bad=[]
for h in sorted(hw):
    dist,names=cm.min_distance_single(M[h],return_name=True) if False else (None,None)
    near=[]
    c=CollisionManager(); c.add_object(h,M[h])
    for n in oth:
        c2=CollisionManager(); c2.add_object(n,M[n])
        d=c.min_distance_other(c2)
        if d<0.35: near.append((n,round(d,2)))
    near=sorted(near,key=lambda x:x[1])
    if len(near)<2: bad.append((h,near))
    elif len(near)<2: pass
print("hardware bodies:",len(hw)); print("with <2 neighbours within 0.35 mm (possibly floating):")
for h,n in bad: print(" ",h,n)
