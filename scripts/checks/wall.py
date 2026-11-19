"""Rough wall-thickness probe (ray inward from surface samples) on print/parts. Over-reports at sharp edges."""
import glob,os,numpy as np,trimesh
for f in sorted(glob.glob("print/parts/*.stl")):
    m=trimesh.load(f); n=os.path.basename(f)[:-4]
    p,fi=trimesh.sample.sample_surface(m,4000,seed=1)
    nr=m.face_normals[fi]
    o=p-nr*0.01
    loc,ri,_=m.ray.intersects_location(o,-nr,multiple_hits=False)
    d=np.linalg.norm(loc-o[ri],axis=1)+0.01
    q=np.percentile(d,1)
    print(f"{n:30s} min {d.min():5.2f}  p1 {q:5.2f}  thin(<0.8) {100*(d<0.8).sum()/len(p):4.1f}%")
