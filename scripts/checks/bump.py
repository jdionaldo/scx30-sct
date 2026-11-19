"""Rear wheel arc vs shell (bump sweep about the trailing-arm pivot) and front wheel envelope vs shell. Run from repo root."""
import numpy as np,trimesh
from trimesh.collision import CollisionManager
L=lambda p:trimesh.load(p)
shell=L("stl/v4/Tacoma_Trophy_Body_Shell.stl")
pv=L("stl/v4/HW_Rear_Trailing_Pivot_Screw_L.stl").bounds.mean(axis=0)
ax=L("stl/v4/Ref_Rear_Axle.stl").bounds.mean(axis=0)
print("pivot",pv.round(1),"axle ctr",ax.round(1))
for nm in ("Ref_Wheel_RL","Ref_Wheel_RR"):
    w=L(f"stl/v4/{nm}.stl")
    for dz in (0,3,5,7,9,11):
        # rotate about pivot (Y axis) so axle rises dz
        r=np.hypot(ax[0]-pv[0],ax[2]-pv[2]); a0=np.arctan2(ax[2]-pv[2],ax[0]-pv[0])
        # axle is behind pivot (x<pv): raising z -> angle toward... solve a so that z=ax.z+dz
        s=(ax[2]+dz-pv[2])/r; a=np.arcsin(np.clip(s,-1,1)); a=np.pi-a if np.cos(a0)<0 else a
        th=a-a0
        c,sn=np.cos(-th),np.sin(-th)
        T=np.eye(4); T[0,0]=c;T[0,2]=sn;T[2,0]=-sn;T[2,2]=c
        T2=np.eye(4);T2[:3,3]=-pv; T3=np.eye(4);T3[:3,3]=pv
        M=T3@T@T2; ww=w.copy(); ww.apply_transform(M)
        cm=CollisionManager(); cm.add_object("s",shell)
        d=cm.min_distance_single(ww)
        print(nm,f"bump {dz:2d} mm: wheel->shell min distance {d:6.2f} mm  (axle rise {ww.bounds.mean(axis=0)[2]-w.bounds.mean(axis=0)[2]:.1f}, x shift {ww.bounds.mean(axis=0)[0]-w.bounds.mean(axis=0)[0]:.1f})")
fe=L("stl/v3/Front_Wheel_Envelope.stl"); cm=CollisionManager(); cm.add_object("s",shell)
print("front sweep envelope -> shell:",round(cm.min_distance_single(fe),2),"mm; intersects:",cm.in_collision_single(fe))
