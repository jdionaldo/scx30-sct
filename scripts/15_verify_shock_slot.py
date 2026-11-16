"""Shock eye / rod / body vs the slotted trailing arm over rear travel. BODY_DIR=<dir> python scripts/15_verify_shock_slot.py"""
import math,numpy as np,trimesh,sys
sys.path.insert(0,"scripts"); import rearlib as R
from trimesh.collision import CollisionManager
arm=trimesh.load("stl/v3/Rear_Trailing_Arm_L.stl"); PX,PZ=-44.0,0.5; U=np.array([-26.0,21.5]); YC=20.25; ROD_END=6.5
def cylY(x,z,r,y0,y1):
    c=trimesh.creation.cylinder(radius=r,height=y1-y0,sections=32); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0])); c.apply_translation([x,(y0+y1)/2,z]); return c
bad={}
for ta in range(-10,7):
    a=math.radians(ta); P=R.rot(np.array([PX,PZ]),a,R.PL); v=U-P; u=v/np.linalg.norm(v)
    am=arm.copy(); am.apply_transform(R.roty(a,*R.PL)); cm=CollisionManager(); cm.add_object("arm",am)
    for nm,ts,r,w in (("eye",np.linspace(0,3.2,5),1.6,1.0),("rod",np.linspace(3.2,ROD_END,6),0.75,0.75),("body",np.linspace(ROD_END,ROD_END+6,7),2.1,2.1)):
        m=trimesh.util.concatenate([cylY(*(P+u*t),r,YC-w,YC+w) for t in ts])
        if cm.in_collision_single(m): bad.setdefault(nm,[]).append(ta)
L=[np.linalg.norm(U-R.rot(np.array([PX,PZ]),math.radians(t),R.PL)) for t in range(-10,7)]
print("shock eye-to-eye %.1f..%.1f mm (SCX24 32 mm: 22.0..32.0)"%(min(L),max(L)))
print("contacts with arm (ta deg):",bad if bad else "none -10..+6")
