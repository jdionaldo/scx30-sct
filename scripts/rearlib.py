import sys,os,math,numpy as np,trimesh
from scipy.optimize import brentq
from trimesh.collision import CollisionManager
B=os.environ["BODY_DIR"]
names=[f[:-4] for f in os.listdir(B) if f.endswith(".stl")]
M={n:trimesh.load(os.path.join(B,n+".stl")) for n in names}
def roty(a,x0,z0):
    c,s=math.cos(a),math.sin(a); R=np.eye(4); R[0,0]=c;R[0,2]=s;R[2,0]=-s;R[2,2]=c
    R[0,3]=x0-(c*x0+s*z0); R[2,3]=z0-(-s*x0+c*z0); return R
PL=np.array([-7.75,-0.5]);PU=np.array([-15.5,4.5]);A=np.array([-59.0,-0.5]);Bp=np.array([-59.0,5.25])
def rot(p,a,o): d=p-o; return o+np.array([d[0]*math.cos(a)+d[1]*math.sin(a),-d[0]*math.sin(a)+d[1]*math.cos(a)])
def solve(ta):
    a=math.radians(ta); L=rot(A,a,PL); d0=np.linalg.norm(Bp-A)
    f=lambda u:np.linalg.norm(rot(Bp,u,PU)-L)-d0
    xs=np.radians(np.linspace(-40,40,321)); fv=[f(x) for x in xs]; best=None
    for i in range(len(xs)-1):
        if fv[i]*fv[i+1]<=0:
            r=brentq(f,xs[i],xs[i+1])
            if best is None or abs(r-a)<abs(best-a): best=r
    U=rot(Bp,best,PU); v0=Bp-A; v1=U-L
    rho=math.atan2(-(v1[1]),v1[0])-math.atan2(-(v0[1]),v0[0])  # rotation about Y convention
    # build axle transform: rotate by rho about A then translate to L
    T=roty(-rho if False else 0,0,0)
    ang=math.atan2(v1[0]*0+v1[1],v1[0])-math.atan2(v0[1],v0[0])
    # rotation about +Y by angle g maps (x,z)->(x c+z s,-x s+z c); angle of vector in (x,z) changes by -g
    g=-ang
    Tg=roty(g,A[0],A[1]); Tt=np.eye(4); Tt[0,3]=L[0]-A[0]; Tt[2,3]=L[1]-A[1]
    return a,best,Tt@Tg
GROUP_AX=lambda n: n in("Ref_Rear_Axle","Ref_Wheel_RL","Ref_Wheel_RR") or n.startswith("HW_Rear_Trailing_Axle") or n.startswith("HW_Rear_Upper_Link_Axle")
SKIP=lambda n: n.startswith("Ref_Shock_R") or n in("Ref_Rear_Driveshaft","Ref_Rear_Sway_Bar") or n.startswith("HW_") 
def scene(ta,ids_only=False):
    a,u,Tax=solve(ta); sc={}
    for n,m in M.items():
        if n.startswith("HW_Rear") and not GROUP_AX(n): 
            continue
        if n.startswith("HW_") and not GROUP_AX(n): continue
        if n.startswith("Ref_Shock_R") or n=="Ref_Rear_Driveshaft" or n=="Ref_Rear_Sway_Bar": continue
        T=np.eye(4)
        if n.startswith("Rear_Trailing_Arm"): T=roty(a,PL[0],PL[1])
        elif n.startswith("Rear_Upper_Link"): T=roty(u,PU[0],PU[1])
        elif GROUP_AX(n): T=Tax
        sc[n]=(m,T)
    return sc
def hits(sc):
    cm=CollisionManager()
    for n,(m,T) in sc.items(): cm.add_object(n,m,transform=T)
    h,nm=cm.in_collision_internal(return_names=True)
    return {tuple(sorted(p)) for p in nm}
