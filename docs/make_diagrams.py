#!/usr/bin/env python3
"""Render the diagrams used by ASSEMBLY.md and print/PLATES.md (orthographic, painter's algorithm, no GL needed).

    python docs/make_diagrams.py            # all
Source meshes: $SCX_SRC (os.pathsep list of dirs) or stl/v4, stl/v3, print/parts (first hit per file name).
Assembly frame: X forward, Y left, Z up, mm.
"""
import os, sys, json, glob, math
import numpy as np, trimesh
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
SRC = os.environ.get('SCX_SRC', '').split(os.pathsep) if os.environ.get('SCX_SRC') else [REPO + '/stl/v4', REPO + '/stl/v3', REPO + '/print/parts']
C = dict(front='#2f6db5', steer='#d9822b', tub='#8a8f98', cage='#c0392b', rear='#2e8b57', hw='#222222', ref='#b7bcc4', shell='#6fa8dc', motor='#7d5fa6', servo='#e6c229')
_cache = {}
def L(n):
    if n in _cache: return _cache[n]
    for d in SRC:
        p = os.path.join(d, n + '.stl')
        if os.path.exists(p):
            m = trimesh.load(p); _cache[n] = m; return m
    return None

def project(view):
    # returns (right, up, toward-viewer) unit vectors in world coordinates
    if view == 'top':   return np.array([0, -1, 0]), np.array([1, 0, 0]), np.array([0, 0, 1])   # X fwd = up the page, +Y (left of the truck) = left on the page
    if view == 'side':  return np.array([1, 0, 0]), np.array([0, 0, 1]), np.array([0, -1, 0])   # viewer on the right side (-Y): front of the truck to the right
    if view == 'front': return np.array([0, 1, 0]), np.array([0, 0, 1]), np.array([1, 0, 0])
    if view == 'iso':
        v = np.array([0.55, -0.75, 0.45]); v /= np.linalg.norm(v)
        r = np.cross([0, 0, 1], v); r /= np.linalg.norm(r); u = np.cross(v, r)
        return r, u, v
def render(ax, items, view, light=(0.3, -0.4, 0.85), xlim=None, ylim=None):
    r, u, v = project(view)
    polys, cols, depth = [], [], []
    li = np.array(light) / np.linalg.norm(light)
    for m, col, alpha in items:
        if m is None: continue
        tri = m.triangles; n = m.face_normals
        d = tri.mean(1) @ v
        sh = 0.55 + 0.45 * np.clip(n @ li, -1, 1) * 0.9
        base = np.array(matplotlib.colors.to_rgb(col))
        front = (n @ v) > -0.2
        for t, dd, s, f in zip(tri, d, sh, front):
            if not f: continue
            polys.append(np.c_[t @ r, t @ u]); depth.append(dd); cols.append(np.r_[np.clip(base * s, 0, 1), alpha])
    o = np.argsort(depth)
    pc = PolyCollection([polys[i] for i in o], facecolors=[cols[i] for i in o], edgecolors='none', linewidths=0)
    ax.add_collection(pc); ax.set_aspect('equal'); ax.axis('off')
    ax.autoscale_view()
    allp = np.vstack(polys) if polys else np.zeros((2, 2))
    lo, hi = allp.min(0), allp.max(0); pad = 0.04 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad) if xlim is None else ax.set_xlim(*xlim)
    ax.set_ylim(lo[1] - pad, hi[1] + pad) if ylim is None else ax.set_ylim(*ylim)
    return r, u

def pt(ax, view, p, text, dx=0, dy=0, color='k', fs=7, arrow=True, ha='left'):
    r, u, v = project(view)
    x, y = np.dot(p, r), np.dot(p, u)
    ax.annotate(text, (x, y), (x + dx, y + dy), fontsize=fs, color=color, ha=ha, va='center',
                arrowprops=dict(arrowstyle='-', color=color, lw=0.6) if arrow else None,
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=color, lw=0.4, alpha=0.9))

FRONT = ['IFS_Bulkhead_Convertible', 'IFS_Lower_Arm_L', 'IFS_Lower_Arm_R', 'IFS_Upper_Arm_L', 'IFS_Upper_Arm_R', 'Front_Carrier_L', 'Front_Carrier_R',
         'Front_Spindle_L', 'Front_Spindle_R', 'Front_Pin_Retainer_Plate_L', 'Front_Pin_Retainer_Plate_R', 'Front_Bump_Stop_L', 'Front_Bump_Stop_R', 'Front_Bumper', 'Front_Y_Brace']
STEER = ['Steering_Rack', 'Steering_BellCrank', 'Steering_Drag_Link', 'Tie_Rod_L', 'Tie_Rod_R', 'Servo_Horn']
REAR = ['Rear_Trailing_Arm_L', 'Rear_Trailing_Arm_R', 'Rear_Upper_Link_L', 'Rear_Upper_Link_R', 'Rear_Axle_Top_Bracket']
def many(names, col, alpha=1.0): return [(L(n), col, alpha) for n in names if L(n) is not None]
def hw(prefixes):
    out = []
    for d in SRC:
        for f in glob.glob(d + '/HW_*.stl'):
            n = os.path.basename(f)[:-4]
            if any(n.startswith(p) for p in prefixes) and n not in [o[3] for o in out]:
                out.append((trimesh.load(f), C['hw'], 1.0, n))
    return [(m, c, a) for m, c, a, _ in out]

def fig_overview():
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    items = (many(['Chassis_Tub'], C['tub']) + many(['Cage'], C['cage']) + many(FRONT, C['front']) + many(STEER, C['steer']) + many(REAR, C['rear']) +
             [(L(n), C['ref'], 0.9) for n in ['Ref_Gearbox', 'Ref_Motor_GM1614', 'Ref_Rear_Axle', 'Ref_Wheel_RL', 'Ref_Wheel_RR', 'Ref_Wheel_FL', 'Ref_Wheel_FR', 'Servo_Ref', 'Ref_ESC_G40'] if L(n) is not None])
    render(axs[0][0], items, 'iso'); axs[0][0].set_title('Overview (iso, shell removed; grey = purchased / stock bodies)')
    render(axs[0][1], items, 'top'); axs[0][1].set_title('Top view (X forward = up the page, +Y left = left on the page)')
    render(axs[1][0], items, 'side'); axs[1][0].set_title('Side view (front of truck to the right)')
    sh = L('Tacoma_Trophy_Body_Shell')
    render(axs[1][1], items + ([(sh, C['shell'], 0.25)] if sh is not None else []), 'side'); axs[1][1].set_title('Side view with the body shell (ghost)')
    from matplotlib.patches import Patch
    axs[0][0].legend(handles=[Patch(color=C[k], label=l) for k, l in [('front', 'front IFS (printed)'), ('steer', 'steering (printed)'), ('tub', 'tub'), ('cage', 'cage'), ('rear', 'rear arms / links (printed)'), ('motor', 'ESC/RX shelves'), ('ref', 'purchased / stock')]], loc='lower left', fontsize=7)
    fig.tight_layout(); fig.savefig(HERE + '/01_overview.png', dpi=110); plt.close(fig)

def fig_front():
    fig, axs = plt.subplots(1, 2, figsize=(16, 7.5))
    pr = many(FRONT, C['front']) + many(STEER, C['steer']) + [(L(n), C['ref'], 0.85) for n in ['Servo_Ref', 'Ref_Front_Diff_Ring_Gear', 'Ref_Front_Input_Worm_Gear', 'Ref_Front_Center_Driveshaft', 'CVD_Inner_L', 'CVD_Inner_R', 'CVD_Outer_L', 'CVD_Outer_R', 'Front_Bearing_L', 'Front_Bearing_R', 'Ref_Shock_FL', 'Ref_Shock_FR'] if L(n) is not None] + \
         hw(['HW_Front_', 'HW_Crank', 'HW_TieRod', 'HW_Link', 'HW_Rack', 'HW_Horn'])
    render(axs[0], pr, 'top'); axs[0].set_title('Front end, top view (X forward is up the page)')
    ax = axs[0]
    marks = [('1 Bulkhead', (36, 0, 12), 'top', 34, 0), ('2 Upper arm', (48, 22, 7), 'top', 2, 0), ('3 Lower arm', (48, 20, 0), 'top', 2, 0), ('4 Carrier', (48, 33, 3), 'top', 2, 0), ('5 Spindle', (46, 38, 6), 'top', 2, 0),
             ('6 Bell crank', (32.4, -8.4, 1), 'top', 2, 0), ('7 Servo', (30, 6, 6), 'top', 2, 0)]
    for lab, p, dx, dy in [('1 Front bulkhead (fused into Chassis_Tub)', (60, 0, 8), 0, 16), ('2 IFS_Upper_Arm L/R', (47, 17, 7), -22, 20), ('3 IFS_Lower_Arm L/R', (46, 24, 0), -30, -18), ('4 Front_Carrier', (48, 33, 3), -14, 26), ('5 Front_Spindle', (44, 38, 4), -12, -34),
                           ('6 Steering_BellCrank', (32.4, -8.4, 1), 40, -8), ('7 Servo (ref) + horn', (24, 7, 6), -45, -8), ('8 Tie_Rod L/R', (38, 22, 3.5), 12, -22), ('9 Front_Pin_Retainer_Plate', (57.4, 11, 3), 28, 14), ('10 Front_Bump_Stop (on shock)', (58, 19, 3), -30, 22)]:
        pt(axs[0], 'top', np.array(p), lab, dx, dy, fs=7)
    render(axs[1], pr, 'front'); axs[1].set_title('Front end, seen from the front (truck-left +Y is on the right of the page, Z up)')
    fig.tight_layout(); fig.savefig(HERE + '/02_front_end.png', dpi=110); plt.close(fig)

def fig_steer():
    fig, ax = plt.subplots(figsize=(13, 6))
    items = many(STEER, C['steer']) + [(L(n), C['ref'], 0.6) for n in ['Servo_Ref', 'IFS_Bulkhead_Convertible'] if L(n) is not None] + hw(['HW_Crank', 'HW_TieRod', 'HW_Link', 'HW_Rack', 'HW_Horn']) + many(['Front_Spindle_L', 'Front_Spindle_R'], C['front'], 0.6)
    render(ax, items, 'top'); ax.set_title('Steering linkage (top view, X forward = up): servo horn -> drag link -> bell crank -> rack -> tie rods -> spindle steering arms')
    for name, p, dx, dy in [('A  Servo_Horn on N30 spline  (X 45 side servo)', (36.2, 6.1, 1.5), -30, 18), ('B  Steering_Drag_Link (pin-to-pin 16.0)', (33, -2, 1), -34, -8), ('C  Steering_BellCrank pivot X32.4 Y-8.4', (32.4, -8.4, 1), -30, -14),
                            ('D  Steering_Rack (+-2.8 mm = full lock)', (37.8, 0, 2), 22, 6), ('E  Tie_Rod_L / R', (39, 20, 3.5), 15, 15), ('F  HW_TieRod_Outer_Screw', (41.8, 34, 3.5), 0, 14)]:
        pt(ax, 'top', np.array(p), name, dx, dy, fs=8)
    fig.tight_layout(); fig.savefig(HERE + '/03_steering_linkage.png', dpi=110); plt.close(fig)

def fig_rear():
    fig, axs = plt.subplots(1, 2, figsize=(16, 6.5))
    it = many(REAR, C['rear']) + many(['Chassis_Tub'], C['tub'], 0.6) + many(['Cage'], C['cage'], 0.85) + [(L(n), C['ref'], 0.9) for n in ['Ref_Rear_Axle', 'Ref_Shock_RL', 'Ref_Shock_RR', 'Ref_Rear_Driveshaft', 'Ref_Rear_Sway_Bar', 'Ref_Sway_Drop_Link_L', 'Ref_Sway_Drop_Link_R', 'Ref_Wheel_RL', 'Ref_Wheel_RR', 'Ref_Shock_Eye_L', 'Ref_Shock_Eye_R'] if L(n) is not None] + \
         hw(['HW_Rear_', 'HW_Cage_Screw_L3', 'HW_Cage_Screw_R3'])
    render(axs[0], it, 'side', xlim=(-95, -22)); axs[0].set_title('Rear 4-link, side view (front of the truck to the right)')
    render(axs[1], it, 'top'); axs[1].set_title('Rear, top view')
    for lab, p, dx, dy in [('Trailing arm (lower link)', (-50, 21, 0), 0, -14), ('Upper link', (-50, 8, 10), 5, 10), ('Axle top bracket + pin', (-66.5, 0, 7.6), -10, 12), ('Shock', (-45, 20, 12), 0, 14), ('Rear axle (stock)', (-64, 0, 0), -12, -16)]:
        pt(axs[0], 'side', np.array(p), lab, dx, dy, fs=8)
    fig.tight_layout(); fig.savefig(HERE + '/04_rear_suspension.png', dpi=110); plt.close(fig)

def fig_fasteners():
    fig, ax = plt.subplots(figsize=(16, 7))
    it = many(['Chassis_Tub'], C['tub'], 0.55) + many(['Cage'], C['cage'], 0.6)
    render(ax, it, 'top')
    groups = [('HW_Cage_Screw', '#d62728', 'cage screws'),
              ('HW_Rear_Trailing_Pivot', '#8c564b', 'trailing arm pivots (x2)'), ('HW_Rear_Upper_Link_Pivot', '#e377c2', 'upper link pivots (x2)'), ('HW_Rear_Axle_Top_Pin', '#17becf', 'axle top pin'), ('HW_Rear_Hinge_Screw', '#ff7f0e', 'shell hinge screws (x2)'), ('HW_Sway_Link_Pin', '#9467bd', 'sway-link pins (x2)')]
    seen = set()
    for pre, col, lab in groups:
        first = True
        for d in SRC:
            for f in sorted(glob.glob(d + '/' + pre + '*.stl')):
                n = os.path.basename(f)
                if n in seen: continue
                seen.add(n); m = trimesh.load(f); c = m.bounds.mean(0)
                ax.plot(-c[1], c[0], 'o', ms=8, mfc='none', mec=col, mew=2, label=lab if first else None); first = False
    ax.legend(loc='center right', fontsize=8); ax.set_title('Fastener map, top view (X forward = up the page, +Y left = left on the page)')
    fig.tight_layout(); fig.savefig(HERE + '/05_fastener_map.png', dpi=110); plt.close(fig)

def fig_orient():
    od = REPO + '/print/oriented'; rep = json.load(open(REPO + '/print/plates_report.json'))
    names = []; plate_of = {}
    for p in rep['plates']:
        for n in p['parts']: names.append(n); plate_of[n] = p['plate']
    cols = 6; rows = math.ceil(len(names) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 3.3, rows * 3.1))
    axs = np.array(axs).reshape(-1)
    for ax in axs: ax.axis('off')
    for ax, n in zip(axs, names):
        m = trimesh.load(od + '/' + n + '.stl')
        r, u, v = project('iso')
        render(ax, [(m, C['front'] if plate_of[n] == 1 else C['rear'] if plate_of[n] in (2, 3) else C['tub'] if plate_of[n] == 4 else C['cage'] if plate_of[n] == 5 else C['shell'], 1.0)], 'iso')
        ax.set_title(f"{n}\nplate {plate_of[n]}, H {m.extents[2]:.1f} mm", fontsize=7)
        # build plate line
        ax.annotate('', xy=(0.5, 0.06), xytext=(0.5, 0.2), xycoords='axes fraction', arrowprops=dict(arrowstyle='<-', color='k', lw=0.8))
    fig.suptitle('Print orientation (build direction = up the page; bed is the lowest face)', fontsize=12)
    fig.tight_layout(); fig.savefig(HERE + '/06_print_orientation.png', dpi=100); plt.close(fig)

def fig_plates():
    rep = json.load(open(REPO + '/print/plates_report.json')); bw, bh = rep['bed']
    import zipfile, re
    for p in rep['plates']:
        z = zipfile.ZipFile(REPO + f"/print/plate_{p['plate']}.3mf"); x = z.read('3D/3dmodel.model').decode()
        fig, ax = plt.subplots(figsize=(7, 7 * bh / bw))
        ax.add_patch(plt.Rectangle((0, 0), bw, bh, fc='#f3f3f3', ec='k'))
        for m in re.finditer(r'<object id="(\d+)" name="([^"]+)".*?</object>', x, re.S):
            v = np.array(re.findall(r'<vertex x="([^"]+)" y="([^"]+)" z="([^"]+)"', m.group(0)), float)
            f = np.array(re.findall(r'<triangle v1="(\d+)" v2="(\d+)" v3="(\d+)"', m.group(0)), int)
            mesh = trimesh.Trimesh(v, f, process=False)
            tri = mesh.triangles[:, :, :2]
            ax.add_collection(PolyCollection(tri, facecolors='#4a78b8', edgecolors='none'))
            c = (v.min(0) + v.max(0)) / 2
            ax.text(c[0], v[:, 1].max() + 1.2, m.group(2).replace('_', ' '), fontsize=5.5, ha='center', va='bottom', color='#222')
        ax.set_xlim(-2, bw + 2); ax.set_ylim(-2, bh + 2); ax.set_aspect('equal'); ax.set_title(f"plate_{p['plate']}.3mf  ({bw:.0f} x {bh:.0f} mm bed)", fontsize=9)
        ax.tick_params(labelsize=6)
        fig.tight_layout(); fig.savefig(HERE + f"/plate_{p['plate']}.png", dpi=90); plt.close(fig)

if __name__ == '__main__':
    which = sys.argv[1:] or ['overview', 'front', 'steer', 'rear', 'fasteners', 'orient', 'plates']
    for w in which:
        {'overview': fig_overview, 'front': fig_front, 'steer': fig_steer, 'rear': fig_rear, 'fasteners': fig_fasteners, 'orient': fig_orient, 'plates': fig_plates}[w](); print('done', w)
