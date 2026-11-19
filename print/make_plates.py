#!/usr/bin/env python3
"""Orient the printed parts, pack them on plates and write Bambu Studio 3MF files.

    python print/make_plates.py [--bed 256 256] [--out print] [--check]

Needs numpy, scipy, trimesh.  Reads the part meshes from stl/v4, then stl/v3 (assembly
frame X fwd / Y left / Z up) and mirrors exactly the printable set into
print/parts/ (copies, rewritten on every run; do not edit them).  Writes print/plate_N.3mf, print/oriented/*.stl
and print/plates_report.json.  Bed size is a parameter: pass --bed 350 320 and
plates with the same (material, layer, profile) key are merged automatically.

PART table: name -> (down vector in the assembly frame, material, layer height,
walls, infill %, supports, brim, note).  'down' is the direction that ends up
pointing at the bed; it is snapped to the nearest flat facet (<= 12 deg) so
the part really sits flat.
"""
import argparse, json, os, sys, zipfile, itertools, math
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GAP = 5.0          # mm between parts on a plate (brim and supports fit)
MARGIN = 6.0       # mm to the bed edge

Z = (0, 0, -1)
# name: (down, profile key, walls, infill, support, brim_mm, orientation/strength rationale)
# profile keys are defined in PROFILES below
PARTS = {
 # --- front end, precision (0.12 mm)
 'IFS_Bulkhead_Convertible':((-1,0,0),'front',5,100,'build-plate','5','Rear face (X-) on the bed, nose up: both hinge-pin bores and the Z floor-screw pilots print as vertical holes, ear loads are radial (in-layer). Supports only where the part overhangs the bed.'),
 'IFS_Lower_Arm_L':(Z,'front',6,100,'build-plate','0','Flat (Z down): arm length runs along the layers, shock/bump bending is in-plane. Hinge and outer-pin bores horizontal: ream 1.5 mm.'),
 'IFS_Lower_Arm_R':(Z,'front',6,100,'build-plate','0','Mirror of the L arm, same orientation.'),
 'IFS_Upper_Arm_L':(Z,'front',6,100,'build-plate','0','Flat (Z down): bending in-plane. Pin bores horizontal: ream 1.5 mm.'),
 'IFS_Upper_Arm_R':(Z,'front',6,100,'build-plate','0','Mirror of the L arm.'),
 'Front_Carrier_L':((0,1,0),'front',6,100,'build-plate','0','Y-face down: widest flat face on the bed (smallest overhang); check the 1.5 mm outer-pin bores and kingpin bore, ream.'),
 'Front_Carrier_R':((0,-1,0),'front',6,100,'build-plate','0','Mirror of the L carrier.'),
 'Front_Spindle_L':((1,0,0),'front',6,100,'build-plate','3','X-face down: the wheel stub axle (Y) lies in the layer plane, so the stub is not cleaved by layer lines; the Z kingpin bore is a horizontal hole, ream 1.2 mm.'),
 'Front_Spindle_R':((1,0,0),'front',6,100,'build-plate','3','Same orientation as L (axle horizontal).'),
 'Front_Pin_Retainer_Plate_L':((-1,0,0),'front',4,100,'none','0','Flat 0.7 mm plate, no support.'),
 'Front_Pin_Retainer_Plate_R':((-1,0,0),'front',4,100,'none','0','Flat 0.7 mm plate, no support.'),
 # --- steering / small (0.12 mm)
 'Steering_Rack':((0,0,1),'front',5,100,'none','0','Flat (1.6 mm): in-plane load, vertical bores.'),
 'Steering_BellCrank':(Z,'front',5,100,'none','0','Flat (1.2 mm), vertical pivot bore.'),
 'Steering_Drag_Link':(Z,'front',5,100,'build-plate','4','Flat; the pin bosses stand proud so a few small support dots are needed under the stem.'),
 'Tie_Rod_L':(Z,'front',5,100,'none','0','Flat 1.1 mm, tension/compression is in the layer plane, vertical bores.'),
 'Tie_Rod_R':(Z,'front',5,100,'none','0','Flat 1.1 mm.'),
 'Front_Bumper':(Z,'rear',6,100,'build-plate','3','Oval skid loop and centre plate flat on the bed (Z down), hoop arches up: the tubes are chamfered square 2.6 mm so layers run along them; small supports only under the hoop arch; keep supports out of the Ø2.8 bolt counterbores.'),
 'Front_Y_Brace':((0,0,1),'front',4,100,'none','0','Upside down (feet up) so the 1.6 mm plate lies flat on the bed; vertical bores.'),
 'Servo_Horn':((1,0,0),'front',5,100,'none','0','Flat (1.5 mm), vertical spline bore (print 0.1 mm undersize and drill/ream to 3.9 mm).'),
 'Servo_Wire_Clip':((0,0,1),'rear',4,100,'none','0','Flat, no support.'),
 # --- rear (0.16 mm)
 'Rear_Trailing_Arm_L':((0,-1,0),'rear',6,100,'build-plate','0','Side face (Y) down: long axis and the vertical bending load are both in the layer plane; the pivot / axle / shock bores run vertically.'),
 'Rear_Trailing_Arm_R':((0,1,0),'rear',6,100,'build-plate','0','Mirror of the L arm.'),
 'Rear_Upper_Link_L':((0,0,1),'rear',6,100,'build-plate','0','Flat: link is a plate loaded axially, three vertical D1.5 pilots (anti-squat holes) on the tub-end stadium boss.'),
 'Rear_Upper_Link_R':((0,0,1),'rear',6,100,'build-plate','0','Flat, same as L.'),
 'Rear_Axle_Top_Bracket':((0,0,1),'rear',5,100,'none','0','Upside down so the broad foot is on the bed; the vertical pin bore stays vertical.'),
 # --- bump stops (TPU)
 'Front_Bump_Stop_L':((0,-0.72,0.70),'tpu',3,100,'none','0','Ring flat on the bed (axis vertical).'),
 'Front_Bump_Stop_R':((0,0.72,0.70),'tpu',3,100,'none','0','Ring flat on the bed (axis vertical).'),
 # --- tub and shelves (0.20 mm)
 'Chassis_Tub':(Z,'tub',4,100,'build-plate','8','Floor on the bed. Rear deck / tail overhang and the arch tops need support from the plate; supports off every bore and pocket.'),
 'ESC_RX_Shelf_L':(Z,'tub',4,100,'none','0','Flat 1.0 mm lid plate, countersunk screw holes up.'),
 'ESC_RX_Shelf_R':(Z,'tub',4,100,'none','0','Flat 1.0 mm lid plate, countersunk screw holes up.'),
 # --- cage (0.16 mm)
 'Cage':((0,0,1),'cage',4,100,'build-plate','6','Upside down: roof rails and cross tube lie on the bed (tube axes in the layer plane, so tubes are not cleaved by layer lines), legs grow up from them, foot plates print last. In the assembly orientation only ~2 mm2 would touch the bed. Brim 6 mm; supports from the plate under the tail frame and rungs.'),
 'Cabin_Cage':(Z,'cage',4,100,'build-plate','6','Feet down (best bed contact 125 mm2): the roof/A-pillar frame is an arch with almost no flat top, so roof-down would stand on 4 mm2. Posts are vertical (layer-wise): print slow and hot, 100 % walls. Tree supports from the plate under the hood rails and the A-pillar slope; the arch top needs none. Alternative: lie on its side (46 mm tall, supports land on the part).'),
 # --- shell (0.20 mm)
 'Tacoma_Trophy_Body_Shell':(Z,'shell',3,10,'tree-build-plate','8','Sill rim on the bed, roof up (39 mm tall, stable). Tree supports from the plate fill the cabin and hood underside. Alternative: nose down (X+), no support in the cabin but 142 mm tall and wobbly.'),
}

PROFILES = {
 'front':{'material':'PETG-CF (PA12-CF / PAHT-CF if you have a dry box)','layer':0.12,'nozzle':0.4,'label':'Front end and steering (precision)'},
 'rear': {'material':'PETG-CF (PA-CF if available)','layer':0.16,'nozzle':0.4,'label':'Rear suspension'},
 'tpu':  {'material':'TPU 95A','layer':0.16,'nozzle':0.4,'label':'Bump stops'},
 'tub':  {'material':'PETG-CF','layer':0.20,'nozzle':0.4,'label':'Tub and shelves'},
 'cage': {'material':'PETG (tough) or PETG-CF','layer':0.16,'nozzle':0.4,'label':'Cage'},
 'shell':{'material':'PETG (PLA also fine)','layer':0.20,'nozzle':0.4,'label':'Body shell'},
}
DENS = {'PETG-CF':1.30,'TPU':1.21,'PETG':1.27}
def density(prof):
    m = PROFILES[prof]['material']
    return 1.21 if m.startswith('TPU') else 1.30 if 'CF' in m else 1.27
RATE = {'front':1.5,'rear':2.5,'tpu':0.8,'tub':5.0,'cage':2.5,'shell':5.0}   # mm3/s effective
LAYER_S = {'front':14,'rear':9,'tpu':10,'tub':6,'cage':8,'shell':6}              # s per layer overhead

def find_src(name):
    """Assembly-frame source: stl/v4 first, then stl/v3 (both are the settled CAD output).  print/parts is a mirror written by
    this script (last-resort fallback only)."""
    for d in ('stl/v4', 'stl/v3', 'print/parts'):
        p = os.path.join(REPO, d, name + '.stl')
        if os.path.exists(p): return p
    raise FileNotFoundError(name)

def load(name):
    m = trimesh.load(find_src(name), process=True)
    m.merge_vertices()
    # collapse sliver triangles (< 0.1 um) so float32 re-quantisation cannot merge distinct vertices
    try:
        import manifold3d as mf
        mm = mf.Manifold(mf.Mesh(vert_properties=m.vertices.astype(np.float32), tri_verts=m.faces.astype(np.uint32)))
        if mm.status() == mf.Error.NoError and mm.num_tri() > 0:
            o = mm.simplify(1e-4).to_mesh()
            q = trimesh.Trimesh(np.array(o.vert_properties)[:, :3].astype(np.float64), np.array(o.tri_verts), process=False)
            parts = q.split(only_watertight=False)
            if len(parts) > 1:                      # drop zero-volume sliver shells the collapse can leave behind
                keep = [c for c in parts if abs(c.volume) > 1e-2]
                if len(keep) < len(parts): q = trimesh.util.concatenate(keep)
            if q.is_watertight and abs(q.volume - m.volume) < 1e-3 * m.volume:
                return q
    except Exception as e:
        print('manifold clean skipped for', name, e)
    return m

def snap_down(m, d):
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    best, bd = None, 12.0
    for n, a in zip(m.facets_normal, m.facets_area):
        if a < 3.0: continue
        ang = math.degrees(math.acos(np.clip(n @ d, -1, 1)))
        if ang < bd: best, bd = n, ang
    return best if best is not None else d

def rot_to_down(d):
    return trimesh.geometry.align_vectors(d, [0, 0, -1])

def f32(m):
    m2 = m.copy(); m2.vertices = m2.vertices.astype(np.float32).astype(np.float64)
    return m2

def separate_dups(m):
    """vertices that were distinct in float64 but collide in float32 (pinch points) are pushed 0.3 um apart"""
    v = m.vertices.astype(np.float32); m = m.copy()
    for _ in range(3):
        _, inv, cnt = np.unique(v, axis=0, return_inverse=True, return_counts=True)
        inv = np.asarray(inv).reshape(-1)
        dup = np.where(cnt[inv] > 1)[0]
        if not len(dup): break
        seen = set(); vv = v.astype(np.float64)
        for i in dup:
            key = tuple(v[i])
            if key in seen: vv[i, 0] += 3e-4; vv[i, 2] += 2e-4
            else: seen.add(key)
        v = vv.astype(np.float32)
    m.vertices = v.astype(np.float64)
    return m

def watertight_f32(m):
    v = m.vertices.astype(np.float32)
    m2 = trimesh.Trimesh(v.astype(np.float64), m.faces, process=True)   # merges exactly-equal vertices like an STL/3MF loader
    return m2.is_watertight and m2.volume > 0 and len(m2.faces) == len(m.faces)

NUDGES = [(0, 0, 0)] + [(a * 0.0137, b * 0.0211, 0) for a in range(0, 6) for b in range(0, 6) if (a or b)] + \
         [(a * 0.0131, b * 0.0173, c * 0.0007) for a in range(6) for b in range(6) for c in range(1, 4)]

def settle(mm, lo_xy, centre=False):
    """translate so the bbox minimum sits at lo_xy (or bbox centre at 0,0 when centre) and z=0; nudge until float32-safe"""
    lo, hi = mm.bounds
    base = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]]) if centre else np.array([lo_xy[0] - lo[0], lo_xy[1] - lo[1], -lo[2]])
    for nx in NUDGES:
        q = mm.copy(); q.apply_translation(base + np.array(nx)); q = separate_dups(q)
        if watertight_f32(q):
            q.vertices = q.vertices.astype(np.float32).astype(np.float64)
            return trimesh.Trimesh(q.vertices, q.faces, process=False)
    return None

def orient(name):
    """returns mesh oriented (down->-Z), yaw by min-area rectangle, centred at 0,0, zmin=0, float32-safe"""
    m = load(name)
    d = snap_down(m, PARTS[name][0])
    T = rot_to_down(d); m.apply_transform(T)
    # min area rectangle yaw
    try:
        T2, ext = trimesh.bounds.oriented_bounds_2D(m.vertices[:, :2])
        yaw = math.atan2(T2[1, 0], T2[0, 0])
    except Exception:
        yaw = 0.0
    tries = [yaw] + [yaw + k * math.radians(a) for a in (90, 180, 270, 7, 13, 21) for k in (1,)]
    for y in tries:
        c, s = math.cos(y), math.sin(y)
        R = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        mm = m.copy(); mm.apply_transform(R)
        q = settle(mm, None, centre=True)
        if q is not None:
            return q, d, y
    raise RuntimeError('not watertight in float32: ' + name)

def metrics(m):
    n = m.face_normals; a = m.area_faces; z = m.triangles_center[:, 2]
    bedmask = (n[:, 2] < -0.99) & (z < 0.06)
    oh = (n[:, 2] < -0.7071) & ~bedmask
    onpart = 0.0; sup_vol = 0.0
    if oh.any():
        idx = np.where(oh)[0]
        o = m.triangles_center[idx] + np.array([0, 0, -0.02])
        hit = m.ray.intersects_any(o, np.tile([0, 0, -1.0], (len(idx), 1)))
        onpart = float(a[idx][hit].sum())
        # support volume estimate: projected area * height * 12 % density (build-plate columns only)
        proj = a[idx] * np.abs(n[idx, 2])
        sup_vol = float((proj * z[idx] * 0.12).sum())
    zt = m.triangles[:, :, 2].min(1)
    contact = float((a * np.abs(n[:, 2]))[zt < 0.15].sum())
    return dict(contact=contact, bed=float(a[bedmask].sum()), overhang=float(a[oh].sum()), overhang_on_part=onpart,
                support_vol=sup_vol, height=float(m.bounds[1][2]), size=[float(x) for x in m.extents])

def pack(items, bw, bh):
    """items: list of (name, w, h). Simple shelf packing with 90 deg rotation. returns list of placements or None"""
    order = sorted(items, key=lambda t: -max(t[1], t[2]))
    shelves = []   # [y, height, x_used]
    place = {}
    y = MARGIN
    for name, w, h in order:
        done = False
        for rot in (False, True):
            ww, hh = (h, w) if rot else (w, h)
            for sh in shelves:
                if hh <= sh[1] + 1e-6 and sh[2] + ww <= bw - MARGIN + 1e-6:
                    place[name] = (sh[2], sh[0], rot, ww, hh); sh[2] += ww + GAP; done = True; break
            if done: break
        if done: continue
        # new shelf; prefer the orientation with the smaller height if it fits
        for rot in (False, True):
            ww, hh = (h, w) if rot else (w, h)
            if MARGIN + ww <= bw - MARGIN and y + hh <= bh - MARGIN:
                shelves.append([y, hh, MARGIN + ww + GAP]); place[name] = (MARGIN, y, rot, ww, hh); y += hh + GAP; done = True; break
        if not done: return None
    return place

def write_3mf(path, plate_no, objs, bed, prof):
    """objs: list of dict(name, mesh(placed, float32-clean), settings)"""
    ns = 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
    vlines = []
    res = ['<?xml version="1.0" encoding="UTF-8"?>',
           f'<model unit="millimeter" xml:lang="en-US" xmlns="{ns}">',
           f' <metadata name="Title">SCX30 SCT plate {plate_no}: {PROFILES[prof]["label"]}</metadata>',
           ' <metadata name="Designer">scx30-sct make_plates.py</metadata>', ' <resources>']
    for i, o in enumerate(objs, 1):
        m = o['mesh']
        res.append(f'  <object id="{i}" name="{o["name"]}" type="model"><mesh><vertices>')
        res += [f'   <vertex x="{v[0]:.9g}" y="{v[1]:.9g}" z="{v[2]:.9g}"/>' for v in m.vertices.astype(np.float32)]
        res.append('  </vertices><triangles>')
        res += [f'   <triangle v1="{f[0]}" v2="{f[1]}" v3="{f[2]}"/>' for f in m.faces]
        res.append('  </triangles></mesh></object>')
    res.append(' </resources><build>')
    res += [f'  <item objectid="{i}"/>' for i in range(1, len(objs) + 1)]
    res.append(' </build></model>')
    cfg = ['<?xml version="1.0" encoding="UTF-8"?>', '<config>']
    for i, o in enumerate(objs, 1):
        cfg.append(f'  <object id="{i}">')
        cfg.append(f'    <metadata key="name" value="{o["name"]}"/>')
        for k, v in o['settings'].items():
            cfg.append(f'    <metadata key="{k}" value="{v}"/>')
        cfg.append('  </object>')
    cfg.append('  <plate>')
    cfg.append('    <metadata key="plater_id" value="1"/>')
    cfg.append(f'    <metadata key="plater_name" value="plate_{plate_no}"/>')
    for i in range(1, len(objs) + 1):
        cfg.append(f'    <model_instance><metadata key="object_id" value="{i}"/><metadata key="instance_id" value="0"/></model_instance>')
    cfg += ['  </plate>', '</config>']
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
          '<Default Extension="config" ContentType="text/xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ct); z.writestr('_rels/.rels', rels)
        z.writestr('3D/3dmodel.model', '\n'.join(res)); z.writestr('Metadata/model_settings.config', '\n'.join(cfg))

def support_policy(name, mi):
    sup = PARTS[name][4]
    if sup in ('none', 'tree-build-plate'): return sup
    if mi['overhang'] > 0 and mi['overhang_on_part'] / mi['overhang'] > 0.25: return 'tree-any'
    return sup

def settings_for(name, prof, mi):
    d, p, walls, infill, sup, brim, note = PARTS[name]
    sup = support_policy(name, mi)
    s = {'layer_height': PROFILES[prof]['layer'], 'wall_loops': walls, 'sparse_infill_density': f'{infill}%',
         'top_shell_layers': 7 if PROFILES[prof]['layer'] < 0.15 else 5, 'bottom_shell_layers': 7 if PROFILES[prof]['layer'] < 0.15 else 5}
    if sup == 'none':
        s['enable_support'] = 0
    else:
        s.update({'enable_support': 1, 'support_type': 'tree(auto)' if sup.startswith('tree') else 'normal(auto)',
                  'support_on_build_plate_only': 0 if sup == 'tree-any' else 1, 'support_top_z_distance': 0.22, 'support_bottom_z_distance': 0.22,
                  'support_interface_top_layers': 2, 'support_interface_bottom_layers': 2, 'support_interface_spacing': 0.4,
                  'support_threshold_angle': 45})
    if brim != '0':
        s.update({'brim_type': 'outer_only', 'brim_width': brim})
    return s

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bed', nargs=2, type=float, default=[256, 256])
    ap.add_argument('--out', default=HERE)
    a = ap.parse_args()
    bw, bh = a.bed
    os.makedirs(os.path.join(a.out, 'oriented'), exist_ok=True)
    # keep print/parts = exactly the printable set, copied from the CAD output (assembly pose)
    import shutil
    pp = os.path.join(REPO, 'print/parts'); os.makedirs(pp, exist_ok=True)
    for f in os.listdir(pp):
        if f.endswith('.stl') and f[:-4] not in PARTS: os.remove(os.path.join(pp, f))
    for n in PARTS:
        src = find_src(n)
        if os.path.abspath(src) != os.path.abspath(os.path.join(pp, n + '.stl')): shutil.copyfile(src, os.path.join(pp, n + '.stl'))
    oriented, info = {}, {}
    for name in PARTS:
        m, d, yaw = orient(name)
        oriented[name] = m
        info[name] = metrics(m); info[name]['down'] = [round(float(x), 4) for x in d]
        info[name]['volume'] = float(m.volume); info[name]['support'] = support_policy(name, info[name]); info[name]['brim'] = PARTS[name][5]
        m.export(os.path.join(a.out, 'oriented', name + '.stl'))
    # group by profile, keep each profile's parts on its own plate(s); merge profiles only if keys equal
    groups = {}
    for name, v in PARTS.items():
        groups.setdefault(v[1], []).append(name)
    plates = []
    for prof in ['front', 'rear', 'tpu', 'tub', 'cage', 'shell']:
        names = groups.get(prof, [])
        rest = names[:]
        while rest:
            items = [(n, oriented[n].extents[0], oriented[n].extents[1]) for n in rest]
            # take as many as fit
            sel = []
            for it in sorted(items, key=lambda t: -t[1] * t[2]):
                if pack(sel + [it], bw, bh) is not None: sel.append(it)
            plates.append((prof, [s[0] for s in sel], pack(sel, bw, bh)))
            rest = [n for n in rest if n not in [s[0] for s in sel]]
    # merge plates of the same layer height + material when the bed is large enough
    report = []
    for pi, (prof, names, place) in enumerate(plates, 1):
        objs = []
        for n in names:
            x, y, rot, ww, hh = place[n]
            m = oriented[n].copy()
            if rot:
                R = trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]); m.apply_transform(R)
            m = settle(m, (x, y))
            assert m is not None, n + ' cannot be placed float32-watertight'
            objs.append(dict(name=n, mesh=m, settings=settings_for(n, prof, info[n])))
        write_3mf(os.path.join(a.out, f'plate_{pi}.3mf'), pi, objs, (bw, bh), prof)
        vol = sum(info[n]['volume'] for n in names); sup = sum(info[n]['support_vol'] if support_policy(n, info[n]) != 'none' else 0 for n in names)
        hmax = max(info[n]['height'] for n in names)
        g = (vol + sup) * density(prof) / 1000
        t = (vol + sup) / RATE[prof] + hmax / PROFILES[prof]['layer'] * LAYER_S[prof]
        report.append(dict(plate=pi, profile=prof, parts=names, volume_mm3=vol, support_mm3=sup, grams=g,
                           hours=t / 3600, hmax=hmax, placement={n: [float(place[n][0]), float(place[n][1]), bool(place[n][2]), float(place[n][3]), float(place[n][4])] for n in names}))
    json.dump(dict(bed=[bw, bh], plates=report, parts=info), open(os.path.join(a.out, 'plates_report.json'), 'w'), indent=1)
    for r in report:
        print(f"plate_{r['plate']}.3mf  {r['profile']:6s} {len(r['parts']):2d} parts  {r['grams']:.1f} g  ~{r['hours']:.1f} h  Hmax {r['hmax']:.1f}")

if __name__ == '__main__':
    main()
