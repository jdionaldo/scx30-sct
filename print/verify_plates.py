#!/usr/bin/env python3
"""Verify print/plate_N.3mf and print/oriented/*.stl: watertight (after reload), on the bed, inside the bed, no overlaps, support need."""
import glob, os, sys, zipfile, re, json, itertools
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__))
rep = json.load(open(os.path.join(HERE, 'plates_report.json')))
BX, BY = rep['bed']; bad = 0
def parse(path):
    z = zipfile.ZipFile(path); x = z.read('3D/3dmodel.model').decode()
    objs = []
    for m in re.finditer(r'<object id="(\d+)" name="([^"]+)".*?</object>', x, re.S):
        v = np.array(re.findall(r'<vertex x="([^"]+)" y="([^"]+)" z="([^"]+)"', m.group(0)), float)
        f = np.array(re.findall(r'<triangle v1="(\d+)" v2="(\d+)" v3="(\d+)"', m.group(0)), int)
        objs.append((m.group(2), trimesh.Trimesh(v, f, process=True)))
    return objs, z
for p in sorted(glob.glob(os.path.join(HERE, 'plate_*.3mf'))):
    objs, z = parse(p)
    names = z.namelist(); print(os.path.basename(p), len(objs), 'objects', 'config' if 'Metadata/model_settings.config' in names else 'NO CONFIG')
    boxes = []
    for n, m in objs:
        _, ec = np.unique(m.edges_sorted, axis=0, return_counts=True)
        oe = int((ec == 1).sum()); nme = int((ec != 2).sum())
        sh = len(m.split(only_watertight=False))
        deg = int((m.area_faces < 1e-6).sum())
        ok = m.is_watertight and m.volume > 0 and oe == 0 and nme == 0 and sh == 1
        lo, hi = m.bounds; inbed = lo[0] >= 0 and lo[1] >= 0 and hi[0] <= BX and hi[1] <= BY and abs(lo[2]) < 1e-3
        mi = rep['parts'][n]
        flag = 'OK ' if ok and inbed and (mi['contact'] >= 5 or mi['brim'] != '0') else 'BAD'
        if flag == 'BAD': bad += 1
        print(f"  {flag} {n:28s} wt={ok} sh={sh} oe={oe} nme={nme} deg={deg:4d} inbed={inbed} vol={m.volume:8.1f} h={hi[2]:5.1f} contact={mi['contact']:6.1f}mm2 sup={mi['support']:16s} overhang={mi['overhang']:6.1f} (on part {mi['overhang_on_part']:5.1f})")
        boxes.append((n, lo, hi))
    for (a, la, ha), (b, lb, hb) in itertools.combinations(boxes, 2):
        gx = max(la[0] - hb[0], lb[0] - ha[0]); gy = max(la[1] - hb[1], lb[1] - ha[1])
        if max(gx, gy) < 2.0: print('  OVERLAP/TOO CLOSE', a, b, round(max(gx, gy), 2)); bad += 1
for f in sorted(glob.glob(os.path.join(HERE, 'oriented', '*.stl'))):
    m = trimesh.load(f)
    if not (m.is_watertight and m.volume > 0): print('STL not watertight', f); bad += 1
print('STL roundtrip checked:', len(glob.glob(os.path.join(HERE, 'oriented', '*.stl'))), 'files')
print('FAILURES:', bad); sys.exit(1 if bad else 0)
