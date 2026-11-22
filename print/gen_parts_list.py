#!/usr/bin/env python3
"""Generate parts_list.md, parts_list.csv (repo root) and print/PLATES.md from print/plates_report.json + the tables below.
    python print/gen_parts_list.py
Hardware dimensions were measured from the stl/v3 + stl/v4 HW_* bodies (axis-aligned bbox + slice widths); see parts_list.md."""
import os, sys, json, csv
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.argv = sys.argv[:1]
import make_plates as M
rep = json.load(open(HERE + '/plates_report.json')); est = json.load(open(HERE + '/slicer_estimates.json'))
plate_of = {n: p['plate'] for p in rep['plates'] for n in p['parts']}

# ------------------------------------------------------------------ printed parts
# name: (display, qty-in-this-file, material, why)
MAT = {
 'IFS_Lower_Arm_L': ('PA12-CF (PETG-CF OK)', 'Takes shock + bump + wheel loads in bending; pin bores wear. Highest-loaded printed part at the front.'),
 'IFS_Lower_Arm_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'IFS_Upper_Arm_L': ('PA12-CF (PETG-CF OK)', 'Bending + hinge-pin bearing loads.'),
 'IFS_Upper_Arm_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Front_Carrier_L': ('PA12-CF (PETG-CF OK)', 'Clamps the kingpin + outer arm pins; sees cornering loads.'),
 'Front_Carrier_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Front_Spindle_L': ('PA12-CF (PETG-CF OK)', 'Wheel stub / bearing seat and steering arm: bending of the stub is the failure mode.'),
 'Front_Spindle_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Front_Pin_Retainer_Plate_L': ('PETG-CF or PETG', 'Only keeps the hinge pins from walking out (very low load), 0.7 mm thick.'),
 'Front_Pin_Retainer_Plate_R': ('PETG-CF or PETG', 'As L.'),
 'Steering_Rack': ('PA12-CF (PETG-CF OK)', 'Cross link carrying both tie-rod pickups; thin (1.6 mm), in-plane load, wear at the slot.'),
 'Steering_BellCrank': ('PA12-CF (PETG-CF OK)', '1.2 mm plate loaded in-plane, pivots on a steel pin.'),
 'Steering_Drag_Link': ('PA12-CF (PETG-CF OK)', 'Tension/compression link, elbow plate.'),
 'Tie_Rod_L': ('PA12-CF (PETG-CF OK)', '1.1 mm x 31 mm strut: compression buckling governs, stiffest material wins.'),
 'Tie_Rod_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Front_Bumper': ('PETG-CF or PA12-CF (impact)', 'Bull-bar / guard in one part (new guard profile), takes front hits; ties to the bulkhead with 2 bolts. PA12-CF absorbs impacts better than PETG-CF.'),
 'Front_Y_Brace': ('PETG-CF', '1.6 mm plate over the nose, ties the hoop top bar to the nose posts; light load.'),
 'Servo_Horn': ('PA12-CF (or the metal horn from the servo kit)', 'Press fit on a 3.93 mm spline, 1.5 mm thick, takes the full servo torque (4 kgf.cm).'),
 'Front_Bump_Stop_L': ('TPU 95A', 'Elastic stop on the shock rod; must compress, not crack.'),
 'Front_Bump_Stop_R': ('TPU 95A', 'As L.'),
 'Rear_Trailing_Arm_L': ('PA12-CF (PETG-CF OK)', 'Rear lower link: carries axle torque reaction, shock load and cornering; D3 plate-beam arm (deeper flat beam) with three screw bores, a mirrored sway-bar clevis tab and a Ø1.2 steel pin.'),
 'Rear_Trailing_Arm_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Rear_Upper_Link_L': ('PETG-CF', 'Axially loaded link with a stadium boss carrying three D1.5 pilots (anti-squat hole choice); PETG-CF is stiff enough, PA-CF only if it flexes.'),
 'Rear_Upper_Link_R': ('PETG-CF', 'As L.'),
 'Rear_Axle_Top_Bracket': ('PETG-CF', 'Carries both upper links on one pin; small but 4.3 mm tall and loaded in shear.'),
 'Chassis_Tub': ('PETG-CF (PA12-CF upgrade)', 'Unified monolithic chassis: tub + fused front bulkhead (dish floor) + chamfered diff drop + 36-deg boat-side chamfers + one rib lattice, skirt taper and ledge chamfer; carries hinge pins, servo, bell-crank pivot and cage mounts.'),
 'Cage': ('PETG (tough); PETG-CF if you want it stiffer', 'One-piece roll cage (rear cage + cabin cage merged; no light bar or radiator parts): CF makes it stiff but brittle in a crash; plain PETG bends instead of snapping.'),
 'Tacoma_Trophy_Body_Shell': ('PETG', 'Non-structural; PETG survives rolls. No PLA: the shell covers the motor and ESC.'),
}
ASSUMED_PARTS = {'Chassis_Tub': 'unified tub + fused front bulkhead + 36-deg boat-side chamfers + chamfered diff drop (floor top Z +0.9, diff drop Z -2.7) + rear driveshaft U-slot + 2 shell magnet seats on the front shock tower (X=57.3, Y=+-7.6: Ø4.8 boss, Ø3.2 x 2.1 pocket open upward, seat top Z 27.2, Ø1.0 push-out hole); ModRax longitudinal gearbox 2 diagonal M1.4 bottom foot holes at (-4.8,-9.4),(4.8,9.4)',
                 'Front_Bumper': 'Yeti-style guard / bull bar (new guard profile), zero tyre interference across full lock x bump',
                 'Front_Y_Brace': 'Yeti-style Y-brace plate tying front hoop top bar and nose posts back to bulkhead deck',
                 'Rear_Upper_Link_L': 'anti-squat numbers assume CG 21.5 mm and wheelbase 116.5 mm: top hole 22.8 %, middle 54.7 %, lowest 85.9 %', 'Rear_Upper_Link_R': 'as L',
                 'Servo_Horn': 'verified for Injora N30 PRO 28T Ø3.93 mm spline: print a test horn first or use kit horn',
                 'Front_Bump_Stop_L': 'TPU stop ring on shock rod for Injora SCX30-08 (26.0 ext / 18.3 comp, 7.7 mm travel)', 'Front_Bump_Stop_R': 'as L',
                 'Cage': 'One-piece Yeti-style cage (rear cage + cabin cage merged) with twin-position shock tabs, rear tilt-open hinge ears (pin axis along Y at X=-64.0, Z=29.0, Ø2.2 bore + head pocket for the M2 x 8 hinge screws), and the rear sway-bar saddles; mounts with 12 M2 x 6 screws on foot bosses (clean joints); the rear sway-bar saddles are part of it',
                 'Tacoma_Trophy_Body_Shell': 'Tilt-open mounting with tail fins and a rear arch (circle R 21.1), no front body posts: 2 magnet pockets under the hood over the tower seats (X=57.3, Y=+-7.6, Ø3.0 x 2.0 neodymium discs) + 2 rear tilt-hinge lugs outside the Cage ears (pin axis along Y at X=-64.0, Z=29.0, |Y| 18.9..21.3)'}

def printed_rows():
    rows = []
    for n, v in M.PARTS.items():
        mi = rep['parts'][n]
        rows.append(dict(name=n, qty=1, file=f'print/parts/{n}.stl', oriented=f'print/oriented/{n}.stl', plate=plate_of[n], material=MAT[n][0], why=MAT[n][1],
                         down=mi['down'], height=round(mi['height'], 1), size=[round(x, 1) for x in mi['size']], volume=round(mi['volume'], 1),
                         contact=round(mi['contact'], 1), overhang=round(mi['overhang'], 1), support=mi['support'], brim=v[5], orient=v[6],
                         layer=M.PROFILES[v[1]]['layer'], walls=v[2], infill=v[3], verify=ASSUMED_PARTS.get(n, '')))
    return rows

# ------------------------------------------------------------------ hardware per joint
# (joint, fastener, qty, modelled Ø mm / under-head length mm / head, bodies, suggested stock part, status)
HW = [
 # FRONT
 ('Front: upper arm inboard hinge', 'plain pin', 2, 'Ø1.55 x 18.4', 'HW_Front_Upper_Hinge_Pin_L/R', 'Ø1.5 mm stainless dowel / music wire cut to 18.4 mm (bore 1.55)', 'ok'),
 ('Front: lower arm inboard hinge', 'plain pin', 2, 'Ø1.55 x 16.8 (trimmed at X>=40.2)', 'HW_Front_Lower_Hinge_Pin_L/R', 'Ø1.5 mm dowel cut to 16.8 mm', 'ok'),
 ('Front: hinge-pin retainer plate', 'screw (self-tapping)', 2, 'shank Ø1.4, head Ø2.4 x 0.7, 3.0 under head', 'HW_Front_Retainer_Screw_L/R', 'M1.4 x 3', 'ok'),
 ('Front: upper arm to carrier', 'plain pin', 2, 'Ø1.5 x 8.0', 'HW_Front_Upper_Outer_Pin_L/R', 'Ø1.5 mm dowel cut to 8.0 mm', 'ok'),
 ('Front: lower arm to carrier', 'plain pin', 2, 'Ø1.5 x 8.0', 'HW_Front_Lower_Outer_Pin_L/R', 'Ø1.5 mm dowel cut to 8.0 mm', 'ok'),
 ('Front: kingpin, upper (carrier to spindle, vertical)', 'plain pin, no nut', 2, 'Ø1.2 x 4.7', 'HW_Front_Upper_Kingpin_L/R', 'Ø1.2 mm music wire cut to 4.7 mm, press fit, a dot of CA to retain', 'check retention'),
 ('Front: kingpin, lower (carrier to spindle, vertical)', 'plain pin, no nut', 2, 'Ø1.2 x 3.9', 'HW_Front_Lower_Kingpin_L/R', 'Ø1.2 mm music wire cut to 3.9 mm', 'check retention'),
 ('Front: shock lower (arm)', 'screw (self-tapping)', 2, 'shank Ø1.35, head Ø2.4 x 0.9, 4.0 under head', 'HW_Front_Shock_Lower_Screw_L/R', 'M1.4 x 4', 'ok'),
 ('Front: shock upper (bulkhead fork tower, through wall-eye-wall)', 'screw (self-tapping)', 2, 'shank Ø1.35, head Ø2.4 x 0.9, 3.3 under head (4.2 total)', 'HW_Front_Shock_Upper_Screw_L/R', 'M1.4 x 3 (3.3 modelled; eye 1.4 wide between two 0.8 mm tower walls)', 'verify before print'),
 ('Front: wheel axle nut', 'nut', 2, 'hex 4.2 across corners x 1.6 (shank 2.9)', 'HW_Front_Wheel_Nut_L/R', 'stock SCX30 wheel nut (reuse)', 'ok'),
 ('Front: servo ears (tub floor pocket / bulkhead)', 'screw (self-tapping)', 2, 'shank Ø1.0, head Ø2.0 x 0.6, 3.5 under head', 'HW_Servo_Mount_Screw_L/R', 'M1.0 x 3.5 (M1.0 x 3 per FRONT_V3.md; use the servo kit screws if they fit)', 'verify before print (servo hole spacing 10.0 is a pixel estimate)'),
 ('Front: servo horn centre screw', 'screw', 1, 'Ø2.65 x 2.4 overall envelope (head and shank not separable in the mesh)', 'HW_Horn_Screw', 'screw supplied with the servo (FRONT_V3.md says ~M1.2 x 3: mismatch with the model)', 'verify before print'),
 ('Steering: horn to drag link (pin A)', 'headed pin / screw', 1, 'shank Ø1.2, head Ø1.7 x 0.2, 2.4 under head', 'HW_Link_Pin_A', 'M1.0-class screw or Ø1.2 pin with a flat head, ~2.5 mm', 'ok'),
 ('Steering: bell crank pivot', 'headed steel pin', 1, 'shank Ø1.2, head Ø2.2 x 0.3, 2.8 under head', 'HW_Crank_Pivot_Pin', 'Ø1.2 steel pin pressed into the Ø1.1 pilot in the floor boss', 'ok'),
 ('Steering: drag link to crank (input)', 'headed pin', 1, 'shank Ø1.0, head Ø1.7 x 0.3, 2.35 under head', 'HW_Crank_In_Pin', 'Ø1.0 pin / M1.0 x 2.5', 'ok'),
 ('Steering: crank to rack (output)', 'plain pin (headless in model)', 1, 'Ø1.0 x 2.35', 'HW_Crank_Out_Pin', 'Ø1.0 pin cut to 2.3 mm (retention not modelled)', 'check'),
 ('Steering: rack guide', 'screw (self-tapping)', 1, 'shank Ø1.1, head Ø1.8 x 0.5, 3.6 under head', 'HW_Rack_Guide_Screw', 'M1.0-1.2 x 3.5 (FRONT_V3.md says M1.4 x 4, model shank is Ø1.1: mismatch)', 'verify before print'),
 ('Steering: tie rod inner (rack end)', 'screw (self-tapping)', 2, 'shank Ø1.4, head Ø2.0 x 0.5, 2.75 under head', 'HW_TieRod_Inner_Screw_L/R', 'M1.4 x 3 (2.75 modelled; rack pilot Ø1.2)', 'check'),
 ('Steering: tie rod outer (spindle end)', 'screw (self-tapping)', 2, 'shank Ø1.4, head Ø2.0 x 0.3, 2.45 under head', 'HW_TieRod_Outer_Screw_L/R', 'M1.4 x 2.5', 'check'),
 ('Front: gearbox to worm shaft coupler (gearbox end)', 'sleeve coupler', 1, 'OD 3.5 x 3.5 long, bore Ø3.1', 'HW_Front_Driveshaft_UJ_Gbx', 'stock SCX30 driveshaft yoke/sleeve; seats over the gearbox Ø3 output stub', 'verify before print (gearbox stub height Z 4.1 and X unmeasured)'),
 ('Front: worm shaft coupler (diff end)', 'U-joint / dogbone coupler', 1, 'Ø3.0 x 4.3', 'HW_Front_Driveshaft_UJ_Worm', 'stock SCX30 driveshaft U-joint; worm-end seat is a Ø4.0 pocket in the bulkhead tunnel', 'verify before print'),
 # TUB
 ('Front: bumper through-bolts (cheese head below the bumper plate, rear-open slots)', 'bolt', 2, 'shank Ø2.0 x 14.0 under head, head Ø3.8 x 1.3 (15.3 total)', 'HW_Front_Bumper_Bolt_L/R', 'M2 x 14 cheese head (passes plate, pad, bulkhead floor, post Ø3.8, top skin, Y-brace ear)', 'verify before print'),
 ('Front: bumper nut (on the Y-brace ear)', 'nut', 2, '4.6 across corners / 4.0 across flats x 1.6', 'HW_Front_Bumper_Nut_L/R', 'M2 hex nut (4.0 AF)', 'ok'),
 ('Front: Y-brace front lug to the hoop top bar', 'screw (self-tapping)', 1, 'head Ø2.6, 4.0 total, into a Ø1.2 pilot', 'HW_Front_YBrace_Front_Screw', 'M1.4 x 3-4 (FRONT_V3.md says x4)', 'verify before print'),
 ('Motor to gearbox motor face', 'gearbox motor screws (supplied with the gearbox)', 0, 'pattern, size and count UNMEASURED (Motor_Clamp and HW_Motor_Clamp_Screw_1..4 were deleted: the motor mounts directly on the gearbox)', 'gearbox kit screws (motor body modelled Ø17 x 10, axis X, Z 18.1)', 'stock screws from the gearbox kit', 'verify before print'),
 # CAGE
('Cage: 6 per side (rim feet and tail foot; one-piece cage)', 'screw (self-tapping)', 10, 'shank Ø2.0, head Ø3.8 x 1.3, 4.7 mm in the tub pilot (4.5 at the tail)', 'HW_Cage_Screw_L1, L2, L4, L5, L6, R1, R2, R4, R5, R6', 'M2 x 6 pan head', 'ok'),
('Cage: tail foot (1 per side, inboard tail)', 'screw (self-tapping)', 2, 'shank Ø2.0, head Ø3.8 x 1.3, 4.5 mm in the pilot', 'HW_Cage_Screw_L3, R3', 'M2 x 6 pan head', 'ok'),
 # REAR
 ('Rear: trailing arm to tub pad', 'screw (self-tapping, no nut)', 2, 'shank Ø2.0, head Ø3.8 x 1.1, 11.05 under head (12.15 total; 1.05 longer since the rear track grew by 2.1 mm)', 'HW_Rear_Trailing_Pivot_Screw_L/R', 'M2 x 12', 'verify before print (was M2 x 10)'),
 ('Rear: upper link to tub pad', 'screw (self-tapping into the link eye, no nut)', 2, 'shank Ø1.4, head Ø2.7 x 0.9, 5.8 under head', 'HW_Rear_Upper_Link_Pivot_Screw_L/R', 'M1.4 x 6', 'ok'),
 ('Rear: upper links to axle bracket', 'headed pin / screw', 1, 'shank Ø1.4, head Ø2.6 x 0.4, 6.7 under head, vertical', 'HW_Rear_Axle_Top_Pin', 'M1.4 x 7 -> x6 + washer, or Ø1.4 pin with a flat head', 'check'),
 ('Rear: trailing arm to axle ear', 'bolt', 2, 'shank Ø1.8, head Ø2.8 x 1.1, 4.8 under head (head seated at Y +-23.15)', 'HW_Rear_Trailing_Axle_Screw_L/R', 'M2 x 5 (shank modelled Ø1.8)', 'verify before print (see nut)'),
 ('Rear: trailing arm axle nut', 'hex nut', 2, 'about 2.4 across flats / 2.75 across corners (unchanged this round)', 'HW_Rear_Trailing_Axle_Nut_L/R', 'a real M2 nut is 4.0 AF: modelled nut is M1.2-M1.4 size and will not match the Ø1.8 bolt; measure the stock SCX30 axle-ear fastener and re-check the nut seat on the arm inner cheek', 'ASSUMED: verify before print'),
 ('Rear: sway-link pin in the trailing-arm clevis', 'steel pin, CA-retained', 2, 'Ø1.2 x 3.4 along Y through the clevis hole Ø1.3 (clevis is part of Rear_Trailing_Arm_L/R)', 'HW_Sway_Link_Pin_L/R', 'steel pin Ø1.2 x 3.4 + a drop of CA', 'check: retention is glue only'),
 ('Rear: shock lower (arm outer cheek)', 'screw (self-tapping, no nut)', 2, 'shank Ø1.44, head Ø2.2 x 0.75, 8.0 under head (8.75 total)', 'HW_Rear_Shock_Lower_Screw_L/R', 'M1.6 x 8', 'ok'),
 ('Rear: shock upper (cage tail-post hanger tab, spacer sleeve under the eye)', 'screw (self-tapping into the tab pilot) + spacer sleeve', 2, 'shank Ø1.44 (M1.6 pilot), head Ø2.2 x 0.8, 7.4 under head (8.2 total); sleeve Ø2.6 x 0.8', 'HW_Rear_Shock_Upper_Screw_L/R', 'M1.6 x 8 (7.4 modelled); sleeve = 0.8 mm spacer (cut 1.7 ID brass tube or the shock bushing)', 'verify before print'),
 ('Shell: rear tilt hinge (1 per side, along Y at X -64.0, Z 29.0, head inboard of the Cage ear)', 'screw (self-tapping into the shell lug)', 2, 'shank Ø2.0, head Ø3.8 x 1.3, 8.0 under head (9.3 total); Cage ear bore Ø2.2', 'HW_Rear_Hinge_Screw_L/R', 'M2 x 8', 'check: driver access from inboard is untested (the radiator that blocked it is gone)'),
 ('Rear: wheel nut', 'nut', 2, 'hex 4.2 across corners x 3.8 long', 'HW_Rear_Wheel_Nut_L/R', 'stock SCX30 rear wheel nut (reuse)', 'ok'),
]

# ------------------------------------------------------------------ purchased / off the shelf
# (group, item, qty, spec / modelled size, source, status)
BUY = [
 ('SCX30 donor (retained)', 'Rear axle assembly (housing, worm/ring, bearings, hubs)', 1, 'Ref_Rear_Axle 16.5 x 63.0 x 12.5 mm (axle centre X -64, shifted 5 mm back vs stock layout)', 'Axial SCX30 (AXI-3311 axle set)', 'ok; modelled axle-ear fastener unmeasured'),
 ('SCX30 donor (retained)', 'Rear driveshaft (with U-joints)', 1, 'Ref_Rear_Driveshaft 40.3 x 4.0 x 6.7 mm; angle limit 21 deg at droop (double-joint rear shaft assumed, rating unverified)', 'Axial AXI-4428 long set (BUILD_GUIDE.md) / stock', 'verify before print (length and joint rating)'),
 ('SCX30 donor (retained)', 'Wheels + tyres', 4, 'Ø36.0 x 13.5 mm (Ref_Wheel_*), stock Tacoma tyre is Ø34 in BUILD_GUIDE.md', 'Axial SCX30 / SCT 0.7"', 'verify tyre OD (shell clearance is computed with Ø36)'),
 ('SCX30 donor (retained)', 'Wheel nuts', 4, 'M2 hex, see HW_*_Wheel_Nut', 'stock', 'ok'),
 ('SCX30 donor (retained)', 'Front diff housing + ring gear + input worm', 1, 'Ref_Front_Diff_Ring_Gear (Y +-10.3, ring axis X 48, Z 3.0), Ref_Front_Input_Worm_Gear (12.2 long, Ø3.3, axis Z 5.2..6.85)', 'stock SCX30 front axle diff, REPLACED by Injora planetary set below if you use it', 'worm/ring centre distance 3.85 mm may change with the 2/13T Injora gear: verify'),
 ('Front axle upgrade', 'Injora +4.5 mm Planet Axle set SCX30-34BKFR (diff + stainless double-joint shafts, ~52 deg joints)', 1, '2 x double-joint shaft modelled as CVD_Inner (27.5 x Ø3.4) + CVD_Outer (7.2 x Ø2.0); front gear 2/13T', 'Injora SCX30-34BKFR', 'ok; joint angle margin 15.3 deg at +-2.8 mm rack'),
 ('Front axle upgrade', 'Front hub bearings', 4, '2 per hub: CAD seat Ø4.10 x 1.6 (Front_Bearing_*, Front_Bearing_Inner_*, ID Ø2.0); stock SCX30 AXI-3318 uses 2x5x2.5 (MR52ZZ) outer + 3x6x2 (MR63ZZ) inner', 'MR42ZZ (2 x 4 x 1.5 mm) + 0.2 mm shim for current Ø4.10 spindle bore (or ream to Ø5.10 for stock MR52ZZ 2x5x2.5)', 'verified OTS option (MR42ZZ fits current CAD bore)'),
 ('Drivetrain', 'ModRax Longitudinal CNC Metal Gearbox (B0HMR5D54N class, eliminates transmission worm stage)', 1, 'inline longitudinal spur train (replaces stock AXI-3311 internal 2T/16T worm stage); Ø3.0 mm front/rear output stubs at Z 9.90 (raised +5.8); 2 diagonal M1.4 bottom foot holes at (-4.8,-9.4),(4.8,9.4)', 'ModRax longitudinal SCX30 gearbox (user confirmed)', 'confirmed user hardware lock; pair with Injora 2/12T overdrive axle worms or FMS FCX24M/FMT24 2:1 bevel diffs for zero worm drag'),
 ('Drivetrain', 'Pinion 0.3 mod, 11T steel (pre-installed on GAC GM1614; 11T-15T compatible)', 1, '0.3 MOD 11T steel pinion pre-installed on GM1614 shaft; meshes with ModRax adjustable 9-10 mm motor mount slot', 'GAC Racing GM1614 + G40 Pro kit', 'verified 0.3 MOD 11T pre-installed'),
 ('Drivetrain', 'Front driveshaft centre shaft', 1, 'Ø2.2 x 23.6 mm (Ref_Front_Center_Driveshaft, 6.7 deg between U-joints)', 'MEUS SCX30 steel shaft (26-30 mm) or cut Ø2.2 steel rod + stock UJ couplers', 'ok'),
 ('Power', 'GAC Racing GM1614 outrunner motor (3500/7500 KV, 14-pole, 2S-3S)', 1, 'verified ~Ø17.0 x 10.5 mm incl. front 4-lobe SCX30 mounting flange (CAD Ø17 x 10 at Z 23.9); radial 90-deg PH2.0 3P wire boot at front flange -> clock wire exit +Z (upward)', 'GAC Racing GM1614 + G40 Pro Power System ($54.98)', 'confirmed user hardware lock'),
 ('Power', 'ESC: GAC Racing G40 Pro nano (FOC, IP67, Bluetooth, 5A 6.0V/7.4V BEC)', 1, 'verified 30 L x 17 D x 6 H mm; all 9 wires exit perpendicular to flat 30x17 potted face -> install upright in side bay with potted wire face INBOARD (+Y) and power leads at TOP (Z~28..30)', 'GAC Racing GM1614 + G40 Pro Power System ($54.98)', 'confirmed user hardware lock'),
 ('Power', 'Receiver (3-ch/4-ch micro, e.g. Injora 4CH micro or Flysky GT5-class decased)', 1, 'user-measured 20 L x 15 D x 7 H mm', 'user choice / Injora 4CH ($12.99)', 'verified fit in side bay / centreline pocket'),
 ('Power', 'Battery 2S LiPo <= 45 x 20 x 11 mm (e.g. GNB 300mAh 2S 44x15.5x11 or Tattu 300mAh 2S 45x16x12)', 1, 'sits on rear deck + cage cradle rungs (X -74.9..-29.6, Z 18.1..29.3), strapped through deck slots', 'GNB / Tattu 300mAh 2S LiPo (PH2.0 / JST)', 'verified OTS fit'),
 ('Steering', 'Servo: Injora N30 PRO NANO Servo (28T Ø3.93 mm spline, 15.8 x 12.85 ear flange x 21.98 mm)', 1, 'mounted on its side in bulkhead cradle (X 13.4..33.5, Y 1.8..11.7, Z 0.8..16.6); ear hole spacing 10.0 mm, M1.0 x 3.5 mount screws', 'Injora N30 PRO NANO ($38.99)', 'verified OTS dimensions'),
 ('Suspension', 'Front oil shocks: Injora SCX30-08 (alu threaded body, nylon lower end; 26.0 mm extended / 18.3 mm compressed, 7.7 mm travel)', 2, 'ride 21.64 mm eye-to-eye at arm -4 deg;SHOCK_UP=[58.6,12.0,20.5] gooseneck fork tower, SHOCK_LO0=[58.6,22.5,2.5]; verified >= 1.45 mm to wheel envelope and >= 1.03 mm body-to-bulkhead across -12..+4 deg', 'Injora SCX30-08 ($17.99)', 'verified OTS fit (>= 1.0 mm clearance)'),
 ('Suspension', 'Rear oil shocks: Injora 36 mm big-bore SCX24-198 (36.0 ext / 29.0 comp, Ø9.5 spring) OR slim-spring alternatives Injora LOS-07 (35.5 mm, Ø7.8 spring, $17.99) / SCX24-155 (39.0 ext / 28.5 comp, Ø8.0 spring, $19.99)', 2, 'mounted on Yeti twin-position rear cage shock tabs; LOS-07 (35.5 mm) or SCX24-155 (39 mm slim Ø8.0 spring) recommended for +1.5..2.0 mm extra spring clearance and bump margin', 'Injora SCX24-198 ($20.99) / LOS-07 ($17.99) / SCX24-155 ($19.99)', 'verified OTS options (use M1.4 screws if using stock pivot balls)'),
 ('Suspension', 'Rear shock NOT used: Injora 40 mm SCX24-179 (needs the upper pin at roof-rail height: shell clearance fails) and 52 mm AX24-06 (upper pin would sit over the ESC bay, only 4 of 12 mm used)', 0, 'fit-checked in TUB_V4.md Rev 38', 'Injora SCX24-179 / AX24-06', 'rejected'),
 ('Suspension', 'Rear shock hardware of the LOS-07 class, for reference (if the Injora 36 mm kit lacks it): M1.4 x 11.5 top screw + M1.4 nut + 6.7 mm brass bushing + 2 o-rings; M1.4 x 8 bottom screw + brass bushing', 0, 'reference only; the model uses M1.6 x 8 self-tapping screws into printed pilots', 'Injora LOS-07', 'reference'),
('Suspension', 'Rear sway bar: bought Ø0.8 steel wire, bent (cross wire at Z 17.2 plus two levers with a jog); reference body Ref_Rear_Sway_Bar, NOT printed', 1, 'the saddles that carry the cross wire are now part of the printed Cage', 'steel wire Ø0.8', 'verify the bend before print'),
('Suspension', 'Sway-bar drop links: bought Ø0.9 steel wire with a flattened eye, reference only (Ref_Sway_Drop_Link_L/R), NOT printed', 2, 'one per side, from the bar lever to the clevis on the trailing arm', 'steel wire Ø0.9', 'verify before print'),
 ('Wiring & consumables', 'Motor/ESC/battery leads, servo extension, heat shrink, zip ties', 1, 'lead length not modelled', 'misc', '-'),
 ('Wiring & consumables', 'Threadlock (low strength), CA glue, grease for gears, 1.5 mm and 1.2 mm drill bits / reamers', 1, 'for press fits and reaming printed bores', 'misc', '-'),
 ('Body mounting', 'Shell front magnets: neodymium disc Ø3.0 x 2.0 mm (N35 or stronger)', 4, '2 pairs: one disc glued (CA) in each tower seat pocket (Ø3.2, X 57.3, Y +-7.6, seat top Z 27.2) and one in each hood-underside pocket above it; check polarity before gluing (pairs must attract); Ø1.0 push-out hole under each tower seat', 'generic Ø3 x 2 mm disc magnets', 'not modelled as bodies; holding force not tested'),
 ('Body mounting', 'Shell rear tilt-hinge screws: M2 x 8 (modelled as HW_Rear_Hinge_Screw_L/R, counted in the fastener table)', 0, 'one per side along Y at X -64.0, Z 29.0: from inboard through the Cage ear (bore Ø2.2, head pocket) into the shell lug', 'M2 x 8', 'driver access from inboard is blocked by the radiator: verify on the print'),
]


CAND = [
 ('Front diff + double-joint shafts (used)', 'Injora +4.5 mm Planet Axles with Double Joint Shafts (SCX30-34BKFR)', '$41.99-59.99', 'alternative: +3 mm Planet Axles with Brass Steering Knuckles $36.99-52.99'),
 ('Servo', 'Injora High Torque N30 PRO NANO Servo with Servo Saver (JR/ZH1.5)', '$38.99', 'alt: Aluminum N30 NANO Servo with Servo Saver $33.99; Aluminum Servo Saver $4.99 (Pro vs non-Pro fit unconfirmed)'),
 ('Screws / nuts', 'Injora Grade 12.9 Alloy Steel M1.0 / M1.4 / M2.0 Screws Set; M2 Stainless Steel Nuts', '$7.99 (listed sold out when read); $4.99', 'covers M1.0/M1.4/M2 (cage, pivot, hinge and bumper screws are M2 now); M1.6 (rear shock screws) is NOT in that set: buy M1.6 separately; check the set has M2 x 6 / x8 / x10 / x14'),
 ('Front shocks', 'Injora SCX30-08 26 mm (selected); alternatives 26 mm / 23 mm Aluminum Oil Shocks for SCX30', '$17.99 each set', 'SCX30-08 price not read; verify the SCX30-08 body size'),
 ('Receiver / ESC', 'Injora 4CH 2.4 GHz Micro Receiver; MBL32 NANO Brushless ESC; MB100 NANO Brushed ESC 15A; T6M transmitter + micro receiver', '$12.99; $43.99; $19.99; $37.99', 'our plan is the GAC G40 Pro + GM1614; MB100 is brushed so NOT used'),
 ('Transmission / axles (donor parts)', 'Complete Transmission with Brass Skid Plate; Aluminum Transmission Kit (unassembled); Axle Worm Gear Set; Axle Transmission Bearing Set; Stainless Steel Axle Shafts; Aluminum Front & Rear Axle Housings; Stainless Steel Drive Shafts for SCX30 Tacoma', '$29.99; $20.99; $8.99-16.99; $4.99-9.99; $5.99-12.99; $24.99; $14.99', 'only if the donor SCX30 parts are missing; gearbox in the model is the longitudinal metal box supplied by the user'),
 ('Wheels / tyres (0.7 in)', 'Swamp Claw 36 x 14 (T7019, 4 pcs); alternatives 35 x 14 King Trekker (T7014), 37 x 14 MAVERIX M/T (T7035); silicone inserts for 0.7 in 36 mm tyres; 0.7 in D18/D19 wheels (aluminium W7002/W7001 D18, W7008/W7013 D19, brass beadlock W7016)', '$12.99 / 4; inserts $10.99', 'model tyre is about O36 x 13.5, so the 36 x 14 is the closest; check shell clearance'),
 ('Motor (NOT used)', 'Green Viper 1616 4800KV / 1621 4300KV', '$21.99', 'can/inrunner types, not the GM1614 outrunner in the model: not used unless the motor choice changes'),
 ('Rear shocks (selected)', 'Injora 36 mm big-bore shocks (4 + springs per pack; verify SKU, earlier listing SCX24-198)', 'price not read', 'see section C; 2 spare shocks; the 40 mm SCX24-179 ($21.99) does NOT fit'),
]

def write_parts():
    pr = printed_rows()
    with open(REPO + '/parts_list.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['section', 'item', 'qty', 'file_or_bodies', 'material_or_spec', 'plate', 'orientation_or_size', 'notes', 'status'])
        for r in pr:
            w.writerow(['printed', r['name'], r['qty'], r['file'], r['material'], r['plate'], f"down={r['down']}; H {r['height']} mm; layer {r['layer']}", f"{r['orient']} | {r['why']} | supports: {r['support']}, brim {r['brim']}", ('verify before print: ' + r['verify']) if r['verify'] else 'ok'])
        for j, fa, q, dim, bodies, stock, st in HW:
            w.writerow(['hardware', j, q, bodies, fa, '', dim, stock, st])
        for g, item, q, spec, src, st in BUY:
            w.writerow(['purchased:' + g, item, q, '', spec, '', '', src, st])
    L = []
    L.append('# Parts list - SCX30 SCT 107 mm trophy-truck kit (v7)\n')
    L.append('Generated by `print/gen_parts_list.py` from the STL set (stl/v3 + stl/v4) and `print/plates_report.json`; the CSV twin is `parts_list.csv`. Coordinates X forward, Y left, Z up, mm.\n')
    L.append('**Status words.** `ok` = taken from the CAD and consistent. `verify before print` = ASSUMED or UNMEASURED input that the geometry depends on (shock stroke 7 mm, pinion mesh, servo wire exit, motor screw pattern, gearbox "12 deep x 25 wide x 28 tall", servo N30 vs Pro, motor Ø17 x 10): confirm with calipers on the real part before committing the matching printed part.\n')
    L.append('Totals: **%d printed files / %d plates**, %d hardware bodies in %d joint groups, %d purchased lines.\n' % (len(M.PARTS), len(rep['plates']), sum(h[2] for h in HW), len(HW), len(BUY)))
    L.append('## A. Printed parts (%d files, qty 1 each; L/R are separate files)\n' % len(M.PARTS))
    L.append('| Part | File | Plate | Material (default / upgrade) | Why | Print orientation | Supports | Notes / verify |')
    L.append('|---|---|---|---|---|---|---|---|')
    for r in pr:
        L.append(f"| {r['name']} | `{r['file']}` | {r['plate']} | {r['material']} | {r['why']} | {r['orient']} (H {r['height']} mm, flat contact {r['contact']} mm2) | {r['support']}{'' if r['brim']=='0' else ', brim ' + r['brim'] + ' mm'} | {('**verify before print:** ' + r['verify']) if r['verify'] else ''} |")
    L.append('\nSpares: print 2 extra of every part under 100 mm3 (tie rods, rack, bell crank, drag link, horn, retainer plates, bump stops, axle bracket); they are the ones that get lost or snapped during test fits. Spare pins and screws: buy 2x the quantities in section B.\n')
    L.append('## B. Hardware by joint (derived from every `HW_*` body in stl/v3 and stl/v4)\n')
    L.append('Sizes are measured from the STL bodies (bbox + slice widths): "Ø" is the shank / pin diameter, "under head" is the usable length below the head, head size given where the model has one. Where TUB_V4.md / FRONT_V3.md quote a different nominal size it is noted. Self-tapping screws go into printed pilots (no nuts) unless a nut is listed. Obsolete bodies (v3 `HW_Rear_*_Pivot_Nut_*`, `HW_Rear_Shock_Lower_Nut_*`, `HW_Rear_Upper_Link_Axle_*`) are deleted in the working tree and are NOT counted.\n')
    L.append('| Joint | Fastener | Qty | Modelled size (mm) | Bodies | Suggested stock part | Status |')
    L.append('|---|---|---|---|---|---|---|')
    for j, fa, q, dim, bodies, stock, st in HW:
        L.append(f'| {j} | {fa} | {q} | {dim} | `{bodies}` | {stock} | {st} |')
    # buy list by size
    from collections import defaultdict
    L.append('\n**Fastener shopping summary** (add 20 % spares; M1.0-M2 self-tapping or machine screws, pan/button head):\n')
    L.append('| Size | Qty | Used at |\n|---|---|---|')
    L += ['| M2 x 6 pan head | 12 | one-piece cage (L1..L6, R1..R6) |',
          '| Ø1.2 x 3.4 steel pin + CA | 2 | sway-link pins in the trailing-arm clevis |',
          '| M2 x 8 | 2 | shell rear tilt-hinge screws |',
          '| M2 x 12 | 2 | rear trailing-arm pivots |',
          '| M2 x 14 cheese head + M2 nut | 2 + 2 | front bumper through-bolts |',
          '| M1.4 x 3-4 | 1 | Y-brace front screw |',
          '| M1.4 x 6 / x4 / x3 / x2.5 | 2+1 / 2 / 2+2+2 / 2 | upper-link pivots + axle top pin / front shock lower / front shock upper + tie rod inner + retainer plates / tie rod outer |',
          '| M1.6 x 8 | 2 + 2 | rear shock lower (arm cheek) + rear shock upper (cage tab, with 0.8 mm sleeve) |',
          '| M2 x 5 + nut | 2 | trailing arm axle bolt (nut size ASSUMED) |',
          '| M1.0 x 3.5 | 2 | servo ears |', '| M1.0-class pins / screws ~2.5 | 3 | link pin A, crank in, crank out (headless) |',
          '| Ø1.5 mm dowel | ~100 mm total | 2 x 18.4, 2 x 16.8, 4 x 8.0 |', '| Ø1.2 mm wire | ~25 mm total | 2 x 4.7, 2 x 3.9 kingpins, crank pivot 2.8 |']
    L.append('\n## C. Purchased / off-the-shelf (not printed)\n')
    L.append('| Group | Item | Qty | Spec / modelled size | Source | Status |\n|---|---|---|---|---|---|')
    for g, item, q, spec, src, st in BUY:
        L.append(f'| {g} | {item} | {q if q else "-"} | {spec} | {src} | {st} |')

    L.append('\n## D. Candidate purchases (Injora SCX30 collection, read 2026-11-18, USD as listed; check stock; fits are NOT verified)\n')
    L.append('| Need | Candidate | Price | Note |\n|---|---|---|---|')
    for r in CAND: L.append('| ' + ' | '.join(r) + ' |')
    L.append('\n## E. Open items (blocking or risky)\n')
    L += ['1. Everything marked `verify before print` above (calipers on gearbox, motor screw pattern on the gearbox motor face, servo, real Injora shock sizes: rear 36 mm spring Ø and eye width, front SCX30-08 body Ø). ESC (30 x 17 x 6) and RX (20 x 15 x 7) are user-measured and the tub / shelves are resized for them.\n1b. Shocks: the rear Injora 36 mm hangs from the cage tail-post tab (leaning 28.6 deg, spacer sleeve; spring Ø9.5 has 0.1 mm to spare against a Ø10 spring); the front SCX30-08 sits on a bulkhead fork tower with only 0.16 mm lower-eye to wheel-envelope clearance at droop',
          '2. Rear axle fastener: modelled M2 bolt (Ø1.8) with a 2.4 AF hex nut does not describe a real M2 nut (4.0 AF). Measure the stock SCX30 axle-ear screw/nut and update the arm inner-cheek seat.',
          '3. Front bulkhead is fused into `Chassis_Tub` (one print): the former bulkhead floor / wall screws no longer exist.',
          '4. Cage, trailing-arm pivot, shell-hinge and bumper fasteners are M2 in the STLs (x6 / x10 / x8 / x14); check TUB_V4.md and FRONT_V3.md say the same. Driver access to the hinge screws from inboard is untested (the radiator that blocked it is gone).',
          '5. Body shell mounting: no front body posts or clips. Front is held by 2 pairs of Ø3.0 x 2.0 neodymium magnets (tower seats at X 57.3, Y +-7.6 and hood pockets), rear by the tilt hinge on the Cage ears (2 x M2 x 8 screws, `HW_Rear_Hinge_Screw_L/R`). Magnets are not modelled as bodies; holding force and hinge-screw access are untested. `docs/SHELL_CLIPS.md` (snap hooks) is superseded.',
          '5b. Rear travel: stops at -16 / +8 deg of trailing-arm angle, 11.94 mm wheel travel; this needs a rear shock stroke of 28.82..35.15 mm eye-to-eye, NOT verified against the real Injora shock. Rear arm/axle joint is on the axle bolt, link-eye gap 0.30 mm.',
          '6. Sources: the plates and this list are built from the current `stl/v4` + `stl/v3` (assembly frame). `print/parts/*.stl` is a mirror of the printable parts written by `make_plates.py`. Re-run `print/make_plates.py`, `print/verify_plates.py`, this script and `docs/make_diagrams.py` after any geometry change.']
    open(REPO + '/parts_list.md', 'w').write('\n'.join(L) + '\n')

def write_plates():
    bw, bh = rep['bed']
    P = ['# Print plates (Bambu Studio 3MF)\n',
         f"Bed assumed **{bw:.0f} x {bh:.0f} mm** (H3C exact model and bed unconfirmed). Layout is generated by `print/make_plates.py` (`--bed W H` re-packs for a bigger bed). Files: `print/plate_N.3mf` (orientation baked into the mesh, one object per part, per-object settings in `Metadata/model_settings.config`), `print/oriented/*.stl` (the same oriented parts as STL, float32 round-trip checked), `print/plates_report.json`, `print/slicer_estimates.json`, `docs/plate_N.png` (layout previews). All 6 plates were opened with `BambuStudio --info` (manifold = yes, 1 part per object) and sliced headless once (see numbers below).\n",
         '**Source geometry:** the plates are built from the CAD output in `stl/v4` + `stl/v3` (assembly pose); `print/parts/*.stl` is a mirror of exactly the printable files, rewritten by `print/make_plates.py` on every run, and `print/oriented/*.stl` holds the print-oriented copies. Re-run `print/make_plates.py` after any geometry change.\n',
         '## Plate table\n', '| Plate | Contents (qty) | Material | Layer | Walls / infill | Orientation (bed face) | Supports | Slicer estimate* | Heuristic estimate** |', '|---|---|---|---|---|---|---|---|---|']
    contents = {1: 'Lower arm L/R; upper arm L/R; carrier L/R; spindle L/R; retainer plate L/R; rack; bell crank; drag link; tie rod L/R; servo horn; Y-brace (17 parts; the front bulkhead is fused into the chassis tub on plate 4)',
                2: 'Trailing arm L/R; upper link L/R; axle top bracket; front bumper (6 parts)', 3: 'Front bump stop L/R (2 parts, TPU)', 4: 'Chassis tub (1 part)', 5: 'Cage (1 part, one-piece)', 6: 'Body shell (1 part)'}
    ori = {1: 'arms/links/plates flat; spindles X-face down; carriers Y-face down', 2: 'arms side-face down; links flat (boss side up); bracket upside down; bumper oval/plate flat, hoop up',
           3: 'rings flat', 4: 'floor on the bed', 5: 'one-piece Cage printed inverted (roof down), 35.6 mm tall, about 200 mm2 bed contact on the foot bosses: 4 mm brim (6 if it lifts), slow hot first layers', 6: 'sill rim on the bed, roof up'}
    sup = {1: 'arms: normal, plate only (about 15-55 mm2 of overhang each); spindle/carrier: tree; flat parts none', 2: 'tree/normal on the trailing arms (they overhang on the part itself), plate-only on the links; bumper: tree under the hoop arch',
           3: 'none', 4: 'tub: tree (plate preferred) for the deck/tail/arches', 5: 'tree supports for the one-piece cage (mostly under the tail and cross bars)', 6: 'tree supports from the plate fill the cabin and hood underside'}
    WALLS = {1: '4-6 walls / 100 %', 2: '4-6 walls / 100 %', 3: '3 walls / 100 %', 4: '4 walls / 100 %', 5: '4 walls / 100 %', 6: '3 walls / 10 % gyroid'}
    for p in rep['plates']:
        i = p['plate']; s = est[str(i)]; prof = M.PROFILES[p['profile']]
        g = s['filament_mm3'] * M.density(p['profile']) / 1000
        P.append(f"| {i} | {contents[i]} | {prof['material']} | {prof['layer']} mm | {WALLS[i]} | {ori[i]} | {sup[i]} | {s['time']}, {s['filament_mm3']/1000:.1f} cm3 = {g:.1f} g, {s['layers']} layers | {p['hours']:.1f} h, {p['grams']:.1f} g |")
    P.append('\n\\* Bambu Studio 2.x headless slice with the per-object settings of the 3MF and the **default** machine/filament/process profile (PLA speeds, no printer chosen), so time scales with your real printer and filament; filament volume includes supports. Mass = volume x density (PETG-CF 1.30, TPU 1.21, PETG 1.27 g/cm3).\n\\** Volume-based guess from `make_plates.py` (model + estimated support volume at 12 % fill, throughput per layer height). Use the slicer number once the printer profile is chosen.\n')
    P.append('Total with the slicer numbers: **%.1f h, %.0f cm3 filament (about %.0f g)** for the whole kit, shell included (the shell is %.1f h and %.0f cm3 of that, 54 %% of it tree support).\n' % (sum(e['seconds'] for e in est.values()) / 3600, sum(e['filament_mm3'] for e in est.values()) / 1000, sum(est[str(p['plate'])]['filament_mm3'] * M.density(p['profile']) / 1000 for p in rep['plates']), est['6']['seconds'] / 3600, est['6']['filament_mm3'] / 1000))
    P.append('## Per-part verification (after orientation and float32 export)\n')
    P.append('`print/verify_plates.py` reloads every 3MF and oriented STL: watertight, volume > 0, on the bed (z min 0), inside the bed, bounding boxes >= 2 mm apart (packer uses 5 mm). Overhang = area of faces steeper than 45 deg from vertical (excluding the bed face); "on part" = the share whose straight-down ray hits the part itself first, so supports there cannot start on the build plate (those parts use tree supports that may touch the part; the rest are plate-only).\n')
    P.append('| Plate | Part | Bed-contact area (mm2) | Height (mm) | Overhang area (mm2) | of which on the part | Support | Brim | Volume (mm3) |\n|---|---|---|---|---|---|---|---|---|')
    for p in rep['plates']:
        for n in p['parts']:
            m = rep['parts'][n]
            P.append(f"| {p['plate']} | {n} | {m['contact']:.1f} | {m['height']:.1f} | {m['overhang']:.1f} | {m['overhang_on_part']:.1f} | {m['support']} | {m['brim']} | {m['volume']:.1f} |")
    P.append('\nResult: %d/%d parts watertight, in-bed, non-overlapping. Weakest adhesion: `Steering_Drag_Link` (2 mm2 contact, because its pin bosses stand proud): 4 mm brim and a few support dots; `Front_Bump_Stop` rings 10.7 mm2 each (TPU, slow, brim optional); `Cage` stands on its foot bosses (about 200 mm2): 4 mm brim, print slowly for the first layers.\n' % (len(rep['parts']), len(M.PARTS)))
    P.append('## Layout previews\n')
    for p in rep['plates']:
        P.append(f"### plate_{p['plate']}\n![plate {p['plate']}](../docs/plate_{p['plate']}.png)\n")
    P.append('## Orientation overview\n![orientation](../docs/06_print_orientation.png)\n')
    P.append('## Alternatives and notes\n')
    P += ['- **Shell:** sill-down with tree supports (this plate). The supports are heavier than the shell (15 cm3). Nose-down (X+ face on the bed, 142 mm tall) needs almost no support (487 mm2 overhang) but is tall and thin-walled; the headless slicer rejected it with the default machine (height), so it is untested. If the H3C is a small-height model use sill-down.',
          '- **Merge plates on a bigger bed:** `python print/make_plates.py --bed 350 320` keeps one plate per material/layer group and only packs more per plate if the group is split. Plates with different layer heights are never merged (per-object layer height does work in Bambu Studio, see below, but is easier to get wrong).',
          '- **TPU plate:** the two bump stops take 1 minute: print them together with a spare pair.', '- **Spares:** duplicate the small-parts rows in Bambu Studio (right click -> clone) before slicing plate 1.']
    open(HERE + '/PLATES.md', 'w').write('\n'.join(P) + '\n')

if __name__ == '__main__':
    write_parts(); write_plates(); print('wrote parts_list.md, parts_list.csv, print/PLATES.md')
