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
 'IFS_Bulkhead_Convertible': ('PETG-CF (PA12-CF upgrade)', 'Carries both pairs of hinge pins, the servo, the bell-crank pivot and four floor screws: stiffness and creep resistance matter most.'),
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
 'Front_Bumper': ('PETG-CF or PA12-CF (impact)', 'Bull-bar hoop + skid oval in one part: chamfered square 2.6 mm tube, takes front hits; 574 mm3, ties to the bulkhead with 2 bolts. PA12-CF absorbs impacts better than PETG-CF.'),
 'Front_Y_Brace': ('PETG-CF', '1.6 mm plate over the nose, ties the hoop top bar to the nose posts; light load.'),
 'Servo_Horn': ('PA12-CF (or the metal horn from the servo kit)', 'Press fit on a 3.93 mm spline, 1.5 mm thick, takes the full servo torque (4 kgf.cm).'),
 'Servo_Wire_Clip': ('PETG or PLA', 'Strain relief only; use a zip tie through its groove if the 0.85 mm walls are fragile.'),
 'Front_Bump_Stop_L': ('TPU 95A', 'Elastic stop on the shock rod; must compress, not crack.'),
 'Front_Bump_Stop_R': ('TPU 95A', 'As L.'),
 'Rear_Trailing_Arm_L': ('PA12-CF (PETG-CF OK)', 'Rear lower link: carries axle torque reaction, shock load and cornering; three screw bores.'),
 'Rear_Trailing_Arm_R': ('PA12-CF (PETG-CF OK)', 'As L.'),
 'Rear_Upper_Link_L': ('PETG-CF', 'Axially loaded link with a stadium boss carrying three D1.5 pilots (anti-squat hole choice); PETG-CF is stiff enough, PA-CF only if it flexes.'),
 'Rear_Upper_Link_R': ('PETG-CF', 'As L.'),
 'Rear_Axle_Top_Bracket': ('PETG-CF', 'Carries both upper links on one pin; small but 4.3 mm tall and loaded in shear.'),
 'Chassis_Tub': ('PETG-CF (PA12-CF upgrade)', 'Main structure; 1.4-1.8 mm walls, threads cut into it, near the motor/ESC heat.'),
 'ESC_RX_Shelf_L': ('PETG-CF or PETG', '1.0-1.2 mm lid plate over the ESC / RX side pockets (the parts stand upright in the pocket under it); light load.'),
 'ESC_RX_Shelf_R': ('PETG-CF or PETG', 'As L.'),
 'Cage': ('PETG (tough); PETG-CF if you want it stiffer', '2.4 mm chamfered square tubes + two sway-bar saddles: CF makes them stiff but brittle in a crash; plain PETG bends instead of snapping.'),
 'Cabin_Cage': ('PETG (tough); PETG-CF if you want it stiffer', 'Cabin + under-hood roll cage, same chamfered 2.4 mm square tube as the rear cage; bolts to rim bosses and front wall tops with 6 x M1.6.'),
 'Tacoma_Trophy_Body_Shell': ('PETG (PLA also fine)', 'Non-structural; PETG survives rolls, PLA is stiffer/looks better.'),
}
ASSUMED_PARTS = {'Cabin_Cage': 'print orientation untested (feet down chosen: arched roof has almost no flat top); shell clearance 1.68 mm',
                 'Chassis_Tub': 'tub stack raised 5.8 mm (floor top Z +0.9); rear droop stop posts are part of the tub; gearbox 12/25/28 envelope still unconfirmed','Front_Bumper': 'hoop width, oval size and the 2.6 mm tube are estimates from the Yeti brief (no dimensions given); print orientation and supports untested',
                 'Front_Y_Brace': 'rear feet only locate on the deck (0.1 mm gap, no screw); front screw into a D1.2 pilot',
                 'Rear_Upper_Link_L': 'anti-squat numbers assume CG 21.5 mm and wheelbase 116.5 mm (assumed): top hole 22.8 %, middle 54.7 %, lowest 85.9 %', 'Rear_Upper_Link_R': 'as L','Servo_Horn': 'spline Ø3.93 and the bore clearance are datasheet-image estimates: print a test horn first',
                 'Servo_Wire_Clip': 'servo wire exit assumed at the rear face (unmeasured)',
                 'ESC_RX_Shelf_L': 'covers only: the receiver no longer fits under the roof on the shelf (10.2 mm), it goes on the floor of the free bay next to the ESC. Resized for the measured ESC 30x17x6 / RX 20x15x7 (tub Rev 37): pocket Y 10.2..19.2 x X gap 19.1 x 31 high, shelf lid Z 26.1..27.1', 'ESC_RX_Shelf_R': 'as L',
                 'Front_Bump_Stop_L': 'positioned for an ASSUMED 7 mm shock stroke', 'Front_Bump_Stop_R': 'as L',
                 'IFS_Bulkhead_Convertible': 'servo ears/pilots from an image-read datasheet (hole spacing 10.0 is a pixel estimate); servo "Pro" variant unconfirmed',
                 'Cage': 'screws M1.6: STL under-head length is 6.2-6.5 mm, TUB_V4.md says 5 mm; buy M1.6 x 6 and check'}

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
 ('Front: hinge-pin retainer plate', 'screw (self-tapping)', 2, 'shank Ø1.35, head Ø2.4 x 0.7, 2.5 under head', 'HW_Front_Retainer_Screw_L/R', 'M1.4 x 2.5 (or M1.4 x 3 cut)', 'ok'),
 ('Front: upper arm to carrier', 'plain pin', 2, 'Ø1.5 x 8.0', 'HW_Front_Upper_Outer_Pin_L/R', 'Ø1.5 mm dowel cut to 8.0 mm', 'ok'),
 ('Front: lower arm to carrier', 'plain pin', 2, 'Ø1.5 x 8.0', 'HW_Front_Lower_Outer_Pin_L/R', 'Ø1.5 mm dowel cut to 8.0 mm', 'ok'),
 ('Front: kingpin, upper (carrier to spindle, vertical)', 'plain pin, no nut', 2, 'Ø1.2 x 4.7', 'HW_Front_Upper_Kingpin_L/R', 'Ø1.2 mm music wire cut to 4.7 mm, press fit, a dot of CA to retain', 'check retention'),
 ('Front: kingpin, lower (carrier to spindle, vertical)', 'plain pin, no nut', 2, 'Ø1.2 x 3.9', 'HW_Front_Lower_Kingpin_L/R', 'Ø1.2 mm music wire cut to 3.9 mm', 'check retention'),
 ('Front: shock lower (arm)', 'screw (self-tapping)', 2, 'shank Ø1.35, head Ø2.4 x 0.9, 4.0 under head', 'HW_Front_Shock_Lower_Screw_L/R', 'M1.4 x 4', 'ok'),
 ('Front: shock upper (bulkhead fork tower, through wall-eye-wall)', 'screw (self-tapping)', 2, 'shank Ø1.35, head Ø2.4 x 0.9, 3.3 under head (4.2 total)', 'HW_Front_Shock_Upper_Screw_L/R', 'M1.4 x 3 (3.3 modelled; eye 1.4 wide between two 0.8 mm tower walls)', 'verify before print'),
 ('Front: wheel axle nut', 'nut', 2, 'hex 4.2 across corners x 1.6 (shank 2.9)', 'HW_Front_Wheel_Nut_L/R', 'stock SCX30 wheel nut (reuse)', 'ok'),
 ('Front: servo ears (tub floor pocket / bulkhead)', 'screw (self-tapping)', 2, 'shank Ø1.0, head Ø2.0 x 0.6, 3.4 under head', 'HW_Servo_Mount_Screw_L/R', 'M1.0 x 3.5 (M1.0 x 3 per FRONT_V3.md; use the servo kit screws if they fit)', 'verify before print (servo hole spacing 10.0 is a pixel estimate)'),
 ('Front: servo horn centre screw', 'screw', 1, 'shank Ø1.6, head Ø2.3 x 1.0, 1.4 under head', 'HW_Horn_Screw', 'screw supplied with the servo (FRONT_V3.md says ~M1.2 x 3: mismatch with the model)', 'verify before print'),
 ('Steering: horn to drag link (pin A)', 'headed pin / screw', 1, 'shank Ø1.2, head Ø2.0 x 0.4, 2.4 under head', 'HW_Link_Pin_A', 'M1.0-class screw or Ø1.2 pin with a flat head, ~2.5 mm', 'ok'),
 ('Steering: bell crank pivot', 'headed steel pin', 1, 'shank Ø1.2, head Ø2.2 x 0.3, 2.8 under head', 'HW_Crank_Pivot_Pin', 'Ø1.2 steel pin pressed into the Ø1.1 pilot in the floor boss', 'ok'),
 ('Steering: drag link to crank (input)', 'headed pin', 1, 'shank Ø1.0, head Ø1.8 x 0.3, 2.7 under head', 'HW_Crank_In_Pin', 'Ø1.0 pin / M1.0 x 2.5', 'ok'),
 ('Steering: crank to rack (output)', 'headed pin', 1, 'shank Ø1.0, head Ø1.8 x 0.3, 2.7 under head', 'HW_Crank_Out_Pin', 'Ø1.0 pin / M1.0 x 2.5', 'ok'),
 ('Steering: rack guide', 'screw (self-tapping)', 1, 'shank Ø1.1, head Ø1.8 x 0.5, 3.6 under head', 'HW_Rack_Guide_Screw', 'M1.0-1.2 x 3.5 (FRONT_V3.md says M1.4 x 4, model shank is Ø1.1: mismatch)', 'verify before print'),
 ('Steering: tie rod inner (rack end)', 'screw / pin (headless in model)', 2, 'Ø1.4 x 4.0', 'HW_TieRod_Inner_Screw_L/R', 'M1.4 x 4 pan head (head not modelled: check it clears the rod)', 'check'),
 ('Steering: tie rod outer (spindle end)', 'screw / pin (headless in model)', 2, 'Ø1.4 x 3.4', 'HW_TieRod_Outer_Screw_L/R', 'M1.4 x 3.5 pan head', 'check'),
 ('Front: gearbox to worm shaft coupler (gearbox end)', 'sleeve coupler', 1, 'OD 3.5 x 3.5 long, bore Ø3.1', 'HW_Front_Driveshaft_UJ_Gbx', 'stock SCX30 driveshaft yoke/sleeve; seats over the gearbox Ø3 output stub', 'verify before print (gearbox stub height Z 4.1 and X unmeasured)'),
 ('Front: worm shaft coupler (diff end)', 'U-joint / dogbone coupler', 1, 'Ø3.0 x 4.3', 'HW_Front_Driveshaft_UJ_Worm', 'stock SCX30 driveshaft U-joint; worm-end seat is a Ø4.0 pocket in the bulkhead tunnel', 'verify before print'),
 # TUB
 ('Tub: bulkhead to tub floor', 'screw (self-tapping)', 4, 'shank Ø1.4, head Ø2.6 x 0.8, 2.85 under head (3.65 total, was 6.2 before the 5.8 mm raise): through the 1.25 mm bulkhead web seat into the 1.8 mm tub skid', 'HW_Bulkhead_Floor_Screw_1..4', 'M1.6 x 3 (verify)', 'TEST on a print: about 0.5-0.6 mm3 of thread engagement in the skid'),
 ('Tub: bulkhead to tub wall (-Y side, 2x; +Y side, 1x)', 'screw (self-tapping)', 3, 'shank Ø1.6, head Ø3.0 x 1.3 (in a Ø3.3 counterbore), 4.6 under head', 'HW_Bulkhead_Screw_L1, R1, R2', 'M1.6 x 4.5 -> M1.6 x 5 (TUB_V4.md Rev 9 says ~5)', 'ok'),
 ('Tub: ESC / RX shelf lids (countersunk)', 'flat-head screw (self-tapping)', 8, 'head Ø2.6 countersunk (cone R1.3 -> 0.7, 0.6 deep, flush with the 1.0 mm lid), 5.2 total', 'HW_Shelf_Screw_L1..L4, R1..R4', 'M1.6 x 5 countersunk / flat head (verify length)', 'verify before print'),
 ('Front: bumper through-bolts (head up from below, in a Ø2.8 counterbore in the oval)', 'bolt', 2, 'Ø1.4 shank x 15.0 total, head Ø2.6', 'HW_Front_Bumper_Bolt_L/R', 'M1.4 x 14 (passes plate, pad, bulkhead floor, post, top skin, Y-brace ear)', 'verify before print'),
 ('Front: bumper nut (on the Y-brace ear)', 'nut', 2, '2.9 across corners / 2.5 across flats x 1.2', 'HW_Front_Bumper_Nut_L/R', 'M1.4 nut (real M1.4 nut is 2.8 AF: check the modelled 2.5 AF)', 'verify before print'),
 ('Front: Y-brace front lug to the hoop top bar', 'screw (self-tapping)', 1, 'head Ø2.6, 4.0 total, into a Ø1.2 pilot', 'HW_Front_YBrace_Front_Screw', 'M1.4 x 3-4 (FRONT_V3.md says x4)', 'verify before print'),
 ('Motor to gearbox motor face', 'gearbox motor screws (supplied with the gearbox)', 0, 'pattern, size and count UNMEASURED (Motor_Clamp and HW_Motor_Clamp_Screw_1..4 were deleted: the motor mounts directly on the gearbox)', 'gearbox kit screws (motor body modelled Ø17 x 10, axis X, Z 18.1)', 'stock screws from the gearbox kit', 'verify before print'),
 # CAGE
 ('Cage: rim feet (2 per side)', 'screw (self-tapping)', 4, 'shank Ø1.4, head Ø2.6 x 0.95, 6.2 under head (7.2 total)', 'HW_Cage_Screw_L1, L2, R1, R2', 'M1.6 x 6', 'ok (3 M1.6 per side with the tail foot; TUB_V4.md says x5)'),
 ('Cabin cage: rim bosses X 1.2 and X 17.0 and front wall tops X 28.7 (3 per side)', 'screw (self-tapping)', 6, 'shank Ø1.4, head Ø2.6 x 0.9, 6.2 under head (7.2 total)', 'HW_Cage_Screw_L4, L5, L6, R4, R5, R6', 'M1.6 x 6 into pilots', 'ok'),
 ('Cage: tail foot (1 per side, inboard tail, foot link out to the lug screw)', 'screw (self-tapping)', 2, 'shank Ø1.4, head Ø2.6 x 0.9, 6.5 under head (7.4 total; tail post X -35.4)', 'HW_Cage_Screw_L3, R3', 'M1.6 x 6 (6.5 modelled -> x6, check the tip clears)', 'ok'),
 # REAR
 ('Rear: trailing arm to tub pad', 'screw (self-tapping, no nut)', 2, 'shank Ø1.4, head Ø2.7 x 0.9, 8.2 under head', 'HW_Rear_Trailing_Pivot_Screw_L/R', 'M1.4 x 8', 'ok'),
 ('Rear: upper link to tub pad', 'screw (self-tapping into the link eye, no nut)', 2, 'shank Ø1.4, head Ø2.7 x 0.9, 5.8 under head', 'HW_Rear_Upper_Link_Pivot_Screw_L/R', 'M1.4 x 6', 'ok'),
 ('Rear: upper links to axle bracket', 'headed pin / screw', 1, 'shank Ø1.4, head Ø2.6 x 0.5, 6.5 under head, vertical', 'HW_Rear_Axle_Top_Pin', 'M1.4 x 6.5 -> x6 + washer, or Ø1.4 pin with a flat head', 'check'),
 ('Rear: trailing arm to axle ear', 'bolt', 2, 'shank Ø1.8, head Ø2.8 x 1.1, 4.3 under head', 'HW_Rear_Trailing_Axle_Screw_L/R', 'M2 x 4 (TUB_V4.md Rev 34: M2; shank modelled Ø1.8)', 'verify before print (see nut)'),
 ('Rear: trailing arm axle nut', 'hex nut', 2, '2.4 across flats / 2.8 across corners x 1.3', 'HW_Rear_Trailing_Axle_Nut_L/R', 'a real M2 nut is 4.0 AF: modelled nut is M1.2-M1.4 size and will not match the Ø1.8 bolt; measure the stock SCX30 axle-ear fastener and re-check the nut seat on the arm inner cheek', 'ASSUMED: verify before print'),
 ('Rear: shock lower (arm outer cheek)', 'screw (self-tapping, no nut)', 2, 'shank Ø1.44, head Ø2.2 x 0.8, 4.5 under head, 1.8 mm engagement', 'HW_Rear_Shock_Lower_Screw_L/R', 'M1.4-M1.6 x 4.5 (TUB_V4.md Rev 34 says M1.6)', 'verify before print'),
 ('Rear: shock upper (cage tail-post hanger tab, spacer sleeve under the eye)', 'screw (self-tapping into the tab pilot) + spacer sleeve', 2, 'shank Ø1.45 (M1.6 pilot), head Ø2.4 x 0.7, 7.5 under head (8.2 total); sleeve 0.8 long (Y 18.55..19.35)', 'HW_Rear_Shock_Upper_Screw_L/R', 'M1.6 x 8 (verify); sleeve = 0.8 mm spacer (cut 1.7 ID brass tube or the shock bushing)', 'verify before print'),
 ('Rear: wheel nut', 'nut', 2, 'hex 4.2 across corners x 3.8 long', 'HW_Rear_Wheel_Nut_L/R', 'stock SCX30 rear wheel nut (reuse)', 'ok'),
]

# ------------------------------------------------------------------ purchased / off the shelf
# (group, item, qty, spec / modelled size, source, status)
BUY = [
 ('SCX30 donor (retained)', 'Rear axle assembly (housing, worm/ring, bearings, hubs)', 1, 'Ref_Rear_Axle 16.5 x 63.0 x 12.5 mm (axle centre X -64, shifted 5 mm back vs stock layout)', 'Axial SCX30 (AXI-3311 axle set)', 'ok; modelled axle-ear fastener unmeasured'),
 ('SCX30 donor (retained)', 'Rear driveshaft (with U-joints)', 1, 'Ref_Rear_Driveshaft 40.3 x 4.0 x 6.7 mm; 12.55 deg at full droop', 'Axial AXI-4428 long set (BUILD_GUIDE.md) / stock', 'verify before print (length and joint rating)'),
 ('SCX30 donor (retained)', 'Wheels + tyres', 4, 'Ø36.0 x 13.5 mm (Ref_Wheel_*), stock Tacoma tyre is Ø34 in BUILD_GUIDE.md', 'Axial SCX30 / SCT 0.7"', 'verify tyre OD (shell clearance is computed with Ø36)'),
 ('SCX30 donor (retained)', 'Wheel nuts', 4, 'M2 hex, see HW_*_Wheel_Nut', 'stock', 'ok'),
 ('SCX30 donor (retained)', 'Front diff housing + ring gear + input worm', 1, 'Ref_Front_Diff_Ring_Gear (Y +-10.3, ring axis X 48, Z 3.0), Ref_Front_Input_Worm_Gear (12.2 long, Ø3.3, axis Z 5.2..6.85)', 'stock SCX30 front axle diff, REPLACED by Injora planetary set below if you use it', 'worm/ring centre distance 3.85 mm may change with the 2/13T Injora gear: verify'),
 ('Front axle upgrade', 'Injora +4.5 mm Planet Axle set SCX30-34BKFR (diff + stainless double-joint shafts, ~52 deg joints)', 1, '2 x double-joint shaft modelled as CVD_Inner (27.5 x Ø3.4) + CVD_Outer (7.2 x Ø2.0); front gear 2/13T', 'Injora SCX30-34BKFR', 'ok; joint angle margin 15.3 deg at +-2.8 mm rack'),
 ('Front axle upgrade', 'Front hub bearings', 4, '2 per hub: Ø4 OD x 1.6 thick (Front_Bearing_*, Front_Bearing_Inner_*); ID not modelled (Ø2.0 to match the CVD stub)', 'MR-type 2 x 4 x 1.6 (flanged or plain)', 'verify ID and thickness'),
 ('Drivetrain', 'Longitudinal metal gearbox ("Modrax"/B0HMR5D54N class)', 1, 'modelled envelope 31 x 25 x 28 (X x Y x Z), output stub Ø3 at Z 4.1; drawing says 26 x 23 x 28 and "12 deep x 25 wide x 28 tall" whose meaning is UNCONFIRMED', 'user supplied', 'ASSUMED: verify before print'),
 ('Drivetrain', 'Pinion 0.3 mod, 11T-15T (steel) matching the motor shaft', 1, 'mesh with the gearbox input gear not measured', 'GAC / Injora', 'ASSUMED pinion mesh: verify before print'),
 ('Drivetrain', 'Front driveshaft centre shaft', 1, 'Ø2.2 x 23.6 mm (Ref_Front_Center_Driveshaft, 6.7 deg between U-joints)', 'cut from Ø2.2 steel rod or stock SCX30 shaft', 'ok'),
 ('Power', 'GAC GM1614 outrunner motor (3500/7500 KV, 14-pole, 2S-3S)', 1, 'modelled Ø17 x 10 mm, axis X at Z 18.1 (GAC page gives no body size); PH2.0 3P lead', 'GACRacing GM1614 + G40 Pro set', 'ASSUMED size and motor screw pattern: verify before print'),
 ('Power', 'ESC: GAC G40 Pro nano (FOC, built-in BEC 6 / 7.4 V, Bluetooth)', 1, 'user-measured 30 L x 17 D x 6 H mm (same setup runs in other SCX30s); the model still has the old 26 x 18 x 8 placeholder until the shelves are resized', 'GACRacing', 'size measured by the user (CAD placeholder pending)'),
 ('Power', 'Receiver (3-ch micro, e.g. Flysky GT5-class decased)', 1, 'user-measured 20 L x 15 D x 7 H mm', 'user choice', 'size measured by the user (CAD placeholder pending)'),
 ('Power', 'Battery 2S/3S LiPo 44.5 x 20 x 10.5 mm', 1, 'user supplied, NOT modelled; cradle is the cage rungs (X -71 / -45, Z 16.6-16.9, tops 0.05-0.35 under the pack at Z 18.1)', 'user supplied', 'size per user; connector/wiring not modelled'),
 ('Steering', 'Servo Injora N30 class (nano), base 21.45 x 15.2 x 12.85; modelled case 15.8 x 9.9 x 20.1 + 12.85 ear flange, spline Ø3.93', 1, 'N30 vs N30 Pro unconfirmed; ear hole spacing 10.0 / wire exit (rear face) UNMEASURED', 'Injora N30 / N30 Pro', 'ASSUMED: verify before print'),
 ('Suspension', 'Front oil shocks: Injora SCX30-08 (alu body, nylon lower end; 26 mm extended / 18.3 compressed, 7.7 mm travel, 1.0 g; modelled body Ø8 incl. spring = ESTIMATE, eye 1.4 wide, bore Ø1.7)', 2, 'ride 21.63 mm eye-to-eye, bump-stop contact 20.07, droop 23.09; uses 3.0 of 7.7 mm. Upper eye on a two-wall fork tower on the bulkhead top. Lower eye clearance to the wheel envelope is only 0.16-0.5 mm (arm lug may move 1 mm inboard). Not an off-the-shelf fit yet: verify the real body diameter', 'Injora SCX30-08', 'selected (modelled); verify body size and price'),
 ('Suspension', 'Rear oil shocks: Injora 36 mm big-bore (eye-to-eye 36, full 42 mm, nylon lower end, brass eye bushings; 7-9 mm travel ASSUMED; modelled body Ø8, spring Ø9.5, eyes 3.0 wide, bore Ø1.7). Earlier listing: SCX24-198 (verify SKU)', 2, 'mounted on the cage tail-post hanger tab, leaning 28.6 deg forward, with a spacer sleeve under the upper eye; eye-to-eye 34.48 / 31.05 / 29.18 mm at droop / ride / bump. A Ø10 spring breaks the 1.5 mm shell margin by 0.1 mm; spring Ø9.5 fits only thanks to the 2 deg Y tilt and the cage tail moved inboard. Spares: a pack has 2 more shocks', 'Injora 36 mm big bore (pack of 4 + springs; oil and hardware not included)', 'selected (modelled); verify real spring Ø, eye width and travel before print'),
 ('Suspension', 'Rear shock NOT used: Injora 40 mm SCX24-179 (needs the upper pin at roof-rail height: shell clearance fails) and 52 mm AX24-06 (upper pin would sit over the ESC shelf, only 4 of 12 mm used)', 0, 'fit-checked in TUB_V4.md Rev 38', 'Injora SCX24-179 / AX24-06', 'rejected'),
 ('Suspension', 'Rear shock hardware of the LOS-07 class, for reference (if the Injora 36 mm kit lacks it): M1.4 x 11.5 top screw + M1.4 nut + 6.7 mm brass bushing + 2 o-rings; M1.4 x 8 bottom screw + brass bushing', 0, 'reference only; the model uses M1.6 self-tapping screws into printed pilots', 'Injora LOS-07', 'reference'),
 ('Suspension', 'Rear sway bar cross wire: steel Ø0.8 x 39.5 mm (straight cross wire; the bent lever wires are listed separately)', 1, 'runs through the D1.4 bores of the two printed chamfered-square saddles (3.6 x 3.6 x 2.8 mm) on the cage hoop legs; retained by the lever elbows, about 1.6 mm axial play, no clip or screw', 'steel / music wire Ø0.8', 'ok (bore D1.4 is a print size: ream or test with the wire)'),
 ('Suspension', 'Rear sway bar lever arms: bent Ø0.8 music wire, elbow ends retain the cross wire (Ref_Rear_Sway_Bar 23 x 39.5 mm shape)', 2, 'about 1.6 mm axial play on the cross wire', 'music wire Ø0.8', 'verify the bend before print'),
 ('Suspension', 'Sway-bar drop links Ø0.93 x 17.3 (wire or 2 mm rod ends)', 2, 'Ref_Sway_Drop_Link_L/R', 'music wire', 'ok'),
 ('Wiring & consumables', 'Motor/ESC/battery leads, servo extension, heat shrink, zip ties (Ø1.4 groove in the wire clip)', 1, 'lead length not modelled', 'misc', '-'),
 ('Wiring & consumables', 'Threadlock (low strength), CA glue, grease for gears, 1.5 mm and 1.2 mm drill bits / reamers', 1, 'for press fits and reaming printed bores', 'misc', '-'),
 ('Wiring & consumables', 'Body mounting: printed PETG clip-on hooks (DECIDED, NOT MODELLED yet; spec in docs/SHELL_CLIPS.md)', 0, '4 snap hooks on the shell underside + optional M1.4 x 5 rear screw; add to the shell generator before final print', '-', 'clip-on decided; hooks not modelled'),
]


CAND = [
 ('Front diff + double-joint shafts (used)', 'Injora +4.5 mm Planet Axles with Double Joint Shafts (SCX30-34BKFR)', '$41.99-59.99', 'alternative: +3 mm Planet Axles with Brass Steering Knuckles $36.99-52.99'),
 ('Servo', 'Injora High Torque N30 PRO NANO Servo with Servo Saver (JR/ZH1.5)', '$38.99', 'alt: Aluminum N30 NANO Servo with Servo Saver $33.99; Aluminum Servo Saver $4.99 (Pro vs non-Pro fit unconfirmed)'),
 ('Screws / nuts', 'Injora Grade 12.9 Alloy Steel M1.0 / M1.4 / M2.0 Screws Set; M2 Stainless Steel Nuts', '$7.99 (listed sold out when read); $4.99', 'covers M1.0/M1.4/M2; M1.6 is NOT in that set: buy M1.6 separately'),
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
    L.append('Totals: **33 printed files / 6 plates**, %d hardware bodies in %d joint groups, %d purchased lines.\n' % (sum(h[2] for h in HW), len(HW), len(BUY)))
    L.append('## A. Printed parts (33 files, qty 1 each; L/R are separate files)\n')
    L.append('| Part | File | Plate | Material (default / upgrade) | Why | Print orientation | Supports | Notes / verify |')
    L.append('|---|---|---|---|---|---|---|---|')
    for r in pr:
        L.append(f"| {r['name']} | `{r['file']}` | {r['plate']} | {r['material']} | {r['why']} | {r['orient']} (H {r['height']} mm, flat contact {r['contact']} mm2) | {r['support']}{'' if r['brim']=='0' else ', brim ' + r['brim'] + ' mm'} | {('**verify before print:** ' + r['verify']) if r['verify'] else ''} |")
    L.append('\nSpares: print 2 extra of every part under 100 mm3 (tie rods, rack, bell crank, drag link, horn, wire clip, retainer plates, bump stops, axle bracket); they are the ones that get lost or snapped during test fits. Spare pins and screws: buy 2x the quantities in section B.\n')
    L.append('## B. Hardware by joint (derived from every `HW_*` body in stl/v3 and stl/v4)\n')
    L.append('Sizes are measured from the STL bodies (bbox + slice widths): "Ø" is the shank / pin diameter, "under head" is the usable length below the head, head size given where the model has one. Where TUB_V4.md / FRONT_V3.md quote a different nominal size it is noted. Self-tapping screws go into printed pilots (no nuts) unless a nut is listed. Obsolete bodies (v3 `HW_Rear_*_Pivot_Nut_*`, `HW_Rear_Shock_Lower_Nut_*`, `HW_Rear_Upper_Link_Axle_*`) are deleted in the working tree and are NOT counted.\n')
    L.append('| Joint | Fastener | Qty | Modelled size (mm) | Bodies | Suggested stock part | Status |')
    L.append('|---|---|---|---|---|---|---|')
    for j, fa, q, dim, bodies, stock, st in HW:
        L.append(f'| {j} | {fa} | {q} | {dim} | `{bodies}` | {stock} | {st} |')
    # buy list by size
    from collections import defaultdict
    L.append('\n**Fastener shopping summary** (add 20 % spares; M1.0-M1.6 self-tapping or machine screws, pan/button head):\n')
    L.append('| Size | Qty | Used at |\n|---|---|---|')
    L += ['| M1.6 x 6 | 12 | cage x6 (rim x4, tail x2), cabin cage x6 |',
          '| M1.6 x 5 countersunk | 8 | ESC/RX shelf lids |',
          '| M1.6 x 5 | 3 | bulkhead wall screws |',
          '| M1.4 x 14 + M1.4 nut | 2 + 2 | front bumper through-bolts |',
          '| M1.4 x 3-4 | 1 | Y-brace front screw |',
          '| M1.4 x 8 / x6 / x5.5-6 / x4 / x3 / x2.5 | 2 / 2+1 / 2 / 4 / 4 / 2 | rear trailing pivots / upper-link pivots + axle top pin / rear shock upper / front shock lower + tie rod inner / front shock upper + tie rod outer / retainer plates |',
          '| M1.4-M1.6 x 4.5 | 2 | rear shock lower (arm cheek) |',
          '| M2 x 4 + nut | 2 | trailing arm axle bolt (nut size ASSUMED) |',
          '| M1.0 x 3.5 | 2 | servo ears |', '| M1.0-class pins / screws ~2.5 | 3 | link pin A, crank in, crank out |',
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
          '3. Bulkhead floor screws: now 2.85 mm under head (M1.6 x 3) into the 1.8 mm tub skid with only about 0.5 mm3 of thread; coupon first (print a 20 x 20 mm piece of the skid at the same settings), fall back to nuts from below or glue.',
          '4. Cage screws are M1.6 x 6 by the STL (6.2-6.5 mm under head), TUB_V4.md says x5.',
          '5. Body shell mounting: clip-on decided, hooks not modelled yet (docs/SHELL_CLIPS.md).',
          '6. Sources: the plates and this list are built from the current `stl/v4` + `stl/v3` (assembly frame). `print/parts/*.stl` is a mirror of the 32 printable parts written by `make_plates.py`. Re-run `print/make_plates.py`, `print/verify_plates.py`, this script and `docs/make_diagrams.py` after any geometry change.']
    open(REPO + '/parts_list.md', 'w').write('\n'.join(L) + '\n')

def write_plates():
    bw, bh = rep['bed']
    P = ['# Print plates (Bambu Studio 3MF)\n',
         f"Bed assumed **{bw:.0f} x {bh:.0f} mm** (H3C exact model and bed unconfirmed). Layout is generated by `print/make_plates.py` (`--bed W H` re-packs for a bigger bed). Files: `print/plate_N.3mf` (orientation baked into the mesh, one object per part, per-object settings in `Metadata/model_settings.config`), `print/oriented/*.stl` (the same oriented parts as STL, float32 round-trip checked), `print/plates_report.json`, `print/slicer_estimates.json`, `docs/plate_N.png` (layout previews). All 6 plates were opened with `BambuStudio --info` (manifold = yes, 1 part per object) and sliced headless once (see numbers below).\n",
         '**Source geometry:** the plates are built from the CAD output in `stl/v4` + `stl/v3` (assembly pose); `print/parts/*.stl` is a mirror of exactly those 33 printable files, rewritten by `print/make_plates.py` on every run, and `print/oriented/*.stl` holds the print-oriented copies. Re-run `print/make_plates.py` after any geometry change.\n',
         '## Plate table\n', '| Plate | Contents (qty) | Material | Layer | Walls / infill | Orientation (bed face) | Supports | Slicer estimate* | Heuristic estimate** |', '|---|---|---|---|---|---|---|---|---|']
    contents = {1: 'Bulkhead; lower arm L/R; upper arm L/R; carrier L/R; spindle L/R; retainer plate L/R; rack; bell crank; drag link; tie rod L/R; servo horn; Y-brace (18 parts)',
                2: 'Trailing arm L/R; upper link L/R; axle top bracket; front bumper; servo wire clip (7 parts)', 3: 'Front bump stop L/R (2 parts, TPU)', 4: 'Chassis tub; ESC/RX shelf L/R (3 parts)', 5: 'Cage; Cabin cage (2 parts)', 6: 'Body shell (1 part)'}
    ori = {1: 'arms/links/plates flat; bulkhead rear face down (nose up); spindles X-face down; carriers Y-face down', 2: 'arms side-face down; links flat (boss side up); bracket upside down; bumper oval/plate flat, hoop up',
           3: 'rings flat', 4: 'floor on the bed; shelves flat', 5: 'Cage inverted: roof arch tube on the bed, rails 4.6 mm above it (supports), legs up; Cabin cage feet down, tree supports under the hood rails', 6: 'sill rim on the bed, roof up'}
    sup = {1: 'bulkhead: tree, may touch the part; arms: normal, plate only (about 15-55 mm2 of overhang each); spindle/carrier: tree; flat parts none', 2: 'tree/normal on the trailing arms (they overhang on the part itself), plate-only on the links; bumper: tree under the hoop arch',
           3: 'none', 4: 'tub: tree (plate preferred) for the deck/tail/arches, shelves none', 5: 'plate supports under the cage roof rails (they hang 4.6 mm above the bed), tail frame, rungs and feet; cabin cage: tree from the plate under hood rails and the A-pillar slope', 6: 'tree supports from the plate fill the cabin and hood underside'}
    WALLS = {1: '4-6 walls / 100 %', 2: '6 walls / 100 %', 3: '3 walls / 100 %', 4: '4 walls / 100 %', 5: '4 walls / 100 %', 6: '3 walls / 10 % gyroid'}
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
    P.append('\nResult: 33/33 parts watertight, in-bed, non-overlapping. Weakest adhesion: `Steering_Drag_Link` (2 mm2 contact, because its pin bosses stand proud): 4 mm brim and a few support dots; `Front_Bump_Stop` rings 10.7 mm2 each (TPU, slow, brim optional); `Cage` 22 mm2 of line contact: 6 mm brim, print slowly for the first layers.\n')
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
