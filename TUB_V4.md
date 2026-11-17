# TUB v4: one printed tub, rear links on the tub tail (stl/v4, `scripts/18_tub.py`)

Run: `BODY_DIR=<per-body stl dir> python scripts/18_tub.py` (about 3 min). Reads stl/v3, writes only stl/v4. Every dimension below is a named constant at the top of the script.
Axes X forward, Y left, Z up, mm. All 36 exports are watertight single shells after an STL roundtrip.

## What changed vs v3
| v3 | v4 |
|---|---|
| Chassis_Rail_L/R + Slider_L/R | **Chassis_Tub** (one part, 100.4 x 35.8 x 24.8, 17.5 cm³) |
| rail-mounted pivots | pivot pads in the tub tail wall at X -26 |
| Rear_Trailing_Arm / Rear_Upper_Link (51 / 43 mm) | rebuilt, **33.0 / 33.0 mm**, same axle end, shock clevis and sway-bar tab |
| Rear_Sway_Bridge | **removed in v4**: replaced by `Rear_Cage_Tail` (separate printed hoop that screws to the tub deck) |

## Tub
* Floor top Z -4.9 (the gearbox base, unchanged), floor 2.6 thick, **skid line Z -7.5** (-8.5 under the ESC pocket). Outer half width 17.9 (arms start at Y 18.0, shell inner face is Y 22.3). Rim Z 8.0, so the cage's lowest tubes (Z 8.1) sit on it.
* Gearbox zone X -12..36: walls Y 13.5..17.9. Tail X -46..-12: walls 1.5 thick (Y 16.4..17.9), raised to the deck behind the motor.
* **Gearbox screws to the floor top**: 4 x M1.4 pilots Ø1.2 x 2.2 deep at (±10, ±10). **GUESS: foot pattern unknown.** Edit `FOOT_HOLES`. Motor sits over the tail, rear side, on `MOTOR_*` (Ø16.4 x 14.5, centre Z 18.1, 1.9 above the rim).
* **Front/bulkhead interface**: bulkhead stays between the walls at X 24..36. Pilots Ø1.2 through each wall at (X 27, Z 2.5) and (X 33, Z 5.0) (read from the v3 rail holes). The tub is notched 0.1 around the bulkhead (47 mm³ removed); the bulkhead is untouched.
* **Servo bay** is `SERVO_BAY` = X 12..36, Y 2..13.45, Z -4.9..11.1. It is only a named box (the tub is open there, no walls/pockets). The front servo is still in the IFS cradle. Resize in the script, then cut or add walls as needed.
* Gearbox/ESC cavity is open at the top (no lid). Wire slots: 2 deck slots (2.4 x 1.4), one 3 x 6 deck slot behind the motor, and a 3 mm notch in each bay lip.

## Electronics (all sizes ASSUMED, constants `ESC_*`, `RX_*`, `BATT_*`)
| item | size L x W x H | where | status |
|---|---|---|---|
| ESC G40 PRO nano | 24 (Y) x 18 (X) x 8 (assumed 26 x 18 x 8; 26 does not fit between the cage feet) | floor pocket under the motor, X -30.8..-12.4, top Z 1.5 | fits; 0.32 mm³ touch with the rear driveshaft reference (tight, keep ESC ≤ 8 mm tall) |
| Receiver | 22 (X) x 14 (Y) x 7 | rear deck, Y 1.8..16.2, X -28..-50.4 | fits on the deck |
| **2S LiPo** | **50 x 17 x 10 does NOT fit anywhere.** Bay sized 29 x 17 x 10 | rear deck, Y -17..0.8, X -28..-57.4, Z 13.3..23.3 | see "Battery problem" |

### Battery problem (needs a decision)
I searched the whole assembly (cage, shell, gearbox, motor, bulkhead, axle sweep, bridge, bar, shocks) with FCL for a free 50.8 x 17.8 x 10.8 box in X orientation, and also 45, 40 and 35 long: **none exists**. The only free 50 mm slot (across Y at X 16..34, Z 21..32) is outside the shell. Longest battery that fits the rear deck is **about 29 mm** (limited by the motor at X -26.4 and the bridge at X -58.5). Options: a ≤ 29 mm 2S pack, or 50 mm pack in a side pod (needs the cage rocker rerouted outboard and a shell bump), or move the spare carrier and cut the cage bed hoop for a longer deck (not done).
The deck is a cantilever past X -46 (13 mm, over the axle): print it with supports or add a rib. Pack bottom Z 13.3 clears the axle housing top at +7° (≈ 11).

## Rear 4-link on the tub tail
Pivots (X, Z): trailing arm **PL (-26.0, -0.5)**, upper link **PU (-26.0, 4.5)**. Axle side unchanged: A (-59, -0.5), B (-59, 5.25). Pivots are M1.4 screws tapped into pads inside the tail wall (pilot Ø1.25, 3 mm deep from outside), no nuts. Arm eye boss R 3.4, Ø1.6 bore. Pads: Y 14.4..16.4 behind the wall; upper-link boss 2.7 thick against the pad (Y 11.7..14.4). Env overrides: `PL_X`, `PU_X`.

Travel is in **arm angle ta** (+ = bump). Because the arm is 33 mm (v3: 51), the same angle moves the axle less; the range needed for ≥ 8 mm droop / ≥ 4 mm bump is **-14° .. +7°** (v3 was -10..+6).

| ta | axle dX | axle dZ | axle pitch | link angle | shock eye-eye | driveshaft (L, angle*) |
|---|---|---|---|---|---|---|
| -14 | +0.98 | **-7.98** | -1.85° | -13.97 | 30.95 | 31.54, 15.1° |
| -10 | +0.50 | -5.73 | -1.31° | -9.98 | 30.05 | 31.48, 11.1° |
| -5 | +0.13 | -2.88 | -0.65° | -5.00 | 28.88 | 31.40, 6.1° |
| 0 | 0 | 0 | 0 | 0 | 27.66 | 31.31, 1.1° |
| +4 | +0.08 | +2.30 | +0.52° | +4.00 | 26.64 | 31.22, -2.9° |
| +7 | +0.25 | **+4.02** | +0.92° | +7.01 | 25.86 | 31.16, -5.9° |
| +9 | +0.41 | +5.16 | +1.18° | +9.01 | 25.33 | 31.11, -8.0° |

* Link lengths: **arm 33.00, upper link 33.01** (a near-perfect parallelogram, so pitch stays ≤ 1.85°, well under 3°). Axle path is nearly vertical (dX ≤ 1 mm).
* **Anti-squat**: instant centre at (X +194, Z -0.5), far ahead of the axle and at axle height: about **18 %** (assumed CG Z 5.0, wheelbase 107). Mild, no pro-squat/extreme. Raise PU_Z for more.
* Driveshaft (stub X -15.5, Z 4.1 → axle input (-46.8, 3.5)): length change only **0.4 mm** over the whole range, angle 15° at full droop, 6° at full bump.
* Shock (SCX24 32 mm): **25.9..31.0 mm** over -14..+7, inside 22..32. Motion ratio is lower than v3 (clevis is 18 mm from the pivot): `14_shock_slot.py` was **not** re-run, the v3 clevis (PX -44, PZ 0.5, YC) is kept inside the rebuilt arm. If a softer feel is wanted, move the clevis rearward (PX -50 gives ~0.7 ratio) and re-cut.
* 2D solve only (rotation axes along Y), same assumption as v3. `*` driveshaft angle is its total tilt, not the U-joint angle difference.

## Verification (rest pose + sweep -14, -10, 0, +7, FCL + exact volumes)
Gearbox, motor, ESC, receiver and battery boxes vs everything: **no overlap**, except the front session's `Ref_Front_Center_Driveshaft` vs `Ref_Gearbox` (37 mm³, not mine) and `Ref_ESC_G40` vs `Ref_Rear_Driveshaft` (0.32 mm³).
Rear sweep: arms, links, axle, wheels, axle hardware vs tub, cage, shell, deck, bridge, bar, shocks: **no contact over -14..+7** except known artifacts of the 2D model (axle-HW screw vs arm/link bores, 1–5 mm³, same as v3) and the sway drop-link tip in the arm tab (0.4 mm³ intentional insertion).
Intentional/inherited at rest: pivot and bridge screws in their pilots (0.3–0.9 mm³ each), bar/drop-link joints (0.16), and **arm vs axle end (now cut clear, see Rev 5) (inherited from v3, still open)**.

## Cage, shell (the weak part)
* **Cage**: rear bed lattice crosses the new deck, receiver and battery, so those volumes (+0.3 mm) are subtracted: **1066 mm³ removed (23 % of the cage)**, 4 stub fragments (≤ 5.4 mm³) dropped, result is one shell. Four cage feet that used to land on the v3 rails inside the tail (X -44.5 and -27.9, Y ±12.5..14.7) get printed columns from the floor. **The cage needs a real rework in the bed (rear hoop/cross tubes); this is a cut, not a redesign.**
* **Shell**: 7 mm³ relief only (tub/deck). Stl/v4 shell is a regenerated copy of v3: **the orchestrator must re-run `17_hood_scoop.py` on stl/v4 (or v3's scooped shell was used?)**: v4 copies the *current* stl/v3 shell file, so scoop state follows whatever v3 had when I ran.
* Sway bar and drop links: v3 files copied unchanged. Link tilt with the new arm (tab radius 20.75 from the pivot) was **not re-solved**: at -14° it will exceed v3's 6.2°. Needs a check when the bar is next touched.

## Print notes
Tub: floor flat on the bed. Rear deck overhangs (cantilever X -46..-64.5) and the raised tail walls need supports under the deck; the rear cage hoop sits on the deck pads. Walls 1.4–1.5, floor 2.6 (1.6 under the ESC). Arms/links: print on their side, PETG-CF/PA-CF, 100 % infill.

## Parts in stl/v4
Chassis_Tub, Rear_Trailing_Arm_L/R, Rear_Upper_Link_L/R, HW_Rear_Trailing_Pivot_Screw_L/R, HW_Rear_Upper_Link_Pivot_Screw_L/R HW_Rear_Cage_Screw_L/R, Rear_Sway_Bridge (2 screw holes), Roll_Cage, Tacoma_Trophy_Body_Shell, Ref_Gearbox, Ref_Motor_GM1614, Ref_ESC_G40, and unchanged copies of the shock refs/screws, axle-end HW, sway bar, drop links and rear driveshaft.
Removed vs v3: Chassis_Rail_L/R, Slider_L/R, HW_Rear_Trailing_Pivot_Nut_L/R, HW_Rear_Upper_Link_Pivot_Nut_L/R.


---
# Rev 2 (rear cage, shared axle bracket, Modrax feet, servo envelope)
All 39 exports watertight single shells after roundtrip. Run time ~3 min. Reference photos used for style only.

## Changes
* **Gearbox mounting (Modrax photo)**: trans separates from the skid and screws straight to the tub. The photo shows **2 feet on the case bottom, diagonal (one per case half)**: `FOOT_HOLES` = (-10,-11) and (+10,+11), M1.4 from below through Ø1.7 holes with a Ø3.2 x 1.0 counterbore in the skid face. Positions are read from the photo, not measured: verify against the real case.
* **Servo** (front session envelope, case X 13.4..33.5 Y 1.8..11.7, ears to Y 14.65, pilot wall X 33.65..36.1): `Ref_Servo_Envelope` exported; tub has the ear pocket (+0.4) and a pilot block X 33.65..36.1, Y 11.85..13.6. No overlap with the tub in the sweep. No pilot holes yet (screw axis/heights unknown). I kept the servo where the front session put it.
* **Rear cage (tail hoop)**, `Rear_Cage_Tail`, printed separately, screws to the deck: two legs at X -61, Y ±15.5 (R 1.15 = Roll_Cage tube), top bar at Z deck+9.5, a sway-bar cross member, centre saddle, bar collars on the legs, foot pads with 1 x M1.4 each (`HW_Rear_Cage_Screw_L/R`, pilots in the deck). **It replaces `Rear_Sway_Bridge`** (not exported in v4). No forward stays in Rev 2 (added in Rev 8); stiffness comes from the pads and deck. Cross braces and a rear bumper bar from the photos are not added.
* **Sway bar**: wire Ø0.8 through the collars at X -60, Z 20.5 (Y ±19.75), levers to X -46.75, drop links Ø0.9 **down to the existing trailing-arm tabs** (works: the tab already moves with the arm; tab needs the Ø1.0 x 1.0 blind hole, unchanged request). Link length 16.40 constant. **Tilt from vertical: -1.9° (-14°), -0.1° (-10°), 0.7° (-5°), 0° (0), -1.1° (+3°), -2.7° (+6°), -3.3° (+7°)** (max 3.3°, limit 10°). Lever angle +23° at -14°, -10.7° at +7°. Lever touches the shell by 0.03-0.08 mm³ at droop (not cut: relief with the lever sweep broke the shell mesh). Clears tub, deck lips, cage, spare, wheels and axle in the sweep.

## Upper links: shared axle-top bracket (per photo 13)
* Corner pivots: trailing arm outer/low at PL (-26.0, -0.5), upper link inner/high at PU (-24.53, 7.0), both on the tail wall pads (stacked in the corner block). Upper links are **diagonal blades**, pivot boss (Y ±13.05, axis Y, tapped screw from outside) to **one bracket on the axle** (`Rear_Axle_Top_Bracket` + `HW_Rear_Axle_Top_Pin`, vertical Ø1.4 pin at X -57.5, Y 0, eyes stacked 1.1 thick with a 0.3 gap, L link above R). Bracket sits on the pumpkin (Z 7.0), BP = (-57.5, 8.5).
* Why shared bracket: it locates the axle laterally (no panhard) and PU/BP were free to choose; the XZ kinematics (bounce) are identical to the two-pin case because Y does not change in bounce. Arm 33.00 / link 33.00 mm.
* Numbers (-14..+7): axle dZ -7.98/+4.02, dX ≤ 0.98, **pitch -2.43° (droop) .. +1.15° (bump)** (v3-like layout was 1.85°; <3° OK), link 2D angle follows arm, **shock 25.9..31.0 mm**, driveshaft 31.1..31.6 mm long (0.4 mm change), angle 14.8° at droop.
* **Anti-squat ~23 %** (instant centre X +140, Z -0.5; assumes CG Z 5, wheelbase 107). Pitch trend: nose-down on droop (anti-lift small).
* **Lateral / cross-articulation (analytic estimate, no 3D solve)**: in pure bounce the axle has **0 lateral motion**. At full cross-articulation (one side +7°, other -14°) the axle rolls ~17°, the bracket 9 mm above the roll axis shifts ≈ 2.6 mm sideways and the two diagonal links change 3D length by about ±1 mm: this must be absorbed by **ball/compliant ends** (a rigid revolute at both ends will bind). Roll-steer from this is ±~1 mm of axle yaw at the extreme. Wheels (inner face Y 29) and the tub (Y 17.9) have > 8 mm margin laterally.
* Links clear the driveshaft, tub, tail legs, bar and electronics in the sweep; axle-top pin vs R link 0.013 mm³ at -14° only (contact).

## Verification rev 2
Rest overlaps > 0.01 mm³ are only: pivot/cage screws in their pilots (0.2-0.7), bar/link joints (0.3), upper link boss on pad (1.0, intentional seat), tub vs hoop foot pads/legs (2.8, seat), link/link eye (0.38, from the hull, to trim), ESC vs driveshaft (0.32), arm vs axle end (11.2, inherited), front centre driveshaft vs gearbox (37, front session). Sweep (-14, -10, 0, +7): nothing new beyond axle-screw bore artifacts and the two lever/shell scratches above. Gearbox, motor, ESC, receiver, battery boxes: no overlap except the two noted.
Cage: further relief for the hoop, bar and link sweeps: **1264 mm³ removed in total (28 %)**; fragments dropped: the old rear lower ring (239 mm³ at X -63..-49, Z 11..14) and one cross tube (19 mm³). The cage is now one shell but the rear section is a stub; the new hoop is the intended replacement.

## Not done (from the later reference photos)
Side pods / electronics trays beside the gearbox (no room: shell inner Y 22.3, cage rocker at Y 18.9; use the vertical standoff tray), hex-rib floor, sloped nose/rear ends, ribbed skid, tall tapered perimeter. Say which to add next; none changes the numbers above.

---
# Rev 3 (corner blocks, cage braces, nose/tail, hex ribs)
All 39 parts watertight single shells; same checks re-run on the full set (front arms, spindles, rack, horn, bulkhead added to the overlap set).
* **Corner link blocks**: solid block per side on the tail wall, X -30.2..-20.0, Y 14.4..16.4 (behind the 1.5 wall), floor to Z 9.6: trailing pivot low/outer (-26.0, -0.5), upper pivot stacked above (-24.53, 7.0). Constants `PL`, `PU`, `LUG_X`, `PAD_Y0`.
* **Rear cage**: added an upper brace from each collar to the top-bar centre (above the sway wire), a lower cross bar on the deck, two rear stays to a **rear bumper bar** at X -66.5, Z deck+4 (`BUMP_X`, `BUMP_Z`). Sway wire still clears all braces.
* **Nose and tail**: skid nose under the bulkhead, X 36..52, Y ±13.4, kick-up over the last 7 mm (`NOSE_*`), no overlap with the front arms, rack or horn. Tail floor kicks up 0.9 mm over 7 mm (`TAIL_KICK_*`, floor stays ≥ 1.7).
* **Hex ribs**: triangular/hex web, 6 mm pitch, 1.0 wide x 1.5 tall, on the tail floor (X -46..-31.5) and the front -Y bay (X 12.5..23.2). Not under the gearbox/ESC/servo. They add stiffness and screw-boss area only; the floor under them is unchanged (2.6 mm).
* **Overlaps**: same list as rev 2; no new tub-vs-front contacts. `IFS_Bulkhead_Convertible` vs `Steering_Rack` (10.9 mm³) and vs `Servo_Horn` (8.3 mm³) are front-session items. Cage relief 1274 mm³ total (same dropped fragments).
* **Axle**: the FMT24 diff idea is dropped (user decision). Stock SCX30 rear axle (`Ref_Rear_Axle` from the per-body folder), SCX30 hubs and front diff are kept; nothing in v4 was changed for the FMT24. The axle-top bracket is built for that housing (pedestal seated at Z 7.0 on the pumpkin top at X -57.5, Y 0); axle-side knobs: `A_`, `BP`, `BR_EYE_T`, bracket box in `Rear_Axle_Top_Bracket`.
* Still open: ball-end links for cross-articulation, `17_hood_scoop.py` re-run on the v4 shell, cage lower-ring replacement checked only by the hoop, 50 mm battery needs the vertical standoff tray (not built).
* Axle seat check: at X -57.5 the stock housing top is Z 7.0 at Y ±2 but dips to Z 2.2 at Y 0 (a groove/slot), so the bracket pedestal (Y ±2.4, bottom Z 7.0) bridges the groove and rests on the two ridges. Confirm on the real axle; if the ridge is narrower, widen `Rear_Axle_Top_Bracket`.

---
# Rev 4: v4 shell without the hood scoop (`scripts/19_shell_v4.py`)
* **Source**: `stl/tacoma_trophy_shell.stl` widened like scripts/12 (side panels +2.1 outboard, ramp Y 8..10). **No hood scoop, no 17_hood_scoop.py** (servo lies on its side; the bulkhead and servo envelope do not touch the hood: shell vs bulkhead 0 mm³, vs servo box 0). `18_tub.py` no longer writes the shell; 19 owns `stl/v4/Tacoma_Trophy_Body_Shell.stl` (run 18, then 19).
* **Relief applied** (all dilated by 0.3 mm): tub (5 mm³ of overlap with the raw body), rear shocks (0.2 each), sway-bar levers and drop links (one convex hull per mover over -14..+7: this clears the two 0.03–0.08 mm³ lever scratches), front wheel envelope (hull per wheel over the poses that touch, 10 of 30 lock x rack poses touch the raw shell), gearbox + motor pocket (raw shell already clear, 0 mm³). Volume 7699 → 7572 mm³ (127 mm³ removed).
* **Result**: watertight, 0 open edges, 1 shell after STL roundtrip. Overlap with Chassis_Tub, Rear_Cage_Tail, ESC, receiver, battery, servo envelope, bulkhead, cage, shocks, gearbox, motor: **0.000 mm³ each**. Rear sweep -14..+7 (arms, links, axle, wheels, bracket, bar): **no contact**. Front wheels over lock ±8° x rack ±2.75: **no contact** (07 pose_scene = v2 front kinematics, same check as 10_verify_front_shell.py; the v3 front links the rack through Steering_Link, so recheck once with the v3 pose function if it differs).
* Union-of-poses sweeps produced 138 sliver shells after STL export, so sweeps are single convex hulls; hulls over-cut slightly in the wheel arches (visible in the 127 mm³ total).

---
# Rev 5: integration clean-up (final overlap volumes, mm³)
Removed stale `Rear_Sway_Bridge.stl` and `HW_Rear_Sway_Bridge_Screw_L/R.stl` from stl/v4. Hoop moved to X -61.0 (collar was clipping the battery lip), legs start 1.3 above the deck, pad bottom on the deck (0.0 gap, no intersection). Corner feet columns no longer reach inboard of the pad face (cleared the 1.0 mm³ link boss overlap). Links: L blade kept above Z 8.65 and R blade within Z 7.4..8.35 at the axle end (0.3 gap between the two, clear of the axle top); arms cut 0.1 clear of the axle end; ESC pocket sunk 2.0 (ESC top Z 1.1). All 38 parts incl. `Rear_Upper_Link_R` (volume +305 mm³) positive volume, watertight, single shell after STL roundtrip; shell (19) re-run: watertight, 0 overlaps, no rear/front contact.
Final rest-pose overlaps (everything above 0.01 mm³):
| pair | mm³ | class |
|---|---|---|
| Chassis_Tub / HW_Rear_Trailing_Pivot_Screw_L, _R | 0.74 each | screw in tapped pilot (intentional) |
| Chassis_Tub / HW_Rear_Cage_Screw_L, _R | 0.73 each | screw in deck pilot (intentional) |
| HW_Rear_Cage_Screw / Rear_Cage_Tail | 0.18 each | screw in pad hole (thread seat) |
| HW_Rear_Upper_Link_Pivot_Screw / Rear_Upper_Link | 0.61 each | screw tapped into boss (intentional) |
| Bar lever / drop link, bar cross / lever | 0.33 / 0.30 | wire joints (intentional) |
| Ref_Front_Center_Driveshaft / Ref_Gearbox | 37.2 | **front session, not mine** |
Gone vs rev 3: tub/Rear_Cage_Tail 2.8, tub/upper links 1.0 each, link L/R 0.4, link R/axle 0.1, arm/axle 11.2 per side, ESC/rear driveshaft 0.3. Sweep (-14..+7): nothing except the axle-end screw bore artifacts (2D model) and 0.01–0.02 mm³ pin/link and link/axle touches at the extremes.

## Rev 6 - axle-top bracket regenerated (was a 0.1 mm flake)
- `Rear_Axle_Top_Bracket`: 5.3 x 9.0 x 4.3 mm (X -60.9..-55.6, Y +-4.5, Z 3.2..7.5), 42.7 mm3. 0.5 mm plate on the two stock axle-top ridges (|Y| 1.7..3, Z 7.0) with a 2.0 x 3.1 mm key dropping into the groove between them (locates in Y); the pin goes through plate + key. Plate stops at BP_X+1.9 so the lower link blade clears it over travel.
- Eye stack restacked: BP Z 8.85 (was 8.5), R eye Z 7.7..8.8, L eye 8.9..10.0, pin head top Z 10.4. Upper pivot PU Z 7.35 (was 7.0) keeps the link slope. Kinematics: arm 33.00 / link 33.00, anti-squat ~22%, shock 25.9..31.0.
- Overlaps: bracket vs Ref_Rear_Axle 0.000; pin vs bracket 1.6 mm3 (pin seat); bracket/pin vs R link only 0.06/0.05 mm3 at ta -14 (0.014 at -10) from pin tilt in the bore; no bracket-to-deck contact over ta -14..+7.
- Roll_Cage: zero-volume sliver that appeared after the STL roundtrip is now dropped (single shell, watertight). Shell (19) re-run: 0 overlaps, no contacts.

## Rev 7 - battery 44.5 x 20 x 10.5 (supplied size) and rear extension
- No placement avoids the extension: deck+shell give 31.6 mm along X (hoop collar), 35.8 mm across Y (wheel/rocker/levers outside), pack on its 20 mm side tops out at Z 36.3 (shell 31.9, cage 30), motor-side pod is 5.3 mm wide. So the tub deck is extended 17 mm to X -81.5 (was -64.5) and the pack lies flat: 44.5 (X) x 20 (Y) x 10.5 (Z), X -74.1..-29.6, Y +-10, Z 16.3..26.8.
- Retention: 0.9 mm lips on four sides plus four flex clips (2 per long side at X -45/-62, hook 0.4 mm over the pack top edge, hook top Z 27.25). Leads leave through a notch in the front-right side lip and drop through a 3.6 x 2.2 deck slot (X -33..-29.4, Y 11.4..13.6).
- Receiver (assumed 22 x 14 x 7) moved off the deck into a floor pocket sunk 2.0 mm (Z -6.9..0.1, X -46..-32, Y +-11), under the rear driveshaft (0.17 mm clear), 1.0 mm wall to the ESC pocket, open to the rear (slides in from behind; retain with tape/foam). Local skid pad under it. Hex ribs in that area removed.
- Deck stiffened by two side beams under the extension (Y 14.5..17.9, Z 11.2..14.7, X -81.5..-50); tub floor end unchanged at X -46 (axle droop clears, sweep shows no tub/axle contact).
- Shell (19): tail stretched 17 mm (x < -48 mapped linearly, tail end -61.5 -> -78.5); shell bounds X -78.5..55.5, vol 8602, watertight, 1 shell.

## Rev 8 - rear cage rebuilt (Rear_Cage_Tail, one printed part, screws to the tub)
- Hoop at X -77.5 (behind the pack): legs Y +-15.5, top bar Z 24.8, lower cross bar on the deck, sway-bar collars + tunnel and cross member (bar axis moved to X -77.5, levers 30.75 mm long, drop links still 16.4, tilt <= 2.9 deg over -14..+7), tail/bumper bar at X -83.5 with rear stays.
- Forward structure per side: stay from the hoop top to the main Roll_Cage rear node (ends 0.09 mm short of the cage tube = glued lap joint, no overlap), post under the stay at X -62, deck rail hoop->post, diagonal hoop top->post foot. Triangulated side frames; stays run at Y 15.5..16.6, Z 20.3..24.8, outside the pack (Y +-10).
- Screwed to the tub with 4 screws (hoop feet X -80.1, post feet X -59.2) into the deck side beams; pilots 3.4 deep.
- Roll_Cage relief no longer cuts around the tail (joined by the stays): Roll_Cage now X -54.9..36.2, 3667 mm3 (908 mm3 removed vs the v3 cage; old rear ring fragment 137 mm3 dropped).
- Printable: tubes dia 2.3 solid; single part 36.4 x 35.8 x 9.7 mm, print on the deck-foot face; small supports only under the diagonal and rear stays.
- Final rest overlaps (mm3): only screws in their pilots (cage screws vs tub 1.47 each, vs tail 0.18), pivot screws 0.74/0.61, bar joints 0.30/0.33, axle pin seat 1.6, front-session Ref_Front_Center_Driveshaft/Ref_Gearbox 37.2. Sweep (ta -14..+7): only axle-screw seats, bracket/pin vs R link 0.06/0.05 at -14; tub/axle, shell/levers, cage/levers clear. Shell: 0.000 mm3 vs everything, no rear or front-wheel contact. Anti-squat ~22%, arm/link 33.00.

## Rev 9 - spare tire removed, bulkhead mounted, motor size
- Spare_Tire_Carrier, Ref_Wheel_Spare, HW_Spare_Wheel_Nut deleted from stl/v4 and from scripts 18/19 (no bed pocket existed in the shell for it; shell vs spare overlap was 0). Earlier revs' spare notes removed; v3 files untouched.
- Layout study with the spare gone: still no placement without the rear extension (X length is the limit; roof Z 31.9 still caps a 20 mm tall pack at Z 36.3; widening the tub beside the motor/trans leaves 9.8 mm between the case (Y +-12.5) and shell inner wall 22.3, and the cage rockers at Y 17.4 and rear/front wheel faces at Y 29 stop any widening). Chosen layout unchanged: pack centred on the extended deck, receiver sunk in the floor, ESC under the motor.
- Bulkhead: tub walls now have through holes (dia 2.0) at (X27,Z2.5) and (X33,Z5.0) on -Y and at (X27,Z2.5) on +Y with dia 3.3 x 1.4 counterbores; HW_Bulkhead_Screw_R1/R2/L1 (M1.6 x ~5 self-tapping into the bulkhead's own dia 1.4..1.6 holes, head in the counterbore). +Y (33, 5) is the servo-ear position (ears X 32.3..33.5, Y 11.7..14.65) so no screw there. Positive location: the 0.1 mm notch pocket (X/Z), plus two rear stop blocks (X 23.0..23.9, Y +-11.9..13.8, beside each wall) and the floor as the Z seat. Screw vs bulkhead 0.25 (thread engagement), vs tub 0, vs servo envelope 0 (L1 tip at Y 11.85).
- Motor GM1614 set to dia 17 x 10 (was 16.4 x 14.5): X -21.5..-11.5, same axis (Z 18.1). Shell pocket and keepouts follow. The motor is still carried only by the gearbox; a tub motor cradle/clamp is NOT built (needs the motor mounting-face pattern).
- Gearbox 12 x 25 x 28: NOT applied. 12 mm deep contradicts the earlier 26 x 23 x 28 reading and the foot-hole pattern (feet at X +-10 imply >= 20 mm along X), so 12 is taken to be the gear case only; envelope kept at 23 (X +-11.5), Y +-13 vs 25 measured (0.5 mm margin each side).
- Fastening audit (fastened = screw/pin modelled in stl/v4): rear arms/links/axle/shocks/cage tail/bulkhead (3 of 4 positions): yes. NOT fastened (location only): servo (pocket + ears, no HW), gearbox (2 foot pilots in the tub, no HW screws modelled), motor (via gearbox), ESC and receiver (pockets, tape), battery (lips + clips), shell (no mounts), Roll_Cage (feet columns in the tub, glued, no HW), stays to Roll_Cage (glued).

## Rev 10 - rear cage split into screw-jointed pieces (supersedes the Rev 8 single `Rear_Cage_Tail` and its glued stay ends)
- Pieces: `Rear_Cage_Hoop` (legs, top bar, sway-bar collars + cross member, bumper/tail bar with rear stays, feet, joint tabs), `Rear_Cage_Stay_L/R` (forward stay, post, deck rail, diagonal, hoop-joint plate, main-cage joint plate), and lugs added to `Roll_Cage`. `Rear_Cage_Tail.stl` no longer exists (kept only as an internal keep-out union).
- Joints (M1.4-class, 8 screws): hoop<->stay, 2 per side, along Y from outside (plate Y 15.8..17.0 over hoop tab Y 13.7..15.2, X -71.5, Z 19.5 and 23.0); stay<->main cage, 2 per side, vertical, stay plate Z 20.1..21.3 lying on a cage lug Z 17.8..20.0 (X -52.4..-44.4, Y 15.4..18.4), screws at X -50.6/-48.4, Y +-16.9. Sway levers (Y 19.35.., Z 20.1) and drop links stay clear. `HW_Rear_Cage_Joint_Hoop_{L,R}{1,2}`, `HW_Rear_Cage_Joint_Node_{L,R}{1,2}`.
- Tub joints unchanged: 4 deck screws (hoop feet, post feet) into the deck side beams.
- Roll_Cage trimmed to X >= -46.2 (the thin remnants of the old rear ring, 0.4..1.6 mm slivers along the deck, are removed) plus the two lugs: X -52.4..36.2, 3680 mm3. The shell got pockets for the lugs (19 now includes Roll_Cage in its relief).
- Extents / volume: Rear_Cage_Hoop X -84.7..-69.0, Y +-17.9, Z 16.3..26.0, 937 mm3; Rear_Cage_Stay_L/R X -74.0..-46.8, Y 14.5..18.4 (mirror), Z 16.3..25.5, 229 mm3 each.
- Result: 51 parts in stl/v4 all watertight, positive volume, single shell after roundtrip. Rest overlaps: only screw seats (deck screws vs tub 1.47 each, vs hoop 0.18), pivot screws 0.74/0.61, bar joints 0.30/0.33, axle pin seat 1.6, front-session Ref_Front_Center_Driveshaft/Ref_Gearbox 37.2. Hoop vs stay, stay vs cage, stay vs levers/links: 0. Rear sweep -14..+7: unchanged (axle-screw seats; bracket/pin vs R link 0.06/0.05 at -14). Shell: 0 overlap, no rear or front-wheel contact, vol 8570.

## Rev 11 - wide middle tub, photo layout (transmission | motor side by side), battery 45.3 x 21.0 x 11.2
- Transmission 25 (X) x 12 (Y) x 28 (Z), centred on the shaft line (Y 0) so the front 4WD shaft and the rear output keep their old path (stubs still end at X +-15.5). The photo's off-centre transmission was NOT copied: it would put the front output 9 mm off the front axle input over ~10 mm of shaft. The earlier "12 deep contradiction" is resolved: 25 along X matches the old 23-26, 12 is the Y thickness. Foot holes moved to (-10,-4.5) and (10,4.5) (inside the 12 mm body; guess).
- Motor GM1614 dia 17 x 10, axis along Y on the transmission's -Y side (Y -16.1..-6.1, centre X +3.0 = a bit forward, Z 18.1; X -5.5..11.5): the pinion enters the transmission's side input (0.1 mm gap). Carried by two tub saddle pillars + a printed `Motor_Clamp` screwed down with 4 screws (`HW_Motor_Clamp_Screw_1..4`, pilots 4.2 deep in the pillars). Coupling and input height are assumptions (input at Z 18.1).
- ESC standing on its edge (24 x 8 x 18, assumed) on the +Y side (X +-12, Y 7.2..15.2, floor to Z 13.1; 1.2 from the transmission); two floor towers with `ESC_Clip_F/R` + `HW_ESC_Clip_Screw_F/R`. Receiver pocket (sunk 2 mm, 22 across Y x 14 along X) moved behind the transmission, X -30.4..-16, with `Rx_Clip_L/R` (+ screws) on floor posts. Battery: clips as before plus `Battery_Hold_Down` (bridge over the pack, legs screwed into deck bosses, `HW_Battery_Hold_Down_Screw_L/R`).
- Tub widened to +-20.7 for X -21..22.5 (shell inner at the mid-body 22.3): floor and 1.5 mm walls, closing walls at both ends (rear step clears the trailing-arm lug and shocks; front step keeps the bulkhead zone and screws untouched). Roll_Cage rails are pushed outward 2.8 mm over that zone (centre Y 20.2, tube outer 21.35); cage now Y +-20.2, X -52.4..36.2, 3650 mm3 (cut back around motor, transmission, clamp, ESC clips; 13 small fragments dropped). The shell is pocketed (shell 8270 mm3, 0.000 overlap with everything, no rear/front-wheel contact).
- Battery 45.3 x 21.0 x 11.2: "left to right" (45.3 along Y) does NOT fit: the shell is only about +-16..21 wide at deck height (measured), tub max +-20.7. So it stays lengthwise, which keeps the rear extension (deck to X -82.5, hoop at -78.5, shell tail +18). Pack X -74.9..-29.6, Y +-10.5, Z 16.3..27.5.
- Final: 67 parts, all watertight single shells. Rest overlaps only screw seats (as Rev 10) + front session Ref_Front_Center_Driveshaft/Ref_Gearbox 42.7 (their shaft vs the 12.5-long case). Sweep -14..+7 unchanged (axle-screw seats, bracket/pin vs R link 0.06). Gaps: ESC-cage 0.99, ESC-servo 1.4, motor-clamp-shell 0.42, receiver-driveshaft 1.18, battery-cage 1.6, battery-shell 7.2.

## Rev 12 - transmission 12 (X) x 25 (Y) x 28 (Z), in-line motor, two identical ESC/RX pockets

- Transmission: X +-6, Y +-12.5, Z -4.9..23.1 (shaft line and Z unchanged; output stubs still end at X +-15.5 so the driveshaft ends do not move). Foot holes scaled to the narrower body: (-4.8,-9.4), (4.8,9.4) - ASSUMED pattern.
- Motor GM1614 (D17 x 10) is longitudinal, in line behind the transmission on the input axis: X -16.1..-6.1, Y 0, Z 18.1. Held by two saddle bridges (Z 8.0 up, over the pockets, standing on 4 posts at |Y| 13.4..16.4) and Motor_Clamp with 4 screws.
- A symmetric left/right pair of pockets does not fit: behind the motor the side strips are only X -21..-6 (rear link pads and trailing arms block the wide zone further back), beside the transmission only 6.7 mm. Instead: two identical flat floor pockets (26.4 across Y x 18.4 along X, sunk 2 mm) under the shaft line, behind the transmission: A X -44.0..-25.6, B X -24.8..-6.4. ESC (26x18x8) is in B, receiver (22x14x7) in A; either part fits either pocket. Each has two screwed Pocket_Clip_{A,B}_{L,R}. All mass is on Y 0 (balanced).
- Drivetrain X extents: case -6..+6; motor -16.1..-6.1; stubs to +-15.5. Freed X length used for the longer ESC pocket pair.
- Results: 66 parts, all watertight single shells after STL round trip. Rest overlaps unchanged from Rev 11 (screw seats only; front session's Ref_Front_Center_Driveshaft vs Ref_Gearbox 28.9 mm3). Shell 8404 mm3, 0.000 overlap with tub, cage, ESC, RX, battery, gearbox, motor; no rear or front wheel/shell contact in the sweeps.

## Rev 13 - battery tray removed, rear cage rebuilt (Yeti style), bulkhead floor screws + longer nose

- Battery tray removed: Battery_Hold_Down, lips, snap clips, lead notch/slot, hold-down bosses and screws are gone; the deck is a plain flat plate (two wire slots at X -30, Y +-2 remain; leads exit the pack front and drop through them).
- Rear cage rebuilt from scratch as three printable pieces (tube D2.3, solid): Rear_Cage_Hoop (legs to Z 30.0, top bar, sway-bar collars + tunnel, low cross = pack rear stop, X braces, tail bumper bar with two struts per side, deck feet) and Rear_Cage_Stay_L/R (hoop joint plate -> riser -> upper rail at Z 29.6 -> drop diagonal to the Roll_Cage lug; post, deck rail and two diagonals triangulating the panel; screwed to hoop, Roll_Cage lug and deck as before).
- Battery (45.3 x 21.0 x 11.2, lengthwise, X -74.9..-29.6) is held by the cage: side cradle rails at Y +-12.0 (Z 22.6, above the main-cage rail tops), pack rear stop (hoop cross), a front hook per side down to a screwed deck foot (HW_Battery_Hook_Screw_L/R at X -28.1, Y +-9.2) and two half strap bars (X -69.0, -61.5, Z 29.6, 1 mm over the pack top) that lap at Y 0 with two screws each (HW_Rear_Cage_Strap_Screw_*): unscrew those four to lift the pack out. Wire path: leads leave the front face between the hooks (|Y| < 5) and drop through the deck slots. The old 44.5 x 20 x 10.5 figures were taken as L x W x H (pack lying flat); a 20 mm tall pack does not fit under the cage.
- Bulkhead floor screws: four vertical D1.8 holes found in stl/v3 IFS_Bulkhead_Convertible at X 44.5/52.0, Y +-3.5 (web top ~Z 0.5). Tub nose extended under the bulkhead to X 64 (NOSE_L 28, kick-up from X 56, width +-14.2), local boss 1.2 mm under X 43..54 for deeper pilots, 4 x D1.2 pilots and HW_Bulkhead_Floor_Screw_1..4 (M1.6 x ~8). ASSUMPTION: head seat Z +0.5 and screw length taken from the web geometry; confirm with the front session.
- Results: 73 parts exported, all watertight single shells after STL round trip; shell 8404 mm3, 0.000 overlap with everything; rest overlaps are screw seats only plus the front session's Ref_Front_Center_Driveshaft vs Ref_Gearbox 28.9 mm3; rear sweep -14..+7 unchanged.

## Rev 14 — longer wheelbase, taller cabin, stacked ESC/RX bays

- **Wheelbase 107.5 -> 116.5 mm** (front axle fixed; `REAR_SHIFT` = 9 in 18/19, `R.shift_rear`). 4-link re-solved: arm = link = 33.0, IC x=159.5, anti-squat ~21 %. Driveshaft 40.3 mm, 11.6 deg at ta -14 (0.85 deg at rest). Rear sweep -14..+7 clean (only seat-type overlaps; trailing arm/tub 0.15 mm3 at ta -14 is the one new contact).
- **Overall:** L 160.7 (front tyre X 66 to hoop X -94.7), W 85 over tyres, H 75 (roof Z 60.0, ground Z -15). Shell X -88.5..55.5, Y +-30.5, Z 6.7..60.0, watertight single shell (RAIL_DY 4.0 widening; cage tubes cut with 0.3 mm relief).
- **Roof lift** 28.1 mm via smoothstep warp (cabin only; hood/beltline lift modest); Roll_Cage follows (`cage_warp`).
- **ESC/RX stacked** in side bays: ESC upright on the floor, RX on a printed shelf (ESC_RX_Shelf_L/R) over it, pillars + HW_Shelf_Screw_*, wire slot, finger notch; L/R swappable.
- **Bulkhead floor screws** at (44.5/52.0, +-3.5), pilots in the nose, head seat Z -0.5.
- Not done: tub side-profile scalloping; rear shock upper mounts moved with the rear suspension (check shock-top attachment).

## Rev 14b — pitched rear arms
- Before: lower trailing arm horizontal (pivot Z -0.5 = axle joint Z -0.5, 0 deg); upper link pivot Z 7.49 vs axle-side eye Z 8.85, i.e. pitched UP toward the axle (2.4 deg). Pivots are tapped into a pad inside the tub tail wall (screw from outside), so they are carried by the tub; the arms run beside the tail wall (Y 18.0..23.4, wall outer face 17.9).
- Now: `PL_Z` 2.4 (arm 5.0 deg down toward the axle), `PU_Z` 12.0 (link ~5.5 deg down), arm 33.13 / link 33.12 mm, IC ahead and far (near-parallel links), anti-squat ~23 %. Hub/axle Z unchanged (unsprung parts not raised with the body).
- Tub tail relieved around the droop envelope of axle + arms (0.2 mm); upper-link stacking cuts limited to the bracket (|Y| < 2).
- Not done: neutral mid-stroke stance (the shock Ref has no stroke data; eye-eye 30.5 at ta -14 .. 25.9 at +7, 27.7 at ride), bump stops, dense 1-deg sweep, front-arm pitch.

## Rev 14c — dense sweep, shell relief, audit (status)
- `18_tub.py` now ends with a dense rear sweep (ta -14..+7, every 1 deg): min clearance of each mover to every fixed body. Pivot joints (arm/link vs tub pad, bar lever vs cross bar) read 0.00 by design; Bar_link vs shell read 0.00 at ta +7 BEFORE the CLR=0.8 shell run (re-measure with the new shell).
- `19_shell_v4.py`: `CLR` env (run with CLR=0.8), movers that merely graze the base shell are now relieved too. Shell watertight, single shell, X -88.5..55.5, Y +-30.5, Z 6.7..60.0. Min to shell: wheels RL 0.46, FL/FR 0.38; Bar_lever_L still reads -0.00 (unresolved, see report); Roll_Cage 0.00 (mount contact).
- Floating-part audit (nearest body > 0.05 mm): HW_Rear_Shock_Upper_Screw_L/R (2.0 mm from the shock, no seat after the +9 wheelbase shift), Ref_Shock_RL/RR upper ends are not fastened to anything (0.25 mm to tub), HW_Bulkhead_Screw_L1/R1/R2 (0.14 mm, bulkhead holes vs tub wall).
- Assumptions NOT yet implemented: shock stroke 7 mm, mid-stroke neutral at ride (needs Lmax = ride eye-eye 27.66 + 3.5 = 31.16 mm; sweep range eye-eye 30.5..26.1 fits inside), bump stops, pinion/gear mesh (0.5 mod, 10T/30T, 10 mm centre distance) and slotted clamp holes, Front_Wheel_Envelope arch cut (file not delivered yet).

## Rev 14d — shock hangers, bump stops, chamfer, relief fix, front envelope
- **Relief bug fixed:** `dil()` (6 axis shifts) left diagonal gaps around thin parts (bar levers r 0.4, links), so keep-out cuts left 0.00 mm. Now 18 directions (axes + edge diagonals) in 18 and 19. Shell relief `CLR=0.8` (run 19 with `CLR=0.8`).
- **Shell:** watertight, 1 shell, X -88.5..55.5, Y +-30.5, Z 6.7..60.0; also cut around `stl/v3/Front_Wheel_Envelope.stl` (two non-watertight halves -> convex hulls, relieved by CLR 0.8 >= 0.5).
- **Shock upper mount:** printed hanger tabs grown on the Roll_Cage sill bar (X -37.5..-32.5, |Y| 16.0..18.1, Z 17..23.1) with an M1.6 pilot at (X -35, Z 21.5) through them into the shock eye; fixes the floating upper screw. The Ref shock models only the body (no rod end), hence its 0.1 mm gap to the cage.
- **Bump stops:** tub pads under the deck, |Y| 10..14, X -67.5..-61.0, bottom Z 11.4 (axle top Z 7.0 + 4.04 at ta +7 = 11.04; 0.36 mm designed gap). Droop end: shock fully extended at ta -14 (assumed stroke 7 mm, Lmax 30.48 eye-eye, Lmin 23.48; ride 27.66 is 0.7 above mid-stroke; bump +7 = 26.09).
- **Hub height:** rear wheel centre Z 1.50 at ta 0 vs front 1.47.
- **Tub side:** 3.2 mm, 45 deg chamfer along the lower outer edge of the rear/middle tub (partial answer to the slab look; no window scallops).
- **Motor clamp:** holes are +-0.6 mm slots along the motor axis. Assumed mesh: 0.5 mod, 10T pinion on a 30T input gear, 10.0 mm centre distance (input axis 10 mm below the motor axis, Z 8.1), set with shim washers under the clamp ears (+-0.5 mm). Real gearbox internals are not in the reference geometry.
- **Dense sweep** (every 1 deg, ta -14..+7): only pivot contacts (arm/link vs tub pad, bar lever vs cross bar, axle vs driveshaft joint) read ~0; Bar_link vs shell 0.34, rear wheels vs shell 0.62, front wheels 0.63, Bar_lever_L 0.52, Roll_Cage 0.32.
- **Still open:** open top deck (ESC/RX bays and cage tubes still on it), tub window scallops, `HW_Bulkhead_Screw_L1/R1/R2` 0.14 mm from the tub wall, bump-pad contact not separately measured, battery-to-cage-tube clearance not re-measured.

## Rev 14e — open deck, bulkhead seats, shock eye, bump stops
- **Open top deck:** strap bars and their lap screws removed (`STRAP_BARS = False`); main Roll_Cage cut out of the battery column (pack + 1.5 mm, nothing crosses it); 4 strap slots 8 x 2.2 mm through the deck at X -73.0 / -54.5, |Y| 11.7..13.9 (free windows between the cage frames); the side cradle rails + front hooks stay. Battery 9.2 mm below the shell; not over the motor (X -74.9..-29.6 vs motor -16..-6).
- **Bulkhead:** screw heads now seat on the counterbore floor (-0.05 overlap). Floor pilots verified against `IFS_Bulkhead_Convertible`: holes at (44.5/52.0, +-3.5), clear dia 1.8, web Z -2.0..-0.3, counterbore floor Z -0.3 -> head seat Z -0.3 (was -0.5); tub nose reaches X 64.0.
- **Shock top:** `Ref_Shock_Eye_L/R` (rod end, stem + eye) pinned to the hanger tab by the upper screw.
- **Bump stops:** pads under the deck, |Y| 10..14, X -67.5..-61.0, bottom Z 7.2; axle-to-tub gap 2.61 (ta 0), 1.38 (5), 0.83 (6), 0.28 (7).
- **Shell clearance:** static keep-outs dilated by CLR+0.9 (1.7), parts that graze the shell are relieved too; shell watertight, 1 shell. Bulkhead 0.79, Roll_Cage 0.35 (cage foot at X -15, |Y| 24.4, Z 8.1 vs shell lower rim; the relief does not reach it, cause not found), Bar_lever 0.54, Bar_link 0.38, rear wheels 0.65, front wheels 0.66.

## Rev 15 — no battery / receiver bodies
- `Ref_Battery_2S` and `Ref_Receiver` are no longer exported (stl/v4 files deleted). Their volumes (44.5 x 21 x 11.2 pack on the deck, RX box in the bay) stay as internal keep-outs in `18_tub.py` (`BATT_BOX`, `RX_BOX`) and as boxes in `19_shell_v4.py`, so the cage, deck and shell still clear a pack. `Ref_ESC_G40` is kept (the ESC bay and shelf are built around it).
- Rear deck/cage = the Rev 13/14 rebuild: no Battery_Hold_Down, no tray lips, no pocket/Rx/ESC clips, no strap bars (open deck, four strap slots). Leftovers removed: Battery_Hold_Down, HW_Battery_Hold_Down_Screw_*, Pocket_Clip_*, ESC_Clip_*, Rx_Clip_*, HW_*Clip_*, HW_Battery_Rail_Screw_*, HW_Rear_Cage_Strap_Screw_A/B1/2 (earlier revisions). `HW_Battery_Hook_Screw_L/R` stay: they fasten the cradle front hooks of the Rear_Cage_Stay pieces to the deck.

## Rev 16 — deck ends at the link pads; cantilevered rear cage
- **Deck / tub rear end: X -46.0** (was -91.5). Tub floor, tail walls and deck end there; the link pivot pads (X -39.2..-29.8, |Y| 11.7..16.4) stay on the tail walls, so every arm/link pivot is still carried by the tub. No deck, beams or feet behind X -46.
- **Rear cage floats above the links/axle**, carried only by joints at the tub's rear end: (1) node plates on the main-cage rear lugs (X -54..-46.8): 2 x M1.6 per side at (-50.6 / -48.4, |Y| 16.9), the Roll_Cage itself is screwed to the tub rim; (2) front cradle hooks: 1 x M1.6 per side at (-28.1, |Y| 9.2) into the deck; (3) hoop-to-stay joints 2 x M1.6 per side; plus the hoop carries the sway-bar bar + collars. Side frames are triangulated (upper rail, drop diagonal, new lower diagonal to the node, post, diagonals). Removed: hoop feet, post feet, HW_Rear_Cage_Screw_*(deck feet), side beams.
- **Battery cradle:** pack floor raised to Z 18.1 (keep-out Z 18.1..29.3, 44.5 x 21 x 11.2), resting on 2 deck ribs over the deck and 3 tube rungs (X -72/-62/-52, risers to the cradle rails) behind it; strap wraps around the rungs (deck strap slots no longer exist, the deck there is gone). Keep-out box in 19_shell_v4.py updated.
- **Bump stops:** posts hanging from the rung at X -62, |Y| 11.8 (bottom Z 4.9); the axle comes within 1.54 mm of the cage at ta +7 (not touching, stop not yet tuned to the 0.3 mm contact).
- **Clearances (ta -14..+7), movers to cage/shock bodies:** axle 1.54, upper link L 1.61 / R 2.42, bar lever 1.61, bar link 1.90, axle-top bracket 4.59; trailing arm to shock 0.45 (shock lower mount sits on the arm). Shell: Roll_Cage 0.35, bulkhead 0.79, rear wheels 0.65, front wheels 0.66.
- **Joint loads (estimate):** cage 12 g + pack ~35 g = 0.5 N static, 5 N at 10 g bump; moment arm ~35 mm -> 175 N mm over the 21 mm node-to-hook span = ~8 N per joint at 10 g (static < 1 N). M1.6 self-tapping in PLA pulls out at roughly 20-30 N, so margin is ~3x at 10 g: use M2 or add a third joint if the car is jumped hard.

## Rev 17 - shell restored to the original proportions (rigid lift)
- The stretched cabin (ROOF_LIFT 28.1, smooth z 17..29 lift) is gone. The shell is the original faceted Tacoma trophy-truck body again (cab, pillars, roof, hood, bed lines unchanged), still widened (WARP_DY), tail shifted/stretched for the 116.5 mm wheelbase.
- **Minimum rigid lift ROOF_LIFT = 3.2 mm** (computed by raising the unmodified shell and measuring shell-to-keepout distance): transmission top Z 23.1 vs ceiling 29.2 already clears by 6.1 (6.3 at +3.2); controlling items are the ESC/RX shelf tops (needs +3.1 for 1.5 mm), shock eyes (+2.2), motor clamp, bulkhead nose (+1.5 -> 2.4 mm). Roof Z 31.9 -> 35.1.
- The sill stays at Z 6.7: the lower body is stretched over Z 6.7..14.7 (`lowlift`, smoothstep) so everything above rises rigidly by 3.2 mm and the added height goes into the rocker/lower panels. Arches are still cut around the front wheel envelope and the rear sweep.
- Roll cage top tubes follow the roof (`lift` in 18 now H = 3.2, top Z 33.2, 1.9 mm under the roof).
- Overall (wheels at ground -15): shell X -88.5..55.5 (144.0), Y +-30.5, Z 6.7..35.1; kit L 160.7 x W 85 x H 50.1.
- Known: the upright receiver keep-out (Z 23..45) now cuts a 14 x 7 mm window through the roof over the right shelf (22 mm receiver does not fit under a 32 mm ceiling); decide: lay it flat / smaller receiver / keep the window.

## Rev 18 - receiver flat, shell clearances >= 0.8, bump stop tuned
- **Receiver:** keep-out is now 22 x 14 x 7 lying FLAT (RX_BOX X 14..36, Y +-7, Z 24.4..31.4: above the centreline in front of the transmission, 1.2 mm from the shell, 4.6 from the cage); the 7 mm wide shelf bay cannot hold a 14 mm footprint (grid search over the whole cab found no other free spot). No roof window, no shell cut. The shelf fences were removed. The receiver mount (strap/tray from the Roll_Cage) is NOT modelled yet.
- **Shell relief:** translated-copy dilation left gaps at cage knife edges (0.37 mm). dil() now has 26 directions at c and c/2, plus a patch pass (spheres at every cage/mover point still < CLR+0.04 from the shell). Min clearances to the shell: Roll_Cage 0.80, Bar_lever_L 0.80, front wheels 0.82/0.85, rear wheel 0.87; everything else >= 1.2. Shell: 1 watertight shell, positive volume after roundtrip, bounds X -88.5..55.5, Y +-30.5, Z 6.7..35.1.
- **Bump stops:** posts moved to X -71 (over the axle top), foot Z 6.7 (BUMP_FOOT): gap to the axle 4.0 (ta 0) / 2.34 (3) / 1.27 (5) / 0.77 (6) / 0.30 mm at ta +7.
- **Deck / cage (status):** deck cut back to the tub rear end X -46.0 (deck X -46.0..-26.9, behind it only the cantilevered cage); the rear cage floats above links/axle, held by node plates on the main-cage lugs (X -54..-46.8, 2 x M1.6 per side), 2 hoop joints per side and the front battery hooks.

## Rev 19 - deck tray cut back to the link pads
- Deck plate now X -35.0..-26.9 (was -46.0..-26.9): it ends at the front of the pivot pads (pads X -39.2..-29.8 stay, lower pivot X -35, upper -33.5). Tail side walls (Y 14.4..20.7, to Z 16.3), the pivot pads/ears and the main-cage lugs for the cage node plates (X -54..-46.8) are unchanged; the rear cage stays cantilevered (node plates, 2 hoop joints per side, front hooks), not carried by the deck. Pack ribs start at X -34.5.
- Nothing is exported for the receiver or battery (Ref_Receiver / Ref_Battery_2S are keep-out boxes only); Chassis_Tub, Roll_Cage and the shell are each one watertight shell after the STL roundtrip; 67 printed/hardware/reference parts, none loose.
- Shell patch pass threshold CLR+0.09, 3 passes: min clearances Roll_Cage 0.83, Bar_lever_L 0.85, rear wheel 0.86, front wheels 0.88/0.90; no sweep contacts. Bump stop gap 0.30 mm at ta +7. Overlaps unchanged (screws in pilots, driveshaft/gearbox ref 28.9 mm3); trailing arm to shock 0.45 mm at ta 4 (joint).

## Rev 20 - shell = the original faceted shell, no keep-out relief
- Pipeline (19_shell_v4.py): original stl/tacoma_trophy_shell.stl -> `lengthen` (fastback/bed zone X -50..-22 stretched uniformly 1.6x so planes stay planes; tail panel, blade and bed end move back rigidly by TAIL_E = 17; cab/roof/pillars/hood untouched) -> `warp` (sides widened +5.5 for |Y|>8: rail widening 4.0 + SHELL_MARGIN 1.5) -> `lowlift` (rigid +3.2 above the sill band, sill stays Z 6.7).
- **No relief cuts for the cage, ESC shelves, shock eyes, bulkhead etc.** (STATIC_CUTS/PATCH are off by default; the old patch-pass spheres that carved jagged slivers are gone). The only cuts are the wheel arches: rear wheel sweep hulls and the front wheel envelope hulls, dilated CLR 0.8 + 0.25.
- The parts moved instead: Roll_Cage top tubes / pillar heads lowered smoothly above Z 24 (CAGE_TOP_DROP 2.6, cage top Z 30.6, was 33.2); shell sides 1.5 mm wider; tail 17 mm instead of 27 so the sway-bar levers and rear hoop are behind/clear of the tail blade.
- Result: ONE watertight shell (positive volume after STL roundtrip), vol 9703 mm3, X -78.5..55.5 (L 134), Y +-31.9, Z 6.7..35.1. Min clearance shell to: Roll_Cage 0.88, rear wheel sweep 0.84, front wheels 0.93, bar levers 0.95, receiver keep-out 0.86; all other parts >= 2.3. No sweep contacts. Overlaps unchanged (screws in pilots, driveshaft/gearbox ref). Bump gap 0.30 at ta +7.
- Silhouette check (side XZ / top XY / front YZ vs the original): same roof, A/C pillar, hood and bed lines; differences are only the longer fastback/tail and the 5.5 mm wider sides.

## Rev 21 - rear unsprung bodies exported at the shifted position
- Bug: Ref_Rear_Axle, Ref_Wheel_RL/RR and the rear axle-side hardware were not in stl/v4, so downstream checks/the .sc read the stale per-body copies at the old wheelbase (axle X -63.5..-47.0, wheels -77.5..-41.5). The sweeps inside 18/19 were already using the shifted copies (R.shift_rear).
- 18_tub.py now exports shifted copies (REAR_SHIFT 9, rest pose = identity, same file names): Ref_Rear_Axle (X -72.5..-56.0), Ref_Wheel_RL/RR (X -86.5..-50.5), HW_Rear_Wheel_Nut_L/R, HW_Rear_Upper_Link_Axle_Nut_L/R, HW_Rear_Upper_Link_Axle_Screw_L/R, HW_Rear_Shock_Lower_Nut_L/R (11 bodies, 78 parts total). Ref_Rear_Axle is a 4-shell reference body (as in the source folder).
- Not exported (obsolete since v4, pivots are tapped into the tub): HW_Rear_Trailing_Pivot_Nut_*, HW_Rear_Upper_Link_Pivot_Nut_*, Rear_Sway_Bridge(+screws) (replaced by the v4 sway bar). Spare-wheel bodies (Ref_Wheel_Spare, Spare_Tire_Carrier, HW_Spare_Wheel_Nut) are still the old v3 ones: out of scope here.
- Rest-pose check of every new body against all stl/v4 bodies: no overlaps (>0.01 mm3); touching only their own fasteners / the axle-top bracket (0.0 by design). Rear wheel sweep vs shell 0.84 mm, no contacts (arches are cut from the shifted sweep).
