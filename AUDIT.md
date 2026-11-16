# SCX30 SCT — Audit (2026-11-16)

Checked from `scx30_sct_107mm_kit.sc` (103 bodies, re-exported per body) and the shipped STLs.

## Passing
- **Interference (rest pose):** `inspect.interference` → 103 bodies, **0 interferences**.
- **Watertight:** every shipped STL (9 files, 36 shells) and every per-body export is closed, consistently wound and positive-volume — **except** the `.sc` copy of `Tacoma_Trophy_Body_Shell` (23 edges shared by 4 faces = non-manifold; slicers may repair/fail). The older `stl/tacoma_trophy_shell.stl` is manifold, so **`print/parts/Tacoma_Trophy_Body_Shell.stl` uses that file**. The `.sc` shell should be regenerated from `build_shell_cadquery.py` (needs `.venv` with CadQuery; not present here).
- **Mount holes (screw surface vs. part surface within 0.45 mm):** all rear-suspension, front hinge-pin, shock, retainer, tie-rod and drag-link screws engage both mating parts.
- **Print files:** `print/parts/*.stl` (28 parts, each oriented for fewest overhangs, all watertight) and `print/plate_1.3mf` (300×300 mm shelf layout; fits any Bambu bed). Parts are tiny (largest 117 mm), so plate size is not a constraint. Note: "H3C" is not a Bambu model I know; assumed H2C/H2D-class bed (≥ 320 mm).

## FAILING — needs design changes (not fixed here)
1. **Front steering cannot work as modeled.** Knuckle/arm joints (`HW_Front_*_Kingpin_*`) are pins along **X**, so the knuckle can only hinge like a flap, never steer about a (near-)vertical kingpin. The Ackermann numbers in `MOTION_ANALYSIS.md` assume a kingpin that does not exist. Fix: bore the arm ends and knuckle along the steering axis (line through (Y32.5,Z0)→(Y30.9,Z6.8)) and use that pin direction.
2. **Upper kingpin doesn't engage the knuckle** (touches only `IFS_Upper_Arm`); lower kingpin barely (19 %). Knuckle upper joint is unmounted.
3. **Servo mounting screws engage only the servo** — the bulkhead has no screw holes at `[27.7..29.9, ±6.2..8.4]`.
4. **Path A (4WD) is impossible:** `Servo_Ref` (X25.8–31.8, Z2.3–9.7) sits on the centerline in the exact path of the front driveshaft (Y±1.8, Z2.4–7.3); the shaft also stops at X23.9 vs worm input at X40.
5. **Ride height mismatch 1.5 mm:** front tire bottoms at Z −15.0, rear at −16.5 (equal Ø36) → front tires hover 1.5 mm.
6. **No transmission/motor body is modeled**, so gearbox ↔ rail/cage/servo clearance is unverified; rear driveshaft (−46.8→−9.8) is geometry-only.
7. `MOTION_ANALYSIS.md` itself reports collisions at front bump/steer (shell, CVD, tie rod) and rear full bump (axle into sway bridge, rails, cage, shell). Rest-pose 0 interference does not cover these; I did not re-run pose sweeps.

## Print-optimization flags
- Too thin for 0.4 mm nozzle (<0.8 mm walls on 17–70 % of surface): `Front_Pin_Retainer_Plate` (0.7 mm thick), `Servo_Horn`, `Drag_Link`, `Steering_Bell_Crank`, `IFS_Knuckle` (40 %), `Rear_Upper_Link` (25 %). Print these at 0.1 mm layers in PA-CF/PETG-CF with 100 % infill, or make them from metal/PCB stock.
- `Tie_Rod` is a 1.1–1.2 mm plate → use M1.4/1.5 mm wire instead.
- Roll cage prints flat (99×37×22 mm, ~1500 mm² overhang → tree supports); shell prints standing (~300 mm² overhang).
- BUILD_GUIDE tri-counts/dimensions and the Kit-envelope numbers are stale vs. current files.

## Update — fixes applied (see FRONT_V2.md)
Items 1-5 are fixed in the v2 front end (`scripts/07_front_v2.py`, `stl/v2/`, `print/`). Item 6 (gearbox) is now a clearance envelope only;
item 7 (pose collisions) was re-run for the front: usable travel ≈ 6 mm, steering ±25-35° free. Rear pose collisions and shell arches remain open.

## Update 2 — rear travel relief (scripts/08_relief.py, 09_verify_rear.py)
- Finding: at the .sc pose the rear axle housing cleared the rails/sway bridge by <1 mm, so rear bump travel was ~0 (droop was fine).
- Fix: `Rear_Sway_Bridge` raised 10 mm; rail rear tips and the shell arches relieved by the swept axle/arm/wheel envelope (+0.5 mm clearance); front-wheel lock/bump envelope also cut from the shell. Cut parts are in `stl/v2/` (watertight, 1 shell each).
- Verified (python-fcl, 4-bar solve, HW ignored): rear free from -10° (−9.7 mm droop) to +6° (+5.1 mm bump); front wheels clear the relieved shell over full lock/travel. Fit bump stops at ~+5 mm.
- Not changed: sway-bar tunnel position (bridge raised, so the wire line moves up 10 mm; re-check drop-link length).

## Update 3 — rear trailing arm lower shock clevis (scripts/14_shock_slot.py, 15_verify_shock_slot.py)
- Problem: the lower shock screw crossed the 3.5 mm-wide arm at (X -44, Z 0.5) with no slot, so the shock eye had nowhere to sit.
- Fix: `Rear_Trailing_Arm_L/R` (stl/v3) now have an outboard clevis boss (arm Y 16.5..21.9, flush top Z 2.2), a 2.4 mm slot with a round bottom and a ±25° swing wedge cut along the shock axis, Ø1.6 clear bore in the inner cheek (1.05 mm) and Ø1.25 self-tap pilot in the outer cheek (1.95 mm); no nut trap. Arm also shifted +0.5 mm outboard (rails moved).
- Shock centre line moves from Y 17.75 to 18.75; `Ref_Shock_RL/RR` and the lower/upper shock screws are re-exported in stl/v3 at the new Y. The shock upper mount on the cage/rail tower must move +1.0 mm in Y too, or the shock leans ~2°.
- Verified: eye, rod and body clear the arm over -10..+6° arm angle. Shock eye-to-eye 24.7..32.5 mm: the 32 mm SCX24 shock tops out at 32.0, so **limit droop to about -9° (or add a droop stop)**.
- ASSUMED, not measured: shock lower eye width 2.0 mm, screw Ø1.4. Change `EYE_W`/`EYE_D` in the script after measuring the real shock. Fit-check with the real shock before printing the arm.

## Update 4 — v3 integration (real Modrax gearbox + N30 Pro servo)
- Sources: `CHASSIS_V3.md` (rails/cage/links/shell around the 26x23x28 gearbox), `FRONT_V3.md` (servo upside-down in a bulkhead cradle), arms per Update 3. Everything is in `stl/v3/` (v3 > v2 > .sc exports); scripts 12-17.
- Shell: hood scoop for the servo (X 36..56, Y -1.4..20, top Z ~32) plus shock pockets re-cut (`17_hood_scoop.py`). Run order if rebuilding: 12 -> 17 (17 is not idempotent; re-run 12 first).
- `16_integrate_check.py` (97 bodies incl. gearbox and servo boxes): only intentional overlaps remain (rail/cage/slider/bridge mounts, spindle-in-wheel, horn/rack/bulkhead guides, arm-in-axle bolts, hardware in bores). No overlap of the gearbox or servo with any body. Front wheel vs shell: none over arm -8..+8 deg x rack ±2.75 mm. Rear sweep (new link pivot, `rearlib.PU` updated): free -11..+6 deg = −9.8 / +3.9 mm axle travel (limit droop to ~-9 deg for the 32 mm shock).
- Print set: 27 watertight parts + `print/plate_1.3mf` regenerated from stl/v3 (fixed an inside-out `Rear_Upper_Link_R`).
- Unverified / needs your measurements: servo ear geometry and spline offset, gearbox shaft height (3.1), motor size/direction, shock eye width. Full lock + large bump on the front still limited (see FRONT_V3.md).
- `scx30_sct_107mm_kit.sc` still has the v2 front end and the old chassis; it is not updated for v3.

## Update 5 — rotated gearbox (23 x 26 x 28), GM1614 motor, corrected servo (rev 4 of CHASSIS_V3 / FRONT_V3)
- Inferred from the user's drawings (GEARBOX_FIT.md Update 3): gearbox X ±11.5 / Y ±13 / Z -4.9..23.1, shaft at Z 4.1; GAC GM1614 modelled as Ø16.4 x 14.5 on the rear side (size is a guess from the name); servo case 15.8 x 9.9 x 20.13 with a one-sided ear flange.
- Rails at Y 13.5..18.5; cage and shell widened ~2 mm; trailing arms shifted +2.0 outboard (`14_shock_slot.py`, shock line now Y 20.25, shock cheeks Y 18.0..23.4); hood scoop re-cut (`17_hood_scoop.py`: servo case X 40.7..50.6, Y 1.0..16.8, Z to 30.73). Rebuild order: 12 -> 14 -> 17 -> 13 as needed -> plate.
- Verified: gearbox/motor/servo boxes overlap nothing (only gearbox-in-motor-bore and intentional mounts remain); rear sweep free −11..+6 deg (−9.8 / +3.9 mm); shock eye/rod/body clear of the arm slot over -10..+6 deg; front wheels clear of the shell over full lock and ±8 deg travel. Print set regenerated, all parts watertight.
- OPEN: (1) `Ref_Rear_Sway_Bar` still follows the old geometry (arms Y ±17.75, Z 3..6): it now crosses the rails and needs re-routing outboard (+2.0 Y) and up with the raised bridge (+10 Z); drop-link length changes. (2) `Rear_Sway_Bridge` overlaps `Ref_Wheel_Spare` by ~165 mm³. (3) The .sc is still on the previous v3 (rotated gearbox not pushed). (4) Unmeasured: GM1614 size and whether it fits the Modrax motor opening, gearbox shaft height, servo ear geometry, shock eye width.

## Update 6 — sway bar, spare tire, .sc (rev 5)
- `Ref_Rear_Sway_Bar` rebuilt: bar at Z 16 through the raised bridge (X -63.5), arms at Y ±19.75 to X -46.75, outboard of the rails; new `Ref_Sway_Drop_Link_L/R` (Ø0.9, 11.9 mm centre to centre, max tilt 6.2° over -10..+6°). Arm/link sweeps cut slots in shell and cage (~135 mm³ shell). 0 contacts through travel.
- Spare tire: bridge could not be reshaped (1.3 mm web), so spare, nut and `Spare_Tire_Carrier` moved +6.0 mm in Z; clearances: bridge 1.37, cage 4.7, shell 6.3 mm.
- Shell re-cut with the hood scoop after the chassis regeneration; print set regenerated (27 watertight parts); rear sweep still free -11..+6 deg.
- `.sc`: stage 1 (47 bodies, rotated gearbox/motor/servo box) installed; stage 2 (sway bar, links, bridge, spare carrier, spare tire) pending.
