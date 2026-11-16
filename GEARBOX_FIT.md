# Gearbox & servo fit check (research only)

Scope: compare the assumptions in `scripts/07_front_v2.py` against published data.
**Result: almost nothing could be verified from public sources. Nothing in the script is proven wrong by a datasheet, but the servo model looks inconsistent with every real SCX30-class servo found, and the gearbox envelope is unverified. Measure the real parts before printing.**

## Sources (what was actually read)
| # | URL | Outcome |
|---|-----|---------|
| 1 | https://www.amazon.com/dp/B0HMR5D54N | **Blocked (HTTP 503).** Title, speed count, dimensions, motor inclusion all unknown. Search engines do not index this ASIN either. |
| 2 | https://www.axialadventure.com/product/axial-transmission-assembled-scx30/AXI-3311.html | Read. Stock SCX30 transverse transmission (discontinued). No outer dimensions; only part list (30T/40T 0.3 mod gears, 3x6 bearings). |
| 3 | https://www.bigsquidrc.com/injora-n30-nano-servo-and-servo-horn-for-the-axial-scx30/ and https://limitlessrc.com/products/injora-aluminum-servo-saver-for-1-30-axial-scx30-scx30-09bk-copy-1 | Search snippets: Injora N30 Nano servo 21.45 x 15.2 x 12.85 mm, 7 g. No ear spacing / shaft offset. |
| 4 | https://www.smalladdictionsrc.com/reefs-rc-sx30-sub-micro-hi-torque-servo-for-the-axial-scx30 | Snippet: Reefs SX30 sub-micro 24.4 x 15.2 x 11.1 mm (case only), 8.2 g. |
| 5 | https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf | Fetched, but the dimension drawing is an image; not readable here. |
| 6 | https://www.kpower.com/insight_servo/2399.html , https://protosupplies.com/product/servo-motor-micro-sg90/ | SG90 body ~23 x 12 x 26-31 mm, ears to ~32 mm; vendor-dependent; no shaft offset. |
| 7 | https://www.bigsquidrc.com/injora-complete-transmission-50t-brushed-motor-for-the-axial-scx24-and-ax24/ (search listing) | An aftermarket SCX24 gearbox is listed 28 x 18 x 33 mm. Different product, only a scale reference. |

## Which servo does the repo assume?
Neither 9 g nor 5.5 g is stated. `FRONT_V2.md:35` says "previous project's numbers (not a datasheet)". `BUILD_GUIDE.md:108` says "stock SCX30 servo or compact ~22.8 x 12 mm micro servo" (22.8 x 12 is SG90-class, i.e. the 9 g class). The *modelled* body in the script is much smaller than either (see below).

## Dimension table
| Item | Script value | Published value | Source | Confidence |
|------|-------------|-----------------|--------|-----------|
| Servo body footprint | X 6.0 x Y 12.0 (`07_front_v2.py:161`), slot 6.8 x 12.8 (`:122`), Z height 6.8 | SCX30 servos: 21.45 x 15.2 x 12.85 (N30), 24.4 x 15.2 x 11.1 (SX30); SG90 ~23 x 12 x 26+ | #3, #4, #6 | Medium (vendor listings) |
| Servo ear spacing (EAR_DY) | 7.3 -> 14.6 centre-to-centre | Not found. SG90 ears ~32 mm overall (~27.5 hole spacing, from memory, **unverified**) | #6 | Low / unverified |
| Output spline offset from body centre | 4.0 (HORN_Y 5.5 - SERVO_YC 1.5) | Not found | - | Unverified |
| Spline height | boss top Z 5.525, horn Z 6.15 | Not found | - | Unverified |
| Gearbox envelope X +-9, Y +-9.6, Z -1.9..8 | 18 x 19.2 x 9.9 | Overall size of the B0HMR5D54N unit not obtainable. A reference SCX24 aftermarket gearbox is 28 x 18 x 33 | #1, #7 | Unverified |
| Front output shaft Z 6.85, rear Z 5.5 | as stated | Not found for B0HMR5D54N | - | Unverified |

## Pass / fail per assumption
| Assumption | Verdict | Reason |
|------------|---------|--------|
| Servo body 6 x 12 x 6.8 mm | **LIKELY FAIL** | Every real SCX30-class servo found is >= 11 mm in its smallest dimension and ~21-24 mm long. A 6 mm dimension matches nothing found. Either the model is a pocket for something else or the numbers are wrong. |
| Ears at +-7.3 (14.6 spacing) | **LIKELY FAIL / unverified** | Ears are only ~1.3 mm outside a 12 mm body; real micro-servo ears extend several mm beyond a ~22-24 mm body. |
| Horn offset 4.0 from body centre | Unverified | No datasheet found. |
| Gearbox fits X +-9, Y +-9.6, Z -1.9..8 | **Unverified** | Cannot be checked without the real part's dimensions. Note that the SCX24 reference gearbox is 18 mm wide; a longitudinal SCX30 box with motor likely exceeds 19.2 mm in Y at the motor plate. |
| Front output at Z 6.85 | Unverified | No shaft-height data for the unit. |
| Rear output at Z 5.5 | Unverified | Same. |

## Constants to change in `07_front_v2.py`
**None can be justified from sourced data.** Constants only worth changing after you measure the physical servo and gearbox with calipers:
- `SERVO_X` (line 64) -- keep unless servo length changes.
- `SERVO_YC` (line 65) and `HORN_Y` (line 64): set `SERVO_YC = HORN_Y - offset` where offset = measured spline-to-body-centre distance.
- `EAR_DY` (line 66): half of the measured ear-hole centre-to-centre distance.
- Servo slot box at line 122 (`25.4, 32.2, SERVO_YC +- 6.4`), body at line 161 (`25.8, 31.8, SERVO_YC +- 6.0`, Z -1.4..5.4), ear plate at line 161 (`SERVO_YC +- 8.5`): resize to the real case width/length (e.g. N30: 12.85 and 21.45 mm) plus 0.4 mm clearance.
- Gearbox envelope at line 173 `box(-9.0, 9.0, -9.6, 9.6, -1.9, 8.0)` and shaft heights at line 168 `6.85`: update from measured values.

## Suggested measurements (cheapest way to close the unknowns)
1. Gearbox: overall X/Y/Z, front and rear output-shaft centre height above the skid-plate bottom, shaft protrusion.
2. Servo: case length x width x height excluding ears, ear hole centre-to-centre, spline centre distance from case end, spline top height above ear plane.
3. Or paste the Amazon listing text / dimension image for B0HMR5D54N and the servo datasheet drawing; those could be reviewed in a follow-up.

## Update — user-supplied gearbox drawing (Modrax, 26 x 23 x 28 mm)
Source: product image supplied by the user (not independently verified). Values: **26 mm** along the output-shaft axis (X), **23 mm** across (Y), **28 mm** tall (Z), motor bore **9–10 mm**, pinion **0.3 mod 18T** (Ø6 mm, 4 mm bore ~2 mm). Output shaft sits low on the case (roughly 7–9 mm above the base from the picture; unverified).
Assumed orientation: shaft axis = truck X, centred at X0/Y0, base on the skid at Z −1.9.
| Assumption in script | Drawing | Result |
|---|---|---|
| X ±9 (18 mm) | 26 mm → ±13 | **FAIL** (+8 mm) |
| Y ±9.6 (19.2 mm) | 23 mm → ±11.5 | **FAIL** (+3.8 mm), box reaches the rail inner faces (Y 11.5) |
| Z −1.9…8 (≈10 mm) | 28 mm → −1.9…26.1 | **FAIL**, collides with `Roll_Cage` |
| Rear upper-link pivots at X −12.5 | box reaches X −13 | **FAIL**, collides with `Rear_Upper_Link_L/R` |
Bounding-box collision (stl/v2 + .sc bodies): the 26x23x28 box hits `Chassis_Rail_L/R`, `Rear_Upper_Link_L/R`, `Roll_Cage`; the old envelope hit nothing. The real case is stepped, so a bounding box over-reports, but the envelope is clearly too small. Needed: a real case profile (side + top views), motor position, and the true output-shaft heights before re-laying the cage, upper-link pivots and the front shaft tunnel.

## Update 2 — servo datasheet (Injora N30 Pro nano servo, user-supplied image)
Case 15.8 x 12.85 x 21.98 mm (case height 20.13 mm, spline boss +2.8 mm, overall 25.08 mm), spline Ø3.93, main case 9.9 mm deep, 4 kgf·cm, 7 g class. Mounting-hole spacing is **not printed**: ~10.5 mm and the spline offset (~4.5 mm from one edge, ~11 mm from the other) are my pixel estimates from the drawing (low confidence; measure the part).
Script assumed a 6 x 12 mm body, Z −1.4…5.4 (6.8 mm tall) — the real servo is **~3x taller (20–22 mm)**. The servo mount, horn height, rack and the shaft tunnel are all based on the wrong body. See result lines in the session for the bounding-box collision check of the real body at the script's servo position.

## Update 3 — inferred from the user's drawings + GAC power-system page (not yet applied to the CAD)
Source for the power system: https://gacracing.com/products/gm1614-and-g40-pro-power-system-for-scx30 (page text + 12 product photos; no dimension drawings).
**Power system (stated):** GM1614 brushless, 14 poles, 3500 or 7500 KV, 2S–3S, PH2.0 3P connector, 0.3 mod **11T steel pinion**, "compatible with the stock SCX30 motor plate", gear ratio >= 100. G40 PRO Nano ESC: FOC, 2S–3S, built-in 5 A BEC (6 V / 7.4 V), Bluetooth app, IP67, 2-channel radio. **Not stated anywhere:** motor body Ø/length, shaft Ø, mount-hole pattern, ESC size/weight. Inferred from the "1614" name and photos: motor ≈ Ø16 x ~14 mm, short and wide, 4-screw face plate on the pinion side (confidence medium, measure it).
**Modrax gearbox, re-read from the drawing:**
- The hex output shaft leaves the large flat face and runs parallel to the **23 mm** dimension line (not the 26 mm one). So the likely axes are shaft axis X = **23 mm**, across Y = **26 mm** (foot to foot), height Z = 28 mm. v3 assumed X 26 / Y 23: **probably rotated 90°**. At Y ±13 the box hits the rail inner faces (Y 12.0) unless the feet are narrower than the case flange (unknown).
- Output shaft height ≈ 9 mm above the foot base (pixel estimate; v3 used 8), i.e. Z ≈ 4.1 with the base at −4.9.
- The "9–10 mm" red line on the drawing spans the motor-position slot (mesh adjustment), not a Ø9–10 bore. The round motor bore is similar in size (~8–10 mm) and the 18T pinion has a 1.98 mm bore: that is an N20-size motor. A Ø16 GM1614 will not go through it; the GAC motor is meant for the stock SCX30 motor plate and 11T pinion. **Check that this gearbox accepts the GM1614 at all.**
**Servo (Injora N30 Pro), re-read from the top view (22 px/mm):** main case is 15.8 x **9.9** mm (not 12.85); the 12.85 mm is the ear flange (ears extend ~3 mm past one long side). Two mount holes ~10.0 mm apart, ~1.4 mm beyond the 9.9 mm face, centred on the 15.8 mm length; spline ~4.3 mm from the non-ear edge and ~4.4 mm from one 15.8 mm end. Ears are at the spline-end plane (case top), so upside-down in the bulkhead they sit on the cradle floor, not at the wall top as FRONT_V3 modelled (it assumed a tab at 9.9 mm height).
