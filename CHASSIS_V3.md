# CHASSIS_V3 — real Modrax gearbox + GM1614 motor fit (rev 4)
Script `scripts/12_chassis_gbx.py` -> `stl/v3/`. Run: `BODY_DIR=<per-body stl dir> python scripts/12_chassis_gbx.py` (about 2–3 min).
**It regenerates `stl/v3/Tacoma_Trophy_Body_Shell.stl` from `stl/tacoma_trophy_shell.stl`, so the hood scoop from `17_hood_scoop.py` is gone. Re-run 17 afterwards.** I did not run 17.

## Spec (named constants at the top of the script)
- Gearbox box X ±11.5 (shafts along X), Y ±13, Z -4.9..23.1. Shafts at `GBX_SHAFT_Z`=4.1 (unverified), exit the X faces.
- Motor `GM1614`: Ø16.4 x 14.5, axis X, centre Z 18.1, on the rear side (`MOTOR_DIR`=-1), X -11.5..-26.0. Pocket = Ø17.2 x 15.3 (+0.4).
- Pocket through cage and shell: box +0.4, motor pocket, Ø4.2 shaft slot at Z 4.1 through X ±16.5.
- Rails: inner face Y ±13.5 (Y 13.5..18.5), i.e. +2.0 vs the original rails (the brief said +1.5; the Y range given implies +2.0). Sliders +2.0.

## Outputs in stl/v3 (all watertight single shells after an STL roundtrip)
- `Ref_Gearbox` (box + Ø3 shaft stubs, X ±15.5), `Ref_Motor_GM1614` (new)
- `Chassis_Rail_L/R`: +2.0 outboard, old upper-link pivot hole plugged, new Ø3 hole, nut pocket for the trailing pivot, axle relief re-cut
- `Slider_L/R`: +2.0
- `Rear_Sway_Bridge`: +10 mm lift (as v2), widened +2.0 each side. `HW_Rear_Sway_Bridge_Screw_L/R`: +2.0 outboard (new, not in the brief).
- `Roll_Cage`: everything at |Y| >= 10 shifted +2.0 (ramp |Y| 8..10); gearbox/motor pocket cut
- `Tacoma_Trophy_Body_Shell`: everything at |Y| >= 10 shifted +2.1 (ramp |Y| 8..10); pocket cut; rear relief kept
- `Spare_Tire_Carrier`: pocket cut (no real change)
- `Rear_Upper_Link_L/R`: rebuilt for the new pivot. `HW_Rear_Upper_Link_Pivot_Screw/Nut_L/R`: moved to match.
- `HW_Rear_Trailing_Pivot_Nut/Screw_L/R`: nut recessed in the rail at |Y| 13.6..14.8, screw |Y| 13.6..23.0 (all outboard of the pocket)
- `Ref_Rear_Driveshaft`: trimmed to start at X=-15.5 (stub tip)
- **Not exported (other session):** `Rear_Trailing_Arm_*`, `HW_Rear_Trailing_Axle_*`, `Ref_Shock_R*`, `HW_Rear_Shock_*`. The script shifts the arms and axle HW +2.0 in memory for the sweep check only.

## Pivot coordinates (X, Z)
| | old | now |
|---|---|---|
| Rear upper link front pivot | (-12.5, 5.25) | **(-15.5, 4.5)**, boss centre Y ±12.1 (against the rail inner face) |
| Rear trailing arm front pivot | (-7.75, -0.5) | unchanged |
Link: rod from the old line at X -45, kinked to the new boss; plan skew is bigger than v3.

## Verification (run by the script)
- `Ref_Gearbox` and `Ref_Motor_GM1614` vs every non-front body except the arms, shocks and axle HW: **0 overlap** (includes all rear HW, cage, shell, links, rails).
- Rear sweep -10..+6 deg: **no contact**. Only `Ref_Front_Center_Driveshaft` vs `Ref_Gearbox` hits (front session). Axle dZ -8.75 at -10°, +3.89 at +6°.
- Cage vs rails 0 mm³; cage vs shell 0 mm³ (after the shell widening).

## Rev 5: rear sway bar and spare tire
Constants: `BAR_X/BAR_Z/BAR_Y` = -63.5 / 16.0 / 19.75, `BAR_E_X` -46.75, `TAB_X` -46.75, `TAB_TOP_Z` 4.9, `LINK_INS` 0.8, `WIRE_R` 0.4, `LINK_R` 0.45, `TUN_R` 0.9, `SPARE_DZ` 6.0.
- **Bar** (`Ref_Rear_Sway_Bar`): wire Ø0.8. Cross-span along Y at X -63.5, Z 16.0, from Y -19.75 to +19.75, i.e. behind the rail ends (X -56.3). Arms run straight forward to X -46.75 at Y ±19.75, outboard of the rails (18.5) and above the trailing arm centre. The arms rotate about the cross-span axis.
- **Bridge tunnel:** Ø1.8 along Y at (X -63.5, Z 16.0) through the whole bridge. The bridge centre also got a web down to Z 13.5 (it used to end at Z 16, which left the tunnel half open). No separate saddle model exists, so none was changed.
- **Drop links** `Ref_Sway_Drop_Link_L/R`: Ø0.9 rods, vertical at rest from the arm end (Z 16.0) to Z 4.1, so 11.9 mm long (centre to centre). The bottom 0.8 mm goes into the tab.
- **Link angle over travel** (arm angle: link tilt from vertical / lever angle): -10°: -6.2° / +24.4°; -6°: -1.2° / +14.2°; 0°: 0° / 0°; +4°: -3.1° / -9.2°; +6°: -5.7° / -13.8°. Link length is constant by construction; the largest tilt is 6.2°.
- **Relief:** the arm and link sweeps cut a slot in the shell (about 135 mm³, where the wheel-arch sill is at Z 15–18.5 at Y 19.75) and the cage. The bar clears rails, bridge, spare carrier, spare tire, wheels and axle over -10..+6° (0 contacts).
- **Tab change needed (arm owner):** a Ø1.0 blind hole 1.0 mm deep in the tab top at X -46.75, Y ±19.75 for the link end (the link end enters the tab by 0.8 mm). Tab top is Z 4.9. The sweep check used `stl/v3/Rear_Trailing_Arm_*` as exported at run time.
- **Spare tire:** the bridge was 165 mm³ into `Ref_Wheel_Spare`. Reshaping the bridge was not workable: an arch cut left only about 1.3 mm of web around the tunnel. Instead the spare, `HW_Spare_Wheel_Nut` and `Spare_Tire_Carrier` move **+6.0 mm in Z** (carrier feet stretched down 6 mm to keep the carrier on the cage). Bridge–spare clearance is 1.37 mm; spare–cage 4.7; spare–shell 6.3.
- New exports: `Ref_Rear_Sway_Bar`, `Ref_Sway_Drop_Link_L/R`, `Ref_Wheel_Spare` (moved), `HW_Spare_Wheel_Nut` (moved), `Spare_Tire_Carrier`, `Rear_Sway_Bridge` (tunnel + web), plus the shell and cage (relief).
- Overlap check of bar, links, bridge, carrier and spare tire against all non-front bodies: only intentional insertions (bar/link joints; link end in the arm tab, which the check skips); the bridge touches the cage by a 0.08 mm³ sliver that was already there in v2.

## Risks
1. Arms are in memory only. The in-memory arm touches the axle end. Check it in the arm redesign; it now sits at Y 18..21.5.
2. The cage and shell shifts are vertex warps with a ramp at |Y| 8..10. Sloped shell faces there change wall thickness by up to 2.1 × slope; vertical side panels stay exact. The whole body is widened 4.2 mm.
3. The kinematics solve is 2D. The link's plan skew (about 8 mm over 44) is bigger now, so the real link is about 1 mm shorter in effect than the solve assumes. Re-check with a 3D model if travel is critical.
4. (fixed in rev 5) Bridge vs spare tire: spare moved +6 Z.
5. `GBX_SHAFT_Z` and the GM1614 size are inferred. The rear driveshaft in the body set is at Z about 4, ~0.2 mm from the shaft axis in this model.
6. `Ref_Front_Center_Driveshaft` hits the gearbox (front session). Whether the GM1614 fits this gearbox at all is unverified (see GEARBOX_FIT.md Update 3).
7. Rail width now 13.5: shocks and arms (other session) must follow the +2.0 shift.
8. `HW_Rear_Sway_Bridge_Screw_L/R` sit at Z 5.3–6.3, X -61.3..-60.3 (not lifted with the bridge +10 mm). They no longer meet the bridge. Fix when the bridge mounting is decided.
