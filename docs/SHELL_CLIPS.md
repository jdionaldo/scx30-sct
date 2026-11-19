# Shell attachment: printed clip-on (spec for the generator owners)

Status: DECIDED (clip-on, not magnets), NOT MODELLED. `scripts/19_shell_v4.py` and the cage generator are owned by other sessions; this file is the brief.

## Scheme
- 4 snap hooks on the shell underside, 2 front + 2 rear, each hooking over a cage rail / the tub deck edge (choose the nearest straight tube). **The cage rails are now chamfered SQUARE 2.4 mm tube (Rev 40)**: a hook must seat on the flat top face and the flat side face (the 1.4 mm flat of the side between the 0.5 mm chamfers), not on a round tube. Roof rails sit at Z 30.4..31.6 and leave the shell 1.53 mm; the hook lip must fit in that gap or hook the side face below the roof plane.
- **No hook at the rear centre:** the X brace crosses the roof plane at Y 0 there. Put the rear hooks on the tail-rail sides, off the centre.
- The tilt-open body mount (front posts + rear pivot pins) is NOT modelled: it needs a front bar on the bulkhead and shell lugs at the open tail (X -76..-78). Clip-on remains the plan.
- Optional 1 x M1.4 x 5 screw at the rear centre as a safety retention (self-tapping pilot in a cage tab); skip if the clips hold in a drop test.
- No magnets (needs a steel counterpart, too weak in a rollover).

## Hook geometry (starting values, test on a coupon)
| Item | Value |
|---|---|
| Material | PETG, printed as part of the shell (same print) |
| Arm thickness (flex direction) | 1.2-1.5 mm |
| Arm length | >= 6 mm free length |
| Arm width | 4-5 mm |
| Hook lip | 0.8 mm deep, 30 deg lead-in ramp, 90 deg back face |
| Clearance to rail | 0.15 mm on the 2.4 mm square flat |
| Strain limit | tip deflection <= 0.8 mm over 6 mm arm (PETG about 3 percent) |
| Shell lift | 11 mm: arms drop from the shell inner roof to the rail, keep >= 2.0 mm from every tube, tyre and the sway-bar lever (CAD gaps: cage 2.00, tub 1.94) |

## Print
- Arms must run along the layers' strong direction: flex axis in the XY plane, i.e. arms printed standing up from the shell roof when the shell prints sill-down (as plate 6 does: shell sill down, roof up). Hooks then print roof-down hanging in the air: add them as small tree supports or design them as 45 deg chamfered overhangs (no support).
- Brim stays 8 mm; verify no brim fuses to the hooks.

## Done when
- Shell snaps on and off 10 times without whitening or cracking, does not rattle, survives a 0.5 m drop of the finished truck on carpet.
- `print/make_plates.py` re-run (shell hooks may change plate 6 orientation and support recipe), then `print/verify_plates.py`, `print/gen_parts_list.py`, `docs/make_diagrams.py`.

## Cage update (sway-bar saddles)
The cage now has two chamfered-square sway-bar saddles (3.6 x 3.6 x 2.8 mm) on the hoop legs at the tail (X about -78). They sit below the roof plane, so the hook rules above are unchanged; just keep hooks and shell lugs at least 1.5 mm away from them (shell has no wall at the open tail, so nothing to rub).

## Cabin cage and ride-height update (Rev 42-43)
The truck stack was raised 5.8 mm and a `Cabin_Cage` (chamfered square 2.4 mm, hood rails Z 26.8, roof arch Z 35.1 over the cabin) now fills the cabin and hood. Shell clearance to it is 1.68 mm, to the rear cage 1.53 mm. The front hooks can seat on the cabin cage's front hoop (X 26) or the hood rails, which are flat on top and sides like the rear cage; the rear hooks stay on the tail rails, still not at Y 0 (X brace). The tilt-open body mount remains not modelled.
