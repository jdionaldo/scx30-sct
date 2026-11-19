# Shell attachment: printed clip-on (spec for the generator owners)

Status: DECIDED (clip-on, not magnets), NOT MODELLED. `scripts/19_shell_v4.py` and the cage generator are owned by other sessions; this file is the brief.

## Scheme
- 4 snap hooks on the shell underside, 2 front + 2 rear, each hooking over a cage rail / the tub deck edge (choose the nearest straight tube; the single-tube cage rails are 2.3 mm).
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
| Clearance to rail | 0.15 mm on the rail diameter |
| Strain limit | tip deflection <= 0.8 mm over 6 mm arm (PETG about 3 percent) |
| Shell lift | 11 mm: arms drop from the shell inner roof to the rail, keep >= 2.0 mm from every tube, tyre and the sway-bar lever (CAD gaps: cage 2.00, tub 1.94) |

## Print
- Arms must run along the layers' strong direction: flex axis in the XY plane, i.e. arms printed standing up from the shell roof when the shell prints sill-down (as plate 6 does: shell sill down, roof up). Hooks then print roof-down hanging in the air: add them as small tree supports or design them as 45 deg chamfered overhangs (no support).
- Brim stays 8 mm; verify no brim fuses to the hooks.

## Done when
- Shell snaps on and off 10 times without whitening or cracking, does not rattle, survives a 0.5 m drop of the finished truck on carpet.
- `print/make_plates.py` re-run (shell hooks may change plate 6 orientation and support recipe), then `print/verify_plates.py`, `print/gen_parts_list.py`, `docs/make_diagrams.py`.
