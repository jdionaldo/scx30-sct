# Shell rear slope and tail end (for the cage tail)

Source: `scripts/19_shell_v4.py` (shell worker). Units mm; X fwd, Y left, Z up. L/R symmetric.

Status: built and verified in the shell script (top ridge 0.0002 mm over X -28..-67, underside 0.0005 mm over X -28..-55). The slope lines are **measured on the current `stl/v4/Tacoma_Trophy_Body_Shell.stl`** (8 stations, X -28..-66.5, top-ridge fit residual < 0.001 mm; underside fit 0.0005 mm over X -28..-55 only) and are final: the shortened bed does not change them. The tail end and hinge position are the new design values; the shortened shell is **not built yet** (BODY_DIR was wiped by the restart).

## The slope

The "rear shell slope" is the sloping rail that runs from the roof down to the tail on each side (|Y| 21.10..22.98).
Its inner face is the plane |Y| = 21.100. The reference line is the **top ridge on that inner face**; it is the side-view silhouette.

| Line (on the face \|Y\| = 21.100) | Equation | Slope | Angle |
|---|---|---|---|
| Top ridge (outer surface, reference) | Z_top(X) = 33.480 + 0.28400 * (X + 64.0) | 0.28400 | **15.855 deg** |
| Underside (inner surface) | Z_under(X) = 30.998 + 0.26150 * (X + 64.0) | 0.26150 | 14.655 deg (valid only for X >= -55; behind that the hinge gusset fills the underside, so use the top ridge) |

- Valid range: X -28.0 (front) to X -67.0 (tail end). Both lines are straight over the whole range.
- The rail tapers: 2.48 mm deep at X -64, 3.29 mm at X -28. So top and underside are not parallel (1.20 deg apart). Use the **top ridge, 15.855 deg**, for "parallel to the shell slope".
- The top face is not flat across the rail: its outer edge (|Y| about 22.9) falls at about 15.0 deg. Only the inner ridge is the silhouette.

| X | Z_top | Z_under |
|---|---|---|
| -28.0 | 43.704 | 40.412 |
| -34.0 | 42.000 | 38.843 |
| -40.0 | 40.296 | 37.274 |
| -46.0 | 38.592 | 35.705 |
| -52.0 | 36.888 | 34.136 |
| -58.0 | 35.184 | 32.567 |
| -64.0 | 33.480 | 30.998 |
| -67.0 (tail end) | 32.628 | 30.214 |

## Tail end and hinge (shortened bed)

| Item | Old | New |
|---|---|---|
| Shell rear end X | -79.1 | **-67.0** |
| Tilt hinge axis (X, Z) | (-76.5, 29.0) | **(-64.0, 29.0)** |
| Shell hinge lug \|Y\| | 18.90..21.30 | unchanged |
| Cage ear outer face \|Y\| (0.30 gap) | 18.60 | unchanged |
| Lug OD / pilot bore | 5.2 / 1.7 (M2) | unchanged |
| Bed ledge top Z | 30.00 | unchanged |

- The rear axle is at X -68.5. I ended the shell at -67.0, not -68.5: X -67.0 is exactly where the straight-slope zone ends. Behind it the stock rail kinks to a steeper angle, so ending at -68.5 would leave 1.5 mm of rail that is not on the slope.
- The rear fins and the old tail frame are behind the new end and are gone.
- The shell is open between the rails (no deck or roof skin for \|Y\| < 20.8 behind X -22).

## Constants in the script

`TAIL_END_X`, `REAR_RAIL_Y_IN`, `REAR_SLOPE_X0`, `REAR_SLOPE_Z0`, `REAR_SLOPE`, `REAR_UNDER_Z0`, `REAR_UNDER_SLOPE`, `REAR_SLOPE_XA`, `REAR_SLOPE_DEG`, `REAR_UNDER_DEG`, `REAR_LEDGE_Z`, `REAR_HINGE_X`, `REAR_HINGE_Z`, functions `rear_rail_z_top(x)`, `rear_rail_z_under(x)`.
The script re-measures the built shell against these lines at the end of every run and prints the maximum deviation.
