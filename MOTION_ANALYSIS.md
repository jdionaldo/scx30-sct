# SCX30 SCT (107mm) Steering & Motion Analysis

## 1. Steering Mechanism (`scripts/05_steering_and_motion_check.json`)
- **STL Export:** `stl/steering_parts.stl` (`Steering_Bell_Crank_L`, `Steering_Bell_Crank_R`, `Steering_Ackermann_Bar`, `Servo_Horn`, `Tie_Rod_L`, `Tie_Rod_R`, `Drag_Link`)
- **Layout (Axial-Yeti-style dual-post wiper bellcrank):**
  - `Servo_Ref`: `6.0 x 12.0 x 6.8 mm` micro servo body (plus `17.0 mm` mounting ears and `Ø3.2` spline boss at `[28.8, +3.0, 9.1]`) seated inside the `IFS_Bulkhead_Convertible` servo pocket (`X=25.5..39.5, Z=2.0..10.5`, clearing `Roll_Cage` `Z >= 10.83`).
  - `Servo_Horn`: `Z = 9.80..10.75 mm`, swinging from `[28.8, +3.0]` to `[28.8, +8.0]`.
  - `Steering_Bell_Crank_L/R`: dual-post chamfered-square bellcranks pivoting on `Steering_Pivot_Pin_L/R` (`Ø1.9` pins at `[33.0, ±8.0]`) with tie-rod arms extending forward to `[39.5, ±8.0]` (`Z = 2.2..3.7 mm`) and `Bell_Crank_Drag_Ear` on `L`.
  - `Steering_Ackermann_Bar`: links `[37.0, +6.4]` to `[37.0, -6.4]` at `Z = 4.0..5.0 mm`.
  - `Drag_Link`: connects `Servo_Horn` (`[28.8, 7.8, 9.25]`) to `Bell_Crank_Drag_Ear` (`[35.4, 8.3, 6.15]`).
  - `Tie_Rod_L/R`: `24.63 mm` center-to-center (`[39.5, ±8.0, 4.65]` -> `Knuckle_TieRod_Boss` `[42.0, ±32.5, 4.65]`), exiting `IFS_Bulkhead_Convertible` through `Steering_Window_Cut_L/R` (`X=36.5..41.7, Y=±4.2..±13.5, Z=2.0..5.6`) and running nearly parallel to `IFS_Lower_Arm_L/R` for low bump steer.
- **Ackermann Geometry (4-bar solve):**
  - Bellcrank `±20°` -> Inner wheel **`31.40°`**, Outer wheel **`20.20°`** (`11.20°` Ackermann toe-out on turn).
  - Bellcrank `±15°` -> Inner wheel **`20.60°`**, Outer wheel **`15.60°`** (`5.00°` Ackermann).

## 2. Baseline Rest Interference Check (`01 + 02 + 03 + 04 + 05`)
- **Result:** **`0` interferences** across all bodies in `01 + 02 + 03 + 04 + 05` (`full_rest_out.json`), `0` errors in `05` (`10/10` closed bodies).

## 3. Pose & Interference Analysis Table
| Pose | Configuration | Travel / Angle | Shock Eye-to-Eye | Interference Count | Key Findings |
|---|---|---|---|---|---|
| **0. Rest (Ride Height)** | `01+02+03+04+05` static | `0 mm` / `0°` | Front `20.84 mm`, Rear `27.66 mm` | **0** | Clean baseline clearance across all 44 bodies. |
| **1. Front Steer Left** | `bc = -20°` at ride height | L `+31.4°` (in), R `+20.2°` (out) | `20.84 mm` | 16 (pre-window-fix; `Tacoma_Trophy_Body_Shell` front arch `19.94 mm³`, `IFS_Lower_Arm_L` `0.82 mm³`, `Tie_Rod_L` `0.92 mm³`) | Front `Ø36` tire clips `Tacoma_Trophy_Body_Shell` front wheel arch at `31.4°` lock and lightly touches `IFS_Lower_Arm_L` / `Tie_Rod_L` inner edge. |
| **2. Front Steer Right** | `bc = +20°` at ride height | L `-20.2°` (out), R `-31.4°` (in) | `20.84 mm` | 16 (symmetric) | Symmetric with Pose 1 (`Tacoma_Trophy_Body_Shell` `19.94 mm³`). |
| **3. Front Full Bump** | `arm = +22°`, `0°` steer | `+8.20 mm` (`wheel_Z = +11.20 mm`) | `17.03 mm` (SCX30 24mm min) | 21 | `IFS_Upper_Arm_L/R` hits `Tacoma_Trophy_Body_Shell` underside (`3.80 mm³` at `Z=14.87`); `Ref_Shock_FL/FR` tilts into bulkhead (`3.47 mm³`). |
| **4. Front Bump + Steer Left** | `arm = +22°`, `bc = -20°` | `+8.20 mm`, L `+31.4°` | `17.03 mm` | 24 | `Ref_Wheel_FL` severely clips `Tacoma_Trophy_Body_Shell` fender (`84.49 mm³` at `Z=15.21`) and `IFS_Lower_Arm_L` (`9.01 mm³`). |
| **5. Front Full Droop** | `arm = -23°`, `0°` steer | `-8.60 mm` (`wheel_Z = -5.60 mm`) | `23.99 mm` (SCX30 24mm max) | 13 | `IFS_Upper_Arm_L/R` hits `Ref_Front_CVD_L/R` (`6.18 mm³`); `IFS_Lower_Arm` touches `IFS_Bulkhead` edge (`0.0006 mm³`). |
| **6. Front Droop + Steer Left** | `arm = -23°`, `bc = -20°` | `-8.60 mm`, L `+31.4°` | `23.99 mm` | 18 | Same CVD/arm droop limits + `Ref_Wheel_FL` vs `Tie_Rod_L` (`6.00 mm³`). |
| **7. Rear Full Bump** | `ta = +11.4°` (SCX24 32mm compressed) | `+10.14 mm` (`wheel_Z = +11.64 mm`) | `22.00 mm` | 21 | **Axle housing** hits `Rear_Sway_Bridge` (`279.0 mm³`), `Chassis_Rail_L/R` (`27.11 mm³`), `Roll_Cage` (`125.9 mm³`), `Tacoma_Trophy_Body_Shell` (`203.2 mm³`), `Spare_Tire_Carrier` (`11.57 mm³`). **Rear tires (`Ref_Wheel_RL/RR`) have 0 interferences.** |
| **8. Rear Full Droop** | `ta = -9.0°` (SCX24 32mm extended) | `-8.01 mm` (`wheel_Z = -6.51 mm`) | `32.03 mm` | **0** (excluding pre-fix pin overlap) | **Completely clear** across all rear links, axle, tires, body, cage, and sway bar. |
| **9. Rear Cross-Articulation** | L `+11.4°` bump / R `-9.0°` droop | `27.08°` axle roll | L `22.00 mm`, R `32.03 mm` | 15 | Left bump side hits `Rear_Sway_Bridge` (`86.88 mm³`), `Chassis_Rail_L` (`51.17 mm³`), `Tacoma_Trophy_Body_Shell` (`99.11 mm³`); right droop side clear except lower arm/axle roll binding. |

## 4. Travel, Shock, CVD, and Driveshaft Summary
- **Front Suspension (SCX30 24mm Shock):**
  - Full shock travel: **`16.80 mm`** vertical wheel travel (`-8.60 mm` droop at `23.99 mm` shock to `+8.20 mm` bump at `17.03 mm` shock; ride height `20.84 mm`).
  - Front CVD distance (`[44.0, ±16.1, 4.95]` to outer hub): **`12.29 mm`** (`+10°`) to **`14.84 mm`** (`-23°` droop), **`12.61 mm`** at ride height (`2.55 mm` total plunge).
  - Recommended bump-stop / steering-endpoint limits without trimming `Tacoma_Trophy_Body_Shell` wheel arches: ~`±20°` steer / `+4.5 mm` front bump.
- **Rear Suspension (SCX24 32mm Shock):**
  - Full shock travel: **`18.15 mm`** vertical wheel travel (`-8.01 mm` droop at `32.03 mm` shock to `+10.14 mm` bump at `22.00 mm` shock; ride height `27.66 mm`).
  - Physical up-travel bump stop: `Rear_Sway_Bridge` (`Z_min = 3.5 mm`) and `Chassis_Rail_L/R` sit `~3.0 mm` above `Ref_Rear_Axle` at ride height (`ta ≈ +3.5°`, `shock ≈ 25.9 mm`). Rear droop (`-8.01 mm` at `32.03 mm` shock) is **100% interference-free**.
- **Rear Driveshaft (`Ref_Rear_Driveshaft`, updated in `04_wheels_and_rear_axle_ref.json`):**
  - Transmission output `[-10.0, 0.0, 5.5]` to axle pinion `[-46.7, 0.0, 1.5]`:
  - Required length range over full rear travel (`-9.0°` droop to `+11.4°` bump): **`L_min = 36.14 mm`** (bump) to **`L_max = 38.04 mm`** (droop); ride height = **`36.92 mm`**.
  - **Previous design** (`20.0 mm` outer sleeve + `17.42 mm` inner shaft): only `0.50 mm` (`2.9%`) engagement at ride height and **pulled apart (`-0.62 mm` gap)** at full droop!
  - **Redesigned in `04_wheels_and_rear_axle_ref.json`** (`24.0 mm` Ø4.0 outer sleeve + `23.0 mm` Ø2.6 inner shaft):
    - **Min engagement (full droop, `38.04 mm`):** `24.0 + 23.0 - 38.04 = 8.96 mm` (**`39.0%` >= 30%**).
    - **Max compression slack (full bump, `36.14 mm`):** inner shaft tip sits `13.14 mm` from transmission output (**`13.14 mm` >> 0.5 mm**).
