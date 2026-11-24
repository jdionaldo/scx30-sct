# Axial SCX30 1/30 Short Course / Trophy Truck Conversion — Engineering & Build Guide

**Platform:** Axial SCX30 (107 mm Tacoma Long-Wheelbase Stretch)  
**Architecture:** Convertible Path C (2WD High-Speed Trophy Truck) / Path A (4WD Prerunner)  
**Target Top Speed:** ~8.5–9.8 mph (~255–295 scale mph at 1:30) vs. 0.9 mph stock

> **Status 2026-11-23:** this guide is the early engineering write-up and is out of date in places (part names, rails, the 7 STL sets). The current kit is 120 bodies, 28 printed parts on 6 plates; ASSEMBLY.md and parts_list.md are the current references. The cage has 4 M2 x 6 screws (not the 12 of earlier rounds), and the cage door-bay posts sit in pegged pockets.

---

## 1. Architecture Overview & CAD Coordinate System

This conversion transforms the 1/30-scale Axial SCX30 crawler into a high-speed desert Short Course / Rock-Racer Trophy Truck combining the **FMS FSC24 Chaser** suspension layout (independent double-wishbone front + long-travel trailing-arm solid rear axle) with an **Axial Yeti (AXIC9025)** aesthetic overhaul, stretched to the **107.0 mm SCX30 Toyota Tacoma** wheelbase.

### Coordinate Reference (Used Across All CAD & STL Files)
* **Origin `[0, 0, 0]`:** Centered on the SCX30 center transmission skid plate.
* **Axes:** `+X` = Forward, `+Y` = Left, `+Z` = Up (symmetric across the `XZ` plane).
* **Axle Centerlines:**
  * **Front IFS Axle Line:** `X = +48.0 mm`
  * **Rear Solid Axle Line:** `X = -59.0 mm`
  * **Wheelbase:** `48.0 - (-59.0) = 107.0 mm` (vs. 99 mm stock SCX30 Bronco/Jeep).
* **Track Width (~85 mm):** Outer knuckle bearing faces (`IFS_Knuckle_L/R`) sit at `Y = ±38.5 mm` (**77.0 mm** knuckle-to-knuckle), bringing overall tire-to-tire outer track width with `Ø36.0 × 13.5 mm` BFGoodrich-style rock-racer tires (`Ref_Wheel_*`) to **85.0 mm** (`Y = ±29.0..42.5 mm`), leaving front and rear tires 100% exposed outside the tight hugging Yeti body (`Y = ±15.6..22.8 mm`).
* **Overall Printable Kit Envelope:** **126.50 mm** chassis/stinger-bumper length (`X = -65.00` to `+61.50 mm`) × **77.00 mm** knuckle-to-knuckle width (`Y = ±38.50 mm`, body width **45.60 mm** at `Y = ±22.80 mm`) × **35.60 mm** structural height (`Z = -4.20` to `+31.40 mm`). (With `Ø36.0 mm` reference wheels/tires: `142.98 × 85.00 × 47.89 mm`.)

### Sub-System Design Highlights (Blue & White Axial Yeti AXIC9025 Aesthetic)
1. **3D Boat-Side Tub Sliders & LCG Chassis (`Chassis_Rail_L/R`, `Slider_L/R`):**
   * Twin 2.5 mm `AX31103`/`AX31104`-style sculpted LCG rails (`Y = ±13.0–15.5 mm`) with angled front/rear lower boat-tail bevels, truss lightening windows, flat `Z = +8.0 mm` upper cage mounting deck, and angled `AX31102` rear shock hoop gusset (`X = -16..-60 mm`, peak `X = -36.5, Z = +20.47 mm` with dual `Ø1.6 mm` shock holes and 2 triangular truss windows).
   * 3D Axial Yeti `AX31103` boat-side tub walls (`Slider_L/R`: `X = -14.2..+24.0 mm`, `Y = ±15.5..20.5 mm`, `Z = -2.0..+6.5 mm`) with `1.4 mm` wall/floor cavity, lower boat-side bevel, sharp angled rear boat-tail step, 3 vertical upper-link tuning holes (`X = -12.5 mm`), and trailing-arm pivot clearance.
2. **Chamfered-Square Billet Convertible Front IFS (`IFS_Bulkhead_Convertible`, `IFS_Lower_Arm_L/R`, `IFS_Upper_Arm_L/R`, `IFS_Knuckle_L/R`):**
   * Beveled sub-micro steering servo pocket (`X = +25.5` to `+39.5 mm`), tucked `AX31111` A-frame front shock tower (`Z <= +14.2 mm`, upper shock holes at `Y = ±13.5, Z = +12.5 mm`) with two `Ø2.0 mm` vertical body-post pins (`X = +46.5, Y = ±9.5, Z = +14.2..+17.0 mm`) that pass into the hood, and a compact narrow `AX31112` prerunner stinger bumper (`Y = ±14.5 mm`, `X = +54.0..+61.5 mm`, `Z = +5.0..+12.5 mm`) + angled skid (`Y = ±8.5 mm`, `Z = -2.0..+5.0 mm`), leaving the front gullwing A-arms and tires completely open.
   * **Chamfered-Square Billet Wishbones:** `IFS_Lower_Arm_L/R` (`0.35 mm` 45° longitudinal top/bottom chamfers + chamfered-square shock pedestal at `Z <= +4.4 mm`) and `IFS_Upper_Arm_L/R` (`0.30 mm` 45° chamfers around all top/bottom outer & inner A-arm window edges) on `1.5 mm` steel hinge pins (`Ø1.6 mm` CAD bores), paired with `AX31110`-style `IFS_Knuckle_L/R` uprights (`Ø6.4 mm` bearing hubs, `Ø4.0 mm` Y-bores, angled Ackermann arms).
   * **Convertible Worm-Gear Cavity (`X = +43.0` to `+53.0 mm`, `Y = ±6.0 mm`):** Left **empty** in **Path C (2WD)** so the `IFS_Knuckle_L/R` hubs spin freely on `Ø4.0 mm` OD ball bearings with zero worm friction; accepts an SCX30 `2/12T` worm gear + CVDs/dogbones for **Path A (4WD)** conversion.
3. **Chamfered-Square Billet Trailing Arms & `AX31125` Sway Bridge (`Rear_Trailing_Arm_L/R`, `Rear_Upper_Link_L/R`, `Rear_Sway_Bridge`):**
   * **51.25 mm** eye-to-eye `AX31109`+`AX31245` **chamfered-square billet** trailing arms (`X = -7.75` to `-59.0 mm`, `Z = -4.2..+4.9 mm`) with square/45°-chamfered pivot ends, trapezoidal billet sway-bar tab (`X = -46.75, Z = +3.25 mm`), `0.4 mm` 45° perimeter chamfers on inner/outer faces, octagonal `AX31245` recessed outer pocket with octagonal dual-shock boss (`X = -36.5 / -39.5 mm`), and octagonal truss windows, paired with rounded **46.5 mm** eye-to-eye `Rear_Upper_Link_L/R` (`X = -12.5` to `-59.0 mm`, `Z = +5.25 mm`).
   * **Rear Anti-Sway Bridge:** `AX31125`-inspired `Rear_Sway_Bridge` (`X = -65.0` to `-58.5 mm`, `Z = +3.5..+10.2 mm`) with fuel-cell top cover, filler cap boss, sway-bar clamp saddles, and `Ø1.2 mm` torsion tunnel for `0.8 mm` piano wire.
4. **Sleek Low-Slung Axial Yeti Spaceframe & Hugging Body (`Roll_Cage`, `Tacoma_Hood_Roof_Shell`, `Tacoma_Fender_L/R`):**
   * `AX31115`/`AX31116`/`AX31117` fastback tube spaceframe (`Z = +8.09..+30.01 mm`) with ~42° raked A-pillars, windshield center V-strut, under-hood nose cage, roof X-brace, and fastback kicker bars.
   * **Closed Sleek White Yeti Hood & Roof (`Tacoma_Hood_Roof_Shell`, `Z = +8.40..+31.40 mm`):** Solid continuous hood over the tucked front shock tower with recessed central & corner vents, `Ø2.4 mm` body-post holes at `[46.5, ±9.5]`, pinched Yeti nose behind the compact stinger bumper, and sleek low-slung roof panel.
   * **Tall Tight-Hugging Cyan-Blue Side Panels (`Tacoma_Fender_L/R`, `Y = ±15.6..22.8 mm`, `Z = -2.0..+23.5 mm`):** `17.8 mm` tall side body (`Z = -2.0` to `+15.8 mm` window sill) with diagonal front/rear wheel-arch cutouts leaving all 4 wheels exposed, plus vertical trapezoidal **`"380"` number-plate tail fins** rising to `Z = +23.5 mm` over the rear wheels.

---

## 2. Power, Gearing & Top-Speed Engineering Math

### Why Path C (2WD) + Overdrive Worm + Longitudinal Metal Transmission?
* **Stock SCX30 Baseline:** Uses a brushed `030` 88T motor on 2S (`~29,700 RPM` est.), an `11T` `0.3 MOD` pinion, a transverse `AXI-3311` transmission (`16.36:1` internal gear ratio using `30T` transfer / `40T` idler gears), and `2/16T` (`8.0:1`) worm axles for a **`131.58:1` Final Drive Ratio (FDR)** and **~0.9 mph** top speed (`~27 scale mph`).
* **3S Brushless Multiplier:** Upgrading to the **GACRacing GM1614 (7500KV)** 14-pole brushless outrunner + **G40 Pro Nano ESC** on **3S LiPo** (`11.1V` nominal / `12.6V` fresh peak) raises unloaded motor speed to **83,250–94,500 RPM** (~2.8–3.2× stock motor speed).
* **Axle & Pinion Speedup:** Swapping the rear worm gear to **B0GX5L4J64 `2/12T`** drops the axle ratio from `8.0:1` to **`6.0:1`** (**+33.3%** wheel speed). Stepping from an `11T` to a **`14T` or `15T` `0.3 MOD`** pinion adds another **+27.3% to +36.4%** speed.
* **Path C (2WD) Parasitic Drag Elimination:** Because worm gears rely on sliding rather than rolling mesh, spinning two worm axles at high speed wastes substantial power in viscous drag and heat. Leaving the front IFS differential cavity empty in **Path C (2WD)** cuts worm-gear friction in half and stops front-axle scrubbing in high-speed turns.

### Complete Top-Speed Math Table

> **Note on `B0HMR5D54N` Metal Longitudinal Transmission Ratios:** Aftermarket longitudinal SCX30 gearboxes eliminate the stock transverse transfer stage and typically run a lower internal reduction (~`6.0:1–6.5:1` with `14T/15T` pinions, **Regime A** below) compared to stock-ratio `16.36:1`-scaled gearboxes (**Regime B** below). Verify your unit's ratio with the **10-Turn Bench Test** immediately below the table.

$$\text{Wheel RPM} = \frac{\text{KV} \times V_{\text{pack}} \times \eta_{\text{load}}}{\text{FDR}}, \qquad \text{Speed (mph)} = \frac{\text{Wheel RPM} \times (\pi \times D_{\text{tire, mm}}) \times 60}{1{,}609{,}344}$$

*(Calculations assume `90%` under-load RPM retention $\eta_{\text{load}} = 0.90$ for realistic loaded ground speed; theoretical no-load peak shown in parentheses. Stock Tacoma tire OD = `34.0 mm`; optional micro SCT tire OD = `38.0 mm`.)*

| Build Configuration | Motor / Power | Pinion (`0.3M`) | Trans. Ratio | Rear Axle Worm | Total FDR | Tire OD | Wheel RPM (Loaded) | Ground Speed (Loaded / No-Load) | 1:30 Scale Speed |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Stock SCX30 (Baseline 4WD)** | 88T 030 Brushed (2S ~7.4V) | 11T | `16.36:1` | `2/16T` (`8.0:1`) | **`131.58:1`** | 34 mm | ~226 RPM | **0.90 mph** *(ref.)* | **27 mph** |
| **Path C — Regime A (14T, 3S Nom)** | GM1614 7500KV (3S `11.1V`) | **14T** | `~6.43:1` | `2/12T` (`6.0:1`) | **`38.57:1`** | 34 mm | 1,943 RPM | **7.74 mph** *(8.60 no-load)* | **232–258 mph** |
| **Path C — Regime A (14T, 3S Peak)** | GM1614 7500KV (3S `12.6V`) | **14T** | `~6.43:1` | `2/12T` (`6.0:1`) | **`38.57:1`** | 34 mm | 2,205 RPM | **8.78 mph** *(9.76 no-load)* | **263–293 mph** |
| **Path C — Regime A (15T, 3S Nom)** | GM1614 7500KV (3S `11.1V`) | **15T** | `~6.00:1` | `2/12T` (`6.0:1`) | **`36.00:1`** | 34 mm | 2,081 RPM | **8.29 mph** *(9.21 no-load)* | **249–276 mph** |
| **Path C — Regime A (15T, 3S Peak)** | GM1614 7500KV (3S `12.6V`) | **15T** | `~6.00:1` | `2/12T` (`6.0:1`) | **`36.00:1`** | 34 mm | 2,363 RPM | **9.41 mph** *(10.45 no-load)* | **282–314 mph** |
| **Path C — Regime A (14T, 38mm SCT)**| GM1614 7500KV (3S `12.6V`) | **14T** | `~6.43:1` | `2/12T` (`6.0:1`) | **`38.57:1`** | 38 mm | 2,205 RPM | **9.81 mph** *(10.90 no-load)*| **294–327 mph** |
| *Regime B Ref. (Stock-Scaled IGR, 14T)*| GM1614 7500KV (3S `12.6V`) | 14T | `12.86:1` | `2/12T` (`6.0:1`) | `77.14:1` | 34 mm | 1,103 RPM | **4.39 mph** *(4.88 no-load)* | 132–146 mph |
| *Regime B Ref. (Stock-Scaled IGR, 15T)*| GM1614 7500KV (3S `12.6V`) | 15T | `12.00:1` | `2/12T` (`6.0:1`) | `72.00:1` | 34 mm | 1,181 RPM | **4.70 mph** *(5.23 no-load)* | 141–157 mph |

#### Quick 10-Turn Bench Ratio Check for `B0HMR5D54N`
Before first drive, mark the output yoke of your assembled `B0HMR5D54N` + `14T/15T` pinion with a paint dot and spin the GM1614 motor bell **exactly 10 revolutions** by hand:
* **~1.55–1.67 output turns:** Your gearbox is in **Regime A (`~6.0:1–6.43:1`)** $\rightarrow$ expect **~8.5–9.8 mph (~295 scale mph)** on 3S!
* **~0.78–0.83 output turns:** Your batch ships with **Regime B (`~12.0:1–12.9:1`)** crawler gears $\rightarrow$ expect ~4.4–5.2 mph (still ~5× stock speed with massive wheelie torque; ideal if running heavier 38–40 mm tires or tight backyard tracks).

---

### CRITICAL ESC SETTING: Why G40 Pro Drag Brake MUST Be `0%`

> [!CAUTION]
> **NEVER run electronic Drag Brake (`>0%`) on a high-speed worm-geared SCX30.** Set **Drag Brake = `0%`** in the GACRacing Bluetooth App *before* your first test run.

1. **Worm-Gear Self-Locking Physics:** A `2/12T` worm gear has a shallow helix lead angle, making it mechanically **non-back-drivable**. At `0.9 mph` crawling speeds, stopping the motor locks the axle cleanly to hold steep inclines.
2. **Shock-Load Snap / Front-Flip at ~9 mph:** At `8.5–9.8 mph` (~2,200+ wheel RPM), releasing the throttle with any active ESC Drag Brake commands the G40 Pro's FOC stage to clamp the GM1614 rotor dead-stop. Because the worm wheel *cannot* back-drive the stationary worm shaft, all rotational and forward kinetic energy hits as an instantaneous mechanical shock spike:
   * **Immediate Endos / Rollovers:** Rear tires lock solid on pavement/packed dirt, pitching the 1/30 chassis forward into high-speed cartwheels.
   * **Drivetrain Failure:** Shock torque concentrates directly on the tiny **2-start worm teeth**, the composite **`AXI-4428` driveshaft U-joints**, and the `0.3 MOD` pinion teeth—shearing pins or galling bronze/steel gears.
   * **Thermal Spike:** Regenerated braking current dumps heat straight into the GM1614 windings and G40 Pro MOSFETs.
3. **Mandatory GACRacing App Checklist (iOS / Android):**
   * Connect via the **G40 Pro's built-in Bluetooth**.
   * Set **Drag Brake Force = `0%`**.
   * **Disconnect or Zero the Yellow AUX Wire:** The G40 Pro supports *real-time drag-brake override* via its yellow auxiliary signal lead. Leave the yellow AUX lead **unplugged** from your receiver (or bind that AUX channel's transmitter endpoint/knob strictly to **0%**) so your radio cannot re-enable drag brake mid-run!
   * Set **LiPo Cell Count / LVC** for **3S** (`~3.3–3.4 V/cell` = `9.9–10.2 V` cutoff).
   * Set **BEC Voltage** to **`6.0 V`** for a standard micro steering servo (select `7.4 V` *only* if your servo is explicitly rated for HV 7.4 V).

---

## 3. Bill of Materials & Hardware Checklist

### A. Core Kit, Drivetrain & Power Parts
| Qty | Component / Part ID | Specification & Notes |
| :---: | :--- | :--- |
| 1 | **Axial SCX30 Donor Platform** | Provides rear axle housing, transmission skid base, rear wheel hexes, and stock hardware base. |
| 1 | **`AXI-4428` Axial Driveshaft Set** | 107 mm Tacoma Long-Wheelbase extended driveshaft set. Install the **long rear shaft** for Path C 2WD; store the front shaft for Path A 4WD. |
| 1 | **`B0HMR5D54N` Metal Transmission** | Longitudinal CNC metal gearbox for SCX30; mount inline on the central skid plate (`X = 0`). |
| 1 | **`B0GX5L4J64` INJORA 2/12T Overdrive Worm Gear Set** | Pack of 2 gear sets (`6.0:1`). Install 1 set in the **rear axle** for Path C 2WD; reserve the 2nd set for the front IFS bulkhead when converting to Path A 4WD. |
| 1 ea | **`14T` & `15T` `0.3 MOD` Pinion Gears** | Match bore to GM1614 motor shaft (`1.5 mm` typical—verify with calipers). Start with `14T` for cooler motor temps; move to `15T` for peak speed. |
| 1 | **GACRacing `GM1614` (7500KV) Motor** | 14-pole micro brushless outrunner, 2S–3S capable, PH2.0 3P lead. |
| 1 | **GACRacing `G40 Pro` Nano ESC** | 40A IP67 FOC brushless ESC with built-in Bluetooth and 5A switchable 6V/7.4V BEC. Mount on `Slider_L` or `Slider_R`. |
| 1 | **3S `300 mAh` LiPo Pack** | Compact block pack (e.g., ~`42 × 19 × 18 mm` JST-RCY or micro FPV ~`48 × 17 × 16 mm` XT30). Mount on opposite `Slider` using the twin `3.6 mm` strap slots (`Y = ±30.25 mm`) for left/right balance; verify plug matches G40 Pro lead. |
| 1 | **Standalone Micro Receiver** | 3-channel micro PWM receiver matching your aftermarket transmitter (e.g., **Flysky Nano GT5 / AFHDS2A** decased micro RX, or standalone 3-wire PWM RX). *Stock Axial SLT 2-in-1 ESC/RX cannot run brushless.* |
| 1 | **Sub-Micro Steering Servo** | Fits beveled pocket in `IFS_Bulkhead_Convertible` (**`14.0 mm L × 23.0 mm W × 8.5 mm D`** at `X = +25.5..+39.5 mm, Z = +2.0..+10.5 mm`; e.g., stock SCX30 servo or compact ~`22.8 × 12 mm` micro servo). |
| 4 | **Wheels & Tires (`0.7"` SCX30)** | Stock `34 mm` Tacoma tires fit cleanly inside wheel arches; `36–38 mm` micro SCT tires give higher top speed (verify full-bump arch clearance). |

### B. Suspension, Hinge Pins, Sway Bar & Micro Fasteners
*(Every diameter below matches the CAD bores verified in Scripts 01–03.)*

| Qty | Hardware Item | CAD Location & Purpose |
| :---: | :--- | :--- |
| 4 | **Oil Shocks (2× SCX24 `32 mm` Rear + 2× SCX30 `24 mm` Front)** | **Rear:** Off-the-shelf **SCX24 `32 mm` oil shocks** (`27.7 mm` static eye-to-eye from `[-26.0, ±17.75, 21.5]` to `[-44.0, ±17.75, 0.5]`, `Ø1.6 mm` mount holes). **Front:** Stock/aftermarket **SCX30 `23–24 mm` oil shocks** (`20.8 mm` static eye-to-eye from `[48.0, ±6.0, 13.0]` to `[48.0, ±24.0, 2.5]`). Fill with **`10wt–20wt` silicone oil**. |
| 4 | **`Ø1.5 mm` Stainless Hinge Pins** | Cut from `1.5 mm` hardened stainless dowel/music wire for zero-slop `Ø1.6 mm` IFS bores:<br>• **2× Lower Inner:** `~22 mm` long (`X = +39.8..62.5 mm`, `Y = ±7.5..10.5, Z = 0.0`)<br>• **2× Upper Inner:** `~17 mm` long (`X = +39.0..56.5 mm`, `Y = ±8.5..11.6, Z = +7.0`)<br>• **4× Outer Knuckle Pins:** `~10–12 mm` long (`Y = ±32.5` lower / `±30.9` upper). |
| 1 | **`Ø0.8 mm` Piano / Music Wire** | ~`80 mm` length for rear anti-sway bar: threads through `Ø1.2 mm` tunnel in `Rear_Sway_Bridge` & `Chassis_Rail_L/R` (`26–31 mm` cross-span at `X = -63.0, Z = +6.0`), bent 90° forward (`16.25 mm` arms) to `Rear_Trailing_Arm_L/R` tabs (`X = -46.75, Z = +3.25`). |
| 4 | **`2 × 4 × 1.5 mm` or `1.5 × 4 × 2 mm` Flanged Bearings** | Press-fit into `IFS_Knuckle_L/R` `Ø4.0 mm` bores (`X = +48.0, Z = +3.0`) for frictionless Path C free-rolling front hubs (+ 2 extra `4 mm` OD bearings for Path A bulkhead outdrives). |
| 2 | **`Ø1.5 × Ø2.0 × 3.5 mm` Brass/Nylon Bushings** | Adapts `Rear_Trailing_Arm_L/R` `Ø2.0 mm` pivot bores (`X = -7.75` and `-59.0`) to `M1.4` screws without binding or slop. |
| 2 | **Micro Steering Tie-Rods + Sway Drop Links** | `M1.4` threaded rod + micro rod ends (servo horn $\rightarrow$ `IFS_Knuckle_L/R` Ackermann holes at `X = +42.0, Y = ±32.5`; plus 2 short links from sway bar arms to `Rear_Trailing_Arm_L/R` tabs). |
| 16 | **`M1.4 × 5 mm` Socket/Button Screws** | `Chassis_Rail` $\leftrightarrow$ `IFS_Bulkhead` (`[27, 2.5]`, `[33, 5.0]`), skid plate (`[9.5, 0]`), `Roll_Cage` mounting, fender mounts. |
| 16 | **`M1.4 × 8 mm` Socket/Button Screws** | `Rear_Upper_Link_L/R` pivots (`[-12.5, 5.25]`, `[-59.0, 5.25]`), `Rear_Trailing_Arm` front pivot (`[-7.75, -0.5]`), shock pocket (`[-38.5, -1.0]`), front/rear shock tower mounts, IFS lower arm shock bosses (`[48, ±22, 3]`). |
| 6 | **`M1.4 × 12 mm` Socket/Button Screws** | Long rear trailing-arm through-bolts at axle (`[-59.0, -0.5]`) and rear sway-bridge tie-ins. |
| 20+ | **`M1.4` Nyloc Nuts & Micro Washers** | Retain all suspension pivots, shock bolts, and steering tie-rod ball studs (use blue threadlocker on metal-to-metal). |
| 2 | **Micro Velcro Battery Straps (`3.5 mm` wide)** | Thread through `Slider_L/R` twin strap slots (`Y = ±30.25 mm`, `3.6 mm` wide) to secure 3S LiPo & G40 Pro ESC. |

---

## 4. 3D Printing & Post-Processing Guide (All 7 STL Sets)

> [!IMPORTANT]
> **Split Multi-Body STLs Before Slicing!** Several STL files contain multiple bodies positioned in their 3D assembly coordinates (for instance, `Rear_Sway_Bridge` floats at `Z = +3.5 mm` above `Slider_L/R` at `Z = -2.0 mm`). In PrusaSlicer, Bambu Studio, or OrcaSlicer, right-click each STL and select **Split $\rightarrow$ To Objects**, then **Lay on Face** so every part prints flat with maximum layer strength.

### Recommended Materials
* **Structural Chassis, IFS Bulkhead, Arms & Knuckles:** **PA12-CF / PAHT-CF (Carbon Nylon)**, **PETG-CF**, or **ABS/ASA** (or **Tough / ABS-like Engineering Resin** such as Siraya Tech Blu on MSLA printers). **Do NOT print the chassis, bulkhead, or transmission mounts in standard PLA**—3S brushless heat from the GM1614/G40 Pro (>55°C) will warp PLA, and standard brittle resin will crack on 9 mph impacts.
* **Baja Fenders & Front Skid Bumper Impact Zones:** **PETG**, **ASA**, or **TPU 95A** (TPU fenders survive cartwheels almost indefinitely).

### All-PETG Print Profiles by Component

| Setting | Chassis Rails, Links & Towers | Bumpers & Nerf Bars | Short Course Body Shell |
| :--- | :--- | :--- | :--- |
| **Layer Height** | `0.12mm – 0.16mm` | `0.16mm` | `0.16mm` |
| **Wall Loops (Perimeters)** | **4 – 5 walls** (Maximum rigidity) | **3 walls** (Allows slight flex) | **2 walls** (Keeps weight low) |
| **Infill Density & Pattern** | `40% Gyroid` | `15% – 20% Gyroid` | `10% Lightning or Gyroid` |
| **Nozzle Temp** | `245°C – 255°C` | `245°C` | `235°C – 240°C` |
| **Cooling Fan Speed** | `0% – 15%` | `20%` | `30% – 50%` (Better overhangs) |
| **Print Speed** | `40 – 50 mm/s` | `50 mm/s` | `60 mm/s` |

### All 7 STL Sets — Dimensions, Orientation & Print Profiles

| # | STL File | Contained Bodies & Bounding Envelope | Recommended Print Orientation & Supports | Layer / Walls / Infill |
| :-: | :--- | :--- | :--- | :--- |
| **1** | [`lcg_chassis_rails_and_sliders.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/lcg_chassis_rails_and_sliders.stl) | `Chassis_Rail_L/R`, `Slider_L/R`, `Rear_Sway_Bridge`<br>`101.0 × 41.0 × 22.5 mm` (`12,780` tris, `639,084` B) | **Split to 5 objects:** Lay `Chassis_Rail_L/R` flat on `Y` outer faces; lay `Slider_L/R` & `Rear_Sway_Bridge` flat on `Z` bottom faces. **Zero supports needed.** | `0.12–0.16 mm` layers, 4–5 walls, `40% Gyroid` (or `100%` for max rigidity). |
| **2** | [`rear_trailing_arms_107mm.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/rear_trailing_arms_107mm.stl) | `Rear_Trailing_Arm_L/R` (chamfered-square billet)<br>`55.95 × 39.0 × 9.1 mm` (`6,860` tris, `343,084` B) | **Split to 2 objects:** Lay flat on `Y` inner face (`Z = -4.2..+4.9`); use tiny support only inside the dual-shock pocket if bridging droops. | `0.12 mm` layers, 4–5 walls, `40% Gyroid` / `100%` infill. |
| **3** | [`rear_upper_links_107mm.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/rear_upper_links_107mm.stl) | `Rear_Upper_Link_L/R`<br>`49.8 × 24.0 × 3.3 mm` (`4,752` tris, `237,684` B) | **Split to 2 objects:** Lay flat on `Y` or `Z` face (`3.3 mm` high). **Zero supports.** | `0.12 mm` layers, 4–5 walls, `40% Gyroid` / `100%` infill. |
| **4** | [`ifs_front_bulkhead_convertible.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/ifs_front_bulkhead_convertible.stl) | `IFS_Bulkhead_Convertible` (tucked tower + narrow stinger)<br>`37.5 × 31.6 × 19.0 mm` (`127,460` tris, `6,373,084` B) | Print **upright on `Z = -2.0` bottom** with **Tree Supports (build plate only)** for servo pocket, worm cavity, and `AX31112` stinger bumper overhang—OR print on MSLA Tough Resin tilted 30° nose-up. | `0.12 mm` layers, 4–5 walls, `40% Gyroid` / `100%` infill (or 0.05 mm SLA). |
| **5** | [`ifs_arms_and_knuckles.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/ifs_arms_and_knuckles.stl) | `IFS_Lower_Arm_L/R`, `IFS_Upper_Arm_L/R` (chamfered-square billet), `IFS_Knuckle_L/R`<br>`13.3 × 77.0 × 10.2 mm` (`65,148` tris, `3,257,484` B) | **Split to 6 objects:** Print `Upper Arms` flat on `Z` bottom; print gullwing `Lower Arms` with light tree supports under `Z = -2.2` drop; print `IFS_Knuckle_L/R` on `Y` outer face with tree supports under Ackermann arm. | `0.12 mm` layers, 4–5 walls, `40% Gyroid` / `100%` infill. |
| **6** | [`roll_cage.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/roll_cage.stl) | `Roll_Cage`<br>`99.42 × 30.92 × 21.92 mm` (`332,456` tris, `16,622,884` B) | Print **upside-down** (`Z = +30.01` roof rails on bed) with tree supports to hood bars, V-strut, and rear kicker tubes, **or MSLA Tough Resin** for cleanest `1.8–2.4 mm` Yeti spaceframe tubes. | `0.12–0.16 mm` layers, 3 walls, `15–20% Gyroid` (or `60–100%` for bash durability). |
| **7** | [`tacoma_trophy_shell.stl`](file:///Users/johndionaldo/.gemini/antigravity/scratch/scx30_sct/stl/tacoma_trophy_shell.stl) | `Tacoma_Trophy_Body_Shell` (unified single-piece shell: hood, roof, fenders, `"380"` fins & rear shock pockets)<br>`112.0 × 45.6 × 33.4 mm` (`7,240` tris, `362,084` B) | **Single watertight manifold body (`shells=1`):** Print upside-down (`Z = +31.4` roof on bed) or upright at 35° nose-up with tree supports under overhangs. | `0.16 mm` layers, 2 walls, `10% Lightning or Gyroid`, `235–240°C`, `30–50%` fan, `60 mm/s`. |

### Post-Processing & Bore Calibration Checklist
Run a pin-vise hand drill through every printed bore *before* test-fitting pins or screws (never force pins into undersized FDM holes, which splits layers):
* **`Ø1.2 mm` bit:** Clear `Rear_Sway_Bridge` (`X = -63.0, Z = +6.0`) and `Chassis_Rail_L/R` sway-bar tunnel so `0.8 mm` piano wire rotates freely with zero drag.
* **`Ø1.5 mm` / `Ø1.6 mm` bit:** Ream all hinge-pin bores (`IFS_Bulkhead_Convertible`, `IFS_Lower_Arm_L/R`, `IFS_Upper_Arm_L/R`) so `1.5 mm` stainless pins slide smoothly with zero binding—an A-arm must drop freely under its own weight when lifted! Also clear `M1.4` pass-through holes on rails, towers, and links.
* **`Ø2.0 mm` bit:** Clear `Rear_Trailing_Arm_L/R` front/rear pivot bores (`X = -7.75` and `-59.0`) and press in `1.5 × 2.0 mm` bushings.
* **`Ø4.0 mm` bit / reamer:** Gently dress `IFS_Knuckle_L/R` wheel-bearing seats (`X = +48.0, Z = +3.0`) until `4.0 mm` OD micro ball bearings seat flush by thumb pressure.
* **Shell Finishing:** Dry-fit `Tacoma_Fender_L/R` against `Slider_L/R` outer edges (`Y = ±33.5–36.0`) and `Tacoma_Hood_Roof_Shell` nose/A-pillars; bond internal styrene/ABS gusset tabs or bolt micro L-brackets at the sliders and cage so the shell lifts off cleanly as a single clipless assembly or bolts solidly to the sliders.

---

## 5. Step-by-Step Assembly & Path C (2WD) $\rightarrow$ Path A (4WD) Conversion

### Phase 1: Drivetrain & Rear Solid Axle Prep (Path C 2WD)
1. **Rear Axle Overdrive (`B0GX5L4J64`):** Disassemble the stock SCX30 rear axle housing, remove the stock `2/16T` worm shaft and ring gear, clean all stock thick grease, and install **1 set of `B0GX5L4J64` `2/12T` (6:1) worm gears** with a light coat of high-speed PTFE/synthetic silicone grease (avoid heavy sticky crawler grease that builds heat at 9 mph). Verify smooth free rotation by hand.
2. **Longitudinal Metal Gearbox (`B0HMR5D54N`):** Mount the **`14T` (or `15T`) `0.3 MOD` pinion** onto the **GM1614 (7500KV)** shaft using a drop of blue threadlocker on the set screw; set gear mesh with a thin paper strip so there is tiny, crisp backlash with zero binding. Perform the **10-Turn Bench Test** (Section 2) to verify your gearbox ratio.
3. **Skid & Rear Driveshaft:** Bolt `B0HMR5D54N` onto the center SCX30 skid plate (`[0, 0, 0]`) and install the long rear **`AXI-4428`** Tacoma 107 mm driveshaft onto the rear gearbox output (ensure U-joint phasing lines up end-to-end to prevent driveline vibration at speed). Leave the front gearbox output **open** for Path C 2WD.

### Phase 2: LCG Chassis, Rear Trailing Arms & 0.8 mm Anti-Sway Bar
1. **Main Frame Assembly:** Bolt `Chassis_Rail_L/R`, `Slider_L/R`, and the center skid/transmission together using `M1.4 × 5 mm` screws (`[9.5, 0.0]` and `[-7.75, -0.5]`). Install `Rear_Sway_Bridge` between the rear rail tips (`X = -65.0` to `-60.0 mm`).
2. **4-Link & Trailing Arms:**
   * Attach `Rear_Trailing_Arm_L/R` front pivots to `Chassis_Rail_L/R` (`X = -7.75, Z = -0.5`) using `1.5 × 2.0 mm` bushings and `M1.4 × 8 mm` screws + nyloc nuts (snug, not pinched).
   * Attach `Rear_Upper_Link_L/R` front pivots to `Chassis_Rail_L/R` (`X = -12.5, Z = +5.25`) with `M1.4 × 8 mm` screws.
   * Bolt the rear axle (`X = -59.0 mm`) to the trailing arms (`M1.4 × 12 mm`) and upper links (`M1.4 × 8 mm`). Connect the `AXI-4428` rear driveshaft to the rear axle pinion yoke.
3. **Rear Shocks & `0.8 mm` Anti-Sway Bar:**
   * Fill all 4 `24–26 mm` micro shocks with `10wt–20wt` silicone oil and bleed air bubbles.
   * Mount rear shocks from the middle tower hole (`X = -37.5, Z = +19.5`) down into the **60% dropped mid-arm shock pocket** (`X = -38.5, Z = -1.0`) using `M1.4 × 8 mm` screws (use the `-35.0 mm` hole for firmer progressive damping or `-40.0 mm` for softer travel).
   * Slide a straight piece of **`0.8 mm` piano wire** through `Rear_Sway_Bridge` (`X = -63.0, Z = +6.0`), bend both ends 90° forward (`16.25 mm` arms reaching `X = -46.75 mm`), and link each arm tip down to the top tab on `Rear_Trailing_Arm_L/R` (`X = -46.75, Z = +3.25`). Test by lifting one rear tire—the opposite tire should rise slightly with smooth, even torsion resistance.

### Phase 3: Convertible Double-Wishbone Front IFS (Path C 2WD)
1. **Servo & Bulkhead Installation:** Drop your sub-micro steering servo into the `IFS_Bulkhead_Convertible` servo pocket (`X = +25.5` to `+39.5 mm`) with the output spline facing down/forward for clear tie-rod geometry. Bolt `IFS_Bulkhead_Convertible` between the front chassis rail tips (`[27.0, 2.5]` and `[33.0, 5.0]`) using `M1.4 × 5 mm` screws.
2. **Wishbone Arms & Hinge Pins:**
   * Leave the convertible central worm-gear cavity (`X = +43.0` to `+53.0 mm`) **empty** for Path C 2WD.
   * Slide `IFS_Lower_Arm_L/R` (`Y = ±10.5`) and `IFS_Upper_Arm_L/R` (`Y = ±11.6`) into the bulkhead clevises and push the `~22 mm` (lower) and `~17 mm` (upper) `1.5 mm` stainless hinge pins cleanly through from front to rear; secure pin ends with micro E-clips or tiny dabs of E6000/shoe-goo.
3. **Free-Rolling Knuckles & Steering:**
   * Press `4.0 mm` OD micro ball bearings into `IFS_Knuckle_L/R` (`X = +48.0, Z = +3.0`).
   * Pin `IFS_Knuckle_L/R` to the outer lower/upper A-arm bores (`10–12 mm` `1.5 mm` pins).
   * Attach `M1.4` Ackermann steering tie-rods from the servo horn to the rearward knuckle arms (`X = +42.0, Y = ±32.5`) and adjust for ~`0° to 1°` slight toe-out for high-speed stability.
   * Mount front `24–26 mm` oil shocks between `IFS_Bulkhead_Convertible` tower (`Y = ±14.5, Z = +18.5`) and `IFS_Lower_Arm_L/R` shock bosses (`Y = ±22.0, Z = +3.0`). *(Note: With `24–26 mm` dampers, the A-arms rest at ~15–20° positive downward droop at static ride height and compress smoothly to level at `17.25 mm` shock length.)*
4. **Wheels & Tires:** Install front stub axles through the knuckle bearings with wheel hexes (path C runs totally free with zero shaft drag) and mount all 4 wheels/tires.

### Phase 4: Electronics, Bluetooth Setup & Body Fitment
1. **Mount & Wire Electronics:** Strap the **G40 Pro Nano ESC** and standalone **micro receiver** onto one `Slider` (e.g., `Slider_L`) and strap the **3S 300 mAh LiPo** onto the opposite `Slider_R` (twin `3.6 mm` strap slots at `Y = ±30.25 mm`) for 50/50 left-right balance. Plug GM1614 PH2.0 3P into the ESC, ESC signal into RX Ch2, servo into RX Ch1, and **leave the yellow AUX drag-brake wire unplugged** (or endpoint-locked at 0%).
2. **Bluetooth ESC Programming (Mandatory Before Wheels Touch Ground):** Power on with wheels elevated, open the **GACRacing App**, connect to **G40 Pro**, set **Drag Brake = `0%`**, verify **3S LVC**, set **BEC = `6.0V`** (or `7.4V` if HV servo), calibrate throttle endpoints on your radio, and confirm motor rotation direction.
3. **Axial Yeti Tube Cage & Tacoma Trophy Shell:** Seat `Roll_Cage` onto the flat `Z = +8.0 mm` top chassis rail deck (`Y = ±13.0–15.5 mm`, cage feet sit cleanly at `Z >= +8.05 mm`). Fit `Tacoma_Hood_Roof_Shell` over the cage so the `AX31111` front IFS shock tower passes cleanly through the **`8.5 × 35.0 mm` hood window**, and secure `Tacoma_Fender_L/R` along the outer `Slider` edges (`Y = ±36.0 mm`) with the low-cut door sills exposing the side cage truss and `"05"` rear tail fins framing the fastback bed.
   > **Update 2026-11-23 (current design; the paragraph above is the old Rev 1 text):** the part is now the one-piece `Cage`. Its four door-bay posts (B-post X -25.5, A-post X 17.0, both sides) end in a locating peg 2.4 x 1.4 mm, 2.5 mm deep, in a pocket in the tub rim (0.15 mm clearance), with no plate and no screw. Lower the cage straight down so the four pegs enter the pockets, then fit the 4 screws `HW_Cage_Screw_L3`, `R3` (tail foot) and `L6`, `R6` (front foot), M2 x 6 pan head. The shell is no longer a separate hood/roof/fender set: it is the single `Tacoma_Trophy_Body_Shell`, built from the shell saved in the SOMA app on 2026-11-22, with a 1.4 mm nominal wall. See ASSEMBLY.md sections 6 and 7 for the current steps.

---

### Path C (2WD) $\rightarrow$ Path A (4WD) Conversion Instructions (~15 Minutes, Fully Reversible)
Whenever you want 4WD desert rock-bouncer / trail traction instead of maximum 2WD top speed:
1. **Drop In the Front Overdrive Worm Gear:** Unbolt the front shock tower top access or remove the 4 front hinge-pin retainers and drop the **2nd `B0GX5L4J64` `2/12T` (6:1) worm gear set** + two `4.0 mm` OD outdrive bearings into the convertible cavity of `IFS_Bulkhead_Convertible` (`X = +43.0` to `+53.0 mm`, `Y = ±6.0 mm`). *(The front and rear axles will now match at `6.0:1`, or keep a `2/13T` or `2/16T` in rear for front-pull overdrive.)*
2. **Connect Front Driveshaft:** Slide the front **`AXI-4428`** driveshaft (or trimmed SCX30 front shaft) through the bulkhead's `Ø3.5 mm` input tunnel (`Z = +6.5 mm`) from the front output of `B0HMR5D54N` to the front worm pinion.
3. **Install Front Half-Shafts (CVDs / Dogbones):** Pop out the Path C free-rolling dummy stub axles from `IFS_Knuckle_L/R` and install `~22 mm` micro dogbones / telescopic CVDs from the bulkhead `Ø4.0 mm` outdrives (`Y = ±9.0 mm`) into the `IFS_Knuckle_L/R` `Ø4.0 mm` wheel bearings (`Y = ±33.5–38.5 mm`).
4. **Re-Check ESC Tuning:** Keep **G40 Pro Drag Brake at `0%`** for high-speed 4WD bashing, or bump it gently (`15–30%`) via the Bluetooth app/AUX knob *only* when slow crawling below ~2 mph.
