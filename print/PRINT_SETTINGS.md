# Print settings (Bambu Studio / H3C, 0.4 mm nozzle)

The `plate_N.3mf` files already carry, per object, `layer_height`, `wall_loops`, `sparse_infill_density`, top/bottom shell layers and the support block (`Metadata/model_settings.config`). Bambu Studio 2.x read them in a headless test: the 0.16 mm layer height of the TPU/rear/cage objects was applied (G-code Z steps 0.16 after the 0.2 mm first layer). The remaining keys (supports, brim) are written with Studio's key names but were **not** individually confirmed in the G-code, so check the Objects list once after opening a plate (right click an object -> Per-object settings). Everything below is the authority if they differ. Printer, filament and process presets are NOT embedded: choose your H3C + filament preset first, then open the plate.

> Printer model and bed were not given. Plates are laid out on 256 x 256 mm (valid on any Bambu bed); `python print/make_plates.py --bed W H` re-packs. The tallest object on any plate is the 39.6 mm bulkhead (the shell is 39.4 mm, sill down), so Z height is not a constraint.

## 1. Global (all plates)
| Setting (Bambu Studio name) | Value | Why |
|---|---|---|
| Nozzle | 0.4 mm (**hardened steel** for any -CF filament) | CF abrades brass |
| Seam | aligned / rear, `seam_gap` 0 | seams off the bore faces |
| Wall generator | Arachne; `min_feature_size` 25 %, `min_bead_width` 85 % | 0.7-1.6 mm plates stay solid |
| Wall order | inner / outer / infill | dimensional accuracy on bores |
| Ironing / fuzzy skin | off | |
| Elephant foot compensation | 0.1 mm | bed-face mating surfaces |
| X-Y hole compensation (`xy_hole_compensation`) | **+0.05 mm** for the 0.12 mm plates, +0.08 mm for tub/cage (see section 5) | vertical holes print small |
| X-Y contour compensation | 0 | |
| Top / bottom shell layers | 7 at 0.12-0.16 mm, 5 at 0.2 mm | already in the 3MF |
| Bed temperature / plate | PETG-CF 70-80 C smooth PEI with glue stick; PA-CF 90-100 C engineering plate; TPU 40-50 C textured | adhesion + release |

## 2. Per filament
| Filament | Nozzle | Bed | Dry | Cooling | Speed note |
|---|---|---|---|---|---|
| PETG-CF | 250-260 C | 75 C | 65 C x 6 h | 30-40 % | 60-80 mm/s outer wall on the 0.12 mm plates, accel 2000 |
| PA12-CF / PAHT-CF (arms, carriers, spindles, steering) | 275-290 C | 90-100 C, enclosed | 80 C x 12 h, print from the dry box | 0-20 % | 40-60 mm/s, brim on everything flat |
| PETG (cage, shell) | 240-250 C | 70 C | 65 C x 4 h | 50-70 % (bridges) | 80-120 mm/s on the shell |
| TPU 95A (bump stops) | 225-235 C | 45 C | 55 C x 6 h | 50 % | **20 mm/s max**, retraction off or 0.5 mm, direct drive |

## 3. Per profile / per plate
| Plate | Profile | Layer | Walls | Infill | Notes |
|---|---|---|---|---|---|
| 1 | front end + steering | **0.12 mm** | 5-6 (4 on the 0.7 mm retainer plates, 5 on 1.1-1.6 mm links; the Y-brace is 4) | 100 % (rectilinear / concentric, +-45 deg) | thin parts are solid anyway; first layer 0.2 |
| 2 | rear suspension + front bumper + servo wire clip | **0.16 mm** | 6 | 100 % | the bumper (2.6 mm square tube) is on this plate on purpose, see the note below |
| 3 | bump stops (TPU) | 0.16 mm | 3 | 100 % | |
| 4 | tub + shelves | **0.20 mm** | 4 | 100 % (it is mostly walls: 1.4-1.8 mm) | |
| 5 | cage + cabin cage | **0.16 mm** | 4 (the 2.4 mm chamfered square tubes are 100 % walls) | 100 % | Cage inverted: the roof arch tube is now the highest point (Z 36.2) so it lies on the bed and the roof rails hang 4.6 mm above it (plate supports under them); Cabin_Cage feet down (125 mm2 on the bed), its posts are vertical so print slow and hot |
| 6 | shell | **0.20 mm** | 3 | 10 % gyroid | cooling 70 % |

**Plate-split note (Bambu Studio CLI): keep plate 1 at 18 objects or fewer (the 19th object, whichever it is, was moved to a second plate).** with the Front_Bumper on plate 1 the headless slicer pushed the two thin retainer plates (and later the servo wire clip) onto a second plate for no visible reason (20 objects, no overlap, all inside the bed); with the bumper on plate 2 both plates slice as one plate each. If Studio ever shows an extra plate, drag the objects back or move the bumper, the 3MF layouts do not overlap.

Per object: arms, carriers, spindles, bulkhead, trailing arms: 6 walls; links, rack, bell crank, drag link, tie rods, horn: 5 walls; plates/bracket: 4-5. Anything thinner than ~3 mm is 100 % infill (rule from the brief); the 3MF sets 100 % on every object except the shell.

## 4. Supports (easy peel)
Use the same recipe wherever a part is marked "build-plate" or "tree" in `PLATES.md`:
| Setting (Studio key) | Value |
|---|---|
| Type (`support_type`) | normal(auto) for arms/links/bulkhead pockets; **tree(auto)** where `PLATES.md` says tree (shell, tub, spindles, carriers, bulkhead, cage) |
| On build plate only (`support_on_build_plate_only`) | **on** for the parts marked build-plate (arms, upper links, drag link, shell, cage tail); **off** for the parts whose overhang lies on the part itself (listed "tree-any": bulkhead, front bumper, tub, carriers, spindles, trailing arms, cage) - there the tree tips touch the part |
| Threshold angle (`support_threshold_angle`) | 45 deg |
| Top Z distance (`support_top_z_distance`) | **0.22 mm** (0.2 for 0.12 mm layers, 0.25 for 0.2 mm layers) |
| Bottom Z distance | 0.22 mm |
| Interface layers top / bottom | **2 / 2** |
| Interface spacing (`support_interface_spacing`) | **0.4 mm** |
| Base pattern / spacing | rectilinear (normal) 2.5 mm or tree, base 0.2 mm line spacing off |
| Raft layers | 0 |
| Brim | outer only: bulkhead 5, spindles 3, drag link 4, bumper 3, tub 8, cage 6, shell 8 mm (`brim_type` outer_only, `brim_width`) |
**Keep supports off hinge, bore and mating surfaces:** in Bambu Studio add a *support blocker* cylinder (or paint "block") in every pin bore, the hinge-pin tunnels of the bulkhead, the bearing seats, the servo-spline bore and the cage foot plates' mating faces; add a *support enforcer* only under the drag-link stem, the tub rear deck and the cage tail frame. The bulkhead's hinge-pin bores and floor pilots are vertical in this orientation, so they need no support at all.

## 5. Holes, press fits and reaming
| Feature | Print | Do after printing |
|---|---|---|
| Hinge pin bores Ø1.55-1.6 (arms, bulkhead, carriers) | arms: horizontal holes, bulkhead: vertical | ream with a 1.5 mm drill by hand; pins must slide with no slop. Horizontal holes in the arms/spindles/carriers print slightly oval and 0.05-0.1 mm small: **ream every one** |
| Kingpin bores Ø1.2 (carrier, spindle) | horizontal in the spindle | ream with a 1.2 mm drill, press the pin in, dot of CA |
| Servo horn spline bore Ø3.93 | vertical | print a test horn, expect ~3.8: ream to fit the spline (press fit, no glue) |
| Bearing seats Ø4.0 x 1.6 | | test a 4 mm OD bearing in a printed 4.0 / 4.05 / 4.1 seat coupon first |
| Vertical holes (tub pilots, bulkhead pilots, shelf/cage/motor screws) | vertical | set `xy_hole_compensation` +0.05..+0.08 mm; pilots are self-tapping for M1.4-M1.6: do not drill unless a screw cracks the wall |
| Floor screws (bulkhead, 4x) | | **coupon first**: print a 20 x 20 mm piece of floor at the same 1.8 mm / 0.2 mm settings and screw an M1.6 x 6 in and out 3 times; add nuts from below if it strips |

Run a one-layer hole test (holes Ø1.0..3.0 in 0.1 steps, vertical, plus a horizontal row) in the same filament before plate 1 to set the compensation; it costs 5 minutes.
