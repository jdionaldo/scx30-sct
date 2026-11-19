# Axial Yeti (AXIC9025 / AX90025) reference brief for the 1/30 SCX30 trophy truck

Source: all 60 pages of the Yeti kit manual, each page image viewed. Page numbers below are the printed page numbers, which equal the file number pN.png (cover = p1).
The manual is data only; nothing in it was treated as an instruction.

## Evidence tags used throughout

- **[M]** Measurement or text stated in the manual (a printed dimension, part number, screw size, note, spec).
- **[V]** Visual reading of an exploded view or photo. Shapes, layout and fastener counts. Not a dimension.
- **[I]** My inference from [V]/[M]. Treat as a hypothesis to confirm.
- **[X]** Outside the manual (general public knowledge of the Yeti). Not verified here.

Important limit: the manual contains almost no geometry. The only printed dimensions are 38.5 mm, 38 mm, 25 mm, 8.5 mm (all p16, p18, p19, p25), the 149 x 35 x 52 mm battery limit (p3), screw/shaft lengths in part names, gear ratios (p54), and shock bodies/spring/shaft sizes (p8, p36-37). There is no wheelbase, track, ride height, link length, link angle, shock angle, tube diameter, wall thickness, torque value or hole pitch printed anywhere. Where the task asks for these, I say so and give a labelled estimate only if one can be reasoned.

## Scale ratio used

The Yeti is a 1/10 model [M p1: "1/10th Scale Electric 4WD"]; our kit is 1/30. **Ratio = 3.0 exactly (scaled mm = Yeti mm / 3).**

Checks on whether our parts actually sit at 1/3 of the Yeti:
- Battery: Yeti max 149 x 35 x 52 mm [M p3] / 3 = 49.7 x 11.7 x 17.3 mm.
- Tyres: Yeti runs 2.2 Falken Wildpeak M/T R35 tyres on 2.2 Method beadlock IFD wheels [M p57: AX31077, AX31178]. A 2.2 crawler tyre is about 5.5 to 5.7 in (about 140 to 145 mm) tall [X, not in the manual]. /3 = about 47 mm. Our tyre is O36 x 13.5, i.e. about 0.26 of the Yeti tyre, not 0.33. **Our tyre is about 22 to 25 % smaller than a true 1/3 copy** [I, depends on [X]]. Likewise the Yeti wheelbase is about 12.9 in (about 328 mm) [X]; /3 = about 109 mm, so our 116.5 mm wheelbase is about 6 % long. Net effect: our truck looks "long and low-tyred" against the Yeti. Tyre to wheelbase: Yeti about 0.43 [I from X], ours 0.31.
- Fasteners do not scale. The Yeti uses M3 (3 mm) main screws and M2.6 body screws [M p10, p57]; at 1/3 they would be 1.0 mm and 0.9 mm. Our M1.6 is 1.6x over-scale, so visible screw heads will always read bigger than on the Yeti.

## 1. Overview

**What it is.** A 1/10 electric 4WD "rock racer" (a Utah-style desert buggy / rock racer), kit version, with a body that is a faceted truck-like shell over a tube cage with two driver figures in an open cockpit [M p1, p42, p63].

**Layout (all [V] unless noted):**
- Mid-engine layout: longitudinal 2-speed Hi/Lo transmission lies in the tub centre with a brushless motor beside it, parallel to the transmission, driving through a 64T spur and 16T pinion with slipper [M p23-26, p57; ratio table p54: 38/13 bevel, 64T/16T gives 22.67:1]. Front driveshaft (AX31139, 103 mm [M p9, p18]) runs forward to the front diff, rear WB8-HD telescopic shaft runs back to the rear axle [M p31].
- Front: **independent** suspension. EXO bulkhead (the diff housing and the front structure in one, AX80091 [M p4, p11-12]), a lower control arm per side (AX80111 [M p15]), a single upper link per side (M4x55 turnbuckle, AX31272, to the shock tower [M p16]), steering knuckle plus carrier on vertical steering kingpins (AX30382 EXO Steering King Pin x4 [M p15]), and a double-shear steering rack with a servo saver [M p17-19].
- Rear: solid AR60 OCP axle (AX80069 housing [M p4, p13]) on a 4-link (2 lower trailing arms plus 2 upper links) [M p33-35]. Rear shocks stand on the lower links and go to cantilevered rear cage towers [M p39].
- Anti-roll: rear sway bar only (AX31058) [M p8, p30].
- Wheelbase/track: not in the manual. About 328 mm wheelbase [X]. Front hex adapters are "narrow" and rear are "wide" (AX31266, p47-48), so the rear track is slightly wider than the front [M p47, p48]. We already do this (71 vs 71.6 mm), which is the right instinct.
- Weight/ride: tub is low and wide, tyres are big; the stance is "tyre-dominant, tub tucked between them" [V p1].

**Major assemblies (from the manual's own build order, bags A to I):**
| Step range | Bag | Assembly | Pages |
|---|---|---|---|
| 1-9 | A | Front diff, bulkhead, shock tower; rear diff and AR60 axle with 4-link mount | p11-14 |
| 10-17 | B | Front knuckles, lower arms, upper links, front module bolted to the tub, steering rack | p15-18 |
| 18-25 | C | Front driveshaft, servo, front Y-brace, ESC and receiver trays, radio-box lid | p19-22 |
| 26-32 | D | 2-speed transmission, spur, motor, installed from below into the tub | p23-27 |
| 33-40 | E | Rear cage components, fuel cell (battery box), sway bar, rear sub-assembly bolted on the tub | p28-32 |
| 41-46 | F | Rear 4-link, axle bolted in, rear driveshaft | p33-35 |
| 47-52 | G | Shocks (front and rear), front bumper, rear shocks to the rear cage | p36-39 |
| 53-63 | H | Shell: wash, mask, paint inside, light buckets, drivers, Y-380 cage assembly, body mounts | p40-46 |
| 64-67 | I | Wheels and tyres | p47-48 |
Order of build [M]: axles and bulkheads first, then the tub gets the front module, then electronics, then the powertrain is dropped in from above and screwed from underneath, then rear sub-assembly, rear axle, shocks, bumper, and the body/cage is built as a separate unit and hinged on last.

## 2. Tub / chassis

**Parts:** Yeti Molded Chassis Tub AX31103 (single piece), Chassis Components AX31104 (a set of brackets, brace, mounts), Chassis Electronic Components AX31105 (trays, lids, plate, switch holder), Fuel Cell AX31125 (battery box) [M p6-7, p57].

**Shape [V p7, p17, p21, p24, p32, p51]:**
- A single deep molded tub, open on top with a rear-open end where the rear sub-assembly attaches. The inside is a pocketed **isogrid / diamond lattice of ribs** (clearly visible on the underside view p7 and top-down p51). This is the Yeti's signature finish detail.
- Side walls are tall: they roughly reach the motor/ESC top line. In p32 and p39 the motor sits "in" the tub; the wall height is of the order of one motor diameter (about 36 mm for a 540, so 30 to 45 mm at 1/10; /3 = 10 to 15 mm) [I, not measured].
- The belly is a chamfered, faceted "boat" shape: flat floor, bevelled lower side faces, and a long sloping nose ramp at the front (dark band along the bottom edge on p17, p24, p25). The nose rises toward the front axle like a skid ramp. [V]
- The top of the front half is a **sealed radio-box deck** with faceted raised ridges and a recessed perimeter and six countersunk screws; a one-piece sealed lid with a perimeter gasket (AX31083 "Radio Box Seals") screws on with 4 flat-head M3x12 [M p22, steps 24-25].

**Layout [V]:**
- Servo: **transverse, in the front of the tub**, behind the steering rack, on 2 small plastic servo mounts (AX31104 #7/#8) with 4 M3x10 oversized flat-head screws; the servo horn drives a 25 mm M3x35 turnbuckle link to the servo-saver bellcrank [M p19-20 including the 25 mm dimension].
- ESC: on double-sided tape on a plate/pocket in the floor on the side of the front half, beside the servo and ahead of the motor/transmission. Receiver: in the sealed front radio box with a seal grommet at the wire exit and an on/off-switch holder (AX31105 #7, seals) [M p23-24].
- Transmission and motor: dropped in from above into a pit in the floor and held by **6 M3x12 flat-head screws from underneath** (4 under the trans, 2 under the second boss) [M p27, step 32]. The motor sits above and beside the trans, mount clamp rotates to set gear mesh (2 x M4x12 cap-head, loosen/rotate) [M p26, p56].
- Battery: **not in the tub**. It lives in the "fuel cell" at the rear. A boxed ladder-frame with an access door (clip), height-adjustable drop-down clamp (4 x M3x8 AXA114; "snug fit required") [M p55]. Battery limit 149 x 35 x 52 mm [M p3]. The fuel cell is the core of the rear sub-assembly. It appears to lie transverse (box wider than deep) [I from p33-34, p55].
- Extra: a flat bridging brace across the tub side walls at the rear (AX31104 #6) and a small tab/clip retainer (AX31104 #4/#11, one body clip) [M p32 step 39]. Function not stated [I: transmission hold-down or battery/body latch].
- Front structure: a Y-shaped top plate (AX31104 #1) braces the front of the tub to the front module (3 x M3x18 AXA148 plus 4 x AXA146 flat-head, thread lock) [M p20 step 21; shape on p5].

**How the tub is held together / attaches (all [M] screws, [V] layout):**
- Front module (bulkhead + diff + shock tower + lower arms + bumper-mount block AX31112 #3/4) bolts to the tub nose by 2 x M3x30 AXA121 plus 2 x M3x61 shaft screws (AXA0128, these are also the lower arm pivots: "NOTE: Do not over tighten. Should move freely") and 4 x M3x12 AXA146 flat-heads up from under the nose [M p16 step 12; p18 step 17].
- Rear: the rear sub-assembly (fuel cell + rear hoops) screws to the tub top with 3 x M3x12 plus 2 x M3x16 [M p32 step 40] and 2 more M3x12 [M p34 step 44]. Two 4-link upper mounts (AX31104 #9/#10) sit on top at the tub rear with 2 x M3x12 each [M p34]. The lower arms pivot on long M3x35 AX31119 screws going through the tub's rear (nuts AXA1052), and the upper links on M3x20 AXA119 through the mounts [M p35 step 45].
- Standoffs: there are screw bosses molded into the tub, so no separate standoffs. The tub is the hardpoint for everything [V].

**Skid:** no separate skid plate is called out in the manual. The tub belly is the skid [V]. Our 8 mm ground clearance under the tub vs the Yeti's: not given.

## 3. Suspension geometry

**Front (all [V]/[M] as marked):**
- Independent. Lower A-arm AX80111 with a pivot block at the front tub nose (pivot = M3x61 through-screw). The arm has a clearly visible shock pocket near the knuckle end and a wishbone shape (two inboard pivot ears) [M p15 step 11; V p5].
- Single upper link: **M4x55 turnbuckle (AX31272) with M4 rod ends (AX31186)**, 38.5 mm shown between rod-end faces in the build [M p16 step 13: "38.5mm"]. Inboard end on the shock tower (AXA119 M3x20 into the tower), outboard on the carrier (AXA118 M3x18) [M p16].
- Knuckle: Yeti Steering Knuckle (AX31110) held to the EXO Steering Knuckle Carrier (AX80106) by two kingpins (AX30382, M3 set screw at each end: AXA0181) i.e. a split vertical kingpin [M p15 step 10].
- Shock tower AX31111: bolts to the bulkhead with 2 x M3x18 AXA118 and to the bumper-mount block with 4 x M3x16 flat-heads [M p12 step 3]. The tower has rows of holes in 4 positions each side for shock top position, and a cross bar with two body posts (see section 5) [V p5, p12, p39].
- Front shocks: 12 x 41.5 mm aluminum bodies (AX31187), shaft 3 x 53.5 mm (AX30147), spring 14 x 54 mm 4.33 lb/in yellow (AX30229), 10 mm pistons, 30 wt oil [M p8, p36-37]. Top on tower with ball-end bushing (AX80033 #7) + M3x18, bottom on the lower arm with M3x18 [M p39 step 51].
- Steering: **double-shear rack** AX31122 with a centre plate (#5), servo-saver bellcrank (#1, #7 spring AX31060 13 x 9.25 mm [M p8]) on a steel post (AX30384) with 5x8x2.5 bearings, and a slave bellcrank (#3). Two M4x60 turnbuckles (AX31273) at **38 mm** between rod-end faces link rack to knuckle arms [M p17-18, steps 14-16]. Servo link is 25 mm [M p19].

**Rear:**
- AR60 OCP axle: housing AX80069, differential cover AX80070, straight axle adapters AX80071, 4-link mount AX80072 on the top of the housing, shock mount ears AX80070 #1 on the tubes [M p4, p13-14]. Axle shafts 6 x 104.2 mm and 6 x 160.2 mm [M p13, p57] (their length difference implies a housing with offset gearing [I]).
- 4-link: 2 lower trailing arms (AX31109 #1) each reinforced with 2 stamped link plates AX31245 sandwiched per arm (steel; p41 labels "Stamped Link Plate 1 Lower" x4) [M p33 step 41]; 2 upper links (AX31109 #2, #3) [M p34 step 43]. The upper links converge at the **centre ears of the 4-link mount** (one M3x30 AXA091 through the centre), and spread to two chassis mounts (AX31104 #9/#10) i.e. a triangulated upper link [V p34, p35, p56; I]. Lower arms trail forward to the tub's rear bay. Link lengths and angles: not given. From the drawings on p33 the lower arm is about 3 to 4 times the axle-adapter width, which is roughly 100 to 130 mm [I, low confidence]; /3 = 33 to 43 mm.
- Anti-squat: **the upper link chassis mount (AX31104) has 3 stacked holes**; the kit uses the **top hole** "RTR stock: upper hole, more anti-squat for more rear traction under power", lower holes "less anti-squat" [M p56]. The mounts swap left/right, and swapping them "provides a suspension more suited to low speed technical terrain" and requires flipping the upper shock bushings [M p56]. Hole pitch not dimensioned. The drawing implies about 6 to 8 mm spacing at 1/10 [I, low confidence]; /3 = 2 to 2.7 mm.
- Rear shocks: 12 x 59 mm bodies (AX31189), shaft 3 x 71 mm (AX31054), spring 14 x 90 mm 1.14 lb/in orange, 10 wt oil [M p8, p36-37]. Bottom: on the lower link mid-span (ball + M3x18) [M p39 step 52]. Top: on the rear cage via a **shock mount plate** (AX31102 #7/#8, "Do not over tighten") with two flip-able bushings (AX31102 #1/#2) held by 2 x M3x18, and the plate itself bolted to the rear hoop side [M p39, p56 "Upper shock bushings rotated to relocate shocks and line up with AX31109 lower links"]. The shocks stand close to vertical, with a small forward lean at most [V p39, p52; I]. **Ours is 28.6 deg forward. That is far steeper than the Yeti, which looks about 5 to 15 deg** [I]. Flag: our shock lean is the single biggest geometry deviation I can see, but it is dictated by our shorter shock and cage position.
- Bump stops: rubber bump stops 3.6 x 7 x 4 mm (AX30113) sit on the **shock shaft at the lower end** inside the body (one per shock) [M p36 step 47, p10: x4 total]. At 1/3 that's 1.2 x 2.3 x 1.3 mm, so use a printed collar. Ours already have bump stops; consider moving to the shock-shaft (internal) rather than separate [I].
- Sway bar: AX31058 steel wire bar (3 stiffnesses in kit: soft/medium/firm [M p57]), 6 mm? [not stated], mounted in two hinged sleeves (AX31113 #2) on the rear cage legs, M3x4 set screws "Do not over tighten, should move freely", with 2 x vertical links (AX31113 #7) down to the lower links [M p30, steps 35-36].
- Driveshaft layout: rear **WB8-HD** telescopic shaft with a ball/pin coupler (AX31148, 2x11 pin AX31028 [M p31]), thread-locked M4x12 set screws [M p31].

## 4. Cage

**Important structural finding:** the Yeti has **two different cages.**
1. **Chassis-mounted "Rear Cage Components" (AX31102)** [M p28-35]: a structural rear sub-frame. Two bent-tube hoops + an X cross-brace + shock mount plates + sway bar sleeves, bolted around the fuel cell with 5-8 M3x18 screws, then screwed to the rear of the tub. This carries the **rear shock tops** and the sway bar. It cantilevers rearward past the tub; the rear legs stand on the tub and the hoop tails extend behind it as a rear bumper/tail [V p28-29, p32, p35, p52].
2. **Body-mounted "Y-380 Cage" (AX31115 sides, AX31116 front, AX31117 top/rear/tire carrier)** [M p7, p43-46]: a 1/10-scale tubular spaceframe that is **part of the body assembly**, not the chassis. Built from 5 sub-assemblies (L side, R side, front hoop, roof X-hoop, rear carrier), joined by 15 M3x15 and M3x12 tapping button-head screws, plus 4 M2.6x6 to the interior tub. It is **screwed to the cockpit interior (AX31141 / driver tub)** (4 x M2.6x6) [M p44 step 60], then the roof panel (4 x M2.6x6) and the shell sit over it [M p45]. It never touches the chassis directly. It attaches to the chassis only through the front body posts and the rear carrier pivot [M p46].

**Tube layout (Y-380) [V p7, p43-46]:**
- Side cage: one large piece per side with lower rocker rail, A-pillar sloping forward, B-pillar, a high beltline, 2 to 3 diagonal braces and a forward leg. Tubes meet at nodes with small round bosses and a screw hole where sub-assemblies join (screw into the tube end) [V p43-44].
- Front cage: a trapezoid hoop with a lower crossbar and a triangulated top ("A" shape) plus a lower tubular bumper section (AX31116 #1 and #2) [V p7, p43].
- Top/rear: roof hoop with a **big X** diagonal (AX31117 #2), a rear tire carrier frame with a row of holes (8 holes on the cross bar; used for the light bar and spare) [V p7, p43, p46]. Hoop count: roughly 3 principal hoops (front, mid/roof, rear) plus side rockers = a 4 to 5 hoop cage [I].
- Tube diameter: not given. Compared with the M3 screw head (5.5 mm) and the 6 mm body clip, tubes look about 6 to 8 mm [I, low confidence]; /3 = 2.0 to 2.7 mm. This is at the printable limit. Note [I]: at that size they only print as a straight standing tube if oriented along the tube axis with a 0.4 mm nozzle.
- Node style: **tubes are round, uniform diameter**, with no gussets or flanges. The chassis-mounted rear hoop has flat tab feet with screws; the body cage has a round boss foot [V].

**Attachment:**
- Rear cage to tub: see section 2 (5 to 6 screws into the tub top; hoop legs have flat pads) [M p32, p34]. Each hoop leg is a tube that stands on a pad and takes a screw from the top (M3x12 AXA146 flat-head) so the cantilever load goes into **screw pads on the tub deck, not into a single tube** [V].
- Rear shock tower function: shock mount plate with 2 x M3x18 on a plate, mounted on a **hoop node** about 2/3 of the way up the rear hoop [V p39].
- Spare tyre carrier: the rear carrier frame has a cross-bar with a row of holes, a rear light bar bezel (AX80103 #1, with 5 optional 5 mm LEDs and AX80049 lenses [M p43]) and body-post mounts. No actual spare wheel is supplied [V p43, p46].
- Body posts: see section 5.

## 5. Shell / body

**Parts:** Y-380 1/10 body 0.040" clear (AX31140), Y-380 interior 0.040" clear (AX31141), pre-painted option AX31174 [M p57-58]. Light buckets, grille and trim parts are separate molded plastic: AX31116 #3 (grille), #9, #11, #12 (headlight buckets and lenses), #4-#7 (side lenses), #13/#14 (hood pads) [M p41, p45]. Decal sheet [M p42]. EXO rear light bezel and radiator set AX80103 [M p5, p43]. Helmets AX80131 [M p4, p41].

**Shape [V p42, p63, p1]:**
- A long "hood + fenders + door skin" shell: sloping hood with a recessed centre panel and two small round pads, integrated front fenders with large wheel arches, a flat side panel (the "door" skin) running back to the cockpit opening, then a flared rear section. At the rear the shell has **two upswept fins** at the top corners (p42) [V].
- **There is no windshield, no glass and no roof in the shell.** The cockpit is open and the cage provides the roof; a separate **flat roof panel** (4 x M2.6x6) caps the cage [M p45 step 62]. Window mask is only used for the rear "window" area where the shell opens [M p40 step 53].
- Rear: flat/near-vertical tail panel; the radiator (AX80103) and a 5-segment light bar sit on the rear carrier [M p43].
- Proportions: hood is about 1/3 of the shell length; the cockpit about 1/2; the tail about 1/6 [I, from p42 plan view].

**Mounting [M/V]:**
- The shell, interior tub, cage, helmets and roof are built as **one unit** [M p40-46]. The shell has 2 holes at the front of the hood that take 2 front body posts (on the shock tower bar) with 6 mm body clips (AXA1059 x4 in the kit, x2 at the front, x2 rear) [M p46 step 63]. Posts have several holes in a vertical column so the body height is adjustable [V p5, p12 "Note Direction"].
- Rear: 2 posts + a flip-up bracket on the rear carrier (AX31117 #3-#7, AXA440 M3x25, AXA115 M3x10, AXA1053 nuts) pinned with clips. **The body hinges up from the front: the inset on p46 shows it swinging open about the rear mount** [V p46 inset arrow].
- Clearance: the shell has large rounded arches that frame the tyres with generous clearance [V p1, p63].

**What the EXO parts add [M p5-6, p15, p43]:** EXO Bulkhead (the front diff housing + mounts, p4), EXO Lower Front Control Arms Set (AX80111, p5), EXO Steering Knuckle Carrier (AX80106, p5), EXO Rear Light Bezel and Radiator Set (AX80103, p5). The Rear Light Bezel is a lens-covered lamp bar for 4 LEDs; the "radiator" is a finned cooler bolted on the rear carrier (visible "radiator + two fans" on p5). These are decorative rear details [V].

## 6. Front bumper (AX31112 set)

- Two parts: a **bull-bar tube hoop** (AX31112 #1: a wide rounded arch with an upper bar, side returns and a flat centre plate with tabs) and a **closed oval "skid" loop** (AX31112 #2) held below it with 2 x M3x18 + nuts, plus the front **mount block (#3/#4)** [M p38 step 50, p7].
- It ties to the chassis through the **lower control-arm pivot block**: the block carries the M3x61 pivot screws, plus 1 x M3x20 AXA119 and 2 x M3x12 AXA116 into the bumper hoop [M p16, p38]. Also the shock tower is bolted to the same block with 4 x M3x16 [M p12]. [I: the bumper acts as a crash structure that protects the arm pivots and tower.]
- The oval loop works as a **skid/bash loop** under the nose; there is no separate tow hook on the front [V p38]. There is a rear tab with a clip (AX31104 #11/#4) [M p32 step 39; I: tow or latch point].

## 7. Fit and finish details worth copying

All [V] unless stated.
1. **Isogrid rib pattern** inside the tub (p7, p17, p51). It looks finished, lightens the part and is printable.
2. **Sealed lids and gaskets** on the radio box: perimeter gasket, 4 flush screws, small switch holder, rubber grommets (p22, p24-25) [M p22].
3. **Two-tone hardware:** all fasteners black (parts list p57 lists every screw "Black"), countersunk flat-heads on the underside and body-contact surfaces, button-heads on visible cage joints.
4. **Hex/ball-end pivots** with rounded hardware (AX80018 hardware tree balls) at every link end.
5. **"Do not over-tighten / should move freely"** is called out on every pivot (p16, p17, p30, p36, p39). Joints are made with a screw into a nylon nut and free play is wanted.
6. **Visible metal parts:** aluminum shock bodies with shield logos (AX31187/AX31189), steel link plates on the rear arms (AX31245), steel king pins (TiN coated, p57), a metal sway bar and turnbuckles. Our printed equivalents can be colour-coded (silver/anodised shades).
7. **Separate molded light buckets and grille** (headlights with 5 mm LED pockets, p41), vented side outlets, hood pads, light bar with lenses (p43). At 1/3 these are 3 to 8 mm features and can be separate small inserts.
8. **Driver figures with helmets and a dash/interior** (p41-42), seen through the open cockpit.
9. **Paint and decals:** shell is painted from the inside after a soap wash and masking of the window area (p40), with a sponsor-heavy decal sheet (p42). Clear shell shows a glossy finish from the outside.
10. **Panel gaps:** the manual does not show or specify gaps. The shell is a single vacuum-formed piece with no door/hood seams; the only separate panels are the roof panel, the hood pads and the vent lenses (p42, p45). The "finish" comes from tight separate parts rather than from gaps [I].
11. **Fastener visibility:** the shell is held with only 2 clips front and 2 rear, all other fixings hide inside the cage and interior.
12. **Adjustability:** 3 holes for anti-squat, flip-able shock bushings, multiple body-post holes, shock-tower holes, sway-bar clamps, adjustable battery clamp.

## 8. Mapping to our project

Scale 1/3 (Yeti mm / 3). Our files: Chassis_Tub, Cage, Tacoma_Trophy_Body_Shell, IFS bulkhead, rear arms (names as given by the user; I did not inspect the repo geometry).

| Yeti feature | Yeti numbers / pages | Scaled to 1/30 | How to adapt to our design | Feasibility with SCX30/SCX24 parts, printed |
|---|---|---|---|---|
| Shell hinges open from the front, 2 front body posts with a vertical column of clip holes, rear hinge posts | Posts on tower bar p12, p46; clips AXA1059 6 mm p10 | Post spacing about half tub width; clip about 2 mm | Add a post bar to the IFS bulkhead/tower (Chassis_Tub front), with a column of 3 to 4 holes at 2 mm pitch; add 2 rear pivot pins on the Cage. Shell gets 2 front holes + 2 rear tabs | Easy. Printed posts at 2.5 mm; use 1.5 mm pins or SCX24 body clips. |
| Deep tub with isogrid ribs | p7, p17, p51 | Rib about 0.8 to 1.0 mm thick, 6 to 8 mm pitch | Replace the flat inside faces of the tub walls and floor with a diamond lattice pattern in Chassis_Tub | Feasible: ribs at 2 perimeters, adds negligible mass; lattice pockets should be 5 mm min. |
| Sealed radio-box lid + gasket + countersunk screws | p22, p24-25 (4 x M3x12 flat head) | 4 x M1.6 | Make the ESC/RX pockets covered with a faceted deck lid with a perimeter lip and 4 countersunk M1.6 screws; seal with a thin TPU strip | Feasible; lid is printed face-down. |
| Faceted sloped nose and belly chamfer | p17, p24, p32 | Nose ramp about 15 to 20 mm | Chamfer the belly edge and slope the nose in Chassis_Tub (we have 8 mm ground clearance). Keep the same lowest-line | Easy (geometry only). |
| Front module bolted as a unit to the tub nose with Y-brace | p16, p20; screws M3x30, M3x61, 4 x M3x12 | M1.6 equivalents | IFS bulkhead: keep as one module bolted from under the tub nose; add the Y-shaped top brace tying the bulkhead to the tub side walls | Feasible, and it stiffens the front. |
| Lower arm pivot through a block with long through-screws, "should move freely" | p16 | M1.4/M1.6 shaft | Use through-screw pivots with nylon bush, no thread engagement | Already close; check play. |
| Servo transverse behind the rack, 25 mm link, servo-saver bellcrank | p17-20 | Link about 8 mm | We use a side servo; keep it, and make the saver a sprung bellcrank if there is room | Partly. Spring 13 x 9.25 mm scales to about 4 x 3 mm (SCX24 servo saver springs are close). |
| Front upper link (M4x55 turnbuckle, 38.5 mm span) + lower A-arm | p16 | Span about 12.8 mm | Our IFS is a double wishbone; shorten upper link to about 0.9 to 1.0 of lower arm length; no need to copy | Feasible; keep. |
| Rear 4-link with 2 lower plate-reinforced arms and triangulated upper links | p33-35, p43; lengths not given | Lower arm about 33 to 43 mm [I] | Rear arms: add a plate/rib detail (steel plates become printed ribs or 0.3 mm steel shim), triangulate upper links to the centre top of the axle | Feasible. Triangulated upper links need a centre ear on the axle mount (SCX30 axle housing allows a printed clamp). |
| Anti-squat 3-hole upper link mount | p56 (top hole = kit) | Pitch about 2 to 2.7 mm [I] | Make the rear upper-link tub mount a 3-hole vertical block, mark the top hole "kit" | Feasible; needs 2 mm stack with 1.6 mm holes, wall about 0.8 mm, OK. |
| Rear shock top on a bolt-on shock-mount plate with 2 flippable bushings, plate on a hoop node | p39, p56; plate AX31102 #7/#8 | Plate about 10 x 15 mm | Make the shock top a separate printed plate on the Cage with 2 positions | Feasible; keeps our Injora 36 mm shock; reduce lean angle if possible. |
| Rear sub-frame = fuel cell + hoops + sway bar bolted to the tub top | p28-35 | Hoop tube about 2.0 to 2.7 mm [I] | Our single-tube cantilever cage is the same idea; add the flat feet pads and a second tube (or X brace) behind | Tube at 2 mm barely prints; use 2.4 mm min and print upright. |
| Sway bar | p30 | Wire about 0.6 to 0.8 mm | Optional; a 0.7 mm steel wire in 2 printed sleeves | Feasible but low value. |
| Front bumper: bull-bar hoop + closed skid oval + block that also holds the arm pivots | p38 | Hoop span about 18 to 22 mm [I] | Make the bumper tube hoop attach to the IFS arm-pivot block with 2 through screws; add an oval skid loop underneath | Feasible, 1.6 mm tube is too thin; use 2.2 mm minimum. |
| Body-mounted cage (Y-380) separate from the chassis | p43-46 | Tube 2.0 to 2.7 mm | Move to a body-attached cosmetic cage or fuse into the shell/interior; keep the structural cage separate | Worth considering; a large print. |
| Interior with drivers and helmets | p41-42, p54 | Helmet about 10 to 12 mm [I] | Add a printed interior tub with 2 helmet/torso bumps and a dash | Feasible; helmets are 10 mm spheres. |
| Open cockpit, flat roof panel | p45 | Roof panel about 40 x 30 mm [I] | Our Tacoma shell has windows; a Yeti-like "roof panel on cage" is a separate style | Style decision (see open questions). |
| Separate light buckets, grille, vents, side lenses | p41, p45 | 3 to 8 mm | Add recesses for clear inserts in Tacoma_Trophy_Body_Shell; print inserts in clear/black | Feasible; clear PLA/PETG prints. |
| Rear light bar and radiator on the rear carrier (AX80103) | p43, p5 | Bar about 33 x 5 mm | Add a light-bar bezel at the cage top-rear | Feasible; LEDs not needed. |
| Beadlock wheels with 6 screws | p47 (6 x M3x12 per wheel), AX31178 | Ring about 1.2 mm tall, 6 dummy screw heads | Print a beadlock ring over the SCX wheel | Feasible if wheel allows. |
| Hardware finish: black screws, countersinks | p57 | M1.6 black | Standard | Easy. |

### Prioritised list of changes (value for "same fit and finish")

**(a) Must-do now**
1. **Body mounting like the Yeti:** 2 front body posts on a bar at the IFS bulkhead/tower, hinged rear pins, shell tilts forward/open (p12, p46). This is the biggest "fit" improvement (the shell stops being a loose lid).
2. **Sealed, lidded electronics deck** with 4 countersunk screws and a gasket line (p22, p24-25); removes the open-pocket look.
3. **Isogrid ribs + chamfered belly** in the tub (p7, p17, p51).
4. **Anti-squat 3-hole upper-link mount**, with the top hole as kit default (p56). Adds tunability and the Yeti look.
5. **Rear shock top on a bolt-on mount plate** with 2 positions (p39, p56) instead of a fixed boss, and check the shock angle (our 28.6 deg lean is far steeper than the Yeti's near-vertical shocks [I]).
6. **Triangulated upper links** converging on a centre ear of the axle mount, and plate-style lower arms (p33-35, p43).
7. **Front bumper:** bull-bar hoop + oval skid loop tied to the arm pivot block (p38).
8. **Y-brace over the front of the tub** tying the bulkhead to the tub side walls (p20).

**(b) Nice-to-have**
9. Separate headlight/grille/vent inserts (p41, p45).
10. Interior tub with 2 helmeted figures and a dash (p41-42, p54).
11. Rear light bar and radiator at the cage top-rear (p43).
12. Sway bar with flippable sleeves (p30).
13. Beadlock ring with 6 dummy screws (p47).
14. Decal sheet and two-tone paint, black fasteners (p42, p57).
15. Adjustable battery clamp and access door (p55) if the battery moves to the rear.

**(c) Cannot be adapted (and why)**
- **Vacuum-formed 0.040" (1.0 mm) polycarbonate shell, painted from the inside** (p40, p57). At 1/3 this is 0.33 mm; a printed shell is 0.8 mm minimum, so ours is always 2.4x thick. Accept the thicker wall and compensate with chamfered edges.
- **Real scale screws** (M3 = 1 mm at 1/3): not available; M1.6 is over-scale.
- **2-speed transmission, bevel-gear diff, WB8 telescopic shaft, AR60 axle internals** (p11-14, p23-27, p31): we use SCX30 parts instead.
- **Aluminum / TiN / hard-anodised metal parts** (p57-58): printed only.
- **True 1/3 tyre size** (47 mm vs our 36): our wheel/tyre are fixed unless a larger SCX/Injora tyre is chosen (see questions).
- **Tyre-dominant stance** follows from the previous point.
- **Tube diameter of the Y-380 cage at true scale** (2 to 2.7 mm): at the printable limit, so strength relies on short spans and X-bracing.

## 9. Open questions for the user

1. Do you want the Yeti's **open-cockpit** style (no windshield, roof panel on a cage, drivers) or keep the Tacoma windshield and cab? The Yeti has no glass (p42, p63).
2. Are the **tyres** fixed at O36 x 13.5? A true 1/3 Yeti tyre is about 47 mm [X], so everything else on our truck would look relatively smaller.
3. Should the **battery move to a rear fuel cell** with an access door and an adjustable clamp (p55), or stay in the tub?
4. Should the **cosmetic cage be body-mounted** (like the Y-380, p43-46) while the current cage stays as the structural rear hoop?
5. Do we want a **front-hinged shell** that opens for service (p46)? It needs a rear pivot on the cage.
6. Is the 28.6 deg **rear shock lean** negotiable? The Yeti shock is near vertical [I]. A taller shock tower or a shorter shock would reduce it.
7. Is a **sway bar** wanted at all?
8. Should I fetch the true **wheelbase, track and tyre OD** for the Yeti from outside the manual to confirm the 1/3 ratio? I only have [X] values.
9. How many tubes/hoops do you want on the cage given the 2.0 to 2.7 mm tube limit?
10. Is an **interior with 2 figures** worth the print time?

## Confidence notes

- Strong [M]: part numbers, bag/step order, screw sizes, shock/spring/shaft specs, 38.5 / 38 / 25 / 8.5 mm dimensions, anti-squat 3-hole note, battery limit, oils, ratio table.
- Medium [V]: assembly layout, which part attaches to which (bumper-block/arm-pivot, Y-brace, fuel cell + hoops, body-mounted cage).
- Weak [I]: all scaled link lengths, tube diameter, hole pitch, wall height, shock lean angle, hood/cabin proportions. Do not design to these numbers without checking a physical Yeti or an overhead photo.
- [X]: wheelbase about 328 mm and tyre about 140 to 145 mm are from general knowledge, not the manual.
