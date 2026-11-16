"""Parametric Surface-First CadQuery (OCCT) generator for the 1/30 SCX30 Stealth Trophy Body Shell."""

import cadquery as cq

# --- 1. PARAMETRIC CONSTANTS (1/30 SCX30 Conversion Scale) ---
params = {
    "scale_wheelbase": 107.5,     # mm (SCX30 107.5 mm stretch; front X=+48.0, rear X=-59.5)
    "track_width": 85.0,          # mm outer tire-to-tire
    "wall_thickness": 1.2,        # mm 3D-printable shell wall (1/30 scale)
    "joint_overlap": 2.5,         # mm joggle flange overlap
    "joint_clearance": 0.25,      # mm slip/glue clearance
    "min_crease_fillet": 0.6,     # mm styling crease fillet
    "blend_fillet": 1.5,          # mm flare/aero transition fillet
    "shutline_gap": 0.5,          # mm styled panel reveal gap
    "arch_corner_r": 3.0,         # mm minimum wheel-arch / cutout corner radius
}

wall_t = params["wall_thickness"]

def loft_yz(sections, ruled=True):
    """Loft a list of (x_pos, pts) closed YZ profiles into one solid."""
    wp = cq.Workplane("YZ")
    for x_pos, pts in sections:
        w = cq.Workplane("YZ").workplane(offset=x_pos).polyline(pts).close().wires().val()
        wp.ctx.pendingWires.append(w)
    return wp.loft(ruled=ruled)

def loft_xz(sections, ruled=True):
    """Loft a list of (y_pos, pts) closed XZ profiles into one solid."""
    wp = cq.Workplane("XZ")
    for y_pos, pts in sections:
        w = cq.Workplane("XZ").workplane(offset=-y_pos).polyline(pts).close().wires().val()
        wp.ctx.pendingWires.append(w)
    return wp.loft(ruled=ruled)

def loft_xy(sections, ruled=True):
    """Loft a list of (z_pos, pts) closed XY profiles into one solid."""
    wp = cq.Workplane("XY")
    for z_pos, pts in sections:
        w = cq.Workplane("XY").workplane(offset=z_pos).polyline(pts).close().wires().val()
        wp.ctx.pendingWires.append(w)
    return wp.loft(ruled=ruled)

# --- 2. MASTER SURFACE CONSTRUCTION (+Y HALF-BODY LOFT) ---
# Continuous envelope from nose -> hood -> cowl -> windshield -> raked roof -> B-pillar -> rear deck -> tail.
# Clears:
# - Roll_Cage (Y<=15.4, nose Z<=12.3, mid-nose Z<=14.3, cowl Z<=16.95, roof Z<=29.95)
# - Slider_L/R (Y<=20.5, Z<=6.5)
# - Chassis_Rail_L/R (Y<=15.5, rear Z<=8.6) & Ref_Rear_Sway_Bar (Z<=7.95)
# - Front Shock Tower (X=52, Z<=12.2) & Rear Axle (X=-55, Z<=5.0)
master_sections = [
    (55.5, [(0.0, 11.75), (11.0, 11.75), (15.0, 11.95), (15.4, 12.9), (10.0, 13.9), (3.8, 14.4), (0.0, 14.4)]),
    (40.0, [(0.0, 13.4), (13.6, 13.4), (18.4, 14.2), (18.8, 16.2), (12.2, 17.8), (4.6, 18.5), (0.0, 18.5)]),
    (22.0, [(0.0, 12.8), (17.2, 12.8), (20.8, 13.4), (20.2, 16.4), (14.4, 19.1), (5.2, 19.9), (0.0, 19.9)]),
    (17.0, [(0.0, 6.7),  (20.6, 6.7),  (22.0, 10.0), (20.6, 16.8), (16.4, 19.6), (5.5, 20.4), (0.0, 20.4)]),
    (3.0,  [(0.0, 6.7),  (20.6, 6.7),  (22.0, 10.0), (17.6, 28.4), (15.6, 29.3), (7.0, 29.7), (0.0, 29.7)]),
    (-21.5,[(0.0, 6.7),  (20.6, 6.7),  (22.0, 10.0), (17.2, 30.6), (15.4, 31.5), (6.8, 31.9), (0.0, 31.9)]),
    (-42.0,[(0.0, 8.9),  (21.4, 8.9),  (23.0, 11.6), (21.9, 15.8), (15.8, 15.8), (8.0, 15.8), (0.0, 15.8)]),
    (-58.0,[(0.0, 8.9),  (19.6, 8.9),  (21.2, 11.8), (19.4, 15.8), (15.6, 15.8), (8.0, 15.8), (0.0, 15.8)]),
]
master_half = loft_yz(master_sections, ruled=True)

# --- 3. SECONDARY FEATURES: FLUSH-LOFTED FENDER FLARES & REAR V-WING ---
# Front fender flare: clean 1.4mm uniform-thickness arch lofted flush from body shoulder (Y=15.0..26.4)
f_flare = loft_xz([
    (15.0, [(22.0, 12.6), (31.0, 14.8), (48.0, 14.2), (50.8, 12.8), (50.8, 14.1), (48.0, 15.5), (40.0, 16.2), (30.5, 16.4), (22.0, 14.0)]),
    (26.4, [(23.5, 12.2), (31.0, 14.4), (44.0, 13.8), (46.5, 12.0), (46.5, 13.3), (44.0, 15.1), (40.0, 15.6), (30.5, 15.8), (23.5, 13.6)]),
], ruled=True)

# Rear fender flare (uniform 1.4mm arch lip from Y=15.6 out to Y=26.4, level Z=15.8 top, clearing rear sway bar Z=7.95)
r_flare = (
    cq.Workplane("XZ")
    .workplane(offset=-15.6)
    .polyline([(-27.5, 8.8), (-38.5, 14.4), (-55.0, 14.4), (-58.0, 12.6), (-58.0, 14.0), (-55.0, 15.8), (-38.5, 15.8), (-27.5, 10.2)])
    .close()
    .extrude(-10.8)
)

# Swept rear V-wing (+Y, entire base embedded flush into Z=15.8 fender deck behind X=-46.0 window)
r_wing = loft_xy([
    (15.0, [(-46.5, 16.0), (-46.5, 18.8), (-57.5, 18.8), (-57.5, 16.0)]),
    (24.8, [(-53.5, 20.5), (-53.5, 22.8), (-61.5, 22.8), (-61.5, 20.5)]),
], ruled=True)

master_half = master_half.union(f_flare).union(r_flare).union(r_wing)

# Blend exterior flare transitions
for box in [((22, 18, 8), (54, 22, 17)), ((-59, 18, 8), (-26, 22, 17))]:
    try:
        master_half = master_half.edges(cq.selectors.BoxSelector(*box)).fillet(0.8)
    except Exception:
        pass

# --- 4. HOLLOWING & CLEAN WINDOW / WHEEL-ARCH / SHOCK CUTOUTS ---
# Inner core hollowed cleanly to clear Roll_Cage (Y<=15.4, roof Z<=29.95, cowl Z<=16.95, nose Z<=12.3)
inner_sections = [
    (54.2, [(0.0, 11.6), (10.0, 11.6), (14.0, 11.8), (14.2, 12.2), (9.5, 12.8), (3.6, 13.3), (0.0, 13.3)]),
    (40.0, [(0.0, 12.0), (16.2, 12.0), (17.0, 14.2), (17.4, 15.2), (11.5, 16.6), (4.4, 17.3), (0.0, 17.3)]),
    (22.0, [(0.0, 11.8), (16.8, 11.8), (19.6, 13.4), (19.0, 15.4), (13.6, 17.9), (4.8, 18.7), (0.0, 18.7)]),
    (17.0, [(0.0, 6.6),  (19.4, 6.6),  (20.6, 10.0), (19.4, 16.4), (15.5, 18.4), (5.0, 19.2), (0.0, 19.2)]),
    (3.0,  [(0.0, 6.6),  (19.4, 6.6),  (20.6, 10.0), (16.4, 27.8), (15.5, 28.2), (6.5, 28.6), (0.0, 28.6)]),
    (-21.5,[(0.0, 6.6),  (19.4, 6.6),  (20.6, 10.0), (16.0, 30.1), (15.4, 30.4), (6.4, 30.8), (0.0, 30.8)]),
    (-42.0,[(0.0, 8.8),  (20.1, 8.8),  (21.6, 11.6), (20.7, 15.0), (15.5, 16.0), (7.5, 16.0), (0.0, 16.0)]),
    (-56.8,[(0.0, 8.8),  (18.2, 8.8),  (19.8, 11.8), (18.0, 14.6), (15.4, 15.2), (7.5, 15.2), (0.0, 15.2)]),
]
master_half = master_half.cut(loft_yz(inner_sections, ruled=True))

# Cut Windshield Opening (Y = 0..15.0) — clean full-height opening with zero hanging brow plate
wind_cut_xz = (
    cq.Workplane("XZ")
    .workplane(offset=0.0)
    .polyline([(15.5, 17.5), (4.2, 28.0), (4.2, 31.8), (18.5, 21.5)])
    .close()
    .extrude(-15.0)
)
master_half = master_half.cut(wind_cut_xz)

# Extend Rear C-Pillar Fastback Sail Panel flush with roof bevel (15.4,31.5)-(17.2,30.6) at uniform 2.4mm perpendicular width down to Z=15.8 fender deck
c_pillar_sail = loft_yz([
    (-21.5, [(15.4, 28.0), (17.2, 28.0), (17.2, 30.6), (15.4, 31.5)]),
    (-50.1, [(15.55, 15.0), (17.5, 15.0), (17.5, 18.4), (15.55, 18.4)]),
], ruled=True)
master_half = master_half.union(c_pillar_sail)

# Cut Side Window Opening (Y = 14.0..24.0) with chamfered stealth corners
side_win_cut = (
    cq.Workplane("XZ")
    .workplane(offset=-14.0)
    .polyline([(15.04, 18.0), (0.40, 27.2), (-16.8, 29.2), (-18.2, 27.8), (-18.2, 18.0)])
    .close()
    .extrude(-10.0)
)
master_half = master_half.cut(side_win_cut)

# Cut Rear Bed / Fastback Open Bay (inboard of Y=15.6 behind B-pillar X=-21.8)
rear_bay_cut = (
    cq.Workplane("XY")
    .workplane(offset=7.0)
    .moveTo(-21.8, 0.0)
    .lineTo(-60.0, 0.0)
    .lineTo(-60.0, 15.6)
    .lineTo(-21.8, 15.6)
    .close()
    .extrude(25.0)
)
master_half = master_half.cut(rear_bay_cut)

# Cut Rear Shock Hoop Clearance Window (Y=15.4..23.9, behind X=-23.5) — parallel to C-pillar sail bottom
shock_hoop_cut = (
    cq.Workplane("XZ")
    .workplane(offset=-15.4)
    .polyline([(-50.5, 15.82), (-23.5, 27.15), (-23.5, 15.82)])
    .close()
    .extrude(-8.5)
)
master_half = master_half.cut(shock_hoop_cut)

# Front & rear arches already clear wheels/tires with uniform 1.4mm flare walls (no extra arch chop needed)

# Recessed Hood Louver + 3 Angled Slats + Grille Detail (never piercing through)
hood_louver_cut = (
    cq.Workplane("XY")
    .workplane(offset=17.6)
    .center(33.5, 0.0)
    .rect(11.0, 7.6)
    .extrude(3.0)
)
louver_floor = (
    cq.Workplane("XY")
    .workplane(offset=16.7)
    .center(33.5, 0.0)
    .rect(12.2, 8.8)
    .extrude(0.8)
)
slat_0 = cq.Workplane("XY").workplane(offset=17.2).center(30.5, 0.0).rect(1.2, 7.8).extrude(1.0)
slat_1 = cq.Workplane("XY").workplane(offset=17.0).center(33.5, 0.0).rect(1.2, 7.8).extrude(1.0)
slat_2 = cq.Workplane("XY").workplane(offset=16.8).center(36.5, 0.0).rect(1.2, 7.8).extrude(1.0)

grille_cut = (
    cq.Workplane("YZ")
    .workplane(offset=54.8)
    .center(0.0, 12.0)
    .rect(14.0, 1.4)
    .extrude(1.2)
)
master_half = (
    master_half
    .cut(hood_louver_cut)
    .union(louver_floor)
    .union(slat_0).union(slat_1).union(slat_2)
    .cut(grille_cut)
)

# Underside Front Hood + Rear Deck Magnetic Mount Bosses + Pockets (recessed underneath, never piercing top!)
mag_boss = (
    cq.Workplane("XY")
    .workplane(offset=12.35)
    .center(46.5, 9.5)
    .rect(7.6, 7.6)
    .extrude(2.0)
)
mag_pocket = (
    cq.Workplane("XY")
    .workplane(offset=12.35)
    .center(46.5, 9.5)
    .circle(3.1)
    .extrude(1.45)
)
rear_mag_boss = (
    cq.Workplane("XY")
    .workplane(offset=14.4)
    .center(-52.0, 17.3)
    .rect(6.8, 3.4)
    .extrude(1.4)
)
rear_mag_pocket = (
    cq.Workplane("XY")
    .workplane(offset=14.4)
    .center(-52.0, 17.3)
    .circle(3.1)
    .extrude(1.0)
)
master_half = master_half.union(mag_boss).cut(mag_pocket).union(rear_mag_boss).cut(rear_mag_pocket)

# --- 5. MIRRORING & VALIDATION ---
final_shell = master_half.mirror("XZ", union=True)

is_valid = final_shell.val().isValid()
print("CadQuery final_shell isValid():", is_valid)
assert is_valid, "OCCT validation failed!"

step_out = "/Users/johndionaldo/GitHub/scx30-sct/step/tacoma_trophy_shell_cq.step"
stl_out = "/Users/johndionaldo/GitHub/scx30-sct/stl/tacoma_trophy_shell.stl"
cq.exporters.export(final_shell, step_out)
cq.exporters.export(final_shell, stl_out)
print("Exported:", step_out)
print("Exported:", stl_out)
