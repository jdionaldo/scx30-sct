"""Headless Blender script to render multi-view CAD previews of the SCX30 SCT Option 2 x Option 3
Desert Trophy Truck shell RIGHT ON TOP OF the actual SCX30 SCT chassis, wheels, red cage,
blue front IFS, and green rear trailing arms!

Run with:
    /Applications/Blender.app/Contents/MacOS/Blender --background --python sct_shell_variant/render_cad.py
"""
import math
import os
import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXPORT_DIR = os.path.join(HERE, "exports")
RENDER_DIR = os.path.join(HERE, "renders")
os.makedirs(RENDER_DIR, exist_ok=True)

NEW_PARTS = {
    "SCT_Cyber_Trophy_Body": ((0.18, 0.21, 0.17, 1.0),   0.15, 0.50),
    "SCT_Hex_Mesh_Grille":   ((0.07, 0.08, 0.09, 1.0),   0.45, 0.40),
    "SCT_Cyan_DRL_Bar":      ((0.02, 0.90, 0.98, 1.0),   0.0,  0.10),
}

# Existing SCX30 SCT chassis/suspension/wheel context STLs for visual verification
CONTEXT_PARTS = [
    ("print/parts/Cage.stl",                (0.55, 0.08, 0.07, 1.0), 0.15, 0.50),
    ("print/parts/Chassis_Tub.stl",         (0.15, 0.16, 0.18, 1.0), 0.15, 0.65),
    ("print/parts/Front_Bumper.stl",        (0.08, 0.22, 0.55, 1.0), 0.20, 0.45),
    ("print/parts/IFS_Lower_Arm_L.stl",     (0.08, 0.28, 0.65, 1.0), 0.20, 0.45),
    ("print/parts/IFS_Lower_Arm_R.stl",     (0.08, 0.28, 0.65, 1.0), 0.20, 0.45),
    ("print/parts/IFS_Upper_Arm_L.stl",     (0.08, 0.28, 0.65, 1.0), 0.20, 0.45),
    ("print/parts/IFS_Upper_Arm_R.stl",     (0.08, 0.28, 0.65, 1.0), 0.20, 0.45),
    ("print/parts/Rear_Trailing_Arm_L.stl", (0.08, 0.42, 0.18, 1.0), 0.20, 0.45),
    ("print/parts/Rear_Trailing_Arm_R.stl", (0.08, 0.42, 0.18, 1.0), 0.20, 0.45),
    ("stl/v3/Ref_Wheel_FL.stl",             (0.06, 0.06, 0.065, 1.0), 0.05, 0.78),
    ("stl/v3/Ref_Wheel_FR.stl",             (0.06, 0.06, 0.065, 1.0), 0.05, 0.78),
    ("stl/v4/Ref_Wheel_RL.stl",             (0.06, 0.06, 0.065, 1.0), 0.05, 0.78),
    ("stl/v4/Ref_Wheel_RR.stl",             (0.06, 0.06, 0.065, 1.0), 0.05, 0.78),
]

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items else "CYCLES"
if scene.render.engine == "CYCLES":
    scene.cycles.samples = 48
scene.render.resolution_x = 1280
scene.render.resolution_y = 800

world = bpy.data.worlds.new("StudioWorld")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
if bg:
    bg.inputs[0].default_value = (0.90, 0.91, 0.92, 1.0)
    bg.inputs[1].default_value = 0.9

def import_stl(path, name, rgba, metal, rough, emit=False):
    if not os.path.exists(path):
        return
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=path, global_scale=0.001)
    else:
        bpy.ops.import_mesh.stl(filepath=path, global_scale=0.001)
    obj = bpy.context.active_object
    obj.name = name
    mat = bpy.data.materials.new(name=f"Mat_{name}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = rgba
        bsdf.inputs["Metallic"].default_value = metal
        bsdf.inputs["Roughness"].default_value = rough
        if emit and "Emission Color" in bsdf.inputs:
            bsdf.inputs["Emission Color"].default_value = rgba
            bsdf.inputs["Emission Strength"].default_value = 4.0
    obj.data.materials.append(mat)

for name, (rgba, m, r) in NEW_PARTS.items():
    import_stl(os.path.join(EXPORT_DIR, f"{name}.stl"), name, rgba, m, r, emit=(name == "SCT_Cyan_DRL_Bar"))

for rel, rgba, m, r in CONTEXT_PARTS:
    import_stl(os.path.join(ROOT, rel), os.path.basename(rel)[:-4], rgba, m, r)

# Ground pad at z = -0.0165 (SCX30 ground plane)
bpy.ops.mesh.primitive_plane_add(size=0.45, location=(-0.01, 0.0, -0.0165))
ground = bpy.context.active_object
gmat = bpy.data.materials.new("Ground")
gmat.use_nodes = True
gbsdf = gmat.node_tree.nodes.get("Principled BSDF")
if gbsdf:
    gbsdf.inputs["Base Color"].default_value = (0.25, 0.26, 0.28, 1.0)
    gbsdf.inputs["Roughness"].default_value = 0.85
ground.data.materials.append(gmat)

def add_light(name, ltype, loc, energy, rot=(0, 0, 0), size=0.35):
    ldata = bpy.data.lights.new(name=name, type=ltype)
    ldata.energy = energy
    if ltype == "AREA":
        ldata.size = size
    lobj = bpy.data.objects.new(name=name, object_data=ldata)
    scene.collection.objects.link(lobj)
    lobj.location = loc
    lobj.rotation_euler = rot

add_light("Key", "AREA", (0.30, -0.28, 0.35), 14.0, (math.radians(50), 0, math.radians(40)), 0.4)
add_light("Fill", "AREA", (-0.25, 0.30, 0.25), 8.0, (math.radians(60), 0, math.radians(-135)), 0.4)
add_light("Rim", "AREA", (-0.30, -0.15, 0.30), 10.0, (math.radians(-50), 0, 0), 0.4)

cam_data = bpy.data.cameras.new("Cam")
cam_data.lens = 55.0
cam = bpy.data.objects.new("Cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam

def look_at(pos, target):
    cam.location = Vector(pos)
    cam.rotation_euler = (Vector(target) - cam.location).to_track_quat("-Z", "Y").to_euler()

# SCX30 frame: +X forward, +Y left, +Z up
views = [
    ("sct_iso.png",   (0.22, -0.19, 0.12), (-0.005, 0.0, 0.018)),
    ("sct_front.png", (0.26,  0.00, 0.04), ( 0.055, 0.0, 0.022)),
    ("sct_side.png",  (-0.01, -0.26, 0.03), (-0.01,  0.0, 0.018)),
    ("sct_top.png",   (-0.01,  0.00, 0.28), (-0.01,  0.0, 0.020)),
]

for fname, pos, tgt in views:
    look_at(pos, tgt)
    scene.render.filepath = os.path.join(RENDER_DIR, fname)
    bpy.ops.render.render(write_still=True)
    print("Rendered:", scene.render.filepath)
