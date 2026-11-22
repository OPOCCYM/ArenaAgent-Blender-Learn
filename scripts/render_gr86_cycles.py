"""Render the current GR86 Blender scene with desktop Blender's Cycles engine.

Run from repository root:
  blender -b assets/gr86_studio.blend --python scripts/render_gr86_cycles.py

This must run in a desktop/native Blender build with Cycles; Blender WASM used by
Arena can build/save the scene but does not support rendering. PNGs are saved to
assets/renders/cycles/ and the source .blend is left with its exterior camera active.
"""
import os
import bpy

ROOT=os.getcwd()
BLEND_PATH=os.path.join(ROOT,'assets','gr86_studio.blend')
OUT_DIR=os.path.join(ROOT,'assets','renders','cycles')

if bpy.app.background is False:
    print('Rendering in Blender UI session')
if not bpy.app.build_options.cycles:
    raise RuntimeError('This Blender build has no Cycles support; use a desktop Blender build with Cycles enabled')

scene=bpy.context.scene
scene.render.engine='CYCLES'
scene.cycles.samples=64
scene.cycles.use_denoising=True
scene.render.resolution_x=1600
scene.render.resolution_y=1100
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.render.image_settings.color_mode='RGBA'
scene.render.film_transparent=False
try:
    scene.view_settings.view_transform='AgX'
except Exception:
    pass
os.makedirs(OUT_DIR,exist_ok=True)

shots=(
    ('front_3_4','Cam_Exterior'),
    ('side','Cam_Side'),
    ('rear_3_4','Cam_Rear'),
    ('cockpit','Cam_Cockpit'),
)
for slug,camera_name in shots:
    camera=bpy.data.objects.get(camera_name)
    if camera is None:
        raise RuntimeError(f'Required render camera missing: {camera_name}')
    scene.camera=camera
    scene.render.filepath=os.path.join(OUT_DIR,f'gr86_{slug}.png')
    bpy.ops.render.render(write_still=True)
    print('CYCLES_RENDERED',scene.render.filepath)

scene.camera=bpy.data.objects['Cam_Exterior']
scene.render.filepath='//renders/cycles/gr86_front_3_4.png'
bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH)
print('SAVED_BLEND',BLEND_PATH)
