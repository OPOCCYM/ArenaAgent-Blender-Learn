#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Render the built car scene with desktop Blender's Cycles engine.

Run from repository root:

    blender -b -noaudio car_realistic.blend --python render_blender_cycles.py

This must run in a desktop/native Blender build with Cycles enabled.
Blender WASM (Arena runtime) can build/save the scene but cannot render.

PNGs are saved to renders/cycles/ and car_realistic.blend is left with
its exterior camera active.
"""
import os
import bpy

ROOT = os.getcwd()
BLEND_PATH = os.path.join(ROOT, 'car_realistic.blend')
OUT_DIR = os.path.join(ROOT, 'renders', 'cycles')

if bpy.app.background is False:
    print('Rendering in Blender UI session')
if not bpy.app.build_options.cycles:
    raise RuntimeError(
        'This Blender build has no Cycles support; '
        'use a desktop Blender build with Cycles enabled'
    )

scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.device = 'CPU'
scene.cycles.samples = 48
scene.cycles.use_denoising = True
scene.cycles.transparent_max_bounces = 48   # обязательно (иначе муар на стёклах)
scene.cycles.blur_glossy = 2.0              # обязательно (иначе муар на стёклах)
scene.cycles.max_bounces = 8
scene.cycles.transmission_bounces = 6
scene.cycles.glossy_bounces = 4
scene.cycles.caustics_reflective = False
scene.cycles.caustics_refractive = False
scene.cycles.sample_clamp_indirect = 6.0
scene.render.resolution_x = 1440
scene.render.resolution_y = 810
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.render.film_transparent = False
try:
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.look = 'AgX - Medium High Contrast'
except Exception:
    pass

os.makedirs(OUT_DIR, exist_ok=True)

shots = (
    ('front_3_4', 'Cam_Exterior'),
    ('side', 'Cam_Side'),
    ('rear_3_4', 'Cam_Rear'),
    ('interior', 'Cam_Interior'),
    ('interior2', 'Cam_Interior2'),
    ('doors_open', 'Cam_Exterior'),
)

for slug, camera_name in shots:
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        raise RuntimeError(f'Required render camera missing: {camera_name}')
    scene.camera = camera
    scene.render.filepath = os.path.join(OUT_DIR, f'car_ue_{slug}.png')
    bpy.ops.render.render(write_still=True)
    print('CYCLES_RENDERED', scene.render.filepath)

# вернуться к внешней камере и сохранить сцену
scene.camera = bpy.data.objects['Cam_Exterior']
scene.render.filepath = '//renders/cycles/car_ue_front_3_4.png'
bpy.ops.wm.save_as_mainfile(filepath=BLEND_PATH)
print('SAVED_BLEND', BLEND_PATH)
