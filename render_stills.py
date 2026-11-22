#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_stills.py — перерендер 5 финальных кадров без пересборки сцены.

    blender -b -noaudio car_realistic.blend -P render_stills.py

Рендерит камеры Cam_Exterior, Cam_Rear, Cam_Side, Cam_Interior,
Cam_Interior2 в renders/*.png (настройки рендера берутся из сцены:
Cycles CPU, 48 spp, 1440x810, transparent_max_bounces=48, blur_glossy=2.0).
"""
import os

import bpy

CAMERAS = ('Cam_Exterior', 'Cam_Rear', 'Cam_Side',
           'Cam_Interior', 'Cam_Interior2')


def main():
    scene = bpy.context.scene
    os.makedirs('renders', exist_ok=True)
    for name in CAMERAS:
        cam = bpy.data.objects.get(name)
        if cam is None:
            print(f'пропуск: камера {name} не найдена')
            continue
        scene.camera = cam
        scene.render.filepath = os.path.abspath(
            os.path.join('renders', name + '.png'))
        bpy.ops.render.render(write_still=True)
        print('отрендерено:', name)


if __name__ == '__main__':
    main()
