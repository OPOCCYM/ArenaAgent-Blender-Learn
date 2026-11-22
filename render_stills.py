#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_stills.py — перерендер финальных кадров без пересборки сцены.

    blender -b -noaudio car_realistic.blend -P render_stills.py

Рендерит в renders/*.png:

  * Cam_Exterior, Cam_Rear, Cam_Side, Cam_Interior, Cam_Interior2
    (текущий кадр сцены — обычно 1, двери закрыты)
  * Cam_Exterior_DoorsOpen — камера Cam_Exterior на кадре 60 анимации,
    когда все 4 двери открыты наружу на 70°.

Настройки рендера берутся из сцены:
Cycles CPU, 48 spp, 1440x810, transparent_max_bounces=48, blur_glossy=2.0.
"""
import os

import bpy

CAMERAS = ('Cam_Exterior', 'Cam_Rear', 'Cam_Side',
           'Cam_Interior', 'Cam_Interior2')

DOORS_OPEN_FRAME = 60


def main():
    scene = bpy.context.scene
    os.makedirs('renders', exist_ok=True)

    # --- обычные 5 кадров (текущий кадр сцены) ---
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

    # --- кадр с открытыми дверями (кадр 60 анимации) ---
    cam = bpy.data.objects.get('Cam_Exterior')
    if cam is not None:
        scene.camera = cam
        scene.frame_set(DOORS_OPEN_FRAME)
        scene.render.filepath = os.path.abspath(
            os.path.join('renders', 'Cam_Exterior_DoorsOpen.png'))
        bpy.ops.render.render(write_still=True)
        print('отрендерено: Cam_Exterior_DoorsOpen (кадр', DOORS_OPEN_FRAME, ')')
        # вернуть кадр сцены назад к 1
        scene.frame_set(1)
    else:
        print('пропуск: Cam_Exterior не найдена для кадра с открытыми дверями')


if __name__ == '__main__':
    main()
