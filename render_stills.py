#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_stills.py — перерендер 5 финальных кадров без пересборки сцены.

    blender -b -noaudio car_realistic.blend -P render_stills.py
    blender -b -noaudio car_realistic.blend -P render_stills.py -- --samples 16 --out-dir ./renders

Рендерит камеры Cam_Exterior, Cam_Rear, Cam_Side, Cam_Interior,
Cam_Interior2 в renders/*.png (настройки рендера берутся из сцены:
Cycles CPU, 48 spp, 1440x810, transparent_max_bounces=48, blur_glossy=2.0).
"""
import argparse
import os
import sys

import bpy

CAMERAS = ('Cam_Exterior', 'Cam_Rear', 'Cam_Side',
           'Cam_Interior', 'Cam_Interior2')


def render_all(out_dir='renders', samples=None):
    """Рендерит все камеры из CAMERAS. Возвращает список путей; при отсутствии
    хотя бы одной камеры возвращает False."""
    scene = bpy.context.scene
    if samples is not None:
        scene.cycles.samples = samples
    renders_dir = os.path.abspath(out_dir)
    os.makedirs(renders_dir, exist_ok=True)

    missing = []
    rendered = []
    for name in CAMERAS:
        cam = bpy.data.objects.get(name)
        if cam is None:
            print(f'ошибка: камера {name} не найдена')
            missing.append(name)
            continue
        scene.camera = cam
        filepath = os.path.join(renders_dir, name + '.png')
        scene.render.filepath = filepath
        bpy.ops.render.render(write_still=True)
        rendered.append(filepath)
        print('отрендерено:', name)

    if missing:
        print('render_stills.py: отсутствуют обязательные камеры:', ', '.join(missing))
        return False
    return bool(rendered)


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Рендер 5 финальных кадров камер')
    parser.add_argument('--out-dir', default='renders',
                        help='Директория сохранения PNG (по умолчанию renders)')
    parser.add_argument('--samples', type=int, default=None,
                        help='Переопределить число сэмплов Cycles')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or [])
    if not render_all(out_dir=args.out_dir, samples=args.samples):
        sys.exit(1)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
