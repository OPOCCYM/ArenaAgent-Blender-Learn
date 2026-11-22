#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ue_preview_export.py — предварительные рендеры + экспорт FBX для Unreal Engine.

Запуск после сборки сцены car_build.py:

    blender -b -noaudio car_realistic.blend -P ue_preview_export.py
    blender -b -noaudio car_realistic.blend -P ue_preview_export.py -- --samples 16

Последовательность:
  1. Рендерит 5 финальных кадров камер в renders/ (через render_stills.py).
  2. Экспортирует FBX в exports/ для Unreal Engine (через export_ue.py):
     car_ue.fbx, car_ue_anim.fbx, car_ue_col.fbx.
"""
import argparse
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__)) or os.getcwd()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def render_stills(renders_dir='renders', samples=None):
    """Рендер 5 финальных кадров через render_stills.py."""
    mod_path = os.path.join(HERE, 'render_stills.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: render_stills.py не найден')
        return False
    mod = load_module('render_stills_mod', mod_path)
    return bool(mod.render_all(out_dir=renders_dir, samples=samples))


def export_ue(exports_dir='exports'):
    """Экспорт FBX под UE через export_ue.py."""
    mod_path = os.path.join(HERE, 'export_ue.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: export_ue.py не найден')
        return False
    mod = load_module('export_ue_mod', mod_path)
    return bool(mod.export_all(out_dir=exports_dir))


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Предварительные рендеры + экспорт FBX для UE')
    parser.add_argument('--renders-dir', default='renders',
                        help='Директория сохранения рендеров (по умолчанию renders)')
    parser.add_argument('--exports-dir', default='exports',
                        help='Директория сохранения FBX (по умолчанию exports)')
    parser.add_argument('--samples', type=int, default=None,
                        help='Переопределить число сэмплов Cycles')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or [])
    print('ue_preview_export.py: начинаем предварительный рендер и экспорт')

    # --- этап 1: предварительные рендеры 5 кадров камер ---
    print('ue_preview_export.py: этап 1/2 — рендер статичных кадров')
    if not render_stills(renders_dir=args.renders_dir, samples=args.samples):
        print('ue_preview_export.py: рендер не удался, прерываем')
        sys.exit(1)

    # --- этап 2: экспорт FBX под UE ---
    print('ue_preview_export.py: этап 2/2 — экспорт FBX для Unreal Engine')
    if not export_ue(exports_dir=args.exports_dir):
        print('ue_preview_export.py: экспорт FBX не удался')
        sys.exit(1)

    print('ue_preview_export.py: ГОТОВО')
    print(f'  рендеры: {os.path.abspath(args.renders_dir)}/Cam_*.png')
    print(f'  FBX:     {os.path.abspath(args.exports_dir)}/car_ue{{,_anim,_col}}.fbx')


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
