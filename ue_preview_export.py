#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ue_preview_export.py — файловые карты → предварительные рендеры → экспорт FBX для UE.

Запуск после сборки сцены car_build.py:

    blender -b -noaudio car_realistic.blend -P ue_preview_export.py

Что делает (в этой строгой последовательности):

  0. Генерирует файловые карты текстур (normal / roughness / AO) в maps/
     через bake_maps.py — важно для UE, т.к. процедурные ноды Blender не
     экспортируются как текстуры.
  1. Рендерит 5 финальных кадров камер в renders/ — промежуточные рендеры
     после наложения файловых карт для проверки качества материалов,
     освещения, отражений на стёклах, silhouette по макро-кадрам:
     нос, корма, арка, двери, крыша.
  2. Экспортирует FBX в exports/ для Unreal Engine (логика export_ue.py):
     car_ue.fbx, car_ue_anim.fbx, car_ue_col.fbx.

Полный порядок итерации:

    blender -b -noaudio -P car_build.py                 # сцена + .blend
    blender -b -noaudio car_realistic.blend -P qa.py     # геометрия (нули)
    blender -b -noaudio car_realistic.blend -P ue_preview_export.py  # карты → рендеры → FBX
"""
import os
import importlib.util

import bpy

HERE = os.path.dirname(os.path.abspath(__file__)) or os.getcwd()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def bake_maps():
    """Этап 0/3 — генерация файловых карт текстур в maps/."""
    mod_path = os.path.join(HERE, 'bake_maps.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: bake_maps.py не найден — пропускаем bake')
        return False
    mod = load_module('bake_maps_mod', mod_path)
    mod.main()
    return True


def render_stills():
    """Этап 1/3 — рендер 5 финальных кадров камер (промежуточные рендеры после bake)."""
    mod_path = os.path.join(HERE, 'render_stills.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: render_stills.py не найден')
        return False
    mod = load_module('render_stills_mod', mod_path)
    mod.main()
    return True


def export_ue():
    """Этап 2/3 — экспорт FBX под UE через export_ue.py."""
    mod_path = os.path.join(HERE, 'export_ue.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: export_ue.py не найден')
        return False
    mod = load_module('export_ue_mod', mod_path)
    mod.main()
    return True


def main():
    scene = bpy.context.scene
    print('ue_preview_export.py: начинаем пайплайн карты → рендеры → экспорт')

    # --- этап 0: файловые карты ---
    print('ue_preview_export.py: этап 0/3 — bake файловых карт текстур')
    bake_maps()

    # --- этап 1: предварительные рендеры 5 кадров камер ---
    print('ue_preview_export.py: этап 1/3 — рендер статичных кадров (после bake)')
    if not render_stills():
        print('ue_preview_export.py: рендер не удался, прерываем')
        return

    # --- этап 2: экспорт FBX под UE ---
    print('ue_preview_export.py: этап 2/3 — экспорт FBX для Unreal Engine')
    export_ue()

    print('ue_preview_export.py: ГОТОВО')
    print('  карты:   maps/*.png')
    print('  рендеры: renders/Cam_*.png')
    print('  FBX:     exports/car_ue{,_anim,_col}.fbx')


if __name__ == '__main__':
    main()
