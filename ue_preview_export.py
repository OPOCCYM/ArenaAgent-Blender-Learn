#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ue_preview_export.py — предварительные рендеры + экспорт FBX для Unreal Engine.

Запуск после сборки сцены car_build.py:

    blender -b -noaudio car_realistic.blend -P ue_preview_export.py

Что делает (в этой последовательности):

  1. Рендерит 5 финальных кадров камер в renders/  (предварительные рендеры
     перед экспортом — для проверки качества материалов, освещения, отражений
     на стёклах, silhouette по макро-кадрам: нос, корма, арка, двери, крыша).
  2. Экспортирует FBX в exports/ для Unreal Engine (логика export_ue.py):
     car_ue.fbx, car_ue_anim.fbx, car_ue_col.fbx.

Полный порядок итерации:

    blender -b -noaudio -P car_build.py                 # сцена + .blend
    blender -b -noaudio car_realistic.blend -P qa.py     # геометрия (нули)
    blender -b -noaudio car_realistic.blend -P ue_preview_export.py  # рендер + FBX

Все скрипты проверены на синтаксис (py_compile). Фактический запуск требует
среды с Blender 4.2.
"""
import os
import sys
import importlib.util

import bpy

HERE = os.path.dirname(os.path.abspath(__file__)) or os.getcwd()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def render_stills():
    """Рендер 5 финальных кадров через render_stills.py."""
    mod_path = os.path.join(HERE, 'render_stills.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: render_stills.py не найден')
        return False
    mod = load_module('render_stills_mod', mod_path)
    mod.main()
    return True


def export_ue():
    """Экспорт FBX под UE через export_ue.py."""
    mod_path = os.path.join(HERE, 'export_ue.py')
    if not os.path.exists(mod_path):
        print('ue_preview_export.py: export_ue.py не найден')
        return False
    mod = load_module('export_ue_mod', mod_path)
    mod.main()
    return True


def main():
    scene = bpy.context.scene
    print('ue_preview_export.py: начинаем предварительный рендер и экспорт')

    # --- этап 1: предварительные рендеры 5 кадров камер ---
    print('ue_preview_export.py: этап 1/2 — рендер статичных кадров')
    if not render_stills():
        print('ue_preview_export.py: рендер не удался, прерываем')
        return

    # --- этап 2: экспорт FBX под UE ---
    print('ue_preview_export.py: этап 2/2 — экспорт FBX для Unreal Engine')
    export_ue()

    print('ue_preview_export.py: ГОТОВО')
    print('  рендеры: renders/Cam_*.png')
    print('  FBX:     exports/car_ue{,_anim,_col}.fbx')


if __name__ == '__main__':
    main()
