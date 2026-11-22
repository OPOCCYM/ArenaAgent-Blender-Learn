#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
turntable.py — облёт камерой на 360°, 240 уникальных кадров, видео 20 с (12 fps)
или пинг-понг вперёд+реверс (--pingpong).

    blender -b -noaudio car_realistic.blend -P turntable.py
    blender -b -noaudio car_realistic.blend -P turntable.py -- --frames 240 --fps 12

Кадры пишутся в renders/turntable/frame_####.png; если в системе есть ffmpeg,
собирается renders/turntable.mp4.
"""
import argparse
import math
import os
import shutil
import subprocess
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE and HERE not in sys.path:
    sys.path.insert(0, HERE)

from car_geometry import TARGET_COORDS  # noqa: E402

TARGET = Vector(TARGET_COORDS)
RADIUS = 7.6
HEIGHT = 1.75
DEFAULT_FRAMES = 240
DEFAULT_FPS = 12


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def build_ffmpeg_command(frames_dir, mp4_path, fps=DEFAULT_FPS, pingpong=False):
    """Формирует безопасный список аргументов ffmpeg (без shell=True)."""
    pattern = os.path.join(frames_dir, 'frame_%04d.png')
    cmd = ['ffmpeg', '-y', '-framerate', str(fps), '-i', pattern]
    if pingpong:
        cmd += ['-vf', 'split[a][b];[b]reverse,trim=start_frame=1[r];[a][r]concat']
    cmd += ['-c:v', 'libx264', '-pix_fmt', 'yuv420p', mp4_path]
    return cmd


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Облёт автомобиля камерой на 360°')
    parser.add_argument('--frames', type=int, default=DEFAULT_FRAMES,
                        help='Число уникальных кадров на полный круг (по умолчанию 240)')
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS,
                        help='Частота кадров MP4 (по умолчанию 12 -> 20 с)')
    parser.add_argument('--out-dir', default='renders',
                        help='Базовая директория рендеров (по умолчанию renders)')
    parser.add_argument('--pingpong', action='store_true',
                        help='Добавить обратный ход (вперёд + реверс без дубля стыка)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or [])
    n_frames = max(1, args.frames)
    scene = bpy.context.scene
    cam = bpy.data.objects.get('Cam_Turntable')
    if cam is None:
        camd = bpy.data.cameras.new('Cam_Turntable')
        camd.lens = 62.0
        camd.sensor_width = 36.0
        cam = bpy.data.objects.new('Cam_Turntable', camd)
        bpy.context.scene.collection.objects.link(cam)
    scene.camera = cam

    base_dir = os.path.abspath(args.out_dir)
    frames_dir = os.path.join(base_dir, 'turntable')
    os.makedirs(frames_dir, exist_ok=True)

    # n_frames шагов от 0 до 2*pi*(n-1)/n: бесшовный цикл без дублирования кадра 0 и 2*pi
    for i in range(n_frames):
        t = 2.0 * math.pi * i / float(n_frames)
        cam.location = Vector((RADIUS * math.cos(t), -RADIUS * math.sin(t), HEIGHT))
        look_at(cam, TARGET)
        scene.render.filepath = os.path.join(frames_dir, f'frame_{i:04d}.png')
        bpy.ops.render.render(write_still=True)
        if i % 30 == 0 or i == n_frames - 1:
            print('кадр', i + 1, 'из', n_frames)

    mp4_path = os.path.join(base_dir, 'turntable.mp4')
    if shutil.which('ffmpeg'):
        cmd = build_ffmpeg_command(frames_dir, mp4_path, fps=args.fps, pingpong=args.pingpong)
        res = subprocess.run(cmd, check=False)
        if res.returncode != 0:
            print(f'ошибка ffmpeg (код {res.returncode})')
            sys.exit(res.returncode)
        print('видео:', mp4_path)
    else:
        print('ffmpeg не найден — сохранены только кадры PNG в', frames_dir)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
