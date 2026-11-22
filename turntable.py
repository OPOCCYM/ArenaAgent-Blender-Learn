#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
turntable.py — облёт камерой на 360°, 241 кадр, видео 20 с (вперёд + реверс).

    blender -b -noaudio car_realistic.blend -P turntable.py

Кадры пишутся в renders/turntable/frame_###.png; если в системе есть ffmpeg,
собирается renders/turntable.mp4.
"""
import math
import os
import subprocess
import sys

import bpy
from mathutils import Vector

TARGET = Vector((0.05, 0.0, 0.74))
RADIUS = 7.6
HEIGHT = 1.75
N_FRAMES = 241


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def main():
    scene = bpy.context.scene
    cam = bpy.data.objects.get('Cam_Turntable')
    if cam is None:
        camd = bpy.data.cameras.new('Cam_Turntable')
        camd.lens = 62.0
        camd.sensor_width = 36.0
        cam = bpy.data.objects.new('Cam_Turntable', camd)
        bpy.context.scene.collection.objects.link(cam)
    scene.camera = cam

    out_dir = os.path.abspath('renders/turntable')
    os.makedirs(out_dir, exist_ok=True)

    for i in range(N_FRAMES):
        t = 2.0 * math.pi * i / (N_FRAMES - 1)
        cam.location = Vector((RADIUS * math.cos(t), -RADIUS * math.sin(t), HEIGHT))
        look_at(cam, TARGET)
        scene.render.filepath = os.path.join(out_dir, f'frame_{i:04d}.png')
        bpy.ops.render.render(write_still=True)
        if i % 30 == 0:
            print('кадр', i, 'из', N_FRAMES - 1)

    mp4 = os.path.abspath('renders/turntable.mp4')
    if subprocess.run(['which', 'ffmpeg'], capture_output=True).returncode == 0:
        cmd = ('ffmpeg -y -r 12 -i renders/turntable/frame_%04d.png '
               '-vf "split[a][b];[b]reverse[r];[a][r]concat" '
               '-c:v libx264 -pix_fmt yuv420p ' + mp4)
        subprocess.run(cmd, shell=True, check=False)
        print('видео:', mp4)
    else:
        print('ffmpeg не найден — кадрами ограничились')


if __name__ == '__main__':
    main()
