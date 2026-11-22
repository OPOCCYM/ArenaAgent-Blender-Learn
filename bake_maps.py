#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bake_maps.py — генерация файловых карт текстур (normal / roughness / AO)
для кузова, колёс и салона. Сохраняет в maps/ как PNG.

Запуск:
    blender -b -noaudio -P bake_maps.py

Генерируемые карты (procedural, без внешних файлов, 1024×1024, PNG):
  maps/normal_body.png      — нормали кузова (волновой узор панелей)
  maps/roughness_body.png   — шероховатость кузова (шум + градиент)
  maps/ao_body.png          — ambient occlusion кузова (в person затемнения)
  maps/normal_tire.png      — нормали протектора шины (ламели + канавки)
  maps/roughness_tire.png   — шероховатость шины
  maps/normal_leather.png   — нормали кожи сидений (зерно + прострочка)
  maps/roughness_leather.png — шероховатость кожи

Все карты — 32-bit RGBA PNG, sRGB выключен (linear) для normal/roughness/AO.
Материалы в car_build.py подхватывают эти карты через Image Texture ноды,
если файлы существуют; иначе работают чисто процедурные fallback-ноды.
"""
import os
import math
import random

import bpy

MAPS_DIR = os.path.abspath('maps')
SIZE = 1024


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ensure_maps_dir():
    os.makedirs(MAPS_DIR, exist_ok=True)


def new_image(name):
    if name in bpy.data.images:
        img = bpy.data.images[name]
        # очистить старые пиксели
        img.pixels = [0.0] * (SIZE * SIZE * 4)
    else:
        img = bpy.data.images.new(name, width=SIZE, height=SIZE, alpha=True)
    return img


def save_image(img, filename):
    img.filepath = os.path.join(MAPS_DIR, filename)
    img.file_format = 'PNG'
    img.colorspace_settings.name = 'Linear' if 'Normal' in img.name or 'Roughness' in img.name or 'AO' in img.name else 'sRGB'
    img.save()


def fill_grayscale(img, fn):
    """Заполнить изображение оттенками серого по функции fn(vx, vy) -> 0..1."""
    pixels = []
    for y in range(SIZE):
        vy = y / (SIZE - 1)
        for x in range(SIZE):
            vx = x / (SIZE - 1)
            val = clamp(fn(vx, vy), 0.0, 1.0)
            pixels.extend([val, val, val, 1.0])
    img.pixels.foreach_set(pixels)


def fill_normal(img, fn):
    """Заполнить нормальную карту: fn(vx, vy) -> (nx, ny, nz) в диапазоне -1..1,
    упаковывается в RGB (0..1)."""
    pixels = []
    for y in range(SIZE):
        vy = y / (SIZE - 1)
        for x in range(SIZE):
            vx = x / (SIZE - 1)
            nx, ny, nz = fn(vx, vy)
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            if length > 0:
                nx /= length
                ny /= length
                nz /= length
            pixels.extend([nx * 0.5 + 0.5, ny * 0.5 + 0.5, nz * 0.5 + 0.5, 1.0])
    img.pixels.foreach_set(pixels)


# --------------------------------------------------------------------------- #
# Карты кузова
# --------------------------------------------------------------------------- #
def make_body_normal():
    img = new_image('Normal_Body')

    def fn(vx, vy):
        # волновой узор панелей кузова по X и Y
        nx = 0.15 * math.sin(vx * 24.0) * math.cos(vy * 12.0)
        ny = 0.10 * math.sin(vy * 18.0) * math.cos(vx * 8.0)
        nz = 0.95
        return (nx, ny, nz)

    fill_normal(img, fn)
    save_image(img, 'normal_body.png')
    print('  saved maps/normal_body.png')


def make_body_roughness():
    img = new_image('Roughness_Body')
    random.seed(42)

    def fn(vx, vy):
        base = 0.22 + 0.08 * vy  # чуть больше шероховатости к низу кузова
        noise = 0.04 * random.random()
        panel = 0.025 * math.sin(vx * 36.0) * math.cos(vy * 24.0)
        return base + noise + panel

    fill_grayscale(img, fn)
    save_image(img, 'roughness_body.png')
    print('  saved maps/roughness_body.png')


def make_body_ao():
    img = new_image('AO_Body')

    def fn(vx, vy):
        cx = vx - 0.5
        cy = vy - 0.5
        dist = math.sqrt(cx * cx + cy * cy)
        vignette = 1.0 - clamp(dist * 0.75, 0.0, 0.55)
        bottom = 1.0 - clamp((0.5 - cy) * 1.4, 0.0, 0.35)
        return clamp(vignette * bottom, 0.45, 1.0)

    fill_grayscale(img, fn)
    save_image(img, 'ao_body.png')
    print('  saved maps/ao_body.png')


# --------------------------------------------------------------------------- #
# Карты колёс (шины)
# --------------------------------------------------------------------------- #
def make_tire_normal():
    img = new_image('Normal_Tire')

    def fn(vx, vy):
        # ламели протектора по окружности (вертикальные полосы в UV)
        lamella = 0.30 * math.sin(vx * 72.0)
        # канавки по ширине шины
        groove = 0.20 * math.sin(vy * 48.0)
        nz = 0.90
        return (lamella, groove, nz)

    fill_normal(img, fn)
    save_image(img, 'normal_tire.png')
    print('  saved maps/normal_tire.png')


def make_tire_roughness():
    img = new_image('Roughness_Tire')
    random.seed(11)

    def fn(vx, vy):
        base = 0.88
        noise = 0.09 * random.random()
        return clamp(base + noise, 0.0, 1.0)

    fill_grayscale(img, fn)
    save_image(img, 'roughness_tire.png')
    print('  saved maps/roughness_tire.png')


# --------------------------------------------------------------------------- #
# Карты кожи салона
# --------------------------------------------------------------------------- #
def make_leather_normal():
    img = new_image('Normal_Leather')
    random.seed(13)

    def fn(vx, vy):
        grain = 0.06 * random.random()
        # прострочка — горизонтальные полосы с промежутками
        stitch = 0.18 * math.sin(vy * 96.0) * (0.5 + 0.5 * math.sin(vx * 240.0))
        return (grain, stitch, 0.88)

    fill_normal(img, fn)
    save_image(img, 'normal_leather.png')
    print('  saved maps/normal_leather.png')


def make_leather_roughness():
    img = new_image('Roughness_Leather')
    random.seed(17)

    def fn(vx, vy):
        base = 0.55
        noise = 0.12 * random.random()
        return clamp(base + noise, 0.0, 1.0)

    fill_grayscale(img, fn)
    save_image(img, 'roughness_leather.png')
    print('  saved maps/roughness_leather.png')


# --------------------------------------------------------------------------- #
# Точка входа
# --------------------------------------------------------------------------- #
def main():
    ensure_maps_dir()
    print('bake_maps.py: генерация файловых карт текстур в', MAPS_DIR)
    make_body_normal()
    make_body_roughness()
    make_body_ao()
    make_tire_normal()
    make_tire_roughness()
    make_leather_normal()
    make_leather_roughness()
    print('bake_maps.py: ГОТОВО — 7 карт сохранены в maps/')


if __name__ == '__main__':
    main()
