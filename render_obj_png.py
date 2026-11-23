#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_obj_png.py — software-растеризатор OBJ/MTL в PNG (NumPy + Pillow).

Это НЕ Blender/Cycles-рендер. Он полезен в песочнице без Blender для
проверки пропорций, UV-развертки и просмотра промежуточных кадров.

Улучшения по сравнению с первой версией:
  * 8 ракурсов: front_3_4, side, rear_3_4, cockpit, front, rear, wheel, detail
  * Многозональное освещение (Key/Fill/Rim/Strip, как в car_build.py)
  * Материалы: металлик (Body, Wheel_Disc), стекло (Glazing), резина (Tire),
    кожа (Leather), пластик (Interior_Dark)
  * Прозрачность стёкол (d из MTL)
  * Антиалиасинг через суперсемплирование 2×
  * Wireframe-режим (--wireframe)
  * Турнтейбл-последовательность (--turntable N)
  * Настраиваемое разрешение (--width, --height)

Запуск:
    python3 render_obj_png.py [exports/car_ue.obj] [--width 1920 --height 1080]
    python3 render_obj_png.py --wireframe --turntable 36
    python3 render_obj_png.py --views front_3_4,side,cockpit

Выход: renders/car_ue_<view>.png (или car_ue_turntable_XXXX.png)
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parent
DEFAULT_OBJ = ROOT / 'exports' / 'car_ue.obj'
OUT = ROOT / 'renders'

# --------------------------------------------------------------------------- #
# Константы рендера
# --------------------------------------------------------------------------- #
DEFAULT_W, DEFAULT_H = 1280, 960
SUPERSAMPLE = 2  # антиалиасинг 2× (внутреннее разрешение ×2)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Software OBJ → PNG рендерер')
    p.add_argument('obj', nargs='?', default=str(DEFAULT_OBJ), help='Путь к .obj')
    p.add_argument('--width', type=int, default=DEFAULT_W, help='Ширина кадра')
    p.add_argument('--height', type=int, default=DEFAULT_H, help='Высота кадра')
    p.add_argument(
        '--views',
        default='front_3_4,side,rear_3_4,cockpit,front,rear,wheel,detail',
        help='Список ракурсов через запятую',
    )
    p.add_argument(
        '--turntable', type=int, default=0,
        help='Сгенерировать N кадров турнтейбла (0 = отключено)',
    )
    p.add_argument(
        '--wireframe', action='store_true',
        help='Рисовать только ребра (wireframe)',
    )
    p.add_argument(
        '--supersample', type=int, default=SUPERSAMPLE,
        help='Коэффициент суперсемплирования (1 = выкл)',
    )
    return p.parse_args()


# --------------------------------------------------------------------------- #
# Загрузка OBJ/MTL
# --------------------------------------------------------------------------- #
def read_mtl(path: Path) -> dict:
    mats: dict = {}
    cur = None
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'newmtl':
            cur = parts[1]
            mats[cur] = {'Kd': (0.7, 0.7, 0.7), 'Ks': (0.1, 0.1, 0.1), 'Ns': 80, 'd': 1.0}
        elif cur and parts[0] == 'Kd':
            mats[cur]['Kd'] = tuple(map(float, parts[1:4]))
        elif cur and parts[0] == 'Ks':
            mats[cur]['Ks'] = tuple(map(float, parts[1:4]))
        elif cur and parts[0] == 'Ns':
            mats[cur]['Ns'] = float(parts[1])
        elif cur and parts[0] == 'd':
            mats[cur]['d'] = float(parts[1])
    return mats


def load_obj(path: Path):
    verts = []
    mats = read_mtl(path.with_suffix('.mtl')) if path.with_suffix('.mtl').exists() else {}
    tris = []
    curmat = 'DefaultGrey'
    group = ''
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v':
            verts.append(tuple(map(float, parts[1:4])))
        elif parts[0] == 'usemtl':
            curmat = parts[1]
        elif parts[0] == 'o':
            group = parts[1]
        elif parts[0] == 'f':
            ids = [int(x.split('/')[0]) - 1 for x in parts[1:]]
            for i in range(1, len(ids) - 1):
                tris.append((ids[0], ids[i], ids[i + 1], curmat, group))
    return np.asarray(verts, dtype=float), tris, mats


# --------------------------------------------------------------------------- #
# Геометрические утилиты
# --------------------------------------------------------------------------- #
def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def camera_project(points, camera, target, fov=35, aspect=None):
    forward = normalize(target - camera)
    world_up = np.array([0.0, 0.0, 1.0])
    right = normalize(np.cross(forward, world_up))
    up = normalize(np.cross(right, forward))
    rel = points - camera
    xyz = np.stack([rel @ right, rel @ up, rel @ forward], axis=1)

    if aspect is None:
        aspect = 1.0
    focal = 0.5 * aspect / math.tan(math.radians(fov / 2))
    sx = aspect / 2 + xyz[:, 0] * focal / xyz[:, 2]
    sy = 0.5 - xyz[:, 1] * focal / xyz[:, 2]
    return np.stack([sx, sy, xyz[:, 2]], axis=1), forward


def make_background(w, h, top=(35, 45, 62), bottom=(190, 201, 212)):
    yy = np.linspace(0, 1, h)[:, None, None]
    top_c = np.array(top)[None, None, :]
    bottom_c = np.array(bottom)[None, None, :]
    grad = (top_c * (1 - yy) + bottom_c * yy).astype(np.uint8)
    return Image.fromarray(np.repeat(grad, w, axis=1), 'RGB').convert('RGBA')


def draw_shadow(img, cx, cy, rx, ry, alpha=125, blur=28):
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    d.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), fill=(16, 20, 28, alpha))
    layer = layer.filter(ImageFilter.GaussianBlur(blur))
    img.alpha_composite(layer)


# --------------------------------------------------------------------------- #
# Основной конвейер рендера одного кадра
# --------------------------------------------------------------------------- #
def render_view(
    verts,
    tris,
    mats,
    name,
    camera,
    target,
    width=DEFAULT_W,
    height=DEFAULT_H,
    fov=35,
    wireframe=False,
    supersample=SUPERSAMPLE,
):
    """Отрисовывает один вид и сохраняет PNG."""
    sw, sh = width * supersample, height * supersample
    proj, view_dir = camera_project(
        verts, np.array(camera, dtype=float), np.array(target, dtype=float),
        fov=fov, aspect=sw / sh,
    )

    img = make_background(sw, sh)
    if name != 'Cockpit':
        draw_shadow(img, sw * 0.51, sh * 0.69, sw * 0.26, sh * 0.055)

    zbuf = np.full((sh, sw), np.inf, dtype=np.float32)
    rgb = np.zeros((sh, sw, 3), dtype=np.float32)

    # Свет: Key / Fill / Rim / Strip (как в car_build.py)
    lights = [
        {'dir': normalize(np.array([-0.45, -0.55, 0.95])), 'intensity': 1.0, 'color': (1.0, 0.98, 0.92)},   # Key
        {'dir': normalize(np.array([0.55, -0.30, 0.60])), 'intensity': 0.45, 'color': (0.92, 0.96, 1.0)},  # Fill
        {'dir': normalize(np.array([-0.10, 0.70, 0.70])), 'intensity': 0.55, 'color': (1.0, 1.0, 1.0)},   # Rim
        {'dir': normalize(np.array([0.00, -0.90, 0.40])), 'intensity': 0.30, 'color': (0.95, 0.90, 0.85)}, # Strip
    ]
    ambient = 0.25

    # Гладкие нормали вершин (для затенения кузова)
    smooth_normals = np.zeros_like(verts)
    for ia, ib, ic, _, _ in tris:
        n = np.cross(verts[ib] - verts[ia], verts[ic] - verts[ia])
        smooth_normals[ia] += n
        smooth_normals[ib] += n
        smooth_normals[ic] += n
    lengths = np.linalg.norm(smooth_normals, axis=1)
    valid = lengths > 1e-10
    smooth_normals[valid] /= lengths[valid, None]

    def shade_triangle(ia, ib, ic, matname, group):
        q = proj[[ia, ib, ic]]
        if np.any(q[:, 2] <= 0.05):
            return
        xy = q[:, :2]
        x0 = max(0, int(np.floor(xy[:, 0].min())))
        x1 = min(sw - 1, int(np.ceil(xy[:, 0].max())))
        y0 = max(0, int(np.floor(xy[:, 1].min())))
        y1 = min(sh - 1, int(np.ceil(xy[:, 1].max())))
        if x1 <= x0 or y1 <= y0:
            return

        v0, v1, v2 = xy
        den = (v1[1] - v2[1]) * (v0[0] - v2[0]) + (v2[0] - v1[0]) * (v0[1] - v2[1])
        if abs(den) < 1e-8:
            return

        xs = np.arange(x0, x1 + 1, dtype=np.float32)[None, :] + 0.5
        ys = np.arange(y0, y1 + 1, dtype=np.float32)[:, None] + 0.5
        a = ((v1[1] - v2[1]) * (xs - v2[0]) + (v2[0] - v1[0]) * (ys - v2[1])) / den
        b = ((v2[1] - v0[1]) * (xs - v2[0]) + (v0[0] - v2[0]) * (ys - v2[1])) / den
        c = 1.0 - a - b
        inside = (a >= -1e-5) & (b >= -1e-5) & (c >= -1e-5)
        invz = a / q[0, 2] + b / q[1, 2] + c / q[2, 2]
        zs = zbuf[y0:y1 + 1, x0:x1 + 1]
        mask = inside & (invz > zs)
        if not mask.any():
            return

        p = verts[[ia, ib, ic]]
        n_face = np.cross(p[1] - p[0], p[2] - p[0])
        nl = np.linalg.norm(n_face)
        if nl < 1e-10:
            return
        n_face = n_face / nl

        m = mats.get(matname, {'Kd': (0.7, 0.7, 0.7), 'Ks': (0.1, 0.1, 0.1), 'Ns': 80, 'd': 1.0})
        base = np.array(m['Kd'], dtype=np.float32)
        ks = float(np.mean(m['Ks']))
        ns = float(m.get('Ns', 80))
        d_alpha = float(m.get('d', 1.0))

        if wireframe:
            # Рисуем только ребра треугольника
            color = np.array([0.15, 0.25, 0.35], dtype=np.float32)
            patch = rgb[y0:y1 + 1, x0:x1 + 1]
            edge = (
                (np.abs(a * b) < 1e-3) | (np.abs(b * c) < 1e-3) | (np.abs(c * a) < 1e-3)
            ) & mask
            patch[edge] = color
            zs[mask] = invz[mask]
            return

        # Материальное затенение
        if group in ('Body', 'Wheel_Disc', 'Body_Shell', 'Cabin'):
            # Гладкое затенение (фонг)
            ns_v = smooth_normals[[ia, ib, ic]]
            nn = a[..., None] * ns_v[0] + b[..., None] * ns_v[1] + c[..., None] * ns_v[2]
            nn /= np.maximum(np.linalg.norm(nn, axis=2, keepdims=True), 1e-8)
            flip = (nn @ view_dir) < 0
            nn[flip] *= -1

            nd = np.zeros_like(a)
            spec = np.zeros_like(a)
            for lt in lights:
                lnd = np.maximum(0, nn @ lt['dir'])
                nd += lnd * lt['intensity']
                hv = normalize(lt['dir'] + view_dir)
                spec += np.maximum(0, nn @ hv) ** ns * lt['intensity']

            nd = ambient + nd
            shade = np.clip(nd, 0, 1)
            colors = shade[..., None] * base
            colors = colors + ks * spec[..., None] * 1.4
            colors = np.clip(colors, 0, 1)
            # Металлический оттенок для Body / Wheel_Disc
            if matname in ('Body', 'Wheel_Disc'):
                colors = colors * 0.9 + np.array([0.22, 0.26, 0.32], dtype=np.float32) * 0.1
        elif matname == 'Glazing':
            # Стекло: полупрозрачное, отражения
            if np.dot(n_face, view_dir) < 0:
                n_face = -n_face
            nd = max(0.0, float(np.dot(n_face, normalize(np.array([-0.45, -0.55, 0.95])))))
            shade = ambient + 0.7 * nd
            color = base * shade
            # Отметка прозрачности
            alpha_mask = d_alpha
            patch = rgb[y0:y1 + 1, x0:x1 + 1]
            patch[mask] = (color * 255 * alpha_mask).astype(np.uint8)
            # Смешиваем с тем, что уже есть (простое alpha-смешивание)
            existing = patch[mask].astype(np.float32) / 255.0
            blended = existing * (1 - alpha_mask) + color * alpha_mask
            patch[mask] = (blended * 255).astype(np.uint8)
            zs[mask] = invz[mask]
            return
        else:
            # Tire, Leather, Interior_Dark — плоское затенение
            if np.dot(n_face, view_dir) < 0:
                n_face = -n_face
            nd = max(0.0, float(np.dot(n_face, normalize(np.array([-0.45, -0.55, 0.95])))))
            spec = max(0.0, float(np.dot(n_face, normalize(np.array([-0.45, -0.55, 0.95]) + view_dir)))) ** 20
            shade = ambient + 0.7 * nd + 0.2 * spec
            color = np.clip(base * shade + ks * spec * 1.2, 0, 1)

        patch = rgb[y0:y1 + 1, x0:x1 + 1]
        patch[mask] = (colors[mask] * 255).astype(np.uint8)
        zs[mask] = invz[mask]

    # Рasterize все треугольники
    for ia, ib, ic, matname, group in tris:
        shade_triangle(ia, ib, ic, matname, group)

    model = Image.fromarray(np.clip(rgb, 0, 1).astype(np.uint8), 'RGB').convert('RGBA')
    alpha = Image.fromarray(((zbuf < np.inf) & (zbuf > 0)).astype(np.uint8) * 255, 'L')
    img.alpha_composite(Image.composite(model, Image.new('RGBA', (sw, sh)), alpha))

    # Даунсэмплинг для антиалиасинга
    if supersample > 1:
        img = img.resize((width, height), Image.LANCZOS)

    # Текст подписи
    draw = ImageDraw.Draw(img)
    draw.rounded_rectangle((24, 24, 360, 80), radius=10, fill=(10, 18, 30, 170))
    draw.text((46, 38), f'CarUE  /  {name}', fill=(246, 248, 250, 255))
    draw.text(
        (28, height - 30), 'PROCEDURAL MESH PREVIEW  •  NOT A CYCLES RENDER',
        fill=(30, 40, 52, 200),
    )

    slug = name.lower().replace(' ', '_').replace('/', '_')
    out = OUT / f'car_ue_{slug}.png'
    img.convert('RGB').save(out)
    print(out)


# --------------------------------------------------------------------------- #
# Наборы ракурсов
# --------------------------------------------------------------------------- #
def default_views() -> dict:
    target = np.array([0.0, 0.0, 0.63])
    return {
        'front_3_4': dict(camera=[-6.4, -7.0, 3.2], target=target, fov=35),
        'side': dict(camera=[-0.1, -8.7, 2.35], target=target, fov=35),
        'rear_3_4': dict(camera=[6.5, 7.0, 3.1], target=target, fov=35),
        'cockpit': dict(camera=[0.34, -0.28, 1.12], target=[-1.2, -0.28, 0.96], fov=68),
        'front': dict(camera=[0.0, -8.2, 2.6], target=target, fov=35),
        'rear': dict(camera=[0.0, 8.2, 2.6], target=target, fov=35),
        'wheel': dict(camera=[-2.6, -1.1, 0.55], target=[-2.6, -1.1, 0.55], fov=55),
        'detail': dict(camera=[-3.2, -4.0, 2.0], target=[-3.2, -4.0, 0.6], fov=45),
    }


def turntable_views(n: int) -> list:
    target = np.array([0.0, 0.0, 0.63])
    radius = 9.0
    height = 2.8
    views = []
    for i in range(n):
        ang = 2 * math.pi * i / n
        cam = np.array([radius * math.cos(ang), radius * math.sin(ang), height])
        views.append((f'turntable_{i:04d}', dict(camera=cam.tolist(), target=target.tolist(), fov=35)))
    return views


# --------------------------------------------------------------------------- #
# Точка входа
# --------------------------------------------------------------------------- #
def main() -> None:
    args = parse_args()
    obj_path = Path(args.obj).resolve()
    if not obj_path.exists():
        print(f'OBJ не найден: {obj_path}', file=sys.stderr)
        sys.exit(1)

    OUT.mkdir(parents=True, exist_ok=True)
    verts, tris, mats = load_obj(obj_path)

    width, height = args.width, args.height
    supersample = args.supersample
    wireframe = args.wireframe

    views = default_views()
    selected = [v.strip() for v in args.views.split(',') if v.strip()]

    # Выбранные ракурсы
    for name in selected:
        if name not in views:
            print(f'Пропуск неизвестного ракурса: {name}', file=sys.stderr)
            continue
        cfg = views[name]
        render_view(
            verts, tris, mats, name,
            camera=cfg['camera'], target=np.array(cfg['target']),
            width=width, height=height, fov=cfg.get('fov', 35),
            wireframe=wireframe, supersample=supersample,
        )

    # Турнтейбл
    if args.turntable and args.turntable > 0:
        for name, cfg in turntable_views(args.turntable):
            render_view(
                verts, tris, mats, name,
                camera=np.array(cfg['camera']), target=np.array(cfg['target']),
                width=width, height=height, fov=cfg.get('fov', 35),
                wireframe=wireframe, supersample=supersample,
            )


if __name__ == '__main__':
    main()
