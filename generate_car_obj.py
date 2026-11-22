#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_car_obj.py — процедурная генерация OBJ/MTL модели седана без Blender.

Строит кузов-лофт из сечений по длине автомобиля, колёса, стёкла и упрощённый салон.
Вывод: exports/car_ue.obj + exports/car_ue.mtl

Запуск:
    python3 generate_car_obj.py
    python3 render_obj_png.py exports/car_ue.obj   # → 4 PNG в renders/
"""
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
OUT_OBJ = ROOT / 'exports' / 'car_ue.obj'
OUT_MTL = ROOT / 'exports' / 'car_ue.mtl'

# ---------- параметры автомобиля (седан, ≈ 5.0 м) ----------
L = 5.00   # длина
W = 1.872  # ширина
H = 1.456  # высота
WB = 2.95  # колёсная база
RW = 0.34  # радиус колёса
TW = 0.22  # ширина шины

def section(u):
    """Сечение кузова при u в [0,1] от носа к корме.
    Возвращает список точек (x_offset, y, z) в локальных координатах сечения."""
    # профиль сечения: верхняя линия, боковые, низ
    # упрощённо: полуовальная cabine + плоский низ
    pts = []
    # низ: от левого к правому по y (колесные арки внизу)
    for t in np.linspace(0, 1, 25):
        ang = math.pi * t
        pts.append((0.0, -W * 0.5 * math.sin(ang), 0.15 + 0.08 * (1 - math.cos(ang))))
    # боковые вверх до пояса (пояс на высоте ~0.85*H)
    for t in np.linspace(0, 1, 12):
        pts.append((0.0, W * 0.5 * (1 - t), 0.15 + 0.85 * H * t))
    # крыша: от правого пояса к левому (на pleine высоте H)
    roof_drop = 0.12 * H * (1 - abs(2 * u - 1) ** 0.6)  # легкий скос к корме
    for t in np.linspace(0, 1, 14):
        yy = W * 0.5 * (1 - t)
        zz = 0.15 + 0.85 * H - roof_drop * t
        pts.append((0.0, yy, zz))
    # левая сторона вниз
    for t in np.linspace(0, 1, 12):
        pts.append((0.0, -W * 0.5 * t, 0.15 + 0.85 * H * (1 - t)))
    return pts

def loft_body(n_slices=40):
    """Генерирует вершины и грани кузова-лофта."""
    verts = []
    faces = []
    vbase = 0
    slices = []
    for i in range(n_slices):
        u = i / (n_slices - 1)
        # общая длина: от -L/2 к +L/2
        x = -L / 2 + u * L
        pts = section(u)
        slice_verts = []
        for (dx, yy, zz) in pts:
            verts.append((x + dx * 0.05, yy, zz))
            slice_verts.append(len(verts) - 1)
        slices.append(slice_verts)
    # грани между соседними срезами
    for i in range(n_slices - 1):
        a = slices[i]
        b = slices[i + 1]
        n = len(a)
        for j in range(n):
            j2 = (j + 1) % n
            faces.append((a[j], b[j], b[j2], 'Body'))
            faces.append((a[j], b[j2], a[j2], 'Body'))
    return verts, faces

def wheel(cx, cy, cz, r=RW, w=TW, n=32):
    """Колено: диск + шина. Возвращает verts/faces."""
    verts = []
    faces = []
    # шина: внешний цилиндр
    outer = []
    inner = []
    for i in range(n + 1):
        ang = 2 * math.pi * i / n
        ca, sa = math.cos(ang), math.sin(ang)
        outer.append((cx + r * ca, cy + w * 0.52 * sa, cz + r * sa))
        inner.append((cx + (r - 0.04) * ca, cy + w * 0.5 * sa, cz + (r - 0.04) * sa))
    base = len(verts)
    for p in outer:
        verts.append(p)
    for p in inner:
        verts.append(p)
    for i in range(n):
        i0 = base + i
        i1 = base + (i + 1) % n
        i2 = base + n + (i + 1) % n
        i3 = base + n + i
        faces.append((i0, i1, i2, 'Tire'))
        faces.append((i0, i2, i3, 'Tire'))
    # диск (центр + спицы упрощённо)
    disc_base = len(verts)
    verts.append((cx, cy, cz))  # центр
    for i in range(n):
        ang = 2 * math.pi * i / n
        ca, sa = math.cos(ang), math.sin(ang)
        verts.append((cx + (r - 0.06) * ca, cy, cz + (r - 0.06) * sa))
    for i in range(n):
        faces.append((disc_base, disc_base + 1 + i, disc_base + 1 + (i + 1) % n, 'Wheel_Disc'))
    return verts, faces

def glass(cx, cy, cz, sx, sy, sz, name):
    """Простое стекло — куб/плоскость."""
    verts = []
    faces = []
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    base = len(verts)
    for xx in (-hx, hx):
        for yy in (-hy, hy):
            for zz in (-hz, hz):
                verts.append((cx + xx, cy + yy, cz + zz))
    # 6 граней куба (внешние)
    cube_faces = [
        (0, 1, 3), (0, 3, 2),   # низ
        (4, 6, 7), (4, 7, 5),   # верх
        (0, 4, 5), (0, 5, 1),   # зад
        (2, 3, 7), (2, 7, 6),   # перед
        (0, 2, 6), (0, 6, 4),   # лево
        (1, 5, 7), (1, 7, 3),   # право
    ]
    for tri in cube_faces:
        faces.append((base + tri[0], base + tri[1], base + tri[2], name))
    return verts, faces

def main():
    OUT_OBJ.parent.mkdir(parents=True, exist_ok=True)

    all_verts = []
    all_faces = []
    mat_usage = {}

    def add(vs, fs, mat):
        off = len(all_verts)
        all_verts.extend(vs)
        for f in fs:
            all_faces.append((f[0] + off, f[1] + off, f[2] + off, mat))
        mat_usage[mat] = mat_usage.get(mat, 0) + 1

    # кузов
    bv, bf = loft_body()
    add(bv, bf, 'Body')

    # колёса (4)
    wheel_positions = [
        (-WB / 2 + 0.55, -W / 2 - 0.02, 0.34),
        (-WB / 2 + 0.55,  W / 2 + 0.02, 0.34),
        ( WB / 2 - 0.55, -W / 2 - 0.02, 0.34),
        ( WB / 2 - 0.55,  W / 2 + 0.02, 0.34),
    ]
    for (wx, wy, wz) in wheel_positions:
        wv, wf = wheel(wx, wy, wz)
        add(wv, wf, 'Tire')
        # диск отдельным материалом — перекрасим грани диска
        # (в этом упрощённом генераторе диск входит в Wheel_Disc)

    # стёкла (лобовое, задние — внутри габаритов кузова)
    # лобовое
    gv, gf = glass(0.0, 0.0, 1.05, L * 0.50, 0.05, 0.55, 'Glazing')
    add(gv, gf, 'Glazing')
    # задние (внутри кузова, не выходят за габариты)
    gv, gf = glass(L / 2 - 0.25, 0.0, 1.02, 0.5, 0.05, 0.5, 'Glazing')
    add(gv, gf, 'Glazing')

    # салон упрощённо (панель + сиденья) — группы Interior_* для cockpit рендера
    # панель приборов
    gv, gf = glass(-0.6, -W / 2 - 0.01, 0.85, 1.6, 0.04, 0.5, 'Dashboard')
    add(gv, gf, 'Interior_Dark')
    # сиденье водителя
    gv, gf = glass(-0.4, W / 2 + 0.01, 0.55, 0.5, 0.04, 0.55, 'Front_Seat_D')
    add(gv, gf, 'Leather')
    # сиденье пассажира
    gv, gf = glass(-0.4, -W / 2 - 0.01, 0.55, 0.5, 0.04, 0.55, 'Front_Seat_P')
    add(gv, gf, 'Leather')

    # ---------- запись OBJ ----------
    lines = ['# CarUE procedural sedan', '# generated by generate_car_obj.py', '']
    mtllib = 'car_ue.mtl'
    lines.append(f'mtllib {mtllib}')
    lines.append('')
    for v in all_verts:
        lines.append(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}')
    lines.append('')
    cur_mat = None
    for (ia, ib, ic, mat) in all_faces:
        if mat != cur_mat:
            cur_mat = mat
            lines.append(f'o {cur_mat}')
            lines.append(f'usemtl {cur_mat}')
        lines.append(f'f {ia+1} {ib+1} {ic+1}')
    OUT_OBJ.write_text('\n'.join(lines) + '\n')

    # ---------- запись MTL ----------
    mtl_lines = []
    def mtl(name, Kd, Ks=(0.15, 0.15, 0.15), Ns=60, d=1.0):
        mtl_lines.append(f'newmtl {name}')
        mtl_lines.append(f'Kd {Kd[0]:.3f} {Kd[1]:.3f} {Kd[2]:.3f}')
        mtl_lines.append(f'Ks {Ks[0]:.3f} {Ks[1]:.3f} {Ks[2]:.3f}')
        mtl_lines.append(f'Ns {Ns}')
        mtl_lines.append(f'd {d:.3f}')
        mtl_lines.append('')
    mtl('Body', (0.22, 0.26, 0.32), (0.3, 0.3, 0.3), 90)
    mtl('Tire', (0.06, 0.06, 0.07), (0.05, 0.05, 0.05), 10)
    mtl('Wheel_Disc', (0.35, 0.35, 0.4), (0.5, 0.5, 0.5), 120)
    mtl('Glazing', (0.85, 0.9, 0.95), (0.1, 0.1, 0.1), 50, d=0.65)
    mtl('Interior_Dark', (0.18, 0.18, 0.2), (0.1, 0.1, 0.1), 20)
    mtl('Leather', (0.35, 0.25, 0.2), (0.1, 0.08, 0.06), 15)
    OUT_MTL.write_text('\n'.join(mtl_lines) + '\n')

    print(f'OBJ: {OUT_OBJ}  verts={len(all_verts)} faces={len(all_faces)}')
    print(f'MTL: {OUT_MTL}')
    print('Материалы:', mat_usage)

if __name__ == '__main__':
    main()
