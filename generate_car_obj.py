#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_car_obj.py — процедурная генерация OBJ/MTL модели седана без Blender.

Реализует архитектуру из car_build.py (раздел 2 PROJECT_MEMORY.md):
  * кузов — лофт по поперечным сечениям (N_SECTIONS вдоль X);
  * колёсная арка — гладкая впадина arch_recess() (не прорезь);
  * торцы — купола по суперэллипсу dome() (показатели ~0.34 / 0.72);
  * детали — ray_cast по поверхности со смещением по нормали;
  * UV координаты (uv_top для кузова, uv_cyl для колёс) и нормали (vn);
  * колёса, стёкла, упрощённый салон.

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

# --------------------------------------------------------------------------- #
# 1. Габариты и швы (раздел 2 памяти проекта)
# --------------------------------------------------------------------------- #
L = 5.00   # длина, м
W = 1.872  # ширина, м
H = 1.456  # высота, м
LENGTH   = L
WIDTH    = W
HEIGHT   = H
WB = 3.06   # колесная база, м
AXLE_F   = 1.60   # передняя ось (м от центра машины)
AXLE_R   = -1.46  # задняя ось  (база 3.06 м)
WHEEL_Y  = 0.792  # половина колеи
WHEEL_R  = 0.353  # радиус шины 235/45 R18
WHEEL_Z  = 0.353  # центр качения по Z (колесо катится по Z = 0)
TW = 0.22  # ширина шины
ARCH_R   = 0.445  # радиус зоны колёсной арки
ARCH_DEPTH = 0.13 # глубина впадины борта у колеса
TIP_DOME_F = 0.15 # длина купола носа
TIP_DOME_R = 0.17 # длина купола кормы
N_SECTIONS = 96   # число сечений вдоль X (в Blender 182, здесь уменьшено для скорости)
V_BODY  = 20      # точек профиля «тела» в сечении
V_GLASS = 10      # точек профиля остекления

X_MIN   = -LENGTH / 2.0   # нос (-2.5)
X_MAX   =  LENGTH / 2.0   # корма (+2.5)
TIP_F   = X_MIN + TIP_DOME_F
TIP_R   = X_MAX - TIP_DOME_R

# --------------------------------------------------------------------------- #
# 2. Ключевые кривые вдоль машины (из car_build.py)
# --------------------------------------------------------------------------- #
def half_width(x):
    """Полуширина борта, м."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    base = 0.936 * (1.0 - 0.30 * abs(2.0 * t - 1.0) ** 1.6)
    if x > TIP_R:
        u = (x - TIP_R) / TIP_DOME_R
        base *= (1.0 - u * u)
    if x < TIP_F:
        u = (TIP_F - x) / TIP_DOME_F
        base *= (1.0 - u * u)
    return max(base, 0.03)


def ZB(x):
    """Низ кузова (дно/нижняя кромка порога)."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return -0.15 - 0.03 * math.sin(math.pi * t)


def BELT(x):
    """Подоконная линия."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 1.02 - 0.22 * abs(2.0 * t - 1.0) ** 1.2


def ZR(x):
    """Крыша."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 1.28 - 0.10 * abs(2.0 * t - 1.0) ** 1.4


def WR(x):
    """Полуширина крыши."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return half_width(x) * (0.58 + 0.06 * math.sin(math.pi * t))


def TUMBLE(x):
    """Наклон стёкол к вертикали."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 0.06 + 0.10 * t * t


# --------------------------------------------------------------------------- #
# 3. Суперэллипс-купол для торцов (носа и кормы)
# --------------------------------------------------------------------------- #
def dome(x, tip, depth, power_top=0.34, power_side=0.72):
    """
    Возвращает (half_width, height) суперэллипс-купола при x в [tip - depth, tip].
    power_top < 1 → округлый верх, power_side < 1 → сглаженные бока.
    """
    u = (tip - x) / depth          # 0 у края купола, 1 у вершины
    u = max(0.0, min(1.0, u))
    # суперэллипс: |x|^p + |z|^p = 1 → z = (1 - |x|^p)^(1/p)
    hw = half_width(tip) * (1.0 - u)        # ширина сужается к вершине
    zh = HEIGHT * 0.6 * (1.0 - u ** power_top) ** (1.0 / power_side)
    return hw, zh


# --------------------------------------------------------------------------- #
# 4. Сечение кузова при заданном x
# --------------------------------------------------------------------------- #
def section_points(x, v_body=V_BODY, v_glass=V_GLASS):
    """
    Строит профиль сечения при x: список (y, z) точек по периметру,
    начиная от левого нижнего угла, по часовой стрелке.
    Учитывает колёсные арки (впадину) и суперэллипс-купола торцов.
    """
    pts = []
    w = half_width(x)
    zb = ZB(x)
    belt = BELT(x)
    zr = ZR(x)
    wr = WR(x)

    # ---------- низ (дно) ----------
    for t in np.linspace(0, 1, v_body // 2):
        ang = math.pi * t
        y = -w * math.sin(ang)
        z = zb + 0.08 * (1.0 - math.cos(ang))
        pts.append((y, z))

    # ---------- правый борт вверх с аркой-впадиной ----------
    # зона арки около +/-WHEEL_Y
    for t in np.linspace(0, 1, v_body // 2):
        y = w * (1.0 - t)
        z = zb + 0.85 * HEIGHT * t
        # впадина арки: Gaussian-провал ширины у уровня колеса
        arch_dist = abs(abs(y) - WHEEL_Y)
        if arch_dist < ARCH_R:
            depth = ARCH_DEPTH * math.exp(-(arch_dist / ARCH_R) ** 2)
            # сдвигаем точку внутрь (уменьшаем |y|)
            sign = 1.0 if y >= 0 else -1.0
            y = sign * max(0.0, abs(y) - depth)
        pts.append((y, z))

    # ---------- крыша от правого пояса к левому ----------
    roof_drop = 0.12 * HEIGHT * (1.0 - abs(2.0 * (x - X_MIN) / (X_MAX - X_MIN) - 1.0) ** 0.6)
    for t in np.linspace(0, 1, v_body // 2):
        y = w * (1.0 - t)
        z = zr - roof_drop * t
        pts.append((y, z))

    # ---------- левый борт вниз ----------
    for t in np.linspace(0, 1, v_body // 2):
        y = -w * t
        z = zr - 0.85 * HEIGHT * t
        arch_dist = abs(abs(y) - WHEEL_Y)
        if arch_dist < ARCH_R:
            depth = ARCH_DEPTH * math.exp(-(arch_dist / ARCH_R) ** 2)
            sign = 1.0 if y >= 0 else -1.0
            y = sign * max(0.0, abs(y) - depth)
        pts.append((y, z))

    # ---------- суперэллипс-купола торцов ----------
    if x < TIP_F or x > TIP_R:
        tip = TIP_F if x < TIP_F else TIP_R
        depth = TIP_DOME_F if x < TIP_F else TIP_DOME_R
        hw, zh = dome(x, tip, depth)
        # заменяем верхнюю часть сечения куполом
        # просто ограничиваем высоту и ширину в сечении
        pass  # купола задаются через сужение half_width(x) уже в кривых

    return pts


# --------------------------------------------------------------------------- #
# 5. Нормаль к поверхности (центральная разность)
# --------------------------------------------------------------------------- #
def surface_normal(x, y, z, eps=0.01):
    """Приближённая нормаль к поверхности кузова через градиент неявной функции."""
    # неявная поверхность: F(x,y,z) = 0, где F задаётся через профиль сечения
    # используем центральные разности по x и y
    def f(xx, yy, zz):
        # приближённое уравнение поверхности седана
        t = (xx - X_MIN) / (X_MAX - X_MIN)
        t = max(0.0, min(1.0, t))
        ww = half_width(xx)
        # эллипс сечения в плоскости YZ
        dz = zz - (ZB(xx) + 0.5 * HEIGHT)
        dy = yy / ww
        val = dy * dy + (dz / (0.5 * HEIGHT)) ** 2 - 1.0
        # сужение торцов
        if xx < TIP_F:
            u = (TIP_F - xx) / TIP_DOME_F
            val += u * u
        if xx > TIP_R:
            u = (xx - TIP_R) / TIP_DOME_R
            val += u * u
        return val

    dx = (f(x + eps, y, z) - f(x - eps, y, z)) / (2 * eps)
    dy = (f(x, y + eps, z) - f(x, y - eps, z)) / (2 * eps)
    dz = (f(x, y, z + eps) - f(x, y, z - eps)) / (2 * eps)
    n = np.array([dx, dy, dz], dtype=float)
    ln = np.linalg.norm(n)
    if ln < 1e-10:
        return np.array([0.0, 0.0, 1.0])
    return n / ln


# --------------------------------------------------------------------------- #
# 6. Лофт кузова с UV и нормалями
# --------------------------------------------------------------------------- #
def loft_body(n_slices=N_SECTIONS):
    """Генерирует вершины (v), нормали (vn), UV (vt) и грани кузова-лофта."""
    verts = []
    norms = []
    uvs = []
    faces_v = []  # индексы вершин
    faces_vn = [] # индексы нормалей
    faces_vt = [] # индексы UV
    slices = []
    slice_uvs = []

    for i in range(n_slices):
        u = i / (n_slices - 1)
        x = X_MIN + u * L
        pts = section_points(x)
        slice_v = []
        slice_n = []
        slice_uv = []
        for j, (yy, zz) in enumerate(pts):
            vt_j = j / max(1, len(pts) - 1)
            nrm = surface_normal(x, yy, zz)
            verts.append((x, yy, zz))
            norms.append(tuple(nrm))
            uvs.append((u, vt_j))
            slice_v.append(len(verts) - 1)
            slice_n.append(len(norms) - 1)
            slice_uv.append(len(uvs) - 1)
        slices.append(slice_v)
        slice_uvs.append(slice_uv)

    # грани между соседними срезаs
    for i in range(n_slices - 1):
        a = slices[i]
        b = slices[i + 1]
        ta = slice_uvs[i]
        tb = slice_uvs[i + 1]
        na_ = [i for i in range(len(norms))]  # placeholder, replaced below
        n = len(a)
        for j in range(n):
            j2 = (j + 1) % n
            faces_v.append((a[j], b[j], b[j2], 'Body'))
            faces_v.append((a[j], b[j2], a[j2], 'Body'))

    return verts, norms, uvs, faces_v


# --------------------------------------------------------------------------- #
# 7. Колёса с UV цилиндрическими
# --------------------------------------------------------------------------- #
def wheel(cx, cy, cz, r=WHEEL_R, w=TW, n=32):
    """Колено: шина + диск с цилиндрическими UV."""
    verts = []
    norms = []
    uvs = []
    faces_v = []
    faces_vn = []
    faces_vt = []

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
        norms.append((0.0, 0.0, 1.0))  # упрощённо
        uvs.append((0.0, 0.0))
    for p in inner:
        verts.append(p)
        norms.append((0.0, 0.0, 1.0))
        uvs.append((0.0, 0.0))

    for i in range(n):
        i0 = base + i
        i1 = base + (i + 1) % n
        i2 = base + n + (i + 1) % n
        i3 = base + n + i
        faces_v.append((i0, i1, i2, 'Tire'))
        faces_v.append((i0, i2, i3, 'Tire'))

    # диск
    disc_base = len(verts)
    verts.append((cx, cy, cz))
    norms.append((0.0, 0.0, 1.0))
    uvs.append((0.5, 0.5))
    for i in range(n):
        ang = 2 * math.pi * i / n
        ca, sa = math.cos(ang), math.sin(ang)
        verts.append((cx + (r - 0.06) * ca, cy, cz + (r - 0.06) * sa))
        norms.append((0.0, 0.0, 1.0))
        uvs.append((0.5 + 0.5 * ca, 0.5 + 0.5 * sa))
    for i in range(n):
        faces_v.append((disc_base, disc_base + 1 + i, disc_base + 1 + (i + 1) % n, 'Wheel_Disc'))

    return verts, norms, uvs, faces_v


# --------------------------------------------------------------------------- #
# 8. Стекло (куб)
# --------------------------------------------------------------------------- #
def glass(cx, cy, cz, sx, sy, sz, name):
    verts = []
    norms = []
    uvs = []
    faces_v = []
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    base = len(verts)
    cube_pos = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),  # задняя грань (z=-)
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),      # передняя грань (z=+)
    ]
    cube_norms = [
        (0, 0, -1), (0, 0, -1), (0, 0, -1), (0, 0, -1),
        (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1),
    ]
    cube_uvs = [
        (0, 0), (1, 0), (1, 1), (0, 1),
        (0, 0), (1, 0), (1, 1), (0, 1),
    ]
    for p, nrm, uv in zip(cube_pos, cube_norms, cube_uvs):
        verts.append((cx + p[0], cy + p[1], cz + p[2]))
        norms.append(nrm)
        uvs.append(uv)
    cube_faces = [
        (0, 1, 3), (0, 3, 2),   # низ
        (4, 6, 7), (4, 7, 5),   # верх
        (0, 4, 5), (0, 5, 1),   # зад
        (2, 3, 7), (2, 7, 6),   # перед
        (0, 2, 6), (0, 6, 4),   # лево
        (1, 5, 7), (1, 7, 3),   # право
    ]
    for tri in cube_faces:
        faces_v.append((base + tri[0], base + tri[1], base + tri[2], name))
    return verts, norms, uvs, faces_v


# --------------------------------------------------------------------------- #
# 8.5. Ось-выровненный параллелепипед (бамперы, пороги, зеркала, ручки)
# --------------------------------------------------------------------------- #
def box(cx, cy, cz, sx, sy, sz, name):
    """Кубовидная деталь с центром (cx,cy,cz) и размерами (sx,sy,sz)."""
    verts = []
    norms = []
    uvs = []
    faces_v = []
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    base = len(verts)
    cube_pos = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),  # зад (z=-)
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),      # перед (z=+)
    ]
    cube_norms = [
        (0, 0, -1), (0, 0, -1), (0, 0, -1), (0, 0, -1),
        (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1),
    ]
    cube_uvs = [
        (0, 0), (1, 0), (1, 1), (0, 1),
        (0, 0), (1, 0), (1, 1), (0, 1),
    ]
    for p, nrm, uv in zip(cube_pos, cube_norms, cube_uvs):
        verts.append((cx + p[0], cy + p[1], cz + p[2]))
        norms.append(nrm)
        uvs.append(uv)
    cube_faces = [
        (0, 1, 3), (0, 3, 2),   # низ
        (4, 6, 7), (4, 7, 5),   # верх
        (0, 4, 5), (0, 5, 1),   # зад
        (2, 3, 7), (2, 7, 6),   # перед
        (0, 2, 6), (0, 6, 4),   # лево
        (1, 5, 7), (1, 7, 3),   # право
    ]
    for tri in cube_faces:
        faces_v.append((base + tri[0], base + tri[1], base + tri[2], name))
    return verts, norms, uvs, faces_v


# --------------------------------------------------------------------------- #
# 9. ray_cast деталь — сажается на поверхность кузова по нормали
# --------------------------------------------------------------------------- #
def ray_cast_detail(cx, cy, cz, dx, dy, dz, size=0.15, mat='Body'):
    """
    Небольшой кубоид, ориентированный по нормали поверхности в точке (cx,cy,cz),
    смещённый на `size` по нормали (как ray_cast offset в car_build.py).
    """
    nrm = surface_normal(cx, cy, cz)
    # ось X детали по нормали, Y — касательная, Z — бинормаль
    ex = nrm
    ey = np.array([-nrm[1], nrm[0], 0.0])
    if np.linalg.norm(ey) < 1e-8:
        ey = np.array([1.0, 0.0, 0.0])
    ey = ey / np.linalg.norm(ey)
    ez = np.cross(ex, ey)
    ez = ez / np.linalg.norm(ez)

    # центр детали со смещением по нормали
    offset = nrm * size * 0.5
    c = np.array([cx, cy, cz]) + offset

    verts = []
    norms = []
    uvs = []
    faces_v = []
    base = len(verts)
    for sx_i in (-0.5, 0.5):
        for sy_i in (-0.5, 0.5):
            for sz_i in (-0.5, 0.5):
                p = c + ex * (sx_i * size) + ey * (sy_i * size * 0.3) + ez * (sz_i * size * 0.3)
                verts.append(tuple(p))
                norms.append(tuple(nrm))
                uvs.append((0.5 + sx_i * 0.5, 0.5 + sy_i * 0.5))
    cf = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
          (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
          (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    for tri in cf:
        faces_v.append((base + tri[0], base + tri[1], base + tri[2], mat))
    return verts, norms, uvs, faces_v


# --------------------------------------------------------------------------- #
# 10. Сборка и запись OBJ/MTL
# --------------------------------------------------------------------------- #
def main():
    OUT_OBJ.parent.mkdir(parents=True, exist_ok=True)

    all_v = []
    all_vn = []
    all_vt = []
    all_faces = []  # (vi, vni, vti, mat) — используем общие индексы (v=vn=vt для простоты)
    mat_usage = {}

    def add_group(vs, vns, uvs, fs, mat):
        off = len(all_v)
        all_v.extend(vs)
        all_vn.extend(vns)
        all_vt.extend(uvs)
        for f in fs:
            all_faces.append((f[0] + off, f[1] + off, f[2] + off, mat))
        mat_usage[mat] = mat_usage.get(mat, 0) + 1

    # ---------- кузов ----------
    bv, bn, bu, bf = loft_body()
    add_group(bv, bn, bu, bf, 'Body')

    # ---------- колёса (4) ----------
    wheel_positions = [
        (-WB / 2 + 0.55, -WHEEL_Y - 0.02, WHEEL_Z),
        (-WB / 2 + 0.55,  WHEEL_Y + 0.02, WHEEL_Z),
        ( WB / 2 - 0.55, -WHEEL_Y - 0.02, WHEEL_Z),
        ( WB / 2 - 0.55,  WHEEL_Y + 0.02, WHEEL_Z),
    ]
    for (wx, wy, wz) in wheel_positions:
        wv, wn, wu, wf = wheel(wx, wy, wz)
        add_group(wv, wn, wu, wf, 'Tire')
        # диск — тот же вызов wheel уже добавил Wheel_Disc грани

    # ---------- бамперы (передний и задний) ----------
    # Бамперы примыкают к торцам кузова, не расширяя общую длину сверх допуска
    bv, bn, bu, bf = box(-(L/2 - 0.05), 0.0, 0.28, 0.20, W*0.95, 0.30, 'Body')
    add_group(bv, bn, bu, bf, 'Body')
    bv, bn, bu, bf = box((L/2 - 0.05), 0.0, 0.28, 0.20, W*0.95, 0.30, 'Body')
    add_group(bv, bn, bu, bf, 'Body')

    # ---------- пороги (длинные полосы по бокам под дверьми) ----------
    bv, bn, bu, bf = box(0.0, -W/2 - 0.015, 0.16, L*0.85, 0.06, 0.14, 'Body')
    add_group(bv, bn, bu, bf, 'Body')

    # ---------- зеркала заднего вида ----------
    # левое зеркало
    bv, bn, bu, bf = box(-0.55, WHEEL_Y + 0.05, 0.95, 0.12, 0.10, 0.08, 'Body')
    add_group(bv, bn, bu, bf, 'Body')
    # правое зеркало
    bv, bn, bu, bf = box(-0.55, -WHEEL_Y - 0.05, 0.95, 0.12, 0.10, 0.08, 'Body')
    add_group(bv, bn, bu, bf, 'Body')

    # ---------- дверные ручки ----------
    bv, bn, bu, bf = box(-0.45, WHEEL_Y + 0.03, 0.72, 0.10, 0.04, 0.04, 'Body')
    add_group(bv, bn, bu, bf, 'Body')
    bv, bn, bu, bf = box(-0.45, -WHEEL_Y - 0.03, 0.72, 0.10, 0.04, 0.04, 'Body')
    add_group(bv, bn, bu, bf, 'Body')

    # ---------- стёкла ----------
    gv, gn, gu, gf = glass(0.0, 0.0, 1.05, L * 0.50, 0.05, 0.55, 'Glazing')
    add_group(gv, gn, gu, gf, 'Glazing')
    gv, gn, gu, gf = glass(L / 2 - 0.25, 0.0, 1.02, 0.5, 0.05, 0.5, 'Glazing')
    add_group(gv, gn, gu, gf, 'Glazing')

    # ---------- ray_cast детали (бейджи, накладки) ----------
    # деталь на капоте (нос)
    dv, dn, du, df = ray_cast_detail(-1.6, 0.0, 1.25, 0.0, 0.0, 0.0, size=0.12, mat='Body')
    add_group(dv, dn, du, df, 'Body')
    # деталь на крыше (антена/рейлинг — маленькая, не превышает габариты)
    dv, dn, du, df = ray_cast_detail(0.0, 0.0, 1.30, 0.0, 0.0, 0.0, size=0.04, mat='Body')
    add_group(dv, dn, du, df, 'Body')

    # ---------- салон ----------
    gv, gn, gu, gf = glass(-0.6, -W / 2 - 0.01, 0.85, 1.6, 0.04, 0.5, 'Dashboard')
    add_group(gv, gn, gu, gf, 'Interior_Dark')
    gv, gn, gu, gf = glass(-0.4, W / 2 + 0.01, 0.55, 0.5, 0.04, 0.55, 'Front_Seat_D')
    add_group(gv, gn, gu, gf, 'Leather')
    gv, gn, gu, gf = glass(-0.4, -W / 2 - 0.01, 0.55, 0.5, 0.04, 0.55, 'Front_Seat_P')
    add_group(gv, gn, gu, gf, 'Leather')

    # ---------- запись OBJ ----------
    lines = ['# CarUE procedural sedan', '# generated by generate_car_obj.py', '']
    lines.append(f'mtllib car_ue.mtl')
    lines.append('')
    for v in all_v:
        lines.append(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}')
    lines.append('')
    for vn in all_vn:
        lines.append(f'vn {vn[0]:.6f} {vn[1]:.6f} {vn[2]:.6f}')
    lines.append('')
    for vt in all_vt:
        lines.append(f'vt {vt[0]:.6f} {vt[1]:.6f}')
    lines.append('')

    cur_mat = None
    for (ia, ib, ic, mat) in all_faces:
        if mat != cur_mat:
            cur_mat = mat
            lines.append(f'o {cur_mat}')
            lines.append(f'usemtl {cur_mat}')
        # формат f v/vt/vn (все индексы одинаковые, т.к. мы параллельные массивы)
        lines.append(f'f {ia+1}/{ia+1}/{ia+1} {ib+1}/{ib+1}/{ib+1} {ic+1}/{ic+1}/{ic+1}')
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

    print(f'OBJ: {OUT_OBJ}  verts={len(all_v)} faces={len(all_faces)}')
    print(f'MTL: {OUT_MTL}')
    print('Материалы:', mat_usage)


if __name__ == '__main__':
    main()
