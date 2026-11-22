# -*- coding: utf-8 -*-
"""
====================================================================
  PROCEDURAL CAR  -  генератор реалистичного автомобиля для Blender
  Версия: 4.2 LTS (также работает в 4.0 - 4.5)
  Запуск из Blender:
      blender -b -P car_build.py                (только собрать + сохранить)
      blender -b -P car_build.py -- --render    (собрать + отрендерить кадры)
  Или внутри Blender: Scripting -> Text -> Run Script
====================================================================
  Что строится:
    * кузов (единый лофт-меш с колёсными арками, остеклением, порогами)
    * остекление с рамками, стойками и молдингами (по материалам на гранях)
    * колёса: шина с протектором, диск с 10 спицами, тормозной диск, суппорт
    * интерьер: панель, руль, сиденья, консоль, карты дверей, приборка
    * светотехника, решётка, зеркала, ручки, глушители, диффузор, спойлер
    * материалы на процедурных нодах (металлик с хлопьями, стекло, кожа, хром)
    * анимация: вращение колёс, поворот передних колёс, руль, раскачка кузова
====================================================================
  Оси:  +X = вперёд,  +Y = влево,  +Z = вверх.  Единицы - метры.
"""

import bpy
import bmesh
import math
import bisect
import os
import sys
from math import hypot, radians, sin, cos, sqrt, pi
from mathutils import Vector, Matrix

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
DO_RENDER = "--render" in ARGS
OUT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else "/home/user"
if not os.path.isdir(OUT_DIR):
    OUT_DIR = "/home/user"

# ====================================================================
#  0. МИНИ-БИБЛИОТЕКА ХЕЛПЕРОВ
# ====================================================================

def clamp(x, a, b):
    return a if x < a else (b if x > b else x)


def smoothstep(e0, e1, x):
    t = clamp((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


class Curve1D:
    """Гладкая кривая по опорным точкам (Hermite / Catmull-Rom).
    Позволяет задать профиль кузова 10-15 числами вместо сотен вершин."""

    def __init__(self, keys):
        keys = sorted(keys, key=lambda k: k[0])
        self.xs = [k[0] for k in keys]
        self.vs = [k[1] for k in keys]
        self.n = len(keys)

    def _tan(self, j):
        xs, vs, n = self.xs, self.vs, self.n
        if j == 0:
            return (vs[1] - vs[0]) / (xs[1] - xs[0])
        if j == n - 1:
            return (vs[-1] - vs[-2]) / (xs[-1] - xs[-2])
        return (vs[j + 1] - vs[j - 1]) / (xs[j + 1] - xs[j - 1])

    def __call__(self, x):
        xs, vs = self.xs, self.vs
        if x <= xs[0]:
            return vs[0]
        if x >= xs[-1]:
            return vs[-1]
        i = bisect.bisect_right(xs, x) - 1
        h = xs[i + 1] - xs[i]
        t = (x - xs[i]) / h
        m0 = self._tan(i) * h
        m1 = self._tan(i + 1) * h
        h00 = 2 * t ** 3 - 3 * t ** 2 + 1
        h10 = t ** 3 - 2 * t ** 2 + t
        h01 = -2 * t ** 3 + 3 * t ** 2
        h11 = t ** 3 - t ** 2
        return h00 * vs[i] + h10 * m0 + h01 * vs[i + 1] + h11 * m1


def resample(pts, n):
    """Пересэмплировать ломаную (список 2D-точек) в n точек сплайном Catmull-Rom
    с параметризацией по длине хорды. Даёт гладкий профиль без изломов."""
    P = [Vector((p[0], p[1])) for p in pts]
    m = len(P)
    if m < 2:
        return [(P[0].x, P[0].y)] * n
    d = [0.0]
    for i in range(1, m):
        d.append(d[-1] + (P[i] - P[i - 1]).length)
    total = d[-1]
    if total < 1e-9:
        return [(P[0].x, P[0].y)] * n
    out = []
    for k in range(n):
        u = total * k / (n - 1)
        i = clamp(bisect.bisect_right(d, u) - 1, 0, m - 2)
        seg = d[i + 1] - d[i]
        t = 0.0 if seg < 1e-12 else (u - d[i]) / seg
        p0 = P[i - 1] if i > 0 else P[0] - (P[1] - P[0])
        p1, p2 = P[i], P[i + 1]
        p3 = P[i + 2] if i + 2 < m else P[-1] + (P[-1] - P[-2])
        pos = 0.5 * ((2 * p1) + (-p0 + p2) * t +
                     (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                     (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3)
        out.append((pos.x, pos.y))
    return out


def new_mesh_object(name, verts, faces, coll):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate(verbose=False)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def recalc_normals(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()


def smooth(ob, angle=45.0, subsurf=0):
    """Гладкое затенение по углу + (опционально) Subdivision Surface."""
    if subsurf:
        m = ob.modifiers.new("Subdivision", 'SUBSURF')
        m.levels = subsurf
        m.render_levels = subsurf + 1
    bpy.context.view_layer.objects.active = ob
    for o in bpy.context.selected_objects:
        o.select_set(False)
    ob.select_set(True)
    try:
        bpy.ops.object.shade_auto_smooth(angle=radians(angle))
    except Exception:
        bpy.ops.object.shade_smooth()
    ob.select_set(False)


def bevel(ob, width=0.008, segments=3):
    m = ob.modifiers.new("Bevel", 'BEVEL')
    m.width = width
    m.segments = segments
    m.limit_method = 'ANGLE'
    m.angle_limit = radians(35)
    m.harden_normals = True
    return m


def cube(name, size, loc=(0, 0, 0), rot=(0, 0, 0), coll=None, bv=0.008, seg=3):
    """Скруглённый параллелепипед - базовый кирпичик интерьера."""
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        v.co.x *= size[0]
        v.co.y *= size[1]
        v.co.z *= size[2]
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = rot
    if bv:
        bevel(ob, bv, seg)
    return ob


def cyl(name, r, depth, loc=(0, 0, 0), rot=(0, 0, 0), verts=32, coll=None):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=verts,
                          radius1=r, radius2=r, depth=depth)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    ob.location = loc
    ob.rotation_euler = rot
    return ob


def revolve(profile, segs, axis_offset=0.0):
    """Тело вращения вокруг оси Y (профиль задаётся точками (y, radius))."""
    verts, faces = [], []
    M = len(profile)
    for s in range(segs):
        a = 2 * pi * s / segs
        ca, sa = cos(a), sin(a)
        for (py, pr) in profile:
            verts.append((pr * ca, py + axis_offset, pr * sa))
    for s in range(segs):
        s2 = (s + 1) % segs
        for i in range(M - 1):
            a = s * M + i
            b = s2 * M + i
            faces.append((a, b, b + 1, a + 1))
    return verts, faces


def cyl_between(name, p0, p1, r, coll, verts=24):
    """Цилиндр между двумя точками пространства."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    ob = cyl(name, r, d.length, loc=tuple((p0 + p1) / 2), verts=verts, coll=coll)
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = d.to_track_quat('Z', 'Y')
    return ob


def set_material(ob, mat):
    ob.data.materials.clear()
    ob.data.materials.append(mat)


def put(ob, coll):
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)


# ====================================================================
#  1. ГАБАРИТЫ И ПАРАМЕТРИЧЕСКИЕ КРИВЫЕ КУЗОВА
#     (обмеры сняты с реального седана: колёсная база 3.06 м,
#      колея 1.630 м, ширина 1.87 м, высота 1.456 м)
# ====================================================================

L_FRONT = 2.50          # нос
L_REAR = -2.50          # корма
AXLE_F = 1.60           # передняя ось
AXLE_R = -1.46          # задняя ось
WHEEL_R = 0.353         # радиус шины (235/45 R18)
WHEEL_W = 0.235         # ширина шины
WHEEL_Y = 0.813         # ось колеса (колея 1.626 м)
ARCH_R = 0.445          # радиус колёсной арки
ARCH_DEPTH = 0.130      # глубина впадины арки (ниша колеса)
WHEEL_Z = 0.353         # центр колеса по высоте (= радиус шины)
ARCH_BASE = 0.300       # полувысота проёма под осью (проём не круг, а «сплюснутый»)
Z_ARCH = 0.765          # верх арки (в центре колеса)

# полуширина кузова (максимум на уровне пояса)
W = Curve1D([(-2.50, 0.640), (-2.42, 0.740), (-2.34, 0.830), (-2.26, 0.872),
             (-2.16, 0.898), (-2.00, 0.914), (-1.80, 0.925), (-1.45, 0.933),
             (-1.10, 0.936), (-0.60, 0.936), (0.00, 0.932), (0.40, 0.934),
             (0.80, 0.934), (1.20, 0.938), (1.55, 0.936), (1.80, 0.931),
             (1.98, 0.924), (2.10, 0.910), (2.20, 0.882), (2.30, 0.822),
             (2.40, 0.720), (2.50, 0.620)])

# низ кузова (уровень порога/нижней кромки бампера)
ZB = Curve1D([(-2.50, 0.300), (-2.42, 0.245), (-2.30, 0.250), (-2.10, 0.258),
              (-1.90, 0.265), (-1.80, 0.262), (-1.70, 0.248), (-1.40, 0.232),
              (-1.00, 0.226), (-0.60, 0.220), (0.00, 0.214), (0.40, 0.214),
              (0.90, 0.216), (1.30, 0.218), (1.80, 0.222), (2.10, 0.235),
              (2.30, 0.255), (2.42, 0.262), (2.50, 0.320)])

# «пояс» — линия подоконника (низ бокового и лобового стекла).
# Сзади уходит на уровень крышки багажника, поэтому заднее стекло получает
# реальную площадь (а не тонкую полоску у крыши).
BELT = Curve1D([(-2.50, 0.830), (-2.42, 0.910), (-2.30, 0.965), (-2.16, 0.990),
                (-2.00, 1.000), (-1.86, 1.002), (-1.72, 0.998), (-1.58, 1.000),
                (-1.46, 1.020), (-1.38, 1.055), (-1.30, 1.090), (-1.18, 1.075),
                (-1.02, 1.054), (-0.82, 1.050), (-0.60, 1.046), (-0.42, 1.043),
                (-0.26, 1.038), (-0.06, 1.034), (0.10, 1.031), (0.26, 1.027),
                (0.42, 1.025), (0.58, 1.019), (0.72, 1.008), (0.90, 0.995),
                (1.10, 0.985), (1.40, 0.975), (1.80, 0.960), (2.10, 0.940),
                (2.30, 0.905), (2.42, 0.860), (2.50, 0.800)])

# крыша (и «коронка» капота / багажника там, где крыши нет)
# линия снята по контуру реальной машины (ортогональный вид сбоку)
ZR = Curve1D([(-2.50, 0.700), (-2.44, 0.800), (-2.38, 0.986), (-2.32, 1.041),
              (-2.26, 1.044), (-2.20, 1.048), (-2.14, 1.051), (-2.08, 1.058),
              (-2.02, 1.069), (-1.96, 1.090), (-1.90, 1.111), (-1.84, 1.139),
              (-1.78, 1.160), (-1.72, 1.184), (-1.66, 1.209), (-1.60, 1.230),
              (-1.54, 1.251), (-1.48, 1.272), (-1.42, 1.293), (-1.36, 1.314),
              (-1.30, 1.335), (-1.24, 1.356), (-1.18, 1.373), (-1.12, 1.384),
              (-1.06, 1.391), (-1.00, 1.398), (-0.94, 1.408), (-0.88, 1.415),
              (-0.82, 1.422), (-0.76, 1.429), (-0.70, 1.433), (-0.64, 1.440),
              (-0.58, 1.443), (-0.52, 1.447), (-0.40, 1.450), (-0.10, 1.450),
              (0.02, 1.447), (0.08, 1.440), (0.14, 1.436), (0.20, 1.426),
              (0.26, 1.415), (0.32, 1.398), (0.38, 1.384), (0.44, 1.363),
              (0.50, 1.338), (0.56, 1.314), (0.62, 1.293), (0.68, 1.265),
              (0.74, 1.237), (0.80, 1.205), (0.86, 1.170), (0.92, 1.139),
              (0.98, 1.104), (1.04, 1.072), (1.10, 1.051), (1.16, 1.044),
              (1.22, 1.034), (1.28, 1.030), (1.34, 1.023), (1.40, 1.020),
              (1.46, 1.013), (1.52, 1.006), (1.58, 1.002), (1.64, 0.995),
              (1.70, 0.992), (1.76, 0.981), (1.82, 0.974), (1.88, 0.964),
              (1.94, 0.953), (2.00, 0.946), (2.06, 0.932), (2.12, 0.918),
              (2.18, 0.904), (2.24, 0.887), (2.30, 0.866), (2.36, 0.848),
              (2.42, 0.829), (2.50, 0.780)])

# полуширина крыши (скат: точка перелома «крыша -> окно -> плечо»)
WR = Curve1D([(-2.50, 0.380), (-2.42, 0.470), (-2.30, 0.540), (-2.10, 0.580),
              (-1.90, 0.620), (-1.70, 0.660), (-1.50, 0.690), (-1.30, 0.715),
              (-1.10, 0.735), (-0.80, 0.750), (-0.40, 0.758), (0.00, 0.760),
              (0.30, 0.752), (0.50, 0.740), (0.70, 0.720), (0.90, 0.680),
              (1.10, 0.640), (1.30, 0.600), (1.60, 0.560), (1.90, 0.530),
              (2.10, 0.500), (2.30, 0.470), (2.42, 0.430), (2.50, 0.380)])

# высота наибольшей ширины (подоконная линия)
ZW = Curve1D([(-2.50, 0.55), (-2.20, 0.58), (-1.60, 0.60), (-1.00, 0.62),
              (-0.40, 0.64), (0.00, 0.66), (0.40, 0.68), (1.00, 0.70),
              (1.60, 0.72), (2.10, 0.74), (2.50, 0.75)])

# наклон бокового стекла внутрь («tumblehome»): смещение окна на середине высоты
TUMBLE = Curve1D([(-2.50, 0.060), (-1.90, 0.075), (-1.40, 0.090), (-0.90, 0.100),
                  (-0.30, 0.110), (0.30, 0.110), (0.70, 0.100), (1.10, 0.085),
                  (1.60, 0.070), (2.10, 0.060), (2.50, 0.050)])


def arch_factor(x):
    """0..1 - насколько точка x попала в колёсную арку."""
    f = 0.0
    for xc in (AXLE_F, AXLE_R):
        dx = abs(x - xc) / ARCH_R
        if dx < 1.0:
            f = max(f, sqrt(max(0.0, 1.0 - dx * dx)))
    return f


Z_ARCH_SPRING = 0.362   # высота, на которой «стоит» арка (уровень оси)


def arch_shape(t):
    """Проём арки по обмерам реального крыла: полуэллипс 0.445 по длине и
    ~0.40 по высоте, вершина 0.765 (колесо R=0.353 стоит с зазором ~0.06)."""
    t = clamp(t, 0.0, 1.0)
    return (1.0 - t * t) ** 0.60


def arch_opening(x):
    """Кромка арки в сечении x (проём - окружность вокруг оси колеса, а не
    полуэллипс: так кромка получается ровной и без «ступеньки» у порога)."""
    best = None
    for xc in (AXLE_F, AXLE_R):
        dx = abs(x - xc)
        if dx < ARCH_R:
            best = max(best or 0.0, WHEEL_Z + sqrt(ARCH_R ** 2 - dx ** 2))
    return best


def z_opening(x):
    """Верхняя кромка колёсной арки в сечении x (не ниже порога)."""
    op = arch_opening(x)
    zb = ZB(x)
    return zb if op is None else max(zb, op)


# ---- геометрия сечения -------------------------------------------------
TIP_F = (2.370, 2.500)   # зона закругления носа
TIP_R = (-2.400, -2.500)  # зона закругления кормы
V_BODY = 18        # точек на «тело» половины сечения
V_GH = 9           # точек на «остекление/крышу»


def half_profile(x):
    """Половина поперечного сечения кузова.

    Возвращает (points, i_belt, i_win, i_roof):
      i_belt - подоконная линия (низ стекла)
      i_win  - верх стекла / начало ската крыши
      i_roof - точка перелома на крыше
    """
    w, zb, belt, zr, wr = W(x), ZB(x), BELT(x), ZR(x), WR(x)
    af = arch_factor(x)
    w *= (1.0 + 0.010 * af)
    z_open = z_opening(x)
    zw = clamp(ZW(x), zb + 0.10, belt - 0.05)
    th = TUMBLE(x) * smoothstep(zb + 0.02, belt, belt)   # наклон борта

    body = [
        (0.000 * w, zb),
        (0.330 * w, zb),
        (0.560 * w, zb + 0.002),
        (0.700 * w, zb + 0.010),
        (0.780 * w, zb + 0.026),
        (0.845 * w, zb + 0.058),
        (0.895 * w, zb + 0.110),
        (0.930 * w, zb + 0.185),                  # низ борта: плавная волна
        (0.960 * w, zb + 0.46 * (zw - zb)),
        (0.984 * w, zw - 0.30 * (zw - zb)),
        (1.000 * w, zw),                          # плечо (макс. ширина)
        (0.996 * w, zw + 0.26 * (belt - zw)),
        (0.982 * w, zw + 0.58 * (belt - zw)),
        (0.960 * w, belt - 0.014),
        (0.934 * w, belt),                        # подоконная линия
    ]
    # ---- колёсная арка ----
    # Вместо «прорезания» и поднятия борта (это давало и «полку», и рябь на
    # бликах) делаем плавную ВПАДИНУ: борт уходит внутрь по гладкому полю
    # глубины вокруг оси колеса. Поле непрерывно по x и z, поэтому блики
    # текут ровно, а колесо визуально сидит в нише.
    def arch_recess(y, z):
        if y < 0.45 * w:
            return y
        d = 0.0
        for xc in (AXLE_F, AXLE_R):
            g = hypot((x - xc) / ARCH_R, (z - WHEEL_Z) / ARCH_R)
            if g < 1.55:
                # роликовая кромка: снаружи 0, у самой кромки - резкий, но
                # гладкий заворот внутрь на глубину ниши
                d = max(d, ARCH_DEPTH * (1.0 - smoothstep(0.995, 1.115, g)) -
                        0.010 * (1.0 - smoothstep(1.115, 1.50, g)))
        if d <= 0.0:
            return y
        # у порога впадина сходит на нет (порог остаётся ровным)
        d *= smoothstep(zb + 0.010, zb + 0.115, z)
        # к центру днища впадина тоже сходит на нет
        d *= smoothstep(0.45 * w, 0.86 * w, y)
        return y - d

    body = [(arch_recess(y, z), z) for (y, z) in body]

    # --- остекление: от подоконной линии к крыше с наклоном внутрь ---
    dz = max(zr - belt, 0.0)
    y_g = min(wr, 0.930 * w)
    gh = [
        (y_g,              belt),
        (y_g - 0.35 * th,  belt + 0.30 * dz),
        (wr * 0.930,       belt + 0.62 * dz),
        (wr * 0.845,       belt + 0.845 * dz),    # верх стекла / кромка крыши
        (wr * 0.855,       belt + 0.972 * dz),
        (wr * 0.46,        belt + 0.997 * dz),
        (0.0,              zr),
    ]

    bres = resample(body, V_BODY)
    gres = resample(gh, V_GH)
    pts = bres + gres[1:]

    # --- закругление носа и кормы: сечение «сходится», образуя обтекаемый
    #     передок вместо плоской торцевой стенки ---
    zmid = 0.5 * (zb + belt)
    if x > TIP_F[0]:
        k = smoothstep(TIP_F[0], TIP_F[1], x) ** 1.6
        pts = [(y * (1 - 0.62 * k),
                z - (z - zmid) * 0.26 * k if z > zmid else z + (zmid - z) * 0.12 * k)
               for (y, z) in pts]
    elif x < TIP_R[0]:
        k = smoothstep(TIP_R[0], TIP_R[1], x) ** 1.15
        pts = [(y * (1 - 0.55 * k),
                z - (z - zmid) * 0.22 * k if z > zmid else z + (zmid - z) * 0.10 * k)
               for (y, z) in pts]

    off = V_BODY - 1
    return pts, off + 1, off + 3, off + 4


def build_body(coll, mats):
    """Лофт по сечениям: единая гладкая оболочка кузова."""
    xs = []
    base = [L_REAR + (L_FRONT - L_REAR) * i / 165.0 for i in range(166)]
    extra = [AXLE_F - ARCH_R, AXLE_F, AXLE_F + ARCH_R,
             AXLE_R - ARCH_R, AXLE_R, AXLE_R + ARCH_R,
             1.22, 1.10, 0.98, 0.90, 0.70, 0.50, 0.30, 0.10,
             -0.30, -0.52, -0.72, -0.95, -1.05, -1.18, -1.30]
    for x in sorted(base + extra):
        if not xs or x - xs[-1] > 0.010:
            xs.append(x)
    if xs[-1] < L_FRONT - 1e-4:
        xs.append(L_FRONT)

    rings, bands = [], []
    for x in xs:
        half, ib, ic, id_ = half_profile(x)
        ring = half + [(-y, z) for (y, z) in reversed(half[1:-1])]
        rings.append(ring)
        bands.append((len(half) - 1, ib, ic, id_))

    NH = len(rings[0])
    print("[body] сечений: %d, точек в сечении: %d" % (len(xs), NH))

    verts = []
    for x, ring in zip(xs, rings):
        for (y, z) in ring:
            verts.append((x, y, z))

    faces, fmat = [], []
    M_PAINT, M_GLASS, M_TRIM, M_LINER = 0, 1, 2, 3

    def half_index(j):
        return j if j <= NH // 2 else NH - j

    # --- зоны остекления (по обмерам референса) ---
    X_WS = (0.335, 1.180)         # лобовое: верх у x=0.335, низ у 1.18
    X_RG = (-1.960, -1.195)       # заднее: верх у -1.195, низ у -1.96
    X_SIDE = (-1.330, 0.720)      # боковые стёкла
    X_BPILLAR = (-0.272, -0.185)  # средняя стойка
    X_QUARTER = (-1.330, -1.075)  # форточка

    for i in range(len(xs) - 1):
        xm = 0.5 * (xs[i] + xs[i + 1])
        h_len, ib, ic, id_ = bands[i]
        for j in range(NH):
            j2 = (j + 1) % NH
            a = i * NH + j
            b = i * NH + j2
            c = (i + 1) * NH + j2
            d = (i + 1) * NH + j
            faces.append((a, b, c, d))
            h1, h2 = half_index(j), half_index(j2)
            lo, hi = min(h1, h2), max(h1, h2)
            mat = M_PAINT
            zm_f = 0.25 * (rings[i][j][1] + rings[i][j2][1] +
                           rings[i + 1][j][1] + rings[i + 1][j2][1])
            ym_f = 0.25 * (abs(rings[i][j][0]) + abs(rings[i][j2][0]) +
                           abs(rings[i + 1][j][0]) + abs(rings[i + 1][j2][0]))
            # лобовое и заднее: грани выше подоконной линии, по всей ширине
            if X_WS[0] < xm < X_WS[1] or X_RG[0] < xm < X_RG[1]:
                if lo >= ib and hi <= ic + 2 and ym_f < 0.905 * W(xm):
                    mat = M_GLASS
            # боковые стёкла: строго между поясом и верхом стекла
            elif X_SIDE[0] < xm < X_SIDE[1] and lo >= ib and hi <= ic:
                mat = M_TRIM if (X_BPILLAR[0] < xm < X_BPILLAR[1]) else M_GLASS
            elif X_QUARTER[0] < xm < X_QUARTER[1] and hi <= ic + 2 and lo >= ib:
                mat = M_GLASS
            # внутри колеи арки - тёмная ниша (подкрылок)
            if mat == M_PAINT and ym_f > 0.55 * W(xm):
                for xc in (AXLE_F, AXLE_R):
                    if hypot((xm - xc) / ARCH_R, (zm_f - WHEEL_Z) / ARCH_R) < 1.02:
                        mat = M_LINER
                        break
            fmat.append(mat)

    cap_a = [j for j in range(NH)][::-1]
    cap_b = [j for j in range(NH)]
    faces.append(tuple(j for j in cap_a))
    fmat.append(M_PAINT)
    faces.append(tuple((len(xs) - 1) * NH + j for j in cap_b))
    fmat.append(M_PAINT)

    ob = new_mesh_object("Body", verts, faces, coll)
    for m in (mats['paint'], mats['glass'], mats['trim'], mats['liner']):
        ob.data.materials.append(m)
    for p, mi in zip(ob.data.polygons, fmat):
        p.material_index = mi
    recalc_normals(ob)
    bpy.context.view_layer.objects.active = ob
    for o in bpy.context.selected_objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.ops.object.shade_smooth()
    ob.select_set(False)
    m = ob.modifiers.new("Subdivision", 'SUBSURF')
    m.levels = 1
    m.render_levels = 2
    return ob, xs


# ====================================================================
#  2. МАТЕРИАЛЫ
# ====================================================================

def nt(mat):
    mat.use_nodes = True
    return mat.node_tree


def pbr(name, base, metallic=0.0, rough=0.5, coat=0.0, coat_rough=0.05,
        emission=None, emit_str=0.0, alpha=1.0, transmission=0.0):
    mat = bpy.data.materials.new(name)
    t = nt(mat)
    b = t.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*base, 1.0)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = rough
    if "Coat Weight" in b.inputs:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = coat_rough
    if transmission and "Transmission Weight" in b.inputs:
        b.inputs["Transmission Weight"].default_value = transmission
    if alpha < 1.0:
        b.inputs["Alpha"].default_value = alpha
        mat.blend_method = 'BLEND'
    if emission is not None:
        b.inputs["Emission Color"].default_value = (*emission, 1.0)
        b.inputs["Emission Strength"].default_value = emit_str
    return mat


def mat_paint():
    """Автомобильная эмаль металлик: база + хлопья + слой лака.
    (цвет меняется одной строкой - см. Base Color)"""
    mat = bpy.data.materials.new("CarPaint_MetallicBlue")
    t = nt(mat)
    nodes, links = t.nodes, t.links
    bsdf = nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.055, 0.115, 0.280, 1.0)
    bsdf.inputs["Metallic"].default_value = 0.62
    bsdf.inputs["Roughness"].default_value = 0.26
    bsdf.inputs["Coat Weight"].default_value = 1.0
    bsdf.inputs["Coat Roughness"].default_value = 0.03
    # хлопья металлика -> микро-рельеф
    tex = nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 2400.0
    tex.inputs["Detail"].default_value = 2.0
    bump = nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.02
    bump.inputs["Distance"].default_value = 0.0015
    links.new(tex.outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    # небольшая неоднородность лака (без изменения цвета)
    n2 = nodes.new("ShaderNodeTexNoise")
    n2.inputs["Scale"].default_value = 18.0
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.35
    ramp.color_ramp.elements[0].color = (0.020, 0.020, 0.020, 1)
    ramp.color_ramp.elements[1].position = 0.65
    ramp.color_ramp.elements[1].color = (0.060, 0.060, 0.060, 1)
    links.new(n2.outputs["Fac"], ramp.inputs["Fac"])
    links.new(ramp.outputs["Color"], bsdf.inputs["Coat Roughness"])
    return mat


def mat_window():
    """Стекло: прозрачное в лоб, отражающее под скользящими углами (Fresnel).
    Сделано на Transparent + Glossy - так Cycles не считает каустику внутри
    салона, и внутренние рендеры не получаются чёрными."""
    mat = bpy.data.materials.new("Glass_Window")
    t = nt(mat)
    nodes, links = t.nodes, t.links
    nodes.clear()
    out = nodes.new('ShaderNodeOutputMaterial')
    transp = nodes.new('ShaderNodeBsdfTransparent')
    transp.inputs['Color'].default_value = (0.58, 0.64, 0.65, 1.0)
    gloss = nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Roughness'].default_value = 0.035
    fres = nodes.new('ShaderNodeLayerWeight')
    fres.inputs['Blend'].default_value = 0.12
    mix = nodes.new('ShaderNodeMixShader')
    mm = nodes.new('ShaderNodeMath')
    mm.operation = 'MULTIPLY_ADD'
    mm.inputs[1].default_value = 0.42
    mm.inputs[2].default_value = 0.05
    links.new(fres.outputs['Fresnel'], mm.inputs[0])
    links.new(mm.outputs['Value'], mix.inputs['Fac'])
    links.new(transp.outputs['BSDF'], mix.inputs[1])
    links.new(gloss.outputs['BSDF'], mix.inputs[2])
    links.new(mix.outputs['Shader'], out.inputs['Surface'])
    mat.blend_method = 'BLEND'
    return mat


def mat_leather(name, color, scale=260.0):
    mat = pbr(name, color, 0.0, 0.62)
    t = nt(mat)
    b = t.nodes["Principled BSDF"]
    tex = t.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = scale
    tex.inputs["Detail"].default_value = 4.0
    bump = t.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.14
    bump.inputs["Distance"].default_value = 0.0008
    t.links.new(tex.outputs["Fac"], bump.inputs["Height"])
    t.links.new(bump.outputs["Normal"], b.inputs["Normal"])
    return mat


def build_materials():
    mats = {}
    mats['paint'] = mat_paint()
    mats['glass'] = mat_window()
    mats['trim'] = pbr("Trim_Black", (0.014, 0.014, 0.016), 0.0, 0.35)
    mats['chrome'] = pbr("Chrome", (0.92, 0.93, 0.95), 1.0, 0.035)
    mats['alu'] = pbr("Aluminium", (0.72, 0.73, 0.75), 1.0, 0.22)
    mats['rubber'] = mat_leather("Rubber_Tire", (0.017, 0.017, 0.019), 900.0)
    mats['steel'] = pbr("Steel", (0.30, 0.30, 0.32), 1.0, 0.42)
    mats['disc'] = pbr("Brake_Disc", (0.26, 0.26, 0.27), 1.0, 0.55)
    mats['caliper'] = pbr("Brake_Caliper", (0.32, 0.03, 0.03), 0.30, 0.35)
    mats['liner'] = pbr("WheelWell_Liner", (0.020, 0.020, 0.021), 0.0, 0.78)
    mats['lens'] = pbr("Lens_Clear", (0.12, 0.14, 0.16), 0.0, 0.06,
                       transmission=0.55)
    mats['headlight'] = pbr("Light_Headlight", (0.95, 0.95, 0.92), 0.0, 0.05,
                            emission=(1.0, 0.96, 0.86), emit_str=0.9)
    mats['drl'] = pbr("Light_DRL", (0.90, 0.92, 0.95), 0.0, 0.15,
                      emission=(0.85, 0.93, 1.0), emit_str=2.2)
    mats['tail'] = pbr("Light_Tail", (0.45, 0.02, 0.02), 0.0, 0.15,
                       emission=(1.0, 0.05, 0.02), emit_str=1.1)
    mats['badge'] = pbr("Badge", (0.90, 0.91, 0.93), 1.0, 0.08)
    mats['screen'] = pbr("Screen", (0.010, 0.012, 0.016), 0.0, 0.12,
                         emission=(0.15, 0.28, 0.45), emit_str=1.2)
    mats['carbon'] = mat_carbon()
    mats['leather'] = mat_leather("Leather_Black", (0.042, 0.038, 0.037))
    mats['leather_acc'] = mat_leather("Leather_Accent", (0.055, 0.030, 0.022),
                                      scale=320.0)
    mats['plastic'] = pbr("Interior_Plastic", (0.020, 0.020, 0.022), 0.0, 0.55)
    mats['soft'] = mat_leather("Dash_Soft", (0.028, 0.026, 0.026), 420.0)
    mats['carpet'] = mat_leather("Carpet", (0.030, 0.030, 0.032), 1200.0)
    mats['plate'] = pbr("Plate", (0.86, 0.86, 0.88), 0.0, 0.32)
    mats['mirror'] = pbr("Mirror_Glass", (0.80, 0.82, 0.85), 1.0, 0.02)
    mats['gunmetal'] = pbr("Rim_Gunmetal", (0.12, 0.125, 0.135), 1.0, 0.30)
    mats['gloss_black'] = pbr("Piano_Black", (0.008, 0.008, 0.009), 0.0, 0.08)
    mats['headliner'] = mat_leather("Headliner_Fabric", (0.085, 0.083, 0.080), 900.0)
    mats['badge_r'] = pbr("Red_Badge", (0.40, 0.02, 0.02), 0.0, 0.25)
    return mats


def mat_carbon():
    mat = pbr("Carbon", (0.020, 0.022, 0.026), 0.15, 0.30)
    t = nt(mat)
    nodes, links = t.nodes, t.links
    b = nodes["Principled BSDF"]
    uv = nodes.new("ShaderNodeTexCoord")
    mp = nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (120.0, 120.0, 120.0)
    chk = nodes.new("ShaderNodeTexChecker")
    chk.inputs["Scale"].default_value = 1.0
    chk.inputs["Color1"].default_value = (0.030, 0.032, 0.036, 1)
    chk.inputs["Color2"].default_value = (0.012, 0.013, 0.015, 1)
    links.new(uv.outputs["Object"], mp.inputs["Vector"])
    links.new(mp.outputs["Vector"], chk.inputs["Vector"])
    links.new(chk.outputs["Color"], b.inputs["Base Color"])
    return mat


# ====================================================================
#  3. КОЛЁСА
# ====================================================================

class Geo:
    """Аккумулятор геометрии: несколько закрытых оболочек в одном меше."""

    def __init__(self):
        self.v, self.f, self.m = [], [], []

    def add(self, verts, faces, mat):
        o = len(self.v)
        self.v.extend(verts)
        for fc in faces:
            self.f.append(tuple(i + o for i in fc))
        self.m.extend([mat] * len(faces))

    def mirror_y(self, s):
        self.v = [(x, y * s, z) for (x, y, z) in self.v]

    def build(self, name, coll, mats):
        ob = new_mesh_object(name, self.v, self.f, coll)
        for m in mats:
            ob.data.materials.append(m)
        for p, mi in zip(ob.data.polygons, self.m):
            p.material_index = mi
        recalc_normals(ob)
        smooth(ob, 38.0)
        return ob


def rot_y(p, a):
    c, s = cos(a), sin(a)
    return (p[0] * c + p[2] * s, p[1], -p[0] * s + p[2] * c)


def box_geo(cx, cy, cz, sx, sy, sz, a=0.0):
    pts = []
    for dx in (-1, 1):
        for dy in (-1, 1):
            for dz in (-1, 1):
                pts.append((dx * sx / 2, dy * sy / 2, dz * sz / 2))
    pts = [rot_y((p[0], p[1], p[2]), a) for p in pts]
    v = [(cx + p[0], cy + p[1], cz + p[2]) for p in pts]

    def idx(dx, dy, dz):
        return (0 if dx < 0 else 4) + (0 if dy < 0 else 2) + (0 if dz < 0 else 1)
    f = [
        tuple(idx(-1, -1, z) for z in (-1, 1))[::-1],
        tuple(idx(1, -1, z) for z in (-1, 1)),
        (idx(-1, -1, -1), idx(1, -1, -1), idx(1, 1, -1), idx(-1, 1, -1)),
        (idx(-1, -1, 1), idx(-1, 1, 1), idx(1, 1, 1), idx(1, -1, 1)),
        (idx(-1, -1, -1), idx(-1, -1, 1), idx(1, -1, 1), idx(1, -1, -1)),
        (idx(-1, 1, -1), idx(1, 1, -1), idx(1, 1, 1), idx(-1, 1, 1)),
    ]
    return v, f


def spoke_geo(r0, r1, w0, w1, y0, y1, a):
    """Спица диска: сужается к ступице, «тарелка» уходит внутрь."""
    v = []
    for (r, w, y) in ((r0, w0, y0), (r1, w1, y0), (r1, w1, y1), (r0, w0, y1)):
        p1 = rot_y((r, 0, -w / 2), a)
        p2 = rot_y((r, 0, w / 2), a)
        v.append((p1[0], y, p1[2]))
        v.append((p2[0], y, p2[2]))
    f = [(0, 1, 3, 2), (4, 6, 7, 5),
         (0, 4, 5, 1), (2, 3, 7, 6), (1, 5, 7, 3), (0, 2, 6, 4)]
    return v, f


def tire_geo(segs=120):
    """Шина 235/45 R18: профиль с бортиками, плечевыми закруглениями
    и канавками протектора."""
    prof = [(-0.100, 0.243, 0), (-0.112, 0.262, 0), (-0.118, 0.290, 0),
            (-0.117, 0.320, 0), (-0.110, 0.340, 0), (-0.096, 0.3495, 0),
            (-0.078, 0.3525, 1), (-0.050, 0.3532, 1), (-0.020, 0.3534, 1),
            (0.010, 0.3534, 1), (0.040, 0.3532, 1), (0.070, 0.3525, 1),
            (0.088, 0.3495, 0), (0.103, 0.340, 0), (0.112, 0.320, 0),
            (0.114, 0.290, 0), (0.110, 0.262, 0), (0.100, 0.243, 0)]
    M = len(prof)
    verts, faces = [], []
    for s in range(segs):
        a = 2 * pi * s / segs
        # канавки протектора
        d = 0.0065 if (s % 10) in (0, 1, 8, 9) else 0.0
        for (py, pr, tread) in prof:
            r = pr - (d if tread else 0.0)
            verts.append((r * cos(a), py, r * sin(a)))
    for s in range(segs):
        s2 = (s + 1) % segs
        for i in range(M):
            i2 = (i + 1) % M
            faces.append((s * M + i, s2 * M + i, s2 * M + i2, s * M + i2))
    return verts, faces


def build_wheel(name, sign, coll, mats, rim_mat):
    """Колесо в сборе. Центр ступицы в нуле, ось вращения - Y."""
    g = Geo()
    MT, MR, MC, MD, MK, MS = 0, 1, 2, 3, 4, 5
    g.add(*tire_geo(), mat=MT)
    # обод (бочка 18" -> радиус 0.229)
    v, f = revolve([(-0.092, 0.232), (0.078, 0.232), (0.086, 0.243),
                    (0.092, 0.245), (0.092, 0.231), (0.080, 0.224),
                    (-0.092, 0.224), (-0.092, 0.232)], 80)
    g.add(v, f, MR)
    # полированная закраина
    v, f = revolve([(0.086, 0.237), (0.095, 0.244), (0.097, 0.233),
                    (0.088, 0.229), (0.086, 0.237)], 80)
    g.add(v, f, MC)
    # 5 сдвоенных спиц
    for k in range(5):
        a = 2 * pi * k / 5
        for off in (-0.052, 0.052):
            v, f = spoke_geo(0.070, 0.225, 0.050, 0.036, 0.052, 0.084, a + off / 0.16)
            g.add(v, f, MR)
    # ступица + колпачок + болты
    v, f = revolve([(0.058, 0.080), (0.094, 0.080), (0.100, 0.064),
                    (0.100, 0.000), (0.058, 0.000), (0.058, 0.080)], 44)
    g.add(v, f, MR)
    v, f = revolve([(0.098, 0.048), (0.112, 0.045), (0.114, 0.030),
                    (0.112, 0.020), (0.098, 0.020)], 36)
    g.add(v, f, MC)
    for k in range(5):
        a = 2 * pi * k / 5 + 0.2
        v, f = revolve([(0.100, 0.015), (0.112, 0.014), (0.112, 0.0), (0.100, 0.0)], 8)
        v = [(p[0] + 0.068 * cos(a), p[1], p[2] + 0.068 * sin(a)) for p in v]
        g.add(v, f, MC)
    # тормозной диск + шляпа
    v, f = revolve([(-0.032, 0.186), (-0.004, 0.186), (-0.004, 0.176),
                    (-0.032, 0.176), (-0.032, 0.186)], 72)
    g.add(v, f, MD)
    v, f = revolve([(-0.006, 0.090), (0.058, 0.090), (0.058, 0.184),
                    (-0.006, 0.176), (-0.006, 0.090)], 48)
    g.add(v, f, MD)
    # суппорт
    for ang in (radians(35), radians(215)):
        cx, cz = 0.152 * cos(ang), 0.152 * sin(ang)
        v, f = box_geo(cx, -0.022, cz, 0.145, 0.062, 0.050, a=ang)
        g.add(v, f, MK)
    v, f = revolve([(-0.036, 0.184), (-0.030, 0.184), (-0.030, 0.100),
                    (-0.036, 0.100)], 48)
    g.add(v, f, MS)

    g.mirror_y(sign)
    ob = g.build(name, coll, [mats['rubber'], rim_mat, mats['chrome'],
                              mats['disc'], mats['caliper'], mats['steel']])
    return ob


def build_wheels(coll, mats):
    wheels = []
    for sign, side in ((1, "L"), (-1, "R")):
        for x, ax, tag in ((AXLE_F, "F", "Front"), (AXLE_R, "R", "Rear")):
            rim = mats['gunmetal'] if side == "L" else mats['alu']
            ob = build_wheel("Wheel_%s%s" % (tag, side), sign, coll, mats, rim)
            ob.location = (x, sign * WHEEL_Y, WHEEL_R)
            wheels.append((ob, x, sign))
    return wheels


#  4. ИНТЕРЬЕР
# ====================================================================

STEER_PARTS = []


def build_interior(coll, mats):
    L = mats['leather']
    P = mats['plastic']
    S = mats['soft']
    A = mats['alu']
    parts = []

    def add(ob, m):
        set_material(ob, m)
        parts.append(ob)
        return ob

    # ---- пол и тоннель ----
    add(cube("Floor", (3.55, 1.44, 0.04), (0.02, 0, 0.305), coll=coll,
             bv=0.012), mats['carpet']) if 'carpet' in mats else \
        add(cube("Floor", (3.55, 1.44, 0.04), (0.02, 0, 0.305), coll=coll, bv=0.012), P)

    # ---- торпедо ----
    add(cube("Dash_Main", (0.34, 1.60, 0.20), (0.68, 0, 0.74), coll=coll,
             bv=0.035, seg=4), S)
    add(cube("Dash_Top", (0.46, 1.58, 0.10), (0.63, 0, 0.862), coll=coll,
             bv=0.03, seg=4), S)
    add(cube("Cowl_Cover", (0.22, 1.52, 0.045), (0.90, 0, 0.900),
             rot=(0, radians(-4), 0), coll=coll, bv=0.015), P)
    add(cube("Dash_Lower", (0.26, 1.50, 0.24), (0.75, 0, 0.60), coll=coll,
             bv=0.03), P)
    # центральная консоль (наклонная)
    add(cube("Console", (0.30, 0.30, 0.44), (0.72, 0, 0.60),
             rot=(0, radians(-8), 0), coll=coll, bv=0.02), P)
    # дефлекторы
    for y in (-0.62, 0.62):
        add(cube("Vent", (0.06, 0.16, 0.075), (0.855, y, 0.79), coll=coll,
                 bv=0.018), mats['trim'])
        for i in range(3):
            add(cube("VentSlat", (0.012, 0.15, 0.010),
                     (0.885, y, 0.76 + i * 0.026), coll=coll, bv=0.002), A)
    # ---- приборная панель ----
    add(cube("Cluster_Hood", (0.20, 0.46, 0.10), (0.60, 0.36, 0.845),
             coll=coll, bv=0.03), S)
    add(cube("Cluster_Body", (0.12, 0.40, 0.17), (0.545, 0.36, 0.775),
             coll=coll, bv=0.015), P)
    for dy, r in ((0.115, 0.062), (0.0, 0.062), (-0.115, 0.048)):
        d = cyl("Dial", r, 0.012, (0.478, 0.36 + dy, 0.780),
                rot=(0, radians(78), 0), verts=40, coll=coll)
        add(d, mats['screen'])
        add(cyl("DialRim", r * 1.12, 0.008, (0.470, 0.36 + dy, 0.780),
                rot=(0, radians(78), 0), verts=40, coll=coll), A)
    # ---- руль (локальный кадр: плоскость XY, ось вращения Z) ----
    SW_ORIGIN = Vector((0.335, 0.36, 0.790))
    zax = Vector((-0.94, 0.0, 0.342)).normalized()      # ось вала руля
    xax = Vector((0.0, 1.0, 0.0))
    yax = zax.cross(xax).normalized()
    xax = yax.cross(zax).normalized()
    M_sw = Matrix(((xax.x, yax.x, zax.x, SW_ORIGIN.x),
                   (xax.y, yax.y, zax.y, SW_ORIGIN.y),
                   (xax.z, yax.z, zax.z, SW_ORIGIN.z),
                   (0.0, 0.0, 0.0, 1.0)))

    def sw_part(ob, mat):
        ob.matrix_world = M_sw @ ob.matrix_world
        set_material(ob, mat)
        STEER_PARTS.append((ob, ob.matrix_world.copy()))
        return ob

    bpy.ops.mesh.primitive_torus_add(major_radius=0.176, minor_radius=0.0205,
                                     major_segments=56, minor_segments=14,
                                     location=(0, 0, 0))
    rim = bpy.context.object
    rim.name = "SteeringWheel_Rim"
    put(rim, coll)
    sw_part(rim, L)
    # спицы (горизонтальные + нижняя)
    for loc_, size_ in (((0.088, 0, 0.004), (0.135, 0.048, 0.024)),
                        ((-0.088, 0, 0.004), (0.135, 0.048, 0.024)),
                        ((0, -0.092, 0.004), (0.052, 0.130, 0.024))):
        sp = cube("SW_Spoke", size_, loc_, coll=coll, bv=0.008)
        sw_part(sp, P)
    hub = cube("SW_Hub", (0.145, 0.125, 0.080), (0, 0, 0.012),
               coll=coll, bv=0.022)
    sw_part(hub, P)
    badge_ob = cyl("SW_Badge", 0.027, 0.012, (0, 0, 0.050), verts=28,
                   coll=coll)
    sw_part(badge_ob, mats['badge'])
    # вал руля до торпедо
    set_material(cyl_between("SteeringColumn",
                             SW_ORIGIN - zax * 0.02,
                             SW_ORIGIN - zax * 0.30, 0.036, coll), P)
    parts.append(bpy.data.objects["SteeringColumn"])

    # ---- центральный экран ----
    add(cube("Screen_Frame", (0.03, 0.34, 0.23), (0.66, 0, 0.815),
             rot=(0, radians(-12), 0), coll=coll, bv=0.008), P)
    add(cube("Screen_Glass", (0.008, 0.30, 0.19), (0.645, 0, 0.815),
             rot=(0, radians(-12), 0), coll=coll, bv=0.004), mats['screen'])
    # ---- сиденья ----
    def seat(tag, y):
        # подушка
        add(cube("Seat_Cushion_" + tag, (0.50, 0.46, 0.12),
                 (-0.02, y, 0.415), coll=coll, bv=0.045, seg=4), L)
        # боковые валики подушки
        for s in (-1, 1):
            add(cube("Seat_Cush_Bolster_%s%d" % (tag, s), (0.46, 0.075, 0.15),
                     (-0.02, y + s * 0.238, 0.435), coll=coll, bv=0.035), mats['leather_acc'])
        # спинка (наклон ~22°)
        add(cube("Seat_Back_" + tag, (0.12, 0.46, 0.56),
                 (-0.235, y, 0.715), rot=(0, radians(20), 0), coll=coll,
                 bv=0.04, seg=4), L)
        for s in (-1, 1):
            add(cube("Seat_Back_Bolster_%s%d" % (tag, s), (0.14, 0.07, 0.50),
                     (-0.225, y + s * 0.245, 0.715), rot=(0, radians(20), 0),
                     coll=coll, bv=0.032), mats['leather_acc'])
        # подголовник
        add(cube("Headrest_" + tag, (0.10, 0.26, 0.17), (-0.335, y, 1.075),
                 rot=(0, radians(18), 0), coll=coll, bv=0.04, seg=4), L)
        add(cube("Headrest_Pin_" + tag, (0.03, 0.20, 0.03), (-0.305, y, 0.985),
                 coll=coll, bv=0.008), A)
        # поясничный валик / шов
        add(cube("Seat_Stitch_" + tag, (0.008, 0.36, 0.012),
                 (-0.29, y, 0.80), rot=(0, radians(20), 0), coll=coll, bv=0.004),
            mats['leather_acc'])
    seat("D", 0.375)
    seat("P", -0.375)

    # задний диван
    add(cube("Rear_Cushion", (0.52, 1.28, 0.13), (-1.06, 0, 0.42),
             coll=coll, bv=0.05, seg=4), L)
    add(cube("Rear_Back", (0.13, 1.28, 0.52), (-1.30, 0, 0.70),
             rot=(0, radians(22), 0), coll=coll, bv=0.05), L)
    for y in (-0.44, 0.0, 0.44):
        add(cube("Rear_Headrest", (0.10, 0.24, 0.15), (-1.36, y, 1.045),
                 rot=(0, radians(18), 0), coll=coll, bv=0.04), L)
    add(cube("Rear_Armrest", (0.40, 0.14, 0.10), (-1.10, 0, 0.60),
             coll=coll, bv=0.03), L)

    # ---- центральный тоннель ----
    add(cube("Tunnel", (1.05, 0.26, 0.30), (0.18, 0, 0.47), coll=coll,
             bv=0.03), P)
    add(cube("Tunnel_Top", (1.05, 0.24, 0.05), (0.18, 0, 0.625), coll=coll,
             bv=0.015), mats['carbon'])
    # селектор КПП
    add(cube("Shifter_Base", (0.16, 0.11, 0.02), (0.34, 0, 0.655), coll=coll,
             bv=0.006), A)
    add(cube("Shifter", (0.055, 0.055, 0.085), (0.34, 0, 0.695), coll=coll,
             bv=0.02), L)
    # подстаканники
    for y in (-0.07, 0.07):
        add(cyl("Cupholder", 0.042, 0.05, (0.10, y, 0.655), verts=24,
                coll=coll), A)
    # кнопка старта
    add(cyl("StartStop", 0.021, 0.012, (0.55, 0.09, 0.66),
            rot=(radians(90), 0, 0), verts=24, coll=coll), mats['caliper'])
    # подлокотник
    add(cube("Armrest", (0.32, 0.24, 0.09), (-0.28, 0, 0.64), coll=coll,
             bv=0.035), L)

    # ---- карты дверей ----
    for s in (-1, 1):
        add(cube("Door_Trim_%d" % s, (1.62, 0.05, 0.44), (-0.30, s * 0.755, 0.62),
                 coll=coll, bv=0.02), P)
        add(cube("Door_Armrest_%d" % s, (0.46, 0.11, 0.07),
                 (-0.16, s * 0.735, 0.70), coll=coll, bv=0.03), L)
        add(cube("Door_Handle_%d" % s, (0.16, 0.035, 0.035),
                 (-0.02, s * 0.725, 0.80), coll=coll, bv=0.012), A)
        add(cyl("Door_Speaker_%d" % s, 0.072, 0.012, (-0.34, s * 0.728, 0.50),
                rot=(radians(90), 0, 0), verts=32, coll=coll), mats['trim'])
        add(cyl("Door_Speaker_Ring_%d" % s, 0.082, 0.010, (-0.34, s * 0.723, 0.50),
                rot=(radians(90), 0, 0), verts=32, coll=coll), A)
        add(cube("Door_Sill_%d" % s, (1.30, 0.06, 0.03), (-0.30, s * 0.77, 0.40),
                 coll=coll, bv=0.008), mats['alu'])

    # ---- салонное зеркало ----
    add(cube("Mirror_Rear", (0.045, 0.24, 0.075), (0.36, 0, 1.30),
             rot=(0, radians(-12), 0), coll=coll, bv=0.014), P)
    add(cyl("Mirror_Stem", 0.012, 0.12, (0.40, 0, 1.34),
            rot=(0, radians(30), 0), verts=12, coll=coll), P)

    # ---- педали ----
    for y, h in ((0.28, 0.10), (0.42, 0.07)):
        add(cube("Pedal", (0.04, h, 0.021), (0.98, y, 0.44),
                 rot=(0, radians(28), 0), coll=coll, bv=0.008), A)

    # ---- обивка потолка ----
    verts, faces = [], []
    NX, NY = 26, 9
    for i in range(NX):
        x = -1.50 + 2.10 * i / (NX - 1.0)
        zc = ZR(x) - 0.030
        wr = max(WR(x) - 0.055, 0.12)
        for j in range(NY):
            y = -wr + 2 * wr * j / (NY - 1.0)
            z = zc - 0.055 * (y / wr) ** 2
            verts.append((x, y, z))
    for i in range(NX - 1):
        for j in range(NY - 1):
            a_ = i * NY + j
            faces.append((a_, a_ + NY, a_ + NY + 1, a_ + 1))
    head = new_mesh_object("Headliner", verts, faces, coll)
    add(head, mats['headliner'])

    # ---- крышка багажника (внутренняя полка) ----
    add(cube("Parcel_Shelf", (0.38, 1.24, 0.02), (-1.64, 0, 0.875),
             coll=coll, bv=0.01), P)

    for ob in parts:
        set_material(ob, ob.data.materials[0]) if ob.data.materials else None
    return parts


# ====================================================================
# ====================================================================
#  5. НАРУЖНЫЕ ДЕТАЛИ
# ====================================================================

def surf_y_quiet(x, z):
    """Полуширина борта по сечению (без учёта арки) - для фланца проёма."""
    half, *_ = half_profile(x)
    zmin = min(p[1] for p in half)
    zmax = max(p[1] for p in half)
    z = clamp(z, zmin + 0.02, zmax - 0.02)
    best = 0.0
    for i in range(len(half) - 1):
        (y1, z1), (y2, z2) = half[i], half[i + 1]
        if (z1 - z) * (z2 - z) <= 0 and abs(z2 - z1) > 1e-9:
            t = (z - z1) / (z2 - z1)
            best = max(best, y1 + t * (y2 - y1))
    return best


def surf_y(x, z):
    """Полуширина кузова на высоте z в сечении x (посадка наружных деталей)."""
    half, *_ = half_profile(x)
    zmin = min(p[1] for p in half)
    zmax = max(p[1] for p in half)
    z = clamp(z, zmin + 0.012, zmax - 0.004)
    best = 0.0
    for i in range(len(half) - 1):
        (y1, z1), (y2, z2) = half[i], half[i + 1]
        if (z1 - z) * (z2 - z) <= 0 and abs(z2 - z1) > 1e-9:
            t = (z - z1) / (z2 - z1)
            best = max(best, y1 + t * (y2 - y1))
    return best


def body_hit(y, z, front=True):
    """Точка и нормаль поверхности кузова в (y, z): луч извне внутрь по оси X.
    Возвращает (loc, normal) или None. Работает по реальной (с сабдивом) сетке."""
    dg = bpy.context.evaluated_depsgraph_get()
    body = bpy.data.objects.get("Body")
    if body is None:
        return None
    ob = body.evaluated_get(dg)
    if front:
        origin, d = Vector((L_FRONT + 0.8, y, z)), Vector((-1, 0, 0))
    else:
        origin, d = Vector((L_REAR - 0.8, y, z)), Vector((1, 0, 0))
    hit, loc, nrm, idx = ob.ray_cast(origin, d, distance=2.2)
    if not hit:
        return None
    return loc, nrm.normalized()


def x_face(y, z, front=True):
    r = body_hit(y, z, front)
    return r[0].x if r else (L_FRONT if front else L_REAR)


_DETAIL_COLL = [None]


def face_patch(name, nu, nv, y0, y1, z0, z1, front=True, off=0.004, coll=None):
    """Патч, лежащий на передней/задней поверхности кузова (по нормали).
    off: + наружу, - внутрь кузова."""
    verts, faces = [], []
    last = None
    for i in range(nu):
        y = y0 + (y1 - y0) * i / (nu - 1.0)
        for j in range(nv):
            z = z0 + (z1 - z0) * j / (nv - 1.0)
            r = body_hit(y, z, front)
            if r is None:
                # точка вне силуэта: подтягиваем к центру и внутрь по высоте
                for k in range(1, 13):
                    yy = y * (1.0 - 0.075 * k)
                    zz = z + (0.72 - z) * 0.10 * k
                    r = body_hit(yy, zz, front)
                    if r is not None:
                        y, z = yy, zz
                        break
            if r is None:
                p = last if last is not None else Vector(
                    (L_FRONT if front else L_REAR, y, z))
            else:
                p = r[0] + r[1] * off
                last = p
            verts.append(tuple(p))
    for i in range(nu - 1):
        for j in range(nv - 1):
            a = i * nv + j
            faces.append((a, a + nv, a + nv + 1, a + 1))
    return new_mesh_object(name, verts, faces, coll or _DETAIL_COLL[0])


def surf_patch(name, nu, nv, fn, coll):
    """Патч, лежащий на поверхности кузова: fn(u,v)->(x,y,z)."""
    verts, faces = [], []
    for i in range(nu):
        for j in range(nv):
            verts.append(fn(i / (nu - 1.0), j / (nv - 1.0)))
    for i in range(nu - 1):
        for j in range(nv - 1):
            a = i * nv + j
            faces.append((a, a + nv, a + nv + 1, a + 1))
    return new_mesh_object(name, verts, faces, coll)


def ribbon(name, pts, width_dir, w, coll, mat, close=False):
    """Плоская лента по списку 3D-точек (молдинги, канты арок)."""
    verts, faces = [], []
    for i, p in enumerate(pts):
        p = Vector(p)
        t = Vector(pts[min(i + 1, len(pts) - 1)]) - Vector(pts[max(i - 1, 0)])
        if t.length < 1e-9:
            t = Vector((0, 0, 1))
        t.normalize()
        d = Vector(width_dir).normalized()
        n = t.cross(d)
        if n.length < 1e-9:
            n = d.copy()
        n.normalize()
        verts.append(tuple(p - d * w / 2))
        verts.append(tuple(p + d * w / 2))
    for i in range(len(pts) - 1):
        a = i * 2
        faces.append((a, a + 1, a + 3, a + 2))
    ob = new_mesh_object(name, verts, faces, coll)
    set_material(ob, mat)
    recalc_normals(ob)
    return ob


def build_exterior(coll, mats):
    _DETAIL_COLL[0] = coll
    parts = []
    TR, CH, PA = mats['trim'], mats['chrome'], mats['paint']

    def add(ob, m):
        set_material(ob, m)
        parts.append(ob)
        return ob

    # ---------- фары ----------
    for s in (-1, 1):
        y0, y1 = 0.300, 0.815
        lens = face_patch("Headlight_%d" % s, 12, 6, s * y0, s * y1,
                          0.700, 0.830, front=True, off=0.0015)
        add(lens, mats['lens'])
        bevel(lens, 0.008, 2)
        add(face_patch("HeadlightHouse_%d" % s, 12, 6, s * y0, s * y1,
                       0.692, 0.838, front=True, off=-0.012), TR)
        # проекторы
        for yy, r in ((0.45, 0.040), (0.63, 0.032)):
            z = 0.772
            x = x_face(s * yy, z, True) - 0.022
            add(cyl("Projector_%d_%d" % (s, int(yy * 100)), r, 0.020,
                    (x, s * yy, z), rot=(0, radians(90), 0), verts=28,
                    coll=coll), CH)
            add(cyl("ProjectorGlass_%d_%d" % (s, int(yy * 100)), r * 0.72, 0.014,
                    (x + 0.014, s * yy, z), rot=(0, radians(90), 0), verts=28,
                    coll=coll), mats['headlight'])
        # DRL - «молот Тора»: горизонтальная полоса + вертикальный штрих
        add(face_patch("DRL_H_%d" % s, 10, 2, s * 0.315, s * 0.800,
                       0.806, 0.820, front=True, off=0.0015), mats['drl'])
        add(face_patch("DRL_V_%d" % s, 2, 6, s * 0.315, s * 0.348,
                       0.716, 0.806, front=True, off=0.0015), mats['drl'])

    # ---------- задние фонари ----------
    for s in (-1, 1):
        tl = face_patch("Taillight_%d" % s, 16, 7, s * 0.300, s * 0.840,
                        0.820, 0.930, front=False, off=-0.0015)
        add(tl, mats['tail'])
        bevel(tl, 0.004, 2)
        add(face_patch("TaillightHouse_%d" % s, 16, 7, s * 0.292, s * 0.848,
                       0.814, 0.936, front=False, off=-0.008), TR)
    add(face_patch("Tail_Bar", 2, 20, -0.300, 0.300, 0.872, 0.882,
                   front=False, off=-0.0015), mats['tail'])

    # ---------- решётка ----------
    # корпус решётки утоплен в кузов, ламели лежат чуть ниже кромки рамки
    add(face_patch("Grille_Back", 16, 10, -0.330, 0.330, 0.500, 0.700,
                   front=True, off=-0.022), TR)
    for k in range(5):
        z = 0.530 + k * 0.033
        add(face_patch("Grille_Slat_%d" % k, 16, 2, -0.322, 0.322,
                       z - 0.006, z + 0.006, front=True, off=-0.010), CH)
    # рамка - единый контур по периметру (не «парящие» полосы)
    add(face_patch("Grille_Frame_T", 16, 2, -0.336, 0.336, 0.694, 0.706,
                   front=True, off=-0.002), CH)
    add(face_patch("Grille_Frame_B", 16, 2, -0.336, 0.336, 0.494, 0.506,
                   front=True, off=-0.002), CH)
    for s in (-1, 1):
        add(face_patch("Grille_Frame_S%d" % s, 2, 10, s * 0.330, s * 0.342,
                       0.494, 0.706, front=True, off=-0.002), CH)
    bx = x_face(0.0, 0.600, True)
    add(cyl("Badge_Front", 0.050, 0.008, (bx - 0.006, 0, 0.600),
            rot=(0, radians(90), 0), verts=36, coll=coll), mats['badge'])
    add(cyl("Badge_Front_Ring", 0.062, 0.006, (bx - 0.008, 0, 0.600),
            rot=(0, radians(90), 0), verts=36, coll=coll), CH)

    # ---------- нижний воздухозаборник ----------
    add(face_patch("Intake_Lower", 12, 6, -0.470, 0.470, 0.330, 0.452,
                   front=True, off=-0.024), TR)
    for k in range(3):
        yy = -0.28 + k * 0.28
        add(face_patch("Intake_Fin_%d" % k, 2, 6, yy - 0.006, yy + 0.006,
                       0.330, 0.452, front=True, off=0.002), TR)
    add(cube("Front_Splitter", (0.18, 1.06, 0.022), (2.320, 0, 0.300),
             coll=coll, bv=0.007), mats['carbon'])
    for s in (-1, 1):
        fx = x_face(s * 0.600, 0.478, True)
        add(cyl("Fog_%d" % s, 0.032, 0.05, (fx - 0.010, s * 0.600, 0.478),
                rot=(0, radians(90), 0), verts=28, coll=coll), mats['drl'])

    # ---------- зеркала ----------
    MX, MZ = 0.735, 1.020
    for s in (-1, 1):
        y0 = W(MX) * 1.005
        add(cube("Mirror_Stalk_%d" % s, (0.075, 0.130, 0.055),
                 (MX + 0.022, s * (y0 + 0.010), MZ - 0.030),
                 rot=(0, 0, s * radians(-14)), coll=coll, bv=0.018), TR)
        add(cube("Mirror_Shell_%d" % s, (0.215, 0.145, 0.125),
                 (MX - 0.010, s * (y0 + 0.072), MZ), rot=(0, 0, s * radians(-7)),
                 coll=coll, bv=0.050, seg=4), PA)
        add(cube("Mirror_Glass_%d" % s, (0.012, 0.104, 0.086),
                 (MX - 0.109, s * (y0 + 0.070), MZ), rot=(0, 0, s * radians(-7)),
                 coll=coll, bv=0.006), mats['mirror'])
        add(cube("Mirror_Turn_%d" % s, (0.085, 0.018, 0.011),
                 (MX - 0.062, s * (y0 + 0.111), MZ - 0.048), coll=coll, bv=0.004),
            mats['drl'])

    # ---------- молдинги, ручки, пороги ----------
    for s in (-1, 1):
        add(cube("SideSkirt_%d" % s, (1.52, 0.020, 0.030),
                 (-0.05, s * 0.868, 0.240), coll=coll, bv=0.010), TR)
        pts = []
        for i in range(18):
            x = -1.46 + 2.72 * i / 17.0
            z = 0.640 - 0.010 * abs(x)
            pts.append((x, s * (surf_y(x, z) + 0.0015), z))
        add(ribbon("SideMoulding_%d" % s, pts, (0, s, 0), 0.012, coll, CH), CH)
        for x in (0.420, -0.560):
            y = surf_y(x, 0.900)
            add(cube("HandleRecess_%d_%d" % (s, int(x * 100)),
                     (0.200, 0.022, 0.046), (x, s * (y - 0.002), 0.900),
                     coll=coll, bv=0.008, seg=3), TR)
            add(cube("Handle_%d_%d" % (s, int(x * 100)),
                     (0.170, 0.016, 0.026), (x, s * (y + 0.008), 0.900),
                     coll=coll, bv=0.008, seg=3), CH)
    add(cube("Shark_Fin", (0.16, 0.048, 0.052), (-1.100, 0, ZR(-1.10) - 0.008),
             rot=(0, radians(-8), 0), coll=coll, bv=0.018), PA)

    # ---------- корма ----------
    add(cube("Diffuser", (0.24, 1.16, 0.085), (-2.330, 0, 0.292), coll=coll,
             bv=0.014), mats['carbon'])
    for k in range(6):
        add(cube("Diffuser_Fin_%d" % k, (0.24, 0.013, 0.100),
                 (-2.370, -0.49 + k * 0.196, 0.300), coll=coll, bv=0.004),
            mats['carbon'])
    for s in (-1, 1):
        ex = x_face(s * 0.420, 0.335, False)
        add(cyl("Exhaust_%d" % s, 0.046, 0.09, (ex + 0.030, s * 0.420, 0.335),
                rot=(0, radians(90), 0), verts=28, coll=coll), CH)
        add(cyl("Exhaust_In_%d" % s, 0.037, 0.075, (ex + 0.024, s * 0.420, 0.335),
                rot=(0, radians(90), 0), verts=28, coll=coll), TR)
    rbx = x_face(0.0, 0.930, False)
    add(cyl("Badge_Rear", 0.046, 0.008, (rbx + 0.006, 0, 0.930),
            rot=(0, radians(90), 0), verts=36, coll=coll), mats['badge'])

    # ---------- хром по линии остекления ----------
    for s in (-1, 1):
        pts = []
        for i in range(30):
            x = -1.320 + 2.030 * i / 29.0
            z = BELT(x) + 0.004
            pts.append((x, s * (surf_y(x, z) + 0.0030), z))
        add(ribbon("DLO_Trim_%d" % s, pts, (0, s, 0), 0.012, coll, CH), CH)

    # ---------- номера ----------
    pfx = x_face(0.0, 0.505, True)
    add(cube("Plate_Front", (0.010, 0.36, 0.11), (pfx - 0.012, 0, 0.470),
             coll=coll, bv=0.005), mats['plate'])
    prx = x_face(0.0, 0.700, False)
    add(cube("Plate_Rear", (0.012, 0.36, 0.11), (prx - 0.004, 0, 0.700),
             coll=coll, bv=0.006), mats['plate'])
    add(cube("Plate_Front_Frame", (0.016, 0.39, 0.135), (pfx - 0.016, 0, 0.470),
             coll=coll, bv=0.006), TR)
    add(cube("Plate_Rear_Frame", (0.010, 0.39, 0.135), (prx + 0.002, 0, 0.700),
             coll=coll, bv=0.006), TR)

    # ---------- зазоры панелей ----------
    GAP_M = mats['trim']
    for s in (-1, 1):
        for xg, z0, z1 in ((0.720, 0.225, 0.0), (-0.190, 0.225, 0.0),
                           (-1.075, 0.225, 0.0)):
            # щель: две тонкие панели внахлёст + тёмная сердцевина между ними
            # (лента, «плывущая» по поверхности, на бликах смотрится наклейкой)
            pts_a, pts_b, pts_c = [], [], []
            ztop = BELT(xg) - 0.014
            for i in range(22):
                z = z0 + (ztop - z0) * i / 21.0
                y = surf_y(xg, z)
                pts_a.append((xg - 0.0028, s * (y + 0.0018), z))
                pts_b.append((xg, s * (y + 0.0006), z))
                pts_c.append((xg + 0.0028, s * (y + 0.0018), z))
            add(ribbon("PanelGapA_%d_%d" % (s, int(xg * 100)), pts_a,
                       (1, 0, 0), 0.0012, coll, GAP_M), GAP_M)
            add(ribbon("PanelGapB_%d_%d" % (s, int(xg * 100)), pts_b,
                       (1, 0, 0), 0.0042, coll, GAP_M), GAP_M)
            add(ribbon("PanelGapC_%d_%d" % (s, int(xg * 100)), pts_c,
                       (1, 0, 0), 0.0012, coll, GAP_M), GAP_M)
        # шов капота по борту крыла
        pts = []
        for i in range(18):
            x = 1.170 + 1.280 * i / 17.0
            z = ZR(x) - 0.045
            pts.append((x, s * (surf_y(x, z) + 0.0008), z))
        add(ribbon("HoodGap_%d" % s, pts, (0, 0, 1), 0.0042, coll, GAP_M), GAP_M)
        # шов крышки багажника
        pts = []
        for i in range(12):
            x = -1.560 - 0.560 * i / 11.0
            z = ZR(x) - 0.040
            pts.append((x, s * (surf_y(x, z) + 0.0015), z))
        add(ribbon("TrunkGap_%d" % s, pts, (0, 0, 1), 0.005, coll, GAP_M), GAP_M)
    for xg, tag, ymax in ((1.170, "Hood", 0.62), (-1.560, "Trunk", 0.58)):
        pts = []
        for i in range(23):
            y = (-ymax + 2 * ymax * i / 22.0)
            pts.append((xg, y, ZR(xg) + 0.0015))
        add(ribbon("Gap%s_Cross" % tag, pts, (1, 0, 0), 0.005, coll, GAP_M), GAP_M)

    # ---------- щётки ----------
    for s, x0 in ((1, 0.34), (-1, -0.06)):
        add(cube("Wiper_%d" % s, (0.40, 0.009, 0.005),
                 (1.075, x0, ZR(1.075) + 0.005),
                 rot=(0, radians(-10), radians(s * 5)),
                 coll=coll, bv=0.002), TR)
        add(cube("WiperArm_%d" % s, (0.13, 0.011, 0.007),
                 (1.245, x0 * 0.86, ZR(1.245) + 0.005),
                 rot=(0, radians(-9), radians(s * 7)),
                 coll=coll, bv=0.003), TR)

    # ---------- лючок ----------
    yf = surf_y(-1.75, BELT(-1.75) - 0.120)
    add(cyl("Fuel_Cap", 0.070, 0.012,
            (-1.75, -(yf - 0.001), BELT(-1.75) - 0.120),
            rot=(radians(90), 0, 0), verts=32, coll=coll), PA)

    return parts


#  6. СЦЕНА: свет, студия, камеры, рендер
# ====================================================================

def look_at(ob, target):
    d = Vector(target) - ob.location
    ob.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()


def new_empty(name, loc, coll, rot=(0, 0, 0)):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.25
    e.empty_display_type = 'PLAIN_AXES'
    e.location = loc
    e.rotation_euler = rot
    coll.objects.link(e)
    return e


def world_of(ob):
    """Мировая матрица без обращения к (возможно устаревшему) matrix_world."""
    if ob.parent is None:
        return ob.matrix_basis.copy()
    return ob.matrix_world.copy()


def parent_keep(child, parent):
    """Привязать объект, сохранив мировое положение (без depsgraph-зависимости)."""
    mw = world_of(child)
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.matrix_basis = world_of(parent).inverted() @ mw


def parent_local(child, parent):
    """Привязать объект, трактуя его трансформ в локальных осях родителя."""
    child.parent = parent
    child.matrix_parent_inverse = Matrix.Identity(4)


def build_scene(mats, colls):
    sc = bpy.context.scene
    env = colls['env']

    # ---- земля / циклорама ----
    me = bpy.data.meshes.new("StudioFloor")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=24.0)
    bm.to_mesh(me)
    bm.free()
    fl = bpy.data.objects.new("StudioFloor", me)
    env.objects.link(fl)
    mfloor = pbr("Floor_Studio", (0.055, 0.058, 0.062), 0.0, 0.22)
    set_material(fl, mfloor)

    # ---- мировая среда ----
    w = bpy.data.worlds.new("StudioWorld") if not bpy.data.worlds else bpy.data.worlds[0]
    sc.world = w
    w.use_nodes = True
    wn, wl = w.node_tree.nodes, w.node_tree.links
    wn.clear()
    wout = wn.new('ShaderNodeOutputWorld')
    bg = wn.new('ShaderNodeBackground')
    grad = wn.new('ShaderNodeTexGradient')
    grad.gradient_type = 'EASING'
    mapn = wn.new('ShaderNodeMapping')
    texc = wn.new('ShaderNodeTexCoord')
    ramp = wn.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].position = 0.25
    ramp.color_ramp.elements[0].color = (0.015, 0.017, 0.022, 1)
    ramp.color_ramp.elements[1].position = 0.85
    ramp.color_ramp.elements[1].color = (0.075, 0.085, 0.105, 1)
    wl.new(texc.outputs['Generated'], grad.inputs['Vector'])
    wl.new(grad.outputs['Color'], ramp.inputs['Fac'])
    bg.inputs['Strength'].default_value = 1.0
    wl.new(bg.outputs['Background'], wout.inputs['Surface'])

    # ---- HDRI-окружение (файл ищется рядом со скриптом) ----
    hdri_path = os.path.join(OUT_DIR, "hdri", "studio_small_09.hdr")
    if not os.path.exists(hdri_path):
        hdri_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "hdri", "studio_small_09.hdr")
    if os.path.exists(hdri_path):
        env_t = wn.new('ShaderNodeTexEnvironment')
        env_t.location = (-300, 200)
        env_t.image = bpy.data.images.load(hdri_path)
        wl.new(env_t.outputs['Color'], bg.inputs['Color'])
        bg.inputs['Strength'].default_value = 1.0
        print(">> HDRI:", hdri_path)
    else:
        wl.new(ramp.outputs['Color'], bg.inputs['Color'])
        bg.inputs['Strength'].default_value = 1.0

    # ---- источники света ----
    def area(name, loc, size, energy, color=(1, 1, 1), target=(0, 0, 0.7)):
        ld = bpy.data.lights.new(name, 'AREA')
        ld.shape = 'RECTANGLE'
        ld.size, ld.size_y = size
        ld.energy = energy
        ld.color = color
        ob = bpy.data.objects.new(name, ld)
        ob.location = loc
        env.objects.link(ob)
        look_at(ob, target)
        return ob

    area("Key_Softbox", (-3.4, -5.2, 5.0), (5.0, 2.6), 420.0, (1.0, 0.98, 0.95))
    area("Fill_Top", (4.6, 3.6, 4.2), (4.0, 2.0), 220.0, (0.92, 0.96, 1.0))
    area("Rim_Back", (-5.6, 4.4, 3.4), (3.4, 1.4), 260.0, (0.85, 0.92, 1.0))
    area("Front_Strip", (7.2, 0.0, 2.2), (1.2, 4.6), 180.0, (1.0, 1.0, 1.0))
    area("Top_Strip", (0.4, 0.0, 6.4), (2.0, 5.0), 260.0, (1.0, 1.0, 1.0))

    # ---- свет в салоне (чтобы интерьер читался) ----
    for name, loc, size, energy, tgt in (
            ("Int_RoofLight_F", (0.55, 0.0, 1.335), (0.34, 1.00), 3.4, (0.60, 0.0, 0.62)),
            ("Int_RoofLight_M", (-0.35, 0.0, 1.345), (0.44, 1.10), 3.8, (-0.30, 0.0, 0.58)),
            ("Int_SeatLight", (-1.20, 0.0, 1.290), (0.34, 1.00), 2.6, (-1.30, 0.0, 0.55)),
            ("Int_DashLight", (1.05, 0.0, 1.120), (0.24, 0.80), 1.2, (0.70, 0.0, 0.78)),
            ("Int_Ceiling", (-0.20, 0.0, 1.140), (0.50, 1.20), 1.8, (-0.20, 0.0, 1.45))):
        ob = area(name, loc, size, energy, (1.0, 0.96, 0.90), tgt)
        ob.visible_camera = False
        ob.data.shadow_soft_size = 0.15

    # ---- камеры ----
    def camera(name, loc, target, lens=60.0, fstop=8.0):
        cd = bpy.data.cameras.new(name)
        cd.lens = lens
        cd.sensor_width = 36.0
        cd.dof.use_dof = True
        cd.dof.aperture_fstop = fstop
        ob = bpy.data.objects.new(name, cd)
        ob.location = loc
        env.objects.link(ob)
        look_at(ob, target)
        cd.dof.focus_distance = (Vector(target) - Vector(loc)).length
        return ob

    cam_ext = camera("Cam_Exterior", (6.05, -5.15, 1.66), (0.05, 0.0, 0.74),
                     62.0, 5.6)
    cam_rear = camera("Cam_Rear", (-5.9, -4.7, 1.90), (0.0, 0.0, 0.72), 62.0, 5.6)
    cam_int = camera("Cam_Interior", (-1.14, -0.36, 1.20), (0.80, 0.08, 0.80),
                     19.0, 8.0)
    cam_int2 = camera("Cam_Interior2", (0.62, 0.60, 1.185), (-0.35, -0.05, 0.66),
                      20.0, 6.0)
    cam_side = camera("Cam_Side", (0.10, -10.2, 1.30), (0.0, 0.0, 0.74), 68.0, 8.0)

    # ---- параметры рендера ----
    sc.render.engine = 'CYCLES'
    sc.cycles.device = 'CPU'
    sc.cycles.samples = 48
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 8
    sc.cycles.transmission_bounces = 6
    sc.cycles.glossy_bounces = 4
    sc.cycles.caustics_reflective = False
    sc.cycles.caustics_refractive = False
    sc.cycles.sample_clamp_indirect = 6.0
    sc.render.resolution_x = 1440
    sc.render.resolution_y = 810
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.view_settings.view_transform = 'AgX'
    try:
        sc.view_settings.look = 'AgX - Medium High Contrast'
    except Exception:
        pass
    sc.view_settings.exposure = 0.0
    sc.render.fps = 24
    sc.frame_start = 1
    sc.frame_end = 240
    sc.camera = cam_ext
    return dict(ext=cam_ext, rear=cam_rear, interior=cam_int,
                interior2=cam_int2, side=cam_side)


# ====================================================================
#  7. АНИМАЦИЯ
# ====================================================================

def key(ob, path, frame, value, interp='BEZIER', index=-1):
    if index >= 0:
        getattr(ob, path)[index] = value
        ob.keyframe_insert(data_path=path, index=index, frame=frame)
    else:
        setattr(ob, path, value)
        ob.keyframe_insert(data_path=path, frame=frame)
    ad = ob.animation_data
    if ad and ad.action:
        for fc in ad.action.fcurves:
            if fc.data_path == path and (index < 0 or fc.array_index == index):
                for kp in fc.keyframe_points:
                    if abs(kp.co[0] - frame) < 1e-3:
                        kp.interpolation = interp


def build_animation(root, steer_empties, wheels, steer_wheel_rig, body_rig):
    sc = bpy.context.scene
    F0, F1 = 1, 240

    # ---- качение колёс: непрерывное вращение вокруг локальной оси Y ----
    # 40 оборотов за 240 кадров (24 fps) ~ 87 км/ч при R=0.345 м
    turns = 40.0
    for ob in wheels:
        ob.rotation_mode = 'XYZ'
        ob.rotation_euler = (0, 0, 0)
        key(ob, "rotation_euler", F0, 0.0, 'LINEAR', index=1)
        key(ob, "rotation_euler", F1, 2 * pi * turns, 'LINEAR', index=1)

    # ---- руль: поворот передних колёс на ±18 град ----
    steer_keys = [(F0, 0.0), (55, -16.0), (95, -16.0), (120, 0.0),
                  (165, 17.0), (205, 17.0), (F1, 0.0)]
    for e in steer_empties:
        e.rotation_mode = 'XYZ'
        for f, ang in steer_keys:
            key(e, "rotation_euler", f, radians(ang), 'BEZIER', index=2)

    # ---- баранка: передаточное отношение ~ 12:1 ----
    if steer_wheel_rig:
        steer_wheel_rig.rotation_mode = 'XYZ'
        for f, ang in steer_keys:
            key(steer_wheel_rig, "rotation_euler", f, radians(-ang * 11.0),
                'BEZIER', index=2)

    # ---- раскачка кузова (крен + микролифт) ----
    if body_rig:
        roll_keys = [(F0, 0.0), (55, 0.75), (95, 0.75), (120, 0.0),
                     (165, -0.8), (205, -0.8), (F1, 0.0)]
        for f, deg in roll_keys:
            key(body_rig, "rotation_euler", f, radians(deg), 'BEZIER', index=0)
        bob = [(F0, 0.0), (30, 0.0055), (60, -0.004), (90, 0.005),
               (120, -0.0035), (150, 0.006), (180, -0.004), (210, 0.004),
               (F1, 0.0)]
        for f, dz in bob:
            key(body_rig, "location", f, dz, 'BEZIER', index=2)

    # если хочется «ехать» - раскомментируйте:
    # for f, x in ((F0, 0.0), (F1, 14.0)):
    #     key(root, "location", f, x, 'LINEAR', index=0)
    sc.frame_set(F0)
    return sc


# ====================================================================
#  8. СБОРКА СЦЕНЫ
# ====================================================================

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def make_colls():
    sc = bpy.context.scene
    root = sc.collection
    car = bpy.data.collections.new("CAR")
    env = bpy.data.collections.new("ENV_Studio")
    root.children.link(car)
    root.children.link(env)
    C = {"car": car, "env": env}
    for k, label in (("body", "01_Body"), ("wheels", "02_Wheels"),
                     ("interior", "03_Interior"), ("details", "04_Details"),
                     ("rig", "05_Rig")):
        c = bpy.data.collections.new(label)
        car.children.link(c)
        C[k] = c
    return C


def main():
    reset_scene()
    colls = make_colls()
    mats = build_materials()
    mats['carpet'] = pbr("Carpet", (0.020, 0.020, 0.024), 0.0, 0.92)

    print(">> кузов")
    body, xs = build_body(colls['body'], mats)

    print(">> колёса")
    wheels_data = build_wheels(colls['wheels'], mats)

    print(">> интерьер")
    build_interior(colls['interior'], mats)

    print(">> наружные детали")
    build_exterior(colls['details'], mats)

    # ---------------- риг ----------------
    rig = colls['rig']
    root = new_empty("Car_Root", (0, 0, 0), rig)
    body_rig = new_empty("Body_Rig", (0, 0, 0), rig)

    # кузов + наружные детали -> Body_Rig (качается вместе с кузовом)
    for ob in list(colls['body'].objects) + list(colls['details'].objects):
        parent_keep(ob, body_rig)

    # интерьер -> Car_Root
    for ob in list(colls['interior'].objects):
        parent_keep(ob, root)

    # колёса: держим размеры ОТДЕЛЬНО от выравнивания колёсной базы,
    # чтобы не потерять привязку по знаку x
    wheels = []
    steer_empties = []
    for ob, x, sign in wheels_data:
        if x > 1.0:
            st = new_empty("Steer_" + ("L" if sign > 0 else "R"),
                           (x, sign * WHEEL_Y, WHEEL_R), rig)
            parent_keep(ob, st)
            wheels.append(ob)
            steer_empties.append(st)
        else:
            parent_keep(ob, root)
            wheels.append(ob)

    # руль: наклонный риг -> вращающийся риг -> детали
    bpy.context.view_layer.update()
    if STEER_PARTS:
        sw_tilt = new_empty("SteeringWheel_Rake",
                            tuple(STEER_PARTS[0][1].to_translation()), rig)
        sw_tilt.matrix_world = STEER_PARTS[0][1].to_quaternion().to_matrix().to_4x4()
        sw_tilt.location = STEER_PARTS[0][1].to_translation()
        bpy.context.view_layer.update()
        sw_spin = new_empty("SteeringWheel_Rig", (0, 0, 0), rig)
        sw_spin.parent = sw_tilt
        sw_spin.matrix_parent_inverse = Matrix.Identity(4)
        tilt_inv = sw_tilt.matrix_world.inverted()
        for ob, world_m in STEER_PARTS:
            ob.parent = sw_spin
            ob.matrix_parent_inverse = Matrix.Identity(4)
            ob.matrix_basis = tilt_inv @ world_m
        steer_wheel_rig = sw_spin
    else:
        steer_wheel_rig = None
    build_animation(root, steer_empties, wheels, steer_wheel_rig, body_rig)
    cameras = build_scene(mats, colls)

    sc = bpy.context.scene
    sc.frame_set(1)

    try:
        bpy.ops.file.pack_all()
        print(">> все внешние ресурсы упакованы в .blend")
    except Exception as e:
        print(">> упаковка ресурсов:", e)
    blend_path = os.path.join(OUT_DIR, "car_realistic.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(">> сохранено:", blend_path)

    if DO_RENDER:
        shots = [("render_exterior.png", cameras['ext'], (1440, 810), 48, 0.0),
                 ("render_side.png", cameras['side'], (1440, 810), 40, 0.0),
                 ("render_rear.png", cameras['rear'], (1280, 720), 36, 0.0),
                 ("render_interior.png", cameras['interior'], (1280, 720), 48, 1.0),
                 ("render_interior2.png", cameras['interior2'], (1280, 720), 48, 1.0)]
        for fname, cam, res, samples, expo in shots:
            sc.view_settings.exposure = expo
            sc.camera = cam
            sc.render.resolution_x, sc.render.resolution_y = res
            sc.cycles.samples = samples
            sc.render.filepath = os.path.join(OUT_DIR, fname)
            print(">> рендер:", fname)
            bpy.ops.render.render(write_still=True)
    print(">> готово")


if __name__ == "__main__":
    main()
