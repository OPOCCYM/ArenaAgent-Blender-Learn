#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_geometry.py — чистые математические параметры, кривые кузова и вспомогательные
функции без зависимости от bpy (можно тестировать через pytest вне Blender).

Система координат автомобиля (метры, Blender):
  * +X — перед (нос, X_MAX = +2.50, передняя ось AXLE_F = +1.60)
  * -X — корма (хвост, X_MIN = -2.50, задняя ось AXLE_R = -1.46)
  * +Y — левый борт (сторона водителя при левом руле)
  * -Y — правый борт
  * +Z — вверх (дорога Z = 0.0, крыша до HEIGHT = 1.456)
"""
import math
import re

# --------------------------------------------------------------------------- #
# 1. Габариты и константы
# --------------------------------------------------------------------------- #
LENGTH = 5.00        # длина, м
WIDTH = 1.872        # ширина, м
HEIGHT = 1.432       # высота крыши от дороги, м (низкий спортивный силуэт Volvo S90)
GROUND_CLEARANCE = 0.16  # минимальный дорожный просвет под порогом, м

AXLE_F = 1.60        # передняя ось (+X)
AXLE_R = -1.46       # задняя ось (-X), база 3.06 м
WHEEL_Y = 0.792      # половина колеи (колея 1.584 м)
WHEEL_R = 0.353      # радиус шины 235/45 R18
WHEEL_Z = 0.353      # центр качения колеса (шина касается Z = 0.0)
WHEEL_WIDTH = 0.235  # ширина шины, м

ARCH_R = 0.445       # радиус зоны колёсной арки
ARCH_DEPTH = 0.13    # глубина впадины борта у колеса
TIP_DOME_F = 0.14    # аккуратный скруглённый торец носа (+X, за пределами проёма капота)
TIP_DOME_R = 0.14    # аккуратный скруглённый торец кормы (-X, за пределами проёма багажника)

N_SECTIONS = 200     # повышенная плотность сечений для плавного силуэта S90
V_BODY = 18          # число точек профиля кузова в сечении
V_GH = 9             # число точек профиля остекления

X_MIN = -LENGTH / 2.0  # корма (-2.50)
X_MAX = LENGTH / 2.0   # нос   (+2.50)
TIP_R = X_MIN + TIP_DOME_R  # начало хвостового купола (-2.36)
TIP_F = X_MAX - TIP_DOME_F  # начало носового купола (+2.36)

TARGET_COORDS = (0.05, 0.0, 0.72)  # точка прицеливания камер и света

CAR_COLLECTION = 'Car'
STUDIO_COLLECTION = 'Studio'

# --------------------------------------------------------------------------- #
# 1б. Параметры отделяемого багажника, петель и газовых лифтов (Volvo S90)
# --------------------------------------------------------------------------- #
TRUNK_X_FRONT = -1.76         # передняя линия багажника (под покатым задним стеклом S90)
TRUNK_X_REAR = -2.30          # задняя кромка проёма багажника (строго внутри TIP_R=-2.36)
TRUNK_BAY_Z = 0.52            # уровень дна ниши багажника в кузове (выше ZB ~0.20)
TRUNK_LID_THICKNESS = 0.026   # толщина крышки багажника, м
TRUNK_OPEN_DEG = 62.0         # угол открывания крышки вверх вокруг оси Y

TRUNK_HINGE_X = TRUNK_X_FRONT
TRUNK_HINGE_Y = 0.36          # полуширина установки петель «гусиная шея»
TRUNK_LIFT_Y = 0.45           # полуширина установки газовых упоров (лифтов)
TRUNK_LIFT_BODY_XZ = (-1.86, 0.54)  # точка нижнего крепления лифта на стенке кузова
TRUNK_LIFT_LID_ANCHOR_X = -2.08     # X-координата верхнего крепления лифта на крышке
TRUNK_LIFT_CYL_LEN = 0.430          # длина корпуса цилиндра лифта, м
TRUNK_LIFT_ROD_LEN = 0.410          # длина хромированного штока лифта, м


# --------------------------------------------------------------------------- #
# 1в. Параметры дверных проёмов (вырезов кузова), крыши и стоек A / B / C
# --------------------------------------------------------------------------- #
CABIN_CUT_X_FRONT = 0.92      # передний край выреза под двери (стойка A / моторный щит)
CABIN_CUT_X_REAR = -1.06      # задний край выреза под двери (стойка C / заднее крыло)
CABIN_SILL_Z = 0.245          # верхняя кромка порога дверного проёма, м
CABIN_FLOOR_Z = 0.210         # уровень пола салона в нижней ванне кузова, м

ROOF_X_FRONT = 0.16           # передняя кромка крыши (длинный наклон лобового S90)
ROOF_X_REAR = -0.96           # задняя кромка крыши (купеобразный fastback S90)
ROOF_THICKNESS = 0.028        # толщина панели крыши, м

PILLAR_B_X = -0.04            # центр центральной стойки B между дверями
PILLAR_B_HALF_X = 0.036       # полуширина стойки B вдоль X (изящная стойка ~72 мм)
WINDSHIELD_X_BOT = 0.96       # нижняя кромка лобового стекла у капота
REAR_GLASS_X_BOT = -1.76      # нижняя кромка покатого заднего стекла перед багажником


# --------------------------------------------------------------------------- #
# 1г. Параметры отделяемого капота, моторного отсека, петель и газовых лифтов
# --------------------------------------------------------------------------- #
HOOD_X_REAR = 0.98            # задняя кромка капота (линия шарнира у лобового стекла)
HOOD_X_FRONT = 2.30           # передняя кромка проёма капота (строго внутри TIP_F=+2.36)
HOOD_BAY_Z = 0.48             # уровень дна моторного отсека в кузове
HOOD_LID_THICKNESS = 0.026    # толщина крышки капота, м
HOOD_OPEN_DEG = -48.0         # угол открывания капота вверх (отрицат. Y для носа +X)

HOOD_HINGE_X = HOOD_X_REAR
HOOD_HINGE_Y = 0.40           # полуширина установки петель капота
HOOD_LIFT_Y = 0.46            # полуширина установки газовых упоров капота
HOOD_LIFT_BODY_XZ = (1.28, 0.54)  # нижнее крепление лифта на лонжероне/брызговике
HOOD_LIFT_LID_ANCHOR_X = 1.66     # верхнее крепление лифта на капоте
HOOD_LIFT_CYL_LEN = 0.490         # длина корпуса цилиндра лифта капота, м
HOOD_LIFT_ROD_LEN = 0.480         # длина хромированного штока лифта капота, м

# --------------------------------------------------------------------------- #
# 1д. Разделение передней части: моторный щит Body, отдельные крылья и бампер
# --------------------------------------------------------------------------- #
FRONT_CLIP_X = 0.95           # передняя стенка монокока Body (моторный щит / стойка A)
FENDER_X_REAR = 0.928         # задняя кромка переднего крыла (зазор 6 мм к передней двери)
FENDER_X_FRONT = 2.285        # передняя кромка крыла у фары/бампера
FRONT_BUMPER_X = 2.295        # линия стыка капота и крыльев с передним бампером
FRONT_BUMPER_WING_X = 1.995   # передний край колёсной арки, до которого доходит крылышко бампера
FRONT_ARCH_CUT_R = WHEEL_R + 0.036  # радиус реального выреза колёсной арки в переднем крыле
FENDER_THICKNESS = 0.016      # толщина штампованной панели переднего крыла, м
BUMPER_THICKNESS = 0.020      # толщина оболочки переднего бампера, м


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sanitize_name(name):
    """Очищает имя объекта/материала под требования Unreal Engine
    (только латиница, цифры и подчёркивания, без ведущей цифры)."""
    s = re.sub(r'[^A-Za-z0-9_]', '_', str(name))
    s = re.sub(r'_+', '_', s).strip('_')
    if not s:
        return 'obj'
    if s[0].isdigit():
        return 'obj_' + s
    return s


# --------------------------------------------------------------------------- #
# 2. Ключевые кривые вдоль оси X (-2.50 корма ... +2.50 нос)
# --------------------------------------------------------------------------- #
def _norm_x(x):
    """Нормированная координата t в [0, 1]: 0 — корма (-2.5), 1 — нос (+2.5)."""
    return clamp((x - X_MIN) / (X_MAX - X_MIN), 0.0, 1.0)


def W(x):
    """Полуширина борта Volvo S90 (максимум ~0.936 м -> ширина 1.872 м),
    с мускулистыми бёдрами над колёсами и плавным сужением к носу и корме."""
    t = _norm_x(x)
    # Базовое скругление в плане + лёгкое расширение в зоне задних и передних арок
    plan_taper = 1.0 - 0.14 * (abs(2.0 * t - 1.0) ** 2.1)
    # Дополнительное сужение самого носа (+X > 1.85) и кормы (-X < -1.95)
    nose_taper = 0.09 * (clamp((x - 1.75) / (X_MAX - 1.75), 0.0, 1.0) ** 2.0)
    tail_taper = 0.07 * (clamp((-1.85 - x) / (-1.85 - X_MIN), 0.0, 1.0) ** 2.0)
    base = (WIDTH / 2.0) * (plan_taper - nose_taper - tail_taper)
    if x > TIP_F:
        u = clamp((x - TIP_F) / TIP_DOME_F, 0.0, 1.0)
        base *= max(0.08, dome_scale(u))
    elif x < TIP_R:
        u = clamp((TIP_R - x) / TIP_DOME_R, 0.0, 1.0)
        base *= max(0.08, dome_scale(u))
    return clamp(base, 0.06, WIDTH / 2.0)


def ZB(x):
    """Нижняя кромка кузова (днище/порог), всегда выше дороги Z = 0."""
    t = _norm_x(x)
    # Низкий ровный порог 0.16 м с аккуратным подъёмом спереди (~0.195) и сзади (~0.210)
    overhang_lift = 0.035 * (abs(2.0 * t - 1.0) ** 3.0)
    rear_diff = 0.018 * (clamp((-2.00 - x) / (-2.00 - X_MIN), 0.0, 1.0) ** 2.0)
    return GROUND_CLEARANCE + overhang_lift + rear_diff


def ZR(x):
    """Верхняя линия силуэта Volvo S90: низкая купеобразная дуга крыши (пик HEIGHT = 1.432),
    длинный низкий покатый капот (+X) и стремительный фастбэк-хвост (-X)."""
    if ROOF_X_REAR <= x <= ROOF_X_FRONT:
        # Плавный купол крыши с максимумом HEIGHT в районе x = -0.34
        u = (x - ROOF_X_REAR) / (ROOF_X_FRONT - ROOF_X_REAR)
        return 1.404 + 0.018 * u + 0.019 * math.sin(math.pi * u)
    if x > ROOF_X_FRONT:
        z_rf = 1.422
        z_cowl = 0.980
        if x <= WINDSHIELD_X_BOT:
            u = (x - ROOF_X_FRONT) / (WINDSHIELD_X_BOT - ROOF_X_FRONT)
            e = u * u * (3.0 - 2.0 * u)
            return z_rf * (1.0 - e) + z_cowl * e
        u = clamp((x - WINDSHIELD_X_BOT) / (X_MAX - WINDSHIELD_X_BOT), 0.0, 1.0)
        return z_cowl - 0.165 * (u ** 1.45)
    # Покатое фастбэк заднее стекло (ROOF_X_REAR -> REAR_GLASS_X_BOT) и спортивный багажник S90
    z_rr = 1.404
    z_rdeck = 1.026
    if x >= REAR_GLASS_X_BOT:
        u = (ROOF_X_REAR - x) / (ROOF_X_REAR - REAR_GLASS_X_BOT)
        e = u * u * (3.0 - 2.0 * u)
        return z_rr * (1.0 - e) + z_rdeck * e
    u = clamp((REAR_GLASS_X_BOT - x) / (REAR_GLASS_X_BOT - X_MIN), 0.0, 1.0)
    return z_rdeck - 0.068 * (u ** 1.40)


def BELT(x):
    """Высокая плечевая линия Volvo S90: уверенно восходящая к корме (~0.98),
    плавно ниспадающая по длинному капоту к фарам (~0.78 у передней кромки)."""
    b = 0.966 - 0.025 * clamp((x + 0.5) / 2.0, -0.8, 1.0)
    if x > WINDSHIELD_X_BOT:
        t = clamp((x - WINDSHIELD_X_BOT) / (X_MAX - WINDSHIELD_X_BOT), 0.0, 1.0)
        b -= 0.160 * (t ** 1.50)
    elif x < REAR_GLASS_X_BOT:
        t = clamp((REAR_GLASS_X_BOT - x) / (REAR_GLASS_X_BOT - X_MIN), 0.0, 1.0)
        b -= 0.055 * (t ** 1.55)
    return min(b, ZR(x) - 0.028)


def side_surface_y(x, z):
    """Единая скульптурная кривая наружной поверхности борта Y(x, z) для крыльев и дверей.
    Гарантирует бесшовный переход переднее крыло -> двери -> заднее крыло с выраженной
    плечевой гранью Volvo и нижним поднутрением порога."""
    w = W(x)
    z0 = ZB(x)
    belt = BELT(x)
    v = clamp((z - z0) / max(belt - z0, 0.10), 0.0, 1.0)
    # Скульптурный профиль: поднутрение снизу (0.91), лёгкая выемка в нижней трети,
    # полная «мускулистая» середина (1.00) и выраженная плечевая грань у пояса (0.975)
    tuck_bot = 0.91 + 0.09 * math.sin(0.5 * math.pi * clamp(v / 0.45, 0.0, 1.0))
    cove = (
        -0.014 * math.sin(math.pi * clamp((v - 0.12) / 0.36, 0.0, 1.0))
        if 0.12 < v < 0.48 else 0.0
    )
    shoulder = -0.025 * (max(0.0, (v - 0.78) / 0.22) ** 1.6)
    return w * (tuck_bot + cove + shoulder)


def WR(x):
    """Полуширина верхней кромки (крыши/капота/багажника)."""
    t = _norm_x(x)
    return W(x) * (0.63 + 0.05 * math.sin(math.pi * t))


def TUMBLE(x):
    """Наклон остекления к вертикали (tumblehome): 0.06 … 0.16."""
    t = _norm_x(x)
    return 0.06 + 0.10 * t * t


def arch_mask(x, z):
    """Поле впадины колёсной арки: 0 — нет впадины, 1 — центр впадины у оси колеса.
    Центрировано строго по реальным осям AXLE_R и AXLE_F."""
    m = 0.0
    for cx in (AXLE_R, AXLE_F):
        dx = clamp(1.0 - abs(x - cx) / ARCH_R, 0.0, 1.0)
        dz = clamp(1.0 - abs(z - WHEEL_Z) / 0.28, 0.0, 1.0)
        m = max(m, (dx * dz) ** 1.5)
    return m


def dome_scale(u, pe=0.40, px=0.76):
    """Масштаб поперечного сечения на торцевом куполе по суперэллипсу.
    u = 0 -> 1.0 (стык с основным кузовом), u = 1 -> 0.0 (вершина купола)."""
    u = clamp(u, 0.0, 1.0)
    inner = max(0.0, 1.0 - (u ** (1.0 / px)))
    return inner ** pe


def cabin_cut_mask(x, transition=0.036):
    """Маска сквозного выреза дверных проёмов и салона в нижней ванне кузова:
    1.0 внутри [CABIN_CUT_X_REAR, CABIN_CUT_X_FRONT], 0.0 снаружи."""
    if x <= CABIN_CUT_X_REAR or x >= CABIN_CUT_X_FRONT:
        return 0.0
    edge_r = clamp((x - CABIN_CUT_X_REAR) / transition, 0.0, 1.0)
    edge_f = clamp((CABIN_CUT_X_FRONT - x) / transition, 0.0, 1.0)
    s = min(edge_r, edge_f)
    return s * s * (3.0 - 2.0 * s)


def greenhouse_mask(x, transition=0.05):
    """Маска зоны остекления (лобовое — салон — заднее стекло), где нижняя ванна кузова
    не поднимается до крыши ZR(x), а плавно переходит в линию капота и багажника."""
    x_lo = REAR_GLASS_X_BOT
    x_hi = WINDSHIELD_X_BOT
    if x <= x_lo or x >= x_hi:
        return 0.0
    edge_r = clamp((x - x_lo) / transition, 0.0, 1.0)
    edge_f = clamp((x_hi - x) / transition, 0.0, 1.0)
    s = min(edge_r, edge_f)
    return s * s * (3.0 - 2.0 * s)


def upper_side_yz(x, t):
    """Внешняя координата (y, z) наклонной боковины остекления/стойки (t=0 пояс, t=1 крыша)."""
    t = clamp(t, 0.0, 1.0)
    w_bot = side_surface_y(x, BELT(x))
    w_top = WR(x)
    # Плавная купеобразная выпуклость бокового стекла/стойки (tumblehome S90)
    y = w_bot * (1.0 - t) + w_top * t + 0.026 * math.sin(math.pi * t)
    z_bot = BELT(x)
    z_top = ZR(x) - 0.016
    z = z_bot + (z_top - z_bot) * t
    return y, z


def hood_bay_mask(x, transition=0.04):
    """Маска моторного отсека в кузове: 1.0 внутри [HOOD_X_REAR, HOOD_X_FRONT],
    0.0 снаружи, с крутым плавным переходом на кромках."""
    if x <= HOOD_X_REAR or x >= HOOD_X_FRONT:
        return 0.0
    edge_r = clamp((x - HOOD_X_REAR) / transition, 0.0, 1.0)
    edge_f = clamp((HOOD_X_FRONT - x) / transition, 0.0, 1.0)
    s = min(edge_r, edge_f)
    return s * s * (3.0 - 2.0 * s)


def hood_deck_z(x, y):
    """Высота внешней поверхности капота в точке (x, y) с плавной выштамповкой по центру,
    затухающей у кромок (без ступеньки у лобового стекла и решётки)."""
    wr = max(WR(x), 0.1)
    u = clamp(abs(y) / wr, 0.0, 1.0)
    ux = clamp((x - HOOD_X_REAR) / max(HOOD_X_FRONT - HOOD_X_REAR, 0.1), 0.0, 1.0)
    power_dome = 0.008 * math.sin(math.pi * ux) * max(0.0, 1.0 - (u / 0.55) ** 2)
    return ZR(x) - 0.014 * (u * u) + power_dome


def hood_hinge_z():
    """Z-координата оси петель капота (у задней кромки капота)."""
    return ZR(HOOD_HINGE_X) - 0.045


def hood_lift_lid_local_xz():
    """Локальные координаты (lx, lz) верхнего крепления газового лифта на капоте
    относительно оси петли (HOOD_HINGE_X, hood_hinge_z())."""
    hz = hood_hinge_z()
    lx = HOOD_LIFT_LID_ANCHOR_X - HOOD_HINGE_X
    lz = (hood_deck_z(HOOD_LIFT_LID_ANCHOR_X, HOOD_LIFT_Y) - HOOD_LID_THICKNESS) - hz
    return lx, lz


def hood_lift_endpoints(open_deg):
    """Возвращает ((bx, bz), (wx, wz), dist) — мировые XZ-координаты нижней и верхней
    точек крепления газового лифта капота и расстояние между ними при угле open_deg."""
    bx, bz = HOOD_LIFT_BODY_XZ
    hz = hood_hinge_z()
    lx, lz = hood_lift_lid_local_xz()
    rad = math.radians(open_deg)
    wx = HOOD_HINGE_X + lx * math.cos(rad) + lz * math.sin(rad)
    wz = hz - lx * math.sin(rad) + lz * math.cos(rad)
    dist = math.hypot(wx - bx, wz - bz)
    return (bx, bz), (wx, wz), dist


def fender_bumper_seam_z(x):
    """Высота наклонного стыка переднего крыла с передним бампером на участке
    [FRONT_BUMPER_WING_X .. FENDER_X_FRONT]: от передней кромки арки вверх к фаре."""
    u = clamp((x - FRONT_BUMPER_WING_X) / max(FENDER_X_FRONT - FRONT_BUMPER_WING_X, 0.05),
              0.0, 1.0)
    return 0.50 + 0.21 * (u ** 0.85)


def fender_bottom_z(x):
    """Нижняя кромка отдельного переднего крыла с учётом реального выреза колёсной арки
    вокруг (AXLE_F, WHEEL_Z) и диагонального стыка с передним бампером."""
    # За передним колесом точно выходит в ровную линию порога под передней дверью (без ступеньки)
    z_sill = ZB(x) + 0.052
    dx = abs(x - AXLE_F)
    if dx < FRONT_ARCH_CUT_R - 1e-4:
        z_arch = WHEEL_Z + math.sqrt(max(0.0, FRONT_ARCH_CUT_R * FRONT_ARCH_CUT_R - dx * dx))
        z_lo = max(z_sill, z_arch)
    else:
        z_lo = z_sill
    if x >= FRONT_BUMPER_WING_X:
        z_lo = max(z_lo, fender_bumper_seam_z(x))
    return min(z_lo, BELT(x) - 0.05)


def trunk_bay_mask(x, transition=0.04):
    """Маска ниши багажника в кузове: 1.0 внутри проёма [TRUNK_X_REAR, TRUNK_X_FRONT],
    0.0 снаружи, с крутым плавным переходом на кромках."""
    if x <= TRUNK_X_REAR or x >= TRUNK_X_FRONT:
        return 0.0
    edge_r = clamp((x - TRUNK_X_REAR) / transition, 0.0, 1.0)
    edge_f = clamp((TRUNK_X_FRONT - x) / transition, 0.0, 1.0)
    s = min(edge_r, edge_f)
    return s * s * (3.0 - 2.0 * s)


def trunk_deck_z(x, y):
    """Высота внешней поверхности крышки багажника в точке (x, y)."""
    wr = max(WR(x), 0.1)
    u = clamp(abs(y) / wr, 0.0, 1.0)
    return ZR(x) - 0.012 * (u * u)


def trunk_hinge_z():
    """Z-координата оси петель крышки багажника."""
    return ZR(TRUNK_HINGE_X) - 0.045


def trunk_lift_lid_local_xz():
    """Локальные координаты (lx, lz) точки крепления газового лифта на крышке
    относительно оси петли (TRUNK_HINGE_X, trunk_hinge_z())."""
    hz = trunk_hinge_z()
    lx = TRUNK_LIFT_LID_ANCHOR_X - TRUNK_HINGE_X
    lz = (trunk_deck_z(TRUNK_LIFT_LID_ANCHOR_X, TRUNK_LIFT_Y) - TRUNK_LID_THICKNESS) - hz
    return lx, lz


def trunk_lift_endpoints(open_deg):
    """Возвращает ((bx, bz), (wx, wz), dist) — мировые XZ-координаты нижней и верхней
    точек крепления газового лифта и расстояние между ними при угле open_deg."""
    bx, bz = TRUNK_LIFT_BODY_XZ
    hz = trunk_hinge_z()
    lx, lz = trunk_lift_lid_local_xz()
    rad = math.radians(open_deg)
    # Правое вращение вокруг оси +Y: (x, z) -> (x*cos + z*sin, -x*sin + z*cos)
    wx = TRUNK_HINGE_X + lx * math.cos(rad) + lz * math.sin(rad)
    wz = hz - lx * math.sin(rad) + lz * math.cos(rad)
    dist = math.hypot(wx - bx, wz - bz)
    return (bx, bz), (wx, wz), dist


# --------------------------------------------------------------------------- #
# 3. Поперечные сечения кузова и остекления
# --------------------------------------------------------------------------- #
def body_half_profile(x):
    """Левый борт нижней ванны кузова (y >= 0): 10 точек от центра дна до оси симметрии.

    Учитывает три конструктивные зоны:
      * дверной вырез салона (CABIN_CUT_X_REAR..CABIN_CUT_X_FRONT): опускается до порога
        CABIN_SILL_Z и пола салона CABIN_FLOOR_Z — проёмы дверей полностью сквозные;
      * пояс остекления (REAR_GLASS_X_BOT..WINDSHIELD_X_BOT): верх кузова ограничен
        уровнем торпедо/задней полки, а крыша Roof соединяется с кузовом стойками A/B/C;
      * ниша багажника (TRUNK_X_REAR..TRUNK_X_FRONT): углублена до TRUNK_BAY_Z."""
    z0 = ZB(x)
    w = W(x)
    belt = BELT(x)
    zr = ZR(x)
    wr = WR(x)
    tum = TUMBLE(x)

    z_arch_lo = max(z0 + 0.08, WHEEL_Z - 0.02)
    z_arch_hi = min(belt - 0.12, WHEEL_Z + 0.14)
    z_mid = 0.5 * (z_arch_hi + belt)

    r_lo = arch_mask(x, z_arch_lo)
    r_hi = arch_mask(x, z_arch_hi)
    recess_frac = ARCH_DEPTH / max(w, 0.25)

    # В зоне остекления верх нижней ванны кузова ограничен линией деки (без горбиков на стыках)
    gh = greenhouse_mask(x)
    z_deck = belt + 0.045
    zr_body = zr * (1.0 - gh) + min(zr, z_deck) * gh

    z_glass_mid = belt + 0.52 * max(0.02, zr_body - belt)
    y_glass_mid = max(wr, w - tum * 0.42)

    # В зонах багажника и моторного отсека опускаем точки 8 и 9 вглубь кузова,
    # оставляя точку 7 на уровне гребней задних и передних крыльев.
    tb = trunk_bay_mask(x)
    hb = hood_bay_mask(x)
    bay = max(tb, hb)
    bay_z = TRUNK_BAY_Z if tb >= hb else HOOD_BAY_Z
    y7 = y_glass_mid * (1.0 - bay) + (wr + 0.04) * bay
    z7 = z_glass_mid * (1.0 - bay) + (zr_body - 0.015) * bay
    y8 = wr * (1.0 - bay) + (wr - 0.02) * bay
    z8 = (zr_body - 0.01) * (1.0 - bay) + bay_z * bay
    z9 = zr_body * (1.0 - bay) + bay_z * bay

    y_lo = side_surface_y(x, z_arch_lo) * (1.0 - recess_frac * r_lo)
    y_hi = side_surface_y(x, z_arch_hi) * (1.0 - 0.72 * recess_frac * r_hi)
    y_mid = side_surface_y(x, z_mid)
    y_sh = side_surface_y(x, belt - 0.03)
    y_belt = side_surface_y(x, belt)

    base_pts = [
        (0.0, z0),                                                      # 0 центр дна
        (w * 0.58, z0 + 0.018),                                         # 1 нижний порог
        (y_lo, z_arch_lo),                                              # 2 арка низ
        (y_hi, z_arch_hi),                                              # 3 арка верх
        (y_mid, z_mid),                                                 # 4 скульптура борта
        (y_sh, belt - 0.03),                                            # 5 плечевая грань
        (y_belt, belt),                                                 # 6 подоконник
        (y7, z7),                                                       # 7 крыло / кромка
        (y8, z8),                                                       # 8 край деки / ниши
        (0.0, z9),                                                      # 9 центр деки / ниши
    ]

    cc = cabin_cut_mask(x, transition=0.026)
    if cc <= 0.0:
        return base_pts

    # Профиль порога (rocker sill) и пола салона внутри сквозного дверного выреза
    y_sill_bot = side_surface_y(x, z0 + 0.05)
    y_sill_top = side_surface_y(x, CABIN_SILL_Z)
    cut_pts = [
        (0.0, z0),                                                      # 0 центр дна
        (w * 0.58, z0 + 0.018),                                         # 1 низ порога
        (y_sill_bot, z0 + 0.05),                                        # 2 нижний край порога
        (y_sill_top, CABIN_SILL_Z - 0.016),                             # 3 наружный угол порога
        (y_sill_top - 0.025, CABIN_SILL_Z),                             # 4 верх порога наруж.
        (w * 0.78, CABIN_SILL_Z),                                       # 5 верх порога внутр.
        (w * 0.72, CABIN_FLOOR_Z + 0.012),                              # 6 ступенька в салон
        (w * 0.52, CABIN_FLOOR_Z),                                      # 7 пол салона
        (w * 0.26, CABIN_FLOOR_Z),                                      # 8 пол салона
        (0.0, CABIN_FLOOR_Z),                                           # 9 центр пола салона
    ]
    return [
        (by * (1.0 - cc) + cy * cc, bz * (1.0 - cc) + cz * cc)
        for (by, bz), (cy, cz) in zip(base_pts, cut_pts)
    ]


def body_section(x):
    """Полный замкнутый контур сечения: 18 точек (левый борт вверх, правый вниз)."""
    hp = body_half_profile(x)
    verts = [(y, z) for (y, z) in hp]                  # 0..9
    verts += [(-y, z) for (y, z) in reversed(hp[1:9])]  # 10..17
    return verts


def glass_half_profile(x):
    """Профиль остекления (левый борт): от подоконника до кромки крыши."""
    w = W(x)
    belt = BELT(x)
    zr = ZR(x)
    wr = WR(x)
    dz = max(0.04, zr - belt)
    # Слегка выносим стекло наружу на 6 мм, чтобы избежать z-fighting с кузовом
    off = 0.006
    return [
        (w + off, belt),
        (w * 0.92 + wr * 0.08 + off, belt + 0.30 * dz),
        (w * 0.72 + wr * 0.28 + off, belt + 0.62 * dz),
        (w * 0.35 + wr * 0.65 + off, belt + 0.88 * dz),
        (wr + off, zr - 0.008),
    ]


def glass_section(x):
    """Замкнутый профиль фонаря остекления в сечении (9 точек)."""
    hp = glass_half_profile(x)
    zr = ZR(x)
    # 0..4 левый борт снизу вверх, 5 центр крыши, 6..8 правый борт сверху вниз
    # (для совместимости с V_GH = 9 замыкаем через центр крыши)
    return (
        [(y, z) for (y, z) in hp[:4]]
        + [(0.0, zr + 0.004)]
        + [(-y, z) for (y, z) in reversed(hp[:4])]
    )


# --------------------------------------------------------------------------- #
# 4. Спецификация дверей (шарнир у передней кромки каждой двери, открывание наружу)
# --------------------------------------------------------------------------- #
# Каждая дверь в локальных координатах тянется от x = 0 (шарнир) назад к x = -door_len.
# По правилу правой руки вращение вокруг +Z вектора (-1, 0, 0) на угол theta
# даёт (-cos(theta), -sin(theta), 0):
#   * Левая дверь (side = +1, +Y): для выхода наружу (+Y) нужно -sin(theta) > 0 -> theta = -70°.
#   * Правая дверь (side = -1, -Y): для выхода наружу (-Y) нужно -sin(theta) < 0 -> theta = +70°.
DOORS_SPEC = [
    # (имя, hinge_x, side_sign, door_length, open_angle_deg)
    # Передние двери: закрывают проём от стойки A (+0.922) до шва у стойки B (-0.034)
    ('Door_Front_Left', 0.922, +1.0, 0.956, -70.0),
    ('Door_Front_Right', 0.922, -1.0, 0.956, +70.0),
    # Задние двери: закрывают проём от шва у стойки B (-0.044) до стойки C (-1.062)
    ('Door_Rear_Left', -0.044, +1.0, 1.018, -70.0),
    ('Door_Rear_Right', -0.044, -1.0, 1.018, +70.0),
]


def door_swing_tip_y(side_sign, door_length, open_angle_deg, hinge_y):
    """Возвращает мировую Y-координату свободного края двери при открывании на open_angle_deg."""
    rad = math.radians(open_angle_deg)
    # Локальная точка (-door_length, 0) после поворота вокруг Z
    dy = -door_length * math.sin(rad)
    return hinge_y + dy
