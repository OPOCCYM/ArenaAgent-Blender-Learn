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
HEIGHT = 1.456       # высота крыши от дороги, м
GROUND_CLEARANCE = 0.16  # минимальный дорожный просвет под порогом, м

AXLE_F = 1.60        # передняя ось (+X)
AXLE_R = -1.46       # задняя ось (-X), база 3.06 м
WHEEL_Y = 0.792      # половина колеи (колея 1.584 м)
WHEEL_R = 0.353      # радиус шины 235/45 R18
WHEEL_Z = 0.353      # центр качения колеса (шина касается Z = 0.0)
WHEEL_WIDTH = 0.235  # ширина шины, м

ARCH_R = 0.445       # радиус зоны колёсной арки
ARCH_DEPTH = 0.13    # глубина впадины борта у колеса
TIP_DOME_F = 0.15    # длина купола носа (+X)
TIP_DOME_R = 0.17    # длина купола кормы (-X)

N_SECTIONS = 182     # число поперечных сечений вдоль X
V_BODY = 18          # число точек профиля кузова в сечении
V_GH = 9             # число точек профиля остекления

X_MIN = -LENGTH / 2.0  # корма (-2.50)
X_MAX = LENGTH / 2.0   # нос   (+2.50)
TIP_R = X_MIN + TIP_DOME_R  # начало хвостового купола (-2.33)
TIP_F = X_MAX - TIP_DOME_F  # начало носового купола (+2.35)

TARGET_COORDS = (0.05, 0.0, 0.74)  # точка прицеливания камер и света

CAR_COLLECTION = 'Car'
STUDIO_COLLECTION = 'Studio'

# --------------------------------------------------------------------------- #
# 1б. Параметры отделяемого багажника, петель и газовых лифтов (упоров)
# --------------------------------------------------------------------------- #
TRUNK_X_FRONT = -1.38         # передняя линия проёма багажника (линия шарнира)
TRUNK_X_REAR = -2.28          # задняя кромка проёма багажника
TRUNK_BAY_Z = 0.52            # уровень дна ниши багажника в кузове (выше ZB ~0.20)
TRUNK_LID_THICKNESS = 0.028   # толщина крышки багажника, м
TRUNK_OPEN_DEG = 62.0         # угол открывания крышки вверх вокруг оси Y

TRUNK_HINGE_X = TRUNK_X_FRONT
TRUNK_HINGE_Y = 0.38          # полуширина установки петель «гусиная шея»
TRUNK_LIFT_Y = 0.49           # полуширина установки газовых упоров (лифтов)
TRUNK_LIFT_BODY_XZ = (-1.56, 0.62)  # точка нижнего крепления лифта на стенке кузова
TRUNK_LIFT_LID_ANCHOR_X = -1.86     # X-координата верхнего крепления лифта на крышке
TRUNK_LIFT_CYL_LEN = 0.475          # длина корпуса цилиндра лифта, м
TRUNK_LIFT_ROD_LEN = 0.465          # длина хромированного штока лифта, м


# --------------------------------------------------------------------------- #
# 1в. Параметры дверных проёмов (вырезов кузова), крыши и стоек A / B / C
# --------------------------------------------------------------------------- #
CABIN_CUT_X_FRONT = 0.86      # передний край выреза под двери (стойка A / моторный щит)
CABIN_CUT_X_REAR = -0.96      # задний край выреза под двери (стойка C / заднее крыло)
CABIN_SILL_Z = 0.265          # верхняя кромка порога дверного проёма, м
CABIN_FLOOR_Z = 0.220         # уровень пола салона в нижней ванне кузова, м

ROOF_X_FRONT = 0.44           # передняя кромка панели крыши (верх стойки A / лобового)
ROOF_X_REAR = -0.96           # задняя кромка панели крыши (верх стойки C / заднего стекла)
ROOF_THICKNESS = 0.032        # толщина панели крыши, м

PILLAR_B_X = -0.09            # центр центральной стойки B между дверями
PILLAR_B_HALF_X = 0.048       # полуширина стойки B вдоль X (ширина ~96 мм)
WINDSHIELD_X_BOT = 0.92       # нижняя кромка лобового стекла у капота
REAR_GLASS_X_BOT = -1.34      # нижняя кромка заднего стекла перед багажником


# --------------------------------------------------------------------------- #
# 1г. Параметры отделяемого капота, моторного отсека, петель и газовых лифтов
# --------------------------------------------------------------------------- #
HOOD_X_REAR = 1.04            # задняя кромка капота (линия шарнира у лобового стекла)
HOOD_X_FRONT = 2.26           # передняя кромка проёма капота у решётки радиатора
HOOD_BAY_Z = 0.50             # уровень дна моторного отсека в кузове
HOOD_LID_THICKNESS = 0.028    # толщина крышки капота, м
HOOD_OPEN_DEG = -48.0         # угол открывания капота вверх (отрицат. Y для носа +X)

HOOD_HINGE_X = HOOD_X_REAR
HOOD_HINGE_Y = 0.42           # полуширина установки петель капота
HOOD_LIFT_Y = 0.48            # полуширина установки газовых упоров капота
HOOD_LIFT_BODY_XZ = (1.30, 0.56)  # нижнее крепление лифта на лонжероне/брызговике
HOOD_LIFT_LID_ANCHOR_X = 1.66     # верхнее крепление лифта на капоте
HOOD_LIFT_CYL_LEN = 0.480         # длина корпуса цилиндра лифта капота, м
HOOD_LIFT_ROD_LEN = 0.470         # длина хромированного штока лифта капота, м


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
    """Полуширина борта, м (максимум ~0.936 м -> ширина 1.872 м)."""
    t = _norm_x(x)
    base = (WIDTH / 2.0) * (1.0 - 0.16 * abs(2.0 * t - 1.0) ** 1.8)
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
    # В базе — ровный порог ~0.16 м, к переднему и заднему свесу — лёгкий подъём бамперов
    overhang_lift = 0.09 * (abs(2.0 * t - 1.0) ** 2.4)
    return GROUND_CLEARANCE + overhang_lift


def BELT(x):
    """Подоконная линия (beltline), м."""
    t = _norm_x(x)
    # Слегка выше к корме (0.98..1.04) и плавно спускается к носу (~0.80)
    return 1.01 - 0.12 * abs(2.0 * t - 1.0) ** 1.4 - 0.06 * (t - 0.5)


def ZR(x):
    """Верхняя линия силуэта (крыша, капот, багажник), пик HEIGHT = 1.456 м."""
    # Трёхобъёмный седан: крыша в зоне салона x ∈ [-1.15, 0.65],
    # плавный переход на капот (+X) и багажник (-X).
    if -0.95 <= x <= 0.45:
        # Купол крыши с максимумом HEIGHT в районе x = -0.20
        s = (x + 0.20) / 0.75
        return HEIGHT - 0.025 * (s * s)
    if x > 0.45:
        # Лобовое стекло -> капот -> нос
        u = clamp((x - 0.45) / (X_MAX - 0.45), 0.0, 1.0)
        # Быстрый спад лобового до капота (u ~ 0.35), затем пологий капот
        windshield = 0.42 * (1.0 - math.exp(-5.0 * u))
        hood_drop = 0.24 * (u ** 1.6)
        return max(BELT(x) + 0.03, HEIGHT - 0.02 - windshield - hood_drop)
    # Заднее стекло -> багажник -> корма (x < -0.95)
    u = clamp((-0.95 - x) / (-0.95 - X_MIN), 0.0, 1.0)
    rear_glass = 0.36 * (1.0 - math.exp(-4.5 * u))
    trunk_drop = 0.12 * (u ** 1.8)
    return max(BELT(x) + 0.03, HEIGHT - 0.025 - rear_glass - trunk_drop)


def WR(x):
    """Полуширина верхней кромки (крыши/капота/багажника)."""
    t = _norm_x(x)
    return W(x) * (0.62 + 0.06 * math.sin(math.pi * t))


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


def dome_scale(u, pe=0.34, px=0.72):
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


def greenhouse_mask(x, transition=0.08):
    """Маска зоны остекления (лобовое — салон — заднее стекло), где нижняя ванна кузова
    не поднимается до крыши ZR(x), а ограничивается поясной линией (крышу держат стойки)."""
    x_lo = REAR_GLASS_X_BOT - 0.02
    x_hi = WINDSHIELD_X_BOT + 0.10
    if x <= x_lo or x >= x_hi:
        return 0.0
    edge_r = clamp((x - x_lo) / transition, 0.0, 1.0)
    edge_f = clamp((x_hi - x) / transition, 0.0, 1.0)
    s = min(edge_r, edge_f)
    return s * s * (3.0 - 2.0 * s)


def upper_side_yz(x, t):
    """Внешняя координата (y, z) наклонной боковины остекления/стойки (t=0 пояс, t=1 крыша)."""
    t = clamp(t, 0.0, 1.0)
    w_bot = W(x) * 0.985
    w_top = WR(x)
    # Лёгкая выпуклость бокового стекла/стойки (tumblehome)
    y = w_bot * (1.0 - t) + w_top * t + 0.022 * math.sin(math.pi * t)
    z_bot = BELT(x)
    z_top = ZR(x) - 0.014
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
    """Высота внешней поверхности капота в точке (x, y) с лёгкой выштамповкой по центру."""
    wr = max(WR(x), 0.1)
    u = clamp(abs(y) / wr, 0.0, 1.0)
    # Плавный купол капота + лёгкий рельеф по центру
    power_dome = 0.010 * max(0.0, 1.0 - (u / 0.55) ** 2)
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

    # В зоне остекления верх нижней ванны кузова ограничен уровнем поясной линии/торпедо
    gh = greenhouse_mask(x)
    z_deck = belt + 0.055
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

    base_pts = [
        (0.0, z0),                                                      # 0 центр дна
        (w * 0.55, z0 + 0.02),                                          # 1 нижний порог
        (w * (1.0 - recess_frac * r_lo), z_arch_lo),                    # 2 арка низ
        (w * (1.0 - 0.65 * recess_frac * r_hi), z_arch_hi),             # 3 арка верх
        (w * 0.94, z_mid),                                              # 4 над аркой
        (w * 0.985, belt - 0.025),                                      # 5 плечевая линия
        (w, belt),                                                      # 6 подоконник
        (y7, z7),                                                       # 7 крыло / кромка
        (y8, z8),                                                       # 8 край деки / ниши
        (0.0, z9),                                                      # 9 центр деки / ниши
    ]

    cc = cabin_cut_mask(x)
    if cc <= 0.0:
        return base_pts

    # Профиль порога (rocker sill) и пола салона внутри сквозного дверного выреза
    cut_pts = [
        (0.0, z0),                                                      # 0 центр дна
        (w * 0.55, z0 + 0.02),                                          # 1 низ порога
        (w * 0.95, z0 + 0.055),                                         # 2 нижний край порога
        (w * 0.97, CABIN_SILL_Z - 0.020),                               # 3 наружный угол порога
        (w * 0.93, CABIN_SILL_Z),                                       # 4 верх порога наруж.
        (w * 0.80, CABIN_SILL_Z),                                       # 5 верх порога внутр.
        (w * 0.74, CABIN_FLOOR_Z + 0.012),                              # 6 ступенька в салон
        (w * 0.54, CABIN_FLOOR_Z),                                      # 7 пол салона
        (w * 0.28, CABIN_FLOOR_Z),                                      # 8 пол салона
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
    # Передние двери: закрывают проём от стойки A (+0.855) до середины стойки B (-0.080)
    ('Door_Front_Left', 0.855, +1.0, 0.935, -70.0),
    ('Door_Front_Right', 0.855, -1.0, 0.935, +70.0),
    # Задние двери: закрывают проём от стойки B (-0.098) до стойки C (-0.955)
    ('Door_Rear_Left', -0.098, +1.0, 0.857, -70.0),
    ('Door_Rear_Right', -0.098, -1.0, 0.857, +70.0),
]


def door_swing_tip_y(side_sign, door_length, open_angle_deg, hinge_y):
    """Возвращает мировую Y-координату свободного края двери при открывании на open_angle_deg."""
    rad = math.radians(open_angle_deg)
    # Локальная точка (-door_length, 0) после поворота вокруг Z
    dy = -door_length * math.sin(rad)
    return hinge_y + dy
