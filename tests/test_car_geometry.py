#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Чистые Python-тесты геометрии, кривых и санитизации имён (не требуют bpy)."""
import math

import pytest

from car_geometry import (
    AXLE_F,
    AXLE_R,
    CABIN_CUT_X_FRONT,
    CABIN_CUT_X_REAR,
    CABIN_FLOOR_Z,
    CABIN_SILL_Z,
    DOORS_SPEC,
    GROUND_CLEARANCE,
    HEIGHT,
    HOOD_BAY_Z,
    HOOD_LIFT_CYL_LEN,
    HOOD_LIFT_ROD_LEN,
    HOOD_OPEN_DEG,
    HOOD_X_FRONT,
    HOOD_X_REAR,
    LENGTH,
    N_SECTIONS,
    TIP_F,
    TIP_R,
    TRUNK_BAY_Z,
    TRUNK_LIFT_CYL_LEN,
    TRUNK_LIFT_ROD_LEN,
    TRUNK_OPEN_DEG,
    TRUNK_X_FRONT,
    TRUNK_X_REAR,
    V_BODY,
    V_GH,
    WHEEL_Y,
    WHEEL_Z,
    WIDTH,
    X_MAX,
    X_MIN,
    BELT,
    TUMBLE,
    W,
    WR,
    ZB,
    ZR,
    arch_mask,
    body_half_profile,
    body_section,
    cabin_cut_mask,
    dome_scale,
    door_swing_tip_y,
    glass_section,
    hood_bay_mask,
    hood_lift_endpoints,
    sanitize_name,
    trunk_bay_mask,
    trunk_lift_endpoints,
)


def test_coordinate_convention_front_positive_x():
    """Передняя ось и носовой купол должны находиться в +X, задние — в -X."""
    assert X_MAX == pytest.approx(LENGTH / 2.0)
    assert X_MIN == pytest.approx(-LENGTH / 2.0)
    assert AXLE_F > 0.0
    assert AXLE_R < 0.0
    assert TIP_F > 0.0
    assert TIP_R < 0.0
    assert AXLE_F - AXLE_R == pytest.approx(3.06, abs=1e-6)


def test_ground_clearance_always_positive():
    """Днище кузова ZB(x) нигде не должно опускаться под дорогу Z = 0."""
    xs = [X_MIN + (X_MAX - X_MIN) * i / 100 for i in range(101)]
    for x in xs:
        assert ZB(x) >= GROUND_CLEARANCE - 1e-9
        assert ZB(x) > 0.0


def test_roof_and_belt_ordering_and_height():
    """Крыша всегда выше подоконника, подоконник выше днища; пик крыши равен HEIGHT."""
    xs = [X_MIN + (X_MAX - X_MIN) * i / 100 for i in range(101)]
    for x in xs:
        assert ZB(x) < BELT(x) < ZR(x)
        assert 0.0 < WR(x) <= W(x) <= WIDTH / 2.0 + 1e-9
        assert 0.06 <= TUMBLE(x) <= 0.16 + 1e-9
    peak = max(ZR(x) for x in xs)
    assert peak == pytest.approx(HEIGHT, abs=0.01)


def test_arch_mask_aligned_with_real_axles():
    """Пики впадины арок должны совпадать с реальными осями AXLE_F и AXLE_R."""
    assert arch_mask(AXLE_F, WHEEL_Z) == pytest.approx(1.0)
    assert arch_mask(AXLE_R, WHEEL_Z) == pytest.approx(1.0)
    # В центре базы между колёсами впадины быть не должно
    assert arch_mask(0.0, WHEEL_Z) == pytest.approx(0.0)


def test_dome_scale_endpoints_and_monotonicity():
    """Суперэллипс торца убывает монотонно от 1.0 (u=0) до 0.0 (u=1)."""
    assert dome_scale(0.0) == pytest.approx(1.0)
    assert dome_scale(1.0) == pytest.approx(0.0)
    vals = [dome_scale(i / 20.0) for i in range(21)]
    for a, b in zip(vals, vals[1:]):
        assert a >= b


def test_section_vertex_counts_and_symmetry():
    """Сечение кузова содержит 18 симметричных точек, остекления — 9."""
    sec = body_section(0.0)
    assert len(sec) == V_BODY
    # Точка 0 — центр дна, точка 9 — центр крыши
    assert sec[0][0] == pytest.approx(0.0)
    assert sec[9][0] == pytest.approx(0.0)
    # Симметрия левого и правого борта
    for k in range(1, 9):
        y_l, z_l = sec[k]
        y_r, z_r = sec[V_BODY - k]
        assert y_l == pytest.approx(-y_r)
        assert z_l == pytest.approx(z_r)

    gsec = glass_section(0.0)
    assert len(gsec) == V_GH
    assert gsec[4][0] == pytest.approx(0.0)


def test_doors_open_outward_for_all_four_doors():
    """Все 4 двери при открывании должны отходить наружу (|y_tip| > |hinge_y|)."""
    assert len(DOORS_SPEC) == 4
    for name, _hx, side, door_len, open_deg in DOORS_SPEC:
        hinge_y = side * (WHEEL_Y + 0.06)
        tip_y = door_swing_tip_y(side, door_len, open_deg, hinge_y)
        assert abs(tip_y) > abs(hinge_y) + 0.5, f'{name} открывается внутрь салона!'
        assert math.copysign(1.0, tip_y) == math.copysign(1.0, side)


@pytest.mark.parametrize(
    'raw, expected',
    [
        ('Wheel_FL', 'Wheel_FL'),
        ('Car Paint (Metallic)', 'Car_Paint_Metallic'),
        ('123Mesh', 'obj_123Mesh'),
        ('---', 'obj'),
        ('Door.Front.001', 'Door_Front_001'),
    ],
)
def test_sanitize_name(raw, expected):
    assert sanitize_name(raw) == expected
    # Идемпотентность
    assert sanitize_name(expected) == expected


def test_constants_sane():
    assert N_SECTIONS >= 50
    assert LENGTH == pytest.approx(5.00)
    assert WIDTH == pytest.approx(1.872)
    assert HEIGHT == pytest.approx(1.432)


def test_cabin_door_cutout_recessed_to_sill():
    """В зоне дверных проёмов нижняя ванна кузова вырезана до уровня порога и пола салона."""
    x_mid = 0.5 * (CABIN_CUT_X_REAR + CABIN_CUT_X_FRONT)
    assert cabin_cut_mask(x_mid) == pytest.approx(1.0)
    assert cabin_cut_mask(1.5) == pytest.approx(0.0)
    assert cabin_cut_mask(-1.5) == pytest.approx(0.0)
    hp = body_half_profile(x_mid)
    assert max(z for _, z in hp) == pytest.approx(CABIN_SILL_Z, abs=1e-6)
    assert hp[9][1] == pytest.approx(CABIN_FLOOR_Z, abs=1e-6)


def test_trunk_bay_recessed_inside_body():
    """Внутри проёма багажника верх кузова углублён до TRUNK_BAY_Z, а снаружи — нет."""
    x_mid = 0.5 * (TRUNK_X_FRONT + TRUNK_X_REAR)
    assert trunk_bay_mask(x_mid) == pytest.approx(1.0)
    assert trunk_bay_mask(0.0) == pytest.approx(0.0)
    hp_mid = body_half_profile(x_mid)
    assert hp_mid[9][1] == pytest.approx(TRUNK_BAY_Z, abs=1e-6)
    assert hp_mid[9][1] > ZB(x_mid) + 0.15


def test_trunk_gas_lifts_telescope_validly():
    """Цилиндр и шток лифта не должны вылезать за крепления в закрытом виде
    и не должны разъединяться при полном открытии багажника."""
    _, _, d_closed = trunk_lift_endpoints(0.0)
    _, _, d_open = trunk_lift_endpoints(TRUNK_OPEN_DEG)
    assert d_open > d_closed + 0.20
    assert TRUNK_LIFT_CYL_LEN < d_closed
    assert TRUNK_LIFT_ROD_LEN < d_closed
    # Запас перекрытия штока внутри цилиндра в полностью открытом положении >= 30 мм
    assert TRUNK_LIFT_CYL_LEN + TRUNK_LIFT_ROD_LEN > d_open + 0.03


def test_hood_bay_and_gas_lifts_telescope_validly():
    """Моторный отсек углублён до HOOD_BAY_Z, а газовые упоры капота корректно работают."""
    x_mid = 0.5 * (HOOD_X_REAR + HOOD_X_FRONT)
    assert hood_bay_mask(x_mid) == pytest.approx(1.0)
    assert hood_bay_mask(0.0) == pytest.approx(0.0)
    hp_mid = body_half_profile(x_mid)
    assert hp_mid[9][1] == pytest.approx(HOOD_BAY_Z, abs=1e-6)

    _, (_, wz_closed), d_closed = hood_lift_endpoints(0.0)
    _, (_, wz_open), d_open = hood_lift_endpoints(HOOD_OPEN_DEG)
    assert wz_open > wz_closed + 0.35
    assert d_open > d_closed + 0.25
    assert HOOD_LIFT_CYL_LEN < d_closed
    assert HOOD_LIFT_ROD_LEN < d_closed
    assert HOOD_LIFT_CYL_LEN + HOOD_LIFT_ROD_LEN > d_open + 0.03


