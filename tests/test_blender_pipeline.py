#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Интеграционные тесты пайплайна в реальном Blender 4.2 (bpy).
Автоматически пропускаются, если модуль bpy недоступен в текущем окружении."""
import math
import os

import pytest

bpy = pytest.importorskip('bpy')

import car_build  # noqa: E402
import export_ue  # noqa: E402
import qa  # noqa: E402
import turntable  # noqa: E402
from car_geometry import CAR_COLLECTION, HEIGHT, LENGTH, STUDIO_COLLECTION, WIDTH  # noqa: E402


@pytest.fixture(scope='module')
def built_scene_dir(tmp_path_factory):
    out_dir = tmp_path_factory.mktemp('car_pipeline')
    car_build.main(['--out-dir', str(out_dir), '--samples', '4'])
    blend_path = out_dir / 'car_realistic.blend'
    assert blend_path.exists() and blend_path.stat().st_size > 0
    return out_dir


def test_geometry_qa_all_zero(built_scene_dir):
    bpy.ops.wm.open_mainfile(filepath=str(built_scene_dir / 'car_realistic.blend'))
    ok, totals, _rows = qa.run_qa()
    assert ok is True
    assert totals['dups'] == 0
    assert totals['deg'] == 0
    assert totals['nonman'] == 0
    assert totals['ngon'] == 0
    assert totals['loose'] == 0
    assert totals['empty'] == 0
    assert totals['no_uv'] == 0
    assert totals['dot_name'] == 0


def test_collections_and_bounding_box(built_scene_dir):
    bpy.ops.wm.open_mainfile(filepath=str(built_scene_dir / 'car_realistic.blend'))
    car_col = bpy.data.collections.get(CAR_COLLECTION)
    studio_col = bpy.data.collections.get(STUDIO_COLLECTION)
    assert car_col is not None
    assert studio_col is not None

    car_names = {ob.name for ob in car_col.objects}
    studio_names = {ob.name for ob in studio_col.objects}
    assert 'Cyclorama' in studio_names
    assert 'Cyclorama' not in car_names
    for required in (
        'Body', 'Roof', 'Glass', 'Cabin', 'Dashboard', 'SteeringWheel',
        'Pillar_A_L', 'Pillar_A_R', 'Pillar_B_L', 'Pillar_B_R', 'Pillar_C_L', 'Pillar_C_R',
        'Wheel_FL', 'Wheel_FR', 'Wheel_RL', 'Wheel_RR',
        'Rim_FL', 'Rim_FR', 'Rim_RL', 'Rim_RR',
        'Brake_Disc_FL', 'Brake_Disc_FR', 'Brake_Disc_RL', 'Brake_Disc_RR',
        'Brake_Caliper_FL', 'Brake_Caliper_FR', 'Brake_Caliper_RL', 'Brake_Caliper_RR',
        'Door_Front_Left', 'Door_Front_Right', 'Door_Rear_Left', 'Door_Rear_Right',
        'Door_Glass_Front_Left', 'Door_Glass_Front_Right',
        'Door_Glass_Rear_Left', 'Door_Glass_Rear_Right',
        'Door_Hinge_Front_Left', 'Door_Hinge_Front_Right',
        'Door_Hinge_Rear_Left', 'Door_Hinge_Rear_Right',
        'Door_Handle_Front_Left', 'Door_Handle_Front_Right',
        'Door_Handle_Rear_Left', 'Door_Handle_Rear_Right',
        'Door_Handle_Base_Front_Left', 'Door_Handle_Base_Front_Right',
        'Door_Handle_Base_Rear_Left', 'Door_Handle_Base_Rear_Right',
        'Center_Console', 'Dash_Trim', 'Instrument_Cluster', 'Air_Vents',
        'Steering_Column', 'Pedals', 'Interior_Roof_Details', 'Cabin_Seats_Insert',
        'Seatbelts',
        'PDC_Sensors_Front', 'Headlight_Washers', 'PDC_Sensors_Rear', 'Trunk_Badge',
        'Mirror_L', 'Mirror_R',
        'Hood_Lid', 'Engine_Bay', 'Engine_Metal', 'Coolant_Tank',
        'Fender_Front_L', 'Fender_Front_R', 'Front_Bumper',
        'Front_Subframe', 'Crash_Beam', 'Fender_Apron_L', 'Fender_Apron_R',
        'Radiator_Support', 'Cowl_Panel',
        'Rear_Subframe', 'Rear_Crash_Beam', 'Trunk_Tub',
        'Fender_Rear_L', 'Fender_Rear_R', 'Rear_Bumper',
        'Hood_Hinge_Mount_L', 'Hood_Hinge_Mount_R',
        'Hood_Hinge_Arm_L', 'Hood_Hinge_Arm_R',
        'Hood_Lift_Mount_L', 'Hood_Lift_Mount_R',
        'Hood_Lift_Bracket_L', 'Hood_Lift_Bracket_R',
        'Hood_Lift_Cyl_L', 'Hood_Lift_Cyl_R',
        'Hood_Lift_Rod_L', 'Hood_Lift_Rod_R',
        'Trunk_Lid', 'Trunk_Floor',
        'Trunk_Hinge_Mount_L', 'Trunk_Hinge_Mount_R',
        'Trunk_Hinge_Arm_L', 'Trunk_Hinge_Arm_R',
        'Trunk_Lift_Mount_L', 'Trunk_Lift_Mount_R',
        'Trunk_Lift_Bracket_L', 'Trunk_Lift_Bracket_R',
        'Trunk_Lift_Cyl_L', 'Trunk_Lift_Cyl_R',
        'Trunk_Lift_Rod_L', 'Trunk_Lift_Rod_R',
        'Grille_Front', 'Grille_Chrome', 'Bumper_Intake_Front',
        'Headlight_L', 'Headlight_R', 'Headlight_DRL_L', 'Headlight_DRL_R',
        'Tail_Light_L', 'Tail_Light_R', 'Tail_Housing_L', 'Tail_Housing_R',
        'Tail_Lens_L', 'Tail_Lens_R', 'Rear_Diffuser', 'Exhaust_L', 'Exhaust_R',
        'Wheel_Arch_FL', 'Wheel_Arch_FR', 'Wheel_Arch_RL', 'Wheel_Arch_RR',
        'Wheel_Well_FL', 'Wheel_Well_FR', 'Wheel_Well_RL', 'Wheel_Well_RR',
    ):
        assert required in car_names, f'Отсутствует обязательный объект {required}'

    # В зоне дверных проёмов Body действительно имеет сквозной вырез до уровня порога,
    # а передняя и задняя части (крылья и бамперы) реально отделены от монокока Body
    body = bpy.data.objects['Body']
    assert max(v.co.x for v in body.data.vertices) < 1.00
    assert min(v.co.x for v in body.data.vertices) > -1.12
    for x_probe in (-0.50, 0.35):
        zs_near = [v.co.z for v in body.data.vertices if abs(v.co.x - x_probe) < 0.04]
        assert zs_near and max(zs_near) < 0.32, f'Нет сквозного выреза в Body на x={x_probe}'

    # Фары и задние фонари закреплены на силовом каркасе кузова (НЕ на бамперах и НЕ на багажнике)
    for hl_name in ('Headlight_L', 'Headlight_R', 'Headlight_DRL_L', 'Headlight_DRL_R',
                    'Headlight_Lens_L', 'Headlight_Lens_R'):
        assert bpy.data.objects[hl_name].parent.name == 'Radiator_Support'
    for tl_name in ('Tail_Housing_L', 'Tail_Housing_R', 'Tail_Light_L', 'Tail_Light_R',
                    'Tail_Lens_L', 'Tail_Lens_R'):
        assert bpy.data.objects[tl_name].parent.name == 'Rear_Subframe'
    # Корпуса задних фонарей компактны и не заходят глубоко внутрь багажника
    for th_name in ('Tail_Housing_L', 'Tail_Housing_R'):
        th_ob = bpy.data.objects[th_name]
        th_xs = [(th_ob.matrix_world @ v.co).x for v in th_ob.data.vertices]
        assert (max(th_xs) - min(th_xs)) < 0.14, f'{th_name} слишком глубоко заходит в багажник'
    # Оба бампера — отдельные съёмные детали контрастного цвета (отличного от цвета кузова)
    body_mat = body.data.materials[0].name
    for bmp_name in ('Front_Bumper', 'Rear_Bumper'):
        bmp_ob = bpy.data.objects[bmp_name]
        assert bmp_ob.parent.name == 'Body'
        assert bmp_ob.data.materials[0].name != body_mat, (
            f'{bmp_name} должен быть другого цвета, чем кузов'
        )
    # Детали переднего и заднего бамперов привязаны к соответствующим съёмным бамперам
    for fb_child in ('Grille_Front', 'Grille_Chrome', 'Bumper_Intake_Front',
                     'PDC_Sensors_Front', 'Headlight_Washers'):
        assert bpy.data.objects[fb_child].parent.name == 'Front_Bumper'
    for rb_child in ('Rear_Diffuser', 'Exhaust_L', 'Exhaust_R',
                     'Rear_Reflector_L', 'Rear_Reflector_R', 'PDC_Sensors_Rear'):
        assert bpy.data.objects[rb_child].parent.name == 'Rear_Bumper'
    assert bpy.data.objects['Trunk_Badge'].parent.name == 'Trunk_Lid'
    # Упакованные OLED-текстуры приборной панели 12.3" и планшета Sensus 9" в .blend
    for img_name in ('Cluster_UI_Tex', 'Sensus_UI_Tex'):
        img = bpy.data.images.get(img_name)
        assert img is not None and img.packed_file is not None, f'Не упакована текстура {img_name}'
    # Дверные ручки полностью отделены от дверей и имеют собственный origin в шарнире ручки
    for d_tag in ('Front_Left', 'Front_Right', 'Rear_Left', 'Rear_Right'):
        hdl = bpy.data.objects[f'Door_Handle_{d_tag}']
        hbase = bpy.data.objects[f'Door_Handle_Base_{d_tag}']
        assert hdl.parent.name == f'Door_{d_tag}'
        assert hbase.parent.name == f'Door_{d_tag}'
        assert hdl.location.length > 0.30, f'{hdl.name} должна иметь собственный шарнир-origin'
    assert bpy.data.objects['Wheel_Arch_RL'].parent.name == 'Fender_Rear_L'
    assert bpy.data.objects['Wheel_Arch_RR'].parent.name == 'Fender_Rear_R'

    # Стойки A/B/C реально соединяют нижнюю ванну кузова с отдельной крышей Roof
    roof = bpy.data.objects['Roof']
    roof_min_z = min((roof.matrix_world @ v.co).z for v in roof.data.vertices)
    for pname in ('Pillar_A_L', 'Pillar_A_R', 'Pillar_B_L', 'Pillar_B_R',
                  'Pillar_C_L', 'Pillar_C_R'):
        pob = bpy.data.objects[pname]
        pzs = [(pob.matrix_world @ v.co).z for v in pob.data.vertices]
        assert min(pzs) < 0.30, f'{pname} не доходит до порога кузова'
        assert max(pzs) >= roof_min_z - 0.02, f'{pname} не доходит до крыши Roof'

    bpy.context.scene.frame_set(1)
    bpy.context.view_layer.update()
    pts = [
        ob.matrix_world @ v.co
        for ob in car_col.objects if ob.type == 'MESH'
        for v in ob.data.vertices
    ]
    # Ширина кузова без учёта боковых зеркал соответствует паспортному WIDTH
    body_ys = [
        (ob.matrix_world @ v.co).y
        for ob in car_col.objects
        if ob.type == 'MESH' and ob.name not in ('Mirror_L', 'Mirror_R')
        for v in ob.data.vertices
    ]
    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    zs = [p.z for p in pts]
    assert min(zs) >= -0.005, 'Автомобиль провалился ниже уровня дороги Z=0'
    assert (max(xs) - min(xs)) == pytest.approx(LENGTH, abs=0.10)
    assert (max(body_ys) - min(body_ys)) == pytest.approx(WIDTH, abs=0.10)
    assert (max(ys) - min(ys)) > WIDTH + 0.15  # зеркала выступают за габарит кузова
    assert (max(zs) - min(zs)) == pytest.approx(HEIGHT, abs=0.08)


def test_animations_have_real_motion_and_outward_doors(built_scene_dir):
    bpy.ops.wm.open_mainfile(filepath=str(built_scene_dir / 'car_realistic.blend'))
    scene = bpy.context.scene
    scene.frame_set(1)
    w_fl = bpy.data.objects['Wheel_FL']
    sw = bpy.data.objects['SteeringWheel']
    body = bpy.data.objects['Body']
    assert w_fl.rotation_euler.y == pytest.approx(0.0, abs=1e-4)

    # Проверяем движение на 60-м кадре (пик поворота руля, крена и открытые двери)
    scene.frame_set(60)
    bpy.context.view_layer.update()
    assert abs(w_fl.rotation_euler.y) > 50.0
    assert sw.rotation_euler.x == pytest.approx(math.radians(17.0), abs=1e-3)
    assert body.rotation_euler.x == pytest.approx(math.radians(0.8), abs=1e-4)

    # Все 4 двери должны быть открыты наружу (увеличение |y| свободной кромки)
    for name in ('Door_Front_Left', 'Door_Front_Right', 'Door_Rear_Left', 'Door_Rear_Right'):
        door = bpy.data.objects[name]
        assert abs(door.rotation_euler.z) == pytest.approx(math.radians(70.0), abs=1e-2)
        hinge_y = door.matrix_world.translation.y
        tip_world = door.matrix_world @ bpy.data.objects[name].data.vertices[0].co
        # Ищем вершину с максимальным |y| — она должна быть дальше от оси, чем шарнир
        max_abs_y = max(abs((door.matrix_world @ v.co).y) for v in door.data.vertices)
        assert max_abs_y > abs(hinge_y) + 0.5, f'{name} открывается внутрь (tip={tip_world})'

    # Крышка багажника поднята вверх на 62°, газовые лифты точно соосны креплениям
    from mathutils import Vector
    trunk = bpy.data.objects['Trunk_Lid']
    assert trunk.rotation_euler.y == pytest.approx(math.radians(62.0), abs=1e-2)
    trunk_max_z = max((trunk.matrix_world @ v.co).z for v in trunk.data.vertices)
    assert trunk_max_z > 1.45
    for side_tag in ('L', 'R'):
        cyl = bpy.data.objects[f'Trunk_Lift_Cyl_{side_tag}']
        rod = bpy.data.objects[f'Trunk_Lift_Rod_{side_tag}']
        mnt = bpy.data.objects[f'Trunk_Lift_Mount_{side_tag}']
        brk = bpy.data.objects[f'Trunk_Lift_Bracket_{side_tag}']
        want = (brk.matrix_world.translation - mnt.matrix_world.translation).normalized()
        c_axis = (cyl.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
        r_axis = (rod.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
        assert c_axis.dot(want) > 0.999
        assert r_axis.dot(-want) > 0.999

    # Капот поднят вверх на -48°, его газовые лифты точно соосны креплениям
    hood = bpy.data.objects['Hood_Lid']
    assert hood.rotation_euler.y == pytest.approx(math.radians(-48.0), abs=1e-2)
    hood_max_z = max((hood.matrix_world @ v.co).z for v in hood.data.vertices)
    assert hood_max_z > 1.75
    for side_tag in ('L', 'R'):
        cyl = bpy.data.objects[f'Hood_Lift_Cyl_{side_tag}']
        rod = bpy.data.objects[f'Hood_Lift_Rod_{side_tag}']
        mnt = bpy.data.objects[f'Hood_Lift_Mount_{side_tag}']
        brk = bpy.data.objects[f'Hood_Lift_Bracket_{side_tag}']
        want = (brk.matrix_world.translation - mnt.matrix_world.translation).normalized()
        c_axis = (cyl.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
        r_axis = (rod.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
        assert c_axis.dot(want) > 0.999
        assert r_axis.dot(-want) > 0.999


def test_fbx_export_and_reimport_for_ue(built_scene_dir):
    bpy.ops.wm.open_mainfile(filepath=str(built_scene_dir / 'car_realistic.blend'))
    export_dir = built_scene_dir / 'exports'
    res = export_ue.export_all(out_dir=str(export_dir))
    assert set(res.keys()) == {'static', 'anim', 'collision'}
    for path in res.values():
        assert os.path.exists(path) and os.path.getsize(path) > 1000

    # Проверка статичного FBX: только меши Car + их UCX_*, без Cyclorama, все триангулированы
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=res['static'])
    names = {ob.name for ob in bpy.data.objects}
    assert 'Cyclorama' not in names
    assert 'Body' in names
    assert 'UCX_Body' in names
    non_tris = sum(
        1 for ob in bpy.data.objects if ob.type == 'MESH'
        for p in ob.data.polygons if len(p.vertices) != 3
    )
    assert non_tris == 0

    # Проверка анимированного FBX: шарниры/оси не сброшены в (0,0,0), есть экшены
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=res['anim'])
    w_fl = bpy.data.objects.get('Wheel_FL')
    assert w_fl is not None
    assert w_fl.location.x == pytest.approx(1.60, abs=0.05)
    assert w_fl.animation_data is not None and w_fl.animation_data.action is not None


def test_turntable_ffmpeg_command_safe_with_spaces():
    cmd = turntable.build_ffmpeg_command('/tmp/dir with spaces/turntable',
                                         '/tmp/dir with spaces/out.mp4',
                                         fps=12, pingpong=True)
    assert isinstance(cmd, list)
    assert cmd[0] == 'ffmpeg'
    assert '/tmp/dir with spaces/out.mp4' in cmd
