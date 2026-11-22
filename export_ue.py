#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_ue.py — экспорт сцены автомобиля в FBX для Unreal Engine 4/5.

Запуск после сборки сцены:
    blender -b -noaudio car_realistic.blend -P export_ue.py
    blender -b -noaudio car_realistic.blend -P export_ue.py -- --out-dir ./exports

Результат (в exports/):
    car_ue.fbx       — статичные меши коллекции Car + встроенные коллизии UCX_*
                       (UE автоматически связывает UCX_<Имя> с мешом в одном FBX)
    car_ue_anim.fbx  — анимированный автомобиль (колёса, руль, крен, двери) с
                       сохранением всех шарниров/осей вращения и иерархии parent
    car_ue_col.fbx   — только простые коллизии UCX_* (для отдельного просмотра)

Настройки под UE (встроены в скрипт, совместимы с Blender 4.2):
  * масштаб 100.0 — метры Blender -> сантиметры UE
  * ось вверх Z, вперёд -X (стандартный конвейер Blender -> UE)
  * триангуляция полигонов (use_triangles=True)
  * группы сглаживания по граням (mesh_smooth_type='FACE')
  * UV-канал гарантирован на каждом меше
  * экспортируется только коллекция Car (студийная Cyclorama исключена)
"""
import argparse
import os
import sys

import bpy
import bmesh
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE and HERE not in sys.path:
    sys.path.insert(0, HERE)

from car_geometry import (  # noqa: E402
    CAR_COLLECTION,
    STUDIO_COLLECTION,
    sanitize_name,
)


def ensure_uvs(ob):
    """Генерирует простую проекционную UV-развёртку, если у меша нет UV-канала.
    UE требует хотя бы один UV-канал на меш."""
    me = ob.data
    if me.uv_layers:
        return
    bm = bmesh.new()
    try:
        bm.from_mesh(me)
        if not bm.faces or not bm.verts:
            return
        bm.normal_update()
        avg_normal = sum((f.normal for f in bm.faces), Vector()) / float(len(bm.faces))
        dominant = max(range(3), key=lambda i: abs(avg_normal[i]))
        uv_axes = [i for i in range(3) if i != dominant]
        coords = [v.co.copy() for v in bm.verts]
    finally:
        bm.free()

    mins = [min(c[ax] for c in coords) for ax in uv_axes]
    maxs = [max(c[ax] for c in coords) for ax in uv_axes]
    spans = [maxs[i] - mins[i] if maxs[i] > mins[i] else 1.0 for i in range(2)]

    layer = me.uv_layers.new(name='UVMap')
    uv = layer.data
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            uv[loop_idx].uv = (
                (v.co[uv_axes[0]] - mins[0]) / spans[0],
                (v.co[uv_axes[1]] - mins[1]) / spans[1],
            )


def select_car_meshes():
    """Возвращает меши автомобиля из коллекции Car (исключая Studio/Cyclorama и UCX_*)."""
    car_col = bpy.data.collections.get(CAR_COLLECTION)
    if car_col is not None:
        return [
            ob for ob in car_col.objects
            if ob.type == 'MESH' and not ob.name.startswith('UCX_')
        ]
    # Запасной вариант, если сцена собрана без коллекций
    studio_col = bpy.data.collections.get(STUDIO_COLLECTION)
    studio_names = {ob.name for ob in studio_col.objects} if studio_col is not None else set()
    return [
        ob for ob in bpy.data.objects
        if ob.type == 'MESH'
        and not ob.name.startswith('UCX_')
        and ob.name not in studio_names
        and ob.name != 'Cyclorama'
    ]


def make_box_collision(name, ob, pad=0.02, collection=None):
    """Простая выпуклая коллизия в виде коробки по мировому bounding-box объекта."""
    if ob.type != 'MESH' or not ob.data.vertices:
        return None
    world_verts = [ob.matrix_world @ v.co for v in ob.data.vertices]
    if not world_verts:
        return None
    min_corner = Vector((
        min(v.x for v in world_verts) - pad,
        min(v.y for v in world_verts) - pad,
        min(v.z for v in world_verts) - pad,
    ))
    max_corner = Vector((
        max(v.x for v in world_verts) + pad,
        max(v.y for v in world_verts) + pad,
        max(v.z for v in world_verts) + pad,
    ))

    bm = bmesh.new()
    pts = {}
    for ix, x in enumerate((min_corner.x, max_corner.x)):
        for iy, y in enumerate((min_corner.y, max_corner.y)):
            for iz, z in enumerate((min_corner.z, max_corner.z)):
                pts[(ix, iy, iz)] = bm.verts.new(Vector((x, y, z)))

    def V(i, j, k):
        return pts[(i, j, k)]

    faces = [
        (V(0, 0, 0), V(0, 1, 0), V(1, 1, 0), V(1, 0, 0)),  # низ
        (V(0, 0, 1), V(1, 0, 1), V(1, 1, 1), V(0, 1, 1)),  # верх
        (V(0, 0, 0), V(1, 0, 0), V(1, 0, 1), V(0, 0, 1)),  # -Y
        (V(0, 1, 0), V(0, 1, 1), V(1, 1, 1), V(1, 1, 0)),  # +Y
        (V(0, 0, 0), V(0, 0, 1), V(0, 1, 1), V(0, 1, 0)),  # -X
        (V(1, 0, 0), V(1, 1, 0), V(1, 1, 1), V(1, 0, 1)),  # +X
    ]
    for f in faces:
        bm.faces.new(f)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    me = bpy.data.meshes.new(name + '_colmesh')
    bm.to_mesh(me)
    bm.free()
    me.update()
    cob = bpy.data.objects.new(name, me)
    target_col = collection if collection is not None else bpy.context.scene.collection
    target_col.objects.link(cob)
    return cob


def build_collision_meshes(meshes):
    """Создаёт коллизии UCX_<Имя> для каждого меша автомобиля (соглашение UE)."""
    car_col = bpy.data.collections.get(CAR_COLLECTION)
    collision_obs = []
    for ob in meshes:
        col_name = 'UCX_' + sanitize_name(ob.name)
        cob = make_box_collision(col_name, ob, collection=car_col)
        if cob is not None:
            collision_obs.append(cob)
    return collision_obs


def fbx_export(filepath, use_animation=False, objects=None):
    """Экспорт выбранных объектов в FBX с настройками под UE (Blender 4.2 API)."""
    for ob in bpy.data.objects:
        ob.select_set(False)
    if objects:
        for ob in objects:
            ob.select_set(True)
        bpy.context.view_layer.objects.active = objects[0]

    bpy.ops.export_scene.fbx(
        filepath=filepath,
        check_existing=False,
        use_selection=True,
        use_active_collection=False,
        global_scale=100.0,            # метры -> сантиметры (единица UE)
        apply_unit_scale=True,
        apply_scale_options='FBX_SCALE_ALL',
        use_space_transform=True,
        axis_forward='-X',
        axis_up='Z',
        object_types={'ARMATURE', 'EMPTY', 'MESH'},
        use_mesh_modifiers=True,
        use_mesh_modifiers_render=True,
        mesh_smooth_type='FACE',       # группы сглаживания для UE
        use_subsurf=False,
        use_mesh_edges=False,
        use_tspace=False,              # касательные рассчитывает UE
        use_triangles=True,            # триангуляция полигонов
        use_custom_props=False,
        add_leaf_bones=False,
        primary_bone_axis='Y',
        secondary_bone_axis='X',
        use_armature_deform_only=False,
        bake_anim=use_animation,
        bake_anim_use_all_bones=False,
        bake_anim_use_nla_strips=False,
        bake_anim_use_all_actions=False,
        bake_anim_force_startend_keying=True,
        bake_anim_step=1.0,
        bake_anim_simplify_factor=0.0,
        batch_mode='OFF',
        use_batch_own_dir=False,
        use_metadata=False,
    )


def export_all(out_dir='exports'):
    """Экспортирует статичный, анимированный и коллизионный FBX. Возвращает dict путей."""
    export_dir = os.path.abspath(out_dir)
    os.makedirs(export_dir, exist_ok=True)

    # Возвращаем сцену в кадр 1 (нейтральная поза) перед расчётом bounding-box
    bpy.context.scene.frame_set(1)
    bpy.context.view_layer.update()

    meshes = select_car_meshes()
    if not meshes:
        print('export_ue.py: нет мешевых объектов автомобиля в сцене')
        return {}

    print(f'export_ue.py: найдено мешей автомобиля: {len(meshes)}')

    # Санитизация имён и проверка UV
    for ob in meshes:
        clean = sanitize_name(ob.name)
        if ob.name != clean:
            ob.name = clean
        ensure_uvs(ob)

    # Удаляем старые UCX_-коллизии, если скрипт запускается повторно на той же сцене
    for ob in list(bpy.data.objects):
        if ob.name.startswith('UCX_'):
            bpy.data.objects.remove(ob, do_unlink=True)

    exported = {}

    # 1. Экспорт с анимацией (сохраняем исходные шарниры/оси вращения и иерархию parent!)
    has_anim = any(
        ob.animation_data and ob.animation_data.action
        for ob in meshes
    )
    if has_anim:
        anim_path = os.path.join(export_dir, 'car_ue_anim.fbx')
        fbx_export(anim_path, use_animation=True, objects=meshes)
        exported['anim'] = anim_path
        print('exported:', anim_path)

    # 2. Коллизии UCX_* по мировым габаритам в кадре 1
    collision_obs = build_collision_meshes(meshes)
    print(f'export_ue.py: коллизий создано: {len(collision_obs)}')

    # 3. Статичный экспорт: меши автомобиля + UCX_* в одном FBX (автосвязка в UE)
    static_path = os.path.join(export_dir, 'car_ue.fbx')
    fbx_export(static_path, use_animation=False, objects=meshes + collision_obs)
    exported['static'] = static_path
    print('exported:', static_path)

    # 4. Отдельный экспорт только коллизий
    if collision_obs:
        col_path = os.path.join(export_dir, 'car_ue_col.fbx')
        fbx_export(col_path, use_animation=False, objects=collision_obs)
        exported['collision'] = col_path
        print('exported:', col_path)

    print('export_ue.py: ГОТОВО. FBX-файлы в', export_dir)
    return exported


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Экспорт автомобиля в FBX для Unreal Engine')
    parser.add_argument('--out-dir', default='exports',
                        help='Директория сохранения FBX (по умолчанию exports)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or [])
    res = export_all(out_dir=args.out_dir)
    if not res:
        sys.exit(1)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
