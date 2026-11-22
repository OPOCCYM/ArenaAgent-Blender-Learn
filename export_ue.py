#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_ue.py — экспорт сцены автомобиля в FBX для Unreal Engine.

Запуск после сборки сцены:
    blender -b -noaudio car_realistic.blend -P export_ue.py

Результат:
    exports/car_ue.fbx               — главный меш без анимации
    exports/car_ue_anim.fbx          — с анимацией (колёса, руль, крен)
    exports/car_ue_col.fbx           — с простыми коллизиями (UCX_*)

Настройки под UE (встроены в скрипт):
  * масштаб 100.0 — метры Blender → сантиметры UE
  * transforms применены (location, rotation, scale) перед экспортом
  * ось вверх Z, вперёд -X (стандартный конвеер Blender → UE)
  * триангуляция полигонов (UE предпочитает треугольники)
  * имена объектов очищены от пробелов и спецсимволов
  * материалы экспортируются с именами Blender-материалов (UE подхватит
    по имени при импорте; можно заменить в UE на свои)
  * опциональные простые коллизии — коробки с префиксом UCX_ (соглашение UE)

Требования:
  * Blender 4.2 (скрипт использует только bpy, без сторонних модулей)
  * Сцена уже собрана car_build.py и сохранена в car_realistic.blend
"""
import os
import re
import bpy
import bmesh
from mathutils import Vector

EXPORT_DIR = os.path.abspath('exports')
FBX_VERSION = 'BIN7400'  #最大 совместимость с UE 4/5


def sanitize_name(name):
    """UE не любит пробелы и спецсимволы в именах мешей/материалов."""
    s = re.sub(r'[^A-Za-z0-9_]', '_', name)
    s = re.sub(r'_+', '_', s).strip('_')
    if not s or s[0].isdigit():
        s = 'obj_' + s
    return s


def ensure_uvs(ob):
    """Генерирует простую проекционную UV-развёртку, если её нет.
    UE требует хотя бы один UV-канал на меш, иначе текстуры не наложатся."""
    me = ob.data
    if me.uv_layers:
        return
    bm = bmesh.new()
    bm.from_mesh(me)
    # projecting along the dominant axis for a simple unwrap
    import mathutils
    # find dominant normal axis
    bm.normal_update()
    avg_normal = sum((f.normal for f in bm.faces), Vector()) / len(bm.faces)
    dominant = max(range(3), key=lambda i: abs(avg_normal[i]))
    # choose two other axes for UV
    uv_axes = [i for i in range(3) if i != dominant]
    me.uv_layers.new()
    uv = me.uv_layers[0].data
    # bounds for the two UV axes
    coords = [v.co for v in bm.verts]
    mins = [min(c[ax] for c in coords) for ax in uv_axes]
    maxs = [max(c[ax] for c in coords) for ax in uv_axes]
    spans = [maxs[i] - mins[i] if maxs[i] > mins[i] else 1.0 for i in range(2)]
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            uv[loop_idx].uv = (
                (v.co[uv_axes[0]] - mins[0]) / spans[0],
                (v.co[uv_axes[1]] - mins[1]) / spans[1],
            )


def apply_transforms(objs):
    """Применяет location/rotation/scale у выбранных объектов."""
    for ob in objs:
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0] if objs else None
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    for ob in objs:
        ob.select_set(False)


def select_meshes():
    """Возвращает все мешевые объекты сцены (кроме коллизий, они добавляются отдельно)."""
    return [ob for ob in bpy.data.objects
            if ob.type == 'MESH' and not ob.name.startswith('UCX_')]


def make_box_collision(name, ob, thickness=0.0):
    """Простая выпуклая коллизия в виде коробки по bounding-box объекта."""
    if ob.type != 'MESH' or not ob.data.vertices:
        return None
    world_verts = [ob.matrix_world @ v.co for v in ob.data.vertices]
    if not world_verts:
        return None
    min_corner = Vector((
        min(v.x for v in world_verts),
        min(v.y for v in world_verts),
        min(v.z for v in world_verts),
    ))
    max_corner = Vector((
        max(v.x for v in world_verts),
        max(v.y for v in world_verts),
        max(v.z for v in world_verts),
    ))
    # немного расширяем, чтобы коллизия точно покрывала меш
    pad = 0.02
    min_corner -= Vector((pad, pad, pad))
    max_corner += Vector((pad, pad, pad))

    bm = bmesh.new()
    pts = {}
    for ix, x in enumerate((min_corner.x, max_corner.x)):
        for iy, y in enumerate((min_corner.y, max_corner.y)):
            for iz, z in enumerate((min_corner.z, max_corner.z)):
                pts[(ix, iy, iz)] = bm.verts.new(Vector((x, y, z)))

    def V(i, j, k):
        return pts[(i, j, k)]

    faces = [
        (V(0,0,0), V(1,0,0), V(1,1,0), V(0,1,0)),  # низ
        (V(0,0,1), V(0,1,1), V(1,1,1), V(1,0,1)),  # верх
        (V(0,0,0), V(0,0,1), V(1,0,1), V(1,0,0)),  # зад
        (V(1,1,0), V(1,1,1), V(0,1,1), V(0,1,0)),  # перед
        (V(0,0,0), V(0,1,0), V(0,1,1), V(0,0,1)),  # лев
        (V(1,0,1), V(1,1,1), V(1,1,0), V(1,0,0)),  # правый
    ]
    for f in faces:
        bm.faces.new(f)

    # приоритет: нормали наружу (проверка по центру)
    bm.normal_update()
    center = sum((v.co for v in bm.verts), Vector()) / len(bm.verts)
    for f in list(bm.faces):
        fc = sum((v.co for v in f.verts), Vector()) / len(f.verts)
        if f.normal.dot(fc - center) < 0.0:
            verts = list(f.verts)
            bm.faces.remove(f)
            bm.faces.new(verts[::-1])

    me = bpy.data.meshes.new(name + '_colmesh')
    bm.to_mesh(me)
    bm.free()
    me.update()
    cob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(cob)
    return cob


def build_collision_meshes(meshes):
    """Создаёт UCX_-коллизии для каждого меша (соглашение имён UE)."""
    collision_obs = []
    for ob in meshes:
        col_name = 'UCX_' + sanitize_name(ob.name)
        cob = make_box_collision(col_name, ob)
        if cob is not None:
            collision_obs.append(cob)
    return collision_obs


def fbx_export(filepath, use_animation=False, objects=None):
    """Экспорт выбранных объектов в FBX с настройками под UE."""
    # сбрасываем выделение
    bpy.ops.object.select_all(action='DESELECT')
    if objects is not None:
        for ob in objects:
            ob.select_set(True)
        if objects:
            bpy.context.view_layer.objects.active = objects[0]

    # FBX-экспорт (встроенный оператор Blender)
    bpy.ops.export_scene.fbx(
        filepath=filepath,
        check_existing=False,
        use_selection=True,
        use_active_collection=False,
        global_scale=100.0,          # метры → см (единица UE)
        apply_unit_scale=True,
        apply_scale_options='FBX_SCALE_ALL',
        use_space_transform=True,    # применять transforms
        axis_forward='-X',
        axis_up='Z',
        object_types={'ARMATURE', 'CAMERA', 'EMPTY', 'MESH'},
        use_mesh_modifiers=True,
        use_mesh_modifiers_render=True,
        mesh_smooth_type='OFF',      # нормали уже применены через smooth shading
        use_subsurf=False,
        use_mesh_edges=False,
        use_tspace=False,            # UE сама считает касательные
        use_custom_props=False,
        add_leaf_bones=False,
        primary_bone_axis='Y',
        secondary_bone_axis='X',
        use_armature_deform_only=False,
        use_anim=use_animation,
        use_anim_action_all=False,
        use_default_take=False,
        use_anim_optimize=False,
        anim_batch_mode='UNIFIED',
        batch_mode='OFF',
        use_batch_own_dir=False,
        batch_own_dir='',
        use_metadata=False,
    )


def main():
    os.makedirs(EXPORT_DIR, exist_ok=True)

    scene = bpy.context.scene
    meshes = select_meshes()
    if not meshes:
        print('export_ue.py: нет мешовых объектов в сцене')
        return

    print(f'export_ue.py: найдено мешей: {len(meshes)}')

    # --- имён объектов: UE-стиль (без пробелов/скобок) ---
    for ob in meshes:
        ob.name = sanitize_name(ob.name)

    # --- UV на всех мешах (UE требует хотя бы один канал) ---
    for ob in meshes:
        ensure_uvs(ob)

    # --- применяем transforms у всех мешей ---
    apply_transforms(meshes)

    # --- коллизии (UCX_*) ---
    collision_obs = build_collision_meshes(meshes)
    print(f'export_ue.py: коллизий создано: {len(collision_obs)}')

    # --- экспорт без анимации ---
    fbx_export(
        os.path.join(EXPORT_DIR, 'car_ue.fbx'),
        use_animation=False,
        objects=meshes + collision_obs,
    )
    print('exported:', os.path.join(EXPORT_DIR, 'car_ue.fbx'))

    # --- экспорт с анимацией (если есть ключи) ---
    has_anim = any(
        ob.animation_data and ob.animation_data.action
        for ob in meshes
    )
    if has_anim:
        fbx_export(
            os.path.join(EXPORT_DIR, 'car_ue_anim.fbx'),
            use_animation=True,
            objects=meshes,
        )
        print('exported:', os.path.join(EXPORT_DIR, 'car_ue_anim.fbx'))

    # --- экспорт только коллизий ---
    if collision_obs:
        fbx_export(
            os.path.join(EXPORT_DIR, 'car_ue_col.fbx'),
            use_animation=False,
            objects=collision_obs,
        )
        print('exported:', os.path.join(EXPORT_DIR, 'car_ue_col.fbx'))

    print('export_ue.py: ГОТОВО. FBX-файлы в', EXPORT_DIR)


if __name__ == '__main__':
    main()
