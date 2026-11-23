#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_build.py — параметрический генератор реалистичного седана (по мотивам Volvo S90)
для Blender 4.2.

Запуск:
    blender -b -noaudio -P car_build.py
    blender -b -noaudio -P car_build.py -- --render              # собрать сцену + 5 рендеров
    blender -b -noaudio -P car_build.py -- --render --samples 16 # быстрый превью-рендер
    blender -b -noaudio -P car_build.py -- --out-dir ./out       # каталог для артефактов

Архитектура:
  * кузов — лофт по поперечным сечениям + лапласово сглаживание;
  * колёсная арка — гладкая впадина arch_mask() по реальным осям AXLE_F / AXLE_R;
  * торцы — 3D-купола по суперэллипсу dome_scale() (pe=0.34, px=0.72);
  * остекление — тонкая замкнутая оболочка (solidify) без перекрытия салона;
  * детали сажаются ray_cast'ом (body_hit / top_hit) со смещением off по нормали;
  * все материалы процедурные, по UV (uv_top для кузова, uv_cyl для колёс, uv_box для салона);
  * объекты разделены на коллекции Car (экспортируется в UE) и Studio (свет, камеры, циклорама);
  * вся геометрия замкнута (manifold) — qa.py должен показывать нули.
"""
import argparse
import math
import os
import sys

import bpy
import bmesh
import numpy as np
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE and HERE not in sys.path:
    sys.path.insert(0, HERE)

from car_geometry import (  # noqa: E402
    AXLE_F,
    AXLE_R,
    BELT,
    BUMPER_THICKNESS,
    CABIN_CUT_X_FRONT,
    CABIN_CUT_X_REAR,
    CABIN_FLOOR_Z,
    CABIN_SILL_Z,
    CAR_COLLECTION,
    DOORS_SPEC,
    FENDER_THICKNESS,
    FENDER_X_FRONT,
    FENDER_X_REAR,
    FRONT_BUMPER_WING_X,
    FRONT_BUMPER_X,
    FRONT_CLIP_X,
    HOOD_HINGE_X,
    HOOD_HINGE_Y,
    HOOD_LID_THICKNESS,
    HOOD_LIFT_BODY_XZ,
    HOOD_LIFT_CYL_LEN,
    HOOD_LIFT_ROD_LEN,
    HOOD_LIFT_Y,
    HOOD_OPEN_DEG,
    HOOD_X_REAR,
    LENGTH,
    N_SECTIONS,
    PILLAR_B_HALF_X,
    PILLAR_B_X,
    REAR_BUMPER_TOP_Z,
    REAR_BUMPER_WING_X,
    REAR_CLIP_X,
    REAR_FENDER_X_FRONT,
    REAR_FENDER_X_REAR,
    REAR_GLASS_X_BOT,
    ROOF_THICKNESS,
    ROOF_X_FRONT,
    ROOF_X_REAR,
    STUDIO_COLLECTION,
    TAIL_LIGHT_Y_IN,
    TAIL_LIGHT_Y_OUT,
    TAIL_LIGHT_Z_HI,
    TAIL_LIGHT_Z_LO,
    TARGET_COORDS,
    TRUNK_HINGE_X,
    TRUNK_HINGE_Y,
    TRUNK_LID_THICKNESS,
    TRUNK_LIFT_BODY_XZ,
    TRUNK_LIFT_CYL_LEN,
    TRUNK_LIFT_ROD_LEN,
    TRUNK_LIFT_Y,
    TRUNK_OPEN_DEG,
    TRUNK_TAIL_HALF_Y,
    TRUNK_X_FRONT,
    WHEEL_R,
    WHEEL_WIDTH,
    WHEEL_Y,
    WHEEL_Z,
    WIDTH,
    WINDSHIELD_X_BOT,
    WR,
    X_MAX,
    X_MIN,
    ZB,
    ZR,
    body_section,
    clamp,
    fender_bottom_z,
    fender_bumper_seam_z,
    front_nose_surface_x,
    hood_deck_z,
    hood_hinge_z,
    hood_lift_lid_local_xz,
    rear_fender_bottom_z,
    rear_fender_bumper_seam_z,
    rear_tail_surface_x,
    sanitize_name,
    side_surface_y,
    trunk_deck_z,
    trunk_hinge_z,
    trunk_lift_lid_local_xz,
    upper_side_yz,
)

TARGET = Vector(TARGET_COORDS)


# --------------------------------------------------------------------------- #
# 1. Коллекции и вспомогательные инструменты геометрии
# --------------------------------------------------------------------------- #
def get_or_create_collection(name):
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(col)
    return col


def new_object(name, mesh_data=None, collection=None):
    if mesh_data is None:
        mesh_data = bpy.data.meshes.new(name)
    ob = bpy.data.objects.new(name, mesh_data)
    target_col = collection if collection is not None else bpy.context.scene.collection
    target_col.objects.link(ob)
    return ob


def cleanup(ob, dist=5e-5):
    """Удаляет дубли вершин и вырожденные грани (< 3 вершин или площадь < 1e-12)."""
    if ob.type != 'MESH':
        return
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    degen = [f for f in bm.faces if len(f.verts) < 3 or f.calc_area() < 1e-12]
    if degen:
        bmesh.ops.delete(bm, geom=degen, context='FACES')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()


def _bevel_and_clean_bm(bm, bevel_width=0.0, bevel_segments=2, angle_deg=38.0):
    """Снимает аккуратную фаску с острых рёбер (сохраняя 100% manifold и 0 n-gon)."""
    if bevel_width > 0.0:
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        thr = math.radians(angle_deg)
        sharp_edges = [
            e for e in bm.edges
            if len(e.link_faces) == 2 and e.calc_face_angle(0.0) > thr
        ]
        if sharp_edges:
            bmesh.ops.bevel(
                bm,
                geom=sharp_edges,
                offset=bevel_width,
                offset_type='OFFSET',
                segments=bevel_segments,
                profile=0.55,
                affect='EDGES',
                clamp_overlap=True,
            )
            ngons = [f for f in bm.faces if len(f.verts) > 4]
            if ngons:
                bmesh.ops.triangulate(bm, faces=ngons)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=5e-5)
    degen = [f for f in bm.faces if len(f.verts) < 3 or f.calc_area() < 1e-12]
    if degen:
        bmesh.ops.delete(bm, geom=degen, context='FACES')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)


def apply_catmull_clark(ob, levels=1, auto_smooth_deg=55.0):
    """Запекает Catmull-Clark Subdivision в меш ob.data (чистые квады, 0 дефектов QA)."""
    if levels <= 0 or ob.type != 'MESH':
        return ob
    mod = ob.modifiers.new(name='Subsurf_Bake', type='SUBSURF')
    mod.subdivision_type = 'CATMULL_CLARK'
    mod.levels = levels
    mod.render_levels = levels
    mod.uv_smooth = 'PRESERVE_BOUNDARIES'
    deps = bpy.context.evaluated_depsgraph_get()
    ob_eval = ob.evaluated_get(deps)
    old_mesh = ob.data
    new_mesh = bpy.data.meshes.new_from_object(ob_eval)
    ob.modifiers.remove(mod)
    ob.data = new_mesh
    bpy.data.meshes.remove(old_mesh)
    new_mesh.name = ob.name

    bm = bmesh.new()
    bm.from_mesh(new_mesh)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=5e-5)
    degen = [f for f in bm.faces if len(f.verts) < 3 or f.calc_area() < 1e-12]
    if degen:
        bmesh.ops.delete(bm, geom=degen, context='FACES')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    max_angle = math.radians(auto_smooth_deg)
    for f in bm.faces:
        f.smooth = True
    for e in bm.edges:
        if len(e.link_faces) == 2:
            e.smooth = e.calc_face_angle(0.0) <= max_angle
        else:
            e.smooth = True
    bm.to_mesh(new_mesh)
    bm.free()
    new_mesh.update()
    return ob


def bmesh_to_object(name, bm, material=None, smooth=True, collection=None,
                    auto_smooth_deg=48.0, bevel=0.0, bevel_segs=2,
                    bevel_angle_deg=38.0, subsurf=0):
    """Конвертирует BMesh в объект сцены, опционально снимает фаску по острым рёбрам,
    применяет Catmull-Clark Subdivision (subsurf) и настраивает сглаживание (Blender 4.2 API)."""
    _bevel_and_clean_bm(bm, bevel_width=bevel, bevel_segments=bevel_segs,
                        angle_deg=bevel_angle_deg)
    if smooth:
        max_angle = math.radians(auto_smooth_deg)
        for f in bm.faces:
            f.smooth = True
        for e in bm.edges:
            if len(e.link_faces) == 2:
                e.smooth = e.calc_face_angle(0.0) <= max_angle
            else:
                e.smooth = True
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.update()
    ob = new_object(name, me, collection=collection)
    if material is not None:
        ob.data.materials.append(material)
    cleanup(ob, 5e-5)
    if subsurf > 0:
        apply_catmull_clark(ob, levels=subsurf, auto_smooth_deg=max(auto_smooth_deg, 55.0))
    return ob


def smooth_laplacian(bm, iters=2, factor=0.45):
    """Мягкое лапласово сглаживание вершин (ключ словаря — сам BMVert)."""
    for _ in range(iters):
        new_pos = {}
        for v in bm.verts:
            neigh = [e.other_vert(v) for e in v.link_edges]
            if neigh:
                avg = sum((n.co for n in neigh), Vector()) / float(len(neigh))
                new_pos[v] = v.co.lerp(avg, factor)
        for v, co in new_pos.items():
            v.co = co
    bm.normal_update()


def uv_top(ob):
    """Проекция UV сверху (u = X, v = Y) для кузова."""
    me = ob.data
    layer = me.uv_layers[0] if me.uv_layers else me.uv_layers.new(name='UVMap')
    uv = layer.data
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            uv[loop_idx].uv = (
                clamp((v.co.x - X_MIN) / LENGTH, 0.0, 1.0),
                clamp((v.co.y + WIDTH / 2.0) / WIDTH, 0.0, 1.0),
            )


def uv_cyl(ob, radius, width):
    """Цилиндрическая UV-развёртка вокруг оси Y для шин и дисков:
    u = угол вокруг оси вращения Y, v = координата по ширине шины [0, 1]."""
    me = ob.data
    layer = me.uv_layers[0] if me.uv_layers else me.uv_layers.new(name='UVMap')
    uv = layer.data
    half_w = max(width / 2.0, 1e-4)
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            angle = math.atan2(v.co.z, v.co.x)
            u = (angle + math.pi) / (2.0 * math.pi)
            # На боковинах кодируем радиус в крайние зоны v (<0.15 или >0.85),
            # на протекторе — ширину в центральную зону v ∈ [0.15, 0.85].
            r_frac = math.hypot(v.co.x, v.co.z) / max(radius, 1e-4)
            if abs(v.co.y) > half_w * 0.95 and r_frac < 0.96:
                vv = 0.05 + 0.10 * clamp(r_frac, 0.0, 1.0)
                if v.co.y > 0.0:
                    vv = 1.0 - vv
            else:
                vv = 0.15 + 0.70 * clamp((v.co.y + half_w) / (2.0 * half_w), 0.0, 1.0)
            uv[loop_idx].uv = (u, vv)


def uv_box(ob, scale=1.0):
    """Трипланарная бокс-проекция UV по нормали полигона для деталей салона и дверей."""
    me = ob.data
    layer = me.uv_layers[0] if me.uv_layers else me.uv_layers.new(name='UVMap')
    uv = layer.data
    for poly in me.polygons:
        nx, ny, nz = abs(poly.normal.x), abs(poly.normal.y), abs(poly.normal.z)
        for loop_idx in poly.loop_indices:
            co = me.vertices[me.loops[loop_idx].vertex_index].co
            if nz >= nx and nz >= ny:
                uv[loop_idx].uv = (co.x * scale, co.y * scale)
            elif ny >= nx:
                uv[loop_idx].uv = (co.x * scale, co.z * scale)
            else:
                uv[loop_idx].uv = (co.y * scale, co.z * scale)


def uv_face_x_minus(ob, mat_idx, y_left, y_right, z_bot, z_top):
    """Точный 1:1 UV-маппинг лицевой (-X) грани дисплея в [0..1]×[0..1]
    (слева направо при взгляде водителя вперёд по +X — от y_left (+Y) к y_right (-Y))."""
    me = ob.data
    layer = me.uv_layers[0] if me.uv_layers else me.uv_layers.new(name='UVMap')
    uv = layer.data
    dy = y_left - y_right
    dz = z_top - z_bot
    for poly in me.polygons:
        if poly.material_index == mat_idx and poly.normal.x < -0.5:
            for loop_idx in poly.loop_indices:
                co = me.vertices[me.loops[loop_idx].vertex_index].co
                u = clamp((y_left - co.y) / dy, 0.0, 1.0)
                v = clamp((co.z - z_bot) / dz, 0.0, 1.0)
                uv[loop_idx].uv = (u, v)


def uv_face_z_plus(ob, mat_idx, x_back, x_front, y_right, y_left):
    """Точный 1:1 UV-маппинг верхней (+Z) грани таблички под капотом в [0..1]×[0..1],
    читаемой при взгляде спереди (+X -> -X): слева y_right (-Y) -> справа y_left (+Y),
    снизу x_front -> сверху x_back."""
    me = ob.data
    layer = me.uv_layers[0] if me.uv_layers else me.uv_layers.new(name='UVMap')
    uv = layer.data
    dy = y_left - y_right
    dx = x_front - x_back
    for poly in me.polygons:
        if poly.material_index == mat_idx and poly.normal.z > 0.5:
            for loop_idx in poly.loop_indices:
                co = me.vertices[me.loops[loop_idx].vertex_index].co
                u = clamp((co.y - y_right) / dy, 0.0, 1.0)
                v = clamp((x_front - co.x) / dx, 0.0, 1.0)
                uv[loop_idx].uv = (u, v)


def look_at(obj, target):
    """Направляет локальную ось -Z объекта (камера / Area-лампа) на точку target."""
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def key_linear(ob, data_path, index, frame, value):
    """Задаёт значение свойства и ставит ключевой кадр с последующей линеаризацией."""
    ob.animation_data_create()
    prop = getattr(ob, data_path)
    prop[index] = value
    ob.keyframe_insert(data_path, index=index, frame=frame)


def linearize(ob):
    if ob.animation_data and ob.animation_data.action:
        for fc in ob.animation_data.action.fcurves:
            for kf in fc.keyframe_points:
                kf.interpolation = 'LINEAR'


# --------------------------------------------------------------------------- #
# 2. Материалы (все процедурные, по UV, совместимы с Blender 4.2)
# --------------------------------------------------------------------------- #
def _new_mix_rgba(nt, blend_type='MIX', fac=0.5):
    """Создаёт ShaderNodeMix (RGBA) и возвращает (node, fac_in, a_in, b_in, res_out)."""
    mix = nt.nodes.new('ShaderNodeMix')
    mix.data_type = 'RGBA'
    mix.blend_type = blend_type
    fac_in = mix.inputs[0]
    a_in = mix.inputs[6]
    b_in = mix.inputs[7]
    res_out = mix.outputs[2]
    fac_in.default_value = fac
    return mix, fac_in, a_in, b_in, res_out


def _bsdf_base(m, color, metallic, roughness, coat=0.0, coat_rough=0.05):
    bsdf = m.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (color[0], color[1], color[2], 1.0)
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    if coat:
        bsdf.inputs['Coat Weight'].default_value = coat
        bsdf.inputs['Coat Roughness'].default_value = coat_rough
    return bsdf


def mat_paint():
    """CarPaint_MetallicBlue: двухслойный автолак Volvo Denim Blue Metallic
    с угловым переливом (Layer Weight flop), микрохлопьями металлика и кристальным лаком."""
    m = bpy.data.materials.new('CarPaint_MetallicBlue')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.042, 0.102, 0.255), 0.78, 0.21, coat=1.0, coat_rough=0.016)
    bsdf.inputs['Coat IOR'].default_value = 1.56

    # Угловой цветовой перелив металлика (flop): глубже в лоб, сапфировый на касательных
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.inputs['Blend'].default_value = 0.28
    _, fac_in, a_in, b_in, col_out = _new_mix_rgba(nt, blend_type='MIX', fac=0.5)
    a_in.default_value = (0.030, 0.078, 0.215, 1.0)
    b_in.default_value = (0.095, 0.195, 0.410, 1.0)
    nt.links.new(lw.outputs['Facing'], fac_in)
    nt.links.new(col_out, bsdf.inputs['Base Color'])

    # Тончайшие металлические хлопья (микро-вариация шероховатости под лаком)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    flakes = nt.nodes.new('ShaderNodeTexVoronoi')
    flakes.inputs['Scale'].default_value = 1600.0
    flake_map = nt.nodes.new('ShaderNodeMapRange')
    flake_map.inputs['From Min'].default_value = 0.0
    flake_map.inputs['From Max'].default_value = 1.0
    flake_map.inputs['To Min'].default_value = 0.17
    flake_map.inputs['To Max'].default_value = 0.26
    nt.links.new(tc.outputs['Object'], flakes.inputs['Vector'])
    nt.links.new(flakes.outputs['Distance'], flake_map.inputs['Value'])
    nt.links.new(flake_map.outputs['Result'], bsdf.inputs['Roughness'])

    # Едва заметная «шагрень» на верхнем прозрачном слое лака
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 380.0
    noise.inputs['Detail'].default_value = 2.5
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.007
    bump.inputs['Distance'].default_value = 0.0008
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Coat Normal'])
    return m


def mat_white(color=(0.9, 0.9, 0.9), roughness=0.9):
    m = bpy.data.materials.new('White_Studio')
    m.use_nodes = True
    _bsdf_base(m, color, 0.0, roughness)
    return m


def mat_floor():
    """Сатиновый графитовый пол автостудии с мягким отражением кузова и колёс."""
    m = bpy.data.materials.new('Studio_Floor')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.145, 0.152, 0.168), 0.04, 0.24, coat=0.35, coat_rough=0.14)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 12.0
    noise.inputs['Detail'].default_value = 4.0
    rmap = nt.nodes.new('ShaderNodeMapRange')
    rmap.inputs['To Min'].default_value = 0.20
    rmap.inputs['To Max'].default_value = 0.30
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.012
    bump.inputs['Distance'].default_value = 0.002
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], rmap.inputs['Value'])
    nt.links.new(rmap.outputs['Result'], bsdf.inputs['Roughness'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    return m


def mat_glass():
    """Glass_Window: тонированное автомобильное стекло с чётким зеркальным отражением
    по Layer Weight и прозрачной внутренней гранью (без артефакта TIR на косых углах)."""
    m = bpy.data.materials.new('Glass_Window')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    geom = nt.nodes.new('ShaderNodeNewGeometry')
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.inputs['Blend'].default_value = 0.28
    trans = nt.nodes.new('ShaderNodeBsdfTransparent')
    trans.inputs['Color'].default_value = (0.34, 0.40, 0.44, 1.0)
    gloss = nt.nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Color'].default_value = (0.88, 0.93, 0.98, 1.0)
    gloss.inputs['Roughness'].default_value = 0.016
    front_mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(lw.outputs['Fresnel'], front_mix.inputs['Fac'])
    nt.links.new(trans.outputs[0], front_mix.inputs[1])
    nt.links.new(gloss.outputs[0], front_mix.inputs[2])
    # Внутренняя грань стекла чисто прозрачна — исключает ложное полное внутреннее отражение
    back_mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(geom.outputs['Backfacing'], back_mix.inputs['Fac'])
    nt.links.new(front_mix.outputs[0], back_mix.inputs[1])
    nt.links.new(trans.outputs[0], back_mix.inputs[2])
    nt.links.new(back_mix.outputs[0], out.inputs['Surface'])
    return m


def mat_lens():
    """Lens_Clear: кристально прозрачный поликарбонат фар и фонарей с мягким бликом
    по Layer Weight и чисто прозрачной внутренней гранью (LED видны под любым углом!)."""
    m = bpy.data.materials.new('Lens_Clear')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    geom = nt.nodes.new('ShaderNodeNewGeometry')
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.inputs['Blend'].default_value = 0.14
    scale_f = nt.nodes.new('ShaderNodeMath')
    scale_f.operation = 'MULTIPLY'
    scale_f.inputs[1].default_value = 0.55
    trans = nt.nodes.new('ShaderNodeBsdfTransparent')
    trans.inputs['Color'].default_value = (0.97, 0.98, 0.99, 1.0)
    gloss = nt.nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Color'].default_value = (0.96, 0.98, 1.0, 1.0)
    gloss.inputs['Roughness'].default_value = 0.015
    front_mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(lw.outputs['Fresnel'], scale_f.inputs[0])
    nt.links.new(scale_f.outputs['Value'], front_mix.inputs['Fac'])
    nt.links.new(trans.outputs[0], front_mix.inputs[1])
    nt.links.new(gloss.outputs[0], front_mix.inputs[2])
    # На внутренней (backfacing) грани линзы — чистая прозрачность, без чёрного TIR-зеркала!
    back_mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(geom.outputs['Backfacing'], back_mix.inputs['Fac'])
    nt.links.new(front_mix.outputs[0], back_mix.inputs[1])
    nt.links.new(trans.outputs[0], back_mix.inputs[2])
    nt.links.new(back_mix.outputs[0], out.inputs['Surface'])
    return m


def mat_rubber():
    """Rubber_Tire: протектор по цилиндрической UV (checker 150/13 + волна 5.5)
    и зерно боковины с маской по координате V."""
    m = bpy.data.materials.new('Rubber_Tire')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Base Color'].default_value = (0.022, 0.022, 0.024, 1.0)
    bsdf.inputs['Metallic'].default_value = 0.0

    tc = nt.nodes.new('ShaderNodeTexCoord')
    sep = nt.nodes.new('ShaderNodeSeparateXYZ')
    # Маска зоны протектора: v ∈ [0.15, 0.85] -> 1.0, боковина -> 0.0
    sub_center = nt.nodes.new('ShaderNodeMath')
    sub_center.operation = 'SUBTRACT'
    sub_center.inputs[1].default_value = 0.5
    abs_v = nt.nodes.new('ShaderNodeMath')
    abs_v.operation = 'ABSOLUTE'
    zone_mask = nt.nodes.new('ShaderNodeMapRange')
    zone_mask.inputs['From Min'].default_value = 0.30
    zone_mask.inputs['From Max'].default_value = 0.36
    zone_mask.inputs['To Min'].default_value = 1.0
    zone_mask.inputs['To Max'].default_value = 0.0

    mp = nt.nodes.new('ShaderNodeMapping')
    mp.inputs['Scale'].default_value = (150.0, 13.0, 1.0)
    chk = nt.nodes.new('ShaderNodeTexChecker')
    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.inputs['Scale'].default_value = 5.5

    side_noise = nt.nodes.new('ShaderNodeTexNoise')
    side_noise.inputs['Scale'].default_value = 120.0
    side_noise.inputs['Detail'].default_value = 5.0

    _, _, a_in, b_in, tread_out = _new_mix_rgba(nt, blend_type='MULTIPLY', fac=0.6)
    _, blend_fac, side_in, tread_in, final_pat = _new_mix_rgba(nt, blend_type='MIX', fac=0.5)

    ramp = nt.nodes.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].color = (0.45, 0.45, 0.45, 1.0)
    ramp.color_ramp.elements[1].color = (0.88, 0.88, 0.88, 1.0)

    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.25
    bump.inputs['Distance'].default_value = 0.006

    nt.links.new(tc.outputs['UV'], sep.inputs['Vector'])
    nt.links.new(sep.outputs['Y'], sub_center.inputs[0])
    nt.links.new(sub_center.outputs['Value'], abs_v.inputs[0])
    nt.links.new(abs_v.outputs['Value'], zone_mask.inputs['Value'])

    nt.links.new(tc.outputs['UV'], mp.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], chk.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], wave.inputs['Vector'])
    nt.links.new(tc.outputs['UV'], side_noise.inputs['Vector'])

    nt.links.new(chk.outputs['Color'], a_in)
    nt.links.new(wave.outputs['Fac'], b_in)

    nt.links.new(zone_mask.outputs['Result'], blend_fac)
    nt.links.new(side_noise.outputs['Fac'], side_in)
    nt.links.new(tread_out, tread_in)

    nt.links.new(final_pat, ramp.inputs['Fac'])
    nt.links.new(ramp.outputs['Color'], bsdf.inputs['Roughness'])
    nt.links.new(final_pat, bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return m


def mat_leather(name, color, scale=9.0, roughness=0.65):
    """Кожа / ткань салона: зерно по UV + мягкая вариация шероховатости вокруг roughness."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    base_rgba = (color[0], color[1], color[2], 1.0)

    tc = nt.nodes.new('ShaderNodeTexCoord')
    mp = nt.nodes.new('ShaderNodeMapping')
    mp.inputs['Scale'].default_value = (scale, scale, 1.0)
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 4.5
    noise.inputs['Detail'].default_value = 6.0

    _, _, a_in, b_in, col_out = _new_mix_rgba(nt, blend_type='SOFT_LIGHT', fac=0.30)
    a_in.default_value = base_rgba

    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.inputs['Scale'].default_value = 90.0
    rough_map = nt.nodes.new('ShaderNodeMapRange')
    rough_map.inputs['From Min'].default_value = 0.0
    rough_map.inputs['From Max'].default_value = 1.0
    rough_map.inputs['To Min'].default_value = clamp(roughness - 0.08, 0.15, 0.95)
    rough_map.inputs['To Max'].default_value = clamp(roughness + 0.08, 0.20, 0.98)

    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.08
    bump.inputs['Distance'].default_value = 0.002

    nt.links.new(tc.outputs['UV'], mp.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], noise.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], wave.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], b_in)
    nt.links.new(col_out, bsdf.inputs['Base Color'])
    nt.links.new(wave.outputs['Fac'], rough_map.inputs['Value'])
    nt.links.new(rough_map.outputs['Result'], bsdf.inputs['Roughness'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return m


def mat_leather_perforated(name, color, roughness=0.52):
    """Вентилируемая перфорированная кожа Nappa (Volvo S90 Inscription):
    зерно кожи + регулярная микро-перфорация Voronoi по Object-координатам."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Roughness'].default_value = roughness

    tc = nt.nodes.new('ShaderNodeTexCoord')
    # Регулярная сетка микро-отверстий вентиляции (Randomness = 0.0)
    vor = nt.nodes.new('ShaderNodeTexVoronoi')
    vor.inputs['Scale'].default_value = 145.0
    vor.inputs['Randomness'].default_value = 0.0
    hole = nt.nodes.new('ShaderNodeMapRange')
    hole.inputs['From Min'].default_value = 0.14
    hole.inputs['From Max'].default_value = 0.28
    hole.inputs['To Min'].default_value = 0.0
    hole.inputs['To Max'].default_value = 1.0

    # Мелкое зерно кожи поверх перфорации
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 55.0
    noise.inputs['Detail'].default_value = 5.0

    _, fac_in, a_in, b_in, col_out = _new_mix_rgba(nt, blend_type='MIX', fac=0.5)
    a_in.default_value = (color[0] * 0.38, color[1] * 0.36, color[2] * 0.35, 1.0)
    b_in.default_value = (color[0], color[1], color[2], 1.0)

    math_comb = nt.nodes.new('ShaderNodeMath')
    math_comb.operation = 'MULTIPLY'
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.14
    bump.inputs['Distance'].default_value = 0.0014

    nt.links.new(tc.outputs['Object'], vor.inputs['Vector'])
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(vor.outputs['Distance'], hole.inputs['Value'])
    nt.links.new(hole.outputs['Result'], fac_in)
    nt.links.new(col_out, bsdf.inputs['Base Color'])
    nt.links.new(hole.outputs['Result'], math_comb.inputs[0])
    nt.links.new(noise.outputs['Fac'], math_comb.inputs[1])
    nt.links.new(math_comb.outputs['Value'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return m


def mat_emission(name, strength, color=(1.0, 1.0, 1.0)):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    em = nt.nodes.new('ShaderNodeEmission')
    em.inputs['Color'].default_value = (color[0], color[1], color[2], 1.0)
    em.inputs['Strength'].default_value = strength
    nt.links.new(em.outputs['Emission'], out.inputs['Surface'])
    return m


def mat_carbon():
    """Матовый/сатиновый конструкционный автопластик (диффузор, подкрылки, жабо, моторный отсек)."""
    m = bpy.data.materials.new('Carbon_Trim')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.024, 0.026, 0.030), 0.06, 0.44)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 260.0
    noise.inputs['Detail'].default_value = 4.0
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.024
    bump.inputs['Distance'].default_value = 0.0015
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    return m


def mat_piano_black():
    """Чёрный рояльный лак (стойки B, окантовка решётки, внутренние отражатели фар)."""
    m = bpy.data.materials.new('Piano_Black')
    m.use_nodes = True
    _bsdf_base(m, (0.008, 0.009, 0.011), 0.06, 0.055, coat=1.0, coat_rough=0.012)
    return m


def mat_chrome():
    """Зеркальный хром (решётка, выхлоп, ручки дверей, штоки газовых лифтов)."""
    m = bpy.data.materials.new('Chrome_Steel')
    m.use_nodes = True
    _bsdf_base(m, (0.92, 0.94, 0.96), 1.0, 0.05)
    return m


def mat_alloy():
    """Фрезерованный серебристый алюминий (диски, впуск, КПП, распорка, усилитель бампера)."""
    m = bpy.data.materials.new('Alloy_Silver')
    m.use_nodes = True
    bsdf = _bsdf_base(m, (0.78, 0.80, 0.84), 0.95, 0.14, coat=0.5, coat_rough=0.04)
    bsdf.inputs['Anisotropic'].default_value = 0.30
    return m


def mat_brake_disc():
    """Шлифованная сталь тормозного диска с концентрической проточкой."""
    m = bpy.data.materials.new('Brake_Disc_Steel')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.62, 0.64, 0.67), 0.94, 0.26)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.wave_type = 'RINGS'
    wave.rings_direction = 'Y'
    wave.inputs['Scale'].default_value = 55.0
    wave.inputs['Detail'].default_value = 3.0
    rmap = nt.nodes.new('ShaderNodeMapRange')
    rmap.inputs['To Min'].default_value = 0.18
    rmap.inputs['To Max'].default_value = 0.34
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.04
    bump.inputs['Distance'].default_value = 0.0005
    nt.links.new(tc.outputs['Object'], wave.inputs['Vector'])
    nt.links.new(wave.outputs['Fac'], rmap.inputs['Value'])
    nt.links.new(rmap.outputs['Result'], bsdf.inputs['Roughness'])
    nt.links.new(wave.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    return m


def mat_tail_housing():
    """Тёмно-рубиновый лаковый корпус заднего фонаря под прозрачной линзой."""
    m = bpy.data.materials.new('Tail_Housing_Red')
    m.use_nodes = True
    _bsdf_base(m, (0.16, 0.008, 0.010), 0.12, 0.12, coat=1.0, coat_rough=0.02)
    return m


def mat_caliper():
    """Спортивный красный металлик для тормозных суппортов."""
    m = bpy.data.materials.new('Caliper_Red')
    m.use_nodes = True
    _bsdf_base(m, (0.62, 0.032, 0.032), 0.55, 0.20, coat=1.0, coat_rough=0.03)
    return m


def mat_bumper():
    """Bumper_Anthracite: контрастный глубокий графитовый сатин-металлик для съёмных
    переднего/заднего бамперов и накладок порогов (стиль Volvo S90 Cross Country / R-Design)."""
    m = bpy.data.materials.new('Bumper_Anthracite')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.042, 0.046, 0.056), 0.34, 0.25, coat=0.55, coat_rough=0.07)
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.inputs['Blend'].default_value = 0.26
    _, fac_in, a_in, b_in, col_out = _new_mix_rgba(nt, blend_type='MIX', fac=0.5)
    a_in.default_value = (0.028, 0.032, 0.042, 1.0)
    b_in.default_value = (0.082, 0.092, 0.112, 1.0)
    nt.links.new(lw.outputs['Facing'], fac_in)
    nt.links.new(col_out, bsdf.inputs['Base Color'])
    # Тонкая заводская микротекстура АБС-пластика/структурного покрытия бампера
    tc = nt.nodes.new('ShaderNodeTexCoord')
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 460.0
    noise.inputs['Detail'].default_value = 3.0
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.014
    bump.inputs['Distance'].default_value = 0.001
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    return m


def mat_wood():
    """Скандинавский открытопористый шпон Linear Walnut (горизонтальные волокна Volvo S90)."""
    m = bpy.data.materials.new('Wood_Walnut')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.14, 0.085, 0.052), 0.02, 0.38, coat=0.20, coat_rough=0.16)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.wave_type = 'BANDS'
    wave.bands_direction = 'Z'
    wave.inputs['Scale'].default_value = 9.0
    wave.inputs['Distortion'].default_value = 2.6
    wave.inputs['Detail'].default_value = 3.5
    _, fac_in, a_in, b_in, col_out = _new_mix_rgba(nt, blend_type='MIX', fac=0.5)
    a_in.default_value = (0.088, 0.054, 0.034, 1.0)
    b_in.default_value = (0.175, 0.112, 0.070, 1.0)
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.025
    bump.inputs['Distance'].default_value = 0.0010
    nt.links.new(tc.outputs['Object'], wave.inputs['Vector'])
    nt.links.new(wave.outputs['Fac'], fac_in)
    nt.links.new(col_out, bsdf.inputs['Base Color'])
    nt.links.new(wave.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    return m


# Встроенный компактный 5x7 пиксельный шрифт для чётких OLED-цифр/надписей приборки и Sensus
_FONT_5X7 = {
    '0': ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    '1': ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    '2': ("01110", "10001", "00001", "00110", "01000", "10000", "11111"),
    '3': ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    '4': ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    '5': ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    '6': ("01110", "10000", "11110", "10001", "10001", "10001", "01110"),
    '7': ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    '8': ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    '9': ("01110", "10001", "10001", "01111", "00001", "00010", "01100"),
    ':': ("00000", "00100", "00100", "00000", "00100", "00100", "00000"),
    '.': ("00000", "00000", "00000", "00000", "00000", "01100", "01100"),
    '-': ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
    '+': ("00000", "00100", "00100", "11111", "00100", "00100", "00000"),
    '/': ("00001", "00010", "00100", "00100", "01000", "10000", "00000"),
    '°': ("01100", "10010", "01100", "00000", "00000", "00000", "00000"),
    ' ': ("00000", "00000", "00000", "00000", "00000", "00000", "00000"),
    'A': ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    'B': ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    'C': ("01110", "10001", "10000", "10000", "10000", "10001", "01110"),
    'D': ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    'E': ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    'F': ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    'G': ("01110", "10001", "10000", "10111", "10001", "10001", "01110"),
    'H': ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    'I': ("01110", "00100", "00100", "00100", "00100", "00100", "01110"),
    'K': ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    'L': ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    'M': ("10001", "11011", "10101", "10001", "10001", "10001", "10001"),
    'N': ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    'O': ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    'P': ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    'R': ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    'S': ("01111", "10000", "01110", "00001", "00001", "10001", "01110"),
    'T': ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    'U': ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    'V': ("10001", "10001", "10001", "10001", "01010", "01010", "00100"),
    'W': ("10001", "10001", "10001", "10101", "10101", "11011", "10001"),
    'Y': ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
    '&': ("01100", "10010", "01100", "10101", "10010", "10010", "01101"),
}


def _ui_draw_text(buf, x0, y0, text, color, scale=3):
    h, w, _ = buf.shape
    cx = int(x0)
    for ch in text:
        glyph = _FONT_5X7.get(ch.upper(), _FONT_5X7[' '])
        for r, row in enumerate(glyph):
            for c, bit in enumerate(row):
                if bit == '1':
                    yy0 = max(0, int(y0) + r * scale)
                    yy1 = min(h, yy0 + scale)
                    xx0 = max(0, cx + c * scale)
                    xx1 = min(w, xx0 + scale)
                    buf[yy0:yy1, xx0:xx1, :3] = color
        cx += 6 * scale


def _ui_draw_line(buf, x0, y0, x1, y1, color, width=2):
    h, w, _ = buf.shape
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) * 2))
    ts = np.linspace(0.0, 1.0, n)
    xs = np.round(x0 + (x1 - x0) * ts).astype(np.int32)
    ys = np.round(y0 + (y1 - y0) * ts).astype(np.int32)
    r = max(0, width // 2)
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            px = np.clip(xs + dx, 0, w - 1)
            py = np.clip(ys + dy, 0, h - 1)
            buf[py, px, :3] = color


def _pack_ui_image(name, buf_topdown):
    """Создаёт bpy.data.images из float32 буфера (H,W,4) top-down и упаковывает в .blend."""
    h, w, _ = buf_topdown.shape
    img = bpy.data.images.get(name)
    if img is not None:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(name, width=w, height=h, alpha=False)
    img.colorspace_settings.name = 'sRGB'
    img.pixels.foreach_set(np.ascontiguousarray(buf_topdown[::-1, :, :]).ravel())
    img.pack()
    return img


def _build_cluster_ui_image():
    """Процедурная 12.3" OLED-графика приборной панели Volvo S90 (768×256):
    спидометр 72 км/ч, центральная карта Pilot Assist, знак 80, тахометр 2.1 rpm."""
    w, h = 768, 256
    buf = np.zeros((h, w, 4), dtype=np.float32)
    buf[:, :, 3] = 1.0
    yy, xx = np.mgrid[0:h, 0:w]
    # Глубокий ночной скандинавский OLED-фон
    grad = yy / float(h)
    buf[:, :, 0] = 0.015 + 0.015 * (1.0 - grad)
    buf[:, :, 1] = 0.032 + 0.035 * (1.0 - grad)
    buf[:, :, 2] = 0.068 + 0.060 * (1.0 - grad)

    def _draw_dial(cx, cy, r_out, val_frac, red_from=None):
        dist = np.hypot(xx - cx, yy - cy)
        # Тёмное внутреннее поле шкалы + тонкая голубовато-серебристая окантовка
        buf[dist < r_out, :3] = (0.010, 0.018, 0.034)
        ring = (dist >= r_out - 4.0) & (dist <= r_out)
        buf[ring, :3] = (0.28, 0.42, 0.58)
        # Угол по часовой стрелке от 135° (низ-лево) до 405° (низ-право) — диапазон 270°
        ang = (np.degrees(np.arctan2(yy - cy, xx - cx)) + 360.0) % 360.0
        rel = (ang - 135.0) % 360.0
        # Активная голубая дуга прогресса
        arc_m = (dist >= r_out - 15.0) & (dist <= r_out - 7.0) & (rel <= 270.0 * val_frac)
        buf[arc_m, :3] = (0.14, 0.62, 0.96)
        if red_from is not None:
            red_m = (
                (dist >= r_out - 16.0) & (dist <= r_out - 6.0)
                & (rel >= 270.0 * red_from) & (rel <= 270.0)
            )
            buf[red_m, :3] = (0.95, 0.12, 0.10)
        # 25 радиальных рисок
        for k in range(25):
            a_rad = math.radians(135.0 + 270.0 * k / 24.0)
            r0 = r_out - (20.0 if k % 4 == 0 else 12.0)
            r1 = r_out - 4.0
            col = (0.95, 0.18, 0.14) if (red_from and k / 24.0 >= red_from) else (0.86, 0.92, 0.98)
            _ui_draw_line(
                buf,
                cx + r0 * math.cos(a_rad), cy + r0 * math.sin(a_rad),
                cx + r1 * math.cos(a_rad), cy + r1 * math.sin(a_rad),
                col, width=3 if k % 4 == 0 else 1,
            )
        # Светящаяся рубиново-красная стрелка Volvo
        a_n = math.radians(135.0 + 270.0 * val_frac)
        _ui_draw_line(
            buf,
            cx - 10.0 * math.cos(a_n), cy - 10.0 * math.sin(a_n),
            cx + (r_out - 16.0) * math.cos(a_n), cy + (r_out - 16.0) * math.sin(a_n),
            (1.0, 0.14, 0.10), width=4,
        )
        buf[dist < 11.0, :3] = (0.16, 0.22, 0.30)

    # Левый циферблат: Спидометр (72 км/ч)
    _draw_dial(156, 130, 102, 0.28)
    _ui_draw_text(buf, 118, 104, "72", (0.96, 0.98, 1.0), scale=6)
    _ui_draw_text(buf, 124, 158, "KM/H", (0.55, 0.72, 0.88), scale=2)

    # Правый циферблат: Тахометр (2.1 rpm, красная зона с 0.78)
    _draw_dial(612, 130, 102, 0.26, red_from=0.78)
    _ui_draw_text(buf, 570, 108, "2.1", (0.96, 0.98, 1.0), scale=5)
    _ui_draw_text(buf, 586, 158, "RPM", (0.55, 0.72, 0.88), scale=2)

    # Центральная зона: 3D-карта Pilot Assist / Navigation
    buf[24:224, 286:482, :3] = (0.018, 0.042, 0.088)
    # 3D-перспектива трассы и светящийся голубой маршрут
    for row_y in range(55, 215):
        t = (row_y - 55) / 160.0
        half_rd = int(12 + 64 * t)
        half_ln = max(2, int(4 + 10 * t))
        buf[row_y, 384 - half_rd:384 + half_rd, :3] = (0.045, 0.085, 0.145)
        buf[row_y, 384 - half_ln:384 + half_ln, :3] = (0.12, 0.65, 0.98)
    # Белые линии разметки полосы
    _ui_draw_line(buf, 372, 55, 322, 214, (0.75, 0.86, 0.96), width=2)
    _ui_draw_line(buf, 396, 55, 446, 214, (0.75, 0.86, 0.96), width=2)
    # Знак ограничения скорости "80" в правом верхнем углу центральной зоны
    d_sign = np.hypot(xx - 454, yy - 54)
    buf[d_sign <= 22.0, :3] = (0.95, 0.12, 0.10)
    buf[d_sign <= 16.0, :3] = (0.96, 0.97, 0.98)
    _ui_draw_text(buf, 442, 47, "80", (0.05, 0.05, 0.07), scale=2)
    # Нижняя строка статуса: передача "D" и температура
    _ui_draw_text(buf, 376, 228, "D", (0.20, 0.92, 0.56), scale=3)
    _ui_draw_text(buf, 296, 232, "21°C", (0.68, 0.78, 0.88), scale=2)
    _ui_draw_text(buf, 424, 232, "12:45", (0.68, 0.78, 0.88), scale=2)
    return _pack_ui_image('Cluster_UI_Tex', buf)


def _build_sensus_ui_image():
    """Процедурная 9" вертикальная OLED-графика Volvo Sensus Connect (512×640):
    статус-бар, ночная навигационная карта, плеер B&W, телефон, двухзонный климат."""
    w, h = 512, 640
    buf = np.zeros((h, w, 4), dtype=np.float32)
    buf[:, :, 3] = 1.0
    yy, xx = np.mgrid[0:h, 0:w]
    buf[:, :, 0] = 0.018
    buf[:, :, 1] = 0.030
    buf[:, :, 2] = 0.052

    # 1. Статус-бар сверху (0..44)
    buf[0:44, :, :3] = (0.012, 0.020, 0.036)
    _ui_draw_text(buf, 18, 13, "12:45", (0.92, 0.95, 1.0), scale=3)
    _ui_draw_text(buf, 198, 15, "VOLVO", (0.52, 0.66, 0.82), scale=2)
    # Полоски сигнала 4G справа
    for kb in range(4):
        bh = 6 + kb * 4
        buf[30 - bh:30, 446 + kb * 10:452 + kb * 10, :3] = (0.30, 0.78, 1.0)

    # 2. Плитка 1: Ночная навигационная карта (50..274)
    buf[50:274, 14:498, :3] = (0.024, 0.054, 0.105)
    # Изогнутая река/вода на карте
    river = np.abs((xx - 340) - 45.0 * np.sin(yy / 42.0)) < 28.0
    buf[(yy >= 50) & (yy < 274) & (xx >= 14) & (xx < 498) & river, :3] = (0.035, 0.125, 0.245)
    # Сетка улиц и магистралей
    for gx in range(40, 490, 58):
        _ui_draw_line(buf, gx, 52, gx + 14, 272, (0.065, 0.12, 0.20), width=2)
    for gy in range(74, 268, 44):
        _ui_draw_line(buf, 16, gy, 496, gy - 10, (0.065, 0.12, 0.20), width=2)
    # Яркий маршрут Volvo Navigation (бирюзово-голубая линия + поворот направо)
    _ui_draw_line(buf, 224, 268, 224, 148, (0.12, 0.78, 1.0), width=8)
    _ui_draw_line(buf, 224, 148, 398, 128, (0.12, 0.78, 1.0), width=8)
    # Белый курсор автомобиля на маршруте
    d_car = np.hypot(xx - 224, yy - 214)
    buf[d_car <= 13.0, :3] = (0.12, 0.78, 1.0)
    buf[d_car <= 8.0, :3] = (0.98, 0.99, 1.0)
    # Верхняя подсказка маневра (450M)
    buf[58:102, 24:204, :3] = (0.015, 0.032, 0.062)
    _ui_draw_text(buf, 40, 70, "450M", (0.96, 0.98, 1.0), scale=3)

    # 3. Плитка 2: Медиа / Bowers & Wilkins (286..406)
    buf[286:406, 14:498, :3] = (0.030, 0.046, 0.074)
    # Квадратная мини-обложка альбома слева
    buf[302:390, 30:118, :3] = (0.14, 0.34, 0.56)
    d_alb = np.hypot(xx - 74, yy - 346)
    buf[(d_alb <= 30.0) & (yy >= 302) & (yy < 390), :3] = (0.85, 0.62, 0.24)
    buf[d_alb <= 10.0, :3] = (0.03, 0.05, 0.08)
    # Заголовок, полоса прогресса и кнопки плеера
    _ui_draw_text(buf, 138, 304, "B&W SOUND", (0.92, 0.95, 0.98), scale=3)
    buf[344:350, 138:468, :3] = (0.10, 0.16, 0.24)
    buf[344:350, 138:342, :3] = (0.18, 0.70, 1.0)
    buf[340:354, 338:348, :3] = (0.96, 0.98, 1.0)
    # Иконки управления: ◄◄  ❚❚  ►►
    buf[368:390, 268:276, :3] = (0.92, 0.95, 1.0)
    buf[368:390, 282:290, :3] = (0.92, 0.95, 1.0)
    _ui_draw_line(buf, 224, 379, 240, 369, (0.72, 0.80, 0.90), width=3)
    _ui_draw_line(buf, 224, 379, 240, 389, (0.72, 0.80, 0.90), width=3)
    _ui_draw_line(buf, 334, 379, 318, 369, (0.72, 0.80, 0.90), width=3)
    _ui_draw_line(buf, 334, 379, 318, 389, (0.72, 0.80, 0.90), width=3)

    # 4. Плитка 3: Drive Mode / Автомобиль (418..518)
    buf[418:518, 14:498, :3] = (0.026, 0.042, 0.068)
    _ui_draw_text(buf, 32, 434, "COMFORT", (0.86, 0.92, 0.98), scale=3)
    # 3 активные плитки-переключатели справа
    for k_t, col_t in enumerate(((0.14, 0.64, 0.96), (0.18, 0.78, 0.50), (0.16, 0.24, 0.36))):
        tx0 = 266 + k_t * 74
        buf[438:496, tx0:tx0 + 62, :3] = col_t
        buf[462:472, tx0 + 16:tx0 + 46, :3] = (0.95, 0.98, 1.0)

    # 5. Нижняя полоса климат-контроля (530..634)
    buf[530:634, 0:512, :3] = (0.014, 0.022, 0.038)
    _ui_draw_text(buf, 24, 558, "21.5°", (0.96, 0.98, 1.0), scale=4)
    _ui_draw_text(buf, 358, 558, "22.0°", (0.96, 0.98, 1.0), scale=4)
    # Иконка вентилятора по центру + индикаторы обогрева (красный) и обдува (синий)
    d_fan = np.hypot(xx - 256, yy - 580)
    buf[(d_fan >= 16.0) & (d_fan <= 22.0), :3] = (0.60, 0.76, 0.94)
    buf[d_fan <= 7.0, :3] = (0.92, 0.96, 1.0)
    buf[604:616, 48:116, :3] = (0.94, 0.28, 0.14)
    buf[604:616, 396:464, :3] = (0.16, 0.66, 0.98)
    return _pack_ui_image('Sensus_UI_Tex', buf)


def _mat_oled_image(name, image, emission_strength=1.55):
    """Глянцевый OLED-дисплей с упакованной растровой графикой и защитным стеклом."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.01, 0.015, 0.025), 0.04, 0.04, coat=1.0, coat_rough=0.015)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    tex = nt.nodes.new('ShaderNodeTexImage')
    tex.image = image
    tex.extension = 'CLIP'
    nt.links.new(tc.outputs['UV'], tex.inputs['Vector'])
    nt.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    if 'Emission Color' in bsdf.inputs:
        nt.links.new(tex.outputs['Color'], bsdf.inputs['Emission Color'])
        bsdf.inputs['Emission Strength'].default_value = emission_strength
    return m


def mat_screen_ui():
    """9" вертикальный OLED-дисплей Volvo Sensus Connect с реальной UI-графикой."""
    return _mat_oled_image('Screen_Sensus_UI', _build_sensus_ui_image(), emission_strength=1.35)


def mat_cluster_ui():
    """12.3" цифровая приборная панель Volvo с приборами и картой Pilot Assist."""
    return _mat_oled_image('Cluster_Digital_UI', _build_cluster_ui_image(), emission_strength=1.55)


def _build_engine_badge_image():
    """Генерирует и упаковывает табличку крышки двигателя Volvo Drive-E T6 AWD (512×256)."""
    w, h = 512, 256
    buf = np.zeros((h, w, 4), dtype=np.float32)
    buf[:, :, 3] = 1.0
    # Шлифованный тёмный титан с горизонтальными микролиниями
    yy, xx = np.mgrid[0:h, 0:w]
    brush = 0.014 * np.sin(yy * 0.9) + 0.008 * np.sin(yy * 2.7)
    buf[:, :, 0] = np.clip(0.085 + brush, 0.05, 0.14)
    buf[:, :, 1] = np.clip(0.092 + brush, 0.05, 0.15)
    buf[:, :, 2] = np.clip(0.105 + brush, 0.06, 0.17)
    # Серебряная рамка по периметру
    border = (xx < 12) | (xx >= w - 12) | (yy < 12) | (yy >= h - 12)
    buf[border, :3] = (0.72, 0.76, 0.80)
    inner_b = (
        ((xx >= 12) & (xx < 18)) | ((xx >= w - 18) & (xx < w - 12))
        | ((yy >= 12) & (yy < 18)) | ((yy >= h - 18) & (yy < h - 12))
    )
    buf[inner_b, :3] = (0.04, 0.045, 0.055)
    # Фирменная голубая полоса Volvo/Polestar под логотипом
    buf[118:130, 32:w - 32, :3] = (0.06, 0.56, 0.92)
    # Верхняя надпись VOLVO (крупно с разрядкой)
    _ui_draw_text(buf, 86, 36, "V O L V O", (0.94, 0.96, 0.99), scale=6)
    # Основной шильдик мотора DRIVE-E T6 AWD
    _ui_draw_text(buf, 48, 148, "DRIVE-E  T6  AWD", (0.92, 0.95, 0.98), scale=4)
    # Подпись спецификации
    _ui_draw_text(buf, 68, 198, "2.0L TURBO  316HP", (0.62, 0.72, 0.84), scale=3)
    return _pack_ui_image('Engine_Badge_Tex', buf)


def _build_engine_spec_image():
    """Генерирует и упаковывает сервисную табличку подкапотного пространства (384×128)."""
    w, h = 384, 128
    buf = np.zeros((h, w, 4), dtype=np.float32)
    buf[:, :, 3] = 1.0
    buf[:, :, :3] = (0.03, 0.032, 0.038)
    # Тонкая белая рамка
    buf[4:8, 4:w - 4, :3] = (0.85, 0.86, 0.88)
    buf[h - 8:h - 4, 4:w - 4, :3] = (0.85, 0.86, 0.88)
    buf[4:h - 4, 4:8, :3] = (0.85, 0.86, 0.88)
    buf[4:h - 4, w - 8:w - 4, :3] = (0.85, 0.86, 0.88)
    # Левая жёлтая зона предупреждения с чёрным треугольником и восклицательным знаком
    buf[10:h - 10, 10:112, :3] = (0.95, 0.76, 0.04)
    yy, xx = np.mgrid[0:h, 0:w]
    tri = (yy >= 24) & (yy <= 100) & (np.abs(xx - 61) <= (yy - 24) * 0.52)
    tri_in = (yy >= 34) & (yy <= 92) & (np.abs(xx - 61) <= (yy - 34) * 0.44)
    buf[tri & (~tri_in), :3] = (0.03, 0.03, 0.03)
    buf[48:74, 57:65, :3] = (0.03, 0.03, 0.03)
    buf[80:88, 57:65, :3] = (0.03, 0.03, 0.03)
    # Правая информационная часть
    _ui_draw_text(buf, 124, 18, "VOLVO CAR CORP", (0.95, 0.96, 0.98), scale=2)
    buf[40:44, 124:w - 16, :3] = (0.16, 0.52, 0.86)
    _ui_draw_text(buf, 124, 54, "A/C  R1234YF", (0.96, 0.80, 0.14), scale=2)
    _ui_draw_text(buf, 124, 80, "OIL  0W-20", (0.86, 0.90, 0.95), scale=2)
    _ui_draw_text(buf, 124, 104, "CAUTION FAN", (0.92, 0.28, 0.18), scale=2)
    return _pack_ui_image('Engine_Spec_Tex', buf)


def _mat_printed_plate(name, image, metallic=0.65, roughness=0.28, coat=0.35):
    """Металлическая или виниловая заводская табличка с упакованным изображением."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.12, 0.12, 0.13), metallic, roughness, coat=coat, coat_rough=0.10)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    tex = nt.nodes.new('ShaderNodeTexImage')
    tex.image = image
    tex.extension = 'CLIP'
    nt.links.new(tc.outputs['UV'], tex.inputs['Vector'])
    nt.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    return m


def mat_simple_plastic(name, color, roughness=0.32, coat=0.25):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    _bsdf_base(m, color, 0.0, roughness, coat=coat, coat_rough=0.15)
    return m


def make_materials():
    return {
        'paint': mat_paint(),
        'bumper': mat_bumper(),
        'glass': mat_glass(),
        'lens': mat_lens(),
        'rubber': mat_rubber(),
        'leather': mat_leather('Leather_Seat', (0.055, 0.050, 0.048), scale=10.0, roughness=0.56),
        'leather_light': mat_leather_perforated(
            'Leather_Bolster', (0.105, 0.098, 0.090), roughness=0.50,
        ),
        'wood': mat_wood(),
        'screen_ui': mat_screen_ui(),
        'cluster_ui': mat_cluster_ui(),
        'engine_badge': _mat_printed_plate(
            'Engine_Badge_Plate', _build_engine_badge_image(), metallic=0.72, roughness=0.26,
        ),
        'engine_spec': _mat_printed_plate(
            'Engine_Spec_Label', _build_engine_spec_image(),
            metallic=0.05, roughness=0.24, coat=0.5,
        ),
        'washer_blue': mat_simple_plastic('Washer_Cap_Blue', (0.015, 0.18, 0.68), roughness=0.26),
        'warning_yellow': mat_simple_plastic('Warning_Yellow', (0.86, 0.56, 0.015), roughness=0.28),
        'coolant_fluid': mat_simple_plastic(
            'Coolant_Fluid_Pink', (0.78, 0.09, 0.18), roughness=0.16, coat=0.6,
        ),
        'trunk_led': mat_emission('Trunk_LED', 6.5, (1.0, 0.95, 0.86)),
        'carpet': mat_leather('Carpet', (0.025, 0.022, 0.020), scale=42.0, roughness=0.85),
        'headliner': mat_leather('Headliner_Fabric', (0.085, 0.080, 0.075),
                                 scale=55.0, roughness=0.90),
        'headlight': mat_emission('Headlight_Light', 1.4, (1.0, 0.96, 0.88)),
        'drl': mat_emission('DRL_Light', 4.2, (0.94, 0.97, 1.0)),
        'amber_led': mat_emission('Amber_LED', 3.2, (1.0, 0.42, 0.04)),
        'tail': mat_emission('Tail_Light', 2.2, (0.96, 0.010, 0.006)),
        'tail_housing': mat_tail_housing(),
        'carbon': mat_carbon(),
        'piano_black': mat_piano_black(),
        'chrome': mat_chrome(),
        'alloy': mat_alloy(),
        'brake_disc': mat_brake_disc(),
        'caliper': mat_caliper(),
        'white': mat_white(),
        'floor': mat_floor(),
    }


# --------------------------------------------------------------------------- #
# 3. Детали на поверхности (ray_cast)
# --------------------------------------------------------------------------- #
def body_hit(y, z, front=True):
    """Луч вдоль X снаружи к кузову: front=True бьёт в нос (+X) со стороны +9 м,
    front=False бьёт в корму (-X) со стороны -9 м. Возвращает (точка, нормаль)."""
    deps = bpy.context.evaluated_depsgraph_get()
    ox = 9.0 if front else -9.0
    direction = Vector((-1.0 if front else 1.0, 0.0, 0.0))
    res, loc, normal, _idx, _obj, _mat = bpy.context.scene.ray_cast(
        deps, Vector((ox, y, z)), direction, distance=20.0)
    if res:
        return loc, normal
    return None, None


def top_hit(x, y):
    """Луч сверху вниз на поверхность кузова. Возвращает (точка, нормаль)."""
    deps = bpy.context.evaluated_depsgraph_get()
    res, loc, normal, _idx, _obj, _mat = bpy.context.scene.ray_cast(
        deps, Vector((x, y, 4.0)), Vector((0.0, 0.0, -1.0)), distance=8.0)
    if res:
        return loc, normal
    return None, None


def shell_patch(name, origin, normal, size=(0.4, 0.3), segs=(4, 3),
                off=0.0, thickness=0.008, material=None, project=True,
                collection=None):
    """Тонкая замкнутая накладка на поверхности. off > 0 — смещение наружу по нормали."""
    if project:
        hit, nrm = body_hit(origin.y, origin.z, front=(origin.x >= 0.0))
        if hit is not None:
            origin = hit
            normal = nrm
    normal = normal.normalized()
    ref = Vector((0.0, 0.0, 1.0)) if abs(normal.z) < 0.9 else Vector((1.0, 0.0, 0.0))
    t1 = normal.cross(ref).normalized()
    t2 = normal.cross(t1).normalized()

    bm = bmesh.new()
    grids = []
    for sgn in (1.0, -1.0):
        layer = []
        for j in range(segs[1]):
            row = []
            for i in range(segs[0]):
                u = (i / (segs[0] - 1) - 0.5) * size[0]
                v = (j / (segs[1] - 1) - 0.5) * size[1]
                p = (origin + normal * off + t1 * u + t2 * v
                     + normal * sgn * thickness * 0.5)
                row.append(bm.verts.new(p))
            layer.append(row)
        grids.append(layer)

    # Внешняя сторона (grids[0]) и внутренняя (grids[1] с обратным обходом)
    for j in range(segs[1] - 1):
        for i in range(segs[0] - 1):
            bm.faces.new((grids[0][j][i], grids[0][j][i + 1],
                          grids[0][j + 1][i + 1], grids[0][j + 1][i]))
            bm.faces.new((grids[1][j][i], grids[1][j + 1][i],
                          grids[1][j + 1][i + 1], grids[1][j][i + 1]))

    # Замыкающие стенки по периметру -> замкнутая оболочка
    for j in range(segs[1] - 1):
        bm.faces.new((grids[0][j][0], grids[0][j + 1][0],
                      grids[1][j + 1][0], grids[1][j][0]))
        bm.faces.new((grids[0][j][segs[0] - 1], grids[1][j][segs[0] - 1],
                      grids[1][j + 1][segs[0] - 1], grids[0][j + 1][segs[0] - 1]))
    for i in range(segs[0] - 1):
        bm.faces.new((grids[0][0][i], grids[1][0][i],
                      grids[1][0][i + 1], grids[0][0][i + 1]))
        bm.faces.new((grids[0][segs[1] - 1][i], grids[0][segs[1] - 1][i + 1],
                      grids[1][segs[1] - 1][i + 1], grids[1][segs[1] - 1][i]))

    ob = bmesh_to_object(name, bm, material=material, smooth=True, collection=collection)

    # UV по локальным осям t1, t2 относительно центра накладки origin
    me = ob.data
    layer = me.uv_layers.new(name='UVMap')
    uv = layer.data
    center = origin + normal * off
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            rel = me.vertices[me.loops[loop_idx].vertex_index].co - center
            uv[loop_idx].uv = (
                clamp(rel.dot(t1) / max(size[0], 1e-4) + 0.5, 0.0, 1.0),
                clamp(rel.dot(t2) / max(size[1], 1e-4) + 0.5, 0.0, 1.0),
            )
    return ob


# --------------------------------------------------------------------------- #
# 4. Кузов и остекление
# --------------------------------------------------------------------------- #
def build_body(mats, collection):
    """Строит замкнутую силовую капсулу монокока Body от задней перегородки салона
    (REAR_CLIP_X) до моторного щита (FRONT_CLIP_X): сквозные дверные проёмы, пороги и пол.
    Передние и задние крылья, оба бампера, капот, багажник и силовые каркасы передка/кормы
    строятся отдельными узлами, как на реальном автомобиле."""
    bm = bmesh.new()
    n_sec = max(84, int(N_SECTIONS * 0.45))
    xs = [REAR_CLIP_X + (FRONT_CLIP_X - REAR_CLIP_X) * i / (n_sec - 1) for i in range(n_sec)]
    loops = [body_section(x) for x in xs]
    vcount = len(loops[0])
    verts = []
    for x, loop in zip(xs, loops):
        for (y, z) in loop:
            verts.append(bm.verts.new(Vector((x, y, z))))

    for i in range(n_sec - 1):
        base = i * vcount
        nbase = (i + 1) * vcount
        for k in range(vcount):
            bm.faces.new((
                verts[base + k],
                verts[base + (k + 1) % vcount],
                verts[nbase + (k + 1) % vcount],
                verts[nbase + k],
            ))

    # Замыкание задней стенки монокока (перегородка салона на REAR_CLIP_X)
    zm_r = 0.5 * (ZB(REAR_CLIP_X) + BELT(REAR_CLIP_X))
    center_r = bm.verts.new(Vector((REAR_CLIP_X - 0.004, 0.0, zm_r)))
    for k in range(vcount):
        kn = (k + 1) % vcount
        bm.faces.new((center_r, verts[k], verts[kn]))

    # Замыкание передней стенки монокока (моторный щит на FRONT_CLIP_X)
    last_base = (n_sec - 1) * vcount
    zm_f = 0.5 * (ZB(FRONT_CLIP_X) + BELT(FRONT_CLIP_X))
    center_f = bm.verts.new(Vector((FRONT_CLIP_X + 0.004, 0.0, zm_f)))
    for k in range(vcount):
        kn = (k + 1) % vcount
        bm.faces.new((center_f, verts[last_base + kn], verts[last_base + k]))

    smooth_laplacian(bm, iters=1, factor=0.14)
    ob = bmesh_to_object('Body', bm, material=mats['paint'], smooth=True,
                         collection=collection, auto_smooth_deg=55.0,
                         bevel=0.003, bevel_segs=2, subsurf=1)
    uv_top(ob)
    return ob


def build_front_structure_and_panels(body_ob, mats, collection):
    """Строит разделённую переднюю часть автомобиля по реальной заводской схеме:
      1) Несущий каркас: Front_Subframe (лонжероны, моторный щит, подрамник, краш-боксы),
         Crash_Beam (алюминиевый усилитель бампера), Fender_Apron_L/R (брызговики с чашками
         стоек и полками крыльев), Radiator_Support («телевизор» с замком капота);
      2) Отсек капота: Cowl_Panel (жабо под лобовым стеклом, уплотнитель, стеклоочистители);
      3) Отдельные внешние навесные панели с реальными зазорами ~6 мм:
         Fender_Front_L / Fender_Front_R (передние крылья с вырезом арки и верхним фланцем)
         и Front_Bumper (отдельный съёмный передний бампер с боковыми крылышками)."""
    objs = []

    def _attach(ob):
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(ob)
        return ob

    # 1. Несущий силовой каркас передка (Front_Subframe: моторный щит, лонжероны, подрамник)
    bm_sf = bmesh.new()
    # Моторный щит (перегородка салона и моторного отсека)
    add_box(bm_sf, FRONT_CLIP_X + 0.005, 1.15, -0.74, 0.74, 0.22, 0.84)
    # Продольные силовые лонжероны (левый и правый) + передние краш-боксы
    for side in (1.0, -1.0):
        ya, yb = (side * 0.43, side * 0.53)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_sf, 1.15, 2.22, y0, y1, 0.27, 0.39)
        # Энергопоглощающий краш-бокс к усилителю бампера
        add_box(bm_sf, 2.222, 2.315, y0 + 0.008, y1 - 0.008, 0.29, 0.39)
    # Нижний подрамник двигателя и силовые поперечины
    add_box(bm_sf, 1.24, 1.98, -0.43, 0.43, 0.215, 0.265)
    add_box(bm_sf, 1.24, 1.34, -0.53, 0.53, 0.268, 0.320)
    add_box(bm_sf, 1.92, 2.02, -0.53, 0.53, 0.268, 0.320)
    sf_ob = bmesh_to_object('Front_Subframe', bm_sf, material=mats['carbon'],
                            smooth=True, collection=collection, bevel=0.003)
    uv_box(sf_ob, scale=2.0)
    _attach(sf_ob)

    # 2. Изогнутая алюминиевая балка усилителя переднего бампера (Crash_Beam — строго
    # внутри оболочки съёмного Front_Bumper с запасом >=25 мм, видна при снятии бампера)
    bm_cb = bmesh.new()
    cb_out, cb_in = [], []
    for j in range(3):
        zz = 0.325 + 0.110 * (j / 2.0)
        ro, ri = [], []
        for i in range(9):
            yy = -0.56 + 1.12 * (i / 8.0)
            xf = front_nose_surface_x(yy) - 0.072
            ro.append(Vector((xf, yy, zz)))
            ri.append(Vector((xf - 0.048, yy, zz)))
        cb_out.append(ro)
        cb_in.append(ri)
    _add_thick_grid(bm_cb, cb_out, cb_in)
    cb_ob = bmesh_to_object('Crash_Beam', bm_cb, material=mats['alloy'],
                            smooth=True, collection=collection, bevel=0.004)
    uv_box(cb_ob, scale=4.0)
    _attach(cb_ob)

    # 3. Верхние брызговики / полки крыльев с опорными чашками стоек (Fender_Apron_L/R)
    for label, side in (('L', 1.0), ('R', -1.0)):
        bm_ap = bmesh.new()
        # Продольная силовая полка брызговика (повторяет наклон капота от щита к «телевизору»)
        n_ap = 10
        ap_rings = []
        for i in range(n_ap):
            x = 1.02 + (2.18 - 1.02) * i / (n_ap - 1)
            z_top = BELT(x) - 0.036
            z_bot = z_top - 0.16
            y_in = side * (WR(x) - 0.065)
            y_out = side * (WR(x) + 0.045)
            ya, yb = (y_in, y_out) if y_in < y_out else (y_out, y_in)
            ap_rings.append([
                bm_ap.verts.new(Vector((x, ya, z_bot))),
                bm_ap.verts.new(Vector((x, yb, z_bot))),
                bm_ap.verts.new(Vector((x, yb, z_top))),
                bm_ap.verts.new(Vector((x, ya, z_top))),
            ])
        for i in range(n_ap - 1):
            r0, r1 = ap_rings[i], ap_rings[i + 1]
            for k in range(4):
                kn = (k + 1) % 4
                bm_ap.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        bm_ap.faces.new((ap_rings[0][0], ap_rings[0][3], ap_rings[0][2], ap_rings[0][1]))
        bm_ap.faces.new((ap_rings[-1][0], ap_rings[-1][1], ap_rings[-1][2], ap_rings[-1][3]))

        # Опорная чашка передней амортизационной стойки на оси AXLE_F = +1.60
        _add_z_cylinder(bm_ap, 0.095, 0.50, 0.79, seg=20, center_xy=(AXLE_F, side * 0.54))
        # Резиновый отбойник капота на переднем конце брызговика
        _add_z_cylinder(bm_ap, 0.014, 0.77, 0.805, seg=12, center_xy=(2.08, side * 0.54))
        ap_ob = bmesh_to_object(f'Fender_Apron_{label}', bm_ap, material=mats['carbon'],
                                smooth=True, collection=collection, bevel=0.003)
        uv_box(ap_ob, scale=3.0)
        _attach(ap_ob)

    # 4. «Телевизор» — передняя рамка радиатора и замок капота (Radiator_Support)
    bm_rs = bmesh.new()
    # Верхняя поперечина «телевизора» (соединяет левый и правый брызговики за решёткой)
    add_box(bm_rs, 2.13, 2.245, -0.62, 0.62, 0.700, 0.745)
    # Нижняя траверса под радиатор
    add_box(bm_rs, 2.10, 2.22, -0.54, 0.54, 0.245, 0.290)
    # Боковые вертикальные стойки «телевизора» вокруг радиатора
    for side in (1.0, -1.0):
        ya, yb = (side * 0.46, side * 0.54)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_rs, 2.12, 2.21, y0, y1, 0.292, 0.698)
    # Центральный замок капота на верхней балке
    add_box(bm_rs, 2.16, 2.24, -0.055, 0.055, 0.747, 0.774)
    # Боковые опорные кронштейны/ниши крепления передних фар на кузове (фары сидят на кузове!)
    for side in (1.0, -1.0):
        ya, yb = (side * 0.435, side * 0.705)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        # Основная монтажная полка-ниша под фарой (опирается на рамку радиатора и брызговик)
        add_box(bm_rs, 2.18, 2.310, y0, y1, 0.532, 0.548)
        # Задняя опорная стенка чаши фары
        add_box(bm_rs, 2.18, 2.240, y0, y1, 0.550, 0.672)
        # Верхнее крепёжное ухо фары к верхнему углу «телевизора»
        add_box(bm_rs, 2.21, 2.285, y0 + 0.03, y1 - 0.03, 0.674, 0.692)
    rs_ob = bmesh_to_object('Radiator_Support', bm_rs, material=mats['carbon'],
                            smooth=True, collection=collection, bevel=0.003)
    uv_box(rs_ob, scale=3.0)
    _attach(rs_ob)

    # 5. Жабо (короб водостока под лобовым стеклом), уплотнитель капота и дворники (Cowl_Panel)
    bm_cw = bmesh.new()
    z_cw = BELT(1.04) - 0.012
    # Основная пластиковая панель жабо от основания лобового стекла к моторному щиту
    add_box(bm_cw, FRONT_CLIP_X + 0.006, 1.155, -0.78, 0.78, z_cw - 0.042, z_cw)
    # Резиновый уплотнитель по передней кромке жабо (отделяет жабо от моторного отсека)
    add_box(bm_cw, 1.145, 1.168, -0.68, 0.68, z_cw + 0.002, z_cw + 0.018)
    # Два поводка и щётки стеклоочистителей на жабо
    for wy0, wy1 in ((-0.58, -0.04), (0.04, 0.58)):
        add_box(bm_cw, 0.99, 1.015, wy0, wy1, z_cw + 0.004, z_cw + 0.018)
    cw_ob = bmesh_to_object('Cowl_Panel', bm_cw, material=mats['carbon'],
                            smooth=True, collection=collection, bevel=0.0025)
    uv_box(cw_ob, scale=4.0)
    _attach(cw_ob)

    # 6. ОТДЕЛЬНЫЕ передние крылья (Fender_Front_L / Fender_Front_R) с вырезом арки и фланцем
    # Плотный набор X-сечений: плавное сгущение на переходе порог -> арка и к стыку с бампером
    fender_xs = sorted({
        FENDER_X_REAR, 0.96, 1.01, 1.07, 1.13, 1.175, 1.200, 1.212, 1.225,
        *[1.24 + (1.988 - 1.24) * k / 24.0 for k in range(25)],
        2.02, 2.08, 2.15, 2.22, FENDER_X_FRONT,
    })
    for label, side in (('L', 1.0), ('R', -1.0)):
        bm_f = bmesh.new()
        outer_pts = []
        inner_pts = []
        for x in fender_xs:
            zb_f = fender_bottom_z(x)
            belt_x = BELT(x)
            y_belt = side_surface_y(x, belt_x)
            y_hood_edge = WR(x) + 0.050  # зазор ~6 мм до края капота (WR + 0.044)
            # Плавный передний спад в точном согласовании с передним клювом Hood_Lid!
            u_hx = clamp(
                (x - (HOOD_X_REAR + 0.006)) / max(FENDER_X_FRONT - HOOD_X_REAR, 0.1),
                0.0, 1.0,
            )
            f_nose_drop = 0.064 * (max(0.0, (u_hx - 0.65) / 0.35) ** 1.65)
            z_hood_edge = hood_deck_z(x, y_hood_edge) - 0.008 - f_nose_drop
            y_flange = WR(x) + 0.004     # загнутая внутрь монтажная полка под капотом
            z_flange = z_hood_edge - 0.034

            # 12 точек по профилю сечения крыла:
            # 0..6 — борт от нижней кромки/арки до плеча BELT (строго по side_surface_y,
            #        без искусственного замятия за аркой -> идеально заподлицо с Door_Front!);
            # 7..9 — скруглённый верхний гребень крыла к линии капота;
            # 10..11 — загнутый внутрь монтажный фланец крыла (виден при открытом капоте!)
            # Плавное скругление и опускание переднего угла крыла (x > 2.02) точно к верху фары
            t_fcorn = clamp((x - 2.02) / max(FENDER_X_FRONT - 2.02, 0.05), 0.0, 1.0)
            t_fcorn_s = t_fcorn * t_fcorn * (3.0 - 2.0 * t_fcorn)
            y_ftuck = 1.0 - 0.036 * (t_fcorn ** 1.85)
            z_fdrop = 0.074 * t_fcorn_s
            profile = []
            for j in range(7):
                u = j / 6.0
                zz = zb_f + (belt_x - z_fdrop - zb_f) * u
                yy = side_surface_y(x, zz) * y_ftuck
                profile.append((yy, zz))
            for k in range(1, 4):
                u = k / 3.0
                yy = (y_belt * y_ftuck) * (1.0 - u) + y_hood_edge * u
                zz = (
                    (belt_x - z_fdrop)
                    + (z_hood_edge - (belt_x - z_fdrop)) * (u ** 0.72)
                    + 0.005 * math.sin(math.pi * u)
                )
                profile.append((yy, zz))
            # Загнутый внутрь фланец крепления крыла на брызговике
            profile.append((y_hood_edge - 0.008, z_flange + 0.010))
            profile.append((y_flange, z_flange))

            row_out = []
            row_in = []
            # На переднем участке крыла (x > 2.06) плечо и верхняя бровь над фарой плавно
            # повторяют дугу носа front_nose_surface_x(yy) в одну непрерывную линию с Hood_Lid
            # — без резкого вертикального «обрыва» и без ступеньки над наружным углом фары!
            t_nose_wrap = clamp((x - 2.06) / max(FENDER_X_FRONT - 2.06, 0.05), 0.0, 1.0)
            t_nose_wrap = t_nose_wrap * t_nose_wrap * (3.0 - 2.0 * t_nose_wrap)
            for idx_p, (yy, zz) in enumerate(profile):
                x_loc = x
                if t_nose_wrap > 0.0 and idx_p >= 4:
                    x_arc = front_nose_surface_x(yy) - 0.028
                    if x_arc > x:
                        # Полный выход на дугу капота у стыка (idx_p >= 8 -> 0 ступеньки!)
                        w_top = clamp((idx_p - 4) / 4.0, 0.0, 1.0)
                        w_top = w_top * w_top * (3.0 - 2.0 * w_top)
                        x_loc = x + (x_arc - x) * t_nose_wrap * (0.20 + 0.80 * w_top)
                if idx_p < 7:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS, 0.0))
                elif idx_p < 10:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS * 0.55,
                                    -FENDER_THICKNESS * 0.85))
                else:
                    n_off = Vector((0.0, 0.0, -FENDER_THICKNESS * 0.80))
                p_o = Vector((x_loc, side * yy, zz))
                row_out.append(p_o)
                row_in.append(p_o + n_off)
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm_f, outer_pts, inner_pts)

        f_ob = bmesh_to_object(f'Fender_Front_{label}', bm_f, material=mats['paint'],
                               smooth=True, collection=collection, auto_smooth_deg=55.0,
                               bevel=0.003, bevel_segs=2, subsurf=1)
        uv_box(f_ob, scale=1.2)
        _attach(f_ob)

    # 7. ОТДЕЛЬНЫЙ СЪЁМНЫЙ передний бампер (Front_Bumper) контрастного антрацитового цвета:
    # единая скульптурная U-образная оболочка с плавно скруглёнными передними углами,
    # чётким 7-мм швом от крыльев и капота, вырезами под фары и крепёжными кронштейнами!
    bm_bmp = bmesh.new()
    u_front = []
    y_fcorner = side_surface_y(FENDER_X_FRONT, 0.55) * 0.964
    r_fcorn = 0.155  # плавный автомобильный радиус скругления передних углов в плане XY
    x_fwing_end = FENDER_X_FRONT - r_fcorn
    y_fspan = y_fcorner - r_fcorn
    x_front_edge = front_nose_surface_x(y_fspan)

    def _front_bumper_top_z(yy):
        """Верхняя кромка переднего бампера: 0.716 точно за верхом хром-рамки решётки
        (без торчащей серой полки!), плавная щёчка на |y|=0.39..0.422 к низу фары 0.554."""
        ay = abs(yy)
        t_in = clamp((ay - 0.392) / 0.030, 0.0, 1.0)
        t_in = t_in * t_in * (3.0 - 2.0 * t_in)
        # Под наружным углом трапециевидной фары кромка мягко поднимается к шву крыла
        t_out_rise = clamp((ay - 0.44) / 0.32, 0.0, 1.0)
        z_hl_bot = 0.554 + 0.012 * (t_out_rise ** 1.3)
        return 0.716 * (1.0 - t_in) + z_hl_bot * t_in

    # 7a. Правое крылышко (-Y): от переднего края арки вперёд к правому углу
    n_fw = 8
    for i in range(n_fw):
        t = i / float(n_fw)
        x = FRONT_BUMPER_WING_X + (x_fwing_end - FRONT_BUMPER_WING_X) * t
        z_lo = ZB(x) + 0.036
        z_hi = fender_bumper_seam_z(x) - 0.007
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_front.append((x, -y_abs, z_lo, z_hi, Vector((0.0, -1.0, 0.0)), 0.0))

    # 7b. Правый скруглённый угол (-Y -> +X): верх точно по шву fender_bumper_seam_z(xx) -> 0 щелей!
    for k in range(6):
        ang = (k / 5.0) * (0.5 * math.pi)
        xx = x_fwing_end + (x_front_edge - x_fwing_end) * math.sin(ang)
        yy = y_fspan + r_fcorn * math.cos(ang)
        t_c = k / 5.0
        z_lo = ZB(xx) + 0.034
        z_hi = fender_bumper_seam_z(min(xx, FENDER_X_FRONT)) - 0.006
        n_dir = Vector((math.sin(ang), -math.cos(ang), 0.0)).normalized()
        u_front.append((xx, -yy, z_lo, z_hi, n_dir, t_c))

    # 7c. Центральная передняя дуга от правого угла к левому (выпуклая по front_nose_surface_x)
    n_fmid = 19
    for i in range(1, n_fmid - 1):
        v = -1.0 + 2.0 * i / float(n_fmid - 1)
        yy = v * y_fspan
        xx = front_nose_surface_x(yy)
        z_lo = ZB(xx) + 0.030
        z_hi = _front_bumper_top_z(yy)
        n_dir = Vector((1.0, 0.36 * v, 0.0)).normalized()
        u_front.append((xx, yy, z_lo, z_hi, n_dir, 1.0))

    # 7d. Левый скруглённый угол (+X -> +Y)
    for k in range(6):
        ang = (1.0 - k / 5.0) * (0.5 * math.pi)
        xx = x_fwing_end + (x_front_edge - x_fwing_end) * math.sin(ang)
        yy = y_fspan + r_fcorn * math.cos(ang)
        t_c = 1.0 - k / 5.0
        z_lo = ZB(xx) + 0.034
        z_hi = fender_bumper_seam_z(min(xx, FENDER_X_FRONT)) - 0.006
        n_dir = Vector((math.sin(ang), math.cos(ang), 0.0)).normalized()
        u_front.append((xx, +yy, z_lo, z_hi, n_dir, t_c))

    # 7e. Левое крылышко (+Y): от угла назад к переднему краю арки
    for i in range(1, n_fw + 1):
        t = i / float(n_fw)
        x = x_fwing_end + (FRONT_BUMPER_WING_X - x_fwing_end) * t
        z_lo = ZB(x) + 0.036
        z_hi = fender_bumper_seam_z(x) - 0.007
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_front.append((x, +y_abs, z_lo, z_hi, Vector((0.0, 1.0, 0.0)), 0.0))

    fb_out = []
    fb_in = []
    nz_fb = 8
    for xx, yy, z_lo, z_hi, n_dir, front_w in u_front:
        row_o = []
        row_i = []
        side_s = 1.0 if yy >= 0.0 else -1.0
        w_blend = front_w * front_w * (3.0 - 2.0 * front_w)
        for j in range(nz_fb):
            u = j / float(nz_fb - 1)
            zz = z_lo + (z_hi - z_lo) * u
            # На боковых крылышках (front_w < 1) вычисляем точную side_surface_y(xx, zz)
            # на каждой высоте zz + компенсация subsurf-стяжки -> шов крыло <-> бампер ~1-2 мм!
            if front_w < 0.99:
                y_true = side_s * side_surface_y(xx, zz)
                t_fc = clamp((xx - 2.02) / max(FENDER_X_FRONT - 2.02, 0.05), 0.0, 1.0)
                y_true *= (1.0 - 0.036 * (t_fc ** 1.85))
                y_loc = y_true * (1.0 - w_blend) + yy * w_blend
            else:
                y_loc = yy
            # Автомобильная вертикальная пластика бампера:
            # спортивная нижняя губа (+12 мм) -> ниша воздухозаборников (-8 мм) ->
            # упругий защитный пояс (смягчённый под фарами) -> плавный верхний скос назад
            lip = 0.012 * ((1.0 - u) ** 3.2)
            intake_cove = (
                -0.008 * math.sin(math.pi * clamp((u - 0.12) / 0.34, 0.0, 1.0))
                if 0.12 < u < 0.46 else 0.0
            )
            # Под фарами (|y| > 0.40) смягчаем «раздутость» защитного пояса для чистой стыковки
            side_calm = 1.0 - 0.45 * clamp((abs(yy) - 0.40) / 0.28, 0.0, 1.0)
            belt_bulge = (
                0.013 * side_calm * math.sin(math.pi * clamp((u - 0.42) / 0.46, 0.0, 1.0))
                if 0.42 < u < 0.88 else 0.0
            )
            top_slope = -0.022 * (max(0.0, (u - 0.72) / 0.28) ** 1.7)
            # Боковой подгиб внизу + компенсация Catmull-Clark стяжки у верхнего шва крылышка
            side_tuck = (
                (-0.012 * ((1.0 - u) ** 2.4) + 0.008 * (u ** 1.5)) * (1.0 - w_blend)
            )
            sculpt = (lip + intake_cove + belt_bulge + top_slope) * w_blend + side_tuck
            p_o = Vector((xx, y_loc, zz)) + n_dir * sculpt
            p_i = p_o - n_dir * BUMPER_THICKNESS
            row_o.append(p_o)
            row_i.append(p_i)
        fb_out.append(row_o)
        fb_in.append(row_i)
    _add_thick_grid(bm_bmp, fb_out, fb_in)

    # Верхняя центральная полка бампера перед замком капота (строго за хром-рамкой решётки!)
    sh_fo, sh_fi = [], []
    for i in range(9):
        yy = -0.388 + 0.776 * (i / 8.0)
        x_edge = front_nose_surface_x(yy) - 0.026
        x_back = 2.265
        ro, ri = [], []
        for j in range(3):
            u = j / 2.0
            xx = x_edge * (1.0 - u) + x_back * u
            ro.append(Vector((xx, yy, 0.716)))
            ri.append(Vector((xx, yy, 0.716 - BUMPER_THICKNESS)))
        sh_fo.append(ro)
        sh_fi.append(ri)
    _add_thick_grid(bm_bmp, sh_fo, sh_fi)

    # Нижняя аэродинамическая юбка-поддон под радиатором + 2 монтажных кронштейна к Crash_Beam
    add_box(bm_bmp, FRONT_BUMPER_WING_X + 0.12, 2.38, -0.52, 0.52, 0.212, 0.236)
    for side in (1.0, -1.0):
        ya, yb = (side * 0.44, side * 0.52)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_bmp, 2.32, front_nose_surface_x(0.48) - 0.028, y0, y1, 0.355, 0.405)

    bumper_ob = bmesh_to_object('Front_Bumper', bm_bmp, material=mats['bumper'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.003, bevel_segs=2, subsurf=1)
    uv_box(bumper_ob, scale=1.2)
    _attach(bumper_ob)

    return bumper_ob, objs


def build_rear_structure_and_panels(body_ob, mats, collection):
    """Строит разделённую корму автомобиля по реальной заводской схеме:
      1) Несущий силовой каркас кормы: Rear_Subframe (задняя перегородка, лонжероны,
         ниша багажника, погрузочный порог с ответкой замка, опорные ниши задних фонарей
         на кузове) и Rear_Crash_Beam (алюминиевый усилитель заднего бампера);
      2) Отдельный отсек багажника: Trunk_Tub (ворсовая облицовка ниши и уплотнитель проёма);
      3) Отдельные внешние панели с чёткими швами:
         Fender_Rear_L / Fender_Rear_R (задние крылья с вырезами арок и водосточным фланцем)
         и Rear_Bumper (цельный U-образный задний бампер строго под фонарями и багажником)."""
    objs = []

    def _attach(ob):
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(ob)
        return ob

    # 1. Силовой каркас кормы (Rear_Subframe): перегородка, лонжероны, порог и ниши фонарей
    bm_rsf = bmesh.new()
    # Задняя перегородка за спинками задних сидений
    add_box(bm_rsf, -1.20, REAR_CLIP_X - 0.006, -0.74, 0.74, 0.22, 0.855)
    # Верхняя задняя полка под задним стеклом (между стойками C)
    add_box(bm_rsf, TRUNK_X_FRONT + 0.008, -1.204, -0.66, 0.66, 0.865, 0.935)
    # Продольные силовые лонжероны кормы и задние краш-боксы
    for side in (1.0, -1.0):
        ya, yb = (side * 0.43, side * 0.53)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_rsf, -2.28, -1.204, y0, y1, 0.25, 0.38)
        add_box(bm_rsf, -2.352, -2.284, y0 + 0.008, y1 - 0.008, 0.27, 0.38)
        # Внутренний брызговик заднего крыла и опорная чашка заднего амортизатора
        wa, wb = (side * 0.535, side * 0.635)
        wy0, wy1 = (wa, wb) if wa < wb else (wb, wa)
        add_box(bm_rsf, -2.26, -1.21, wy0, wy1, 0.56, 0.87)
        _add_z_cylinder(bm_rsf, 0.085, 0.54, 0.86, seg=18, center_xy=(AXLE_R, side * 0.58))
        # Опорная задняя панель-ниша крепления заднего фонаря на кузове (точно повторяет
        # дугу кормы rear_tail_surface_x(yy) позади компактного 48 мм корпуса фонаря!)
        p_out, p_in = [], []
        y_t0, y_t1 = TAIL_LIGHT_Y_IN + 0.005, TAIL_LIGHT_Y_OUT - 0.018
        for j in range(3):
            zz = (TAIL_LIGHT_Z_LO + 0.004) + (TAIL_LIGHT_Z_HI - TAIL_LIGHT_Z_LO - 0.008) * (j / 2.0)
            ro, ri = [], []
            for i in range(6):
                yy = y_t0 + (y_t1 - y_t0) * (i / 5.0)
                xb = rear_tail_surface_x(yy) + 0.044
                ro.append(Vector((xb, side * yy, zz)))
                ri.append(Vector((-2.264, side * yy, zz)))
            p_out.append(ro)
            p_in.append(ri)
        _add_thick_grid(bm_rsf, p_out, p_in)
        # Боковая вертикальная стенка-водосток проёма багажника: полностью закрывает внутренний
        # торец фонаря со стороны багажного отсека (чистый заводской проём!)
        ga, gb = (side * (TRUNK_TAIL_HALF_Y + 0.004), side * (TAIL_LIGHT_Y_IN - 0.002))
        gy0, gy1 = (ga, gb) if ga < gb else (gb, ga)
        add_box(bm_rsf, -2.418, -2.240, gy0, gy1, 0.642, TAIL_LIGHT_Z_HI + 0.006)
    # Ниша дна багажника / отсека запасного колеса
    add_box(bm_rsf, -2.26, -1.204, -0.426, 0.426, 0.25, 0.445)
    # Задняя силовая панель погрузочного порога багажника + ответная скоба замка
    add_box(bm_rsf, -2.345, -2.264, -0.52, 0.52, 0.44, 0.636)
    add_box(bm_rsf, -2.33, -2.28, -0.045, 0.045, 0.638, 0.662)

    rsf_ob = bmesh_to_object('Rear_Subframe', bm_rsf, material=mats['carbon'],
                             smooth=True, collection=collection, bevel=0.003)
    uv_box(rsf_ob, scale=2.0)
    _attach(rsf_ob)

    # 2. Изогнутый алюминиевый усилитель заднего бампера (Rear_Crash_Beam — строго внутри
    # оболочки Rear_Bumper с запасом >20 мм, виден при снятии заднего бампера)
    bm_rcb = bmesh.new()
    cb_out, cb_in = [], []
    for j in range(3):
        zz = 0.31 + 0.12 * (j / 2.0)
        ro, ri = [], []
        for i in range(9):
            yy = -0.52 + 1.04 * (i / 8.0)
            xb = rear_tail_surface_x(yy) + 0.048
            ro.append(Vector((xb, yy, zz)))
            ri.append(Vector((xb + 0.046, yy, zz)))
        cb_out.append(ro)
        cb_in.append(ri)
    _add_thick_grid(bm_rcb, cb_out, cb_in)
    rcb_ob = bmesh_to_object('Rear_Crash_Beam', bm_rcb, material=mats['alloy'],
                             smooth=True, collection=collection, bevel=0.004)
    uv_box(rcb_ob, scale=4.0)
    _attach(rcb_ob)

    # 3. Ворсовая ванна багажного отсека (Trunk_Tub — пол, боковины, передняя и задняя стенки)
    bm_tub = bmesh.new()
    # Пол багажника
    add_box(bm_tub, -2.24, -1.22, -0.47, 0.47, 0.455, 0.482)
    # Передняя стенка за задними сиденьями
    add_box(bm_tub, -1.24, -1.208, -0.47, 0.47, 0.486, 0.855)
    # Левая и правая ворсовые стенки (включая кожухи задних арок)
    for side in (1.0, -1.0):
        sa, sb = (side * 0.474, side * 0.515)
        sy0, sy1 = (sa, sb) if sa < sb else (sb, sa)
        add_box(bm_tub, -2.24, -1.244, sy0, sy1, 0.486, 0.865)
    # Задняя облицовка погрузочного порога багажника
    add_box(bm_tub, -2.262, -2.242, -0.47, 0.47, 0.486, 0.642)
    tub_ob = bmesh_to_object('Trunk_Tub', bm_tub, material=mats['carpet'],
                             smooth=True, collection=collection, bevel=0.003)
    uv_box(tub_ob, scale=2.5)
    _attach(tub_ob)

    # 4. ОТДЕЛЬНЫЕ задние крылья (Fender_Rear_L / Fender_Rear_R) с вырезом арки и желобом
    rf_xs = sorted({
        REAR_FENDER_X_REAR, -2.30, -2.22, -2.14, -2.05, -1.96, -1.89,
        *[-1.855 + (-1.085 - (-1.855)) * k / 22.0 for k in range(23)],
        REAR_FENDER_X_FRONT,
    })
    for label, side in (('L', 1.0), ('R', -1.0)):
        bm_rf = bmesh.new()
        outer_pts = []
        inner_pts = []
        for x in rf_xs:
            zb_r = rear_fender_bottom_z(x)
            belt_x = BELT(x)
            y_belt = side_surface_y(x, belt_x)
            # В зоне багажника (x <= TRUNK_X_FRONT) крыло формирует ровный гребень вровень
            # с крышкой багажника (без ступеньки!) и водосточный желоб;
            # в зоне заднего стекла (x > TRUNK_X_FRONT) аккуратно уходит под стойку C / форточку.
            t_trunk = clamp((TRUNK_X_FRONT + 0.04 - x) / 0.10, 0.0, 1.0)
            t_trunk = t_trunk * t_trunk * (3.0 - 2.0 * t_trunk)
            # Повторяем точное сужение крышки багажника Trunk_Lid к проёму между фонарями,
            # чтобы зазор оставался равномерным (~8 мм) до самой задней кромки!
            u_tr_x = clamp((TRUNK_X_FRONT - x) / max(TRUNK_X_FRONT - REAR_FENDER_X_REAR, 0.1),
                           0.0, 1.0)
            t_tr_taper = clamp((u_tr_x - 0.72) / 0.28, 0.0, 1.0)
            t_tr_taper = t_tr_taper * t_tr_taper * (3.0 - 2.0 * t_tr_taper)
            y_edge_tr = (
                (WR(x) + 0.040) * (1.0 - t_tr_taper)
                + (TRUNK_TAIL_HALF_Y + 0.008) * t_tr_taper
            )
            z_edge_tr = trunk_deck_z(x, y_edge_tr) - 0.008
            y_edge_gh = y_belt - 0.048
            z_edge_gh = belt_x + 0.010

            y_top_edge = y_edge_gh * (1.0 - t_trunk) + y_edge_tr * t_trunk
            z_top_edge = z_edge_gh * (1.0 - t_trunk) + z_edge_tr * t_trunk
            y_flange = y_top_edge - (0.025 + 0.020 * t_trunk)
            z_flange = z_top_edge - (0.018 + 0.020 * t_trunk)

            profile = []
            # Мягкий заворот и плавный спад заднего угла крыла (x < -2.14) — убирает резкий
            # 90°-клин над наружным углом C-фонаря и гармонично перетекает в заднюю «бровь»!
            t_rcorn = clamp((-2.14 - x) / (-2.14 - REAR_FENDER_X_REAR), 0.0, 1.0)
            t_rcorn_s = t_rcorn * t_rcorn * (3.0 - 2.0 * t_rcorn)
            y_tuck = 1.0 - 0.032 * (t_rcorn ** 1.85)
            z_rdrop = 0.026 * t_rcorn_s
            belt_loc = belt_x - z_rdrop
            for j in range(7):
                u = j / 6.0
                zz = zb_r + (belt_loc - zb_r) * u
                yy = side_surface_y(x, zz) * y_tuck
                profile.append((yy, zz))
            for k in range(1, 5):
                u = k / 4.0
                yy = (y_belt * y_tuck) * (1.0 - u) + y_top_edge * u
                zz = (
                    belt_loc
                    + (z_top_edge - belt_loc) * (u ** 0.65)
                    + 0.008 * math.sin(math.pi * u)
                )
                profile.append((yy, zz))
            # Загнутый внутрь водосточный желоб/фланец по краю проёма багажника
            profile.append((y_top_edge - 0.008, z_flange + 0.010))
            profile.append((y_flange, z_flange))

            row_out = []
            row_in = []
            # На самом заднем участке крыла (x < -2.16) борт и плечо над C-фонарём плавно
            # вытягиваются по дуге rear_tail_surface_x(yy) -> 0 вертикальной щели за фонарём!
            t_rwrap = clamp((-2.16 - x) / (-2.16 - REAR_FENDER_X_REAR), 0.0, 1.0)
            t_rwrap = t_rwrap * t_rwrap * (3.0 - 2.0 * t_rwrap)
            for idx_p, (yy, zz) in enumerate(profile):
                x_loc = x
                if t_rwrap > 0.0 and idx_p >= 2:
                    x_arc = rear_tail_surface_x(yy) + 0.014
                    if x_arc < x:
                        w_top = clamp((idx_p - 2) / 6.0, 0.0, 1.0)
                        w_top = w_top * w_top * (3.0 - 2.0 * w_top)
                        x_loc = x + (x_arc - x) * t_rwrap * (0.35 + 0.65 * w_top)
                if idx_p < 7:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS, 0.0))
                elif idx_p < 11:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS * 0.55,
                                    -FENDER_THICKNESS * 0.85))
                else:
                    n_off = Vector((0.0, 0.0, -FENDER_THICKNESS * 0.80))
                p_o = Vector((x_loc, side * yy, zz))
                row_out.append(p_o)
                row_in.append(p_o + n_off)
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm_rf, outer_pts, inner_pts)

        # Задний верхний угловой заворот крыла над C-фонарём: мягкий плавный скос сверху вниз
        # точно по профилю гребня Fender_Rear -> единая обтекаемая пластика без «коробки»!
        c_out = []
        c_in = []
        ny_c, nz_c = 9, 5
        belt_r_end = BELT(REAR_FENDER_X_REAR) - 0.026
        y_c0 = TRUNK_TAIL_HALF_Y + 0.008
        y_c1 = side_surface_y(REAR_FENDER_X_REAR, belt_r_end) * 0.968
        z_top_in = trunk_deck_z(REAR_FENDER_X_REAR, y_c0) - 0.008
        for j in range(nz_c):
            vz = j / (nz_c - 1)
            row_co = []
            row_ci = []
            for i in range(ny_c):
                vy = i / (ny_c - 1)
                yy = y_c0 + (y_c1 - y_c0) * vy
                # Нижняя кромка брови мягко повторяет трапециевидный верх C-фонаря
                z_c0_loc = (TAIL_LIGHT_Z_HI + 0.004) - 0.014 * (vy ** 1.4)
                u_sh = 1.0 - vy
                z_crest = (
                    belt_r_end
                    + (z_top_in - belt_r_end) * (u_sh ** 0.65)
                    + 0.008 * math.sin(math.pi * u_sh)
                )
                zz = z_c0_loc + (z_crest - z_c0_loc) * vz
                # Плавный перекат назад (по дуге кормы + скругление к гребню крыла)
                xx = rear_tail_surface_x(yy) + 0.004 + 0.016 * (vz ** 1.45) + 0.008 * (vy ** 2.0)
                row_co.append(Vector((xx, side * yy, zz)))
                row_ci.append(Vector((xx + FENDER_THICKNESS, side * yy, zz)))
            c_out.append(row_co)
            c_in.append(row_ci)
        _add_thick_grid(bm_rf, c_out, c_in)

        rf_ob = bmesh_to_object(f'Fender_Rear_{label}', bm_rf, material=mats['paint'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.003, bevel_segs=2, subsurf=1)
        uv_box(rf_ob, scale=1.2)
        _attach(rf_ob)

    # 5. ОТДЕЛЬНЫЙ СЪЁМНЫЙ задний бампер (Rear_Bumper) контрастного антрацитового цвета:
    # единая скульптурная U-образная оболочка с плавно скруглёнными задними углами,
    # выступающей погрузочной полкой (+24 мм) и чётким 7-мм швом строго ниже фонарей и багажника!
    bm_rb = bmesh.new()

    # Непрерывный U-путь по периметру бампера с большим автомобильным радиусом обоих углов:
    # левое крылышко -> левый радиус угла -> задняя дуга -> правый радиус угла -> правое крылышко
    u_stations = []
    y_corner = side_surface_y(REAR_FENDER_X_REAR, 0.55) * 0.962
    r_corner = 0.155  # плавный радиус скругления угла заднего бампера в плане XY
    x_wing_end = REAR_FENDER_X_REAR + r_corner

    # 5a. Левое боковое крылышко (+Y) от края арки к началу скругления угла
    n_wing = 6
    for i in range(n_wing):
        t = i / float(n_wing)
        x = REAR_BUMPER_WING_X + (x_wing_end - REAR_BUMPER_WING_X) * t
        z_lo = ZB(x) + 0.034
        z_hi = rear_fender_bumper_seam_z(x) - 0.007
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_stations.append((x, +y_abs, z_lo, z_hi, Vector((0.0, 1.0, 0.0)), 0.0))

    # 5b. Левый скруглённый угол бампера (плавный поворот от +Y к -X)
    x_back_edge = rear_tail_surface_x(y_corner - r_corner) - 0.022
    z_hi_seam_end = rear_fender_bumper_seam_z(x_wing_end) - 0.007
    for k in range(6):
        ang = (k / 5.0) * (0.5 * math.pi)
        xx = x_wing_end - (x_wing_end - x_back_edge) * math.sin(ang)
        yy = (y_corner - r_corner) + r_corner * math.cos(ang)
        t_c = (k / 5.0) ** 0.65
        z_lo = ZB(xx) + 0.034
        z_hi = z_hi_seam_end * (1.0 - t_c) + REAR_BUMPER_TOP_Z * t_c
        n_dir = Vector((-math.sin(ang), math.cos(ang), 0.0)).normalized()
        u_stations.append((xx, +yy, z_lo, z_hi, n_dir, k / 5.0))

    # 5c. Задняя скруглённая панель между левым и правым углами (+22 мм погрузочный выступ!)
    y_span = y_corner - r_corner
    n_back = 17
    for i in range(1, n_back - 1):
        v = 1.0 - 2.0 * i / float(n_back - 1)
        yy = v * y_span
        xx = rear_tail_surface_x(yy) - 0.022
        z_lo = ZB(xx) + 0.034
        z_hi = REAR_BUMPER_TOP_Z
        n_dir = Vector((-1.0, 0.34 * v, 0.0)).normalized()
        u_stations.append((xx, yy, z_lo, z_hi, n_dir, 1.0))

    # 5d. Правый скруглённый угол бампера (плавный поворот от -X к -Y)
    for k in range(6):
        ang = (1.0 - k / 5.0) * (0.5 * math.pi)
        xx = x_wing_end - (x_wing_end - x_back_edge) * math.sin(ang)
        yy = (y_corner - r_corner) + r_corner * math.cos(ang)
        t_c = (1.0 - k / 5.0) ** 0.65
        z_lo = ZB(xx) + 0.034
        z_hi = z_hi_seam_end * (1.0 - t_c) + REAR_BUMPER_TOP_Z * t_c
        n_dir = Vector((-math.sin(ang), -math.cos(ang), 0.0)).normalized()
        u_stations.append((xx, -yy, z_lo, z_hi, n_dir, 1.0 - k / 5.0))

    # 5e. Правое боковое крылышко (-Y) к краю задней арки
    for i in range(1, n_wing + 1):
        t = i / float(n_wing)
        x = x_wing_end + (REAR_BUMPER_WING_X - x_wing_end) * t
        z_lo = ZB(x) + 0.034
        z_hi = rear_fender_bumper_seam_z(x) - 0.007
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_stations.append((x, -y_abs, z_lo, z_hi, Vector((0.0, -1.0, 0.0)), 0.0))

    rb_out = []
    rb_in = []
    nz_rb = 8
    for xx, yy, z_lo, z_hi, n_dir, rear_w in u_stations:
        row_o = []
        row_i = []
        side_s = 1.0 if yy >= 0.0 else -1.0
        for j in range(nz_rb):
            u = j / float(nz_rb - 1)
            zz = z_lo + (z_hi - z_lo) * u
            # На боковых крылышках (rear_w < 1) вычисляем точную side_surface_y(xx, zz)
            # на каждой высоте zz -> стык заднее крыло <-> задний бампер заподлицо (< 1 мм!)
            if rear_w < 0.99:
                y_true = side_s * side_surface_y(xx, zz)
                t_rc = clamp((-2.14 - xx) / (-2.14 - REAR_FENDER_X_REAR), 0.0, 1.0)
                y_true *= (1.0 - 0.032 * (t_rc ** 1.85))
                w_blend = rear_w * rear_w * (3.0 - 2.0 * rear_w)
                y_loc = y_true * (1.0 - w_blend) + yy * w_blend
            else:
                y_loc = yy
            # Автомобильная скульптура заднего бампера: мягкий подгиб снизу + упругий верхний пояс
            # (на верхнем шве крылышка u=1 подгиб строго 0 мм!)
            tuck = (0.032 * rear_w + 0.012 * (1.0 - rear_w)) * ((1.0 - u) ** 2.2)
            bulge = (
                0.012 * math.sin(math.pi * clamp((u - 0.35) / 0.65, 0.0, 1.0)) * rear_w
                if u > 0.35 else 0.0
            )
            p_o = Vector((xx, y_loc, zz)) + n_dir * (bulge - tuck)
            p_i = p_o - n_dir * BUMPER_THICKNESS
            row_o.append(p_o)
            row_i.append(p_i)
        rb_out.append(row_o)
        rb_in.append(row_i)
    _add_thick_grid(bm_rb, rb_out, rb_in)

    # Горизонтальная погрузочная полка поверху заднего бампера (строго внутри задней панели
    # до скругления углов y_span -> без выступающего «зубчика» под углом заднего фонаря!)
    sh_out = []
    sh_in = []
    y_shelf = y_span - 0.018
    for i in range(13):
        v = -1.0 + 2.0 * i / 12.0
        yy = v * y_shelf
        x_back = rear_tail_surface_x(yy) - 0.012
        x_front = -2.26
        row_so = []
        row_si = []
        for j in range(4):
            u = j / 3.0
            xx = x_back * (1.0 - u) + x_front * u
            row_so.append(Vector((xx, yy, REAR_BUMPER_TOP_Z - 0.002)))
            row_si.append(Vector((xx, yy, REAR_BUMPER_TOP_Z - 0.002 - BUMPER_THICKNESS)))
        sh_out.append(row_so)
        sh_in.append(row_si)
    _add_thick_grid(bm_rb, sh_out, sh_in)

    # Два внутренних монтажных кронштейна съёмного заднего бампера к Rear_Crash_Beam
    for side in (1.0, -1.0):
        ya, yb = (side * 0.44, side * 0.51)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_rb, rear_tail_surface_x(0.47) + 0.008, -2.33, y0, y1, 0.34, 0.395)

    rear_bumper = bmesh_to_object('Rear_Bumper', bm_rb, material=mats['bumper'],
                                  smooth=True, collection=collection, auto_smooth_deg=55.0,
                                  bevel=0.003, bevel_segs=2, subsurf=1)
    uv_box(rear_bumper, scale=1.2)
    _attach(rear_bumper)

    return rear_bumper, objs


def _add_station_beam(bm, stations, side=1.0):
    """Строит замкнутую балку (стойку/рамку) по списку поперечных сечений.
    Каждое сечение — кортеж (x_back, x_front, y_inner_abs, y_outer_abs, z).
    Параметр side (+1 левый борт, -1 правый) задаёт симметричное зеркалирование по Y."""
    rings = []
    for xb, xf, yin, yout, z in stations:
        xa, xc = (xb, xf) if xb <= xf else (xf, xb)
        ya, yb = (yin, yout) if yin <= yout else (yout, yin)
        if side >= 0.0:
            ring = [
                bm.verts.new(Vector((xa, ya, z))),
                bm.verts.new(Vector((xc, ya, z))),
                bm.verts.new(Vector((xc, yb, z))),
                bm.verts.new(Vector((xa, yb, z))),
            ]
        else:
            ring = [
                bm.verts.new(Vector((xa, -yb, z))),
                bm.verts.new(Vector((xc, -yb, z))),
                bm.verts.new(Vector((xc, -ya, z))),
                bm.verts.new(Vector((xa, -ya, z))),
            ]
        rings.append(ring)

    for i in range(len(rings) - 1):
        r0, r1 = rings[i], rings[i + 1]
        for k in range(4):
            kn = (k + 1) % 4
            bm.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    bm.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
    bm.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))


def _add_thick_grid(bm, outer_pts, inner_pts):
    """Добавляет замкнутую двустороннюю оболочку из двух матриц точек (ni x nj)."""
    ni = len(outer_pts)
    nj = len(outer_pts[0])
    outer = [[bm.verts.new(p) for p in row] for row in outer_pts]
    inner = [[bm.verts.new(p) for p in row] for row in inner_pts]

    for i in range(ni - 1):
        for j in range(nj - 1):
            bm.faces.new((outer[i][j], outer[i][j + 1], outer[i + 1][j + 1], outer[i + 1][j]))
            bm.faces.new((inner[i][j], inner[i + 1][j], inner[i + 1][j + 1], inner[i][j + 1]))

    for i in range(ni - 1):
        bm.faces.new((outer[i][0], outer[i + 1][0], inner[i + 1][0], inner[i][0]))
        bm.faces.new((outer[i][nj - 1], inner[i][nj - 1],
                      inner[i + 1][nj - 1], outer[i + 1][nj - 1]))
    for j in range(nj - 1):
        bm.faces.new((outer[0][j], inner[0][j], inner[0][j + 1], outer[0][j + 1]))
        bm.faces.new((outer[ni - 1][j], outer[ni - 1][j + 1],
                      inner[ni - 1][j + 1], inner[ni - 1][j]))


def build_roof_and_pillars(body_ob, mats, collection):
    """Строит отдельную панель крыши Roof и 6 несущих стоек кузова (A, B, C слева/справа),
    соединяющих нижнюю ванну кузова (Body) с крышей вокруг сквозных дверных вырезов."""
    objs = []

    # 1. Панель крыши (Roof): замкнутая плавно выгнутая оболочка от ROOF_X_REAR до ROOF_X_FRONT
    # с мягким загибом переднего и заднего кантов точно к верху лобового и заднего стёкол
    bm_roof = bmesh.new()
    nx, ny = 20, 11
    x_r0 = ROOF_X_REAR - 0.020
    x_r1 = ROOF_X_FRONT + 0.020
    xs = [x_r0 + (x_r1 - x_r0) * i / (nx - 1) for i in range(nx)]
    outer_pts = []
    inner_pts = []
    for idx_x, x in enumerate(xs):
        u_x = idx_x / float(nx - 1)
        hw = WR(x) + 0.010
        zr = ZR(x)
        # Плавный продольный спад переднего и заднего кантов крыши вровень со стёклами/стойками
        end_f = clamp((u_x - 0.82) / 0.18, 0.0, 1.0)
        end_r = clamp((0.18 - u_x) / 0.18, 0.0, 1.0)
        end_drop = 0.013 * (end_f * end_f + end_r * end_r)
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            z_top = zr - 0.013 * (v * v) - end_drop
            row_out.append(Vector((x, y, z_top)))
            row_in.append(Vector((x, y, z_top - ROOF_THICKNESS)))
        outer_pts.append(row_out)
        inner_pts.append(row_in)
    _add_thick_grid(bm_roof, outer_pts, inner_pts)
    roof_ob = bmesh_to_object('Roof', bm_roof, material=mats['paint'],
                              smooth=True, collection=collection, auto_smooth_deg=55.0,
                              bevel=0.003, bevel_segs=2, subsurf=1)
    uv_top(roof_ob)
    roof_ob.parent = body_ob
    roof_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(roof_ob)

    # 2. Несущие стойки кузова A, B, C (левый и правый борт, фастбэк-силуэт Volvo S90)
    for label, side in (('L', 1.0), ('R', -1.0)):
        # 2a. Стойка A: ровная спортивная стойка вдоль лобового стекла в крышу,
        # строго согласованная с передней наклонной рамкой двери Door_Front по upper_side_yz
        bm_a = bmesh.new()
        xa_bot = CABIN_CUT_X_FRONT + 0.045
        y_a_sill = side_surface_y(xa_bot, CABIN_SILL_Z + 0.015)
        y_a_mid = side_surface_y(xa_bot, 0.58)
        z_a_belt = BELT(WINDSHIELD_X_BOT)
        y_a_belt = side_surface_y(WINDSHIELD_X_BOT, z_a_belt)
        st_a = [
            (CABIN_CUT_X_FRONT - 0.008, xa_bot + 0.036, y_a_sill - 0.115, y_a_sill - 0.068,
             CABIN_SILL_Z + 0.015),
            (CABIN_CUT_X_FRONT - 0.008, xa_bot + 0.036, y_a_mid - 0.105, y_a_mid - 0.056,
             0.58),
            (WINDSHIELD_X_BOT - 0.072, WINDSHIELD_X_BOT + 0.014,
             y_a_belt - 0.092, y_a_belt - 0.022, z_a_belt - 0.026),
        ]
        for step in range(8):
            t = step / 7.0
            x_front_a = (WINDSHIELD_X_BOT + 0.014) * (1.0 - t) + (ROOF_X_FRONT + 0.022) * t
            x_back_a = (WINDSHIELD_X_BOT - 0.072) * (1.0 - t) + (ROOF_X_FRONT - 0.032) * t
            x_mid = 0.5 * (x_front_a + x_back_a)
            y_out, z_val = upper_side_yz(x_mid, t)
            if step == 7:
                y_out = WR(ROOF_X_FRONT) + 0.006
                z_val = ZR(ROOF_X_FRONT) - 0.012
            else:
                y_out += 0.004
            thick_y = 0.058 * (1.0 - 0.16 * t)
            st_a.append((x_back_a, x_front_a, y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_a, st_a, side=side)
        pa = bmesh_to_object(f'Pillar_A_{label}', bm_a, material=mats['paint'],
                             smooth=True, collection=collection, auto_smooth_deg=55.0,
                             bevel=0.0025, bevel_segs=2, subsurf=1)
        uv_box(pa, scale=2.0)
        pa.parent = body_ob
        pa.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pa)

        # 2b. Центральная стойка B: изящная глянцево-чёрная стойка в зоне окон;
        # ниже подоконной линии утоплена по локальному side_surface_y (не выступает из порога)
        bm_b = bmesh.new()
        yb_sill = side_surface_y(PILLAR_B_X, CABIN_SILL_Z + 0.015)
        yb_jamb = side_surface_y(PILLAR_B_X, 0.62)
        yb_belt = side_surface_y(PILLAR_B_X, BELT(PILLAR_B_X) - 0.025)
        st_b = [
            (PILLAR_B_X - PILLAR_B_HALF_X * 1.15, PILLAR_B_X + PILLAR_B_HALF_X * 1.15,
             yb_sill - 0.115, yb_sill - 0.068, CABIN_SILL_Z + 0.015),
            (PILLAR_B_X - PILLAR_B_HALF_X * 1.25, PILLAR_B_X + PILLAR_B_HALF_X * 1.25,
             yb_jamb - 0.110, yb_jamb - 0.052, 0.62),
            (PILLAR_B_X - PILLAR_B_HALF_X * 1.25, PILLAR_B_X + PILLAR_B_HALF_X * 1.25,
             yb_belt - 0.105, yb_belt - 0.048, BELT(PILLAR_B_X) - 0.025),
        ]
        for step in range(7):
            t = step / 6.0
            half_x = PILLAR_B_HALF_X * (1.0 - 0.12 * t)
            y_out, z_val = upper_side_yz(PILLAR_B_X, t)
            if step == 0:
                y_out -= 0.002
            elif step == 6:
                y_out = WR(PILLAR_B_X) + 0.006
                z_val = ZR(PILLAR_B_X) - 0.012
            else:
                y_out += 0.003
            thick_y = 0.052 * (1.0 - 0.18 * t)
            st_b.append((PILLAR_B_X - half_x, PILLAR_B_X + half_x,
                         y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_b, st_b, side=side)
        pb = bmesh_to_object(f'Pillar_B_{label}', bm_b, material=mats['piano_black'],
                             smooth=True, collection=collection, auto_smooth_deg=55.0,
                             bevel=0.0025, bevel_segs=2, subsurf=1)
        uv_box(pb, scale=2.0)
        pb.parent = body_ob
        pb.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pb)

        # 2c. Задняя стойка C: изящная фастбэк-стойка Volvo S90 за боковым окошком
        bm_c = bmesh.new()
        xc_jamb = CABIN_CUT_X_REAR - 0.03
        yc_jamb = side_surface_y(xc_jamb, 0.62)
        st_c = [
            (xc_jamb - 0.08, CABIN_CUT_X_REAR + 0.004,
             yc_jamb - 0.11, yc_jamb - 0.052, CABIN_SILL_Z + 0.015),
            (xc_jamb - 0.12, CABIN_CUT_X_REAR + 0.004,
             yc_jamb - 0.10, yc_jamb - 0.046, 0.62),
        ]
        for step in range(8):
            t = step / 7.0
            x_back = (REAR_GLASS_X_BOT - 0.01) * (1.0 - t) + (ROOF_X_REAR - 0.015) * t
            x_front = (REAR_GLASS_X_BOT + 0.34) * (1.0 - t) + (ROOF_X_REAR + 0.12) * t
            x_mid = 0.5 * (x_back + x_front)
            y_out, z_val = upper_side_yz(x_mid, t)
            if step == 0:
                y_out -= 0.014
            elif step == 7:
                y_out = WR(ROOF_X_REAR) + 0.006
                z_val = ZR(ROOF_X_REAR) - 0.012
            else:
                y_out += 0.004
            thick_y = 0.064 * (1.0 - 0.18 * t)
            st_c.append((x_back, x_front, y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_c, st_c, side=side)
        pc = bmesh_to_object(f'Pillar_C_{label}', bm_c, material=mats['paint'],
                             smooth=True, collection=collection, auto_smooth_deg=55.0,
                             bevel=0.0025, bevel_segs=2, subsurf=1)
        uv_box(pc, scale=2.0)
        pc.parent = body_ob
        pc.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pc)

        # 5. Фирменный хромированный DLO-молдинг Volvo S90 Inscription:
        # верхняя дуга вдоль Pillar_A -> Roof -> Pillar_C + окантовка задней треугольной форточки
        # (поясные планки на дверях крепятся к самим дверям и открываются вместе с ними!)
        bm_wt = bmesh.new()

        def _add_chrome_strip(pts_3d, hw=0.0065, hd=0.0045):
            rings = []
            n_p = len(pts_3d)
            for idx, p in enumerate(pts_3d):
                if idx == 0:
                    tang = (pts_3d[1] - pts_3d[0]).normalized()
                elif idx == n_p - 1:
                    tang = (pts_3d[-1] - pts_3d[-2]).normalized()
                else:
                    tang = (pts_3d[idx + 1] - pts_3d[idx - 1]).normalized()
                y_dir = Vector((0.0, side, 0.0))
                in_p = tang.cross(y_dir)
                if in_p.length < 1e-6:
                    in_p = Vector((0.0, 0.0, 1.0))
                in_p.normalize()
                rings.append([
                    bm_wt.verts.new(p - in_p * hw - y_dir * hd),
                    bm_wt.verts.new(p + in_p * hw - y_dir * hd),
                    bm_wt.verts.new(p + in_p * hw + y_dir * hd),
                    bm_wt.verts.new(p - in_p * hw + y_dir * hd),
                ])
            for i_s in range(n_p - 1):
                r0, r1 = rings[i_s], rings[i_s + 1]
                for k in range(4):
                    kn = (k + 1) % 4
                    bm_wt.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
            bm_wt.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
            bm_wt.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))

        def _dlo_pt(x_v, t_v, y_off=0.006, z_off=0.0):
            ya, zv = upper_side_yz(x_v, t_v)
            return Vector((x_v, side * (ya + y_off), zv + z_off))

        # Непрерывная хромированная дуга со скруглёнными верхними углами:
        # вдоль Pillar_A -> скругление переднего угла -> верх крыши -> скругление заднего угла ->
        # спуск по Pillar_C -> поворот и подоконник задней форточки до заднего стыка Door_Rear
        dlo_pts = []
        xa_b, xa_t = WINDSHIELD_X_BOT - 0.082, ROOF_X_FRONT - 0.030
        for k in range(5):
            t = 0.02 + 0.86 * (k / 4.0)
            dlo_pts.append(_dlo_pt(xa_b * (1.0 - t) + xa_t * t, t, y_off=0.005))
        # Плавный радиус перехода Pillar_A -> Roof
        dlo_pts.append(_dlo_pt(xa_t + 0.015, 0.945, y_off=0.005))
        dlo_pts.append(_dlo_pt(xa_t - 0.018, 0.978, y_off=0.005))
        for k in range(1, 6):
            u = k / 6.0
            xx = (xa_t - 0.04) * (1.0 - u) + (ROOF_X_REAR + 0.06) * u
            dlo_pts.append(_dlo_pt(xx, 0.985, y_off=0.005))
        # Плавный радиус перехода Roof -> Pillar_C (без резкого углового излома!)
        xc_t, xc_b = ROOF_X_REAR + 0.02, REAR_GLASS_X_BOT + 0.34
        dlo_pts.append(_dlo_pt(xc_t + 0.025, 0.976, y_off=0.005))
        dlo_pts.append(_dlo_pt(xc_t - 0.015, 0.940, y_off=0.005))
        for k in range(1, 6):
            t = 0.88 - 0.85 * (k / 5.0)
            dlo_pts.append(_dlo_pt(xc_t * t + xc_b * (1.0 - t), t, y_off=0.005))
        # Нижний поясной отрезок задней форточки от угла C-стойки до стыка задней двери
        xq_f = CABIN_CUT_X_REAR - 0.006
        for k in range(1, 5):
            u = k / 4.0
            xx = xc_b * (1.0 - u) + xq_f * u
            dlo_pts.append(_dlo_pt(xx, 0.03, y_off=0.006))
        _add_chrome_strip(dlo_pts, hw=0.0065, hd=0.0045)

        wt_ob = bmesh_to_object(f'Window_Trim_{label}', bm_wt, material=mats['chrome'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.0015, bevel_segs=1)
        uv_box(wt_ob, scale=8.0)
        wt_ob.parent = body_ob
        wt_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(wt_ob)

    # 6. Панорамный стеклянный люк на крыше (Roof_Sunroof: чёрная рамка + тонированное стекло)
    bm_sr = bmesh.new()
    sr_x0, sr_x1 = ROOF_X_REAR + 0.34, ROOF_X_FRONT - 0.10
    nx_sr, ny_sr = 10, 9
    sr_out, sr_in = [], []
    for i in range(nx_sr):
        u_s = i / float(nx_sr - 1)
        xs = sr_x0 + (sr_x1 - sr_x0) * u_s
        hw_s = WR(xs) - 0.115
        zr_s = ZR(xs)
        ro, ri = [], []
        for j in range(ny_sr):
            v_s = -1.0 + 2.0 * j / float(ny_sr - 1)
            ys = v_s * hw_s
            zs = zr_s - 0.013 * (v_s * v_s) + 0.004
            ro.append(Vector((xs, ys, zs)))
            ri.append(Vector((xs, ys, zs - 0.010)))
        sr_out.append(ro)
        sr_in.append(ri)
    _add_thick_grid(bm_sr, sr_out, sr_in)
    bm_sr.faces.ensure_lookup_table()
    # Наружный ободок люка -> Piano_Black (slot 0), центральная стеклянная панель -> Glass (slot 1)
    for f_sr in bm_sr.faces:
        fc = f_sr.calc_center_median()
        if (sr_x0 + 0.055 < fc.x < sr_x1 - 0.055) and (abs(fc.y) < WR(fc.x) - 0.165):
            f_sr.material_index = 1
        else:
            f_sr.material_index = 0
    sr_ob = bmesh_to_object('Roof_Sunroof', bm_sr, material=mats['piano_black'],
                            smooth=True, collection=collection, auto_smooth_deg=55.0,
                            bevel=0.0020, bevel_segs=2, subsurf=1)
    sr_ob.data.materials.append(mats['glass'])
    uv_top(sr_ob)
    sr_ob.parent = roof_ob
    sr_ob.matrix_parent_inverse = roof_ob.matrix_world.inverted()
    objs.append(sr_ob)

    # 7. Аэродинамическая антенна «акулий плавник» (Roof_Antenna): прокладка + 6-секц. гребень
    bm_ant = bmesh.new()
    x_ant = ROOF_X_REAR + 0.16
    z_ant = ZR(x_ant) - 0.004
    ant_specs = [
        (x_ant + 0.095, 0.015, 0.006, z_ant + 0.004),
        (x_ant + 0.055, 0.021, 0.009, z_ant + 0.018),
        (x_ant + 0.015, 0.023, 0.010, z_ant + 0.036),
        (x_ant - 0.025, 0.019, 0.008, z_ant + 0.054),
        (x_ant - 0.058, 0.014, 0.005, z_ant + 0.064),
        (x_ant - 0.082, 0.008, 0.003, z_ant + 0.060),
    ]
    a_rings = []
    for xa, w_b, w_t, z_hi in ant_specs:
        z_mid = z_ant + 0.45 * (z_hi - z_ant)
        w_m = 0.55 * w_b + 0.45 * w_t
        a_rings.append([
            bm_ant.verts.new(Vector((xa, -w_b, z_ant))),
            bm_ant.verts.new(Vector((xa, +w_b, z_ant))),
            bm_ant.verts.new(Vector((xa, +w_m, z_mid))),
            bm_ant.verts.new(Vector((xa, +w_t, z_hi))),
            bm_ant.verts.new(Vector((xa, -w_t, z_hi))),
            bm_ant.verts.new(Vector((xa, -w_m, z_mid))),
        ])
    for i_a in range(len(a_rings) - 1):
        r0, r1 = a_rings[i_a], a_rings[i_a + 1]
        for k in range(6):
            kn = (k + 1) % 6
            bm_ant.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    for end_a, rev in ((a_rings[0], True), (a_rings[-1], False)):
        if rev:
            bm_ant.faces.new((end_a[0], end_a[5], end_a[2], end_a[1]))
            bm_ant.faces.new((end_a[5], end_a[4], end_a[3], end_a[2]))
        else:
            bm_ant.faces.new((end_a[0], end_a[1], end_a[2], end_a[5]))
            bm_ant.faces.new((end_a[5], end_a[2], end_a[3], end_a[4]))
    # Резиновая прокладка-основание антенны (slot 1 = Piano_Black)
    n_ant_paint = len(bm_ant.faces)
    add_box(bm_ant, x_ant - 0.085, x_ant + 0.098, -0.025, +0.025, z_ant - 0.005, z_ant + 0.002)
    bm_ant.faces.ensure_lookup_table()
    for fi in range(n_ant_paint, len(bm_ant.faces)):
        bm_ant.faces[fi].material_index = 1
    ant_ob = bmesh_to_object('Roof_Antenna', bm_ant, material=mats['paint'],
                             smooth=True, collection=collection, auto_smooth_deg=55.0,
                             bevel=0.0018, bevel_segs=2, subsurf=1)
    ant_ob.data.materials.append(mats['piano_black'])
    uv_box(ant_ob, scale=8.0)
    ant_ob.parent = roof_ob
    ant_ob.matrix_parent_inverse = roof_ob.matrix_world.inverted()
    objs.append(ant_ob)

    # 8. Детализированные бескаркасные стеклоочистители (Wiper_L, Wiper_R):
    #    ось-колпачок на жабо + металлический поводок + адаптер + аэродинамическая щётка + резинка
    z_wb = BELT(WINDSHIELD_X_BOT) + 0.015
    for w_lbl, y_piv, y_tip in (('L', 0.06, 0.64), ('R', -0.48, 0.14)):
        bm_wp = bmesh.new()
        x_piv = WINDSHIELD_X_BOT + 0.042
        x_tip = WINDSHIELD_X_BOT - 0.014
        # 8a. Круглый защитный колпачок оси поводка на жабо
        _add_z_cylinder(bm_wp, 0.015, z_wb - 0.012, z_wb + 0.012, seg=14,
                        center_xy=(x_piv + 0.012, y_piv))
        # 8b. Металлический поводок от оси до центрального адаптера (u = 0.0 .. 0.48)
        arm_pts = []
        for k in range(5):
            u_w = 0.48 * (k / 4.0)
            yy = y_piv * (1.0 - u_w) + y_tip * u_w
            xx = x_piv * (1.0 - u_w) + x_tip * u_w + 0.016 * (1.0 - (yy / 0.72) ** 2)
            zz = z_wb + 0.022 * u_w - 0.006 * (yy * yy) + 0.006
            arm_pts.append(Vector((xx, yy, zz)))
        _sweep_rect_tube(bm_wp, arm_pts, half_y=0.0055, half_n=0.0042)
        # 8c. Центральный адаптер крепления щётки
        p_mid = arm_pts[-1]
        add_box(bm_wp, p_mid.x - 0.012, p_mid.x + 0.012,
                p_mid.y - 0.026, p_mid.y + 0.026, p_mid.z - 0.006, p_mid.z + 0.009)
        # 8d. Аэродинамическая бескаркасная щётка-спойлер (вдоль всей рабочей длины u=0.14..1.0)
        blade_pts = []
        for k in range(8):
            u_w = 0.14 + 0.86 * (k / 7.0)
            yy = y_piv * (1.0 - u_w) + y_tip * u_w
            xx = x_piv * (1.0 - u_w) + x_tip * u_w + 0.015 * (1.0 - (yy / 0.72) ** 2) - 0.006
            zz = z_wb + 0.021 * u_w - 0.006 * (yy * yy) + 0.002
            blade_pts.append(Vector((xx, yy, zz)))
        _sweep_rect_tube(bm_wp, blade_pts, half_y=0.0075, half_n=0.0045)
        # 8e. Нижняя резиновая лента щётки (slot 1 = Rubber)
        n_bef_rub = len(bm_wp.faces)
        rub_pts = [p - Vector((0.0, 0.0, 0.0055)) for p in blade_pts]
        _sweep_rect_tube(bm_wp, rub_pts, half_y=0.0030, half_n=0.0030)
        bm_wp.faces.ensure_lookup_table()
        for fi in range(n_bef_rub, len(bm_wp.faces)):
            bm_wp.faces[fi].material_index = 1
        wp_ob = bmesh_to_object(f'Wiper_{w_lbl}', bm_wp, material=mats['piano_black'],
                                smooth=True, collection=collection, bevel=0.0010, bevel_segs=1)
        wp_ob.data.materials.append(mats['rubber'])
        uv_box(wp_ob, scale=10.0)
        wp_ob.parent = body_ob
        wp_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(wp_ob)

    return roof_ob, objs


def build_glass(mats, collection):
    """Остекление кузова: лобовое стекло (между стойками A), заднее стекло (между стойками C)
    и боковые треугольные форточки задних стоек (фирменный контур DLO Volvo S90).
    Боковые проёмы дверей оставлены открытыми — стёкла дверей крепятся к самим дверям."""
    bm = bmesh.new()
    thickness = 0.005

    def _add_curved_pane(x_bot, z_bot, y_bot, x_top, z_top, y_top, bulge_x, nu=9, nv=13):
        outer_pts = []
        inner_pts = []
        dx = x_top - x_bot
        dz = z_top - z_bot
        n_xz = Vector((-dz, 0.0, dx))
        if n_xz.x * (1.0 if x_bot > 0 else -1.0) < 0.0:
            n_xz = -n_xz
        n_xz.normalize()
        for i in range(nu):
            u = i / (nu - 1)
            xc = x_bot * (1.0 - u) + x_top * u
            zc = z_bot * (1.0 - u) + z_top * u + 0.012 * math.sin(math.pi * u)
            hw = y_bot * (1.0 - u) + y_top * u + 0.024 * math.sin(math.pi * u)
            row_out = []
            row_in = []
            for j in range(nv):
                v = -1.0 + 2.0 * j / (nv - 1)
                y = v * hw
                bow = bulge_x * (1.0 - v * v)
                p_out = Vector((xc + bow, y, zc - 0.008 * (v * v)))
                row_out.append(p_out)
                row_in.append(p_out - n_xz * thickness)
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm, outer_pts, inner_pts)

    # Лобовое стекло (между стойками A, боковые края заходят в паз Pillar_A без зазора)
    _add_curved_pane(
        x_bot=WINDSHIELD_X_BOT,
        z_bot=BELT(WINDSHIELD_X_BOT) + 0.004,
        y_bot=side_surface_y(WINDSHIELD_X_BOT, BELT(WINDSHIELD_X_BOT)) * 0.98,
        x_top=ROOF_X_FRONT + 0.014,
        z_top=ZR(ROOF_X_FRONT) - 0.010,
        y_top=WR(ROOF_X_FRONT) * 0.98,
        bulge_x=+0.022,
    )

    # Заднее стекло (между стойками C, боковые края заходят в паз Pillar_C без зазора)
    _add_curved_pane(
        x_bot=REAR_GLASS_X_BOT,
        z_bot=BELT(REAR_GLASS_X_BOT) + 0.024,
        y_bot=side_surface_y(REAR_GLASS_X_BOT, BELT(REAR_GLASS_X_BOT)) * 0.95,
        x_top=ROOF_X_REAR - 0.008,
        z_top=ZR(ROOF_X_REAR) - 0.010,
        y_top=WR(ROOF_X_REAR) * 0.97,
        bulge_x=-0.020,
    )

    # Задние боковые форточки (C-pillar quarter glass — удлинённый купеобразный силуэт окон S90)
    for side in (1.0, -1.0):
        q_out = []
        q_in = []
        nu_q, nv_q = 6, 6
        for j in range(nv_q):
            t = 0.05 + 0.88 * (j / (nv_q - 1))
            xf_t = (CABIN_CUT_X_REAR - 0.015) * (1.0 - t) + (ROOF_X_REAR + 0.06) * t
            xb_t = (REAR_GLASS_X_BOT + 0.34) * (1.0 - t) + (ROOF_X_REAR - 0.02) * t
            row_o = []
            row_i = []
            for i in range(nu_q):
                u = i / (nu_q - 1)
                x_g = xf_t * (1.0 - u) + xb_t * u
                y_abs, z_g = upper_side_yz(x_g, t)
                # В одной плоскости с дверными стёклами и пазом Pillar_C (не торчит наружу!)
                y_o = side * (y_abs - 0.004)
                y_i = side * (y_abs - 0.009)
                row_o.append(Vector((x_g, y_o, z_g)))
                row_i.append(Vector((x_g, y_i, z_g)))
            q_out.append(row_o)
            q_in.append(row_i)
        _add_thick_grid(bm, q_out, q_in)

    ob = bmesh_to_object('Glass', bm, material=mats['glass'], smooth=True,
                         collection=collection, auto_smooth_deg=60.0, subsurf=1)
    uv_top(ob)
    return ob


# --------------------------------------------------------------------------- #
# 5. Колёса: тороидальная шина, 10-спицевый диск, тормозной диск и суппорт
# --------------------------------------------------------------------------- #
def _add_y_annulus(bm, r_inner, r_outer, y0, y1, seg=36, profile=None):
    """Замкнутое тороидальное/кольцевое тело вокруг оси Y.
    Если profile=None — прямоугольное сечение (r_inner..r_outer, y0..y1).
    Если передан profile — список пар (r, y) замкнутого контура сечения."""
    if profile is None:
        profile = [
            (r_inner, y0), (r_outer, y0), (r_outer, y1), (r_inner, y1),
        ]
    n_p = len(profile)
    rings = []
    for i in range(seg):
        a = 2.0 * math.pi * i / seg
        ca, sa = math.cos(a), math.sin(a)
        rings.append([bm.verts.new(Vector((r * ca, yy, r * sa))) for (r, yy) in profile])
    for i in range(seg):
        inext = (i + 1) % seg
        r0, r1 = rings[i], rings[inext]
        for k in range(n_p):
            kn = (k + 1) % n_p
            bm.faces.new((r0[k], r0[kn], r1[kn], r1[k]))


def build_wheel(tag, axle_x, side, mats, collection, body_ob=None):
    """Строит профилированную шину (Wheel_*), 10-спицевый легкосплавный диск (Rim_*),
    тормозной диск (Brake_Disc_*) и неподвижный красный суппорт (Brake_Caliper_*)."""
    cy = side * WHEEL_Y
    cz = WHEEL_Z
    seg = 56
    hw = WHEEL_WIDTH / 2.0
    r_bead = 0.242

    # 1. Тороидальная шина со скруглёнными плечами, защитным бортиком обода и 4 канавками протектора
    bm_tire = bmesh.new()
    r_out = WHEEL_R
    r_grv = WHEEL_R - 0.0065
    r_sh = WHEEL_R - 0.015
    r_sh2 = WHEEL_R - 0.036
    r_mid = 0.5 * (r_bead + r_out)
    tire_profile = [
        (r_bead, -hw * 0.88),
        (r_bead + 0.010, -hw * 0.95),  # защитный бортик обода
        (r_mid, -hw * 1.02),
        (r_sh2, -hw * 1.00),
        (r_sh, -hw * 0.93),
        (r_out, -hw * 0.76),
        # 4 продольные водоотводящие канавки протектора
        (r_out, -hw * 0.50), (r_grv, -hw * 0.47), (r_grv, -hw * 0.42), (r_out, -hw * 0.39),
        (r_out, -hw * 0.16), (r_grv, -hw * 0.13), (r_grv, -hw * 0.08), (r_out, -hw * 0.05),
        (r_out, +hw * 0.05), (r_grv, +hw * 0.08), (r_grv, +hw * 0.13), (r_out, +hw * 0.16),
        (r_out, +hw * 0.39), (r_grv, +hw * 0.42), (r_grv, +hw * 0.47), (r_out, +hw * 0.50),
        (r_out, +hw * 0.76),
        (r_sh, +hw * 0.93),
        (r_sh2, +hw * 1.00),
        (r_mid, +hw * 1.02),
        (r_bead + 0.010, +hw * 0.95),
        (r_bead, +hw * 0.88),
    ]
    _add_y_annulus(bm_tire, r_bead, r_out, -hw, hw, seg=seg, profile=tire_profile)
    tire = bmesh_to_object(f'Wheel_{tag}', bm_tire, material=mats['rubber'],
                           smooth=True, collection=collection, auto_smooth_deg=60.0,
                           subsurf=1)
    uv_cyl(tire, WHEEL_R, WHEEL_WIDTH)
    tire.location = Vector((axle_x, cy, cz))

    # 2. 19" двухцветный 10-спицевый диск Volvo S90 Inscription Diamond-Cut:
    # slot 0 = Alloy (полированные лицевые грани спиц и наружная кромка обода),
    # slot 1 = Piano_Black (тёмный барабан обода, боковины спиц и центральный колпачок),
    # slot 2 = Chrome (5 объёмных колёсных гаек и хромированное кольцо ступицы)
    bm_rim = bmesh.new()
    # 2a. Внутренний глянцево-чёрный барабан обода (slot 1 = Piano_Black)
    _add_y_annulus(bm_rim, 0.222, 0.244, -hw * 0.88, hw * 0.80, seg=seg)
    bm_rim.faces.ensure_lookup_table()
    for f_b in bm_rim.faces:
        f_b.material_index = 1

    # 2b. Полированный лицевой кант обода Diamond-Cut (slot 0 = Alloy)
    n_before_lip = len(bm_rim.faces)
    y_lip0 = side * (hw * 0.72)
    y_lip1 = side * (hw * 0.89)
    _add_y_annulus(bm_rim, 0.224, 0.245, min(y_lip0, y_lip1), max(y_lip0, y_lip1), seg=seg)
    bm_rim.faces.ensure_lookup_table()
    for f_idx in range(n_before_lip, len(bm_rim.faces)):
        bm_rim.faces[f_idx].material_index = 0

    # 2c. Компактная ступица (утопленная чаша под болты + чёрный центральный колпачок)
    n_before_hub = len(bm_rim.faces)
    y_hub_in = side * (hw * 0.14)
    y_hub_mid = side * (hw * 0.46)
    y_cap_out = side * (hw * 0.56)
    # Основание ступицы под 5 болтов (Alloy)
    _add_y_cylinder(bm_rim, 0.068, min(y_hub_in, y_hub_mid), max(y_hub_in, y_hub_mid),
                    seg=24, center_xz=(0.0, 0.0))
    bm_rim.faces.ensure_lookup_table()
    for f_idx in range(n_before_hub, len(bm_rim.faces)):
        bm_rim.faces[f_idx].material_index = 0
    # Компактный глянцево-чёрный центральный колпачок (r = 0.034, slot 1 = Piano_Black)
    n_before_cap = len(bm_rim.faces)
    _add_y_cylinder(bm_rim, 0.034, min(y_hub_mid, y_cap_out), max(y_hub_mid, y_cap_out),
                    seg=20, center_xz=(0.0, 0.0))
    bm_rim.faces.ensure_lookup_table()
    for f_idx in range(n_before_cap, len(bm_rim.faces)):
        bm_rim.faces[f_idx].material_index = 1
    # Хромированное колечко вокруг колпачка, эмблема Iron Mark и 5 колёсных гаек (slot 2)
    n_before_lugs = len(bm_rim.faces)
    y_ring0 = side * (hw * 0.50)
    y_ring1 = side * (hw * 0.57)
    _add_y_annulus(bm_rim, 0.032, 0.038, min(y_ring0, y_ring1), max(y_ring0, y_ring1), seg=24)
    # Хромированное мини-кольцо и поперечина Volvo Iron Mark на центральном колпачке ступицы
    y_em0 = side * (hw * 0.555)
    y_em1 = side * (hw * 0.574)
    _add_y_annulus(bm_rim, 0.015, 0.019, min(y_em0, y_em1), max(y_em0, y_em1), seg=20)
    add_box(bm_rim, -0.020, +0.020, min(y_em0, y_em1), max(y_em0, y_em1), -0.0035, +0.0035)
    y_lug0 = side * (hw * 0.44)
    y_lug1 = side * (hw * 0.55)
    for k_lug in range(5):
        a_l = 2.0 * math.pi * k_lug / 5.0
        _add_y_cylinder(bm_rim, 0.0085, min(y_lug0, y_lug1), max(y_lug0, y_lug1),
                        seg=6, center_xz=(0.052 * math.cos(a_l), 0.052 * math.sin(a_l)))
    bm_rim.faces.ensure_lookup_table()
    for f_idx in range(n_before_lugs, len(bm_rim.faces)):
        bm_rim.faces[f_idx].material_index = 2

    # 2d. 10 длинных стройных двухцветных V-спиц (вогнутый профиль от r=0.044 к ободу r=0.226):
    # наружная лицевая грань = полированный Alloy [0], боковые грани = Piano_Black [1]
    for pair_idx in range(5):
        base_ang = 2.0 * math.pi * pair_idx / 5.0
        for split in (-0.20, +0.20):
            ang_in = base_ang + split * 0.52
            ang_out = base_ang + split * 1.18
            y_dir = Vector((0.0, side, 0.0))
            sp_specs = [
                (0.044, ang_in, hw * 0.42, 0.0095, 0.011),
                (0.115, 0.5 * (ang_in + ang_out), hw * 0.62, 0.0085, 0.012),
                (0.175, ang_out * 0.85 + ang_in * 0.15, hw * 0.74, 0.0075, 0.011),
                (0.226, ang_out, hw * 0.79, 0.0070, 0.010),
            ]
            rings = []
            for r_s, a_s, y_s, sw, sd in sp_specs:
                rad_d = Vector((math.cos(a_s), 0.0, math.sin(a_s)))
                tan_d = Vector((-math.sin(a_s), 0.0, math.cos(a_s)))
                p = rad_d * r_s + y_dir * y_s
                rings.append([
                    bm_rim.verts.new(p - tan_d * sw - y_dir * sd),
                    bm_rim.verts.new(p + tan_d * sw - y_dir * sd),
                    bm_rim.verts.new(p + tan_d * (sw * 0.82) + y_dir * sd),
                    bm_rim.verts.new(p - tan_d * (sw * 0.82) + y_dir * sd),
                ])
            for i_s in range(len(rings) - 1):
                r0, r1 = rings[i_s], rings[i_s + 1]
                for k in range(4):
                    kn = (k + 1) % 4
                    f_sp = bm_rim.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
                    # k == 2 — наружная лицевая грань спицы (Diamond-Cut Alloy), остальные чёрные
                    f_sp.material_index = 0 if k == 2 else 1
            f_e0 = bm_rim.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
            f_e1 = bm_rim.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))
            f_e0.material_index = 1
            f_e1.material_index = 1

    rim = bmesh_to_object(f'Rim_{tag}', bm_rim, material=mats['alloy'],
                          smooth=True, collection=collection, auto_smooth_deg=50.0,
                          bevel=0.0014, bevel_segs=2)
    rim.data.materials.append(mats['piano_black'])
    rim.data.materials.append(mats['chrome'])
    uv_box(rim, scale=4.0)
    rim.parent = tire
    rim.location = Vector((0.0, 0.0, 0.0))

    # 3. Вентилируемый тормозной диск (рабочая дорожка Brake_Disc [0] + тёмная ступичная чаша [1])
    bm_disc = bmesh.new()
    y_disc_c = side * (hw * 0.14)
    # Наружное рабочее кольцо диска (слот 0 = Brake_Disc)
    _add_y_annulus(bm_disc, 0.092, 0.172, y_disc_c - 0.012, y_disc_c + 0.012, seg=44)
    # Внутренняя тёмная ступичная «шляпа» диска (слот 1 = Carbon)
    n_bef_hat = len(bm_disc.faces)
    y_hat_out = y_disc_c + side * 0.022
    y_hat_in = y_disc_c - side * 0.010
    _add_y_cylinder(bm_disc, 0.094, min(y_hat_in, y_hat_out), max(y_hat_in, y_hat_out),
                    seg=32, center_xz=(0.0, 0.0))
    bm_disc.faces.ensure_lookup_table()
    for f_idx in range(n_bef_hat, len(bm_disc.faces)):
        bm_disc.faces[f_idx].material_index = 1
    disc = bmesh_to_object(f'Brake_Disc_{tag}', bm_disc, material=mats['brake_disc'],
                           smooth=True, collection=collection, auto_smooth_deg=45.0,
                           bevel=0.0015, bevel_segs=1)
    disc.data.materials.append(mats['carbon'])
    uv_box(disc, scale=5.0)
    disc.parent = tire
    disc.location = Vector((0.0, 0.0, 0.0))

    # 4. Двухсекционный спортивный тормозной суппорт (Caliper [0] + серебристая скоба [1])
    bm_cal = bmesh.new()
    cal_x = -0.142 if axle_x > 0 else +0.142  # на передней оси сзади, на задней спереди
    # Основной моноблок суппорта + два выступающих ребра цилиндров (slot 0 = Caliper)
    add_box(bm_cal, cal_x - 0.036, cal_x + 0.034,
            y_disc_c - 0.026, y_disc_c + 0.028, -0.062, 0.072)
    add_box(bm_cal, cal_x - 0.028, cal_x + 0.024,
            y_disc_c + side * 0.024, y_disc_c + side * 0.034, +0.012, +0.056)
    add_box(bm_cal, cal_x - 0.028, cal_x + 0.024,
            y_disc_c + side * 0.024, y_disc_c + side * 0.034, -0.050, -0.006)
    # Серебристая фиксирующая скоба колодок в центре суппорта (slot 1 = Alloy)
    n_bef_clip = len(bm_cal.faces)
    add_box(bm_cal, cal_x - 0.014, cal_x + 0.014,
            y_disc_c + side * 0.029, y_disc_c + side * 0.036, -0.005, +0.011)
    bm_cal.faces.ensure_lookup_table()
    for f_idx in range(n_bef_clip, len(bm_cal.faces)):
        bm_cal.faces[f_idx].material_index = 1
    caliper = bmesh_to_object(f'Brake_Caliper_{tag}', bm_cal, material=mats['caliper'],
                              smooth=True, collection=collection, auto_smooth_deg=55.0,
                              bevel=0.0035, bevel_segs=2)
    caliper.data.materials.append(mats['alloy'])
    uv_box(caliper, scale=8.0)
    caliper.location = Vector((axle_x, cy, cz))
    if body_ob is not None:
        caliper.parent = body_ob
        caliper.matrix_parent_inverse = body_ob.matrix_world.inverted()

    return tire, rim, disc, caliper


# --------------------------------------------------------------------------- #
# 6. Салон (замкнутые непересекающиеся объёмы, UV на каждом меше)
# --------------------------------------------------------------------------- #
def add_box(bm, x0, x1, y0, y1, z0, z1):
    """Добавляет замкнутый параллелепипед в BMesh; автоматически нормализует min/max."""
    xa, xb = (x0, x1) if x0 <= x1 else (x1, x0)
    ya, yb = (y0, y1) if y0 <= y1 else (y1, y0)
    za, zb = (z0, z1) if z0 <= z1 else (z1, z0)
    pts = {}
    for ix, x in enumerate((xa, xb)):
        for iy, y in enumerate((ya, yb)):
            for iz, z in enumerate((za, zb)):
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
    return pts


def _make_box_object(name, bounds, material, collection, uv_scale=1.5):
    bm = bmesh.new()
    add_box(bm, *bounds)
    ob = bmesh_to_object(name, bm, material=material, smooth=True, collection=collection)
    uv_box(ob, scale=uv_scale)
    return ob


def _build_steering_wheel(mats, collection):
    """Скульптурный 3-спицевый руль Volvo S90: круглый тороидальный обод (36×12) с приливами
    «10 и 2 часа», округлая подушка безопасности с Iron Mark, блоки кнопок мультируля и хром."""
    bm = bmesh.new()
    r_maj = 0.172
    r_min = 0.0175
    n_maj, n_min = 36, 12

    # 1. Круглый тороидальный кожаный обод в плоскости YZ (slot 0 = Leather)
    torus_rings = []
    for i in range(n_maj):
        a = 2.0 * math.pi * i / n_maj
        ca, sa = math.cos(a), math.sin(a)
        # Эргономичные утолщения-приливы на «10 и 2 часа» (a ≈ 35° и 145°)
        grip = 1.0 + 0.14 * (
            math.exp(-((a - math.radians(35.0)) / 0.25) ** 2)
            + math.exp(-((a - math.radians(145.0)) / 0.25) ** 2)
        )
        rr = r_min * grip
        ring = []
        for j in range(n_min):
            b = 2.0 * math.pi * j / n_min
            cb, sb = math.cos(b), math.sin(b)
            rad = r_maj + rr * cb
            xx = rr * sb
            ring.append(bm.verts.new(Vector((xx, rad * ca, rad * sa))))
        torus_rings.append(ring)
    for i in range(n_maj):
        inext = (i + 1) % n_maj
        r0, r1 = torus_rings[i], torus_rings[inext]
        for j in range(n_min):
            jnext = (j + 1) % n_min
            bm.faces.new((r0[j], r0[jnext], r1[jnext], r1[j]))

    # 2. Скульптурная кругло-трапециевидная центральная подушка безопасности (slot 0 = Leather)
    _add_x_cylinder(bm, 0.056, -0.032, +0.024, seg=24, center_yz=(0.0, 0.0))
    add_box(bm, -0.024, +0.018, -0.044, +0.044, -0.064, -0.036)

    # 3. Три профилированные спицы (левая, правая, сдвоенная нижняя) + блоки кнопок (slot 1)
    n_pb0 = len(bm.faces)
    # Левая и правая спицы с глянцевыми пультами кнопок мультируля (Piano_Black)
    for sy_s in (-1.0, +1.0):
        ya, yb = sorted((sy_s * 0.058, sy_s * (r_maj - r_min - 0.002)))
        add_box(bm, -0.014, +0.014, ya, yb, -0.019, +0.019)
        # Выступающий блок глянцевых кнопок мультируля со стороны водителя (-X)
        yc0, yc1 = sorted((sy_s * 0.070, sy_s * 0.125))
        add_box(bm, -0.021, -0.015, yc0, yc1, -0.016, +0.016)
    # Нижняя сдвоенная спица (две боковые ветви + нижняя перемычка)
    add_box(bm, -0.012, +0.012, -0.034, -0.014, -(r_maj - r_min - 0.002), -0.066)
    add_box(bm, -0.012, +0.012, +0.014, +0.034, -(r_maj - r_min - 0.002), -0.066)
    # Глянцево-чёрная подложка знака Volvo Iron Mark на ступице руля (slot 1 = Piano_Black)
    _add_x_cylinder(bm, 0.018, -0.0355, -0.032, seg=20, center_yz=(0.0, 0.0))
    n_pb1 = len(bm.faces)

    # 4. Хромированные акценты Volvo: двухслойный 3D Iron Mark (кольцо + лента + стрела)
    seg_sw = 20
    r_sw_in, r_sw_out = 0.0175, 0.0225
    sw_ring = []
    for k_sw in range(seg_sw):
        ang = 2.0 * math.pi * k_sw / seg_sw
        ca, sa = math.cos(ang), math.sin(ang)
        sw_ring.append([
            bm.verts.new(Vector((-0.0325, r_sw_in * ca, r_sw_in * sa))),
            bm.verts.new(Vector((-0.0375, r_sw_in * ca, r_sw_in * sa))),
            bm.verts.new(Vector((-0.0375, r_sw_out * ca, r_sw_out * sa))),
            bm.verts.new(Vector((-0.0325, r_sw_out * ca, r_sw_out * sa))),
        ])
    for k_sw in range(seg_sw):
        r0, r1 = sw_ring[k_sw], sw_ring[(k_sw + 1) % seg_sw]
        for kk in range(4):
            kn = (kk + 1) % 4
            bm.faces.new((r0[kk], r0[kn], r1[kn], r1[kk]))
    # Поперечина VOLVO + стрела Марса вверх-вправо (при взгляде водителя по +X право это -Y!)
    add_box(bm, -0.038, -0.033, -0.025, +0.025, -0.0042, +0.0042)
    add_box(bm, -0.0375, -0.033, -0.024, -0.014, +0.015, +0.020)
    add_box(bm, -0.0375, -0.033, -0.024, -0.019, +0.009, +0.0142)
    # Хромированная окантовка нижней спицы и кнопок
    add_box(bm, -0.016, -0.0125, -0.036, +0.036, -(r_maj - r_min + 0.004), -(r_maj - r_min - 0.014))
    for sy_s in (-1.0, +1.0):
        yc0, yc1 = sorted((sy_s * 0.072, sy_s * 0.122))
        add_box(bm, -0.0225, -0.0212, yc0, yc1, -0.018, -0.015)

    bm.faces.ensure_lookup_table()
    for fi in range(n_pb0, n_pb1):
        bm.faces[fi].material_index = 1
    for fi in range(n_pb1, len(bm.faces)):
        bm.faces[fi].material_index = 2

    sw = bmesh_to_object('SteeringWheel', bm, material=mats['leather'],
                         smooth=True, collection=collection, auto_smooth_deg=55.0)
    sw.data.materials.append(mats['piano_black'])
    sw.data.materials.append(mats['chrome'])
    uv_box(sw, scale=4.0)
    # Левый руль (+Y), слегка наклонён к водителю (-Y поворот на 20° вокруг Y)
    sw.location = Vector((0.52, 0.36, 0.875))
    sw.rotation_euler = (0.0, math.radians(-20.0), 0.0)
    return sw


def build_interior(mats, collection, door_objs=None):
    objs = []
    # Премиальный скандинавский салон Volvo S90 Inscription:
    # * анатомические многосекционные кресла с удлинителем подушки, сужающимися спинками
    #   и стройными подголовниками Volvo на хромированных стойках;
    # * двухъярусная торпеда со шпоном Linear Walnut, цифровой приборкой 12.3",
    #   вертикальным OLED-планшетом Sensus 9", физическими кнопками и 4 дефлекторами;
    # * центральная консоль с хрустальным селектором Orrefors, Start/Stop, подстаканниками;
    # * педали с алюминиевыми накладками, рулевая колонка, потолочный плафон, козырьки,
    #   безрамочное зеркало и многослойные дверные карты со шпоном и динамиками B&W!
    bm = bmesh.new()
    bm_bol = bmesh.new()
    bm_chr = bmesh.new()
    gap = 0.004

    def _add_tapered_block(target_bm, x_b0, x_b1, x_t0, x_t1, yb0, yb1, yt0, yt1, z0, z1):
        """Наклонный сужающийся замкнутый блок (анатомическая спинка/подголовник Volvo)."""
        v = [
            target_bm.verts.new(Vector((x_b0, yb0, z0))),
            target_bm.verts.new(Vector((x_b1, yb0, z0))),
            target_bm.verts.new(Vector((x_b1, yb1, z0))),
            target_bm.verts.new(Vector((x_b0, yb1, z0))),
            target_bm.verts.new(Vector((x_t0, yt0, z1))),
            target_bm.verts.new(Vector((x_t1, yt0, z1))),
            target_bm.verts.new(Vector((x_t1, yt1, z1))),
            target_bm.verts.new(Vector((x_t0, yt1, z1))),
        ]
        for f in (
            (v[0], v[3], v[2], v[1]),
            (v[4], v[5], v[6], v[7]),
            (v[0], v[1], v[5], v[4]),
            (v[2], v[3], v[7], v[6]),
            (v[0], v[4], v[7], v[3]),
            (v[1], v[2], v[6], v[5]),
        ):
            target_bm.faces.new(f)

    # 1. Передние анатомические кресла Volvo (водитель + пассажир)
    for side in (-1.0, 1.0):
        yc = side * 0.37
        # Контурная центральная подушка (лёгкий наклон к спинке) + передний валик под колени
        _add_tapered_block(bm_bol, -0.05, 0.33, -0.05, 0.33,
                           yc - 0.155, yc + 0.155, yc - 0.155, yc + 0.155, 0.365, 0.485)
        # Поперечная стёжка подушки сиденья (3 мягких валика перфорированной кожи Nappa)
        for sx_q in (0.04, 0.14, 0.24):
            add_box(bm_bol, sx_q - 0.036, sx_q + 0.036,
                    yc - 0.148, yc + 0.148, 0.482, 0.493)
        add_box(bm_bol, 0.335, 0.455, yc - 0.155, yc + 0.155, 0.375, 0.495)
        # Обхватывающие боковые валики подушки (основная кожа Cabin)
        add_box(bm, -0.04, 0.44, yc - 0.232, yc - 0.160, 0.365, 0.525)
        add_box(bm, -0.04, 0.44, yc + 0.160, yc + 0.232, 0.365, 0.525)
        # 2-секционная анатомическая спинка: поясничная часть + сужающаяся плечевая часть
        _add_tapered_block(bm_bol, -0.145, -0.045, -0.215, -0.115,
                           yc - 0.155, yc + 0.155, yc - 0.150, yc + 0.150, 0.48, 0.75)
        _add_tapered_block(bm_bol, -0.218, -0.118, -0.275, -0.180,
                           yc - 0.150, yc + 0.150, yc - 0.125, yc + 0.125, 0.754, 0.985)
        # Плавные сужающиеся кверху боковые валики спинки (обнимают торс, не торчат «рогами»)
        _add_tapered_block(bm, -0.125, -0.025, -0.245, -0.155,
                           yc - 0.228, yc - 0.158, yc - 0.185, yc - 0.130, 0.49, 0.90)
        _add_tapered_block(bm, -0.125, -0.025, -0.245, -0.155,
                           yc + 0.158, yc + 0.228, yc + 0.130, yc + 0.185, 0.49, 0.90)
        # Хромированные направляющие подголовника с втулками
        for dy_p in (-0.044, +0.044):
            _add_z_cylinder(bm_chr, 0.0055, 0.980, 1.046, seg=10, center_xy=(-0.228, yc + dy_p))
            _add_z_cylinder(bm_chr, 0.011, 0.982, 0.996, seg=12, center_xy=(-0.228, yc + dy_p))
        # Фирменный тонкий трапециевидный подголовник Volvo (сужается кверху, наклонён вперёд)
        _add_tapered_block(bm, -0.265, -0.185, -0.242, -0.172,
                           yc - 0.088, yc + 0.088, yc - 0.068, yc + 0.068, 1.042, 1.185)

    # 2. Задний диван (3 профилированных места + центральный откидной подлокотник + 3 подголовника)
    for y_rs in (-0.37, +0.37):
        add_box(bm_bol, -0.78, -0.34, y_rs - 0.18, y_rs + 0.18, 0.365, 0.505)
        _add_tapered_block(bm_bol, -0.88, -0.78 - gap, -0.96, -0.86,
                           y_rs - 0.18, y_rs + 0.18, y_rs - 0.16, y_rs + 0.16, 0.46, 0.95)
    # Центральная секция заднего дивана + очерченный откидной подлокотник
    add_box(bm, -0.78, -0.36, -0.185, +0.185, 0.365, 0.498)
    _add_tapered_block(bm, -0.88, -0.78 - gap, -0.96, -0.86,
                       -0.185, +0.185, -0.17, +0.17, 0.46, 0.94)
    for y_hr in (-0.36, 0.0, +0.36):
        hw_b = 0.082 if abs(y_hr) > 0.1 else 0.090
        hw_t = 0.064 if abs(y_hr) > 0.1 else 0.072
        z_hr_top = 1.115 if abs(y_hr) > 0.1 else 1.075
        _add_tapered_block(bm, -0.95, -0.865, -0.928, -0.850,
                           y_hr - hw_b, y_hr + hw_b, y_hr - hw_t, y_hr + hw_t, 0.955, z_hr_top)

    # Задняя полка под покатым задним стеклом (не выступает в нишу багажника TRUNK_X_FRONT)
    add_box(bm, TRUNK_X_FRONT + 0.04, -0.96, -0.56, 0.56, 0.955, 1.00)

    # Центральный тоннель + мягкий подлокотник с раздельной крышкой
    add_box(bm, -0.82, 0.78, -0.125, 0.125, CABIN_FLOOR_Z + 0.028, 0.565)
    add_box(bm, -0.24, 0.14, -0.112, 0.112, 0.568, 0.652)

    cabin = bmesh_to_object('Cabin', bm, material=mats['leather'],
                            smooth=True, collection=collection, auto_smooth_deg=65.0,
                            bevel=0.026, bevel_segs=3)
    uv_box(cabin, scale=1.5)
    objs.append(cabin)

    cabin_bol = bmesh_to_object('Cabin_Seats_Insert', bm_bol, material=mats['leather_light'],
                                smooth=True, collection=collection, auto_smooth_deg=65.0,
                                bevel=0.026, bevel_segs=3)
    uv_box(cabin_bol, scale=1.6)
    objs.append(cabin_bol)

    cabin_chr = bmesh_to_object('Cabin_Chrome', bm_chr, material=mats['chrome'],
                                smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
    uv_box(cabin_chr, scale=8.0)
    objs.append(cabin_chr)

    # 3. Центральная консоль (шпон Linear Walnut + подстаканники + Orrefors + Start/Stop)
    bm_cc = bmesh.new()
    # Наклонная верхняя декоративная площадка из шпона (slot 0 = Wood)
    add_box(bm_cc, 0.15, 0.60, -0.108, +0.108, 0.566, 0.580)
    # 2 утопленных колодца подстаканников справа от селектора (slot 1 = Piano_Black)
    f_cc_pb0 = len(bm_cc.faces)
    _add_z_cylinder(bm_cc, 0.034, 0.576, 0.583, seg=20, center_xy=(0.24, -0.044))
    _add_z_cylinder(bm_cc, 0.034, 0.576, 0.583, seg=20, center_xy=(0.33, -0.044))
    # Основание селектора АКПП и кнопка электроручника
    add_box(bm_cc, 0.36, 0.52, -0.015, +0.085, 0.580, 0.588)
    add_box(bm_cc, 0.18, 0.24, +0.025, +0.075, 0.580, 0.592)
    f_cc_pb1 = len(bm_cc.faces)
    # Хром-периметр консоли, селектор, Start/Stop и Drive Mode (slot 2 = Chrome)
    add_box(bm_cc, 0.148, 0.602, -0.114, -0.105, 0.568, 0.584)
    add_box(bm_cc, 0.148, 0.602, +0.105, +0.114, 0.568, 0.584)
    # Селектор АКПП: хромированный шток + гранёная рукоятка
    _add_z_cylinder(bm_cc, 0.010, 0.586, 0.652, seg=14, center_xy=(0.44, +0.035))
    add_box(bm_cc, 0.422, 0.464, +0.012, +0.058, 0.650, 0.692)
    # Фирменный поворотный выключатель Engine Start/Stop (ромб-цилиндр)
    _add_z_cylinder(bm_cc, 0.015, 0.582, 0.606, seg=8, center_xy=(0.31, +0.048))
    # Металлический цилиндр-роллер Drive Mode
    _add_y_cylinder(bm_cc, 0.010, +0.035, +0.062, seg=14, center_xz=(0.26, 0.590))
    bm_cc.faces.ensure_lookup_table()
    for fi in range(f_cc_pb0, f_cc_pb1):
        bm_cc.faces[fi].material_index = 1
    for fi in range(f_cc_pb1, len(bm_cc.faces)):
        bm_cc.faces[fi].material_index = 2
    cc_ob = bmesh_to_object('Center_Console', bm_cc, material=mats['wood'],
                            smooth=True, collection=collection, bevel=0.0020, bevel_segs=1)
    cc_ob.data.materials.append(mats['piano_black'])
    cc_ob.data.materials.append(mats['chrome'])
    uv_box(cc_ob, scale=3.0)
    objs.append(cc_ob)

    # 4. Двухъярусная скульптурная панель приборов (Dashboard)
    bm_dash = bmesh.new()
    # Основная нижняя база торпеды
    add_box(bm_dash, 0.64, 0.86, -0.73, +0.73, 0.56, 0.87)
    # Верхний кожаный козырёк-навес (чуть выдвинут в салон, создаёт глубокую тень над декором)
    add_box(bm_dash, 0.60, 0.85, -0.73, +0.73, 0.872, 0.940)
    # Скульптурный купол-козырёк над цифровой приборкой водителя (+Y)
    _add_tapered_block(bm_dash, 0.56, 0.82, 0.59, 0.82,
                       +0.15, +0.58, +0.19, +0.54, 0.935, 1.005)
    # Выступающая крышка перчаточного ящика (бардачка) со стороны пассажира (-Y)
    add_box(bm_dash, 0.615, 0.642, -0.68, -0.18, 0.59, 0.745)
    dash = bmesh_to_object('Dashboard', bm_dash, material=mats['leather'],
                           smooth=True, collection=collection, auto_smooth_deg=65.0,
                           bevel=0.018, bevel_segs=3)
    uv_box(dash, scale=1.5)
    objs.append(dash)

    # 4b. Декоративная вставка из шпона Linear Walnut + хромированный кант + ручка бардачка
    bm_dt = bmesh.new()
    # Широкая планка шпона со стороны пассажира и водителя (slot 0 = Wood)
    add_box(bm_dt, 0.612, 0.645, -0.66, -0.175, 0.765, 0.855)
    add_box(bm_dt, 0.612, 0.645, +0.58, +0.69, 0.765, 0.855)
    n_dt_w = len(bm_dt.faces)
    # Тонкий хромированный молдинг под шпоном + хромированная ручка бардачка (slot 1 = Chrome)
    add_box(bm_dt, 0.606, 0.640, -0.66, -0.175, 0.752, 0.762)
    add_box(bm_dt, 0.606, 0.640, +0.58, +0.69, 0.752, 0.762)
    add_box(bm_dt, 0.604, 0.620, -0.32, -0.22, 0.715, 0.732)
    bm_dt.faces.ensure_lookup_table()
    for fi in range(n_dt_w, len(bm_dt.faces)):
        bm_dt.faces[fi].material_index = 1
    dt_ob = bmesh_to_object('Dash_Trim', bm_dt, material=mats['wood'],
                            smooth=True, collection=collection, bevel=0.0025, bevel_segs=2)
    dt_ob.data.materials.append(mats['chrome'])
    uv_box(dt_ob, scale=3.0)
    objs.append(dt_ob)

    # 4c. Цифровая приборная панель 12.3" (Instrument_Cluster) с реальной OLED-графикой!
    bm_ic = bmesh.new()
    # Чёрная глянцевая рамка-ниша приборки (slot 0 = Piano_Black)
    add_box(bm_ic, 0.594, 0.635, +0.195, +0.535, 0.808, 0.942)
    n_ic_ui0 = len(bm_ic.faces)
    # OLED-дисплей приборной панели (slot 1 = Cluster_Digital_UI)
    add_box(bm_ic, 0.588, 0.593, +0.208, +0.522, 0.820, 0.930)
    n_ic_ui1 = len(bm_ic.faces)
    # Два полых хромированных 3D-ободка точно вокруг шкал спидометра и тахометра (slot 2)
    for y_dial in (+0.458, +0.272):
        seg_d = 28
        r_di, r_do = 0.0415, 0.0455
        d_ring = []
        for kd in range(seg_d):
            ang = 2.0 * math.pi * kd / seg_d
            ca, sa = math.cos(ang), math.sin(ang)
            d_ring.append([
                bm_ic.verts.new(Vector((0.5875, y_dial + r_di * ca, 0.875 + r_di * sa))),
                bm_ic.verts.new(Vector((0.5835, y_dial + r_di * ca, 0.875 + r_di * sa))),
                bm_ic.verts.new(Vector((0.5835, y_dial + r_do * ca, 0.875 + r_do * sa))),
                bm_ic.verts.new(Vector((0.5875, y_dial + r_do * ca, 0.875 + r_do * sa))),
            ])
        for kd in range(seg_d):
            r0, r1 = d_ring[kd], d_ring[(kd + 1) % seg_d]
            for kk in range(4):
                kn = (kk + 1) % 4
                bm_ic.faces.new((r0[kk], r0[kn], r1[kn], r1[kk]))
    bm_ic.faces.ensure_lookup_table()
    for fi in range(n_ic_ui0, n_ic_ui1):
        bm_ic.faces[fi].material_index = 1
    for fi in range(n_ic_ui1, len(bm_ic.faces)):
        bm_ic.faces[fi].material_index = 2
    ic_ob = bmesh_to_object('Instrument_Cluster', bm_ic, material=mats['piano_black'],
                            smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
    ic_ob.data.materials.append(mats['cluster_ui'])
    ic_ob.data.materials.append(mats['chrome'])
    uv_box(ic_ob, scale=3.0)
    uv_face_x_minus(ic_ob, 1, +0.522, +0.208, 0.820, 0.930)
    objs.append(ic_ob)

    # 4d. Центральный вертикальный планшет Volvo Sensus 9" + панель кнопок и крутилка громкости
    bm_sc = bmesh.new()
    # Глянцевый корпус экрана + нижняя полоса физических кнопок (slot 0 = Piano_Black)
    add_box(bm_sc, 0.592, 0.638, -0.105, +0.105, 0.670, 0.918)
    # 6 физических клавиш под экраном
    for kb_y in (-0.082, -0.056, -0.032, +0.032, +0.056, +0.082):
        add_box(bm_sc, 0.583, 0.592, kb_y - 0.009, kb_y + 0.009, 0.678, 0.692)
    n_sc_ui0 = len(bm_sc.faces)
    # Подсвеченный OLED-дисплей Sensus (slot 1 = Screen_Sensus_UI)
    add_box(bm_sc, 0.586, 0.591, -0.092, +0.092, 0.708, 0.904)
    n_sc_ui1 = len(bm_sc.faces)
    # Хромированный кант экрана + центральная крутилка громкости + кнопка Home (slot 2 = Chrome)
    add_box(bm_sc, 0.587, 0.594, -0.108, +0.108, 0.916, 0.922)
    add_box(bm_sc, 0.587, 0.594, -0.108, +0.108, 0.665, 0.671)
    _add_x_cylinder(bm_sc, 0.013, 0.574, 0.591, seg=18, center_yz=(0.0, 0.685))
    add_box(bm_sc, 0.584, 0.590, -0.022, +0.022, 0.698, 0.704)
    bm_sc.faces.ensure_lookup_table()
    for fi in range(n_sc_ui0, n_sc_ui1):
        bm_sc.faces[fi].material_index = 1
    for fi in range(n_sc_ui1, len(bm_sc.faces)):
        bm_sc.faces[fi].material_index = 2
    screen = bmesh_to_object('Screen', bm_sc, material=mats['piano_black'],
                             smooth=True, collection=collection, bevel=0.0020, bevel_segs=1)
    screen.data.materials.append(mats['screen_ui'])
    screen.data.materials.append(mats['chrome'])
    uv_box(screen, scale=2.0)
    uv_face_x_minus(screen, 1, +0.092, -0.092, 0.708, 0.904)
    objs.append(screen)

    # 4e. 4 фирменных вертикальных дефлектора обдува Volvo (2 у Sensus + 2 по краям торпеды)
    bm_av = bmesh.new()
    chr_av_ranges = []
    for y_v, z0_v, z1_v in ((-0.136, 0.700, 0.908), (+0.136, 0.700, 0.908),
                            (-0.695, 0.755, 0.875), (+0.695, 0.755, 0.875)):
        # Корпус-рамка дефлектора (slot 0 = Piano_Black)
        add_box(bm_av, 0.596, 0.638, y_v - 0.024, y_v + 0.024, z0_v, z1_v)
        # 5 горизонтальных рельефных жалюзи внутри дефлектора
        for k_l in range(5):
            zl = z0_v + 0.016 + (z1_v - z0_v - 0.032) * (k_l / 4.0)
            add_box(bm_av, 0.588, 0.596, y_v - 0.019, y_v + 0.019, zl - 0.0025, zl + 0.0025)
        # Фирменный вертикальный хромированный ползунок Volvo по центру (slot 1 = Chrome)
        fa_c = len(bm_av.faces)
        add_box(bm_av, 0.582, 0.592, y_v - 0.0045, y_v + 0.0045, z0_v + 0.012, z1_v - 0.012)
        add_box(bm_av, 0.578, 0.586, y_v - 0.0075, y_v + 0.0075,
                0.5 * (z0_v + z1_v) - 0.012, 0.5 * (z0_v + z1_v) + 0.012)
        chr_av_ranges.append((fa_c, len(bm_av.faces)))
    bm_av.faces.ensure_lookup_table()
    for fa, fb in chr_av_ranges:
        for fi in range(fa, fb):
            bm_av.faces[fi].material_index = 1
    av_ob = bmesh_to_object('Air_Vents', bm_av, material=mats['piano_black'],
                            smooth=True, collection=collection, bevel=0.0012, bevel_segs=1)
    av_ob.data.materials.append(mats['chrome'])
    uv_box(av_ob, scale=6.0)
    objs.append(av_ob)

    # 5. Рулевая колонка + подрулевые переключатели (не вращаются вместе с рулём!)
    bm_scol = bmesh.new()
    _add_x_cylinder(bm_scol, 0.044, 0.545, 0.650, seg=18, center_yz=(0.36, 0.875))
    # Левый рычаг поворотников и правый рычаг стеклоочистителей
    _add_y_cylinder(bm_scol, 0.007, 0.405, 0.515, seg=12, center_xz=(0.575, 0.890))
    _add_y_cylinder(bm_scol, 0.007, 0.205, 0.315, seg=12, center_xz=(0.575, 0.890))
    scol_ob = bmesh_to_object('Steering_Column', bm_scol, material=mats['piano_black'],
                              smooth=True, collection=collection, bevel=0.002, bevel_segs=1)
    uv_box(scol_ob, scale=6.0)
    objs.append(scol_ob)

    # 6. Педальный узел водителя (алюминиевая площадка отдыха + тормоз + напольный газ)
    bm_ped = bmesh.new()
    # Площадка отдыха левой ноги + педаль тормоза + педаль газа (slot 0 = Alloy)
    _add_tapered_block(bm_ped, 0.67, 0.69, 0.74, 0.76, +0.52, +0.60, +0.52, +0.60, 0.34, 0.50)
    _add_tapered_block(bm_ped, 0.65, 0.67, 0.69, 0.71, +0.34, +0.44, +0.34, +0.44, 0.38, 0.45)
    _add_tapered_block(bm_ped, 0.68, 0.70, 0.74, 0.76, +0.22, +0.28, +0.22, +0.28, 0.33, 0.48)
    n_ped_al = len(bm_ped.faces)
    # Противоскользящие резиновые накладки и металлические рычаги педалей (slot 1 = Rubber)
    add_box(bm_ped, 0.67, 0.74, +0.38, +0.40, 0.44, 0.56)
    for zp in (0.37, 0.41, 0.45):
        add_box(bm_ped, 0.665, 0.715, +0.53, +0.59, zp, zp + 0.012)
    bm_ped.faces.ensure_lookup_table()
    for fi in range(n_ped_al, len(bm_ped.faces)):
        bm_ped.faces[fi].material_index = 1
    ped_ob = bmesh_to_object('Pedals', bm_ped, material=mats['alloy'],
                             smooth=True, collection=collection, bevel=0.002, bevel_segs=1)
    ped_ob.data.materials.append(mats['rubber'])
    uv_box(ped_ob, scale=8.0)
    objs.append(ped_ob)

    # 7. Безрамочное салонное зеркало заднего вида (строго внутри салона под лобовым стеклом!)
    bm_rvm = bmesh.new()
    # Тонкий корпус + кронштейн + датчик дождя у верхней кромки стекла (slot 0 = Piano_Black)
    add_box(bm_rvm, 0.244, 0.268, -0.098, +0.098, 1.248, 1.298)
    _add_z_cylinder(bm_rvm, 0.007, 1.296, 1.334, seg=12, center_xy=(0.278, 0.0))
    add_box(bm_rvm, 0.258, 0.304, -0.028, +0.028, 1.320, 1.342)
    n_rvm_pb = len(bm_rvm.faces)
    # Зеркальное полотно со стороны водителя (-X, slot 1 = Chrome)
    add_box(bm_rvm, 0.240, 0.243, -0.094, +0.094, 1.251, 1.295)
    bm_rvm.faces.ensure_lookup_table()
    for fi in range(n_rvm_pb, len(bm_rvm.faces)):
        bm_rvm.faces[fi].material_index = 1
    rvm = bmesh_to_object('Rearview_Mirror', bm_rvm, material=mats['piano_black'],
                          smooth=True, collection=collection, bevel=0.0025, bevel_segs=2)
    rvm.data.materials.append(mats['chrome'])
    uv_box(rvm, scale=6.0)
    objs.append(rvm)

    # 8. Потолочная консоль (плафон освещения + 2 солнцезащитных козырька)
    bm_rf_in = bmesh.new()
    z_rf_in = ZR(0.0) - 0.068
    # Два солнцезащитных козырька (slot 0 = Headliner)
    for sy_v in (-1.0, +1.0):
        ya_v, yb_v = sorted((sy_v * 0.14, sy_v * 0.48))
        add_box(bm_rf_in, 0.12, 0.27, ya_v, yb_v, z_rf_in - 0.018, z_rf_in)
    # Центральный потолочный плафон и корпус кнопок SOS (slot 1 = Piano_Black)
    n_rf_pb0 = len(bm_rf_in.faces)
    add_box(bm_rf_in, 0.06, 0.22, -0.095, +0.095, z_rf_in - 0.022, z_rf_in)
    n_rf_pb1 = len(bm_rf_in.faces)
    # Два рассеивателя плафона освещения салона (slot 2 = White)
    for sy_l in (-0.042, +0.042):
        add_box(bm_rf_in, 0.09, 0.17, sy_l - 0.026, sy_l + 0.026, z_rf_in - 0.026, z_rf_in - 0.022)
    bm_rf_in.faces.ensure_lookup_table()
    for fi in range(n_rf_pb0, n_rf_pb1):
        bm_rf_in.faces[fi].material_index = 1
    for fi in range(n_rf_pb1, len(bm_rf_in.faces)):
        bm_rf_in.faces[fi].material_index = 2
    rf_in_ob = bmesh_to_object('Interior_Roof_Details', bm_rf_in, material=mats['headliner'],
                               smooth=True, collection=collection, bevel=0.003, bevel_segs=2)
    rf_in_ob.data.materials.append(mats['piano_black'])
    rf_in_ob.data.materials.append(mats['white'])
    uv_box(rf_in_ob, scale=3.0)
    objs.append(rf_in_ob)

    # Руль с собственным центром вращения
    sw = _build_steering_wheel(mats, collection)
    objs.append(sw)

    # Ковёр салона (лежит прямо поверх пола выреза CABIN_FLOOR_Z)
    carpet = _make_box_object(
        'Carpet',
        (CABIN_CUT_X_REAR + 0.02, CABIN_CUT_X_FRONT - 0.02, -0.70, 0.70,
         CABIN_FLOOR_Z + 0.008, CABIN_FLOOR_Z + 0.026),
        mats['carpet'], collection, uv_scale=2.0,
    )
    objs.append(carpet)

    # Потолок салона (под отдельной панелью крыши Roof)
    zhl = ZR(0.0) - 0.056
    headliner = _make_box_object(
        'Headliner',
        (ROOF_X_REAR + 0.04, ROOF_X_FRONT - 0.04, -0.52, 0.52, zhl, zhl + 0.012),
        mats['headliner'], collection, uv_scale=2.0,
    )
    objs.append(headliner)

    # 8b. Ремни безопасности (Seatbelts) на стойках B и C + замки с красными кнопками у консоли
    bm_sb = bmesh.new()
    # Тканевые ленты ремней (slot 0 = Rubber / матовая стропа)
    for sy_b in (-1.0, +1.0):
        # Передний ремень вдоль внутренней грани стойки B (от верхнего регулятора к катушке)
        y_top_b = sy_b * 0.715
        y_bot_b = sy_b * 0.745
        p_front = [
            Vector((-0.070, y_top_b, 1.135)),
            Vector((-0.072, y_top_b + sy_b * 0.015, 0.760)),
            Vector((-0.068, y_bot_b, 0.340)),
        ]
        _sweep_rect_tube(bm_sb, p_front, half_y=0.0035, half_n=0.022)
        # Задний ремень вдоль внутренней панели стойки C
        p_rear = [
            Vector((-0.895, sy_b * 0.625, 0.960)),
            Vector((-0.875, sy_b * 0.660, 0.520)),
        ]
        _sweep_rect_tube(bm_sb, p_rear, half_y=0.0035, half_n=0.020)
    # Корпуса замков ремней между передними креслами и центральным тоннелем (slot 1 = Piano_Black)
    n_sb_pb0 = len(bm_sb.faces)
    for sy_b in (-1.0, +1.0):
        yb_k = sy_b * 0.145
        add_box(bm_sb, +0.02, +0.095, yb_k - 0.014, yb_k + 0.014, 0.455, 0.525)
        add_box(bm_sb, +0.04, +0.075, yb_k - 0.006, yb_k + 0.006, 0.360, 0.454)
        # Чёрный корпус накладки-регулятора на стойке B и держатель язычка
        y_top_b = sy_b * 0.718
        add_box(bm_sb, -0.098, -0.044, y_top_b - 0.012, y_top_b + 0.012, 1.118, 1.158)
        y_tng = sy_b * 0.728
        add_box(bm_sb, -0.094, -0.050, y_tng - 0.011, y_tng + 0.011, 0.725, 0.768)
    n_sb_pb1 = len(bm_sb.faces)
    # Хромированные проушины регуляторов и металлические язычки пряжек (slot 2 = Chrome)
    for sy_b in (-1.0, +1.0):
        y_top_b = sy_b * 0.706
        add_box(bm_sb, -0.092, -0.050, y_top_b - 0.006, y_top_b + 0.006, 1.124, 1.148)
        y_tng = sy_b * 0.715
        add_box(bm_sb, -0.088, -0.056, y_tng - 0.005, y_tng + 0.005, 0.692, 0.726)
    n_sb_chr1 = len(bm_sb.faces)
    # Красные кнопки PRESS на замках ремней безопасности (slot 3 = Caliper)
    for sy_b in (-1.0, +1.0):
        yb_k = sy_b * 0.145
        add_box(bm_sb, +0.032, +0.082, yb_k - 0.011, yb_k + 0.011, 0.525, 0.538)
    bm_sb.faces.ensure_lookup_table()
    for fi in range(n_sb_pb0, n_sb_pb1):
        bm_sb.faces[fi].material_index = 1
    for fi in range(n_sb_pb1, n_sb_chr1):
        bm_sb.faces[fi].material_index = 2
    for fi in range(n_sb_chr1, len(bm_sb.faces)):
        bm_sb.faces[fi].material_index = 3
    sb_ob = bmesh_to_object('Seatbelts', bm_sb, material=mats['rubber'],
                            smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
    sb_ob.data.materials.append(mats['piano_black'])
    sb_ob.data.materials.append(mats['chrome'])
    sb_ob.data.materials.append(mats['caliper'])
    uv_box(sb_ob, scale=6.0)
    objs.append(sb_ob)

    # 9. Многослойные внутренние дверные карты (кожа + шпон Linear Walnut + кнопки + хром-ручка)
    door_map = {ob.name: ob for ob in (door_objs or [])}
    for d_name, _hx, side, d_len, _deg in DOORS_SPEC:
        parent_door = door_map.get(d_name)
        if parent_door is None:
            continue
        is_rear = 'Rear' in d_name
        bm_dc = bmesh.new()
        # Глубокий отступ внутрь, чтобы карта двери не пробивала наружную обшивку у арки
        y_in_d = -side * (0.084 if is_rear else 0.072)
        x_back_c = -d_len + (0.145 if is_rear else 0.055)
        z_bot_c = 0.16 if is_rear else 0.11
        # Кожаная основа дверной карты + нижний карман + мягкий подлокотник (slot 0 = Leather)
        add_box(bm_dc, x_back_c, -0.048,
                y_in_d, y_in_d - side * 0.020, z_bot_c, 0.60)
        add_box(bm_dc, x_back_c + 0.05, -0.10,
                y_in_d - side * 0.022, y_in_d - side * 0.044, z_bot_c + 0.01, z_bot_c + 0.14)
        add_box(bm_dc, x_back_c + 0.08, -0.13,
                y_in_d - side * 0.022, y_in_d - side * 0.068, 0.34, 0.405)
        # Декоративная планка из скандинавского шпона Linear Walnut (slot 1 = Wood)
        f_wd0 = len(bm_dc.faces)
        add_box(bm_dc, x_back_c + 0.04, -0.08,
                y_in_d - side * 0.022, y_in_d - side * 0.032, 0.495, 0.548)
        f_wd1 = len(bm_dc.faces)
        # Глянцевый блок стеклоподъёмников + тёмная ниша внутренней ручки (slot 2)
        add_box(bm_dc, -0.34, -0.17,
                y_in_d - side * 0.032, y_in_d - side * 0.062, 0.406, 0.418)
        add_box(bm_dc, -0.28, -0.13,
                y_in_d - side * 0.021, y_in_d - side * 0.027, 0.478, 0.525)
        f_pb1 = len(bm_dc.faces)
        # Хром-кант под шпоном, ручка открывания и клавиши стеклоподъёмников (slot 3)
        add_box(bm_dc, x_back_c + 0.04, -0.08,
                y_in_d - side * 0.024, y_in_d - side * 0.034, 0.486, 0.494)
        add_box(bm_dc, -0.26, -0.15,
                y_in_d - side * 0.028, y_in_d - side * 0.045, 0.488, 0.514)
        for sw_x in (-0.30, -0.26, -0.22):
            add_box(bm_dc, sw_x - 0.010, sw_x + 0.010,
                    y_in_d - side * 0.040, y_in_d - side * 0.056, 0.418, 0.425)
        bm_dc.faces.ensure_lookup_table()
        for fi in range(f_wd0, f_wd1):
            bm_dc.faces[fi].material_index = 1
        for fi in range(f_wd1, f_pb1):
            bm_dc.faces[fi].material_index = 2
        for fi in range(f_pb1, len(bm_dc.faces)):
            bm_dc.faces[fi].material_index = 3
        card_name = d_name.replace('Door_', 'Door_Card_')
        dc_ob = bmesh_to_object(card_name, bm_dc, material=mats['leather'],
                                smooth=True, collection=collection, bevel=0.0035, bevel_segs=2)
        dc_ob.data.materials.append(mats['wood'])
        dc_ob.data.materials.append(mats['piano_black'])
        dc_ob.data.materials.append(mats['chrome'])
        uv_box(dc_ob, scale=2.5)
        dc_ob.parent = parent_door
        dc_ob.location = Vector((0.0, 0.0, 0.0))

    # Динамики в передних дверях (привязаны к Door_Front_L/R, открываются вместе с дверью)
    for label, side, door_name in (('L', 1.0, 'Door_Front_Left'),
                                   ('R', -1.0, 'Door_Front_Right')):
        bm_sp = bmesh.new()
        r0, r1, h = 0.048, 0.038, 0.016
        xa = -0.34
        z_loc = 0.22
        y_base = -side * 0.094
        ring_f = []
        ring_b = []
        for i in range(12):
            a = 2.0 * math.pi * i / 12
            ring_f.append(bm_sp.verts.new(Vector((xa + r0 * math.cos(a),
                                                  y_base,
                                                  z_loc + r0 * math.sin(a)))))
            ring_b.append(bm_sp.verts.new(Vector((xa + r1 * math.cos(a),
                                                  y_base - side * h,
                                                  z_loc + r1 * math.sin(a)))))
        for k in range(12):
            kn = (k + 1) % 12
            bm_sp.faces.new((ring_f[k], ring_f[kn], ring_b[kn], ring_b[k]))
        cf = bm_sp.verts.new(Vector((xa, y_base, z_loc)))
        cb = bm_sp.verts.new(Vector((xa, y_base - side * h, z_loc)))
        for k in range(12):
            kn = (k + 1) % 12
            bm_sp.faces.new((cf, ring_f[kn], ring_f[k]))
            bm_sp.faces.new((cb, ring_b[k], ring_b[kn]))
        sp = bmesh_to_object(f'Speaker_{label}', bm_sp, material=mats['alloy'],
                             smooth=True, collection=collection)
        uv_box(sp, scale=8.0)
        parent_door = door_map.get(door_name)
        if parent_door is not None:
            sp.parent = parent_door
            sp.location = Vector((0.0, 0.0, 0.0))
        else:
            objs.append(sp)

    return objs


def build_doors(body_ob, mats, collection):
    """Четыре профилированные двери с верхними оконными рамками, дверными стёклами
    (Door_Glass_*) и петлями (Door_Hinge_*). Все 4 двери открываются наружу на 70°."""
    door_objs = []
    door_thickness = 0.050
    hinge_z = CABIN_SILL_Z + 0.004  # аккуратный зазор 4 мм над порогом кузова

    for name, hx, side, door_len, _open_deg in DOORS_SPEC:
        is_front = 'Front' in name
        hinge_y = side * side_surface_y(hx, 0.60)
        origin = Vector((hx, hinge_y, hinge_z))

        # 1. Контурное нижнее полотно двери по ЕДИНОЙ кривой side_surface_y(x, z)
        # (гарантирует идеально гладкий стык заподлицо с крыльями без ступеньки)
        bm = bmesh.new()
        nx, nz = 12, 10
        x_f = hx - 0.002
        x_b = hx - door_len + 0.002
        xs = [x_f + (x_b - x_f) * i / (nx - 1) for i in range(nx)]

        outer_pts = []
        inner_pts = []
        for x in xs:
            z_top = BELT(x)
            row_out = []
            row_in = []
            for j in range(nz):
                v = j / (nz - 1)
                z_w = hinge_z + (z_top - hinge_z) * v
                y_out_w = side * side_surface_y(x, z_w)
                y_in_w = y_out_w - side * door_thickness
                row_out.append(Vector((x - hx, y_out_w - hinge_y, z_w - hinge_z)))
                row_in.append(Vector((x - hx, y_in_w - hinge_y, z_w - hinge_z)))
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm, outer_pts, inner_pts)

        # 2. Верхняя оконная рамка двери (строго параллельна стойкам A/B/C и крыше заподлицо!)
        if is_front:
            # Передняя балка рамки идёт строго с зазором ~4 мм позади задней кромки Pillar_A
            xf_bot, xf_top = WINDSHIELD_X_BOT - 0.086, ROOF_X_FRONT - 0.038
            xb_bot = PILLAR_B_X + PILLAR_B_HALF_X + 0.012
            xb_top = PILLAR_B_X + PILLAR_B_HALF_X + 0.008
        else:
            xf_bot = PILLAR_B_X - PILLAR_B_HALF_X - 0.012
            xf_top = PILLAR_B_X - PILLAR_B_HALF_X - 0.008
            xb_bot, xb_top = x_b - 0.006, ROOF_X_REAR + 0.062

        def _sash_pt(x_val, t_val, y_off=0.0):
            y_abs, z_val = upper_side_yz(x_val, t_val)
            return Vector((x_val - hx, side * (y_abs + y_off) - hinge_y, z_val - hinge_z))

        # Замкнутая дуга оконной рамки заподлицо с Pillar_A/B/C (наружная грань = y_abs + 0.004)
        frame_w = 0.024
        frame_d = 0.024

        def _add_sash_bar(pts_3d, b_pillar_seg_range=None):
            rings = []
            n_p = len(pts_3d)
            for idx, p in enumerate(pts_3d):
                if idx == 0:
                    tang = (pts_3d[1] - pts_3d[0]).normalized()
                elif idx == n_p - 1:
                    tang = (pts_3d[-1] - pts_3d[-2]).normalized()
                else:
                    tang = (pts_3d[idx + 1] - pts_3d[idx - 1]).normalized()
                y_dir = Vector((0.0, side, 0.0))
                in_plane = tang.cross(y_dir)
                if in_plane.length < 1e-6:
                    in_plane = Vector((0.0, 0.0, 1.0))
                in_plane.normalize()
                hw = frame_w * 0.5
                hd = frame_d * 0.5
                ring = [
                    bm.verts.new(p - in_plane * hw - y_dir * hd),
                    bm.verts.new(p + in_plane * hw - y_dir * hd),
                    bm.verts.new(p + in_plane * hw + y_dir * hd),
                    bm.verts.new(p - in_plane * hw + y_dir * hd),
                ]
                rings.append(ring)
            for i in range(n_p - 1):
                r0, r1 = rings[i], rings[i + 1]
                mat_i = (
                    1 if (b_pillar_seg_range is not None
                          and b_pillar_seg_range[0] <= i < b_pillar_seg_range[1])
                    else 0
                )
                for k in range(4):
                    kn = (k + 1) % 4
                    f_s = bm.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
                    f_s.material_index = mat_i
            f0 = bm.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
            f1 = bm.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))
            if b_pillar_seg_range is not None:
                if b_pillar_seg_range[0] == 0:
                    f0.material_index = 1
                if b_pillar_seg_range[1] == n_p - 1:
                    f1.material_index = 1

        # Непрерывная дуга оконной рамки: передняя стойка -> верхняя планка -> задняя стойка
        # Стойка рамки вдоль Pillar_B окрашена в глянцево-чёрный (слот 1 = Piano_Black),
        # убирая синие вертикальные полоски по бокам центральной стойки B!
        arch_pts = []
        for k in range(6):
            t = 0.015 + 0.965 * (k / 5.0)
            arch_pts.append(_sash_pt(xf_bot * (1.0 - t) + xf_top * t, t, y_off=-0.014))
        for k in range(1, 5):
            u = k / 4.0
            arch_pts.append(_sash_pt(xf_top * (1.0 - u) + xb_top * u, 0.98, y_off=-0.014))
        for k in range(1, 6):
            t = 0.98 - 0.965 * (k / 5.0)
            arch_pts.append(_sash_pt(xb_bot * (1.0 - t) + xb_top * t, t, y_off=-0.014))
        # Сегменты 0..5 — передняя стойка рамки, 6..8 — верх, 9..14 — задняя стойка рамки
        b_seg = (9, 15) if is_front else (0, 6)
        _add_sash_bar(arch_pts, b_pillar_seg_range=b_seg)

        # 2b. Нижний поясной хромированный молдинг двери (slot 2 = Chrome, открывается с дверью!)
        belt_strip_o = []
        belt_strip_i = []
        n_bs = 10
        n_before_chrome = len(bm.faces)
        for ii in range(n_bs):
            u_b = ii / float(n_bs - 1)
            xb_s = (x_f - 0.004) * (1.0 - u_b) + (x_b + 0.004) * u_b
            zb_s = BELT(xb_s)
            yb_s = side * (side_surface_y(xb_s, zb_s) + 0.0055) - hinge_y
            yb_in = side * (side_surface_y(xb_s, zb_s) - 0.004) - hinge_y
            belt_strip_o.append([
                Vector((xb_s - hx, yb_s, zb_s - 0.005 - hinge_z)),
                Vector((xb_s - hx, yb_s, zb_s + 0.008 - hinge_z)),
            ])
            belt_strip_i.append([
                Vector((xb_s - hx, yb_in, zb_s - 0.005 - hinge_z)),
                Vector((xb_s - hx, yb_in, zb_s + 0.008 - hinge_z)),
            ])
        _add_thick_grid(bm, belt_strip_o, belt_strip_i)
        bm.faces.ensure_lookup_table()
        for f_idx in range(n_before_chrome, len(bm.faces)):
            bm.faces[f_idx].material_index = 2

        door_ob = bmesh_to_object(name, bm, material=mats['paint'],
                                  smooth=True, collection=collection, auto_smooth_deg=55.0,
                                  bevel=0.0028, bevel_segs=2, subsurf=1)
        door_ob.data.materials.append(mats['piano_black'])
        door_ob.data.materials.append(mats['chrome'])
        uv_box(door_ob, scale=1.5)
        door_ob.location = origin
        door_ob.parent = body_ob
        door_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        door_objs.append(door_ob)

        # 3. Боковое стекло двери (Door_Glass_*), заходит в паз оконной рамки со всех 4 сторон
        bm_gl = bmesh.new()
        g_out = []
        g_in = []
        nu_g, nv_g = 7, 7
        g_thick = 0.005
        for j in range(nv_g):
            t = 0.01 + 0.965 * (j / (nv_g - 1))
            xf_t = xf_bot * (1.0 - t) + xf_top * t - 0.004
            xb_t = xb_bot * (1.0 - t) + xb_top * t + 0.004
            row_o = []
            row_i = []
            for i in range(nu_g):
                u = i / (nu_g - 1)
                x_g = xf_t * (1.0 - u) + xb_t * u
                y_abs, z_g = upper_side_yz(x_g, t)
                y_o = side * (y_abs - 0.012) - hinge_y
                y_i = side * (y_abs - 0.012 - g_thick) - hinge_y
                row_o.append(Vector((x_g - hx, y_o, z_g - hinge_z)))
                row_i.append(Vector((x_g - hx, y_i, z_g - hinge_z)))
            g_out.append(row_o)
            g_in.append(row_i)
        _add_thick_grid(bm_gl, g_out, g_in)
        glass_name = name.replace('Door_', 'Door_Glass_')
        dg_ob = bmesh_to_object(glass_name, bm_gl, material=mats['glass'],
                                smooth=True, collection=collection, auto_smooth_deg=60.0,
                                subsurf=1)
        uv_box(dg_ob, scale=1.5)
        dg_ob.parent = door_ob
        dg_ob.location = Vector((0.0, 0.0, 0.0))

        # 4. Две дверные петли внутри проёма на стойке A / B (скрыты при закрытой двери!)
        bm_dh = bmesh.new()
        for hz_pin in (0.42, 0.78):
            _add_z_cylinder(bm_dh, 0.009, hz_pin - 0.026, hz_pin + 0.026,
                            seg=12, center_xy=(hx - 0.004, hinge_y - side * 0.056))
        hinge_name = name.replace('Door_', 'Door_Hinge_')
        dh_ob = bmesh_to_object(hinge_name, bm_dh, material=mats['carbon'],
                                smooth=True, collection=collection)
        uv_box(dh_ob, scale=8.0)
        dh_ob.parent = body_ob
        dh_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()

        # 5. Наружная дверная ручка Volvo S90 (корпус в цвет кузова + верхняя хром-планка)
        # 5. Полностью отделённая дверная ручка-скоба Volvo S90 Inscription:
        #    (a) Door_Handle_Base_* — неподвижная теневая чашка под пальцы + задняя крышка
        #        личинки замка (key-cap) с технологическим швом 4 мм;
        #    (b) Door_Handle_* — самостоятельная подвижная рукоятка (origin в переднем шарнире!)
        #        с изогнутым 7-секционным loft-профилем, реальным просветом 22 мм для пальцев
        #        и контурной хромированной накладкой сверху!
        x_hdl_w = hx - door_len + 0.20
        z_hdl_w = BELT(x_hdl_w) - 0.112
        y_surf_w = side * side_surface_y(x_hdl_w, z_hdl_w)
        xh_l = x_hdl_w - hx
        yh_l = y_surf_w - hinge_y
        zh_l = z_hdl_w - hinge_z

        # 5a. Неподвижная база ручки (чашка под пальцы + прокладки + задняя крышка личинки)
        bm_hb = bmesh.new()
        # Задняя крышка личинки замка (key-cap, slot 0 = Paint), отделена швом 4 мм от ручки
        add_box(bm_hb, -0.098, -0.058,
                side * 0.002, side * 0.025, -0.014, +0.011)
        n_hb_paint = len(bm_hb.faces)
        # Верхний хромированный штрих на крышке личинки (slot 2 = Chrome)
        add_box(bm_hb, -0.096, -0.060,
                side * 0.016, side * 0.027, +0.0115, +0.0165)
        n_hb_chr = len(bm_hb.faces)
        # Утопленная тёмная чашка-подложка под пальцы + резиновые прокладки (slot 1 = Piano_Black)
        add_box(bm_hb, -0.052, +0.064,
                -side * 0.001, side * 0.0045, -0.022, +0.018)
        # Резиновая прокладка под крышкой личинки и под передним шарниром
        add_box(bm_hb, -0.101, -0.055,
                -side * 0.001, side * 0.004, -0.016, +0.017)
        add_box(bm_hb, +0.064, +0.096,
                -side * 0.001, side * 0.004, -0.016, +0.017)
        bm_hb.faces.ensure_lookup_table()
        for f_idx in range(n_hb_paint, n_hb_chr):
            bm_hb.faces[f_idx].material_index = 2
        for f_idx in range(n_hb_chr, len(bm_hb.faces)):
            bm_hb.faces[f_idx].material_index = 1
        base_name = name.replace('Door_', 'Door_Handle_Base_')
        hb_ob = bmesh_to_object(base_name, bm_hb, material=mats['paint'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.0018, bevel_segs=2)
        hb_ob.data.materials.append(mats['piano_black'])
        hb_ob.data.materials.append(mats['chrome'])
        uv_box(hb_ob, scale=10.0)
        hb_ob.parent = door_ob
        hb_ob.location = Vector((xh_l, yh_l, zh_l))

        # 5b. Подвижная скульптурная рукоятка (origin строго в переднем шарнире: x = xh_l + 0.080!)
        # Можно независимо поворачивать вокруг Z на ±12°, полностью отделена от двери!
        piv_dx = 0.080
        bm_hdl = bmesh.new()
        # 7 сечений вдоль X относительно переднего шарнира (dx = 0 -> dx = -0.134)
        # (dx, y_inner, y_outer, z_bot, z_top) — реальный сквозной просвет 21 мм по центру!
        h_specs = [
            (+0.012, +0.002, +0.023, -0.012, +0.011),
            (-0.004, +0.015, +0.032, -0.013, +0.012),
            (-0.030, +0.022, +0.038, -0.014, +0.012),
            (-0.066, +0.024, +0.040, -0.014, +0.012),
            (-0.102, +0.022, +0.038, -0.014, +0.012),
            (-0.124, +0.014, +0.031, -0.013, +0.012),
            (-0.134, +0.002, +0.024, -0.012, +0.011),
        ]
        h_rings = []
        for dx, yi, yo, zb, zt in h_specs:
            zm = 0.5 * (zb + zt)
            if side > 0:
                ring = [
                    bm_hdl.verts.new(Vector((dx, yi, zb))),
                    bm_hdl.verts.new(Vector((dx, yo, zb))),
                    bm_hdl.verts.new(Vector((dx, yo + 0.002, zm))),
                    bm_hdl.verts.new(Vector((dx, yo, zt))),
                    bm_hdl.verts.new(Vector((dx, yi, zt))),
                ]
            else:
                ring = [
                    bm_hdl.verts.new(Vector((dx, -yi, zb))),
                    bm_hdl.verts.new(Vector((dx, -yi, zt))),
                    bm_hdl.verts.new(Vector((dx, -yo, zt))),
                    bm_hdl.verts.new(Vector((dx, -(yo + 0.002), zm))),
                    bm_hdl.verts.new(Vector((dx, -yo, zb))),
                ]
            h_rings.append(ring)
        for i_h in range(len(h_rings) - 1):
            r0, r1 = h_rings[i_h], h_rings[i_h + 1]
            for k in range(5):
                kn = (k + 1) % 5
                bm_hdl.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        # Торцы квадами + треугольником (0 n-gons)
        for end_r, rev in ((h_rings[0], True), (h_rings[-1], False)):
            if rev:
                bm_hdl.faces.new((end_r[0], end_r[4], end_r[3], end_r[1]))
                bm_hdl.faces.new((end_r[1], end_r[3], end_r[2]))
            else:
                bm_hdl.faces.new((end_r[0], end_r[1], end_r[3], end_r[4]))
                bm_hdl.faces.new((end_r[1], end_r[2], end_r[3]))
        # Передняя и задняя опорные лапки (входят в прокладки двери)
        add_box(bm_hdl, -0.006, +0.010,
                -side * 0.002, side * 0.014, -0.010, +0.009)
        add_box(bm_hdl, -0.133, -0.118,
                -side * 0.002, side * 0.014, -0.010, +0.009)

        # Изогнутая верхняя хромированная полоса S90 Inscription (slot 1 = Chrome)
        n_paint_faces = len(bm_hdl.faces)
        c_rings = []
        for dx, yi, yo, _zb, zt in h_specs:
            y_ci = side * (yi + 0.004)
            y_co = side * (yo + 0.002)
            y0_c, y1_c = (y_ci, y_co) if y_ci < y_co else (y_co, y_ci)
            c_rings.append([
                bm_hdl.verts.new(Vector((dx, y0_c, zt + 0.0006))),
                bm_hdl.verts.new(Vector((dx, y1_c, zt + 0.0006))),
                bm_hdl.verts.new(Vector((dx, y1_c, zt + 0.0052))),
                bm_hdl.verts.new(Vector((dx, y0_c, zt + 0.0052))),
            ])
        for i_c in range(len(c_rings) - 1):
            r0, r1 = c_rings[i_c], c_rings[i_c + 1]
            for k in range(4):
                kn = (k + 1) % 4
                bm_hdl.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        bm_hdl.faces.new((c_rings[0][0], c_rings[0][3], c_rings[0][2], c_rings[0][1]))
        bm_hdl.faces.new((c_rings[-1][0], c_rings[-1][1], c_rings[-1][2], c_rings[-1][3]))

        bm_hdl.faces.ensure_lookup_table()
        for f_idx in range(n_paint_faces, len(bm_hdl.faces)):
            bm_hdl.faces[f_idx].material_index = 1
        handle_name = name.replace('Door_', 'Door_Handle_')
        hdl_ob = bmesh_to_object(handle_name, bm_hdl, material=mats['paint'],
                                 smooth=True, collection=collection, auto_smooth_deg=55.0,
                                 bevel=0.0018, bevel_segs=2)
        hdl_ob.data.materials.append(mats['chrome'])
        uv_box(hdl_ob, scale=10.0)
        hdl_ob.parent = door_ob
        hdl_ob.location = Vector((xh_l + piv_dx, yh_l, zh_l))

        # 6. Обтекаемое двухцветное зеркало заднего вида Volvo S90 (на передней двери):
        # слот 0 = Paint (верхняя крышка), слот 1 = Piano_Black (ножка + нижняя чаша + безель),
        # слот 2 = Chrome (зеркальное полотно сзади), слот 3 = Amber_LED (повторитель поворотника)
        if is_front:
            m_tag = 'L' if side > 0 else 'R'
            xm_w = hx - 0.16
            zm_w = BELT(xm_w) - 0.015
            ym_w = side * side_surface_y(xm_w, zm_w)
            xm_l = xm_w - hx
            ym_l = ym_w - hinge_y
            zm_l = zm_w - hinge_z
            bm_mir = bmesh.new()

            # 6a. Чёрная глянцевая аэродинамическая ножка-флажок на двери (slot 1 = Piano_Black)
            stalk_pts = [
                Vector((xm_l, ym_l - side * 0.006, zm_l)),
                Vector((xm_l - 0.016, ym_l + side * 0.036, zm_l + 0.026)),
                Vector((xm_l - 0.030, ym_l + side * 0.062, zm_l + 0.050)),
            ]
            _sweep_rect_tube(bm_mir, stalk_pts, half_y=0.013, half_n=0.012)
            # Скульптурный подиум-основание ножки на двери
            add_box(bm_mir, xm_l - 0.036, xm_l + 0.028,
                    ym_l - side * 0.004, ym_l + side * 0.011, zm_l - 0.013, zm_l + 0.015)
            bm_mir.faces.ensure_lookup_table()
            for f_st in bm_mir.faces:
                f_st.material_index = 1

            # 6b. Скульптурный каплевидный корпус (верх = Paint [0], низ = Piano_Black [1])
            m_rings = []
            m_specs = [
                (0.048, -0.044, +0.050, +0.028, +0.092),
                (0.075, -0.056, +0.055, +0.023, +0.103),
                (0.108, -0.059, +0.052, +0.022, +0.104),
                (0.144, -0.056, +0.038, +0.026, +0.098),
                (0.178, -0.045, +0.014, +0.035, +0.087),
                (0.198, -0.030, -0.006, +0.045, +0.073),
            ]
            for dy_off, dx0, dx1, dz0, dz1 in m_specs:
                yy = ym_l + side * dy_off
                dz_mid = 0.5 * (dz0 + dz1) - 0.006
                if side > 0:
                    ring = [
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx1 + 0.007, yy, zm_l + dz_mid))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx0 - 0.003, yy, zm_l + dz_mid))),
                    ]
                else:
                    ring = [
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx0 - 0.003, yy, zm_l + dz_mid))),
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx1 + 0.007, yy, zm_l + dz_mid))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz0))),
                    ]
                m_rings.append(ring)
            n_r = len(m_rings[0])
            for i_m in range(len(m_rings) - 1):
                r0, r1 = m_rings[i_m], m_rings[i_m + 1]
                for k in range(n_r):
                    kn = (k + 1) % n_r
                    f_m = bm_mir.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
                    z_fc = f_m.calc_center_median().z
                    f_m.material_index = 1 if z_fc < (zm_l + 0.056) else 0
            for end_ring, rev in ((m_rings[0], True), (m_rings[-1], False)):
                if rev:
                    f_a = bm_mir.faces.new((end_ring[0], end_ring[5], end_ring[2], end_ring[1]))
                    f_b = bm_mir.faces.new((end_ring[5], end_ring[4], end_ring[3], end_ring[2]))
                else:
                    f_a = bm_mir.faces.new((end_ring[0], end_ring[1], end_ring[2], end_ring[5]))
                    f_b = bm_mir.faces.new((end_ring[5], end_ring[2], end_ring[3], end_ring[4]))
                f_a.material_index = 1
                f_b.material_index = 0

            # 6c. Внутренний чёрный ободок + зеркальное полотно (slot 1 -> slot 2)
            n_bef_bez = len(bm_mir.faces)
            add_box(bm_mir, xm_l - 0.061, xm_l - 0.054,
                    ym_l + side * 0.058, ym_l + side * 0.180, zm_l + 0.029, zm_l + 0.095)
            n_before_glass = len(bm_mir.faces)
            add_box(bm_mir, xm_l - 0.064, xm_l - 0.059,
                    ym_l + side * 0.064, ym_l + side * 0.174, zm_l + 0.034, zm_l + 0.090)
            bm_mir.faces.ensure_lookup_table()
            for f_idx in range(n_bef_bez, n_before_glass):
                bm_mir.faces[f_idx].material_index = 1
            for f_idx in range(n_before_glass, len(bm_mir.faces)):
                bm_mir.faces[f_idx].material_index = 2

            # 6d. Тонкая янтарная LED-полоска повторителя поворотника (slot 3 = Amber_LED)
            n_before_led = len(bm_mir.faces)
            add_box(bm_mir, xm_l + 0.044, xm_l + 0.060,
                    ym_l + side * 0.088, ym_l + side * 0.180, zm_l + 0.052, zm_l + 0.060)
            bm_mir.faces.ensure_lookup_table()
            for f_idx in range(n_before_led, len(bm_mir.faces)):
                bm_mir.faces[f_idx].material_index = 3

            mir_ob = bmesh_to_object(f'Mirror_{m_tag}', bm_mir, material=mats['paint'],
                                     smooth=True, collection=collection, auto_smooth_deg=55.0,
                                     bevel=0.0018, bevel_segs=2, subsurf=1)
            mir_ob.data.materials.append(mats['piano_black'])
            mir_ob.data.materials.append(mats['chrome'])
            mir_ob.data.materials.append(mats['amber_led'])
            uv_box(mir_ob, scale=6.0)
            mir_ob.parent = door_ob
            mir_ob.location = Vector((0.0, 0.0, 0.0))

    return door_objs


def _sweep_rect_tube(bm, points, half_y, half_n):
    """Замкнутая прямоугольная балка вдоль 3D-ломаной points (безопасна для любых осей)."""
    rings = []
    n_pts = len(points)
    for idx, p in enumerate(points):
        if idx == 0:
            tangent = (points[1] - points[0]).normalized()
        elif idx == n_pts - 1:
            tangent = (points[-1] - points[-2]).normalized()
        else:
            t_prev = (points[idx] - points[idx - 1]).normalized()
            t_next = (points[idx + 1] - points[idx]).normalized()
            tangent = (t_prev + t_next)
            tangent = tangent.normalized() if tangent.length > 1e-6 else t_next
        ref = Vector((0.0, 0.0, 1.0)) if abs(tangent.y) > 0.85 else Vector((0.0, 1.0, 0.0))
        normal_xz = tangent.cross(ref).normalized()
        side_v = normal_xz.cross(tangent).normalized()
        ring = [
            bm.verts.new(p - side_v * half_y - normal_xz * half_n),
            bm.verts.new(p + side_v * half_y - normal_xz * half_n),
            bm.verts.new(p + side_v * half_y + normal_xz * half_n),
            bm.verts.new(p - side_v * half_y + normal_xz * half_n),
        ]
        rings.append(ring)

    for i in range(n_pts - 1):
        r0, r1 = rings[i], rings[i + 1]
        for k in range(4):
            kn = (k + 1) % 4
            bm.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    # Торцевые заглушки (квады -> 0 n-gon, 0 non-manifold)
    bm.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
    bm.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))


def _sweep_round_tube(bm, points, radius, seg=12, mat_idx=0):
    """Замкнутый круглый шланг/трубка вдоль 3D-ломаной (100% manifold, без n-gon)."""
    rings = []
    n_pts = len(points)
    prev_n = None
    for idx, p in enumerate(points):
        if idx == 0:
            tangent = (points[1] - points[0]).normalized()
        elif idx == n_pts - 1:
            tangent = (points[-1] - points[-2]).normalized()
        else:
            t_prev = (points[idx] - points[idx - 1]).normalized()
            t_next = (points[idx + 1] - points[idx]).normalized()
            tangent = t_prev + t_next
            tangent = tangent.normalized() if tangent.length > 1e-6 else t_next
        if prev_n is None:
            ref = Vector((0.0, 0.0, 1.0)) if abs(tangent.z) < 0.85 else Vector((1.0, 0.0, 0.0))
            n_vec = tangent.cross(ref).normalized()
        else:
            n_vec = prev_n - tangent * prev_n.dot(tangent)
            if n_vec.length < 1e-6:
                ref = Vector((0.0, 1.0, 0.0)) if abs(tangent.y) < 0.85 else Vector((1.0, 0.0, 0.0))
                n_vec = tangent.cross(ref).normalized()
            else:
                n_vec.normalize()
        prev_n = n_vec
        b_vec = tangent.cross(n_vec).normalized()
        ring = []
        for k in range(seg):
            ang = 2.0 * math.pi * k / seg
            off = (n_vec * math.cos(ang) + b_vec * math.sin(ang)) * radius
            ring.append(bm.verts.new(p + off))
        rings.append(ring)

    new_faces = []
    for i in range(n_pts - 1):
        r0, r1 = rings[i], rings[i + 1]
        for k in range(seg):
            kn = (k + 1) % seg
            new_faces.append(bm.faces.new((r0[k], r0[kn], r1[kn], r1[k])))
    c0 = bm.verts.new(points[0])
    c1 = bm.verts.new(points[-1])
    for k in range(seg):
        kn = (k + 1) % seg
        new_faces.append(bm.faces.new((c0, rings[0][kn], rings[0][k])))
        new_faces.append(bm.faces.new((c1, rings[-1][k], rings[-1][kn])))
    for f in new_faces:
        f.material_index = mat_idx


def _add_z_cylinder(bm, radius, z0, z1, seg=18, center_xy=(0.0, 0.0)):
    """Добавляет замкнутый цилиндр вдоль оси Z от z0 до z1."""
    cx, cy = center_xy
    rings = []
    for zz in (z0, z1):
        ring = []
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            ring.append(bm.verts.new(Vector((cx + radius * math.cos(a),
                                             cy + radius * math.sin(a),
                                             zz))))
        rings.append(ring)
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((rings[0][k], rings[0][kn], rings[1][kn], rings[1][k]))
    c0 = bm.verts.new(Vector((cx, cy, z0)))
    c1 = bm.verts.new(Vector((cx, cy, z1)))
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((c0, rings[0][kn], rings[0][k]))
        bm.faces.new((c1, rings[1][k], rings[1][kn]))


def _add_y_cylinder(bm, radius, y0, y1, seg=16, center_xz=(0.0, 0.0)):
    """Добавляет замкнутый цилиндр вдоль оси Y от y0 до y1 (ось шарнира)."""
    cx, cz = center_xz
    rings = []
    for yy in (y0, y1):
        ring = []
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            ring.append(bm.verts.new(Vector((cx + radius * math.cos(a),
                                             yy,
                                             cz + radius * math.sin(a)))))
        rings.append(ring)
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((rings[0][k], rings[0][kn], rings[1][kn], rings[1][k]))
    c0 = bm.verts.new(Vector((cx, y0, cz)))
    c1 = bm.verts.new(Vector((cx, y1, cz)))
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((c0, rings[0][kn], rings[0][k]))
        bm.faces.new((c1, rings[1][k], rings[1][kn]))


def _add_x_cylinder(bm, radius, x0, x1, seg=16, center_yz=(0.0, 0.0)):
    """Добавляет замкнутый цилиндр вдоль оси X от x0 до x1."""
    xa, xb = (x0, x1) if x0 <= x1 else (x1, x0)
    cy, cz = center_yz
    rings = []
    for xx in (xa, xb):
        ring = []
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            ring.append(bm.verts.new(Vector((xx,
                                             cy + radius * math.cos(a),
                                             cz + radius * math.sin(a)))))
        rings.append(ring)
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((rings[0][k], rings[0][kn], rings[1][kn], rings[1][k]))
    c0 = bm.verts.new(Vector((xa, cy, cz)))
    c1 = bm.verts.new(Vector((xb, cy, cz)))
    for k in range(seg):
        kn = (k + 1) % seg
        bm.faces.new((c0, rings[0][kn], rings[0][k]))
        bm.faces.new((c1, rings[1][k], rings[1][kn]))


def build_trunk(body_ob, mats, collection):
    """Строит отделённую от кузова крышку багажника, коврик ниши, две петли
    («гусиная шея») и два телескопических газовых лифта (цилиндр + шток).

    Все детали замкнуты (manifold), имеют UV и ацикличную схему TRACK_TO,
    поэтому корректно работают при прокрутке кадров и запекаются в FBX."""
    objs = []
    hz = trunk_hinge_z()
    hinge_origin = Vector((TRUNK_HINGE_X, 0.0, hz))

    # 1. Напольный ковёр внутри ворсовой ванны багажника (поверх Trunk_Tub)
    trunk_floor = _make_box_object(
        'Trunk_Floor',
        (-2.22, -1.26, -0.45, 0.45, 0.484, 0.506),
        mats['carpet'], collection, uv_scale=2.5,
    )
    trunk_floor.parent = body_ob
    trunk_floor.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(trunk_floor)

    # 2. Отделённая крышка багажника (origin в оси петли — вращается вокруг локальной Y)
    # Верхняя крышка между гребнями задних крыльев (зазор 6 мм),
    # задняя вертикальная часть строго между задними фонарями (|y| <= TRUNK_TAIL_HALF_Y)
    # и до самой верхней кромки заднего бампера (REAR_BUMPER_TOP_Z + 0.008)!
    bm_lid = bmesh.new()
    nx = 18
    ny = 15
    x_start = TRUNK_X_FRONT - 0.004   # плотный зазор ~4 мм у заднего стекла
    x_end = REAR_FENDER_X_REAR + 0.010

    outer = []
    inner = []
    for i in range(nx):
        u_x = i / (nx - 1)
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            # Верхняя крышка плавно сужается у заднего ребра к проёму между фонарями
            x_nom = x_start + (x_end - x_start) * u_x
            hw_top = WR(x_nom) + 0.032
            hw_back = TRUNK_TAIL_HALF_Y
            t_taper = clamp((u_x - 0.72) / 0.28, 0.0, 1.0)
            t_taper = t_taper * t_taper * (3.0 - 2.0 * t_taper)
            hw = hw_top * (1.0 - t_taper) + hw_back * t_taper
            y = v * hw
            # У заднего ребра X следует плавной дуге кормы rear_tail_surface_x(y)
            x_back_edge = rear_tail_surface_x(y)
            x = x_start * (1.0 - u_x) + x_back_edge * u_x
            edge_drop = 0.008 * (abs(v) ** 2.6) * (1.0 - 0.35 * t_taper)
            # Выразительный купеобразный «утиный хвостик» (ducktail lip), сходящий в 0 на швах
            lip_kick = 0.011 * (max(0.0, (u_x - 0.72) / 0.28) ** 1.55) * (1.0 - v * v)
            z_top = trunk_deck_z(x, y) - edge_drop + lip_kick
            p_out = Vector((x - TRUNK_HINGE_X, y, z_top - hz))
            p_in = Vector((x - TRUNK_HINGE_X, y, z_top - TRUNK_LID_THICKNESS - hz))
            row_out.append(bm_lid.verts.new(p_out))
            row_in.append(bm_lid.verts.new(p_in))
        outer.append(row_out)
        inner.append(row_in)

    # Полновысотная вертикальная задняя панель крышки багажника (между фонарями до бампера!)
    # с фирменной трапециевидной утопленной нишей под номерной знак Volvo S90!
    z_bot_lid = REAR_BUMPER_TOP_Z + 0.008  # зазор 8 мм над верхней полкой заднего бампера
    n_drop = 9
    lip_top_z = []
    lip_x_by_j = []
    for j in range(ny):
        v = -1.0 + 2.0 * j / (ny - 1)
        y = v * TRUNK_TAIL_HALF_Y
        xb = rear_tail_surface_x(y)
        lip_x_by_j.append(xb)
        lip_top_z.append(
            trunk_deck_z(xb, y) - 0.009 * (abs(v) ** 3.0) + 0.011 * (1.0 - v * v)
        )

    for step in range(1, n_drop + 1):
        frac = step / float(n_drop)
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * TRUNK_TAIL_HALF_Y
            z_cur = lip_top_z[j] * (1.0 - frac) + z_bot_lid * frac
            # Плавная трапециевидная штамповка под номерной знак Volvo S90 (глубина +15 мм)
            ay = abs(y)
            y_niche = 0.280 - 0.028 * frac
            w_y = 1.0 - clamp((ay - (y_niche - 0.045)) / 0.055, 0.0, 1.0)
            w_y = w_y * w_y * (3.0 - 2.0 * w_y)
            w_z_hi = clamp((frac - 0.24) / 0.16, 0.0, 1.0)
            w_z_hi = w_z_hi * w_z_hi * (3.0 - 2.0 * w_z_hi)
            w_z_lo = 1.0 - clamp((frac - 0.78) / 0.16, 0.0, 1.0)
            w_z_lo = w_z_lo * w_z_lo * (3.0 - 2.0 * w_z_lo)
            niche_recess = 0.015 * w_y * w_z_hi * w_z_lo
            # Мягкий перекат верхнего спойлера (ducktail) и лёгкое боковое скругление к фонарям
            side_roll = 0.005 * (abs(v) ** 3.2) * math.sin(math.pi * frac)
            x_cur = (
                lip_x_by_j[j]
                - 0.008 * math.sin(math.pi * frac)
                + 0.009 * frac
                + side_roll
                + niche_recess
            )
            row_out.append(bm_lid.verts.new(Vector((x_cur - TRUNK_HINGE_X, y, z_cur - hz))))
            row_in.append(bm_lid.verts.new(Vector((
                x_cur + TRUNK_LID_THICKNESS - TRUNK_HINGE_X, y, z_cur - hz))))
        outer.append(row_out)
        inner.append(row_in)

    total_rows = len(outer)
    for i in range(total_rows - 1):
        for j in range(ny - 1):
            bm_lid.faces.new((outer[i][j], outer[i][j + 1],
                              outer[i + 1][j + 1], outer[i + 1][j]))
            bm_lid.faces.new((inner[i][j], inner[i + 1][j],
                              inner[i + 1][j + 1], inner[i][j + 1]))

    # Замыкающие борта по всему периметру крышки -> 100% manifold
    for i in range(total_rows - 1):
        bm_lid.faces.new((outer[i][0], outer[i + 1][0], inner[i + 1][0], inner[i][0]))
        bm_lid.faces.new((outer[i][ny - 1], inner[i][ny - 1],
                          inner[i + 1][ny - 1], outer[i + 1][ny - 1]))
    for j in range(ny - 1):
        bm_lid.faces.new((outer[0][j], inner[0][j], inner[0][j + 1], outer[0][j + 1]))
        bm_lid.faces.new((outer[total_rows - 1][j], outer[total_rows - 1][j + 1],
                          inner[total_rows - 1][j + 1], inner[total_rows - 1][j]))

    # Внутренние рёбра жёсткости и скоба замка на изнанке крышки багажника
    z_rib = trunk_deck_z(-2.05, 0.0) - TRUNK_LID_THICKNESS - 0.016 - hz
    add_box(bm_lid, -0.52, -0.10, -0.36, -0.32, z_rib - 0.012, z_rib)
    add_box(bm_lid, -0.52, -0.10, +0.32, +0.36, z_rib - 0.012, z_rib)
    add_box(bm_lid, -0.54, -0.50, -0.31, +0.31, z_rib - 0.012, z_rib)
    # Скоба замка на нижней кромке вертикальной панели крышки
    add_box(bm_lid,
            rear_tail_surface_x(0.0) + TRUNK_LID_THICKNESS + 0.008 - TRUNK_HINGE_X,
            rear_tail_surface_x(0.0) + TRUNK_LID_THICKNESS + 0.036 - TRUNK_HINGE_X,
            -0.028, 0.028, z_bot_lid + 0.010 - hz, z_bot_lid + 0.035 - hz)

    trunk_lid = bmesh_to_object('Trunk_Lid', bm_lid, material=mats['paint'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.003, bevel_segs=2, subsurf=1)
    uv_box(trunk_lid, scale=1.0)
    trunk_lid.location = hinge_origin
    trunk_lid.parent = body_ob
    trunk_lid.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(trunk_lid)

    # 2b. Европейский задний номерной знак внутри ниши багажника (открывается вместе с Trunk_Lid!)
    bm_pl_r = bmesh.new()
    z_pl_c = lip_top_z[ny // 2] * 0.44 + z_bot_lid * 0.56 - hz
    x_pl_r = rear_tail_surface_x(0.0) + 0.009 - TRUNK_HINGE_X
    # Тонкая чёрная рамка (slot 1 = Piano_Black)
    add_box(bm_pl_r, x_pl_r - 0.003, x_pl_r + 0.005, -0.245, +0.245, z_pl_c - 0.056, z_pl_c + 0.056)
    bm_pl_r.faces.ensure_lookup_table()
    for f_p in bm_pl_r.faces:
        f_p.material_index = 1
    # Белое поле номера (slot 0 = White)
    n_bef_w = len(bm_pl_r.faces)
    add_box(bm_pl_r, x_pl_r - 0.006, x_pl_r - 0.002, -0.236, +0.236, z_pl_c - 0.048, z_pl_c + 0.048)
    bm_pl_r.faces.ensure_lookup_table()
    for f_idx in range(n_bef_w, len(bm_pl_r.faces)):
        bm_pl_r.faces[f_idx].material_index = 0
    # Синяя евро-полоска с левого края (slot 2 = Paint)
    n_bef_eu = len(bm_pl_r.faces)
    add_box(bm_pl_r, x_pl_r - 0.007, x_pl_r - 0.004, +0.192, +0.236, z_pl_c - 0.048, z_pl_c + 0.048)
    bm_pl_r.faces.ensure_lookup_table()
    for f_idx in range(n_bef_eu, len(bm_pl_r.faces)):
        bm_pl_r.faces[f_idx].material_index = 2
    pl_r = bmesh_to_object('License_Plate_Rear', bm_pl_r, material=mats['white'],
                           smooth=True, collection=collection, bevel=0.0012, bevel_segs=1)
    pl_r.data.materials.append(mats['piano_black'])
    pl_r.data.materials.append(mats['paint'])
    uv_box(pl_r, scale=5.0)
    pl_r.parent = trunk_lid
    pl_r.location = Vector((0.0, 0.0, 0.0))
    objs.append(pl_r)

    # 2c. Ворсовая обивка изнанки крышки багажника, пенал аварийного знака и кнопка закрытия
    # Материалы: 0=carpet, 1=piano_black, 2=caliper (красный знак), 3=trunk_led
    bm_tl = bmesh.new()

    def _tl_box(x0, x1, y0, y1, z0, z1, m_idx):
        n0 = len(bm_tl.faces)
        add_box(bm_tl, x0, x1, y0, y1, z0, z1)
        bm_tl.faces.ensure_lookup_table()
        for fi in range(n0, len(bm_tl.faces)):
            bm_tl.faces[fi].material_index = m_idx

    z_liner = trunk_deck_z(-2.05, 0.0) - TRUNK_LID_THICKNESS - 0.022 - hz
    # Формованная ворсовая панель на изнанке верхней крышки багажника (между петлями)
    _tl_box(-0.49, -0.09, -0.315, +0.315, z_liner - 0.010, z_liner, 0)
    # Рельефная выштамповка-карман и ручка закрывания багажника справа
    _tl_box(-0.46, -0.38, -0.25, -0.11, z_liner - 0.018, z_liner - 0.009, 1)
    # Фирменный красный пенал знака аварийной остановки Volvo по центру изнанки крышки
    _tl_box(-0.34, -0.24, -0.19, +0.19, z_liner - 0.036, z_liner - 0.011, 2)
    # Два чёрных ремешка-держателя пенала аварийного знака
    for sy_s in (-0.13, +0.13):
        _tl_box(-0.345, -0.235, sy_s - 0.012, sy_s + 0.012, z_liner - 0.039, z_liner - 0.009, 1)
    # Кнопка электропривода закрывания багажника с мягкой белой LED-подсветкой (на нижней кромке)
    x_btn = rear_tail_surface_x(0.0) + TRUNK_LID_THICKNESS + 0.016 - TRUNK_HINGE_X
    _tl_box(x_btn - 0.014, x_btn + 0.014, -0.16, -0.09,
            z_bot_lid + 0.014 - hz, z_bot_lid + 0.034 - hz, 1)
    _tl_box(x_btn - 0.008, x_btn + 0.008, -0.145, -0.105,
            z_bot_lid + 0.009 - hz, z_bot_lid + 0.015 - hz, 3)

    tl_ob = bmesh_to_object('Trunk_Lid_Liner', bm_tl, material=mats['carpet'],
                            smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
    tl_ob.data.materials.append(mats['piano_black'])
    tl_ob.data.materials.append(mats['caliper'])
    tl_ob.data.materials.append(mats['trunk_led'])
    uv_box(tl_ob, scale=3.5)
    tl_ob.parent = trunk_lid
    tl_ob.location = Vector((0.0, 0.0, 0.0))
    objs.append(tl_ob)

    # 2d. Оснащение багажного отсека: накладка порога, ручка подполья, проушины, сетки,
    # лючок длинномеров, кассета-шторка и LED-подсветка (Trunk_Details, родитель Body)
    # Материалы: 0=chrome, 1=rubber, 2=piano_black, 3=trunk_led, 4=carbon
    bm_td = bmesh.new()

    def _td_box(x0, x1, y0, y1, z0, z1, m_idx):
        n0 = len(bm_td.faces)
        add_box(bm_td, x0, x1, y0, y1, z0, z1)
        bm_td.faces.ensure_lookup_table()
        for fi in range(n0, len(bm_td.faces)):
            bm_td.faces[fi].material_index = m_idx

    # Металлическая защитная накладка погрузочного порога + 5 резиновых полос
    _td_box(-2.266, -2.238, -0.445, +0.445, 0.643, 0.654, 0)
    for k_st in range(5):
        ys = -0.34 + k_st * 0.17
        if abs(ys) < 0.05:
            continue
        _td_box(-2.264, -2.240, ys - 0.048, ys + 0.048, 0.6545, 0.659, 1)
    # Центральная ответная скоба замка багажника в утопленной чёрной чашке
    _td_box(-2.264, -2.238, -0.052, +0.052, 0.653, 0.660, 2)
    _td_box(-2.256, -2.246, -0.024, +0.024, 0.661, 0.674, 0)

    # Контур люка подполья (докатки) и утопленная хромированная ручка подъёма пола
    _td_box(-2.14, -2.125, -0.34, +0.34, 0.5065, 0.5095, 1)
    _td_box(-1.52, -1.505, -0.34, +0.34, 0.5065, 0.5095, 1)
    _td_box(-2.123, -1.522, -0.355, -0.341, 0.5065, 0.5095, 1)
    _td_box(-2.123, -1.522, +0.341, +0.355, 0.5065, 0.5095, 1)
    _td_box(-2.12, -2.055, -0.068, +0.068, 0.5065, 0.514, 2)
    _td_box(-2.104, -2.088, -0.046, +0.046, 0.5145, 0.522, 0)

    # 4 хромированные такелажные проушины (петли крепления груза) по углам пола
    for tx in (-2.15, -1.36):
        for ty in (-0.405, +0.405):
            _td_box(tx - 0.022, tx + 0.022, ty - 0.018, ty + 0.018, 0.5065, 0.513, 2)
            _td_box(tx - 0.016, tx + 0.016, ty - 0.012, ty + 0.012, 0.5135, 0.525, 0)

    # Задняя натянутая эластичная багажная сетка-органайзер (поперёк багажника на x=-2.04)
    for k_hz in range(4):
        zz_n = 0.528 + k_hz * 0.044
        _sweep_round_tube(
            bm_td,
            [Vector((-2.04, -0.405, zz_n)), Vector((-2.04, 0.0, zz_n)),
             Vector((-2.04, +0.405, zz_n))],
            radius=0.0032 if k_hz in (0, 3) else 0.0020, seg=8, mat_idx=1,
        )
    for k_vy in range(7):
        yy_n = -0.39 + k_vy * 0.13
        _sweep_round_tube(
            bm_td,
            [Vector((-2.04, yy_n, 0.525)), Vector((-2.04, yy_n, 0.662))],
            radius=0.0020, seg=8, mat_idx=1,
        )
    # Боковые карманные сетки-органайзеры вдоль левой и правой стенок (ниже хода лифтов)
    for side in (1.0, -1.0):
        yn_s = side * 0.448
        for zz_s in (0.526, 0.578, 0.628):
            _sweep_round_tube(
                bm_td,
                [Vector((-1.98, yn_s, zz_s)), Vector((-1.62, yn_s, zz_s))],
                radius=0.0028 if zz_s > 0.60 else 0.0018, seg=8, mat_idx=1,
            )
        for xx_s in (-1.96, -1.85, -1.74, -1.63):
            _sweep_round_tube(
                bm_td,
                [Vector((xx_s, yn_s, 0.524)), Vector((xx_s, yn_s, 0.630))],
                radius=0.0018, seg=8, mat_idx=1,
            )
        # Кронштейны боковой сетки
        for xx_c in (-1.985, -1.615):
            ya, yb = (side * 0.446, side * 0.472)
            y0, y1 = (ya, yb) if ya < yb else (yb, ya)
            _td_box(xx_c - 0.009, xx_c + 0.009, y0, y1, 0.622, 0.642, 2)

    # Справа на стенке багажника: розетка 12V и откидной крючок для пакетов
    _td_box(-1.54, -1.48, -0.472, -0.456, 0.665, 0.720, 2)
    _td_box(-1.525, -1.495, -0.454, -0.445, 0.680, 0.705, 0)
    _td_box(-1.74, -1.70, -0.472, -0.444, 0.720, 0.742, 2)
    _td_box(-1.73, -1.71, -0.442, -0.434, 0.704, 0.730, 2)

    # Передняя стенка за спинкой заднего сиденья: лыжный лючок + кассета-шторка с рейкой
    # Рамка лыжного лючка для длинномеров по центру спинки
    _td_box(-1.252, -1.241, -0.125, +0.125, 0.545, 0.770, 2)
    _td_box(-1.258, -1.253, -0.105, +0.105, 0.562, 0.752, 4)
    _td_box(-1.264, -1.259, -0.032, +0.032, 0.718, 0.742, 0)  # хромированная клавиша лючка
    # Верхняя кассета выдвижной полки-шторки с хромированной рукояткой-рейкой
    _td_box(-1.315, -1.242, -0.460, +0.460, 0.795, 0.852, 4)
    _td_box(-1.345, -1.316, -0.445, +0.445, 0.816, 0.832, 0)  # хромированная направляющая шторки

    # Светодиодная подсветка багажного отсека: 2 боковых LED-плафона + 1 верхний центральный
    for side in (1.0, -1.0):
        ya, yb = (side * 0.460, side * 0.473)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        _td_box(-1.60, -1.49, y0, y1, 0.740, 0.768, 2)
        ya_l, yb_l = (side * 0.454, side * 0.461)
        yl0, yl1 = (ya_l, yb_l) if ya_l < yb_l else (yb_l, ya_l)
        _td_box(-1.588, -1.502, yl0, yl1, 0.746, 0.762, 3)
    # Верхний центральный LED-плафон под полкой багажника
    _td_box(-1.38, -1.31, -0.075, +0.075, 0.838, 0.852, 2)
    _td_box(-1.372, -1.318, -0.062, +0.062, 0.832, 0.839, 3)

    td_ob = bmesh_to_object('Trunk_Details', bm_td, material=mats['chrome'],
                            smooth=True, collection=collection, bevel=0.0)
    td_ob.data.materials.append(mats['rubber'])
    td_ob.data.materials.append(mats['piano_black'])
    td_ob.data.materials.append(mats['trunk_led'])
    td_ob.data.materials.append(mats['carbon'])
    uv_box(td_ob, scale=4.0)
    td_ob.parent = body_ob
    td_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(td_ob)

    # 3. Петли багажника («гусиная шея») и телескопические газовые лифты (слева и справа)
    bx, bz = TRUNK_LIFT_BODY_XZ
    lx, lz = trunk_lift_lid_local_xz()

    for label, side in (('L', 1.0), ('R', -1.0)):
        hy = side * TRUNK_HINGE_Y
        ly = side * TRUNK_LIFT_Y

        # 3a. Неподвижный кронштейн петли на кузове + ось шарнира
        bm_hm = bmesh.new()
        add_box(bm_hm, -0.025, 0.045, -0.022, 0.022, -0.055, 0.022)
        _add_y_cylinder(bm_hm, 0.011, -0.026, 0.026, seg=14, center_xz=(0.0, 0.0))
        hinge_mount = bmesh_to_object(f'Trunk_Hinge_Mount_{label}', bm_hm,
                                      material=mats['carbon'], smooth=True,
                                      collection=collection)
        uv_box(hinge_mount, scale=6.0)
        hinge_mount.location = Vector((TRUNK_HINGE_X, hy, hz))
        hinge_mount.parent = body_ob
        hinge_mount.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(hinge_mount)

        # 3b. Изогнутый рычаг петли («гусиная шея»), вращается вместе с Trunk_Lid
        bm_ha = bmesh.new()
        # Локальные точки относительно hinge_origin (0, 0, 0)
        z_pad = (trunk_deck_z(TRUNK_HINGE_X - 0.22, hy) - TRUNK_LID_THICKNESS - 0.008) - hz
        arm_pts = [
            Vector((0.00, hy, 0.00)),
            Vector((0.038, hy, -0.048)),
            Vector((0.018, hy, -0.100)),
            Vector((-0.060, hy, -0.105)),
            Vector((-0.145, hy, -0.075)),
            Vector((-0.220, hy, z_pad)),
        ]
        _sweep_rect_tube(bm_ha, arm_pts, half_y=0.010, half_n=0.013)
        hinge_arm = bmesh_to_object(f'Trunk_Hinge_Arm_{label}', bm_ha,
                                    material=mats['carbon'], smooth=True,
                                    collection=collection)
        uv_box(hinge_arm, scale=6.0)
        hinge_arm.parent = trunk_lid
        hinge_arm.location = Vector((0.0, 0.0, 0.0))
        objs.append(hinge_arm)

        # 3c. Нижняя опора газового лифта на стенке ниши кузова (цель для штока)
        bm_lm = bmesh.new()
        add_box(bm_lm, -0.018, 0.018, -0.018, 0.018, -0.018, 0.018)
        lift_mount = bmesh_to_object(f'Trunk_Lift_Mount_{label}', bm_lm,
                                     material=mats['carbon'], smooth=True,
                                     collection=collection)
        uv_box(lift_mount, scale=8.0)
        lift_mount.location = Vector((bx, ly, bz))
        lift_mount.parent = body_ob
        lift_mount.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(lift_mount)

        # 3d. Верхний кронштейн газового лифта на внутренней стороне крышки (цель для цилиндра)
        bm_lb = bmesh.new()
        add_box(bm_lb, -0.022, 0.022, -0.018, 0.018, -0.016, 0.016)
        lift_bracket = bmesh_to_object(f'Trunk_Lift_Bracket_{label}', bm_lb,
                                       material=mats['carbon'], smooth=True,
                                       collection=collection)
        uv_box(lift_bracket, scale=8.0)
        lift_bracket.parent = trunk_lid
        lift_bracket.location = Vector((lx, ly, lz))
        objs.append(lift_bracket)

        # 3e. Внешний корпус цилиндра газового лифта (отслеживает кронштейн на крышке)
        bm_cyl = bmesh.new()
        _add_z_cylinder(bm_cyl, 0.014, 0.0, TRUNK_LIFT_CYL_LEN, seg=18)
        lift_cyl = bmesh_to_object(f'Trunk_Lift_Cyl_{label}', bm_cyl,
                                   material=mats['carbon'], smooth=True,
                                   collection=collection)
        uv_box(lift_cyl, scale=6.0)
        lift_cyl.location = Vector((bx, ly, bz))
        lift_cyl.parent = body_ob
        lift_cyl.matrix_parent_inverse = body_ob.matrix_world.inverted()
        con_cyl = lift_cyl.constraints.new('TRACK_TO')
        con_cyl.target = lift_bracket
        con_cyl.track_axis = 'TRACK_Z'
        con_cyl.up_axis = 'UP_Y'
        objs.append(lift_cyl)

        # 3f. Хромированный шток газового лифта (закреплён на крышке, отслеживает нижнюю опору)
        bm_rod = bmesh.new()
        _add_z_cylinder(bm_rod, 0.007, 0.0, TRUNK_LIFT_ROD_LEN, seg=16)
        lift_rod = bmesh_to_object(f'Trunk_Lift_Rod_{label}', bm_rod,
                                   material=mats['chrome'], smooth=True,
                                   collection=collection)
        uv_box(lift_rod, scale=6.0)
        lift_rod.parent = trunk_lid
        lift_rod.location = Vector((lx, ly, lz))
        con_rod = lift_rod.constraints.new('TRACK_TO')
        con_rod.target = lift_mount
        con_rod.track_axis = 'TRACK_Z'
        con_rod.up_axis = 'UP_Y'
        objs.append(lift_rod)

    return trunk_lid, objs


def build_hood(body_ob, mats, collection):
    """Строит отделённый капот (Hood_Lid с внутренней силовой рамой и скобой замка),
    реалистичный моторный отсек поперечной компоновки Volvo Drive-E (Engine_Bay, Engine_Metal,
    Coolant_Tank), петли капота и телескопические газовые лифты с ацикличным TRACK_TO."""
    objs = []
    hz = hood_hinge_z()
    hinge_origin = Vector((HOOD_HINGE_X, 0.0, hz))

    def _attach_body(ob):
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(ob)
        return ob

    # 1a. Основные агрегаты моторного отсека (Engine_Bay — тёмный пластик, кожухи, патрубки)
    bm_eng = bmesh.new()
    z_sub = 0.275
    # Поперечный 4-цилиндровый блок ДВС (расположен правее оси Y, КПП — слева, как у Volvo S90)
    add_box(bm_eng, 1.42, 1.84, -0.34, 0.08, z_sub, 0.635)
    # Скульптурная декоративная плита Volvo Drive-E с боковыми и продольными рёбрами
    add_box(bm_eng, 1.44, 1.81, -0.32, 0.06, 0.642, 0.700)
    for k_rib in range(4):
        ry = -0.27 + k_rib * 0.095
        add_box(bm_eng, 1.455, 1.795, ry - 0.018, ry + 0.018, 0.702, 0.714)
    # Маслозаливная горловина на передней правой части крышки двигателя
    _add_z_cylinder(bm_eng, 0.028, 0.700, 0.734, seg=16, center_xy=(1.75, -0.25))

    # Основной радиатор + кожух электровентилятора сразу за «телевизором»
    add_box(bm_eng, 2.02, 2.11, -0.45, 0.45, 0.295, 0.715)
    # Верхний и нижний гнутые резиновые патрубки ОЖ (радиатор <-> термостат/помпа ДВС)
    _sweep_round_tube(bm_eng, [
        Vector((1.82, -0.25, 0.625)),
        Vector((1.89, -0.28, 0.645)),
        Vector((1.96, -0.30, 0.655)),
        Vector((2.03, -0.30, 0.640)),
    ], radius=0.022, seg=12)
    _sweep_round_tube(bm_eng, [
        Vector((1.82, 0.02, 0.365)),
        Vector((1.90, 0.05, 0.355)),
        Vector((2.03, 0.05, 0.365)),
    ], radius=0.020, seg=12)
    # Тонкий возвратный шланг ОЖ от верхней банки радиатора к расширительному бачку
    # (аккуратно огибает синюю крышку бачка омывателя на (1.90, -0.41))
    _sweep_round_tube(bm_eng, [
        Vector((2.03, -0.34, 0.695)),
        Vector((1.88, -0.34, 0.695)),
        Vector((1.66, -0.37, 0.690)),
        Vector((1.46, -0.39, 0.665)),
    ], radius=0.007, seg=10)

    # Корпус воздушного фильтра (Airbox, слева +Y) + рёбра жёсткости + холодный впуск
    add_box(bm_eng, 1.66, 1.98, 0.25, 0.45, 0.44, 0.675)
    for rx_a in (1.72, 1.80, 1.88, 1.94):
        add_box(bm_eng, rx_a - 0.008, rx_a + 0.008, 0.265, 0.435, 0.677, 0.686)
    add_box(bm_eng, 1.982, 2.13, 0.30, 0.42, 0.60, 0.665)  # заборник холодного воздуха
    # Изогнутый гофрированный патрубок впуска от корпуса фильтра к турбине/ДВС
    _sweep_round_tube(bm_eng, [
        Vector((1.66, 0.32, 0.625)),
        Vector((1.58, 0.27, 0.635)),
        Vector((1.53, 0.17, 0.630)),
        Vector((1.53, 0.07, 0.615)),
    ], radius=0.032, seg=14)
    # Гофрированные кольца-усилители на впускном патрубке
    for cy_g in (0.12, 0.16, 0.20, 0.24):
        _add_y_cylinder(bm_eng, 0.037, cy_g - 0.006, cy_g + 0.006, seg=14, center_xz=(1.54, 0.630))

    # Короб АКБ 12V и блок реле/предохранителей у моторного щита (слева +Y)
    add_box(bm_eng, 1.18, 1.42, 0.27, 0.47, 0.39, 0.665)
    add_box(bm_eng, 1.44, 1.62, 0.32, 0.47, 0.44, 0.642)   # блок реле и предохранителей
    add_box(bm_eng, 1.455, 1.605, 0.335, 0.455, 0.644, 0.654)  # крышка блока предохранителей
    # Кабельный жгут вдоль моторного щита к блоку предохранителей и катушкам
    _sweep_round_tube(bm_eng, [
        Vector((1.18, -0.44, 0.705)),
        Vector((1.18, -0.10, 0.705)),
        Vector((1.18, +0.24, 0.705)),
        Vector((1.44, +0.34, 0.650)),
    ], radius=0.011, seg=10)

    # Бачок омывателя стекла у правого брызговика (-Y) с заливной горловиной
    add_box(bm_eng, 1.78, 1.98, -0.47, -0.35, 0.36, 0.635)
    _add_z_cylinder(bm_eng, 0.022, 0.636, 0.688, seg=14, center_xy=(1.90, -0.41))

    engine_bay = bmesh_to_object('Engine_Bay', bm_eng, material=mats['carbon'],
                                 smooth=True, collection=collection, bevel=0.003)
    uv_box(engine_bay, scale=2.5)
    _attach_body(engine_bay)

    # 1b. Металлические узлы моторного отсека (Engine_Metal — алюминий: КПП, коллектор,
    # трубки кондиционера, топливная рампа, хомуты, распорка чашек)
    bm_em = bmesh.new()
    # Корпус поперечной 8-ступенчатой КПП (слева от блока ДВС)
    add_box(bm_em, 1.46, 1.78, 0.085, 0.295, 0.31, 0.575)
    # Алюминиевый впускной ресивер и 4 патрубка + топливная рампа перед блоком цилиндров
    add_box(bm_em, 1.845, 1.94, -0.28, 0.04, 0.54, 0.63)
    for k_run in range(4):
        ry = -0.24 + k_run * 0.082
        add_box(bm_em, 1.81, 1.843, ry - 0.018, ry + 0.018, 0.57, 0.625)
    _add_y_cylinder(bm_em, 0.011, -0.27, +0.03, seg=12, center_xz=(1.835, 0.648))
    # Теплозащитный экран турбины/выпуска у моторного щита
    add_box(bm_em, 1.24, 1.39, -0.28, 0.04, 0.44, 0.635)
    # Верхние опорные фланцы передних амортизационных стоек (с центральной гайкой и 3 болтами)
    for side in (1.0, -1.0):
        _add_z_cylinder(bm_em, 0.072, 0.792, 0.816, seg=18, center_xy=(AXLE_F, side * 0.54))
        _add_z_cylinder(bm_em, 0.018, 0.817, 0.834, seg=12, center_xy=(AXLE_F, side * 0.54))
        for kb in range(3):
            ba = kb * 2.0 * math.pi / 3.0
            _add_z_cylinder(
                bm_em, 0.008, 0.817, 0.826, seg=8,
                center_xy=(AXLE_F + 0.048 * math.cos(ba), side * 0.54 + 0.048 * math.sin(ba)),
            )
    # Алюминиевая поперечная распорка чашек стоек (перед жабо)
    add_box(bm_em, 1.28, 1.32, -0.50, 0.50, 0.745, 0.772)
    # Две алюминиевые магистрали кондиционера A/C вдоль правого брызговика (-Y)
    for dy_ac, r_ac in ((-0.465, 0.0085), (-0.445, 0.0065)):
        _sweep_round_tube(bm_em, [
            Vector((1.20, dy_ac, 0.675)),
            Vector((1.52, dy_ac, 0.695)),
            Vector((1.78, dy_ac, 0.690)),
            Vector((2.04, dy_ac + 0.03, 0.665)),
        ], radius=r_ac, seg=10)
    # Обжимные металлические хомуты на патрубках ОЖ и впуска
    _add_x_cylinder(bm_em, 0.025, 1.845, 1.860, seg=14, center_yz=(-0.265, 0.635))
    _add_x_cylinder(bm_em, 0.025, 1.995, 2.010, seg=14, center_yz=(-0.300, 0.648))
    _add_y_cylinder(bm_em, 0.036, 0.275, 0.290, seg=14, center_xz=(1.59, 0.633))
    em_ob = bmesh_to_object('Engine_Metal', bm_em, material=mats['alloy'],
                            smooth=True, collection=collection, bevel=0.002)
    uv_box(em_ob, scale=3.5)
    _attach_body(em_ob)

    # 1c. Двухцветный расширительный бачок ОЖ (Coolant_Tank: молочный верх + розовый антифриз
    # G12 внутри + чёрная крышка давления)
    bm_ct = bmesh.new()
    # Нижняя часть с видимым уровнем розового антифриза (slot 1 = coolant_fluid)
    add_box(bm_ct, 1.26, 1.46, -0.47, -0.33, 0.55, 0.622)
    bm_ct.faces.ensure_lookup_table()
    for f_c in bm_ct.faces:
        f_c.material_index = 1
    # Верхняя молочно-белая колба бачка (slot 0 = white)
    n_ct_w = len(bm_ct.faces)
    add_box(bm_ct, 1.26, 1.46, -0.47, -0.33, 0.623, 0.678)
    bm_ct.faces.ensure_lookup_table()
    for fi in range(n_ct_w, len(bm_ct.faces)):
        bm_ct.faces[fi].material_index = 0
    # Чёрная ребристая крышка давления бачка ОЖ (slot 2 = rubber)
    n_ct_cap = len(bm_ct.faces)
    _add_z_cylinder(bm_ct, 0.029, 0.679, 0.712, seg=16, center_xy=(1.36, -0.40))
    bm_ct.faces.ensure_lookup_table()
    for fi in range(n_ct_cap, len(bm_ct.faces)):
        bm_ct.faces[fi].material_index = 2
    ct_ob = bmesh_to_object('Coolant_Tank', bm_ct, material=mats['white'],
                            smooth=True, collection=collection, bevel=0.0025)
    ct_ob.data.materials.append(mats['coolant_fluid'])
    ct_ob.data.materials.append(mats['rubber'])
    uv_box(ct_ob, scale=6.0)
    _attach_body(ct_ob)

    # 1d. Детализация моторного отсека Volvo Drive-E T6 AWD (Engine_Details, родитель Body):
    # - Упакованная металлическая табличка крышки ДВС VOLVO / DRIVE-E T6 AWD (slot 0)
    # - Заводская сервисная табличка на телевизоре R1234YF / 0W-20 / CAUTION FAN (slot 1)
    # - Клеммы АКБ (+ красная с силовым кабелем, - хром), синяя крышка омывателя,
    #   жёлтый масляный щуп, сервисные колпачки A/C, окантовка горловины масла
    # Материалы: 0=engine_badge, 1=engine_spec, 2=chrome, 3=caliper, 4=washer_blue,
    #            5=warning_yellow, 6=piano_black
    bm_ed = bmesh.new()

    def _ed_box(x0, x1, y0, y1, z0, z1, m_idx):
        n0 = len(bm_ed.faces)
        add_box(bm_ed, x0, x1, y0, y1, z0, z1)
        bm_ed.faces.ensure_lookup_table()
        for fi in range(n0, len(bm_ed.faces)):
            bm_ed.faces[fi].material_index = m_idx

    def _ed_zcyl(r, z0, z1, xy, m_idx, seg=14):
        n0 = len(bm_ed.faces)
        _add_z_cylinder(bm_ed, r, z0, z1, seg=seg, center_xy=xy)
        bm_ed.faces.ensure_lookup_table()
        for fi in range(n0, len(bm_ed.faces)):
            bm_ed.faces[fi].material_index = m_idx

    # Табличка крышки двигателя VOLVO DRIVE-E T6 AWD + тонкая хромированная подложка-рамка
    # (читаема спереди: x_back=1.525..x_front=1.705, y_right=-0.275..y_left=+0.025)
    _ed_box(1.518, 1.712, -0.282, +0.032, 0.7145, 0.7185, 2)
    _ed_box(1.525, 1.705, -0.275, +0.025, 0.7186, 0.7220, 0)

    # Заводская сервисная табличка на верхней балке «телевизора» (справа от замка капота)
    # (x_back=2.142..x_front=2.226, y_right=-0.400..y_left=-0.148)
    _ed_box(2.142, 2.226, -0.400, -0.148, 0.7465, 0.7505, 1)

    # Хромированный ободок маслозаливной пробки
    _ed_zcyl(0.030, 0.7345, 0.741, (1.75, -0.25), 2, seg=16)
    _ed_zcyl(0.023, 0.7412, 0.745, (1.75, -0.25), 6, seg=14)

    # Синяя фирменная крышка бачка омывателя с откидным язычком (у правого брызговика)
    _ed_zcyl(0.026, 0.6885, 0.704, (1.90, -0.41), 4, seg=16)
    _ed_box(1.888, 1.912, -0.452, -0.432, 0.695, 0.703, 4)

    # Ярко-жёлтая петля-ручка масляного щупа перед блоком цилиндров
    _ed_zcyl(0.0045, 0.615, 0.688, (1.835, -0.04), 6, seg=10)
    _ed_box(1.825, 1.845, -0.062, -0.018, 0.689, 0.718, 5)
    _ed_box(1.828, 1.842, -0.050, -0.030, 0.696, 0.710, 6)

    # АКБ 12V слева у моторного щита: 6 пробок, красная плюсовая клемма +, силовой кабель,
    # хромированная минусовая клемма - и жёлтая сервисная наклейка
    for k_cell in range(6):
        cy_b = 0.295 + k_cell * 0.030
        _ed_zcyl(0.0085, 0.666, 0.673, (1.30, cy_b), 6, seg=10)
    # Красная защитная крышка плюсовой клеммы (+) + красный силовой кабель к предохранителям
    _ed_box(1.345, 1.405, 0.305, 0.355, 0.666, 0.698, 3)
    _sweep_round_tube(bm_ed, [
        Vector((1.400, 0.330, 0.682)),
        Vector((1.445, 0.345, 0.680)),
        Vector((1.485, 0.375, 0.656)),
    ], radius=0.008, seg=10, mat_idx=3)
    # Минусовая клемма (-) из хрома + чёрный провод массы на брызговик
    _ed_zcyl(0.012, 0.666, 0.692, (1.23, 0.43), 2, seg=12)
    _ed_box(1.215, 1.245, 0.415, 0.448, 0.674, 0.688, 2)
    # Жёлтая наклейка предупреждения на крышке АКБ
    _ed_box(1.21, 1.27, 0.295, 0.375, 0.666, 0.670, 5)

    # Сервисные колпачки заправки кондиционера (L = синий, H = красный) на трубках A/C
    _ed_zcyl(0.009, 0.696, 0.716, (1.64, -0.465), 4, seg=12)
    _ed_zcyl(0.009, 0.692, 0.712, (1.72, -0.445), 3, seg=12)

    ed_ob = bmesh_to_object('Engine_Details', bm_ed, material=mats['engine_badge'],
                            smooth=True, collection=collection, bevel=0.0008, bevel_segs=1)
    ed_ob.data.materials.append(mats['engine_spec'])
    ed_ob.data.materials.append(mats['chrome'])
    ed_ob.data.materials.append(mats['caliper'])
    ed_ob.data.materials.append(mats['washer_blue'])
    ed_ob.data.materials.append(mats['warning_yellow'])
    ed_ob.data.materials.append(mats['piano_black'])
    uv_box(ed_ob, scale=4.0)
    uv_face_z_plus(ed_ob, 0, x_back=1.525, x_front=1.705, y_right=-0.275, y_left=+0.025)
    uv_face_z_plus(ed_ob, 1, x_back=2.142, x_front=2.226, y_right=-0.400, y_left=-0.148)
    _attach_body(ed_ob)

    # 2. Отделённая крышка капота (origin в оси задней петли, вращается вокруг -Y)
    # Плавная обтекаемая передняя кромка по дуге front_nose_surface_x(y) с чётким 7-мм зазором
    # до крыльев (WR+0.044) и съёмного переднего бампера Front_Bumper!
    bm_hood = bmesh.new()
    nx = 18
    ny = 13
    x_start = HOOD_X_REAR + 0.006

    outer_pts = []
    inner_pts = []
    for i in range(nx):
        u_x = i / float(nx - 1)
        x_nom = x_start + (FRONT_BUMPER_X - 0.008 - x_start) * u_x
        hw = WR(x_nom) + 0.044
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            # У передней кромки капот плавно повторяет обтекаемую дугу носа front_nose_surface_x
            x_front_edge = front_nose_surface_x(y) - 0.028
            t_f = clamp((u_x - 0.55) / 0.45, 0.0, 1.0)
            t_f = t_f * t_f * (3.0 - 2.0 * t_f)
            x = x_nom * (1.0 - t_f) + (x_start + (x_front_edge - x_start) * u_x) * t_f
            edge_drop = 0.008 * (abs(v) ** 2.6)
            # Фирменные V-образные силовые рёбра жёсткости на капоте Volvo S90 (power creases)
            y_crease = 0.50 * (1.0 - u_x) + 0.34 * u_x
            crease_fade = math.sin(math.pi * clamp((u_x - 0.06) / 0.86, 0.0, 1.0))
            power_crease = 0.009 * crease_fade * math.exp(-(((abs(y) - y_crease) / 0.065) ** 2))
            # Плавный передний клюв капота S90: мягко спускается прямо к верху решётки и фар
            nose_drop = 0.064 * (max(0.0, (u_x - 0.65) / 0.35) ** 1.65)
            z_top = (
                hood_deck_z(min(x, X_MAX - 0.05), y)
                - edge_drop - nose_drop + power_crease + 0.001
            )
            row_out.append(Vector((x - HOOD_HINGE_X, y, z_top - hz)))
            row_in.append(Vector((x - HOOD_HINGE_X, y, z_top - HOOD_LID_THICKNESS - hz)))
        outer_pts.append(row_out)
        inner_pts.append(row_in)
    _add_thick_grid(bm_hood, outer_pts, inner_pts)

    # Внутренняя усиливающая силовая рама капота (рёбра жёсткости + скоба замка на изнанке)
    def _hood_inner_bar(x0, y0, x1, y1, half_w=0.022, depth=0.018):
        p0 = Vector((x0 - HOOD_HINGE_X, y0,
                     hood_deck_z(x0, y0) - HOOD_LID_THICKNESS - 0.006 - hz))
        p1 = Vector((x1 - HOOD_HINGE_X, y1,
                     hood_deck_z(x1, y1) - HOOD_LID_THICKNESS - 0.006 - hz))
        d = (p1 - p0)
        if d.length < 1e-4:
            return
        d_xy = Vector((d.x, d.y, 0.0)).normalized()
        perp = Vector((-d_xy.y, d_xy.x, 0.0)) * half_w
        up = Vector((0.0, 0.0, 1.0))
        r0 = [
            bm_hood.verts.new(p0 - perp),
            bm_hood.verts.new(p0 + perp),
            bm_hood.verts.new(p0 + perp - up * depth),
            bm_hood.verts.new(p0 - perp - up * depth),
        ]
        r1 = [
            bm_hood.verts.new(p1 - perp),
            bm_hood.verts.new(p1 + perp),
            bm_hood.verts.new(p1 + perp - up * depth),
            bm_hood.verts.new(p1 - perp - up * depth),
        ]
        for k in range(4):
            kn = (k + 1) % 4
            bm_hood.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        bm_hood.faces.new((r0[0], r0[3], r0[2], r0[1]))
        bm_hood.faces.new((r1[0], r1[1], r1[2], r1[3]))

    # Периметральные и диагональные рёбра жёсткости на внутренней стороне капота
    _hood_inner_bar(1.12, -0.48, 2.16, -0.42)
    _hood_inner_bar(1.12, +0.48, 2.16, +0.42)
    _hood_inner_bar(1.12, -0.45, 1.12, +0.45)
    _hood_inner_bar(2.16, -0.39, 2.16, +0.39)
    _hood_inner_bar(1.16, -0.42, 2.12, +0.36, half_w=0.016, depth=0.014)
    _hood_inner_bar(1.16, +0.42, 2.12, -0.36, half_w=0.016, depth=0.014)
    # Центральная ответная скоба замка капота (входит в замок на Radiator_Support)
    _hood_inner_bar(2.16, -0.025, 2.22, +0.025, half_w=0.008, depth=0.030)

    hood_lid = bmesh_to_object('Hood_Lid', bm_hood, material=mats['paint'],
                               smooth=True, collection=collection, auto_smooth_deg=55.0,
                               bevel=0.003, bevel_segs=2, subsurf=1)
    uv_box(hood_lid, scale=1.0)
    hood_lid.location = hinge_origin
    _attach_body(hood_lid)

    # 2b. Формованная термошумоизоляция изнанки капота с 6 крепёжными клипсами (Hood_Insulation)
    bm_hi = bmesh.new()
    hi_top, hi_bot = [], []
    for ix in range(7):
        ux = ix / 6.0
        xw = 1.22 + (2.06 - 1.22) * ux
        hw_i = 0.42 - 0.06 * ux
        rt, rb = [], []
        for jy in range(7):
            vy = -1.0 + 2.0 * jy / 6.0
            yw = vy * hw_i
            z_in = hood_deck_z(xw, yw) - HOOD_LID_THICKNESS - 0.020 - hz
            # Рельефная выштамповка по центру мата
            emboss = 0.004 * (1.0 - vy * vy) * math.sin(math.pi * ux)
            rt.append(Vector((xw - HOOD_HINGE_X, yw, z_in)))
            rb.append(Vector((xw - HOOD_HINGE_X, yw, z_in - 0.010 - emboss)))
        hi_top.append(rt)
        hi_bot.append(rb)
    _add_thick_grid(bm_hi, hi_top, hi_bot)
    # 6 круглых пластиковых пистонов крепления шумоизоляции (slot 1 = piano_black)
    n_hi_clips = len(bm_hi.faces)
    for cx_p in (1.30, 1.64, 1.98):
        for cy_p in (-0.30, +0.30):
            zp = hood_deck_z(cx_p, cy_p) - HOOD_LID_THICKNESS - 0.032 - hz
            _add_z_cylinder(bm_hi, 0.013, zp - 0.005, zp + 0.004, seg=12,
                            center_xy=(cx_p - HOOD_HINGE_X, cy_p))
    bm_hi.faces.ensure_lookup_table()
    for fi in range(n_hi_clips, len(bm_hi.faces)):
        bm_hi.faces[fi].material_index = 1
    hi_ob = bmesh_to_object('Hood_Insulation', bm_hi, material=mats['carpet'],
                            smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
    hi_ob.data.materials.append(mats['piano_black'])
    uv_box(hi_ob, scale=3.0)
    hi_ob.parent = hood_lid
    hi_ob.location = Vector((0.0, 0.0, 0.0))
    objs.append(hi_ob)

    # 3. Петли капота и телескопические газовые упоры (слева и справа)
    bx, bz = HOOD_LIFT_BODY_XZ
    lx, lz = hood_lift_lid_local_xz()

    for label, side in (('L', 1.0), ('R', -1.0)):
        hy = side * HOOD_HINGE_Y
        ly = side * HOOD_LIFT_Y

        # 3a. Неподвижный кронштейн петли капота на кузове + ось шарнира
        bm_hm = bmesh.new()
        add_box(bm_hm, -0.042, 0.025, -0.020, 0.020, -0.055, 0.020)
        _add_y_cylinder(bm_hm, 0.011, -0.024, 0.024, seg=14, center_xz=(0.0, 0.0))
        h_mount = bmesh_to_object(f'Hood_Hinge_Mount_{label}', bm_hm,
                                  material=mats['carbon'], smooth=True,
                                  collection=collection)
        uv_box(h_mount, scale=6.0)
        h_mount.location = Vector((HOOD_HINGE_X, hy, hz))
        h_mount.parent = body_ob
        h_mount.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(h_mount)

        # 3b. Изогнутый рычаг петли капота (привязан к Hood_Lid)
        bm_ha = bmesh.new()
        z_pad = (hood_deck_z(HOOD_HINGE_X + 0.24, hy) - HOOD_LID_THICKNESS - 0.008) - hz
        arm_pts = [
            Vector((0.00, hy, 0.00)),
            Vector((-0.035, hy, -0.050)),
            Vector((0.005, hy, -0.100)),
            Vector((0.105, hy, -0.095)),
            Vector((0.235, hy, z_pad)),
        ]
        _sweep_rect_tube(bm_ha, arm_pts, half_y=0.010, half_n=0.012)
        h_arm = bmesh_to_object(f'Hood_Hinge_Arm_{label}', bm_ha,
                                material=mats['carbon'], smooth=True,
                                collection=collection)
        uv_box(h_arm, scale=6.0)
        h_arm.parent = hood_lid
        h_arm.location = Vector((0.0, 0.0, 0.0))
        objs.append(h_arm)

        # 3c. Нижняя опора газового лифта на лонжероне кузова
        bm_lm = bmesh.new()
        add_box(bm_lm, -0.018, 0.018, -0.018, 0.018, -0.018, 0.018)
        hl_mount = bmesh_to_object(f'Hood_Lift_Mount_{label}', bm_lm,
                                   material=mats['carbon'], smooth=True,
                                   collection=collection)
        uv_box(hl_mount, scale=8.0)
        hl_mount.location = Vector((bx, ly, bz))
        hl_mount.parent = body_ob
        hl_mount.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(hl_mount)

        # 3d. Верхний кронштейн газового лифта на внутренней стороне капота
        bm_lb = bmesh.new()
        add_box(bm_lb, -0.022, 0.022, -0.018, 0.018, -0.016, 0.016)
        hl_bracket = bmesh_to_object(f'Hood_Lift_Bracket_{label}', bm_lb,
                                     material=mats['carbon'], smooth=True,
                                     collection=collection)
        uv_box(hl_bracket, scale=8.0)
        hl_bracket.parent = hood_lid
        hl_bracket.location = Vector((lx, ly, lz))
        objs.append(hl_bracket)

        # 3e. Цилиндр газового лифта капота (TRACK_TO верхний кронштейн)
        bm_cyl = bmesh.new()
        _add_z_cylinder(bm_cyl, 0.013, 0.0, HOOD_LIFT_CYL_LEN, seg=18)
        hl_cyl = bmesh_to_object(f'Hood_Lift_Cyl_{label}', bm_cyl,
                                 material=mats['carbon'], smooth=True,
                                 collection=collection)
        uv_box(hl_cyl, scale=6.0)
        hl_cyl.location = Vector((bx, ly, bz))
        hl_cyl.parent = body_ob
        hl_cyl.matrix_parent_inverse = body_ob.matrix_world.inverted()
        con_cyl = hl_cyl.constraints.new('TRACK_TO')
        con_cyl.target = hl_bracket
        con_cyl.track_axis = 'TRACK_Z'
        con_cyl.up_axis = 'UP_Y'
        objs.append(hl_cyl)

        # 3f. Хромированный шток газового лифта капота (TRACK_TO нижняя опора)
        bm_rod = bmesh.new()
        _add_z_cylinder(bm_rod, 0.007, 0.0, HOOD_LIFT_ROD_LEN, seg=16)
        hl_rod = bmesh_to_object(f'Hood_Lift_Rod_{label}', bm_rod,
                                 material=mats['chrome'], smooth=True,
                                 collection=collection)
        uv_box(hl_rod, scale=6.0)
        hl_rod.parent = hood_lid
        hl_rod.location = Vector((lx, ly, lz))
        con_rod = hl_rod.constraints.new('TRACK_TO')
        con_rod.target = hl_mount
        con_rod.track_axis = 'TRACK_Z'
        con_rod.up_axis = 'UP_Y'
        objs.append(hl_rod)

    return hood_lid, objs


# --------------------------------------------------------------------------- #
# 7. Свет, студия (коллекция Studio), мир, камеры
# --------------------------------------------------------------------------- #
def add_light(name, loc, energy, size=0.8, size_y=None, rot=None,
              aim_target=None, color=(1.0, 1.0, 1.0), collection=None,
              hide_direct=False):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.color = color
    if size_y is not None:
        ld.shape = 'RECTANGLE'
        ld.size = size
        ld.size_y = size_y
    else:
        ld.size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = Vector(loc)
    if aim_target is not None:
        look_at(ob, aim_target)
    elif rot is not None:
        ob.rotation_euler = rot
    if hide_direct:
        ob.visible_camera = False
        ob.visible_transmission = False
        ob.visible_glossy = False
    target_col = collection if collection is not None else bpy.context.scene.collection
    target_col.objects.link(ob)
    return ob


def build_volvo_details(body_ob, mats, collection):
    """Строит фирменные элементы экстерьера Volvo S90, точно повторяющие кривизну кузова:
      * вогнутая решётка радиатора с хромированной рамкой, 15 ламелями и эмблемой Iron Mark;
      * контурный нижний воздухозаборник переднего бампера (заподлицо, без выпирания);
      * огибающие фары «Молот Тора» (Thor's Hammer DRL) с линзами;
      * огибающие C-образные задние светодиодные фонари S90;
      * контурный задний диффузор и встроенные двойные трапециевидные выхлопные патрубки;
      * рельефные кромки колёсных арок и тёмные подкрылки."""
    objs = []

    def _surf_x(y, z, front=True):
        if front:
            # Аналитическая поверхность переднего обтекаемого бампера:
            # front_nose_surface_x(y) + плавный верхний скос и смягчённый боковой пояс
            base_x = front_nose_surface_x(y)
            u = clamp((z - 0.22) / 0.50, 0.0, 1.0)
            lip = 0.012 * ((1.0 - u) ** 3.2)
            cove = (
                -0.008 * math.sin(math.pi * clamp((u - 0.12) / 0.34, 0.0, 1.0))
                if 0.12 < u < 0.46 else 0.0
            )
            side_calm = 1.0 - 0.45 * clamp((abs(y) - 0.40) / 0.28, 0.0, 1.0)
            bulge = (
                0.013 * side_calm * math.sin(math.pi * clamp((u - 0.42) / 0.46, 0.0, 1.0))
                if 0.42 < u < 0.88 else 0.0
            )
            slope = -0.022 * (max(0.0, (u - 0.72) / 0.28) ** 1.7)
            return base_x + lip + cove + bulge + slope
        return rear_tail_surface_x(y) - 0.022

    def _add_curved_strip(bm, y0, y1, z0, z1, front=True, off_out=0.008,
                          thick=0.018, ny=9, nz=4, concave=0.0):
        """Добавляет замкнутую изогнутую панель, повторяющую контур переднего/заднего бампера."""
        sgn = 1.0 if front else -1.0
        outer_pts = []
        inner_pts = []
        y_mid = 0.5 * (y0 + y1)
        y_hw = max(0.01, 0.5 * abs(y1 - y0))
        for j in range(nz):
            vz = j / (nz - 1)
            zz = z0 + (z1 - z0) * vz
            row_o = []
            row_i = []
            for i in range(ny):
                vy = i / (ny - 1)
                yy = y0 + (y1 - y0) * vy
                u = (yy - y_mid) / y_hw
                sx = _surf_x(yy, zz, front=front)
                x_o = sx + sgn * (off_out - concave * (1.0 - u * u))
                x_i = x_o - sgn * thick
                row_o.append(Vector((x_o, yy, zz)))
                row_i.append(Vector((x_i, yy, zz)))
            outer_pts.append(row_o)
            inner_pts.append(row_i)
        _add_thick_grid(bm, outer_pts, inner_pts)

    # 1. Фирменная трапециевидная вогнутая решётка радиатора Volvo S90
    # Верх хром-рамки (gz1 + ft = 0.719) плотно примыкает к передней кромке капота (0.724)!
    gw_bot, gw_top = 0.354, 0.384
    gz0, gz1 = 0.452, 0.706
    bm_gr = bmesh.new()
    gr_out, gr_in = [], []
    for j in range(6):
        vz = j / 5.0
        zz = gz0 + (gz1 - gz0) * vz
        gw_loc = gw_bot + (gw_top - gw_bot) * vz
        ro, ri = [], []
        for i in range(15):
            u = -1.0 + 2.0 * i / 14.0
            yy = u * gw_loc
            sx = _surf_x(yy, zz, front=True)
            # Двухосевая вогнутость «водопада» Volvo S90 (по ширине Y и в центре высоты Z)
            conc = 0.008 * (1.0 - u * u) + 0.009 * math.sin(math.pi * vz)
            xo = sx + 0.012 - conc
            ro.append(Vector((xo, yy, zz)))
            ri.append(Vector((xo - 0.028, yy, zz)))
        gr_out.append(ro)
        gr_in.append(ri)
    _add_thick_grid(bm_gr, gr_out, gr_in)
    grille_ob = bmesh_to_object('Grille_Front', bm_gr, material=mats['piano_black'],
                                smooth=True, collection=collection, bevel=0.0020)
    uv_box(grille_ob, scale=4.0)
    objs.append(grille_ob)

    # Хромированная трапециевидная рамка, 17 вогнутых ламелей и двухслойный знак Iron Mark
    bm_gc = bmesh.new()
    ft = 0.013
    # Верхняя и нижняя контурные перекладины рамки (повторяют трапецию решётки)
    _add_curved_strip(bm_gc, -gw_top - ft, gw_top + ft, gz1, gz1 + ft, front=True,
                      off_out=0.018, thick=0.020, ny=13, nz=2)
    _add_curved_strip(bm_gc, -gw_bot - ft, gw_bot + ft, gz0 - ft, gz0, front=True,
                      off_out=0.018, thick=0.020, ny=13, nz=2)
    # Левая и правая наклонные боковые стойки рамки (следуют наклону трапеции gw_bot -> gw_top)
    for sy_g in (1.0, -1.0):
        st_o, st_i = [], []
        for j in range(6):
            vz = j / 5.0
            zz = (gz0 + 0.002) + (gz1 - gz0 - 0.004) * vz
            g_edge = gw_bot + (gw_top - gw_bot) * vz
            ya_s, yb_s = sorted((sy_g * (g_edge + 0.002), sy_g * (g_edge + ft)))
            ro, ri = [], []
            for yy_s in (ya_s, yb_s):
                sx = _surf_x(yy_s, zz, front=True) + 0.018
                ro.append(Vector((sx, yy_s, zz)))
                ri.append(Vector((sx - 0.020, yy_s, zz)))
            st_o.append(ro)
            st_i.append(ri)
        _add_thick_grid(bm_gc, st_o, st_i)

    # Хромированные L-образные рассекатели в боковых воздухозаборниках Volvo S90 Inscription!
    for sy_i in (1.0, -1.0):
        ya_i, yb_i = sorted((sy_i * 0.435, sy_i * 0.655))
        _add_curved_strip(bm_gc, ya_i, yb_i, 0.308, 0.320, front=True,
                          off_out=0.011, thick=0.014, ny=7, nz=2)
        # Короткий вертикальный хром-акцент с внутренней стороны воздухозаборника
        ya_v, yb_v = sorted((sy_i * 0.432, sy_i * 0.446))
        _add_curved_strip(bm_gc, ya_v, yb_v, 0.274, 0.354, front=True,
                          off_out=0.010, thick=0.014, ny=2, nz=4)

    # 17 вогнутых вертикальных хромированных ламелей («водопад» Volvo S90: вогнуты по Z и Y!)
    n_slats = 17
    for i in range(n_slats):
        u = -1.0 + 2.0 * i / (n_slats - 1)
        sl_o, sl_i = [], []
        for j in range(6):
            vz = j / 5.0
            zz = (gz0 + 0.006) + (gz1 - gz0 - 0.012) * vz
            gw_loc = (gw_bot + (gw_top - gw_bot) * vz) - 0.018
            yy = u * gw_loc
            sx = _surf_x(yy, zz, front=True)
            conc = 0.008 * (1.0 - u * u) + 0.010 * math.sin(math.pi * vz)
            xo = sx + 0.018 - conc
            ro = [Vector((xo, yy - 0.0032, zz)), Vector((xo, yy + 0.0032, zz))]
            ri = [Vector((xo - 0.016, yy - 0.0032, zz)), Vector((xo - 0.016, yy + 0.0032, zz))]
            sl_o.append(ro)
            sl_i.append(ri)
        _add_thick_grid(bm_gc, sl_o, sl_i)

    # Гладкая наклонная диагональная полоса Volvo Iron Mark (единая наклонная балка без «лесенки»)
    z_gmid = 0.5 * (gz0 + gz1)
    d_pts = []
    for k in range(9):
        t_m = k / 8.0
        y_m = -0.33 * (1.0 - t_m) + 0.33 * t_m
        z_m = (gz0 + 0.014) * (1.0 - t_m) + (gz1 - 0.014) * t_m
        sx = _surf_x(y_m, z_m, front=True) + 0.017
        d_pts.append(Vector((sx, y_m, z_m)))
    diag_dir = (d_pts[-1] - d_pts[0]).normalized()
    perp_yz = Vector((0.0, -diag_dir.z, diag_dir.y)).normalized()
    d_rings = []
    for p in d_pts:
        d_rings.append([
            bm_gc.verts.new(p - Vector((0.007, 0.0, 0.0)) - perp_yz * 0.0065),
            bm_gc.verts.new(p + Vector((0.007, 0.0, 0.0)) - perp_yz * 0.0065),
            bm_gc.verts.new(p + Vector((0.007, 0.0, 0.0)) + perp_yz * 0.0065),
            bm_gc.verts.new(p - Vector((0.007, 0.0, 0.0)) + perp_yz * 0.0065),
        ])
    for i_d in range(len(d_rings) - 1):
        r0, r1 = d_rings[i_d], d_rings[i_d + 1]
        for k in range(4):
            kn = (k + 1) % 4
            bm_gc.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    bm_gc.faces.new((d_rings[0][0], d_rings[0][3], d_rings[0][2], d_rings[0][1]))
    bm_gc.faces.new((d_rings[-1][0], d_rings[-1][1], d_rings[-1][2], d_rings[-1][3]))

    # Фирменный двухслойный центральный знак Volvo Iron Mark:
    # хромированное кольцо (slot 0) + чёрная центральная плашка (slot 1) + поперечина + стрела
    sx_c = _surf_x(0.0, z_gmid, front=True)
    seg_em = 24
    r_out_em, r_in_em = 0.044, 0.034
    ring_em = []
    for i_e in range(seg_em):
        ang = 2.0 * math.pi * i_e / seg_em
        ca, sa = math.cos(ang), math.sin(ang)
        ring_em.append([
            bm_gc.verts.new(Vector((sx_c + 0.014, r_in_em * ca, z_gmid + r_in_em * sa))),
            bm_gc.verts.new(Vector((sx_c + 0.030, r_in_em * ca, z_gmid + r_in_em * sa))),
            bm_gc.verts.new(Vector((sx_c + 0.030, r_out_em * ca, z_gmid + r_out_em * sa))),
            bm_gc.verts.new(Vector((sx_c + 0.014, r_out_em * ca, z_gmid + r_out_em * sa))),
        ])
    for i_e in range(seg_em):
        r0, r1 = ring_em[i_e], ring_em[(i_e + 1) % seg_em]
        for k in range(4):
            kn = (k + 1) % 4
            bm_gc.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    # Хромированная горизонтальная лента VOLVO поперёк кольца + стрела Марса вверх-вправо
    add_box(bm_gc, sx_c + 0.016, sx_c + 0.031, -0.045, +0.045, z_gmid - 0.0085, z_gmid + 0.0085)
    # Стрела Iron Mark (направлена в +Y/+Z на решётке, спереди зритель видит её вправо-вверх)
    add_box(bm_gc, sx_c + 0.016, sx_c + 0.029, -0.047, -0.027, z_gmid + 0.028, z_gmid + 0.037)
    add_box(bm_gc, sx_c + 0.016, sx_c + 0.029, -0.047, -0.038, z_gmid + 0.017, z_gmid + 0.0265)
    # Утопленная глянцево-чёрная подложка внутри кольца (slot 1 = Piano_Black)
    n_em_blk = len(bm_gc.faces)
    _add_x_cylinder(bm_gc, r_in_em - 0.001, sx_c + 0.013, sx_c + 0.025, seg=24,
                    center_yz=(0.0, z_gmid))
    bm_gc.faces.ensure_lookup_table()
    for fi in range(n_em_blk, len(bm_gc.faces)):
        bm_gc.faces[fi].material_index = 1
    gc_ob = bmesh_to_object('Grille_Chrome', bm_gc, material=mats['chrome'],
                            smooth=True, collection=collection)
    gc_ob.data.materials.append(mats['piano_black'])
    uv_box(gc_ob, scale=6.0)
    objs.append(gc_ob)

    # 2. Контурный нижний воздухозаборник, трапециевидные боковые ниши, LED-ПТФ и передний номер
    bm_in = bmesh.new()
    # Центральная нижняя трапециевидная решётка + 2 горизонтальные аэродинамические ламели
    _add_curved_strip(bm_in, -0.365, 0.365, 0.256, 0.362, front=True,
                      off_out=0.006, thick=0.022, ny=13, nz=4)
    for z_sl in (0.284, 0.328):
        _add_curved_strip(bm_in, -0.355, 0.355, z_sl - 0.0055, z_sl + 0.0055, front=True,
                          off_out=0.012, thick=0.014, ny=11, nz=2)
    # Трапециевидные притопленные боковые воздухозаборники под фарами (повторяют изгиб бампера!)
    for sy_i in (1.0, -1.0):
        bi_o, bi_i = [], []
        for j in range(4):
            vz = j / 3.0
            ro, ri = [], []
            for i in range(8):
                vy = i / 7.0
                # Сужение по высоте к наружному углу бампера (элегантная форма без «кирпича»)
                yy_abs = 0.430 + (0.675 - 0.430) * vy
                z_lo_b = 0.268 + 0.016 * (vy ** 1.5)
                z_hi_b = 0.362 - 0.028 * (vy ** 1.4)
                zz = z_lo_b + (z_hi_b - z_lo_b) * vz
                yy = sy_i * yy_abs
                sx = _surf_x(yy, zz, front=True) + 0.004
                ro.append(Vector((sx, yy, zz)))
                ri.append(Vector((sx - 0.020, yy, zz)))
            if sy_i < 0:
                ro.reverse()
                ri.reverse()
            bi_o.append(ro)
            bi_i.append(ri)
        _add_thick_grid(bm_in, bi_o, bi_i)
    # Аккуратная передняя спортивная губа-сплиттер по нижней кромке бампера
    _add_curved_strip(bm_in, -0.62, 0.62, 0.214, 0.238, front=True,
                      off_out=0.009, thick=0.036, ny=15, nz=2)
    intake_ob = bmesh_to_object('Bumper_Intake_Front', bm_in, material=mats['carbon'],
                                smooth=True, collection=collection, bevel=0.0020)
    uv_box(intake_ob, scale=3.0)
    objs.append(intake_ob)

    # 2b. Узкие светодиодные противотуманки в боковых воздухозаборниках (снимаются с Front_Bumper!)
    for f_lbl, sy_f in (('L', 1.0), ('R', -1.0)):
        bm_fog = bmesh.new()
        ya_f, yb_f = sorted((sy_f * 0.545, sy_f * 0.640))
        _add_curved_strip(bm_fog, ya_f, yb_f, 0.285, 0.298, front=True,
                          off_out=0.011, thick=0.009, ny=5, nz=2)
        fog_ob = bmesh_to_object(f'Fog_Light_{f_lbl}', bm_fog, material=mats['drl'],
                                 smooth=True, collection=collection, bevel=0.0010, bevel_segs=1)
        uv_box(fog_ob, scale=8.0)
        objs.append(fog_ob)

    # 2c. Европейский передний номерной знак на переднем бампере (снимается с Front_Bumper!)
    bm_pl_f = bmesh.new()
    sx_pf = _surf_x(0.0, 0.395, front=True)
    # Чёрная рамка-подиум (slot 1 = Piano_Black)
    add_box(bm_pl_f, sx_pf - 0.006, sx_pf + 0.018, -0.245, +0.245, 0.352, 0.452)
    bm_pl_f.faces.ensure_lookup_table()
    for f_p in bm_pl_f.faces:
        f_p.material_index = 1
    # Белое поле номера (slot 0 = White)
    n_bf_w = len(bm_pl_f.faces)
    add_box(bm_pl_f, sx_pf + 0.016, sx_pf + 0.022, -0.236, +0.236, 0.359, 0.445)
    bm_pl_f.faces.ensure_lookup_table()
    for f_idx in range(n_bf_w, len(bm_pl_f.faces)):
        bm_pl_f.faces[f_idx].material_index = 0
    # Синяя евро-полоска слева спереди (-Y — левый край номера для зрителя, slot 2 = Paint)
    n_bf_eu = len(bm_pl_f.faces)
    add_box(bm_pl_f, sx_pf + 0.018, sx_pf + 0.023, -0.236, -0.192, 0.359, 0.445)
    bm_pl_f.faces.ensure_lookup_table()
    for f_idx in range(n_bf_eu, len(bm_pl_f.faces)):
        bm_pl_f.faces[f_idx].material_index = 2
    pl_f = bmesh_to_object('License_Plate_Front', bm_pl_f, material=mats['white'],
                           smooth=True, collection=collection, bevel=0.0012, bevel_segs=1)
    pl_f.data.materials.append(mats['piano_black'])
    pl_f.data.materials.append(mats['paint'])
    uv_box(pl_f, scale=5.0)
    objs.append(pl_f)

    # 3. Трапециевидные огибающие блок-фары Volvo S90 «Молот Тора» (на Radiator_Support):
    # плотно примыкают к «щёчке» бампера у решётки (|y|=0.402 -> 0 чёрной дыры!),
    # плавно расширяются и затекают назад вдоль крыла до |y|=0.742 со скруглённым силуэтом!
    def _hl_z_bounds(abs_y):
        """Трапециевидный профиль фары Volvo S90: узкий у решётки (0.560..0.712),
        плавно поднимающийся и вытянутый назад к переднему крылу (0.580..0.734)."""
        t = clamp((abs_y - 0.402) / (0.742 - 0.402), 0.0, 1.0)
        # Мягкое скругление самого внутреннего и наружного концов трапеции
        end_round = 0.010 * (max(0.0, (0.14 - t) / 0.14) ** 2.0) + 0.014 * (
            max(0.0, (t - 0.84) / 0.16) ** 2.0
        )
        z_bot = 0.560 + 0.022 * (t ** 1.35) + end_round
        z_top = 0.712 + 0.024 * (t ** 0.85) - end_round * 1.1
        return z_bot, z_top

    def _hl_surf_x(abs_y, vz):
        """Обтекаемая поверхность фары по дуге носа с мягким wrap-around заворотом к крылу."""
        t_wrap = clamp((abs_y - 0.61) / 0.132, 0.0, 1.0)
        return (
            front_nose_surface_x(abs_y)
            - 0.012
            - 0.022 * (vz ** 1.15)
            - 0.036 * (t_wrap ** 1.75)
        )

    for label, sy in (('L', 1.0), ('R', -1.0)):
        y_in, y_out = sy * 0.402, sy * 0.742
        ya, yb = (y_in, y_out) if y_in < y_out else (y_out, y_in)

        # 3a. Трапециевидный глянцево-чёрный корпус блок-фары + 4 би-LED проектора + хром-кант
        bm_hl = bmesh.new()
        outer_h = []
        inner_h = []
        ny_h, nz_h = 15, 5
        for j in range(nz_h):
            vz = j / (nz_h - 1)
            row_o = []
            row_i = []
            for i in range(ny_h):
                vy = i / (ny_h - 1)
                yy = ya + (yb - ya) * vy
                zb_h, zt_h = _hl_z_bounds(abs(yy))
                zz = zb_h + (zt_h - zb_h) * vz
                sx = _hl_surf_x(abs(yy), vz) - 0.004
                row_o.append(Vector((sx, yy, zz)))
                row_i.append(Vector((min(2.240, sx - 0.048), yy * 0.94, zz)))
            outer_h.append(row_o)
            inner_h.append(row_i)
        _add_thick_grid(bm_hl, outer_h, inner_h)

        # Верхняя тонкая хромированная бровь-молдинг внутри фары (slot 1 = Chrome)
        n_bef_brow = len(bm_hl.faces)
        br_o, br_i = [], []
        for jj in range(2):
            ro_b, ri_b = [], []
            for ii in range(11):
                yy_b = ya + 0.010 + (yb - ya - 0.020) * (ii / 10.0)
                zb_h, zt_h = _hl_z_bounds(abs(yy_b))
                zz_b = (zt_h - 0.011) + 0.006 * jj
                vz_b = clamp((zz_b - zb_h) / max(zt_h - zb_h, 0.04), 0.0, 1.0)
                sx_b = _hl_surf_x(abs(yy_b), vz_b) - 0.001
                ro_b.append(Vector((sx_b, yy_b, zz_b)))
                ri_b.append(Vector((sx_b - 0.008, yy_b, zz_b)))
            br_o.append(ro_b)
            br_i.append(ri_b)
        _add_thick_grid(bm_hl, br_o, br_i)
        bm_hl.faces.ensure_lookup_table()
        for fi in range(n_bef_brow, len(bm_hl.faces)):
            bm_hl.faces[fi].material_index = 1

        # Внутренние 3D-проекторы би-LED, выровненные строго вдоль наклонной оси фары!
        # slot 1 = Chrome (бленды), slot 2 = Alloy (линзы)
        for u_p, dz_p in ((0.40, +0.036), (0.64, +0.036), (0.44, -0.035), (0.66, -0.032)):
            yp = y_in + (y_out - y_in) * u_p
            zb_p, zt_p = _hl_z_bounds(abs(yp))
            zp = 0.5 * (zb_p + zt_p) + dz_p
            vz_p = clamp((zp - zb_p) / max(zt_p - zb_p, 0.04), 0.0, 1.0)
            xp_f = _hl_surf_x(abs(yp), vz_p) - 0.001
            seg_p = 16
            r_p = 0.018
            ring_p0 = [
                bm_hl.verts.new(Vector((xp_f - 0.012,
                                        yp + r_p * math.cos(2.0 * math.pi * k / seg_p),
                                        zp + r_p * math.sin(2.0 * math.pi * k / seg_p))))
                for k in range(seg_p)
            ]
            ring_p1 = [
                bm_hl.verts.new(Vector((xp_f,
                                        yp + (r_p * 0.84) * math.cos(2.0 * math.pi * k / seg_p),
                                        zp + (r_p * 0.84) * math.sin(2.0 * math.pi * k / seg_p))))
                for k in range(seg_p)
            ]
            c_p0 = bm_hl.verts.new(Vector((xp_f - 0.012, yp, zp)))
            c_p1 = bm_hl.verts.new(Vector((xp_f + 0.002, yp, zp)))
            for k in range(seg_p):
                kn = (k + 1) % seg_p
                f_side = bm_hl.faces.new((ring_p0[k], ring_p0[kn], ring_p1[kn], ring_p1[k]))
                f_side.material_index = 1
                f_bk = bm_hl.faces.new((c_p0, ring_p0[kn], ring_p0[k]))
                f_bk.material_index = 1
                f_fr = bm_hl.faces.new((c_p1, ring_p1[k], ring_p1[kn]))
                f_fr.material_index = 2
        hl_ob = bmesh_to_object(f'Headlight_{label}', bm_hl, material=mats['piano_black'],
                                smooth=True, collection=collection, bevel=0.0018)
        hl_ob.data.materials.append(mats['chrome'])
        hl_ob.data.materials.append(mats['alloy'])
        uv_box(hl_ob, scale=5.0)
        objs.append(hl_ob)

        # 3b. Изящный наклонный «Молот Тора» Volvo S90 с Y-образным раздвоением у решётки!
        bm_drl = bmesh.new()

        def _drl_bar(u0, u1, dz0_s, dz1_s, h0=0.011, h1=0.011):
            """Строит наклонную криволинейную LED-полосу вдоль оси трапециевидной фары."""
            o_pts, i_pts = [], []
            for jj in range(2):
                ro_d, ri_d = [], []
                for ii in range(8):
                    f = ii / 7.0
                    u_c = u0 + (u1 - u0) * f
                    yy_d = y_in + (y_out - y_in) * u_c
                    zb_d, zt_d = _hl_z_bounds(abs(yy_d))
                    z_c = 0.5 * (zb_d + zt_d) + dz0_s + (dz1_s - dz0_s) * f
                    hh = 0.5 * (h0 + (h1 - h0) * f)
                    zz_d = z_c + (-hh if jj == 0 else +hh)
                    vz_d = clamp((zz_d - zb_d) / max(zt_d - zb_d, 0.04), 0.0, 1.0)
                    sx_d = _hl_surf_x(abs(yy_d), vz_d) + 0.001
                    ro_d.append(Vector((sx_d, yy_d, zz_d)))
                    ri_d.append(Vector((sx_d - 0.007, yy_d, zz_d)))
                if sy < 0:
                    ro_d.reverse()
                    ri_d.reverse()
                o_pts.append(ro_d)
                i_pts.append(ri_d)
            _add_thick_grid(bm_drl, o_pts, i_pts)

        # Длинный сужающийся ствол «Молота Тора» (следует восходящей трапеции к крылу!)
        _drl_bar(0.17, 0.88, -0.002, +0.004, h0=0.011, h1=0.0075)
        # Открытая вперёд фирменная Т-вилка «Молота Тора» Volvo S90 (без закрытого «рупора»!)
        _drl_bar(0.145, 0.195, 0.0, 0.0, h0=0.068, h1=0.068)
        _drl_bar(0.045, 0.185, +0.032, +0.029, h0=0.009, h1=0.009)
        _drl_bar(0.045, 0.185, -0.032, -0.029, h0=0.009, h1=0.009)
        drl_ob = bmesh_to_object(f'Headlight_DRL_{label}', bm_drl,
                                 material=mats['drl'], smooth=True,
                                 collection=collection, bevel=0.0012, bevel_segs=1)
        uv_box(drl_ob, scale=8.0)
        objs.append(drl_ob)

        # 3c. Прозрачный обтекаемый рассеиватель (линза) по точному трапециевидному контуру фары
        bm_ln = bmesh.new()
        ln_out, ln_in = [], []
        for j in range(nz_h):
            vz = j / (nz_h - 1)
            ro, ri = [], []
            for i in range(ny_h):
                vy = i / (ny_h - 1)
                yy = (ya - 0.002) + (yb - ya + 0.004) * vy
                zb_h, zt_h = _hl_z_bounds(abs(yy))
                zz = (zb_h - 0.003) + ((zt_h + 0.003) - (zb_h - 0.003)) * vz
                sx = _hl_surf_x(abs(yy), vz) + 0.006
                ro.append(Vector((sx, yy, zz)))
                ri.append(Vector((sx - 0.005, yy, zz)))
            ln_out.append(ro)
            ln_in.append(ri)
        _add_thick_grid(bm_ln, ln_out, ln_in)
        ln_ob = bmesh_to_object(f'Headlight_Lens_{label}', bm_ln, material=mats['lens'],
                                smooth=True, collection=collection, bevel=0.0012, bevel_segs=1)
        uv_box(ln_ob, scale=5.0)
        objs.append(ln_ob)

    # 4. Огибающие C-образные задние блок-фонари Volvo S90 — установлены строго НА КУЗОВЕ
    # (Rear_Subframe): трапециевидный скруглённый силуэт в точном согласовании с бровью крыла!
    def _tl_z_bounds(abs_y):
        t = clamp((abs_y - TAIL_LIGHT_Y_IN) / max(TAIL_LIGHT_Y_OUT - TAIL_LIGHT_Y_IN, 0.05),
                  0.0, 1.0)
        end_r = 0.008 * (max(0.0, (t - 0.80) / 0.20) ** 2.0)
        z_lo = TAIL_LIGHT_Z_LO + 0.006 * (t ** 1.5) + end_r
        z_hi = TAIL_LIGHT_Z_HI - 0.015 * (t ** 1.35) - end_r
        return z_lo, z_hi

    def _tl_surf_x(abs_y, vz):
        t_wrap = clamp((abs_y - 0.65) / max(TAIL_LIGHT_Y_OUT - 0.65, 0.05), 0.0, 1.0)
        return rear_tail_surface_x(abs_y) + 0.012 * (t_wrap ** 1.75) + 0.006 * (vz ** 1.4)

    def _add_tail_curved_strip(bm, y0, y1, z0, z1, off_out=0.006, thick=0.012, ny=9, nz=4):
        outer_pts = []
        inner_pts = []
        for j in range(nz):
            vz = j / (nz - 1)
            zz = z0 + (z1 - z0) * vz
            row_o = []
            row_i = []
            for i in range(ny):
                vy = i / (ny - 1)
                yy = y0 + (y1 - y0) * vy
                sx = rear_tail_surface_x(yy)
                x_o = sx - off_out
                x_i = x_o + thick
                row_o.append(Vector((x_o, yy, zz)))
                row_i.append(Vector((x_i, yy, zz)))
            outer_pts.append(row_o)
            inner_pts.append(row_i)
        _add_thick_grid(bm, outer_pts, inner_pts)

    for label, sy in (('L', 1.0), ('R', -1.0)):
        y_in, y_out = sy * TAIL_LIGHT_Y_IN, sy * TAIL_LIGHT_Y_OUT
        ya_t, yb_t = (y_in, y_out) if y_in < y_out else (y_out, y_in)

        # 4a. Компактный тёмно-рубиновый корпус заднего блок-фонаря со скруглённым wrap-around
        bm_th = bmesh.new()
        th_out = []
        th_in = []
        ny_t, nz_t = 13, 5
        for j in range(nz_t):
            vz = j / (nz_t - 1)
            row_o = []
            row_i = []
            for i in range(ny_t):
                vy = i / (ny_t - 1)
                yy = ya_t + (yb_t - ya_t) * vy
                zb_t, zt_t = _tl_z_bounds(abs(yy))
                zz = zb_t + (zt_t - zb_t) * vz
                sx = _tl_surf_x(abs(yy), vz) - 0.001
                row_o.append(Vector((sx, yy, zz)))
                row_i.append(Vector((min(-2.300, sx + 0.044), yy, zz)))
            th_out.append(row_o)
            th_in.append(row_i)
        _add_thick_grid(bm_th, th_out, th_in)
        th_ob = bmesh_to_object(f'Tail_Housing_{label}', bm_th, material=mats['tail_housing'],
                                smooth=True, collection=collection, bevel=0.0022)
        uv_box(th_ob, scale=5.0)
        objs.append(th_ob)

        # 4b. Изящные тонкие C-образные светодиодные секции Volvo S90 (повторяют трапецию!)
        bm_tl = bmesh.new()

        def _tl_bar(u0, u1, v_z0, v_z1):
            """Строит тонкую криволинейную LED-полосу в нормированной высоте [v_z0..v_z1]."""
            o_pts, i_pts = [], []
            for jj in range(2):
                vz_c = v_z0 if jj == 0 else v_z1
                ro, ri = [], []
                for ii in range(9):
                    f = ii / 8.0
                    u_c = u0 + (u1 - u0) * f
                    yy = y_in + (y_out - y_in) * u_c
                    zb_t, zt_t = _tl_z_bounds(abs(yy))
                    zz = zb_t + (zt_t - zb_t) * vz_c
                    sx = _tl_surf_x(abs(yy), vz_c) - 0.006
                    ro.append(Vector((sx, yy, zz)))
                    ri.append(Vector((sx + 0.006, yy, zz)))
                if sy < 0:
                    ro.reverse()
                    ri.reverse()
                o_pts.append(ro)
                i_pts.append(ri)
            _add_thick_grid(bm_tl, o_pts, i_pts)

        _tl_bar(0.04, 0.91, 0.84, 0.925)   # тонкая верхняя C-полоса (~16 мм)
        _tl_bar(0.83, 0.91, 0.14, 0.855)   # наружная вертикальная C-стойка на углу
        _tl_bar(0.05, 0.91, 0.065, 0.150)  # тонкая нижняя C-полоса (~16 мм)
        _tl_bar(0.14, 0.74, 0.46, 0.54)    # центральная LED-полоса стоп-сигнала
        # Белая/серебристая полоска секции заднего хода внутри C-фонаря (slot 1 = White)
        n_bef_rev = len(bm_tl.faces)
        _tl_bar(0.14, 0.58, 0.30, 0.375)
        bm_tl.faces.ensure_lookup_table()
        for fi in range(n_bef_rev, len(bm_tl.faces)):
            bm_tl.faces[fi].material_index = 1
        tl_ob = bmesh_to_object(f'Tail_Light_{label}', bm_tl,
                                material=mats['tail'], smooth=True,
                                collection=collection, bevel=0.0012, bevel_segs=1)
        tl_ob.data.materials.append(mats['white'])
        uv_box(tl_ob, scale=6.0)
        objs.append(tl_ob)

        # 4c. Компактная обтекаемая внешняя линза заднего фонаря (чистый 4 мм зазор от LED!)
        bm_tln = bmesh.new()
        tln_out, tln_in = [], []
        for j in range(nz_t):
            vz = j / (nz_t - 1)
            ro, ri = [], []
            for i in range(ny_t):
                vy = i / (ny_t - 1)
                yy = (ya_t - 0.002) + (yb_t - ya_t + 0.004) * vy
                zb_t, zt_t = _tl_z_bounds(abs(yy))
                zz = (zb_t - 0.002) + ((zt_t + 0.002) - (zb_t - 0.002)) * vz
                sx = _tl_surf_x(abs(yy), vz) - 0.013
                ro.append(Vector((sx, yy, zz)))
                ri.append(Vector((sx + 0.005, yy, zz)))
            tln_out.append(ro)
            tln_in.append(ri)
        _add_thick_grid(bm_tln, tln_out, tln_in)
        tln_ob = bmesh_to_object(f'Tail_Lens_{label}', bm_tln, material=mats['lens'],
                                 smooth=True, collection=collection, bevel=0.0012)
        uv_box(tln_ob, scale=5.0)
        objs.append(tln_ob)

    # 5. Задний диффузор (Rear_Diffuser), патрубки (Exhaust_L/R) и катафоты (Rear_Reflector_L/R)
    bm_rd = bmesh.new()
    _add_tail_curved_strip(bm_rd, -0.58, 0.58, 0.245, 0.355,
                           off_out=0.016, thick=0.028, ny=15, nz=4)
    rd_ob = bmesh_to_object('Rear_Diffuser', bm_rd, material=mats['carbon'],
                            smooth=True, collection=collection, bevel=0.0025)
    uv_box(rd_ob, scale=3.0)
    objs.append(rd_ob)

    for label, sy in (('L', 1.0), ('R', -1.0)):
        ey = sy * 0.44
        ex_s = rear_tail_surface_x(ey) - 0.014
        bm_ex = bmesh.new()
        # Трапециевидный хромированный патрубок с центральной перегородкой (slot 0 = Chrome)
        add_box(bm_ex, ex_s - 0.014, ex_s + 0.040, ey - 0.072, ey + 0.072, 0.316, 0.330)
        add_box(bm_ex, ex_s - 0.014, ex_s + 0.040, ey - 0.064, ey + 0.064, 0.262, 0.276)
        add_box(bm_ex, ex_s - 0.014, ex_s + 0.040, ey - 0.074, ey - 0.060, 0.278, 0.314)
        add_box(bm_ex, ex_s - 0.014, ex_s + 0.040, ey + 0.060, ey + 0.074, 0.278, 0.314)
        # Внутренняя тёмная выхлопная труба и глубина канала (slot 1 = Carbon)
        n_ex_chr = len(bm_ex.faces)
        add_box(bm_ex, ex_s + 0.012, ex_s + 0.042, ey - 0.056, ey + 0.056, 0.278, 0.314)
        _add_x_cylinder(bm_ex, 0.019, ex_s - 0.002, ex_s + 0.022, seg=18,
                        center_yz=(ey, 0.296))
        bm_ex.faces.ensure_lookup_table()
        for fi in range(n_ex_chr, len(bm_ex.faces)):
            bm_ex.faces[fi].material_index = 1
        ex_ob = bmesh_to_object(f'Exhaust_{label}', bm_ex, material=mats['chrome'],
                                smooth=True, collection=collection, bevel=0.0020)
        ex_ob.data.materials.append(mats['carbon'])
        uv_box(ex_ob, scale=8.0)
        objs.append(ex_ob)

        # Горизонтальный красный светоотражатель (катафот) заподлицо в тёмной рамке-нише бампера
        ya_r, yb_r = sorted((sy * 0.46, sy * 0.64))
        bm_ref = bmesh.new()
        _add_tail_curved_strip(bm_ref, ya_r, yb_r, 0.434, 0.458,
                               off_out=0.0255, thick=0.014, ny=7, nz=2)
        n_ref_red = len(bm_ref.faces)
        _add_tail_curved_strip(bm_ref, ya_r - 0.006, yb_r + 0.006, 0.430, 0.462,
                               off_out=0.0230, thick=0.016, ny=7, nz=2)
        bm_ref.faces.ensure_lookup_table()
        for fi in range(n_ref_red, len(bm_ref.faces)):
            bm_ref.faces[fi].material_index = 1
        ref_ob = bmesh_to_object(f'Rear_Reflector_{label}', bm_ref, material=mats['caliper'],
                                 smooth=True, collection=collection, bevel=0.0015, bevel_segs=1)
        ref_ob.data.materials.append(mats['carbon'])
        uv_box(ref_ob, scale=8.0)
        objs.append(ref_ob)

    # 5a. Мелкие заводские детали бамперов: датчики парктроника (PDC) и крышки омывателей фар
    bm_pdc_f = bmesh.new()
    for py_s in (-0.54, -0.26, +0.26, +0.54):
        sx_s = _surf_x(py_s, 0.465, front=True)
        # Тонкое чёрное уплотнительное кольцо + датчик заподлицо в бампере (~1.5 мм)
        _add_x_cylinder(bm_pdc_f, 0.0110, sx_s - 0.005, sx_s + 0.0018, seg=14,
                        center_yz=(py_s, 0.465))
    n_pdc_f_ring = len(bm_pdc_f.faces)
    for py_s in (-0.54, -0.26, +0.26, +0.54):
        sx_s = _surf_x(py_s, 0.465, front=True)
        _add_x_cylinder(bm_pdc_f, 0.0085, sx_s - 0.004, sx_s + 0.0026, seg=14,
                        center_yz=(py_s, 0.465))
    bm_pdc_f.faces.ensure_lookup_table()
    for fi in range(n_pdc_f_ring, len(bm_pdc_f.faces)):
        bm_pdc_f.faces[fi].material_index = 1
    pdc_f_ob = bmesh_to_object('PDC_Sensors_Front', bm_pdc_f, material=mats['piano_black'],
                               smooth=True, collection=collection)
    pdc_f_ob.data.materials.append(mats['bumper'])
    uv_box(pdc_f_ob, scale=12.0)
    objs.append(pdc_f_ob)

    # Крышки омывателей фар (Headlight_Washers) заподлицо на переднем бампере прямо под фарами
    bm_hw = bmesh.new()
    for sy_w in (-1.0, +1.0):
        ya_w, yb_w = sorted((sy_w * 0.50, sy_w * 0.585))
        _add_curved_strip(bm_hw, ya_w, yb_w, 0.492, 0.520, front=True,
                          off_out=0.0032, thick=0.008, ny=5, nz=2)
    hw_ob = bmesh_to_object('Headlight_Washers', bm_hw, material=mats['bumper'],
                            smooth=True, collection=collection, bevel=0.0012, bevel_segs=1)
    uv_box(hw_ob, scale=10.0)
    objs.append(hw_ob)

    # Задние датчики парктроника (PDC_Sensors_Rear) — строго ЗАПОДЛИЦО с Rear_Bumper (~1.5 мм!)
    bm_pdc_r = bmesh.new()
    for py_s in (-0.54, -0.22, +0.22, +0.54):
        sx_wall = rear_tail_surface_x(py_s) - 0.0305
        _add_x_cylinder(bm_pdc_r, 0.0110, sx_wall - 0.0015, sx_wall + 0.006, seg=14,
                        center_yz=(py_s, 0.495))
    n_pdc_r_ring = len(bm_pdc_r.faces)
    for py_s in (-0.54, -0.22, +0.22, +0.54):
        sx_wall = rear_tail_surface_x(py_s) - 0.0305
        _add_x_cylinder(bm_pdc_r, 0.0085, sx_wall - 0.0022, sx_wall + 0.005, seg=14,
                        center_yz=(py_s, 0.495))
    bm_pdc_r.faces.ensure_lookup_table()
    for fi in range(n_pdc_r_ring, len(bm_pdc_r.faces)):
        bm_pdc_r.faces[fi].material_index = 1
    pdc_r_ob = bmesh_to_object('PDC_Sensors_Rear', bm_pdc_r, material=mats['piano_black'],
                               smooth=True, collection=collection)
    pdc_r_ob.data.materials.append(mats['bumper'])
    uv_box(pdc_r_ob, scale=12.0)
    objs.append(pdc_r_ob)

    # Широко расставленная хромированная надпись V O L V O ВПЛОТНУЮ к краске крышки багажника
    # (выступ ровно 3 мм, 16-гранные овальные 'O' — 0 парящих теней и чёткая типографика!)
    bm_tb = bmesh.new()
    z_tb = 0.922
    for y_ch, kind in ((+0.20, 'V'), (+0.10, 'O'), (0.0, 'L'), (-0.10, 'V'), (-0.20, 'O')):
        sx_surf = rear_tail_surface_x(y_ch) - 0.003
        x0_c, x1_c = sx_surf - 0.0032, sx_surf + 0.0040
        if kind == 'O':
            seg_o = 16
            ry_out, rz_out = 0.0125, 0.0130
            ry_in, rz_in = 0.0085, 0.0090
            ring_o = []
            for ko in range(seg_o):
                ang = 2.0 * math.pi * ko / seg_o
                ca, sa = math.cos(ang), math.sin(ang)
                ring_o.append([
                    bm_tb.verts.new(Vector((x0_c, y_ch + ry_in * ca, z_tb + rz_in * sa))),
                    bm_tb.verts.new(Vector((x1_c, y_ch + ry_in * ca, z_tb + rz_in * sa))),
                    bm_tb.verts.new(Vector((x1_c, y_ch + ry_out * ca, z_tb + rz_out * sa))),
                    bm_tb.verts.new(Vector((x0_c, y_ch + ry_out * ca, z_tb + rz_out * sa))),
                ])
            for ko in range(seg_o):
                r0, r1 = ring_o[ko], ring_o[(ko + 1) % seg_o]
                for kk in range(4):
                    kn = (kk + 1) % 4
                    bm_tb.faces.new((r0[kk], r0[kn], r1[kn], r1[kk]))
        elif kind == 'L':
            add_box(bm_tb, x0_c, x1_c, y_ch + 0.006, y_ch + 0.0105, z_tb - 0.013, z_tb + 0.013)
            add_box(bm_tb, x0_c, x1_c, y_ch - 0.0105, y_ch + 0.0055, z_tb - 0.013, z_tb - 0.0088)
        else:  # 'V'
            x_mid_v = 0.5 * (x0_c + x1_c)
            p_l = [Vector((x_mid_v, y_ch + 0.0115, z_tb + 0.013)),
                   Vector((x_mid_v, y_ch + 0.0012, z_tb - 0.013))]
            p_r = [Vector((x_mid_v, y_ch - 0.0115, z_tb + 0.013)),
                   Vector((x_mid_v, y_ch - 0.0012, z_tb - 0.013))]
            _sweep_rect_tube(bm_tb, p_l, half_y=0.0022, half_n=0.0036)
            _sweep_rect_tube(bm_tb, p_r, half_y=0.0022, half_n=0.0036)
    # Чёткая криволинейная хромированная планка-ручка над нишей номерного знака (без subsurf!)
    _add_tail_curved_strip(bm_tb, -0.255, +0.255, 0.876, 0.886,
                           off_out=0.0075, thick=0.010, ny=13, nz=2)
    tb_ob = bmesh_to_object('Trunk_Badge', bm_tb, material=mats['chrome'],
                            smooth=True, collection=collection, bevel=0.0006, bevel_segs=1)
    uv_box(tb_ob, scale=15.0)
    objs.append(tb_ob)

    # 5b. Контрастные накладки порогов (Side_Skirt_L/R, в тон съёмным антрацитовым бамперам):
    # плавное сужение концов у арок — органично вписаны в нижнюю линию кузова без торчащих торцов
    for label, side in (('L', 1.0), ('R', -1.0)):
        bm_sk = bmesh.new()
        sk_out, sk_in = [], []
        n_sk = 19
        for i in range(n_sk):
            u_x = i / float(n_sk - 1)
            x = -1.060 + (1.190 - (-1.060)) * u_x
            # Плавное сужение концов накладки вровень с нижней кромкой арок
            end_fade = math.sin(math.pi * clamp(u_x * 6.0, 0.0, 0.5)) * math.sin(
                math.pi * clamp((1.0 - u_x) * 6.0, 0.0, 0.5)
            )
            z_lo = ZB(x) + 0.018 - 0.008 * end_fade
            z_hi = CABIN_SILL_Z - 0.004
            ro, ri = [], []
            for j in range(4):
                u = j / 3.0
                zz = z_lo + (z_hi - z_lo) * u
                flare = 0.004 * math.sin(math.pi * u) * end_fade
                yy = side_surface_y(x, zz) + flare
                ro.append(Vector((x, side * yy, zz)))
                ri.append(Vector((x, side * (yy - 0.026), zz)))
            sk_out.append(ro)
            sk_in.append(ri)
        _add_thick_grid(bm_sk, sk_out, sk_in)
        sk_ob = bmesh_to_object(f'Side_Skirt_{label}', bm_sk, material=mats['bumper'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                bevel=0.0025, subsurf=1)
        uv_box(sk_ob, scale=2.0)
        objs.append(sk_ob)

    # 6. Рельефные кромки колёсных арок (Wheel_Arch_*) и тёмные подкрылки (Wheel_Well_*)
    for tag, axle, side in (('FL', AXLE_F, 1.0), ('FR', AXLE_F, -1.0),
                            ('RL', AXLE_R, 1.0), ('RR', AXLE_R, -1.0)):
        # 6a. Тёмный полуцилиндрический подкрылок за шиной (убирает синий фон внутри арки)
        bm_ww = bmesh.new()
        r_in_w = WHEEL_R + 0.020
        r_out_w = WHEEL_R + 0.042
        y_inner = side * (WHEEL_Y - WHEEL_WIDTH * 0.60)
        y_outer = side * (WHEEL_Y + WHEEL_WIDTH * 0.42)
        ya, yb = (y_inner, y_outer) if y_inner < y_outer else (y_outer, y_inner)
        seg_a = 32
        rings_ww = []
        for i in range(seg_a + 1):
            ang = math.radians(-12.0 + 204.0 * i / seg_a)
            ca, sa = math.cos(ang), math.sin(ang)
            rings_ww.append([
                bm_ww.verts.new(Vector((axle + r_in_w * ca, ya, WHEEL_Z + r_in_w * sa))),
                bm_ww.verts.new(Vector((axle + r_out_w * ca, ya, WHEEL_Z + r_out_w * sa))),
                bm_ww.verts.new(Vector((axle + r_out_w * ca, yb, WHEEL_Z + r_out_w * sa))),
                bm_ww.verts.new(Vector((axle + r_in_w * ca, yb, WHEEL_Z + r_in_w * sa))),
            ])
        for i in range(seg_a):
            r0, r1 = rings_ww[i], rings_ww[i + 1]
            for k in range(4):
                kn = (k + 1) % 4
                bm_ww.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        bm_ww.faces.new((rings_ww[0][0], rings_ww[0][3], rings_ww[0][2], rings_ww[0][1]))
        bm_ww.faces.new((rings_ww[-1][0], rings_ww[-1][1], rings_ww[-1][2], rings_ww[-1][3]))
        ww_ob = bmesh_to_object(f'Wheel_Well_{tag}', bm_ww, material=mats['carbon'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0)
        uv_box(ww_ob, scale=3.0)
        objs.append(ww_ob)

        # 6b. Скруглённая скульптурная бровь колёсной арки в цвет кузова (6-точечный профиль)
        # Ограничена линией шва крыло <-> бампер, чтобы синяя кромка крыла НЕ свисала
        # «клыком» поверх съёмных антрацитовых бамперов!
        bm_wa = bmesh.new()
        r_lip_in = WHEEL_R + 0.024
        r_lip_mid = WHEEL_R + 0.044
        r_lip_out = WHEEL_R + 0.068
        ang_start_deg, ang_end_deg = (8.0, 188.0) if tag.startswith('F') else (-8.0, 164.0)
        rings_wa = []
        for i in range(seg_a + 1):
            ang = math.radians(ang_start_deg + (ang_end_deg - ang_start_deg) * i / seg_a)
            ca, sa = math.cos(ang), math.sin(ang)
            x_in, z_in = axle + r_lip_in * ca, WHEEL_Z + r_lip_in * sa
            x_mid, z_mid = axle + r_lip_mid * ca, WHEEL_Z + r_lip_mid * sa
            x_out, z_out = axle + r_lip_out * ca, WHEEL_Z + r_lip_out * sa
            y_surf = side_surface_y(x_in, max(ZB(x_in) + 0.04, z_in))
            # Профиль с плавным переходом в крыло снаружи и скруглением кромки арки
            pts_sec = [
                (x_in, side * (y_surf - 0.044), z_in),
                (x_out, side * (y_surf - 0.044), z_out),
                (x_out, side * (y_surf + 0.002), z_out),
                (x_mid, side * (y_surf + 0.011), z_mid),
                (x_in, side * (y_surf + 0.006), z_in),
                (x_in, side * (y_surf - 0.012), z_in),
            ]
            if side < 0:
                pts_sec.reverse()
            rings_wa.append([bm_wa.verts.new(Vector(p)) for p in pts_sec])
        n_sec_wa = len(rings_wa[0])
        for i in range(seg_a):
            r0, r1 = rings_wa[i], rings_wa[i + 1]
            for k in range(n_sec_wa):
                kn = (k + 1) % n_sec_wa
                bm_wa.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
        # Замыкаем торцы веером из центра -> чистые треугольники (0 n-gon)
        for end_ring, flip in ((rings_wa[0], True), (rings_wa[-1], False)):
            c_end = sum((v.co for v in end_ring), Vector()) / float(n_sec_wa)
            vc_end = bm_wa.verts.new(c_end)
            for k in range(n_sec_wa):
                kn = (k + 1) % n_sec_wa
                if flip:
                    bm_wa.faces.new((vc_end, end_ring[kn], end_ring[k]))
                else:
                    bm_wa.faces.new((vc_end, end_ring[k], end_ring[kn]))
        wa_ob = bmesh_to_object(f'Wheel_Arch_{tag}', bm_wa, material=mats['paint'],
                                smooth=True, collection=collection, auto_smooth_deg=55.0,
                                subsurf=1)
        uv_box(wa_ob, scale=3.0)
        objs.append(wa_ob)

    return objs


def build_lighting(mats, studio_col):
    # Интерьер: потолочные лампы смотрят строго вниз (-Z); hide_direct=True убирает
    # видимые прямоугольники ламп в лобовом и боковых стёклах на экстерьерных рендерах!
    add_light('Int_RoofLight_F', (0.35, 0.0, 1.30), 3.4, size=0.40,
              rot=(0.0, 0.0, 0.0), color=(1.0, 0.98, 0.95),
              collection=studio_col, hide_direct=True)
    add_light('Int_RoofLight_M', (-0.45, 0.0, 1.30), 3.2, size=0.40,
              rot=(0.0, 0.0, 0.0), color=(1.0, 0.98, 0.95),
              collection=studio_col, hide_direct=True)
    add_light('Int_SeatLight', (0.05, 0.25, 0.85), 2.4, size=0.28,
              aim_target=(0.35, 0.0, 0.55), color=(0.98, 0.98, 1.0),
              collection=studio_col, hide_direct=True)
    add_light('Int_DashLight', (0.35, 0.0, 1.08), 2.2, size=0.35,
              aim_target=(0.75, 0.0, 0.78), color=(0.98, 0.99, 1.0),
              collection=studio_col, hide_direct=True)
    add_light('Int_Ceiling', (-0.05, 0.0, 1.28), 1.4, size=0.45,
              rot=(0.0, 0.0, 0.0), collection=studio_col, hide_direct=True)
    add_light('Int_TrunkLight', (-1.78, 0.0, 1.02), 3.2, size=0.32,
              rot=(0.0, 0.0, 0.0), collection=studio_col, hide_direct=True)
    add_light('Int_EngineLight', (1.65, 0.0, 1.12), 6.5, size=0.45,
              rot=(0.0, 0.0, 0.0), collection=studio_col, hide_direct=True)

    # Автомобильная фотостудия: длинные мягкие софтбоксы-ленты (стрипы) для плавных бликов
    # Верхний продольный софтбокс — формирует цельную бликовую линию по крыше, плечу и капоту
    add_light('Studio_Overhead', (0.15, 0.0, 3.85), 420.0, size=5.8, size_y=1.65,
              rot=(0.0, 0.0, 0.0), color=(0.98, 0.99, 1.0), collection=studio_col)
    # Два длинных боковых софтбокса-стрипа (чистый горизонтальный градиент по дверям и крыльям)
    add_light('Studio_Strip_R', (0.15, -5.2, 2.05), 340.0, size=6.2, size_y=0.95,
              aim_target=(0.15, 0.0, 0.72), color=(0.97, 0.99, 1.0), collection=studio_col)
    add_light('Studio_Strip_L', (0.15, +5.4, 2.25), 290.0, size=6.2, size_y=1.05,
              aim_target=(0.15, 0.0, 0.75), color=(0.94, 0.97, 1.0), collection=studio_col)
    # Передний ключевой тёпло-нейтральный софтбокс (подчёркивает решётку, фары и моторный отсек)
    add_light('Studio_Key_Front', (5.4, -3.4, 2.45), 280.0, size=2.6, size_y=1.35,
              aim_target=(1.40, 0.0, 0.65), color=(1.0, 0.98, 0.95), collection=studio_col)
    # Задний контровой софтбокс (выделяет силуэт фастбэка, C-фонари и диффузор)
    add_light('Studio_Rim_Rear', (-5.4, -3.0, 2.35), 240.0, size=2.6, size_y=1.25,
              aim_target=(-1.35, 0.0, 0.72), color=(0.95, 0.97, 1.0), collection=studio_col)
    # Мягкий низкий заполняющий свет со стороны колёс (читаемые спицы дисков и тормоза)
    add_light('Studio_Wheel_Fill', (0.15, -6.5, 0.75), 110.0, size=5.2, size_y=0.60,
              aim_target=(0.15, 0.0, 0.36), color=(0.98, 0.98, 1.0), collection=studio_col)

    # Студийный сатиновый пол с мягким отражением (замкнутая плита под Z = 0)
    _make_box_object('Cyclorama', (-18.0, 18.0, -18.0, 18.0, -0.04, 0.0),
                     mats['floor'], studio_col, uv_scale=0.1)

    # Мир + HDRI (или процедурный студийный градиентный купол для реалистичных отражений)
    world = bpy.data.worlds.new('World_Studio')
    bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    bg = nt.nodes.get('Background')
    hdri_path = os.path.join(HERE, 'hdri', 'studio_small_09.hdr')
    if os.path.exists(hdri_path):
        env = nt.nodes.new('ShaderNodeTexEnvironment')
        env.image = bpy.data.images.load(hdri_path)
        nt.links.new(env.outputs['Color'], bg.inputs['Color'])
        bg.inputs['Strength'].default_value = 1.0
    else:
        tc = nt.nodes.new('ShaderNodeTexCoord')
        sep = nt.nodes.new('ShaderNodeSeparateXYZ')
        mrange = nt.nodes.new('ShaderNodeMapRange')
        mrange.inputs['From Min'].default_value = -0.15
        mrange.inputs['From Max'].default_value = 0.85
        ramp = nt.nodes.new('ShaderNodeValToRGB')
        # Мягкий студийный градиент: глубокий низ -> светлая лента горизонта -> мягкий потолок
        ramp.color_ramp.elements[0].position = 0.00
        ramp.color_ramp.elements[0].color = (0.045, 0.050, 0.060, 1.0)
        e_hor = ramp.color_ramp.elements.new(0.24)
        e_hor.color = (0.24, 0.26, 0.30, 1.0)
        e_mid = ramp.color_ramp.elements.new(0.45)
        e_mid.color = (0.11, 0.125, 0.15, 1.0)
        ramp.color_ramp.elements[1].position = 0.92
        ramp.color_ramp.elements[1].color = (0.34, 0.36, 0.40, 1.0)
        nt.links.new(tc.outputs['Generated'], sep.inputs['Vector'])
        nt.links.new(sep.outputs['Z'], mrange.inputs['Value'])
        nt.links.new(mrange.outputs['Result'], ramp.inputs['Fac'])
        nt.links.new(ramp.outputs['Color'], bg.inputs['Color'])
        bg.inputs['Strength'].default_value = 1.15


def build_cameras(studio_col):
    cams = {
        'Cam_Exterior': (Vector((6.05, -5.15, 1.66)), 62.0, TARGET),
        'Cam_Rear': (Vector((-5.90, -4.70, 1.90)), 55.0, TARGET),
        'Cam_Side': (Vector((0.10, -10.20, 1.30)), 68.0, TARGET),
        'Cam_Interior': (Vector((-0.85, -0.25, 1.16)), 20.0, Vector((0.85, 0.10, 0.82))),
        'Cam_Interior2': (Vector((0.45, -0.45, 1.15)), 22.0, Vector((0.75, 0.35, 0.85))),
        'Cam_Trunk': (Vector((-3.75, -1.75, 1.85)), 46.0, Vector((-1.72, 0.0, 0.96))),
        'Cam_Door_Cutout': (Vector((-1.35, -3.55, 1.38)), 42.0, Vector((0.02, 0.0, 0.80))),
        'Cam_Hood': (Vector((3.75, -1.85, 1.85)), 46.0, Vector((1.62, 0.0, 0.92))),
    }
    objs = {}
    for name, (loc, lens, aim) in cams.items():
        camd = bpy.data.cameras.new(name)
        camd.lens = lens
        camd.sensor_width = 36.0
        ob = bpy.data.objects.new(name, camd)
        ob.location = loc
        look_at(ob, aim)
        studio_col.objects.link(ob)
        objs[name] = ob
    return objs


# --------------------------------------------------------------------------- #
# 8. Анимация
# --------------------------------------------------------------------------- #
def animate(wheels, steering, body_ob, door_objs=None, trunk_lid=None, hood_lid=None):
    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 240
    scene.render.fps = 24

    # Колёса (диски привязаны к шинам, вращаются вместе с ними) — 40 оборотов вокруг Y
    total = 40.0 * 2.0 * math.pi
    for wh in wheels:
        key_linear(wh, 'rotation_euler', 1, 1, 0.0)
        key_linear(wh, 'rotation_euler', 1, 240, total)
        linearize(wh)

    # Руль ±17° вокруг своей локальной оси X
    if steering is not None:
        ang = math.radians(17.0)
        for frame, val in ((1, 0.0), (60, ang), (120, 0.0), (180, -ang), (240, 0.0)):
            key_linear(steering, 'rotation_euler', 0, frame, val)
        linearize(steering)

    # Крен кузова ±0.8° вокруг продольной оси X
    roll = math.radians(0.8)
    for frame, val in ((1, 0.0), (60, roll), (120, 0.0), (180, -roll), (240, 0.0)):
        key_linear(body_ob, 'rotation_euler', 0, frame, val)
    linearize(body_ob)

    # Двери: открываются наружу по углу из DOORS_SPEC (кадры 45..200), закрываются к 240-му
    if door_objs:
        angle_by_name = {spec[0]: math.radians(spec[4]) for spec in DOORS_SPEC}
        for ob in door_objs:
            ang = angle_by_name.get(ob.name, math.radians(-70.0))
            for frame, val in ((1, 0.0), (45, ang), (200, ang), (240, 0.0)):
                key_linear(ob, 'rotation_euler', 2, frame, val)
            linearize(ob)

    # Крышка багажника: открывается вверх (+Y) на TRUNK_OPEN_DEG (кадры 45..200)
    if trunk_lid is not None:
        trunk_ang = math.radians(TRUNK_OPEN_DEG)
        for frame, val in ((1, 0.0), (45, trunk_ang), (200, trunk_ang), (240, 0.0)):
            key_linear(trunk_lid, 'rotation_euler', 1, frame, val)
        linearize(trunk_lid)

    # Капот: открывается вверх (-Y) на HOOD_OPEN_DEG (кадры 45..200)
    if hood_lid is not None:
        hood_ang = math.radians(HOOD_OPEN_DEG)
        for frame, val in ((1, 0.0), (45, hood_ang), (200, hood_ang), (240, 0.0)):
            key_linear(hood_lid, 'rotation_euler', 1, frame, val)
        linearize(hood_lid)

    # Возвращаем сцену в первый кадр (нейтральное положение) для статичных рендеров
    scene.frame_set(1)


# --------------------------------------------------------------------------- #
# 9. Рендер-настройки и сохранение
# --------------------------------------------------------------------------- #
def setup_render(samples=48):
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = samples
    scene.cycles.max_bounces = 10
    scene.cycles.glossy_bounces = 6
    scene.cycles.transmission_bounces = 6
    scene.cycles.transparent_max_bounces = 48  # обязательно (иначе муар на стёклах)
    scene.cycles.blur_glossy = 1.0             # мягкое подавление шума без замыливания бликов
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = 'OPENIMAGEDENOISE'
    scene.render.resolution_x = 1440
    scene.render.resolution_y = 810
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.exposure = 0.15
    try:
        scene.view_settings.look = 'AgX - Medium High Contrast'
    except Exception as exc:
        print(f'предупреждение: не удалось выставить AgX look: {exc}')


def render_stills(cams, out_dir):
    renders_dir = os.path.join(out_dir, 'renders')
    os.makedirs(renders_dir, exist_ok=True)
    for name in ('Cam_Exterior', 'Cam_Rear', 'Cam_Side', 'Cam_Interior', 'Cam_Interior2'):
        ob = cams.get(name)
        if ob is None:
            continue
        bpy.context.scene.camera = ob
        bpy.context.scene.render.filepath = os.path.abspath(
            os.path.join(renders_dir, name + '.png'))
        bpy.ops.render.render(write_still=True)
        print('rendered', name)


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Генератор сцены автомобиля для Blender 4.2')
    parser.add_argument('--render', action='store_true',
                        help='Отрендерить 5 финальных кадров после сборки')
    parser.add_argument('--samples', type=int, default=48,
                        help='Число сэмплов Cycles (по умолчанию 48)')
    parser.add_argument('--out-dir', default='.',
                        help='Директория сохранения car_realistic.blend и renders/')
    return parser.parse_args(argv)


# --------------------------------------------------------------------------- #
# 10. Главная точка входа
# --------------------------------------------------------------------------- #
def main(argv=None):
    args = parse_args(argv or [])
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # Полная очистка сцены (без использования оператора select_all, безопаснее в headless bpy)
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    for col in list(bpy.data.collections):
        bpy.data.collections.remove(col)
    for datablock in (bpy.data.meshes, bpy.data.materials, bpy.data.lights,
                      bpy.data.cameras, bpy.data.worlds, bpy.data.actions):
        for item in list(datablock):
            if item.users == 0:
                datablock.remove(item)

    car_col = get_or_create_collection(CAR_COLLECTION)
    studio_col = get_or_create_collection(STUDIO_COLLECTION)

    mats = make_materials()
    body_ob = build_body(mats, car_col)
    # Отделённые передняя и задняя части: силовые каркасы, крылья и съёмные бамперы
    front_bumper, _front_objs = build_front_structure_and_panels(body_ob, mats, car_col)
    rear_bumper, _rear_objs = build_rear_structure_and_panels(body_ob, mats, car_col)
    bpy.context.view_layer.update()
    _roof_ob, _pillar_objs = build_roof_and_pillars(body_ob, mats, car_col)
    glass_ob = build_glass(mats, car_col)
    glass_ob.parent = body_ob
    glass_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()

    wheels = []
    wheel_specs = (
        ('FL', AXLE_F, +1),
        ('FR', AXLE_F, -1),
        ('RL', AXLE_R, +1),
        ('RR', AXLE_R, -1),
    )
    for tag, axle, side in wheel_specs:
        tire, _rim, _disc, _caliper = build_wheel(
            tag, axle, side, mats, car_col, body_ob=body_ob)
        wheels.append(tire)

    bpy.context.view_layer.update()

    # Двери (привязаны к кузову)
    door_objs = build_doors(body_ob, mats, car_col)

    # Фирменные детали экстерьера Volvo S90 (решётка, «Молот Тора», C-фонари, диффузор, арки)
    detail_objs = build_volvo_details(body_ob, mats, car_col)

    # Отделённый багажник: крышка, коврик ниши, петли («гусиная шея») и газовые лифты
    trunk_lid, _trunk_objs = build_trunk(body_ob, mats, car_col)

    # Отделённый капот: крышка с силовой рамой, моторный отсек, петли и газовые лифты
    hood_lid, _hood_objs = build_hood(body_ob, mats, car_col)

    interior_objs = build_interior(mats, car_col, door_objs=door_objs)
    build_lighting(mats, studio_col)
    cams = build_cameras(studio_col)
    bpy.context.scene.camera = cams['Cam_Exterior']

    # Привязка деталей по заводской схеме:
    # * решётка и передний воздухозаборник -> Front_Bumper;
    # * задний диффузор и выхлопные патрубки -> Rear_Bumper;
    # * кромки арок -> соответствующие отдельные передние и задние крылья;
    # * передние фары -> Radiator_Support (кузов), задние фонари -> Rear_Subframe (кузов).
    fender_fl = bpy.data.objects.get('Fender_Front_L')
    fender_fr = bpy.data.objects.get('Fender_Front_R')
    fender_rl = bpy.data.objects.get('Fender_Rear_L')
    fender_rr = bpy.data.objects.get('Fender_Rear_R')
    rad_sup = bpy.data.objects.get('Radiator_Support') or body_ob
    rear_sf = bpy.data.objects.get('Rear_Subframe') or body_ob
    parent_map = {
        'Grille_Front': front_bumper,
        'Grille_Chrome': front_bumper,
        'Bumper_Intake_Front': front_bumper,
        'Fog_Light_L': front_bumper,
        'Fog_Light_R': front_bumper,
        'License_Plate_Front': front_bumper,
        'PDC_Sensors_Front': front_bumper,
        'Headlight_Washers': front_bumper,
        'Rear_Diffuser': rear_bumper,
        'Exhaust_L': rear_bumper,
        'Exhaust_R': rear_bumper,
        'Rear_Reflector_L': rear_bumper,
        'Rear_Reflector_R': rear_bumper,
        'PDC_Sensors_Rear': rear_bumper,
        'Trunk_Badge': trunk_lid,
        'Wheel_Arch_FL': fender_fl or body_ob,
        'Wheel_Arch_FR': fender_fr or body_ob,
        'Wheel_Arch_RL': fender_rl or body_ob,
        'Wheel_Arch_RR': fender_rr or body_ob,
        'Headlight_L': rad_sup,
        'Headlight_R': rad_sup,
        'Headlight_DRL_L': rad_sup,
        'Headlight_DRL_R': rad_sup,
        'Headlight_Lens_L': rad_sup,
        'Headlight_Lens_R': rad_sup,
        'Tail_Housing_L': rear_sf,
        'Tail_Housing_R': rear_sf,
        'Tail_Light_L': rear_sf,
        'Tail_Light_R': rear_sf,
        'Tail_Lens_L': rear_sf,
        'Tail_Lens_R': rear_sf,
    }
    for ob in wheels + detail_objs + interior_objs:
        parent = parent_map.get(ob.name, body_ob)
        ob.parent = parent
        ob.matrix_parent_inverse = parent.matrix_world.inverted()

    steering = bpy.data.objects.get('SteeringWheel')
    animate(wheels, steering, body_ob, door_objs, trunk_lid=trunk_lid, hood_lid=hood_lid)

    # Санитизация имён мешей под UE (единая функция sanitize_name)
    for ob in bpy.data.objects:
        if ob.type == 'MESH':
            clean = sanitize_name(ob.name)
            if ob.name != clean:
                ob.name = clean

    setup_render(samples=args.samples)
    blend_path = os.path.join(out_dir, 'car_realistic.blend')
    bpy.ops.wm.save_as_mainfile(filepath=blend_path, check_existing=False)
    print('scene saved:', blend_path)

    if args.render:
        render_stills(cams, out_dir)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
