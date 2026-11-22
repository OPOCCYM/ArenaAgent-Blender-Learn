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
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE and HERE not in sys.path:
    sys.path.insert(0, HERE)

from car_geometry import (  # noqa: E402
    AXLE_F,
    AXLE_R,
    BELT,
    CABIN_CUT_X_FRONT,
    CABIN_CUT_X_REAR,
    CABIN_FLOOR_Z,
    CABIN_SILL_Z,
    CAR_COLLECTION,
    DOORS_SPEC,
    HOOD_BAY_Z,
    HOOD_HINGE_X,
    HOOD_HINGE_Y,
    HOOD_LID_THICKNESS,
    HOOD_LIFT_BODY_XZ,
    HOOD_LIFT_CYL_LEN,
    HOOD_LIFT_ROD_LEN,
    HOOD_LIFT_Y,
    HOOD_OPEN_DEG,
    HOOD_X_FRONT,
    HOOD_X_REAR,
    LENGTH,
    N_SECTIONS,
    PILLAR_B_HALF_X,
    PILLAR_B_X,
    REAR_GLASS_X_BOT,
    ROOF_THICKNESS,
    ROOF_X_FRONT,
    ROOF_X_REAR,
    STUDIO_COLLECTION,
    TARGET_COORDS,
    TIP_F,
    TIP_R,
    TRUNK_BAY_Z,
    TRUNK_HINGE_X,
    TRUNK_HINGE_Y,
    TRUNK_LID_THICKNESS,
    TRUNK_LIFT_BODY_XZ,
    TRUNK_LIFT_CYL_LEN,
    TRUNK_LIFT_ROD_LEN,
    TRUNK_LIFT_Y,
    TRUNK_OPEN_DEG,
    TRUNK_X_FRONT,
    TRUNK_X_REAR,
    WHEEL_R,
    WHEEL_WIDTH,
    WHEEL_Y,
    WHEEL_Z,
    WIDTH,
    WINDSHIELD_X_BOT,
    W,
    WR,
    X_MAX,
    X_MIN,
    ZB,
    ZR,
    body_section,
    clamp,
    dome_scale,
    hood_deck_z,
    hood_hinge_z,
    hood_lift_lid_local_xz,
    sanitize_name,
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


def bmesh_to_object(name, bm, material=None, smooth=True, collection=None,
                    auto_smooth_deg=30.0):
    """Конвертирует BMesh в объект сцены, настраивает сглаживание (Blender 4.2 API)
    и выполняет очистку геометрии."""
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
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
    """CarPaint_MetallicBlue: металлик + лак (Coat) + лёгкий процедурный микрорельеф."""
    m = bpy.data.materials.new('CarPaint_MetallicBlue')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = _bsdf_base(m, (0.055, 0.115, 0.280), 0.62, 0.26, coat=1.0, coat_rough=0.03)
    tc = nt.nodes.new('ShaderNodeTexCoord')
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 420.0
    noise.inputs['Detail'].default_value = 3.0
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.015
    bump.inputs['Distance'].default_value = 0.001
    nt.links.new(tc.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Coat Normal'])
    return m


def mat_white(color=(0.9, 0.9, 0.9), roughness=0.9):
    m = bpy.data.materials.new('White_Studio')
    m.use_nodes = True
    _bsdf_base(m, color, 0.0, roughness)
    return m


def mat_glass():
    """Glass_Window: Transparent + Glossy по Layer Weight (без Transmission),
    тонировка 0.58, Roughness 0.065."""
    m = bpy.data.materials.new('Glass_Window')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    trans = nt.nodes.new('ShaderNodeBsdfTransparent')
    trans.inputs['Color'].default_value = (0.58, 0.63, 0.68, 1.0)
    gloss = nt.nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Roughness'].default_value = 0.065
    _, _, a_in, b_in, res_out = _new_mix_rgba(nt, blend_type='MIX', fac=0.58)
    a_in.default_value = (0.30, 0.40, 0.55, 1.0)
    b_in.default_value = (0.55, 0.70, 0.90, 1.0)
    nt.links.new(res_out, gloss.inputs['Color'])
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.inputs['Blend'].default_value = 0.35
    mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(lw.outputs['Fresnel'], mix.inputs['Fac'])
    nt.links.new(trans.outputs[0], mix.inputs[1])
    nt.links.new(gloss.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs['Surface'])
    return m


def mat_lens():
    """Lens_Clear: Transparent + Glossy по Fresnel (IOR 1.52) для фар и фонарей."""
    m = bpy.data.materials.new('Lens_Clear')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    trans = nt.nodes.new('ShaderNodeBsdfTransparent')
    gloss = nt.nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Roughness'].default_value = 0.045
    fres = nt.nodes.new('ShaderNodeFresnel')
    fres.inputs['IOR'].default_value = 1.52
    mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(fres.outputs['Fac'], mix.inputs['Fac'])
    nt.links.new(trans.outputs[0], mix.inputs[1])
    nt.links.new(gloss.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs['Surface'])
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
    m = bpy.data.materials.new('Carbon_Trim')
    m.use_nodes = True
    _bsdf_base(m, (0.10, 0.10, 0.11), 0.78, 0.32)
    return m


def mat_chrome():
    """Хромированная сталь (штоки газовых лифтов, тормозные диски, ручки дверей)."""
    m = bpy.data.materials.new('Chrome_Steel')
    m.use_nodes = True
    _bsdf_base(m, (0.86, 0.88, 0.91), 0.98, 0.12)
    return m


def mat_alloy():
    """Серебристый сплав для многоспицевых колёсных дисков."""
    m = bpy.data.materials.new('Alloy_Silver')
    m.use_nodes = True
    _bsdf_base(m, (0.72, 0.74, 0.77), 0.92, 0.22, coat=0.4, coat_rough=0.08)
    return m


def mat_caliper():
    """Спортивный красный металлик для тормозных суппортов."""
    m = bpy.data.materials.new('Caliper_Red')
    m.use_nodes = True
    _bsdf_base(m, (0.58, 0.04, 0.04), 0.45, 0.24, coat=0.9, coat_rough=0.05)
    return m


def make_materials():
    return {
        'paint': mat_paint(),
        'glass': mat_glass(),
        'lens': mat_lens(),
        'rubber': mat_rubber(),
        'leather': mat_leather('Leather_Seat', (0.042, 0.038, 0.037), scale=9.0, roughness=0.65),
        'carpet': mat_leather('Carpet', (0.025, 0.022, 0.020), scale=42.0, roughness=0.85),
        'headliner': mat_leather('Headliner_Fabric', (0.07, 0.065, 0.06),
                                 scale=55.0, roughness=0.90),
        'headlight': mat_emission('Headlight_Light', 0.55, (1.0, 0.95, 0.85)),
        'drl': mat_emission('DRL_Light', 0.22, (1.0, 0.92, 0.80)),
        'tail': mat_emission('Tail_Light', 0.16, (0.95, 0.10, 0.12)),
        'carbon': mat_carbon(),
        'chrome': mat_chrome(),
        'alloy': mat_alloy(),
        'caliper': mat_caliper(),
        'white': mat_white(),
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
    """Строит замкнутый лофт кузова между TIP_R и TIP_F с 3D-куполами торцов."""
    bm = bmesh.new()
    xs = [TIP_R + (TIP_F - TIP_R) * i / (N_SECTIONS - 1) for i in range(N_SECTIONS)]
    loops = [body_section(x) for x in xs]
    vcount = len(loops[0])
    verts = []
    for x, loop in zip(xs, loops):
        for (y, z) in loop:
            verts.append(bm.verts.new(Vector((x, y, z))))

    for i in range(N_SECTIONS - 1):
        base = i * vcount
        nbase = (i + 1) * vcount
        for k in range(vcount):
            bm.faces.new((
                verts[base + k],
                verts[base + (k + 1) % vcount],
                verts[nbase + (k + 1) % vcount],
                verts[nbase + k],
            ))

    # Замыкание торцов: 3D-купола по суперэллипсу от TIP_R -> X_MIN и от TIP_F -> X_MAX
    endcaps = (
        (X_MIN, 0, xs[0], True),            # корма (-X)
        (X_MAX, N_SECTIONS - 1, xs[-1], False),  # нос (+X)
    )
    n_rings = 7
    for x_end, i0, x_start, flip_cap in endcaps:
        prev = [verts[i0 * vcount + k] for k in range(vcount)]
        base_loop = loops[i0]
        zm = 0.5 * (ZB(x_start) + ZR(x_start))
        tip = bm.verts.new(Vector((x_end, 0.0, zm)))
        # n_rings - 1 промежуточных колец с ненулевым масштабом; последнее смыкается в tip
        for step in range(1, n_rings):
            u = step / float(n_rings)
            s = max(0.06, dome_scale(u))
            xr = x_start + (x_end - x_start) * u
            ring = [
                bm.verts.new(Vector((xr, y * s, zm + (z - zm) * s)))
                for (y, z) in base_loop
            ]
            for k in range(vcount):
                kn = (k + 1) % vcount
                if flip_cap:
                    bm.faces.new((prev[kn], prev[k], ring[k], ring[kn]))
                else:
                    bm.faces.new((prev[k], prev[kn], ring[kn], ring[k]))
            prev = ring
        for k in range(vcount):
            kn = (k + 1) % vcount
            if flip_cap:
                bm.faces.new((tip, prev[k], prev[kn]))
            else:
                bm.faces.new((tip, prev[kn], prev[k]))

    smooth_laplacian(bm, iters=1, factor=0.22)
    ob = bmesh_to_object('Body', bm, material=mats['paint'], smooth=True, collection=collection)
    uv_top(ob)
    return ob


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

    # 1. Панель крыши (Roof): замкнутая изогнутая оболочка от ROOF_X_REAR до ROOF_X_FRONT
    bm_roof = bmesh.new()
    nx, ny = 18, 11
    xs = [
        (ROOF_X_REAR - 0.02) + ((ROOF_X_FRONT + 0.02) - (ROOF_X_REAR - 0.02)) * i / (nx - 1)
        for i in range(nx)
    ]
    outer_pts = []
    inner_pts = []
    for x in xs:
        hw = WR(x) + 0.012
        zr = ZR(x)
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            z_top = zr - 0.016 * (v * v)
            row_out.append(Vector((x, y, z_top)))
            row_in.append(Vector((x, y, z_top - ROOF_THICKNESS)))
        outer_pts.append(row_out)
        inner_pts.append(row_in)
    _add_thick_grid(bm_roof, outer_pts, inner_pts)
    roof_ob = bmesh_to_object('Roof', bm_roof, material=mats['paint'],
                              smooth=True, collection=collection)
    uv_top(roof_ob)
    roof_ob.parent = body_ob
    roof_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(roof_ob)

    # 2. Несущие стойки кузова A, B, C (левый и правый борт)
    for label, side in (('L', 1.0), ('R', -1.0)):
        # 2a. Стойка A: от порога/моторного щита вдоль лобового стекла в передний угол крыши.
        # Нижняя часть утоплена в дверной проём (~35 мм), чтобы закрытая дверь была заподлицо.
        bm_a = bmesh.new()
        xa_bot = CABIN_CUT_X_FRONT + 0.045
        st_a = [
            (CABIN_CUT_X_FRONT - 0.005, xa_bot + 0.035, W(xa_bot) * 0.84, W(xa_bot) * 0.925,
             CABIN_SILL_Z - 0.03),
            (CABIN_CUT_X_FRONT - 0.005, xa_bot + 0.035, W(xa_bot) * 0.85, W(xa_bot) * 0.935,
             0.62),
        ]
        for step in range(6):
            t = step / 5.0
            x_center = (CABIN_CUT_X_FRONT + 0.035) * (1.0 - t) + (ROOF_X_FRONT + 0.005) * t
            half_x = 0.044 * (1.0 - 0.16 * t)
            y_out, z_val = upper_side_yz(x_center, t)
            if step == 5:
                y_out = WR(x_center) + 0.004
                z_val = ZR(x_center) - 0.020  # аккуратно входит внутрь толщины крыши
            thick_y = 0.064 * (1.0 - 0.16 * t)
            st_a.append((x_center - half_x, x_center + half_x,
                         y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_a, st_a, side=side)
        pa = bmesh_to_object(f'Pillar_A_{label}', bm_a, material=mats['paint'],
                             smooth=True, collection=collection)
        uv_box(pa, scale=2.0)
        pa.parent = body_ob
        pa.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pa)

        # 2b. Центральная стойка B: разделяет передний и задний дверные вырезы,
        # нижняя часть утоплена в проём за кромками закрытых дверей, верхняя идёт в крышу Roof.
        bm_b = bmesh.new()
        wb = W(PILLAR_B_X)
        st_b = [
            (PILLAR_B_X - PILLAR_B_HALF_X, PILLAR_B_X + PILLAR_B_HALF_X,
             wb * 0.84, wb * 0.920, CABIN_SILL_Z - 0.03),
            (PILLAR_B_X - PILLAR_B_HALF_X, PILLAR_B_X + PILLAR_B_HALF_X,
             wb * 0.85, wb * 0.930, 0.62),
        ]
        for step in range(6):
            t = step / 5.0
            half_x = PILLAR_B_HALF_X * (1.0 - 0.16 * t)
            y_out, z_val = upper_side_yz(PILLAR_B_X, t)
            if step == 0:
                y_out = wb * 0.940  # скрывается за стыком закрытых дверей на уровне пояса
            elif step == 5:
                y_out = WR(PILLAR_B_X) + 0.004
                z_val = ZR(PILLAR_B_X) - 0.020
            thick_y = 0.064 * (1.0 - 0.16 * t)
            st_b.append((PILLAR_B_X - half_x, PILLAR_B_X + half_x,
                         y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_b, st_b, side=side)
        pb = bmesh_to_object(f'Pillar_B_{label}', bm_b, material=mats['paint'],
                             smooth=True, collection=collection)
        uv_box(pb, scale=2.0)
        pb.parent = body_ob
        pb.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pb)

        # 2c. Задняя стойка C: широкая трапециевидная стойка от заднего крыла до крыши
        bm_c = bmesh.new()
        xc_jamb = CABIN_CUT_X_REAR - 0.03
        st_c = [
            (xc_jamb - 0.08, CABIN_CUT_X_REAR + 0.005,
             W(xc_jamb) * 0.84, W(xc_jamb) * 0.925, CABIN_SILL_Z - 0.03),
            (xc_jamb - 0.12, CABIN_CUT_X_REAR + 0.005,
             W(xc_jamb) * 0.85, W(xc_jamb) * 0.935, 0.62),
        ]
        for step in range(6):
            t = step / 5.0
            x_back = (REAR_GLASS_X_BOT - 0.02) * (1.0 - t) + (ROOF_X_REAR - 0.015) * t
            x_front = (CABIN_CUT_X_REAR + 0.005) * (1.0 - t) + (ROOF_X_REAR + 0.14) * t
            x_mid = 0.5 * (x_back + x_front)
            y_out, z_val = upper_side_yz(x_mid, t)
            if step == 5:
                y_out = WR(x_mid) + 0.004
                z_val = ZR(x_mid) - 0.020
            thick_y = 0.072 * (1.0 - 0.16 * t)
            st_c.append((x_back, x_front, y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_c, st_c, side=side)
        pc = bmesh_to_object(f'Pillar_C_{label}', bm_c, material=mats['paint'],
                             smooth=True, collection=collection)
        uv_box(pc, scale=2.0)
        pc.parent = body_ob
        pc.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pc)

    return roof_ob, objs


def build_glass(mats, collection):
    """Остекление кузова: лобовое стекло (между стойками A) и заднее стекло (между стойками C).
    Боковые проёмы дверей оставлены открытыми — стёкла дверей крепятся к самим дверям."""
    bm = bmesh.new()
    thickness = 0.005

    def _add_curved_pane(x_bot, z_bot, y_bot, x_top, z_top, y_top, bulge_x, nu=9, nv=13):
        outer_pts = []
        inner_pts = []
        # Нормаль панели в плоскости XZ (направлена наружу от салона)
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
            hw = y_bot * (1.0 - u) + y_top * u
            row_out = []
            row_in = []
            for j in range(nv):
                v = -1.0 + 2.0 * j / (nv - 1)
                y = v * hw
                bow = bulge_x * (1.0 - v * v)
                p_out = Vector((xc + bow, y, zc - 0.010 * (v * v)))
                row_out.append(p_out)
                row_in.append(p_out - n_xz * thickness)
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm, outer_pts, inner_pts)

    # Лобовое стекло (между стойками A, от основания у капота до передней кромки крыши)
    _add_curved_pane(
        x_bot=WINDSHIELD_X_BOT,
        z_bot=BELT(WINDSHIELD_X_BOT) + 0.004,
        y_bot=W(WINDSHIELD_X_BOT) * 0.89,
        x_top=ROOF_X_FRONT + 0.012,
        z_top=ZR(ROOF_X_FRONT) - 0.018,
        y_top=WR(ROOF_X_FRONT) * 0.94,
        bulge_x=+0.024,
    )

    # Заднее стекло (между стойками C, от задней полки перед багажником до кромки крыши)
    _add_curved_pane(
        x_bot=REAR_GLASS_X_BOT,
        z_bot=BELT(REAR_GLASS_X_BOT) + 0.032,
        y_bot=W(REAR_GLASS_X_BOT) * 0.83,
        x_top=ROOF_X_REAR - 0.005,
        z_top=ZR(ROOF_X_REAR) - 0.018,
        y_top=WR(ROOF_X_REAR) * 0.93,
        bulge_x=-0.022,
    )

    ob = bmesh_to_object('Glass', bm, material=mats['glass'], smooth=True, collection=collection)
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
    seg = 36
    hw = WHEEL_WIDTH / 2.0
    r_bead = 0.242

    # 1. Тороидальная шина со скруглёнными боковинами (полая внутри под спицы)
    bm_tire = bmesh.new()
    r_out = WHEEL_R
    r_sh = WHEEL_R - 0.016
    r_mid = 0.5 * (r_bead + r_out)
    tire_profile = [
        (r_bead, -hw * 0.90),
        (r_mid, -hw * 1.02),
        (r_sh, -hw * 0.96),
        (r_out, -hw * 0.78),
        (r_out, +hw * 0.78),
        (r_sh, +hw * 0.96),
        (r_mid, +hw * 1.02),
        (r_bead, +hw * 0.90),
    ]
    _add_y_annulus(bm_tire, r_bead, r_out, -hw, hw, seg=seg, profile=tire_profile)
    tire = bmesh_to_object(f'Wheel_{tag}', bm_tire, material=mats['rubber'],
                           smooth=True, collection=collection)
    uv_cyl(tire, WHEEL_R, WHEEL_WIDTH)
    tire.location = Vector((axle_x, cy, cz))

    # 2. 10-спицевый легкосплавный диск (обод + ступица + 5 сдвоенных спиц)
    bm_rim = bmesh.new()
    # Внешний обод (бочка диска)
    _add_y_annulus(bm_rim, 0.224, 0.244, -hw * 0.88, hw * 0.88, seg=seg)
    # Центральная ступица (колпачок)
    y_face = side * (hw * 0.62)
    y_hub_in = side * (hw * 0.12)
    _add_y_cylinder(bm_rim, 0.064, min(y_hub_in, y_face), max(y_hub_in, y_face),
                    seg=20, center_xz=(0.0, 0.0))
    # 5 сдвоенных (10) объёмных спиц, слегка выгнутых к наружной кромке обода
    for pair_idx in range(5):
        base_ang = 2.0 * math.pi * pair_idx / 5.0
        for split in (-0.17, +0.17):
            ang = base_ang + split
            rad_dir = Vector((math.cos(ang), 0.0, math.sin(ang)))
            tan_dir = Vector((-math.sin(ang), 0.0, math.cos(ang)))
            y_dir = Vector((0.0, side, 0.0))
            pts = [
                rad_dir * 0.068 + y_dir * (hw * 0.44),
                rad_dir * 0.145 + y_dir * (hw * 0.68),
                rad_dir * 0.220 + y_dir * (hw * 0.72),
            ]
            rings = []
            for idx_p, p in enumerate(pts):
                sw = 0.0115 * (1.0 - 0.18 * idx_p)
                sd = 0.013
                rings.append([
                    bm_rim.verts.new(p - tan_dir * sw - y_dir * sd),
                    bm_rim.verts.new(p + tan_dir * sw - y_dir * sd),
                    bm_rim.verts.new(p + tan_dir * sw + y_dir * sd),
                    bm_rim.verts.new(p - tan_dir * sw + y_dir * sd),
                ])
            for i_s in range(len(rings) - 1):
                r0, r1 = rings[i_s], rings[i_s + 1]
                for k in range(4):
                    kn = (k + 1) % 4
                    bm_rim.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
            bm_rim.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
            bm_rim.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))

    rim = bmesh_to_object(f'Rim_{tag}', bm_rim, material=mats['alloy'],
                          smooth=True, collection=collection)
    uv_box(rim, scale=4.0)
    rim.parent = tire
    rim.location = Vector((0.0, 0.0, 0.0))

    # 3. Тормозной диск (вращается вместе с колесом, виден сквозь спицы)
    bm_disc = bmesh.new()
    y_disc_c = side * (hw * 0.14)
    _add_y_cylinder(bm_disc, 0.172, y_disc_c - 0.011, y_disc_c + 0.011,
                    seg=28, center_xz=(0.0, 0.0))
    disc = bmesh_to_object(f'Brake_Disc_{tag}', bm_disc, material=mats['chrome'],
                           smooth=True, collection=collection)
    uv_box(disc, scale=5.0)
    disc.parent = tire
    disc.location = Vector((0.0, 0.0, 0.0))

    # 4. Тормозной суппорт (привязан к кузову, остаётся неподвижным при вращении колеса)
    bm_cal = bmesh.new()
    cal_x = -0.142 if axle_x > 0 else +0.142  # на передней оси сзади, на задней спереди
    add_box(bm_cal, cal_x - 0.036, cal_x + 0.036,
            y_disc_c - 0.028, y_disc_c + 0.028, -0.055, 0.068)
    caliper = bmesh_to_object(f'Brake_Caliper_{tag}', bm_cal, material=mats['caliper'],
                              smooth=True, collection=collection)
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
    """Замкнутый манифолд-руль с origin в центре обода и наклоном к водителю."""
    bm = bmesh.new()
    r_out, r_in, dt, seg = 0.185, 0.155, 0.024, 32
    # Обод строится в плоскости YZ (ось вращения — локальная X, наклон в сторону водителя)
    rings = []
    for xx in (-dt / 2.0, dt / 2.0):
        rout = []
        rin = []
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            cy, cz = math.cos(a), math.sin(a)
            rout.append(bm.verts.new(Vector((xx, r_out * cy, r_out * cz))))
            rin.append(bm.verts.new(Vector((xx, r_in * cy, r_in * cz))))
        rings.append((rout, rin))

    rout0, rin0 = rings[0]
    rout1, rin1 = rings[1]
    for k in range(seg):
        kn = (k + 1) % seg
        # Задняя и передняя плоские грани обода
        bm.faces.new((rout0[k], rin0[k], rin0[kn], rout0[kn]))
        bm.faces.new((rout1[k], rout1[kn], rin1[kn], rin1[k]))
        # Внешняя и внутренняя цилиндрические стенки обода
        bm.faces.new((rout0[k], rout0[kn], rout1[kn], rout1[k]))
        bm.faces.new((rin0[k], rin1[k], rin1[kn], rin0[kn]))

    # Центральная ступица (замкнутый блок) и 3 замкнутые спицы (без касания вершин обода)
    add_box(bm, -0.028, 0.028, -0.050, 0.050, -0.050, 0.050)
    add_box(bm, -0.010, 0.010, 0.054, r_in - 0.004, -0.014, 0.014)    # левая спица
    add_box(bm, -0.010, 0.010, -(r_in - 0.004), -0.054, -0.014, 0.014)  # правая спица
    add_box(bm, -0.010, 0.010, -0.016, 0.016, -(r_in - 0.004), -0.054)  # нижняя спица

    sw = bmesh_to_object('SteeringWheel', bm, material=mats['carbon'],
                         smooth=True, collection=collection)
    uv_box(sw, scale=4.0)
    # Левый руль (+Y), слегка наклонён к водителю (-Y поворот на 20° вокруг Y)
    sw.location = Vector((0.54, 0.36, 0.87))
    sw.rotation_euler = (0.0, math.radians(-20.0), 0.0)
    return sw


def build_interior(mats, collection, door_objs=None):
    objs = []
    # Салон: между соседними блоками оставлен зазор 2 мм, чтобы remove_doubles
    # не создавал внутренних неманифолдных граней. Все объёмы внутри открытого выреза салона.
    bm = bmesh.new()
    gap = 0.002
    for side in (-1.0, 1.0):
        yc = side * 0.38
        add_box(bm, -0.08, 0.44, yc - 0.22, yc + 0.22, 0.36, 0.50)             # подушка
        add_box(bm, -0.20, -0.08 - gap, yc - 0.21, yc + 0.21, 0.44, 1.00)      # спинка
        add_box(bm, -0.18, -0.06, yc - 0.10, yc + 0.10, 1.00 + gap, 1.18)      # подголовник
    # Задний диван (расположен точно в зоне заднего дверного проёма перед перегородкой)
    add_box(bm, -0.78, -0.34, -0.56, 0.56, 0.36, 0.50)
    add_box(bm, -0.92, -0.78 - gap, -0.56, 0.56, 0.44, 0.94)
    add_box(bm, -0.91, -0.80, -0.14, 0.14, 0.94 + gap, 1.10)                   # подголовник центра
    # Задняя полка под задним стеклом (не выступает в нишу багажника)
    add_box(bm, -1.34, -0.96, -0.56, 0.56, 1.00, 1.04)
    # Центральный тоннель (над ковром пола салона)
    add_box(bm, -0.88, 0.82, -0.12, 0.12, CABIN_FLOOR_Z + 0.028, 0.58)
    cabin = bmesh_to_object('Cabin', bm, material=mats['leather'],
                            smooth=True, collection=collection)
    uv_box(cabin, scale=1.5)
    objs.append(cabin)

    # Панель приборов (передняя часть салона +X у моторного щита)
    dash = _make_box_object('Dashboard', (0.62, 0.85, -0.74, 0.74, 0.56, 0.96),
                            mats['leather'], collection, uv_scale=1.5)
    objs.append(dash)

    # Центральный экран на панели
    screen = _make_box_object('Screen', (0.585, 0.618, -0.14, 0.14, 0.72, 0.92),
                              mats['lens'], collection, uv_scale=2.0)
    objs.append(screen)

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

    # Динамики в передних дверях (привязаны к Door_Front_L/R, открываются вместе с дверью)
    door_map = {ob.name: ob for ob in (door_objs or [])}
    for label, side, door_name in (('L', 1.0, 'Door_Front_Left'),
                                   ('R', -1.0, 'Door_Front_Right')):
        bm_sp = bmesh.new()
        r0, r1, h = 0.044, 0.036, 0.016
        # Локальные координаты относительно передней двери (шарнир в x = 0, y = 0, z = sill)
        xa = -0.34
        z_loc = 0.28
        y_base = -side * 0.056
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
        sp = bmesh_to_object(f'Speaker_{label}', bm_sp, material=mats['carbon'],
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
    door_thickness = 0.052
    hinge_z = CABIN_SILL_Z + 0.006  # зазор 6 мм над порогом кузова

    for name, hx, side, door_len, _open_deg in DOORS_SPEC:
        is_front = 'Front' in name
        hinge_y = side * (W(hx) * 0.965)
        origin = Vector((hx, hinge_y, hinge_z))

        # 1. Контурное нижнее полотно двери (повторяет кривизну борта W(x) от порога до пояса)
        bm = bmesh.new()
        nx, nz = 10, 8
        x_f = hx - 0.005
        x_b = hx - door_len + 0.005
        xs = [x_f + (x_b - x_f) * i / (nx - 1) for i in range(nx)]

        outer_pts = []
        inner_pts = []
        for x in xs:
            wx = W(x)
            z_top = BELT(x)
            row_out = []
            row_in = []
            for j in range(nz):
                v = j / (nz - 1)
                z_w = hinge_z + (z_top - hinge_z) * v
                # Лёгкая выпуклость дверного полотна по вертикали (tumblehome кузова)
                profile_frac = 0.960 + 0.040 * math.sin(0.5 * math.pi * v)
                y_out_w = side * (wx * profile_frac)
                y_in_w = y_out_w - side * door_thickness
                row_out.append(Vector((x - hx, y_out_w - hinge_y, z_w - hinge_z)))
                row_in.append(Vector((x - hx, y_in_w - hinge_y, z_w - hinge_z)))
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm, outer_pts, inner_pts)

        # 2. Верхняя оконная рамка двери (следует за стойками A/B/C и завалом остекления)
        if is_front:
            xf_bot, xf_top = x_f, ROOF_X_FRONT + 0.012
            xb_bot, xb_top = x_b + 0.032, PILLAR_B_X + PILLAR_B_HALF_X + 0.006
        else:
            xf_bot, xf_top = x_f - 0.032, PILLAR_B_X - PILLAR_B_HALF_X - 0.006
            xb_bot, xb_top = x_b, ROOF_X_REAR + 0.145

        def _sash_pt(x_val, t_val, y_off=0.0):
            y_abs, z_val = upper_side_yz(x_val, t_val)
            return Vector((x_val - hx, side * (y_abs + y_off) - hinge_y, z_val - hinge_z))

        # Три замкнутые балки оконной рамки (передняя, верхняя, задняя) с зазором 2 мм от полотна
        frame_w = 0.024
        frame_d = 0.024

        def _add_sash_bar(pts_3d):
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
                for k in range(4):
                    kn = (k + 1) % 4
                    bm.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
            bm.faces.new((rings[0][0], rings[0][3], rings[0][2], rings[0][1]))
            bm.faces.new((rings[-1][0], rings[-1][1], rings[-1][2], rings[-1][3]))

        # Непрерывная дуга оконной рамки: передняя стойка -> верхняя планка -> задняя стойка
        arch_pts = []
        for k in range(5):
            t = 0.03 + 0.94 * (k / 4.0)
            arch_pts.append(_sash_pt(xf_bot * (1.0 - t) + xf_top * t, t, y_off=-0.010))
        for k in range(1, 5):
            u = k / 4.0
            arch_pts.append(_sash_pt(xf_top * (1.0 - u) + xb_top * u, 0.97, y_off=-0.010))
        for k in range(1, 5):
            t = 0.97 - 0.94 * (k / 4.0)
            arch_pts.append(_sash_pt(xb_bot * (1.0 - t) + xb_top * t, t, y_off=-0.010))
        _add_sash_bar(arch_pts)

        door_ob = bmesh_to_object(name, bm, material=mats['paint'],
                                  smooth=True, collection=collection)
        uv_box(door_ob, scale=1.5)
        door_ob.location = origin
        door_ob.parent = body_ob
        door_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        door_objs.append(door_ob)

        # 3. Боковое стекло двери (Door_Glass_*), привязано к двери — открывается вместе с ней
        bm_gl = bmesh.new()
        g_out = []
        g_in = []
        nu_g, nv_g = 6, 6
        g_thick = 0.005
        for j in range(nv_g):
            t = 0.04 + 0.90 * (j / (nv_g - 1))
            xf_t = xf_bot * (1.0 - t) + xf_top * t - 0.014
            xb_t = xb_bot * (1.0 - t) + xb_top * t + 0.014
            row_o = []
            row_i = []
            for i in range(nu_g):
                u = i / (nu_g - 1)
                x_g = xf_t * (1.0 - u) + xb_t * u
                y_abs, z_g = upper_side_yz(x_g, t)
                y_o = side * (y_abs - 0.006) - hinge_y
                y_i = side * (y_abs - 0.006 - g_thick) - hinge_y
                row_o.append(Vector((x_g - hx, y_o, z_g - hinge_z)))
                row_i.append(Vector((x_g - hx, y_i, z_g - hinge_z)))
            g_out.append(row_o)
            g_in.append(row_i)
        _add_thick_grid(bm_gl, g_out, g_in)
        glass_name = name.replace('Door_', 'Door_Glass_')
        dg_ob = bmesh_to_object(glass_name, bm_gl, material=mats['glass'],
                                smooth=True, collection=collection)
        uv_box(dg_ob, scale=1.5)
        dg_ob.parent = door_ob
        dg_ob.location = Vector((0.0, 0.0, 0.0))

        # 4. Две дверные петли на стойке A / B (верхняя и нижняя)
        bm_dh = bmesh.new()
        for hz_pin in (0.42, 0.78):
            _add_z_cylinder(bm_dh, 0.010, hz_pin - 0.028, hz_pin + 0.028,
                            seg=12, center_xy=(hx + 0.006, hinge_y - side * 0.026))
        hinge_name = name.replace('Door_', 'Door_Hinge_')
        dh_ob = bmesh_to_object(hinge_name, bm_dh, material=mats['carbon'],
                                smooth=True, collection=collection)
        uv_box(dh_ob, scale=8.0)
        dh_ob.parent = body_ob
        dh_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()

        # 5. Наружная дверная ручка (привязана к двери, движется вместе с ней)
        bm_hdl = bmesh.new()
        x_hdl_w = hx - door_len + 0.17
        z_hdl_w = BELT(x_hdl_w) - 0.125
        y_surf_w = side * (W(x_hdl_w) * 0.996)
        xh_l = x_hdl_w - hx
        yh_l = y_surf_w - hinge_y
        zh_l = z_hdl_w - hinge_z
        # Стойки ручки + вытянутая рукоятка с зазором от полотна двери
        add_box(bm_hdl, xh_l - 0.080, xh_l + 0.080,
                yh_l + side * 0.016, yh_l + side * 0.032, zh_l - 0.013, zh_l + 0.013)
        add_box(bm_hdl, xh_l - 0.075, xh_l - 0.052,
                yh_l - side * 0.002, yh_l + side * 0.014, zh_l - 0.011, zh_l + 0.011)
        add_box(bm_hdl, xh_l + 0.052, xh_l + 0.075,
                yh_l - side * 0.002, yh_l + side * 0.014, zh_l - 0.011, zh_l + 0.011)
        handle_name = name.replace('Door_', 'Door_Handle_')
        hdl_ob = bmesh_to_object(handle_name, bm_hdl, material=mats['chrome'],
                                 smooth=True, collection=collection)
        uv_box(hdl_ob, scale=10.0)
        hdl_ob.parent = door_ob
        hdl_ob.location = Vector((0.0, 0.0, 0.0))

        # 6. Боковое зеркало заднего вида (только на передних дверях, открывается с дверью)
        if is_front:
            m_tag = 'L' if side > 0 else 'R'
            xm_w = hx - 0.15
            zm_w = BELT(xm_w) + 0.035
            ym_w = side * (W(xm_w) * 0.98)
            xm_l = xm_w - hx
            ym_l = ym_w - hinge_y
            zm_l = zm_w - hinge_z
            bm_mir = bmesh.new()
            # Ножка-кронштейн зеркала (от угла передней двери наружу)
            add_box(bm_mir, xm_l - 0.032, xm_l + 0.032,
                    ym_l - side * 0.008, ym_l + side * 0.046,
                    zm_l - 0.015, zm_l + 0.015)
            # Обтекаемый корпус зеркала
            add_box(bm_mir, xm_l - 0.048, xm_l + 0.042,
                    ym_l + side * 0.049, ym_l + side * 0.155,
                    zm_l - 0.024, zm_l + 0.062)
            mir_ob = bmesh_to_object(f'Mirror_{m_tag}', bm_mir, material=mats['paint'],
                                     smooth=True, collection=collection)
            uv_box(mir_ob, scale=6.0)
            mir_ob.parent = door_ob
            mir_ob.location = Vector((0.0, 0.0, 0.0))

    return door_objs


def _sweep_rect_tube(bm, points, half_y, half_n):
    """Замкнутая прямоугольная балка вдоль плоской (в XZ) ломаной points (для петель)."""
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
        side_y = Vector((0.0, 1.0, 0.0))
        normal_xz = tangent.cross(side_y).normalized()
        ring = [
            bm.verts.new(p - side_y * half_y - normal_xz * half_n),
            bm.verts.new(p + side_y * half_y - normal_xz * half_n),
            bm.verts.new(p + side_y * half_y + normal_xz * half_n),
            bm.verts.new(p - side_y * half_y + normal_xz * half_n),
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


def build_trunk(body_ob, mats, collection):
    """Строит отделённую от кузова крышку багажника, коврик ниши, две петли
    («гусиная шея») и два телескопических газовых лифта (цилиндр + шток).

    Все детали замкнуты (manifold), имеют UV и ацикличную схему TRACK_TO,
    поэтому корректно работают при прокрутке кадров и запекаются в FBX."""
    objs = []
    hz = trunk_hinge_z()
    hinge_origin = Vector((TRUNK_HINGE_X, 0.0, hz))

    # 1. Напольный ковёр внутри углублённой ниши багажника (выше сглаженного дна кузова)
    trunk_floor = _make_box_object(
        'Trunk_Floor',
        (TRUNK_X_REAR + 0.08, TRUNK_X_FRONT - 0.06, -0.45, 0.45,
         TRUNK_BAY_Z + 0.045, TRUNK_BAY_Z + 0.068),
        mats['carpet'], collection, uv_scale=2.5,
    )
    trunk_floor.parent = body_ob
    trunk_floor.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(trunk_floor)

    # 2. Отделённая крышка багажника (origin в оси петли — вращается вокруг локальной Y)
    bm_lid = bmesh.new()
    nx = 18
    ny = 11
    x_start = TRUNK_X_FRONT - 0.004   # плотный зазор ~4 мм у заднего стекла
    x_end = TRUNK_X_REAR - 0.040      # доходит до задней кромки кормы
    xs = [x_start + (x_end - x_start) * i / (nx - 1) for i in range(nx)]

    outer = []
    inner = []
    for x in xs:
        hw = WR(x) + 0.042  # точно накрывает гребни задних крыльев при закрытой крышке
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            # Плавное опускание боковых кромок крышки к гребням крыльев (убирает боковую щель)
            edge_drop = 0.022 * (abs(v) ** 3.0)
            z_top = trunk_deck_z(x, y) - edge_drop + 0.002
            # Локальные координаты относительно оси петли hinge_origin
            p_out = Vector((x - TRUNK_HINGE_X, y, z_top - hz))
            p_in = Vector((x - TRUNK_HINGE_X, y, z_top - TRUNK_LID_THICKNESS - hz))
            row_out.append(bm_lid.verts.new(p_out))
            row_in.append(bm_lid.verts.new(p_in))
        outer.append(row_out)
        inner.append(row_in)

    # Добавляем вертикальную заднюю юбку/кромку багажника (3 ряда вниз на ~0.14 м)
    lip_x = x_end
    lip_hw = WR(lip_x) + 0.042
    lip_top_z = [
        trunk_deck_z(lip_x, (-1.0 + 2.0 * j / (ny - 1)) * lip_hw)
        - 0.022 * (abs(-1.0 + 2.0 * j / (ny - 1)) ** 3.0) + 0.002
        for j in range(ny)
    ]
    for step in range(1, 4):
        frac = step / 3.0
        drop = 0.14 * frac
        x_back = lip_x - 0.018 * frac
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * lip_hw
            z_cur = lip_top_z[j] - drop
            row_out.append(bm_lid.verts.new(Vector((x_back - TRUNK_HINGE_X, y, z_cur - hz))))
            row_in.append(bm_lid.verts.new(Vector((
                x_back + TRUNK_LID_THICKNESS - TRUNK_HINGE_X, y, z_cur - hz))))
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

    trunk_lid = bmesh_to_object('Trunk_Lid', bm_lid, material=mats['paint'],
                                smooth=True, collection=collection)
    uv_box(trunk_lid, scale=1.0)
    trunk_lid.location = hinge_origin
    trunk_lid.parent = body_ob
    trunk_lid.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(trunk_lid)

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
        z_pad = (trunk_deck_z(TRUNK_HINGE_X - 0.28, hy) - TRUNK_LID_THICKNESS - 0.008) - hz
        arm_pts = [
            Vector((0.00, hy, 0.00)),
            Vector((0.045, hy, -0.055)),
            Vector((0.025, hy, -0.115)),
            Vector((-0.075, hy, -0.125)),
            Vector((-0.185, hy, -0.095)),
            Vector((-0.275, hy, z_pad)),
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
    """Строит отделённый капот (Hood_Lid), агрегаты моторного отсека (Engine_Bay),
    две петли капота и два телескопических газовых лифта с ацикличным TRACK_TO."""
    objs = []
    hz = hood_hinge_z()
    hinge_origin = Vector((HOOD_HINGE_X, 0.0, hz))

    # 1. Агрегаты моторного отсека внутри углублённой ниши (блок ДВС, ГБЦ, радиатор, чашки стоек)
    bm_eng = bmesh.new()
    gap = 0.003
    z_base = HOOD_BAY_Z + 0.045
    # Основной блок двигателя и декоративная плита впускного коллектора
    add_box(bm_eng, 1.24, 1.96, -0.26, 0.26, z_base, z_base + 0.24)
    add_box(bm_eng, 1.28, 1.92, -0.22, 0.22, z_base + 0.24 + gap, z_base + 0.31)
    # Левая и правая крышки ГБЦ / впускные каналы
    for sgn in (-1.0, 1.0):
        add_box(bm_eng, 1.28, 1.90, sgn * 0.28, sgn * 0.38, z_base + 0.08, z_base + 0.25)
    # Передний радиатор и верхняя рамка телевизора
    add_box(bm_eng, 2.08, 2.16, -0.44, 0.44, z_base, z_base + 0.30)
    # Чашки передних амортизационных стоек (по оси AXLE_F = +1.60)
    for sgn in (-1.0, 1.0):
        _add_z_cylinder(bm_eng, 0.085, z_base, z_base + 0.28, seg=18,
                        center_xy=(AXLE_F, sgn * 0.44))
    # Аккумулятор / блок предохранителей у моторного щита
    add_box(bm_eng, 1.12, 1.23, -0.42, -0.24, z_base, z_base + 0.22)
    add_box(bm_eng, 1.12, 1.23, 0.24, 0.42, z_base, z_base + 0.20)
    engine_bay = bmesh_to_object('Engine_Bay', bm_eng, material=mats['carbon'],
                                 smooth=True, collection=collection)
    uv_box(engine_bay, scale=2.5)
    engine_bay.parent = body_ob
    engine_bay.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(engine_bay)

    # 2. Отделённая крышка капота (origin в оси задней петли, вращается вокруг -Y)
    bm_hood = bmesh.new()
    nx = 18
    ny = 11
    x_start = HOOD_X_REAR + 0.004
    x_end = HOOD_X_FRONT + 0.035
    xs = [x_start + (x_end - x_start) * i / (nx - 1) for i in range(nx)]

    outer_pts = []
    inner_pts = []
    for x in xs:
        hw = WR(x) + 0.042
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            edge_drop = 0.022 * (abs(v) ** 3.0)
            z_top = hood_deck_z(x, y) - edge_drop + 0.002
            row_out.append(Vector((x - HOOD_HINGE_X, y, z_top - hz)))
            row_in.append(Vector((x - HOOD_HINGE_X, y, z_top - HOOD_LID_THICKNESS - hz)))
        outer_pts.append(row_out)
        inner_pts.append(row_in)
    _add_thick_grid(bm_hood, outer_pts, inner_pts)
    hood_lid = bmesh_to_object('Hood_Lid', bm_hood, material=mats['paint'],
                               smooth=True, collection=collection)
    uv_box(hood_lid, scale=1.0)
    hood_lid.location = hinge_origin
    hood_lid.parent = body_ob
    hood_lid.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(hood_lid)

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
def add_light(name, loc, energy, size=0.8, rot=None, aim_target=None, collection=None):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = Vector(loc)
    if aim_target is not None:
        look_at(ob, aim_target)
    elif rot is not None:
        ob.rotation_euler = rot
    target_col = collection if collection is not None else bpy.context.scene.collection
    target_col.objects.link(ob)
    return ob


def build_lighting(mats, studio_col):
    # Интерьер: потолочные лампы смотрят строго вниз (-Z при rot=(0, 0, 0))
    add_light('Int_RoofLight_F', (0.35, 0.0, 1.32), 1.9, size=0.35,
              rot=(0.0, 0.0, 0.0), collection=studio_col)
    add_light('Int_RoofLight_M', (-0.45, 0.0, 1.32), 2.1, size=0.35,
              rot=(0.0, 0.0, 0.0), collection=studio_col)
    add_light('Int_SeatLight', (0.0, 0.35, 0.72), 1.7, size=0.20,
              aim_target=(0.0, 0.0, 0.45), collection=studio_col)
    add_light('Int_DashLight', (0.65, 0.0, 1.05), 0.9, size=0.25,
              aim_target=(0.95, 0.0, 0.75), collection=studio_col)
    add_light('Int_Ceiling', (-0.05, 0.0, 1.30), 0.9, size=0.40,
              rot=(0.0, 0.0, 0.0), collection=studio_col)
    add_light('Int_TrunkLight', (-1.78, 0.0, 1.02), 3.2, size=0.32,
              rot=(0.0, 0.0, 0.0), collection=studio_col)
    add_light('Int_EngineLight', (1.65, 0.0, 1.10), 4.0, size=0.36,
              rot=(0.0, 0.0, 0.0), collection=studio_col)

    # Студийный свет: 6 Area-ламп вокруг машины, каждая нацелена на TARGET
    studio = [
        ('Studio_Light_1', (-4.5, -4.5, 3.2), 260.0),
        ('Studio_Light_2', (4.5, -4.5, 3.2), 260.0),
        ('Studio_Light_3', (-6.0, 2.5, 2.6), 180.0),
        ('Studio_Light_4', (6.0, 2.5, 2.6), 180.0),
        ('Studio_Light_5', (0.0, 7.5, 3.6), 280.0),
        ('Studio_Light_6', (0.0, -8.5, 2.6), 160.0),
    ]
    for name, loc, en in studio:
        add_light(name, loc, en, size=1.4, aim_target=TARGET, collection=studio_col)

    # Студийный пол (тонкая замкнутая плита под Z = 0, не перекрывает ни одну из камер)
    _make_box_object('Cyclorama', (-14.0, 14.0, -14.0, 14.0, -0.04, 0.0),
                     mats['white'], studio_col, uv_scale=0.1)

    # Мир + HDRI (если файл присутствует на диске)
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
        bg.inputs['Color'].default_value = (0.12, 0.13, 0.15, 1.0)
        bg.inputs['Strength'].default_value = 1.0


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
    scene.cycles.transparent_max_bounces = 48  # обязательно (иначе муар на стёклах)
    scene.cycles.blur_glossy = 2.0             # обязательно (иначе муар на стёклах)
    scene.render.resolution_x = 1440
    scene.render.resolution_y = 810
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = 'AgX'
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

    # Навесные детали кузова через ray_cast по (y, z):
    # нос находится в +X (front=True), корма в -X (front=False).
    detail_objs = []
    nose_hit, nose_nrm = body_hit(0.0, 0.56, front=True)
    if nose_hit is not None:
        detail_objs.append(shell_patch(
            'Grille_Front', nose_hit, nose_nrm, size=(0.74, 0.24),
            segs=(10, 4), off=0.015, material=mats['carbon'],
            project=False, collection=car_col))

    for label, sy in (('L', 1.0), ('R', -1.0)):
        hl_hit, hl_nrm = body_hit(sy * 0.58, 0.68, front=True)
        if hl_hit is not None:
            detail_objs.append(shell_patch(
                f'Headlight_{label}', hl_hit, hl_nrm, size=(0.40, 0.14),
                segs=(6, 3), off=0.016, material=mats['headlight'],
                project=False, collection=car_col))
            detail_objs.append(shell_patch(
                f'Headlight_Lens_{label}', hl_hit, hl_nrm, size=(0.42, 0.15),
                segs=(6, 3), off=0.028, material=mats['lens'],
                project=False, collection=car_col))
            detail_objs.append(shell_patch(
                f'Headlight_DRL_{label}', hl_hit + Vector((0.0, 0.0, 0.03)), hl_nrm,
                size=(0.36, 0.03), segs=(6, 2), off=0.036, material=mats['drl'],
                project=False, collection=car_col))

        tl_hit, tl_nrm = body_hit(sy * 0.56, 0.80, front=False)
        if tl_hit is not None:
            detail_objs.append(shell_patch(
                f'Tail_Light_{label}', tl_hit, tl_nrm, size=(0.28, 0.10),
                segs=(5, 3), off=0.008, material=mats['tail'],
                project=False, collection=car_col))

    # Отделённый багажник: крышка, коврик ниши, петли («гусиная шея») и газовые лифты
    trunk_lid, _trunk_objs = build_trunk(body_ob, mats, car_col)

    # Отделённый капот: крышка, моторный отсек, петли и газовые лифты
    hood_lid, _hood_objs = build_hood(body_ob, mats, car_col)

    interior_objs = build_interior(mats, car_col, door_objs=door_objs)
    build_lighting(mats, studio_col)
    cams = build_cameras(studio_col)
    bpy.context.scene.camera = cams['Cam_Exterior']

    # Привязка деталей, салона и колёс к кузову (крен кузова двигает всю машину целиком)
    for ob in wheels + detail_objs + interior_objs:
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()

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
