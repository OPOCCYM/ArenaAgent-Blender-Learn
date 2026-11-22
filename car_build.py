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
    TIP_F,
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
    dome_scale,
    fender_bottom_z,
    fender_bumper_seam_z,
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


def make_materials():
    return {
        'paint': mat_paint(),
        'glass': mat_glass(),
        'lens': mat_lens(),
        'rubber': mat_rubber(),
        'leather': mat_leather('Leather_Seat', (0.042, 0.038, 0.037), scale=9.0, roughness=0.62),
        'carpet': mat_leather('Carpet', (0.025, 0.022, 0.020), scale=42.0, roughness=0.85),
        'headliner': mat_leather('Headliner_Fabric', (0.07, 0.065, 0.06),
                                 scale=55.0, roughness=0.90),
        'headlight': mat_emission('Headlight_Light', 1.4, (1.0, 0.96, 0.88)),
        'drl': mat_emission('DRL_Light', 4.2, (0.94, 0.97, 1.0)),
        'tail': mat_emission('Tail_Light', 3.4, (1.0, 0.055, 0.045)),
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

    # 2. Алюминиевая силовая балка усилителя переднего бампера (Crash_Beam — видна за бампером)
    bm_cb = bmesh.new()
    add_box(bm_cb, 2.318, 2.375, -0.60, 0.60, 0.325, 0.435)
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
    add_box(bm_rs, 2.13, 2.245, -0.62, 0.62, 0.725, 0.785)
    # Нижняя траверса под радиатор
    add_box(bm_rs, 2.10, 2.22, -0.54, 0.54, 0.245, 0.290)
    # Боковые вертикальные стойки «телевизора» вокруг радиатора
    for side in (1.0, -1.0):
        ya, yb = (side * 0.46, side * 0.54)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        add_box(bm_rs, 2.12, 2.21, y0, y1, 0.292, 0.723)
    # Центральный замок капота на верхней балке
    add_box(bm_rs, 2.16, 2.24, -0.055, 0.055, 0.787, 0.818)
    # Боковые опорные кронштейны/ниши крепления передних фар на кузове (фары сидят на кузове!)
    for side in (1.0, -1.0):
        ya, yb = (side * 0.435, side * 0.715)
        y0, y1 = (ya, yb) if ya < yb else (yb, ya)
        # Основная монтажная полка-ниша под фарой (опирается на рамку радиатора и брызговик)
        add_box(bm_rs, 2.18, 2.345, y0, y1, 0.528, 0.548)
        # Задняя опорная стенка чаши фары
        add_box(bm_rs, 2.18, 2.240, y0, y1, 0.550, 0.685)
        # Верхнее крепёжное ухо фары к верхнему углу «телевизора»
        add_box(bm_rs, 2.21, 2.300, y0 + 0.03, y1 - 0.03, 0.688, 0.708)
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
            z_hood_edge = hood_deck_z(x, y_hood_edge) - 0.016
            y_flange = WR(x) + 0.004     # загнутая внутрь монтажная полка под капотом
            z_flange = z_hood_edge - 0.036

            # 12 точек по профилю сечения крыла:
            # 0..6 — борт от нижней кромки/арки до плеча BELT;
            # 7..9 — скруглённый верхний гребень крыла к линии капота;
            # 10..11 — загнутый внутрь монтажный фланец крыла (виден при открытом капоте!)
            t_sill = clamp((1.24 - x) / 0.18, 0.0, 1.0)
            t_sill = t_sill * t_sill * (3.0 - 2.0 * t_sill)
            profile = []
            for j in range(7):
                u = j / 6.0
                zz = zb_f + (belt_x - zb_f) * u
                # Плавный подгиб нижней юбки порога внутрь без резкого скачка у арки
                sill_tuck = 1.0 - 0.022 * t_sill * ((1.0 - u) ** 2.4)
                yy = side_surface_y(x, zz) * sill_tuck
                profile.append((yy, zz))
            for k in range(1, 4):
                u = k / 3.0
                yy = y_belt * (1.0 - u) + y_hood_edge * u
                zz = belt_x + (z_hood_edge - belt_x) * (u ** 0.72) + 0.010 * math.sin(math.pi * u)
                profile.append((yy, zz))
            # Загнутый внутрь фланец крепления крыла на брызговике
            profile.append((y_hood_edge - 0.008, z_flange + 0.010))
            profile.append((y_flange, z_flange))

            row_out = []
            row_in = []
            for idx_p, (yy, zz) in enumerate(profile):
                if idx_p < 7:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS, 0.0))
                elif idx_p < 10:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS * 0.55, -FENDER_THICKNESS * 0.85))
                else:
                    n_off = Vector((0.0, 0.0, -FENDER_THICKNESS * 0.80))
                p_o = Vector((x, side * yy, zz))
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

    # 7. ОТДЕЛЬНЫЙ передний бампер (Front_Bumper): носовой модуль + левое/правое боковые крылышки
    bm_bmp = bmesh.new()
    # 7a. Основной носовой модуль от FRONT_BUMPER_X до X_MAX (с точным 3D-куполом носа)
    nx_b = 12
    b_xs = [FRONT_BUMPER_X + (TIP_F - FRONT_BUMPER_X) * i / (nx_b - 1) for i in range(nx_b)]
    b_loops = [body_section(x) for x in b_xs]
    vc = len(b_loops[0])
    b_verts = []
    for x, loop in zip(b_xs, b_loops):
        for (y, z) in loop:
            b_verts.append(bm_bmp.verts.new(Vector((x, y, z))))
    for i in range(nx_b - 1):
        b0 = i * vc
        b1 = (i + 1) * vc
        for k in range(vc):
            bm_bmp.faces.new((
                b_verts[b0 + k],
                b_verts[b0 + (k + 1) % vc],
                b_verts[b1 + (k + 1) % vc],
                b_verts[b1 + k],
            ))
    # Задняя стенка носового модуля бампера на FRONT_BUMPER_X (с зазором от «телевизора»)
    zm_b0 = 0.5 * (ZB(FRONT_BUMPER_X) + ZR(FRONT_BUMPER_X))
    back_c = bm_bmp.verts.new(Vector((FRONT_BUMPER_X + 0.015, 0.0, zm_b0)))
    for k in range(vc):
        kn = (k + 1) % vc
        bm_bmp.faces.new((back_c, b_verts[k], b_verts[kn]))

    # Передний 3D-купол бампера по суперэллипсу от TIP_F -> X_MAX
    n_rings_b = 10
    prev_b = [b_verts[(nx_b - 1) * vc + k] for k in range(vc)]
    base_b = b_loops[-1]
    zm_bf = 0.5 * (ZB(TIP_F) + ZR(TIP_F))
    tip_bf = bm_bmp.verts.new(Vector((X_MAX, 0.0, zm_bf)))
    for step in range(1, n_rings_b):
        u = step / float(n_rings_b)
        s = max(0.06, dome_scale(u))
        xr = TIP_F + (X_MAX - TIP_F) * u
        ring = [
            bm_bmp.verts.new(Vector((xr, y * s, zm_bf + (z - zm_bf) * s)))
            for (y, z) in base_b
        ]
        for k in range(vc):
            kn = (k + 1) % vc
            bm_bmp.faces.new((prev_b[k], prev_b[kn], ring[kn], ring[k]))
        prev_b = ring
    for k in range(vc):
        kn = (k + 1) % vc
        bm_bmp.faces.new((tip_bf, prev_b[kn], prev_b[k]))

    # 7b. Левое и правое боковые крылышки бампера (плавно заходят в носовой модуль без ступеньки)
    wing_xs = [
        FRONT_BUMPER_WING_X + (FRONT_BUMPER_X + 0.042 - FRONT_BUMPER_WING_X) * i / 11.0
        for i in range(12)
    ]
    for side in (1.0, -1.0):
        w_out = []
        w_in = []
        for x in wing_xs:
            t_w = clamp((x - FRONT_BUMPER_WING_X) / 0.34, 0.0, 1.0)
            z_lo = ZB(x) + 0.042 + 0.010 * (t_w ** 1.4)
            z_hi = fender_bumper_seam_z(min(x, FENDER_X_FRONT)) - 0.005
            row_o = []
            row_i = []
            for j in range(7):
                u = j / 6.0
                zz = z_lo + (z_hi - z_lo) * u
                # Мягкий подгиб нижней юбки и плавное вхождение в кривизну носа
                chin_tuck = 1.0 - 0.065 * ((1.0 - u) ** 2.0) - 0.012 * (t_w ** 2.0)
                yy = side_surface_y(x, zz) * chin_tuck
                row_o.append(Vector((x, side * yy, zz)))
                row_i.append(Vector((x, side * (yy - BUMPER_THICKNESS), zz)))
            w_out.append(row_o)
            w_in.append(row_i)
        _add_thick_grid(bm_bmp, w_out, w_in)

    # Нижняя соединительная «юбка» под краш-боксом между крылышками бампера
    add_box(bm_bmp, FRONT_BUMPER_WING_X + 0.08, FRONT_BUMPER_X + 0.03, -0.52, 0.52, 0.215, 0.242)

    bumper_ob = bmesh_to_object('Front_Bumper', bm_bmp, material=mats['paint'],
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
            # В зоне багажника (x <= TRUNK_X_FRONT) крыло формирует гребень и водосточный желоб;
            # в зоне заднего стекла (x > TRUNK_X_FRONT) аккуратно уходит под стойку C / форточку.
            t_trunk = clamp((TRUNK_X_FRONT + 0.04 - x) / 0.10, 0.0, 1.0)
            t_trunk = t_trunk * t_trunk * (3.0 - 2.0 * t_trunk)
            y_edge_tr = WR(x) + 0.038  # зазор ~6 мм до крышки багажника (WR + 0.032)
            z_edge_tr = trunk_deck_z(x, y_edge_tr) - 0.015
            y_edge_gh = y_belt - 0.048
            z_edge_gh = belt_x + 0.010

            y_top_edge = y_edge_gh * (1.0 - t_trunk) + y_edge_tr * t_trunk
            z_top_edge = z_edge_gh * (1.0 - t_trunk) + z_edge_tr * t_trunk
            y_flange = y_top_edge - (0.025 + 0.020 * t_trunk)
            z_flange = z_top_edge - (0.018 + 0.020 * t_trunk)

            profile = []
            # Мягкий заворот самого заднего ребра крыла (x < -2.24) в тон скруглению бампера
            t_rcorn = clamp((-2.24 - x) / (-2.24 - REAR_FENDER_X_REAR), 0.0, 1.0)
            y_tuck = 1.0 - 0.028 * (t_rcorn ** 2.0)
            for j in range(7):
                u = j / 6.0
                zz = zb_r + (belt_x - zb_r) * u
                yy = side_surface_y(x, zz) * y_tuck
                profile.append((yy, zz))
            for k in range(1, 4):
                u = k / 3.0
                yy = (y_belt * y_tuck) * (1.0 - u) + y_top_edge * u
                zz = belt_x + (z_top_edge - belt_x) * (u ** 0.72) + 0.008 * math.sin(math.pi * u)
                profile.append((yy, zz))
            # Загнутый внутрь водосточный желоб/фланец по краю проёма багажника
            profile.append((y_top_edge - 0.008, z_flange + 0.010))
            profile.append((y_flange, z_flange))

            row_out = []
            row_in = []
            for idx_p, (yy, zz) in enumerate(profile):
                if idx_p < 7:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS, 0.0))
                elif idx_p < 10:
                    n_off = Vector((0.0, -side * FENDER_THICKNESS * 0.55,
                                    -FENDER_THICKNESS * 0.85))
                else:
                    n_off = Vector((0.0, 0.0, -FENDER_THICKNESS * 0.80))
                p_o = Vector((x, side * yy, zz))
                row_out.append(p_o)
                row_in.append(p_o + n_off)
            outer_pts.append(row_out)
            inner_pts.append(row_in)
        _add_thick_grid(bm_rf, outer_pts, inner_pts)

        # Задний верхний угловой заворот крыла над фонарём (плавно и заподлицо без ступеньки)
        c_out = []
        c_in = []
        ny_c, nz_c = 7, 5
        y_c0 = TAIL_LIGHT_Y_IN - 0.002
        y_c1 = side_surface_y(REAR_FENDER_X_REAR, BELT(REAR_FENDER_X_REAR)) * 0.968 - 0.010
        z_c0 = TAIL_LIGHT_Z_HI + 0.005
        z_c1 = BELT(REAR_FENDER_X_REAR) - 0.010
        for j in range(nz_c):
            vz = j / (nz_c - 1)
            zz = z_c0 + (z_c1 - z_c0) * vz
            row_co = []
            row_ci = []
            for i in range(ny_c):
                vy = i / (ny_c - 1)
                yy = y_c0 + (y_c1 - y_c0) * vy
                xx = rear_tail_surface_x(yy) + 0.016 * (vz ** 1.3)
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

    # 5. ОТДЕЛЬНЫЙ съёмный задний бампер (Rear_Bumper): единая U-образная оболочка
    # с плавно скруглёнными задними углами, строго ниже фонарей и багажника (z <= REAR_BUMPER_TOP_Z)
    bm_rb = bmesh.new()

    # Непрерывный U-путь по периметру бампера с мягким скруглением обоих углов:
    # левое крылышко -> левый радиус угла -> задняя дуга -> правый радиус угла -> правое крылышко
    u_stations = []
    y_corner = side_surface_y(REAR_FENDER_X_REAR, 0.55)
    r_corner = 0.11  # радиус скругления угла заднего бампера в плане XY
    x_wing_end = REAR_FENDER_X_REAR + r_corner

    # 5a. Левое боковое крылышко (+Y) от края арки к началу скругления угла
    n_wing = 6
    for i in range(n_wing):
        t = i / float(n_wing)
        x = REAR_BUMPER_WING_X + (x_wing_end - REAR_BUMPER_WING_X) * t
        z_lo = ZB(x) + 0.036
        z_hi = rear_fender_bumper_seam_z(x) - 0.006
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_stations.append((x, +y_abs, z_lo, z_hi, Vector((0.0, 1.0, 0.0))))

    # 5b. Левый скруглённый угол бампера (плавный поворот от +Y к -X)
    x_back_edge = rear_tail_surface_x(y_corner - r_corner)
    z_hi_seam_end = rear_fender_bumper_seam_z(x_wing_end) - 0.006
    for k in range(5):
        ang = (k / 4.0) * (0.5 * math.pi)
        xx = x_wing_end - (x_wing_end - x_back_edge) * (1.0 - math.cos(ang))
        yy = (y_corner - r_corner) + r_corner * math.cos(ang)
        t_c = k / 4.0
        z_lo = ZB(xx) + 0.036
        z_hi = z_hi_seam_end * (1.0 - t_c) + REAR_BUMPER_TOP_Z * t_c
        n_dir = Vector((-math.sin(ang), math.cos(ang), 0.0)).normalized()
        u_stations.append((xx, +yy, z_lo, z_hi, n_dir))

    # 5c. Задняя скруглённая панель между левым и правым углами
    y_span = y_corner - r_corner
    n_back = 15
    for i in range(1, n_back - 1):
        v = 1.0 - 2.0 * i / float(n_back - 1)
        yy = v * y_span
        xx = rear_tail_surface_x(yy)
        z_lo = ZB(xx) + 0.036
        z_hi = REAR_BUMPER_TOP_Z
        n_dir = Vector((-1.0, 0.32 * v, 0.0)).normalized()
        u_stations.append((xx, yy, z_lo, z_hi, n_dir))

    # 5d. Правый скруглённый угол бампера (плавный поворот от -X к -Y)
    for k in range(5):
        ang = (1.0 - k / 4.0) * (0.5 * math.pi)
        xx = x_wing_end - (x_wing_end - x_back_edge) * (1.0 - math.cos(ang))
        yy = (y_corner - r_corner) + r_corner * math.cos(ang)
        t_c = k / 4.0
        z_lo = ZB(xx) + 0.036
        z_hi = REAR_BUMPER_TOP_Z * (1.0 - t_c) + z_hi_seam_end * t_c
        n_dir = Vector((-math.sin(ang), -math.cos(ang), 0.0)).normalized()
        u_stations.append((xx, -yy, z_lo, z_hi, n_dir))

    # 5e. Правое боковое крылышко (-Y) к краю задней арки
    for i in range(1, n_wing + 1):
        t = i / float(n_wing)
        x = x_wing_end + (REAR_BUMPER_WING_X - x_wing_end) * t
        z_lo = ZB(x) + 0.036
        z_hi = rear_fender_bumper_seam_z(x) - 0.006
        y_abs = side_surface_y(x, 0.5 * (z_lo + z_hi))
        u_stations.append((x, -y_abs, z_lo, z_hi, Vector((0.0, -1.0, 0.0))))

    rb_out = []
    rb_in = []
    nz_rb = 7
    for xx, yy, z_lo, z_hi, n_dir in u_stations:
        row_o = []
        row_i = []
        for j in range(nz_rb):
            u = j / float(nz_rb - 1)
            zz = z_lo + (z_hi - z_lo) * u
            # Мягкий подгиб нижней кромки бампера внутрь (под диффузор)
            tuck = 0.024 * ((1.0 - u) ** 2.2)
            p_o = Vector((xx, yy, zz)) - n_dir * tuck
            p_i = p_o - n_dir * BUMPER_THICKNESS
            row_o.append(p_o)
            row_i.append(p_i)
        rb_out.append(row_o)
        rb_in.append(row_i)
    _add_thick_grid(bm_rb, rb_out, rb_in)

    # Горизонтальная погрузочная полка поверху заднего бампера (уходит под проём багажника)
    sh_out = []
    sh_in = []
    for i in range(13):
        v = -1.0 + 2.0 * i / 12.0
        yy = v * (y_corner - 0.045)
        x_back = rear_tail_surface_x(yy) + 0.014
        x_front = -2.26
        row_so = []
        row_si = []
        for j in range(4):
            u = j / 3.0
            xx = x_back * (1.0 - u) + x_front * u
            row_so.append(Vector((xx, yy, REAR_BUMPER_TOP_Z)))
            row_si.append(Vector((xx, yy, REAR_BUMPER_TOP_Z - BUMPER_THICKNESS)))
        sh_out.append(row_so)
        sh_in.append(row_si)
    _add_thick_grid(bm_rb, sh_out, sh_in)

    rear_bumper = bmesh_to_object('Rear_Bumper', bm_rb, material=mats['paint'],
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
                              smooth=True, collection=collection, auto_smooth_deg=55.0,
                              bevel=0.003, bevel_segs=2, subsurf=1)
    uv_top(roof_ob)
    roof_ob.parent = body_ob
    roof_ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
    objs.append(roof_ob)

    # 2. Несущие стойки кузова A, B, C (левый и правый борт, фастбэк-силуэт Volvo S90)
    for label, side in (('L', 1.0), ('R', -1.0)):
        # 2a. Стойка A: длинный спортивный наклон (~31°) вдоль лобового стекла в крышу
        bm_a = bmesh.new()
        xa_bot = CABIN_CUT_X_FRONT + 0.045
        y_a_sill = side_surface_y(xa_bot, CABIN_SILL_Z + 0.015)
        y_a_jamb = side_surface_y(xa_bot, 0.62)
        st_a = [
            (CABIN_CUT_X_FRONT - 0.005, xa_bot + 0.04, y_a_sill - 0.115, y_a_sill - 0.068,
             CABIN_SILL_Z + 0.015),
            (CABIN_CUT_X_FRONT - 0.005, xa_bot + 0.04, y_a_jamb - 0.105, y_a_jamb - 0.048,
             0.62),
        ]
        for step in range(8):
            t = step / 7.0
            x_center = (WINDSHIELD_X_BOT - 0.04) * (1.0 - t) + (ROOF_X_FRONT + 0.01) * t
            half_x = 0.050 * (1.0 - 0.24 * t)
            y_out, z_val = upper_side_yz(x_center, t)
            if step == 0:
                y_out -= 0.018
            elif step == 7:
                y_out = WR(x_center) + 0.004
                z_val = ZR(x_center) - 0.018
            thick_y = 0.060 * (1.0 - 0.18 * t)
            st_a.append((x_center - half_x, x_center + half_x,
                         y_out - thick_y, y_out, z_val))
        _add_station_beam(bm_a, st_a, side=side)
        pa = bmesh_to_object(f'Pillar_A_{label}', bm_a, material=mats['paint'],
                             smooth=True, collection=collection, auto_smooth_deg=55.0,
                             bevel=0.0025, bevel_segs=2, subsurf=1)
        uv_box(pa, scale=2.0)
        pa.parent = body_ob
        pa.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(pa)

        # 2b. Центральная стойка B: изящная глянцево-чёрная стойка в зоне окон;
        # ниже подоконной линии утоплена внутрь проёма (не просвечивает сквозь закрытые двери)
        bm_b = bmesh.new()
        yb_jamb = side_surface_y(PILLAR_B_X, 0.62)
        st_b = [
            (PILLAR_B_X - PILLAR_B_HALF_X * 0.85, PILLAR_B_X + PILLAR_B_HALF_X * 0.85,
             yb_jamb - 0.11, yb_jamb - 0.048, CABIN_SILL_Z + 0.015),
            (PILLAR_B_X - PILLAR_B_HALF_X * 0.85, PILLAR_B_X + PILLAR_B_HALF_X * 0.85,
             yb_jamb - 0.10, yb_jamb - 0.046, BELT(PILLAR_B_X) - 0.02),
        ]
        for step in range(7):
            t = step / 6.0
            half_x = PILLAR_B_HALF_X * (1.0 - 0.14 * t)
            y_out, z_val = upper_side_yz(PILLAR_B_X, t)
            if step == 0:
                y_out -= 0.036
            elif step == 6:
                y_out = WR(PILLAR_B_X) + 0.004
                z_val = ZR(PILLAR_B_X) - 0.018
            else:
                y_out -= 0.003
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
             yc_jamb - 0.11, yc_jamb - 0.044, CABIN_SILL_Z + 0.015),
            (xc_jamb - 0.12, CABIN_CUT_X_REAR + 0.004,
             yc_jamb - 0.10, yc_jamb - 0.040, 0.62),
        ]
        for step in range(8):
            t = step / 7.0
            x_back = (REAR_GLASS_X_BOT - 0.01) * (1.0 - t) + (ROOF_X_REAR - 0.015) * t
            x_front = (REAR_GLASS_X_BOT + 0.34) * (1.0 - t) + (ROOF_X_REAR + 0.12) * t
            x_mid = 0.5 * (x_back + x_front)
            y_out, z_val = upper_side_yz(x_mid, t)
            if step == 0:
                y_out -= 0.018
            elif step == 7:
                y_out = WR(x_mid) + 0.004
                z_val = ZR(x_mid) - 0.018
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
        y_bot=side_surface_y(WINDSHIELD_X_BOT, BELT(WINDSHIELD_X_BOT)) * 0.96,
        x_top=ROOF_X_FRONT + 0.012,
        z_top=ZR(ROOF_X_FRONT) - 0.018,
        y_top=WR(ROOF_X_FRONT) * 0.94,
        bulge_x=+0.024,
    )

    # Заднее стекло (между стойками C, от задней полки перед багажником до кромки крыши)
    _add_curved_pane(
        x_bot=REAR_GLASS_X_BOT,
        z_bot=BELT(REAR_GLASS_X_BOT) + 0.024,
        y_bot=side_surface_y(REAR_GLASS_X_BOT, BELT(REAR_GLASS_X_BOT)) * 0.88,
        x_top=ROOF_X_REAR - 0.005,
        z_top=ZR(ROOF_X_REAR) - 0.018,
        y_top=WR(ROOF_X_REAR) * 0.93,
        bulge_x=-0.022,
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
                y_o = side * (y_abs + 0.006)
                y_i = side * (y_abs + 0.001)
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

    # 1. Тороидальная шина со скруглёнными боковинами (полая внутри под спицы)
    bm_tire = bmesh.new()
    r_out = WHEEL_R
    r_sh = WHEEL_R - 0.015
    r_sh2 = WHEEL_R - 0.036
    r_mid = 0.5 * (r_bead + r_out)
    tire_profile = [
        (r_bead, -hw * 0.88),
        (r_mid, -hw * 1.01),
        (r_sh2, -hw * 1.00),
        (r_sh, -hw * 0.94),
        (r_out, -hw * 0.76),
        (r_out, +hw * 0.76),
        (r_sh, +hw * 0.94),
        (r_sh2, +hw * 1.00),
        (r_mid, +hw * 1.01),
        (r_bead, +hw * 0.88),
    ]
    _add_y_annulus(bm_tire, r_bead, r_out, -hw, hw, seg=seg, profile=tire_profile)
    tire = bmesh_to_object(f'Wheel_{tag}', bm_tire, material=mats['rubber'],
                           smooth=True, collection=collection, auto_smooth_deg=60.0,
                           subsurf=1)
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
                          smooth=True, collection=collection, auto_smooth_deg=50.0,
                          bevel=0.0018, bevel_segs=2)
    uv_box(rim, scale=4.0)
    rim.parent = tire
    rim.location = Vector((0.0, 0.0, 0.0))

    # 3. Тормозной диск (вращается вместе с колесом, виден сквозь спицы)
    bm_disc = bmesh.new()
    y_disc_c = side * (hw * 0.14)
    _add_y_cylinder(bm_disc, 0.172, y_disc_c - 0.011, y_disc_c + 0.011,
                    seg=44, center_xz=(0.0, 0.0))
    disc = bmesh_to_object(f'Brake_Disc_{tag}', bm_disc, material=mats['brake_disc'],
                           smooth=True, collection=collection, auto_smooth_deg=45.0,
                           bevel=0.0015, bevel_segs=1)
    uv_box(disc, scale=5.0)
    disc.parent = tire
    disc.location = Vector((0.0, 0.0, 0.0))

    # 4. Тормозной суппорт (привязан к кузову, остаётся неподвижным при вращении колеса)
    bm_cal = bmesh.new()
    cal_x = -0.142 if axle_x > 0 else +0.142  # на передней оси сзади, на задней спереди
    add_box(bm_cal, cal_x - 0.036, cal_x + 0.036,
            y_disc_c - 0.028, y_disc_c + 0.028, -0.055, 0.068)
    caliper = bmesh_to_object(f'Brake_Caliper_{tag}', bm_cal, material=mats['caliper'],
                              smooth=True, collection=collection, auto_smooth_deg=55.0,
                              bevel=0.0045, bevel_segs=2)
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
    # Задняя полка под покатым задним стеклом (не выступает в нишу багажника TRUNK_X_FRONT)
    add_box(bm, TRUNK_X_FRONT + 0.04, -0.96, -0.56, 0.56, 0.96, 1.00)
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

        # 2. Верхняя оконная рамка двери (повторяет купеобразный наклон стоек A/B/C и крыши)
        if is_front:
            xf_bot, xf_top = x_f, ROOF_X_FRONT + 0.015
            xb_bot, xb_top = x_b + 0.014, PILLAR_B_X + PILLAR_B_HALF_X + 0.004
        else:
            xf_bot, xf_top = x_f - 0.014, PILLAR_B_X - PILLAR_B_HALF_X - 0.004
            xb_bot, xb_top = x_b - 0.010, ROOF_X_REAR + 0.060

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
                                  smooth=True, collection=collection, auto_smooth_deg=55.0,
                                  bevel=0.0028, bevel_segs=2, subsurf=1)
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
                                smooth=True, collection=collection, auto_smooth_deg=60.0,
                                subsurf=1)
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

        # 5. Наружная дверная ручка (привязана к двери, повторяет side_surface_y)
        bm_hdl = bmesh.new()
        x_hdl_w = hx - door_len + 0.18
        z_hdl_w = BELT(x_hdl_w) - 0.115
        y_surf_w = side * side_surface_y(x_hdl_w, z_hdl_w)
        xh_l = x_hdl_w - hx
        yh_l = y_surf_w - hinge_y
        zh_l = z_hdl_w - hinge_z
        # Стойки ручки + вытянутая рукоятка с зазором от полотна двери
        add_box(bm_hdl, xh_l - 0.082, xh_l + 0.082,
                yh_l + side * 0.014, yh_l + side * 0.028, zh_l - 0.012, zh_l + 0.012)
        add_box(bm_hdl, xh_l - 0.075, xh_l - 0.052,
                yh_l - side * 0.002, yh_l + side * 0.012, zh_l - 0.010, zh_l + 0.010)
        add_box(bm_hdl, xh_l + 0.052, xh_l + 0.075,
                yh_l - side * 0.002, yh_l + side * 0.012, zh_l - 0.010, zh_l + 0.010)
        handle_name = name.replace('Door_', 'Door_Handle_')
        hdl_ob = bmesh_to_object(handle_name, bm_hdl, material=mats['chrome'],
                                 smooth=True, collection=collection, auto_smooth_deg=55.0,
                                 bevel=0.0022, bevel_segs=2)
        uv_box(hdl_ob, scale=10.0)
        hdl_ob.parent = door_ob
        hdl_ob.location = Vector((0.0, 0.0, 0.0))

        # 6. Обтекаемое каплевидное зеркало заднего вида (устанавливается на передние двери)
        if is_front:
            m_tag = 'L' if side > 0 else 'R'
            xm_w = hx - 0.16
            zm_w = BELT(xm_w) - 0.015
            ym_w = side * side_surface_y(xm_w, zm_w)
            xm_l = xm_w - hx
            ym_l = ym_w - hinge_y
            zm_l = zm_w - hinge_z
            bm_mir = bmesh.new()
            # Скошенная аэродинамическая ножка на двери (как у Volvo S90 — на полотне двери)
            stalk_pts = [
                Vector((xm_l, ym_l - side * 0.005, zm_l)),
                Vector((xm_l - 0.015, ym_l + side * 0.032, zm_l + 0.028)),
                Vector((xm_l - 0.028, ym_l + side * 0.055, zm_l + 0.052)),
            ]
            _sweep_rect_tube(bm_mir, stalk_pts, half_y=0.014, half_n=0.014)
            # Каплевидный корпус зеркала (5 сечений вдоль Y от ножки к наружному кончику)
            m_rings = []
            m_specs = [
                (0.048, -0.040, +0.048, +0.028, +0.088),
                (0.080, -0.055, +0.052, +0.022, +0.098),
                (0.120, -0.052, +0.038, +0.026, +0.094),
                (0.155, -0.042, +0.015, +0.034, +0.084),
                (0.172, -0.028, -0.005, +0.044, +0.072),
            ]
            for dy_off, dx0, dx1, dz0, dz1 in m_specs:
                yy = ym_l + side * dy_off
                if side > 0:
                    ring = [
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz1))),
                    ]
                else:
                    ring = [
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz0))),
                        bm_mir.verts.new(Vector((xm_l + dx0, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz1))),
                        bm_mir.verts.new(Vector((xm_l + dx1, yy, zm_l + dz0))),
                    ]
                m_rings.append(ring)
            for i_m in range(len(m_rings) - 1):
                r0, r1 = m_rings[i_m], m_rings[i_m + 1]
                for k in range(4):
                    kn = (k + 1) % 4
                    bm_mir.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
            bm_mir.faces.new((m_rings[0][0], m_rings[0][3], m_rings[0][2], m_rings[0][1]))
            bm_mir.faces.new((m_rings[-1][0], m_rings[-1][1], m_rings[-1][2], m_rings[-1][3]))

            mir_ob = bmesh_to_object(f'Mirror_{m_tag}', bm_mir, material=mats['paint'],
                                     smooth=True, collection=collection, auto_smooth_deg=55.0,
                                     bevel=0.0025, bevel_segs=2, subsurf=1)
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
            edge_drop = 0.018 * (abs(v) ** 3.0) * (1.0 - 0.5 * t_taper)
            z_top = trunk_deck_z(x, y) - edge_drop + 0.002
            p_out = Vector((x - TRUNK_HINGE_X, y, z_top - hz))
            p_in = Vector((x - TRUNK_HINGE_X, y, z_top - TRUNK_LID_THICKNESS - hz))
            row_out.append(bm_lid.verts.new(p_out))
            row_in.append(bm_lid.verts.new(p_in))
        outer.append(row_out)
        inner.append(row_in)

    # Полновысотная вертикальная задняя панель крышки багажника (между фонарями до бампера!)
    z_bot_lid = REAR_BUMPER_TOP_Z + 0.008  # зазор 8 мм над верхней полкой заднего бампера
    n_drop = 7
    lip_top_z = []
    lip_x_by_j = []
    for j in range(ny):
        v = -1.0 + 2.0 * j / (ny - 1)
        y = v * TRUNK_TAIL_HALF_Y
        xb = rear_tail_surface_x(y)
        lip_x_by_j.append(xb)
        lip_top_z.append(trunk_deck_z(xb, y) - 0.009 * (abs(v) ** 3.0) + 0.002)

    for step in range(1, n_drop + 1):
        frac = step / float(n_drop)
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * TRUNK_TAIL_HALF_Y
            z_cur = lip_top_z[j] * (1.0 - frac) + z_bot_lid * frac
            # Лёгкий характерный наклон/подгиб кормы Volvo S90 (ниша номерного знака)
            x_cur = lip_x_by_j[j] - 0.012 * math.sin(math.pi * frac) + 0.010 * frac
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
    # Декоративная пластиковая плита Volvo Drive-E с 4 продольными рёбрами жёсткости
    add_box(bm_eng, 1.44, 1.81, -0.32, 0.06, 0.642, 0.695)
    for k_rib in range(4):
        ry = -0.26 + k_rib * 0.095
        add_box(bm_eng, 1.46, 1.79, ry - 0.022, ry + 0.022, 0.698, 0.715)
    # Маслозаливная горловина на передней части крышки двигателя
    _add_z_cylinder(bm_eng, 0.028, 0.696, 0.732, seg=14, center_xy=(1.74, -0.24))

    # Основной радиатор + кожух электровентилятора сразу за «телевизором»
    add_box(bm_eng, 2.02, 2.11, -0.45, 0.45, 0.295, 0.715)
    # Верхний и нижний резиновые патрубки ОЖ (радиатор <-> двигатель)
    add_box(bm_eng, 1.82, 2.018, -0.30, -0.24, 0.61, 0.655)
    add_box(bm_eng, 1.82, 2.018, 0.00, 0.05, 0.34, 0.385)

    # Корпус воздушного фильтра (Airbox, слева +Y) + холодный впуск к «телевизору» и гофра к ДВС
    add_box(bm_eng, 1.66, 1.98, 0.25, 0.45, 0.44, 0.675)
    add_box(bm_eng, 1.982, 2.13, 0.30, 0.42, 0.60, 0.665)  # заборник холодного воздуха
    add_box(bm_eng, 1.56, 1.66, 0.09, 0.32, 0.59, 0.655)   # гофра впуска от фильтра к ДВС

    # Короб АКБ и блок предохранителей у моторного щита (слева +Y)
    add_box(bm_eng, 1.18, 1.42, 0.27, 0.47, 0.39, 0.665)
    add_box(bm_eng, 1.44, 1.62, 0.32, 0.47, 0.44, 0.635)   # блок реле и предохранителей

    # Бачок омывателя стекла у правого брызговика (-Y) с заливной горловиной
    add_box(bm_eng, 1.78, 1.98, -0.47, -0.35, 0.36, 0.635)
    _add_z_cylinder(bm_eng, 0.022, 0.636, 0.685, seg=12, center_xy=(1.90, -0.41))

    engine_bay = bmesh_to_object('Engine_Bay', bm_eng, material=mats['carbon'],
                                 smooth=True, collection=collection, bevel=0.003)
    uv_box(engine_bay, scale=2.5)
    _attach_body(engine_bay)

    # 1b. Металлические узлы моторного отсека (Engine_Metal — алюминий: КПП, коллектор, опоры)
    bm_em = bmesh.new()
    # Корпус поперечной 8-ступенчатой КПП (слева от блока ДВС)
    add_box(bm_em, 1.46, 1.78, 0.085, 0.295, 0.31, 0.575)
    # Алюминиевый впускной ресивер и 4 патрубка перед блоком цилиндров
    add_box(bm_em, 1.845, 1.94, -0.28, 0.04, 0.54, 0.63)
    for k_run in range(4):
        ry = -0.24 + k_run * 0.082
        add_box(bm_em, 1.81, 1.843, ry - 0.018, ry + 0.018, 0.57, 0.625)
    # Теплозащитный экран турбины/выпуска у моторного щита
    add_box(bm_em, 1.24, 1.39, -0.28, 0.04, 0.44, 0.635)
    # Верхние опорные фланцы передних амортизационных стоек (с центральной гайкой)
    for side in (1.0, -1.0):
        _add_z_cylinder(bm_em, 0.072, 0.792, 0.816, seg=18, center_xy=(AXLE_F, side * 0.54))
        _add_z_cylinder(bm_em, 0.018, 0.817, 0.834, seg=12, center_xy=(AXLE_F, side * 0.54))
    # Алюминиевая поперечная распорка чашек стоек (перед жабо)
    add_box(bm_em, 1.28, 1.32, -0.50, 0.50, 0.745, 0.772)
    em_ob = bmesh_to_object('Engine_Metal', bm_em, material=mats['alloy'],
                            smooth=True, collection=collection, bevel=0.0025)
    uv_box(em_ob, scale=3.5)
    _attach_body(em_ob)

    # 1c. Полупрозрачный/светлый расширительный бачок ОЖ (Coolant_Tank — правый брызговик)
    bm_ct = bmesh.new()
    add_box(bm_ct, 1.26, 1.46, -0.47, -0.33, 0.55, 0.675)
    _add_z_cylinder(bm_ct, 0.026, 0.676, 0.708, seg=14, center_xy=(1.36, -0.40))
    ct_ob = bmesh_to_object('Coolant_Tank', bm_ct, material=mats['white'],
                            smooth=True, collection=collection, bevel=0.003)
    uv_box(ct_ob, scale=6.0)
    _attach_body(ct_ob)

    # 2. Отделённая крышка капота (origin в оси задней петли, вращается вокруг -Y)
    # Чёткие 6-мм зазоры до жабо (HOOD_X_REAR+0.006), крыльев (WR+0.044) и бампера (FRONT_BUMPER_X)
    bm_hood = bmesh.new()
    nx = 18
    ny = 11
    x_start = HOOD_X_REAR + 0.006
    x_end = FRONT_BUMPER_X - 0.006
    xs = [x_start + (x_end - x_start) * i / (nx - 1) for i in range(nx)]

    outer_pts = []
    inner_pts = []
    for x in xs:
        hw = WR(x) + 0.044
        row_out = []
        row_in = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * hw
            edge_drop = 0.018 * (abs(v) ** 3.0)
            z_top = hood_deck_z(x, y) - edge_drop + 0.002
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
              aim_target=None, color=(1.0, 1.0, 1.0), collection=None):
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
        # Для нижней кромки бампера (z < 0.34) сэмплируем чуть выше, чтобы луч не промахивался
        z_ray = max(z, 0.34)
        hit, _ = body_hit(y, z_ray, front=front)
        if hit is not None:
            if front and hit.x > X_MAX - 0.32:
                # Лёгкий естественный подгиб нижней губы назад (chin tuck)
                tuck = max(0.0, 0.34 - z) * 0.35
                return hit.x - tuck
            if (not front) and hit.x < X_MIN + 0.32:
                tuck = max(0.0, 0.34 - z) * 0.35
                return hit.x + tuck
        curve = 0.14 * (abs(y) / 0.78) ** 2.2
        return (X_MAX - 0.025 - curve) if front else (X_MIN + 0.025 + curve)

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

    # 1. Фирменная решётка радиатора Volvo S90 (вынесена на передний бампер, контурная!)
    gw, gz0, gz1 = 0.40, 0.44, 0.69
    bm_gr = bmesh.new()
    _add_curved_strip(bm_gr, -gw, gw, gz0, gz1, front=True,
                      off_out=0.020, thick=0.028, ny=13, nz=5, concave=0.007)
    grille_ob = bmesh_to_object('Grille_Front', bm_gr, material=mats['piano_black'],
                                smooth=True, collection=collection, bevel=0.0025)
    uv_box(grille_ob, scale=4.0)
    objs.append(grille_ob)

    # Хромированная рамка, 15 вертикальных вогнутых ламелей и диагональ Volvo Iron Mark
    bm_gc = bmesh.new()
    ft = 0.015
    # Верхняя и нижняя контурные перекладины рамки
    _add_curved_strip(bm_gc, -gw - ft, gw + ft, gz1, gz1 + ft, front=True,
                      off_out=0.026, thick=0.018, ny=11, nz=2)
    _add_curved_strip(bm_gc, -gw - ft, gw + ft, gz0 - ft, gz0, front=True,
                      off_out=0.026, thick=0.018, ny=11, nz=2)
    # Левая и правая вертикальные стойки рамки (с зазором 2 мм -> 0 non-manifold)
    _add_curved_strip(bm_gc, gw + 0.002, gw + ft, gz0 + 0.002, gz1 - 0.002, front=True,
                      off_out=0.026, thick=0.018, ny=2, nz=5)
    _add_curved_strip(bm_gc, -gw - ft, -gw - 0.002, gz0 + 0.002, gz1 - 0.002, front=True,
                      off_out=0.026, thick=0.018, ny=2, nz=5)
    # 15 вертикальных хромированных ламелей (слегка вогнутых к центру, строго перед чёрной основой)
    n_slats = 15
    for i in range(n_slats):
        u = -1.0 + 2.0 * i / (n_slats - 1)
        yy = u * (gw - 0.022)
        sx = _surf_x(yy, 0.5 * (gz0 + gz1), front=True)
        conc = 0.007 * (1.0 - u * u)
        add_box(bm_gc, sx + 0.006 - conc, sx + 0.023 - conc,
                yy - 0.0035, yy + 0.0035, gz0 + 0.006, gz1 - 0.006)
    # Гладкая наклонная диагональная полоса Volvo Iron Mark (единая наклонная балка без «лесенки»)
    z_gmid = 0.5 * (gz0 + gz1)
    d_pts = []
    for k in range(9):
        t_m = k / 8.0
        y_m = -0.35 * (1.0 - t_m) + 0.35 * t_m
        z_m = (gz0 + 0.014) * (1.0 - t_m) + (gz1 - 0.014) * t_m
        sx = _surf_x(y_m, z_m, front=True) + 0.024
        d_pts.append(Vector((sx, y_m, z_m)))
    # Замкнутый наклонный брусок вдоль d_pts (сечение в плоскости X × перпендикуляр в YZ)
    diag_dir = (d_pts[-1] - d_pts[0]).normalized()
    perp_yz = Vector((0.0, -diag_dir.z, diag_dir.y)).normalized()
    d_rings = []
    for p in d_pts:
        d_rings.append([
            bm_gc.verts.new(p - Vector((0.008, 0.0, 0.0)) - perp_yz * 0.007),
            bm_gc.verts.new(p + Vector((0.008, 0.0, 0.0)) - perp_yz * 0.007),
            bm_gc.verts.new(p + Vector((0.008, 0.0, 0.0)) + perp_yz * 0.007),
            bm_gc.verts.new(p - Vector((0.008, 0.0, 0.0)) + perp_yz * 0.007),
        ])
    for i_d in range(len(d_rings) - 1):
        r0, r1 = d_rings[i_d], d_rings[i_d + 1]
        for k in range(4):
            kn = (k + 1) % 4
            bm_gc.faces.new((r0[k], r0[kn], r1[kn], r1[k]))
    bm_gc.faces.new((d_rings[0][0], d_rings[0][3], d_rings[0][2], d_rings[0][1]))
    bm_gc.faces.new((d_rings[-1][0], d_rings[-1][1], d_rings[-1][2], d_rings[-1][3]))

    # Круглый 16-гранный центральный знак Volvo Iron Mark
    sx_c = _surf_x(0.0, z_gmid, front=True)
    seg_em = 24
    r_em = 0.044
    em_f = []
    em_b = []
    for i_e in range(seg_em):
        ang = 2.0 * math.pi * i_e / seg_em
        ey = r_em * math.cos(ang)
        ez = z_gmid + r_em * math.sin(ang)
        em_f.append(bm_gc.verts.new(Vector((sx_c + 0.035, ey, ez))))
        em_b.append(bm_gc.verts.new(Vector((sx_c + 0.019, ey, ez))))
    c_f = bm_gc.verts.new(Vector((sx_c + 0.037, 0.0, z_gmid)))
    c_b = bm_gc.verts.new(Vector((sx_c + 0.018, 0.0, z_gmid)))
    for i_e in range(seg_em):
        jn = (i_e + 1) % seg_em
        bm_gc.faces.new((em_b[i_e], em_f[i_e], em_f[jn], em_b[jn]))
        bm_gc.faces.new((c_f, em_f[i_e], em_f[jn]))
        bm_gc.faces.new((c_b, em_b[jn], em_b[i_e]))
    gc_ob = bmesh_to_object('Grille_Chrome', bm_gc, material=mats['chrome'],
                            smooth=True, collection=collection)
    uv_box(gc_ob, scale=6.0)
    objs.append(gc_ob)

    # 2. Контурный нижний воздухозаборник переднего бампера (аккуратно утоплен в бампер!)
    bm_in = bmesh.new()
    _add_curved_strip(bm_in, -0.56, 0.56, 0.255, 0.365, front=True,
                      off_out=0.004, thick=0.022, ny=15, nz=4)
    # Аккуратная нижняя губа-сплиттер строго в габаритах носа (без выпирающих углов)
    _add_curved_strip(bm_in, -0.48, 0.48, 0.215, 0.242, front=True,
                      off_out=0.006, thick=0.036, ny=13, nz=2)
    intake_ob = bmesh_to_object('Bumper_Intake_Front', bm_in, material=mats['carbon'],
                                smooth=True, collection=collection, bevel=0.0025)
    uv_box(intake_ob, scale=3.0)
    objs.append(intake_ob)

    # 3. Огибающие блок-фары Volvo «Молот Тора» — жёстко установлены на каркасе кузова
    # (Radiator_Support): глубокий корпус уходит внутрь кузова до x = 2.24 без зазора!
    for label, sy in (('L', 1.0), ('R', -1.0)):
        y_in, y_out = sy * 0.43, sy * 0.74
        ya, yb = (y_in, y_out) if y_in < y_out else (y_out, y_in)
        z_lo, z_hi = 0.545, 0.680

        # 3a. Глубокий глянцево-чёрный корпус блок-фары (от линзы до кронштейна кузова x=2.24)
        bm_hl = bmesh.new()
        outer_h = []
        inner_h = []
        ny_h, nz_h = 11, 5
        for j in range(nz_h):
            vz = j / (nz_h - 1)
            zz = z_lo + (z_hi - z_lo) * vz
            row_o = []
            row_i = []
            for i in range(ny_h):
                vy = i / (ny_h - 1)
                yy = ya + (yb - ya) * vy
                sx = _surf_x(yy, zz, front=True) + 0.007
                # Задняя стенка блок-фары садится в опорную чашу Radiator_Support (x = 2.255)
                row_o.append(Vector((sx, yy, zz)))
                row_i.append(Vector((2.255, yy * 0.96, zz)))
            outer_h.append(row_o)
            inner_h.append(row_i)
        _add_thick_grid(bm_hl, outer_h, inner_h)
        hl_ob = bmesh_to_object(f'Headlight_{label}', bm_hl, material=mats['piano_black'],
                                smooth=True, collection=collection, bevel=0.0025)
        uv_box(hl_ob, scale=5.0)
        objs.append(hl_ob)

        # 3b. Светодиодная подпись «Молот Тора» (контурная, повторяет кривизну фары!)
        bm_drl = bmesh.new()
        z_mid = 0.5 * (z_lo + z_hi)

        def _drl_curved(u0, u1, dz0, dz1):
            py0 = y_in + (y_out - y_in) * u0
            py1 = y_in + (y_out - y_in) * u1
            p_lo, p_hi = (py0, py1) if py0 < py1 else (py1, py0)
            _add_curved_strip(bm_drl, p_lo, p_hi, z_mid + dz0, z_mid + dz1,
                              front=True, off_out=0.015, thick=0.008, ny=7, nz=2)

        # Длинный ствол молота Тора (уходит к переднему крылу)
        _drl_curved(0.18, 0.94, -0.009, +0.009)
        # Вертикальная головка молота у решётки радиатора
        _drl_curved(0.06, 0.16, -0.042, +0.042)
        # Верхний и нижний «усик» головки молота
        _drl_curved(0.06, 0.26, +0.044, +0.055)
        _drl_curved(0.06, 0.26, -0.055, -0.044)
        drl_ob = bmesh_to_object(f'Headlight_DRL_{label}', bm_drl,
                                 material=mats['drl'], smooth=True,
                                 collection=collection, bevel=0.0015, bevel_segs=1)
        uv_box(drl_ob, scale=8.0)
        objs.append(drl_ob)

        # 3c. Прозрачный огибающий рассеиватель (линза) поверх фары
        bm_ln = bmesh.new()
        _add_curved_strip(bm_ln, ya - 0.004, yb + 0.004, z_lo - 0.004, z_hi + 0.004,
                          front=True, off_out=0.020, thick=0.005, ny=11, nz=5)
        ln_ob = bmesh_to_object(f'Headlight_Lens_{label}', bm_ln, material=mats['lens'],
                                smooth=True, collection=collection, bevel=0.002, bevel_segs=2)
        uv_box(ln_ob, scale=5.0)
        objs.append(ln_ob)

    # 4. Огибающие C-образные задние блок-фонари Volvo S90 — установлены строго НА КУЗОВЕ
    # (Rear_Subframe): снаружи от крышки багажника (|y| >= TAIL_LIGHT_Y_IN) и выше бампера!
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
        z_lo_t, z_hi_t = TAIL_LIGHT_Z_LO, TAIL_LIGHT_Z_HI
        z_mid = 0.5 * (z_lo_t + z_hi_t)

        # 4a. Компактный тёмно-рубиновый корпус заднего блок-фонаря (глубина 48 мм —
        # сидит в наружной нише Rear_Subframe и НЕ заходит глубоко внутрь багажника!)
        bm_th = bmesh.new()
        th_out = []
        th_in = []
        ny_t, nz_t = 11, 5
        for j in range(nz_t):
            vz = j / (nz_t - 1)
            zz = z_lo_t + (z_hi_t - z_lo_t) * vz
            row_o = []
            row_i = []
            for i in range(ny_t):
                vy = i / (ny_t - 1)
                yy = ya_t + (yb_t - ya_t) * vy
                sx = rear_tail_surface_x(yy) - 0.003
                # Компактная глубина 48 мм (опирается на Rear_Subframe, не торчит в багажник)
                row_o.append(Vector((sx, yy, zz)))
                row_i.append(Vector((sx + 0.048, yy, zz)))
            th_out.append(row_o)
            th_in.append(row_i)
        _add_thick_grid(bm_th, th_out, th_in)
        th_ob = bmesh_to_object(f'Tail_Housing_{label}', bm_th, material=mats['tail_housing'],
                                smooth=True, collection=collection, bevel=0.0025)
        uv_box(th_ob, scale=5.0)
        objs.append(th_ob)

        # 4b. C-образные светодиодные секции Volvo S90
        bm_tl = bmesh.new()

        def _tl_curved(u0, u1, dz0, dz1):
            py0 = y_in + (y_out - y_in) * u0
            py1 = y_in + (y_out - y_in) * u1
            p_lo, p_hi = (py0, py1) if py0 < py1 else (py1, py0)
            _add_tail_curved_strip(bm_tl, p_lo, p_hi, z_mid + dz0, z_mid + dz1,
                                   off_out=0.009, thick=0.006, ny=8, nz=2)

        _tl_curved(0.03, 0.94, +0.066, +0.092)  # верхняя горизонталь
        _tl_curved(0.83, 0.94, -0.082, +0.062)  # наружная вертикальная стойка на углу кормы
        _tl_curved(0.05, 0.94, -0.096, -0.070)  # нижняя горизонталь
        _tl_curved(0.14, 0.78, -0.011, +0.011)  # центральная LED-полоса стоп-сигнала
        tl_ob = bmesh_to_object(f'Tail_Light_{label}', bm_tl,
                                material=mats['tail'], smooth=True,
                                collection=collection, bevel=0.0015, bevel_segs=1)
        uv_box(tl_ob, scale=6.0)
        objs.append(tl_ob)

        # 4c. Прозрачная внешняя линза заднего фонаря (чистый 5 мм зазор от LED без z-fighting)
        bm_tln = bmesh.new()
        _add_tail_curved_strip(bm_tln, ya_t - 0.003, yb_t + 0.003,
                               z_lo_t - 0.003, z_hi_t + 0.003,
                               off_out=0.019, thick=0.005, ny=11, nz=5)
        tln_ob = bmesh_to_object(f'Tail_Lens_{label}', bm_tln, material=mats['lens'],
                                 smooth=True, collection=collection, bevel=0.0012)
        uv_box(tln_ob, scale=5.0)
        objs.append(tln_ob)

    # 5. Контурный задний диффузор (Rear_Diffuser) и двойные встроенные патрубки (Exhaust_L/R)
    bm_rd = bmesh.new()
    _add_tail_curved_strip(bm_rd, -0.58, 0.58, 0.245, 0.355,
                           off_out=-0.002, thick=0.026, ny=15, nz=4)
    rd_ob = bmesh_to_object('Rear_Diffuser', bm_rd, material=mats['carbon'],
                            smooth=True, collection=collection, bevel=0.0025)
    uv_box(rd_ob, scale=3.0)
    objs.append(rd_ob)

    for label, sy in (('L', 1.0), ('R', -1.0)):
        ey = sy * 0.44
        ex_s = rear_tail_surface_x(ey) + 0.004
        bm_ex = bmesh.new()
        # Трапециевидный хромированный патрубок, утопленный заподлицо в диффузор
        add_box(bm_ex, ex_s - 0.012, ex_s + 0.036, ey - 0.070, ey + 0.070, 0.315, 0.330)
        add_box(bm_ex, ex_s - 0.012, ex_s + 0.036, ey - 0.062, ey + 0.062, 0.262, 0.277)
        add_box(bm_ex, ex_s - 0.012, ex_s + 0.036, ey - 0.072, ey - 0.058, 0.279, 0.313)
        add_box(bm_ex, ex_s - 0.012, ex_s + 0.036, ey + 0.058, ey + 0.072, 0.279, 0.313)
        ex_ob = bmesh_to_object(f'Exhaust_{label}', bm_ex, material=mats['chrome'],
                                smooth=True, collection=collection, bevel=0.0022)
        uv_box(ex_ob, scale=8.0)
        objs.append(ex_ob)

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
        bm_wa = bmesh.new()
        r_lip_in = WHEEL_R + 0.024
        r_lip_mid = WHEEL_R + 0.044
        r_lip_out = WHEEL_R + 0.068
        rings_wa = []
        for i in range(seg_a + 1):
            ang = math.radians(-8.0 + 196.0 * i / seg_a)
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
    add_light('Int_EngineLight', (1.65, 0.0, 1.12), 6.5, size=0.45,
              rot=(0.0, 0.0, 0.0), collection=studio_col)

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
        'Rear_Diffuser': rear_bumper,
        'Exhaust_L': rear_bumper,
        'Exhaust_R': rear_bumper,
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
