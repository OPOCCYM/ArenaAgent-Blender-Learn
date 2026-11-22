#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_build.py — параметрический генератор реалистичного седана (по мотивам Volvo S90)
для Blender 4.2. Работает только внутри Blender:

    blender -b -noaudio -P car_build.py
    blender -b -noaudio -P car_build.py -- --render   # собрать сцену + 5 финальных рендеров

Архитектура (см. PROJECT_MEMORY.md, раздел 2):
  * кузов — лофт по поперечным сечениям + лапласоцевое сглаживание;
  * колёсная арка — гладкая впадина arch_recess() (не прорезь);
  * торцы — купола по суперэллипсу dome() (показатели ~0.34 / 0.72);
  * детали сажаются ray_cast'ом (body_hit / top_hit) со смещением off по нормали;
  * все материалы процедурные, по UV (uv_top для кузова, uv_cyl для колёс).
  * все геометрические тела замкнуты (манефолд) — qa.py должен показать нули.
"""

import os
import sys
import math

import bpy
import bmesh
from mathutils import Vector, Matrix

# --------------------------------------------------------------------------- #
# 1. Габариты и швы (раздел 2 памяти проекта)
# --------------------------------------------------------------------------- #
LENGTH   = 5.00   # длина, м
WIDTH    = 1.872  # ширина, м
HEIGHT   = 1.456  # высота, м
AXLE_F   = 1.60   # передняя ось (м от центра машины)
AXLE_R   = -1.46  # задняя ось  (база 3.06 м)
WHEEL_Y  = 0.792  # половина колеи
WHEEL_R  = 0.353  # радиус шины 235/45 R18
WHEEL_Z  = 0.353  # центр качения по Z (колесо катится по Z = 0)
ARCH_R   = 0.445  # радиус зоны колёсной арки
ARCH_DEPTH = 0.13 # глубина впадины борта у колеса
TIP_DOME_F = 0.15 # длина купола носа
TIP_DOME_R = 0.17 # длина купола кормы
N_SECTIONS = 182  # число сечений вдоль X
V_BODY  = 18      # точек профиля «тела» в сечении
V_GH    = 9       # точек профиля остекления
X_MIN   = -LENGTH / 2.0   # нос (-2.5)
X_MAX   =  LENGTH / 2.0   # корма (+2.5)
TIP_F   = X_MIN + TIP_DOME_F
TIP_R   = X_MAX - TIP_DOME_R

TARGET = Vector((0.05, 0.0, 0.74))  # точка прицеливания всех камер


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------- #
# 2. Ключевые кривые вдоль машины
# --------------------------------------------------------------------------- #
def W(x):
    """Полуширина борта, м."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    base = 0.936 * (1.0 - 0.30 * abs(2.0 * t - 1.0) ** 1.6)
    if x > TIP_R:
        u = (x - TIP_R) / TIP_DOME_R
        base *= (1.0 - u * u)
    if x < TIP_F:
        u = (TIP_F - x) / TIP_DOME_F
        base *= (1.0 - u * u)
    return clamp(base, 0.03, 0.96)


def ZB(x):
    """Низ кузова (дно/нижняя кромка порога)."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return -0.30 - 0.06 * math.sin(math.pi * t)


def BELT(x):
    """Подоконная линия."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 1.02 - 0.22 * abs(2.0 * t - 1.0) ** 1.2


def ZR(x):
    """Крыша."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 1.38 - 0.14 * abs(2.0 * t - 1.0) ** 1.4


def WR(x):
    """Полуширина крыши."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return W(x) * (0.58 + 0.06 * math.sin(math.pi * t))


def TUMBLE(x):
    """Наклон стёкол к вертикали: 0.06 … 0.16."""
    t = (x - X_MIN) / (X_MAX - X_MIN)
    return 0.06 + 0.10 * t * t


def arch_mask(x, z):
    """Поле впадины колёсной арки: 0 — нет впадины, 1 — центр впадины.
    Непрерывно по X и Z, поэтому блики текут ровно."""
    m = 0.0
    for cx in (AXLE_R + 0.35, AXLE_F - 0.35):
        dx = clamp(1.0 - abs(x - cx) / ARCH_R, 0.0, 1.0)
        dz = clamp(1.0 - abs(z - WHEEL_Z) / 0.20, 0.0, 1.0)
        m = max(m, (dx * dz) ** 1.5)
    return m


def dome_scale(u, pe=0.34, px=0.72):
    """Замыкание торца по суперэллипсу (пе ~=0.34, px ~=0.72): сечение долго
    остаётся широким и закругляется только у кромки."""
    u = clamp(u, 0.0, 1.0)
    return 1.0 - u ** (1.0 / pe)


# --------------------------------------------------------------------------- #
# 3. Поперечные сечения кузова
# --------------------------------------------------------------------------- #
def body_half_profile(x):
    """Правый борт (y >= 0): центр дна -> центр крыши, 10 точек."""
    z0 = ZB(x)
    w = W(x)
    belt = BELT(x)
    zr = ZR(x)
    wr = WR(x)
    tum = TUMBLE(x)
    r = arch_mask(x, WHEEL_Z)
    return [
        (0.0, z0),                                                      # 0 центр дна
        (w * 0.44, z0 + 0.05),                                          # 1 нижний борт
        (w * (1.0 - 0.62 * r), WHEEL_Z - 0.03),                         # 2 арка низ
        (w * (1.0 - 0.32 * r), WHEEL_Z + 0.10),                         # 3 арка верх
        (w * 0.86, WHEEL_Z + 0.30),                                     # 4 над аркой
        (w * 0.96, belt - 0.015),                                       # 5 низ двери/рельс
        (w, belt),                                                       # 6 подоконник
        (w - tum * 0.42, belt + 0.28),                                  # 7 верх стёкла (наклон внутрь)
        (wr, zr),                                                        # 8 край крыши
        (0.0, zr),                                                        # 9 центр крыши
    ]


def body_section(x):
    """Полный цикл сечения: 18 точек (правый борт вверх, левый вниз)."""
    hp = body_half_profile(x)
    verts = [(y, z) for (y, z) in hp]                                   # 0..9
    verts += [(-y, z) for (y, z) in reversed(hp[1:9])]                 # 10..17
    return verts  # 18 точек (цикл замыкается автоматически)


def glass_half_profile(x):
    """Пробег остекления: подоконник -> край крыши (внутрь по наклону)."""
    w = W(x)
    belt = BELT(x)
    zr = ZR(x)
    wr = WR(x)
    tum = TUMBLE(x)
    return [
        (w, belt),                                                       # 0 подоконник
        (w - tum * 0.22, belt + 0.13),                                   # 1
        (w - tum * 0.34, belt + 0.22),                                   # 2
        (wr * 0.92, zr - 0.02),                                          # 3 верх стёкла
        (wr, zr),                                                        # 4 край крыши
    ]


def glass_section(x):
    hp = glass_half_profile(x)
    return [(y, z) for (y, z) in hp] + [(-y, z) for (y, z) in reversed(hp[1:4])]


# --------------------------------------------------------------------------- #
# 4. Вспомогательные инструменты геометрии
# --------------------------------------------------------------------------- #
def new_object(name, mesh_data=None):
    if mesh_data is None:
        mesh_data = bpy.data.meshes.new(name)
    ob = bpy.data.objects.new(name, mesh_data)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def bmesh_to_object(name, bm, material=None, smooth=True):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.update()
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
        me.use_auto_smooth = True
        me.auto_smooth_angle = math.radians(30.0)
    ob = new_object(name, me)
    if material is not None:
        ob.data.materials.append(material)
    cleanup(ob, 5e-5)  # дубли вершин / вырожденные грани — в ноль
    return ob


def orient_outward(bm):
    """Переворачивает грани, чья нормаль смотрит к центру кузова."""
    bm.normal_update()
    if not bm.verts:
        return
    center = sum((v.co for v in bm.verts), Vector()) / len(bm.verts)
    for f in list(bm.faces):
        fc = sum((v.co for v in f.verts), Vector()) / len(f.verts)
        if f.normal.dot(fc - center) < 0.0:
            verts = list(f.verts)
            bm.faces.remove(f)
            bm.faces.new(verts[::-1])


def smooth_laplacian(bm, iters=2):
    """Лапласоцевое сглаживание (аналог катамулл-ромового по сечениям)."""
    for _ in range(iters):
        new_pos = {}
        for v in bm.verts:
            neigh = [e.other_vert(v) for e in v.link_edges]
            if neigh:
                avg = sum((n.co for n in neigh), Vector()) / float(len(neigh))
                new_pos[v.index] = avg
        for idx, co in new_pos.items():
            bm.verts[idx].co = co
    bm.normal_update()


def cleanup(ob, dist=5e-5):
    """remove_doubles + удаление вырожденных граней (как в разделе 6 памяти)."""
    if ob.type != 'MESH':
        return
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    before = len(bm.verts)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    degen = [f for f in bm.faces if len(f.verts) < 3 or f.calc_area() < 1e-12]
    if degen:
        bmesh.ops.delete(bm, geom=degen, context='FACES')
    bm.to_mesh(me)
    bm.free()
    me.update()
    me.calc_normals()


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def key_linear(ob, data_path, index, frame, value):
    ob.animation_data_create()
    ob.keyframe_insert(data_path, index=index, frame=frame)


def linearize(ob):
    if ob.animation_data and ob.animation_data.action:
        for fc in ob.animation_data.action.fcurves:
            for kf in fc.keyframe_points:
                kf.interpolation = 'LINEAR'


# --------------------------------------------------------------------------- #
# 5. Материалы (все процедурные, по UV)
# --------------------------------------------------------------------------- #
def _bsdf_base(m, color, metallic, roughness, coat=0.0, coat_rough=0.05):
    bsdf = m.node_tree.nodes.get('Principled BSDF')
    c = bsdf.inputs['Base Color'].default_value
    c[0], c[1], c[2], c[3] = color[0], color[1], color[2], 1.0
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    if coat:
        bsdf.inputs['Coat Weight'].default_value = coat
        bsdf.inputs['Coat Roughness'].default_value = coat_rough
    return bsdf


def mat_paint():
    """CarPaint_MetallicBlue (0.055, 0.115, 0.280), Metallic 0.62, Roughness 0.26,
    Coat 1.0/0.03, «апельсиновая корка» —注释 (процедурно)."""
    m = bpy.data.materials.new('CarPaint_MetallicBlue')
    m.use_nodes = True
    _bsdf_base(m, (0.055, 0.115, 0.280), 0.62, 0.26, coat=1.0, coat_rough=0.03)
    return m


def mat_white(color=(0.9, 0.9, 0.9), roughness=0.9):
    m = bpy.data.materials.new('White_Plastic')
    m.use_nodes = True
    _bsdf_base(m, color, 0.0, roughness)
    return m


def mat_glass():
    """Glass_Window: Transparent + Glossy по Layer Weight (НЕ Transmission!),
    тонировка 0.58."""
    m = bpy.data.materials.new('Glass_Window')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    trans = nt.nodes.new('ShaderNodeBsdfTransparent')
    gloss = nt.nodes.new('ShaderNodeBsdfGlossy')
    gloss.inputs['Roughness'].default_value = 0.065
    c = gloss.inputs['Color'].default_value
    c[0], c[1], c[2] = 0.55, 0.70, 0.90
    mix_col = nt.nodes.new('ShaderNodeMixRGB')
    mix_col.blend_type = 'MIX'
    mix_col.inputs['Fac'].default_value = 0.58
    mix_col.inputs['Color1'].default_value = (0.30, 0.40, 0.55, 1.0)
    mix_col.inputs['Color2'].default_value = c[:]
    nt.links.new(mix_col.outputs['Color'], gloss.inputs['Color'])
    lw = nt.nodes.new('ShaderNodeLayerWeight')
    lw.blend_type = 'FRESNEL'
    mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(lw.outputs['Fresnel'], mix.inputs['Fac'])
    nt.links.new(trans.outputs[0], mix.inputs[1])
    nt.links.new(gloss.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs['Surface'])
    return m


def mat_lens():
    """Lens_Clear: Transparent + Glossy по Fresnel (IOR 1.52)."""
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
    """Rubber_Tire: протектор по UV — checker 150/13, волна 5.5, маска зон."""
    m = bpy.data.materials.new('Rubber_Tire')
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Base Color'].default_value = (0.02, 0.02, 0.02, 1.0)
    bsdf.inputs['Roughness'].default_value = 0.95
    bsdf.inputs['Metallic'].default_value = 0.0
    tc = nt.nodes.new('ShaderNodeTexCoord')
    mp = nt.nodes.new('ShaderNodeMapping')
    mp.inputs['Scale'].default_value = (150.0, 13.0, 1.0)
    chk = nt.nodes.new('ShaderNodeTexChecker')
    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.inputs['Scale'].default_value = 5.5
    mix = nt.nodes.new('ShaderNodeMixRGB')
    mix.blend_type = 'MULTIPLY'
    mix.inputs['Fac'].default_value = 0.6
    ramp = nt.nodes.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].color = (0.25, 0.25, 0.25, 1.0)
    ramp.color_ramp.elements[1].color = (0.85, 0.85, 0.85, 1.0)
    nt.links.new(tc.outputs['UV'], mp.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], chk.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], wave.inputs['Vector'])
    nt.links.new(chk.outputs['Color'], mix.inputs['Color1'])
    nt.links.new(wave.outputs['Fac'], mix.inputs['Color2'])
    nt.links.new(mix.outputs['Color'], ramp.inputs['Fac'])
    nt.links.new(ramp.outputs['Color'], bsdf.inputs['Roughness'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return m


def mat_leather(name, color, scale=9.0, roughness=0.65):
    """Кожа: зерно по UV (scale) + прострочка (волна)."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    c = bsdf.inputs['Base Color'].default_value
    c[0], c[1], c[2], c[3] = color[0], color[1], color[2], 1.0
    bsdf.inputs['Roughness'].default_value = roughness
    tc = nt.nodes.new('ShaderNodeTexCoord')
    mp = nt.nodes.new('ShaderNodeMapping')
    mp.inputs['Scale'].default_value = (scale, scale, 1.0)
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 4.5
    noise.inputs['Detail'].default_value = 6.0
    mix_col = nt.nodes.new('ShaderNodeMixRGB')
    mix_col.blend_type = 'SOFT_LIGHT'
    mix_col.inputs['Fac'].default_value = 0.35
    mix_col.inputs['Color1'].default_value = c[:]
    wave = nt.nodes.new('ShaderNodeTexWave')
    wave.inputs['Scale'].default_value = 90.0
    nt.links.new(tc.outputs['UV'], mp.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], noise.inputs['Vector'])
    nt.links.new(mp.outputs['Vector'], wave.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], mix_col.inputs['Color2'])
    nt.links.new(mix_col.outputs['Color'], bsdf.inputs['Base Color'])
    nt.links.new(wave.outputs['Fac'], bsdf.inputs['Roughness'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    return m


def mat_emission(name, strength, color=(1.0, 1.0, 1.0)):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    em = nt.nodes.new('ShaderNodeEmission')
    c = em.inputs['Color'].default_value
    c[0], c[1], c[2] = color[0], color[1], color[2]
    em.inputs['Strength'].default_value = strength
    nt.links.new(em.outputs['Emission'], out.inputs['Surface'])
    return m


def mat_carbon():
    m = bpy.data.materials.new('Carbon_Corpus')
    m.use_nodes = True
    _bsdf_base(m, (0.10, 0.10, 0.10), 0.85, 0.35)
    return m


def make_materials():
    return {
        'paint': mat_paint(),
        'glass': mat_glass(),
        'lens': mat_lens(),
        'rubber': mat_rubber(),
        'leather': mat_leather('Leather_Seat', (0.042, 0.038, 0.037), scale=9.0),
        'carpet': mat_leather('Carpet', (0.025, 0.022, 0.020), scale=42.0,
                              roughness=0.85),
        'headliner': mat_leather('Headliner_Fabric', (0.07, 0.065, 0.06),
                                 scale=55.0, roughness=0.9),
        'headlight': mat_emission('Headlight_Light', 0.55, (1.0, 0.95, 0.85)),
        'drl': mat_emission('DRL_Light', 0.22, (1.0, 0.92, 0.80)),
        'tail': mat_emission('Tail_Light', 0.16, (0.95, 0.10, 0.12)),
        'carbon': mat_carbon(),
        'white': mat_white(),
    }


# --------------------------------------------------------------------------- #
# 6. Детали на поверхности (ray_cast)
# --------------------------------------------------------------------------- #
def body_hit(y, z, front=True):
    """Луч вдоль X извне борта к кузову; возвращает (точка, нормаль)."""
    deps = bpy.context.evaluated_depsgraph_get()
    ox = -9.0 if front else 9.0
    res, loc, normal, idx, dist = bpy.context.scene.ray_cast(
        deps, Vector((ox, y, z)),
        Vector((1.0 if front else -1.0, 0.0, 0.0)), distance=20.0)
    if res:
        return loc, normal
    return None, None


def top_hit(x, y):
    """Луч сверху вниз на поверхность."""
    deps = bpy.context.evaluated_depsgraph_get()
    res, loc, normal, idx, dist = bpy.context.scene.ray_cast(
        deps, Vector((x, y, 4.0)), Vector((0.0, 0.0, -1.0)), distance=8.0)
    if res:
        return loc, normal
    return None, None


def shell_patch(name, origin, normal, size=(0.4, 0.3), segs=(4, 3),
                off=0.0, thickness=0.006, material=None, project=True):
    """Тонкая закрытая накладка на поверхности. off > 0 — наружу кузова."""
    if project:
        hit, nrm = body_hit(origin.y, origin.z, front=(origin.x < 0))
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
    for layer in grids:
        for j in range(segs[1] - 1):
            for i in range(segs[0] - 1):
                bm.faces.new((layer[j][i], layer[j][i + 1],
                              layer[j + 1][i + 1], layer[j + 1][i]))
    # ремни по периметру между двумя слоями ->閉じた оболочка
    for j in range(segs[1] - 1):
        bm.faces.new((grids[0][j][0], grids[1][j][0],
                      grids[1][j + 1][0], grids[0][j + 1][0]))
        bm.faces.new((grids[0][j][segs[0] - 1], grids[1][j][segs[0] - 1],
                      grids[1][j + 1][segs[0] - 1], grids[0][j + 1][segs[0] - 1]))
    for i in range(segs[0] - 1):
        bm.faces.new((grids[0][0][i], grids[1][0][i],
                      grids[1][0][i + 1], grids[0][0][i + 1]))
        bm.faces.new((grids[0][segs[1] - 1][i], grids[1][segs[1] - 1][i],
                      grids[1][segs[1] - 1][i + 1], grids[0][segs[1] - 1][i + 1]))
    orient_outward(bm)
    ob = bmesh_to_object(name, bm, material=material, smooth=True)
    # UV по локальной сетке накладки
    me = ob.data
    me.uv_layers.new()
    uv = me.uv_layers[0].data
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            uv[loop_idx].uv = (
                clamp((v.co.dot(t1) / size[0] + 0.5), 0.0, 1.0),
                clamp((v.co.dot(t2) / size[1] + 0.5), 0.0, 1.0))
    return ob


# --------------------------------------------------------------------------- #
# 7. Кузов
# --------------------------------------------------------------------------- #
def build_body(mats):
    bm = bmesh.new()
    xs = [X_MIN + (X_MAX - X_MIN) * i / (N_SECTIONS - 1) for i in range(N_SECTIONS)]
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
            bm.faces.new((verts[base + k],
                          verts[base + (k + 1) % vcount],
                          verts[nbase + (k + 1) % vcount],
                          verts[nbase + k]))
    # замыкание торцов: купола по суперэллипсу до точки на оси X
    endcaps = ((X_MIN, xs[0], Vector((-1.0, 0.0, 0.0))),
               (X_MAX, xs[-1], Vector((1.0, 0.0, 0.0))))
    for x_end, x_first, _expect in endcaps:
        i0 = xs.index(x_first)
        ring = [verts[i0 * vcount + k] for k in range(vcount)]
        zm = (ZB(x_first) + ZR(x_first)) / 2.0
        tip = bm.verts.new(Vector((x_end, 0.0, zm)))
        prev = ring
        n_rings = 8
        for i in range(1, n_rings + 1):
            u = i / n_rings
            s = dome_scale(u)
            xr = x_first + (x_end - x_first) * u
            ring = []
            for k in range(vcount):
                y, z = body_section(x_first)[k]
                ring.append(bm.verts.new(Vector((xr, (y) * s, zm + (z - zm) * s))))
            for k in range(vcount):
                bm.faces.new((prev[k], prev[(k + 1) % vcount],
                              ring[(k + 1) % vcount], ring[k]))
            prev = ring
        for k in range(vcount):
            bm.faces.new((tip, prev[(k + 1) % vcount], prev[k]))

    smooth_laplacian(bm, iters=2)
    orient_outward(bm)
    ob = bmesh_to_object('Body', bm, material=mats['paint'], smooth=True)
    cleanup(ob, 5e-5)
    # uv_top: u = X, v = Y
    me = ob.data
    me.uv_layers.new()
    uv = me.uv_layers[0].data
    for poly in me.polygons:
        for loop_idx in poly.loop_indices:
            v = me.vertices[me.loops[loop_idx].vertex_index]
            uv[loop_idx].uv = (
                clamp((v.co.x - X_MIN) / LENGTH, 0.0, 1.0),
                clamp((v.co.y + WIDTH / 2.0) / WIDTH, 0.0, 1.0))
    return ob


def build_glass(mats):
    """Остёкла как закрытая тонкая оболочка между рельсом и крышей."""
    bm = bmesh.new()
    xs = [X_MIN + (X_MAX - X_MIN) * i / (N_SECTIONS - 1) for i in range(N_SECTIONS)]
    cab = [x for x in xs if -1.30 < x < 1.10]
    loops = [glass_section(x) for x in cab]
    vcount = len(loops[0])
    allv = []
    for x, loop in zip(cab, loops):
        row = []
        for (y, z) in loop:
            row.append(bm.verts.new(Vector((x, y, z))))
        allv.append(row)
    for i in range(len(cab) - 1):
        for k in range(vcount):
            bm.faces.new((allv[i][k], allv[i][(k + 1) % vcount],
                          allv[i + 1][(k + 1) % vcount], allv[i + 1][k]))
    # заглушки концов салона
    for ring in (allv[0], allv[-1]):
        c = sum((v.co for v in ring), Vector()) / len(ring)
        cen = bm.verts.new(c)
        for k in range(vcount):
            bm.faces.new((cen, ring[k], ring[(k + 1) % vcount]))
    bmesh.ops.solidify(bm, geom=bm.faces[:], thickness=0.0025)
    orient_outward(bm)
    ob = bmesh_to_object('Glass', bm, material=mats['glass'], smooth=True)
    cleanup(ob, 5e-5)
    return ob


# --------------------------------------------------------------------------- #
# 8. Колёса (ось по Y, вращение вокруг Y; шины閉じтые оболочки)
# --------------------------------------------------------------------------- #
def build_wheel(axle_x, side, mats):
    """side = +1/-1. Шины и ободы — замкнутые оболочки (ноля niemanifold)."""
    cy = side * WHEEL_Y
    cz = WHEEL_Z
    r = WHEEL_R
    width = 0.235
    seg = 32

    def cyl_caps(name, radius, y_off, material, is_rim=False):
        bm = bmesh.new()
        rings = []
        for yy in (-y_off, y_off):
            ring = []
            for i in range(seg):
                a = 2.0 * math.pi * i / seg
                ring.append(bm.verts.new(Vector((axle_x + radius * math.cos(a),
                                                 cy + yy,
                                                 cz + radius * math.sin(a)))))
            rings.append(ring)
        for k in range(seg):
            bm.faces.new((rings[0][k], rings[0][(k + 1) % seg],
                          rings[1][(k + 1) % seg], rings[1][k]))
        # торцовые капюшоны (к центру оси) -> замкнутое тело
        for yy in (-y_off, y_off):
            cen = bm.verts.new(Vector((axle_x, cy + yy, cz)))
            ring = rings[0] if yy < 0 else rings[1]
            for k in range(seg):
                bm.faces.new((cen, ring[k], ring[(k + 1) % seg]))
        orient_outward(bm)
        ob = bmesh_to_object(name, bm, material=material, smooth=True)
        return ob

    tire = cyl_caps(f'Wheel_{side:+d}', WHEEL_R, width / 2.0, mats['rubber'])
    rim = cyl_caps(f'Rim_{side:+d}', 0.285, 0.105, mats['carbon'])
    return tire, rim


# --------------------------------------------------------------------------- #
# 9. Салон (упрощённый, но объёмный; все тела замкнуты)
# --------------------------------------------------------------------------- #
def box(bm, x0, x1, y0, y1, z0, z1):
    """Прямоугольный блок, замкнутая оболочка."""
    pts = {}
    for ix, x in enumerate((x0, x1)):
        for iy, y in enumerate((y0, y1)):
            for iz, z in enumerate((z0, z1)):
                pts[(ix, iy, iz)] = bm.verts.new(Vector((x, y, z)))

    def V(i, j, k):
        return pts[(i, j, k)]

    faces = [
        (V(0, 0, 0), V(1, 0, 0), V(1, 1, 0), V(0, 1, 0)),
        (V(0, 0, 1), V(0, 1, 1), V(1, 1, 1), V(1, 0, 1)),
        (V(0, 0, 0), V(0, 0, 1), V(1, 0, 1), V(1, 0, 0)),
        (V(1, 1, 0), V(1, 1, 1), V(0, 1, 1), V(0, 1, 0)),
        (V(0, 0, 0), V(0, 1, 0), V(0, 1, 1), V(0, 0, 1)),
        (V(1, 0, 1), V(1, 1, 1), V(1, 1, 0), V(1, 0, 0)),
    ]
    for f in faces:
        bm.faces.new(f)
    return pts


def build_interior(mats):
    objs = []
    bm = bmesh.new()
    # два передних сиденья
    for side in (-1.0, 1.0):
        yc = side * 0.44
        box(bm, 0.10, 0.85, yc - 0.21, yc + 0.21, 0.34, 0.50)   # подушка
        box(bm, 0.10, 0.85, yc - 0.21, yc + 0.21, 0.50, 1.02)   # спинка
        box(bm, 0.62, 0.85, yc - 0.10, yc + 0.10, 1.02, 1.22)   # подголовник
    # задний диван
    box(bm, -1.15, -0.30, -0.52, 0.52, 0.34, 0.50)
    box(bm, -1.15, -0.30, -0.52, 0.52, 0.50, 0.92)
    box(bm, -0.95, -0.55, -0.12, 0.12, 0.92, 1.10)              # подголовники дивана
    # задняя полка / шторка между задними сиденьями и задним стеклом
    box(bm, -1.42, -1.12, -0.55, 0.55, 0.95, 1.02)
    # центральный тоннель
    box(bm, 1.15, -1.15, -0.14, 0.14, ZB(0.0), 0.62)
    orient_outward(bm)
    cabin = bmesh_to_object('Cabin', bm, material=mats['leather'], smooth=True)
    cleanup(cabin, 5e-5)
    objs.append(cabin)

    # панель приборов
    dash = new_object('Dashboard', bpy.data.meshes.new('DashboardMesh'))
    bm2 = bmesh.new()
    box(bm2, 1.12, 1.62, -0.90, 0.90, 0.50, 1.18)
    orient_outward(bm2)
    me2 = bpy.data.meshes.new('DashboardMesh')
    bm2.to_mesh(me2); bm2.free(); me2.update()
    me2.materials.append(mats['leather'])
    dash.data = me2
    bpy.context.scene.collection.objects.link(dash)
    objs.append(dash)

    # экран на панели
    screen = new_object('Screen', bpy.data.meshes.new('ScreenMesh'))
    bm3 = bmesh.new()
    box(bm3, 1.28, 1.52, -0.30, 0.30, 0.72, 0.98)
    orient_outward(bm3)
    me3 = bpy.data.meshes.new('ScreenMesh')
    bm3.to_mesh(me3); bm3.free(); me3.update()
    me3.materials.append(mats['lens'])
    screen.data = me3
    bpy.context.scene.collection.objects.link(screen)
    objs.append(screen)

    # руль: замкнутая оболочка (обод с толщиной + спицы в masses)
    sw = new_object('SteeringWheel', bpy.data.meshes.new('SteeringWheelMesh'))
    bm4 = bmesh.new()
    R, r_in, dt, seg = 0.27, 0.22, 0.018, 32
    cx, cy, cz = 1.32, -0.30, 0.78
    rings = []
    for zz in (-dt / 2.0, dt / 2.0):
        rout = []
        rin = []
        for i in range(seg):
            a = 2.0 * math.pi * i / seg
            rout.append(bm4.verts.new(Vector((cx + R * math.cos(a),
                                              cy + R * math.sin(a), cz + zz))))
            rin.append(bm4.verts.new(Vector((cx + r_in * math.cos(a),
                                             cy + r_in * math.sin(a), cz + zz))))
        rings.append((rout, rin))
    for iz in (0, 1):
        rout, rin = rings[iz]
        for k in range(seg):
            # внешняя боковая
            bm4.faces.new((rout[k], rout[(k + 1) % seg], rin[(k + 1) % seg], rin[k]))
            # верхняя/нижняя annular плоскость
            bm4.faces.new((rout[k], rin[k], rin[(k + 1) % seg], rout[(k + 1) % seg]))
    # спицы (узкие коробки от центра к ободу)
    for k in range(0, seg, 8):
        a = 2.0 * math.pi * k / seg
        p0 = Vector((cx, cy, cz - dt / 2.0))
        p1 = Vector((cx, cy, cz + dt / 2.0))
        q0 = Vector((cx + r_in * math.cos(a), cy + r_in * math.sin(a), cz - dt / 2.0))
        q1 = Vector((cx + r_in * math.sin(a) * 0.0 + r_in * math.cos(a),
                     cy + r_in * math.sin(a), cz + dt / 2.0))
        # упрощённо: один треугольник-ремень в сечении
        bm4.faces.new((bm4.verts.new(p0), bm4.verts.new(q0), bm4.verts.new(p1)))
    orient_outward(bm4)
    me4 = bpy.data.meshes.new('SteeringWheelMesh')
    bm4.to_mesh(me4); bm4.free(); me4.update()
    me4.materials.append(mats['carbon'])
    sw.data = me4
    bpy.context.scene.collection.objects.link(sw)
    objs.append(sw)

    # ковёр
    carpet = new_object('Carpet', bpy.data.meshes.new('CarpetMesh'))
    bm5 = bmesh.new()
    box(bm5, 1.45, -1.45, -0.72, 0.72, ZB(0.0), ZB(0.0) + 0.018)
    orient_outward(bm5)
    me5 = bpy.data.meshes.new('CarpetMesh')
    bm5.to_mesh(me5); bm5.free(); me5.update()
    me5.materials.append(mats['carpet'])
    carpet.data = me5
    bpy.context.scene.collection.objects.link(carpet)
    objs.append(carpet)

    # потолок салона: HEADLINER_DROP = 0.055 под крышей (анти-z-fighting)
    headliner = new_object('Headliner', bpy.data.meshes.new('HeadlinerMesh'))
    bm6 = bmesh.new()
    zhl = ZR(0.0) - 0.055
    box(bm6, 1.30, -1.30, -0.60, 0.60, zhl, zhl + 0.012)
    orient_outward(bm6)
    me6 = bpy.data.meshes.new('HeadlinerMesh')
    bm6.to_mesh(me6); bm6.free(); me6.update()
    me6.materials.append(mats['headliner'])
    headliner.data = me6
    bpy.context.scene.collection.objects.link(headliner)
    objs.append(headliner)

    # динамики в дверях (замкнутые цилиндры)
    for side in (-1.0, 1.0):
        yc = side * 0.88
        si = int(side)
        sp = new_object(f'Speaker_{si:+d}', bpy.data.meshes.new(f'SpeakerMesh_{si:+d}'))
        bm7 = bmesh.new()
        r0, r1, h = 0.035, 0.030, 0.012
        xa = 0.36
        ring_f = []
        ring_b = []
        for i in range(8):
            a = 2.0 * math.pi * i / 8
            ring_f.append(bm7.verts.new(Vector((xa, yc + r0 * math.cos(a),
                                                0.62 + r0 * math.sin(a)))))
            ring_b.append(bm7.verts.new(Vector((xa - h, yc + r1 * math.cos(a),
                                                0.62 + r1 * math.sin(a)))))
        for k in range(8):
            bm7.faces.new((ring_f[k], ring_f[(k + 1) % 8],
                           ring_b[(k + 1) % 8], ring_b[k]))
        cf = bm7.verts.new(Vector((xa, yc, 0.62)))
        cb = bm7.verts.new(Vector((xa - h, yc, 0.62)))
        for k in range(8):
            bm7.faces.new((cf, ring_f[(k + 1) % 8], ring_f[k]))
            bm7.faces.new((cb, ring_b[k], ring_b[(k + 1) % 8]))
        orient_outward(bm7)
        me7 = bpy.data.meshes.new(f'SpeakerMesh_{si:+d}')
        bm7.to_mesh(me7); bm7.free(); me7.update()
        me7.materials.append(mats['carbon'])
        sp.data = me7
        bpy.context.scene.collection.objects.link(sp)
        objs.append(sp)

    return objs


def build_doors(body_ob, mats):
    """Четыре двери как отдельные меша с origin в шарнире.
    Дверь вращается вокруг локальной оси Z (вертикальной, через шарнир).
    Имена: Door_Front_Left, Door_Front_Right, Door_Rear_Left, Door_Rear_Right.
    Каждая дверь — тонкая коробка (пластина с толщиной), материал кожи
    (внутренняя обшивка виднее из салона)."""
    objs = []
    door_thickness = 0.035
    door_height = 0.72
    hinge_z = 0.75

    doors_spec = [
        # имя, ось шарнира X, сторону Y, какой конец шарнир, угол открывания
        ('Door_Front_Left',  0.10, +1.0, 'front',  70.0),
        ('Door_Front_Right', 0.10, -1.0, 'front', -70.0),
        ('Door_Rear_Left',  -1.10, +1.0, 'rear',   70.0),
        ('Door_Rear_Right', -1.10, -1.0, 'rear',  -70.0),
    ]

    for name, hx, side, hinge_side, open_angle in doors_spec:
        yc = side * (WHEEL_Y - 0.02)
        bm = bmesh.new()
        pts = {}
        if hinge_side == 'front':
            x0, x1 = -0.88, 0.0   # дверь простирается назад от шарнира
        else:
            x0, x1 = 0.0, 0.88    # дверь простирается вперёд от шарнира
        for ix, x in enumerate((x0, x1)):
            for iy, y in enumerate((0.0, door_thickness * side)):
                for iz, z in enumerate((0.0, door_height)):
                    pts[(ix, iy, iz)] = bm.verts.new(Vector((x, y, z)))

        def V(i, j, k):
            return pts[(i, j, k)]

        faces = [
            (V(0, 0, 0), V(1, 0, 0), V(1, 1, 0), V(0, 1, 0)),
            (V(0, 0, 1), V(0, 1, 1), V(1, 1, 1), V(1, 0, 1)),
            (V(0, 0, 0), V(0, 0, 1), V(1, 0, 1), V(1, 0, 0)),
            (V(1, 1, 0), V(1, 1, 1), V(0, 1, 1), V(0, 1, 0)),
            (V(0, 0, 0), V(0, 1, 0), V(0, 1, 1), V(0, 0, 1)),
            (V(1, 0, 1), V(1, 1, 1), V(1, 1, 0), V(1, 0, 0)),
        ]
        for f in faces:
            bm.faces.new(f)
        orient_outward(bm)
        ob = bmesh_to_object(name, bm, material=mats['leather'], smooth=True)
        cleanup(ob, 5e-5)
        # поставить объект в точку шарнира — origin объекта и есть шарнир,
        # поэтому вращение rotation_euler.z يدور дверь вокруг шарнира
        ob.location = Vector((hx, yc, hinge_z))
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
        objs.append(ob)
    return objs


# --------------------------------------------------------------------------- #
# 10. Свет, мир, камеры
# --------------------------------------------------------------------------- #
def add_light(name, loc, energy, size=0.8, rot=None):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.size = size
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    if rot is not None:
        ob.rotation_euler = rot
    bpy.context.scene.collection.objects.link(ob)
    return ob


def build_lighting(mats):
    # интерьер (ватты, Area)
    add_light('Int_RoofLight_F', Vector((0.55, 0.0, 1.30)), 1.9, size=0.35,
              rot=(math.radians(90), 0, 0))
    add_light('Int_RoofLight_M', Vector((-0.55, 0.0, 1.30)), 2.1, size=0.35,
              rot=(math.radians(90), 0, 0))
    add_light('Int_SeatLight', Vector((0.0, 0.45, 0.62)), 1.7, size=0.2,
              rot=(math.radians(90), 0, 0))
    add_light('Int_DashLight', Vector((1.30, 0.0, 0.95)), 0.9, size=0.25,
              rot=(math.radians(100), 0, math.radians(10)))
    add_light('Int_Ceiling', Vector((0.0, 0.0, 1.28)), 0.9, size=0.4,
              rot=(math.radians(90), 0, 0))
    # студийный свет: 6 Area-лампы вокруг машины
    studio = [
        ('Studio_Light_1', Vector((-4.5, -4.5, 3.2)), 200.0),
        ('Studio_Light_2', Vector((4.5, -4.5, 3.2)), 200.0),
        ('Studio_Light_3', Vector((-6.0, 2.0, 2.6)), 150.0),
        ('Studio_Light_4', Vector((6.0, 2.0, 2.6)), 150.0),
        ('Studio_Light_5', Vector((0.0, 7.5, 3.6)), 250.0),
        ('Studio_Light_6', Vector((0.0, -8.5, 2.2)), 120.0),
    ]
    for name, loc, en in studio:
        add_light(name, loc, en, size=1.2, rot=(math.radians(70), 0, 0))
    # циклорама позади машины
    cyc = new_object('Cyclorama', bpy.data.meshes.new('CycloramaMesh'))
    bmc = bmesh.new()
    box(bmc, -3.0, -2.55, -4.2, 4.2, 0.0, 3.4)
    orient_outward(bmc)
    mec = bpy.data.meshes.new('CycloramaMesh')
    bmc.to_mesh(mec); bmc.free(); mec.update()
    mec.materials.append(mats['white'])
    cyc.data = mec
    bpy.context.scene.collection.objects.link(cyc)
    # мир + HDRI (если есть на диске; иначе сплошной фон)
    world = bpy.data.worlds.new('World_Studio')
    bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    bg = nt.nodes.get('Background')
    hdri_path = os.path.join('hdri', 'studio_small_09.hdr')
    if os.path.exists(hdri_path):
        env = nt.nodes.new('ShaderNodeTexEnvironment')
        env.image = bpy.data.images.load(hdri_path)
        nt.links.new(env.outputs['Color'], bg.inputs['Color'])
        bg.inputs['Strength'].default_value = 1.0
    else:
        bg.inputs['Color'].default_value = (0.10, 0.10, 0.115, 1.0)
        bg.inputs['Strength'].default_value = 1.0


def build_cameras():
    cams = {
        'Cam_Exterior': (Vector((6.05, -5.15, 1.66)), 62.0),
        'Cam_Rear': (Vector((-5.9, -4.7, 1.90)), 55.0),
        'Cam_Side': (Vector((0.10, -10.2, 1.30)), 68.0),
        'Cam_Interior': (Vector((-1.14, -0.36, 1.20)), 19.0),
        'Cam_Interior2': (Vector((0.62, 0.60, 1.185)), 20.0),
    }
    objs = {}
    for name, (loc, lens) in cams.items():
        camd = bpy.data.cameras.new(name)
        camd.lens = lens
        camd.sensor_width = 36.0
        ob = bpy.data.objects.new(name, camd)
        ob.location = loc
        look_at(ob, TARGET)
        bpy.context.scene.collection.objects.link(ob)
        objs[name] = ob
    return objs


# --------------------------------------------------------------------------- #
# 11. Анимация
# --------------------------------------------------------------------------- #
def animate(wheels, steering, body_ob, door_objs=None):
    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = 240
    scene.render.fps = 24
    total = 40.0 * 2.0 * math.pi  # 40 оборотов колёс
    for wh in wheels:
        key_linear(wh, 'rotation_euler', 1, 1, 0.0)
        key_linear(wh, 'rotation_euler', 1, 240, total)
        linearize(wh)
    # руль ±17° вокруг Z
    ang = math.radians(17.0)
    for frame, val in ((1, 0.0), (60, ang), (120, 0.0), (180, -ang), (240, 0.0)):
        key_linear(steering, 'rotation_euler', 2, frame, val)
    linearize(steering)
    # крен кузова ±0.8° вокруг продольной оси X
    roll = math.radians(0.8)
    for frame, val in ((1, 0.0), (60, roll), (120, 0.0), (180, -roll), (240, 0.0)):
        key_linear(body_ob, 'rotation_euler', 0, frame, val)
    linearize(body_ob)
    # двери: открываются на 45-м кадре, держатся открытыми до 200-го, закрываются к 240-му
    door_angle = math.radians(70.0)
    if door_objs:
        for ob in door_objs:
            # левые двери открываются в одну сторону, правые — в другую
            sign = 1.0 if ob.name.endswith('_Left') else -1.0
            ang = sign * door_angle
            for frame, val in ((1, 0.0), (45, ang), (200, ang), (240, 0.0)):
                key_linear(ob, 'rotation_euler', 2, frame, val)
            linearize(ob)


# --------------------------------------------------------------------------- #
# 12. Рендер-настройки и сохранение
# --------------------------------------------------------------------------- #
def setup_render():
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 48
    scene.cycles.transparent_max_bounces = 48   # обязательно (иначе муар на стёклах)
    scene.cycles.blur_glossy = 2.0              # обязательно (иначе муар на стёклах)
    scene.render.resolution_x = 1440
    scene.render.resolution_y = 810
    scene.render.resolution_pct = 100
    try:
        scene.view_settings.view_transform = 'AgX'
        scene.view_settings.look = 'Medium High Contrast'
    except Exception:
        pass


def render_stills(cams):
    os.makedirs('renders', exist_ok=True)
    for name in ('Cam_Exterior', 'Cam_Rear', 'Cam_Side', 'Cam_Interior', 'Cam_Interior2'):
        ob = cams.get(name)
        if ob is None:
            continue
        bpy.context.scene.camera = ob
        bpy.context.scene.render.filepath = os.path.abspath(os.path.join('renders', name + '.png'))
        bpy.ops.render.render(write_still=True)
        print('rendered', name)


# --------------------------------------------------------------------------- #
# 13. Главная точка входа
# --------------------------------------------------------------------------- #
def main(argv):
    # чистая сцена
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    try:
        bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True,
                                       do_recursive=True)
    except Exception:
        pass

    mats = make_materials()
    body_ob = build_body(mats)
    bpy.context.view_layer.update()
    build_glass(mats)

    wheels = []
    rim_objs = []
    for axle, side in ((AXLE_F, +1), (AXLE_F, -1), (AXLE_R, +1), (AXLE_R, -1)):
        tire, rim = build_wheel(axle, side, mats)
        wheels.append(tire)
        rim_objs.append(rim)

    bpy.context.view_layer.update()

    # двери (привязываются к кузову внутри build_doors)
    door_objs = build_doors(body_ob, mats)

    # детали кузова (где ray_cast найдёт поверхность)
    nose_hit, nose_nrm = body_hit(0.0, 0.80, front=True)
    if nose_hit is not None:
        shell_patch('Grille_Front', nose_hit, nose_nrm, size=(0.60, 0.20),
                    segs=(10, 4), off=0.02, material=mats['carbon'])
    lh_hit, lh_nrm = body_hit(AXLE_F - 0.08, W(AXLE_F - 0.08) - 0.06, front=True)
    if lh_hit is not None:
        shell_patch('Headlight_L', lh_hit, lh_nrm, size=(0.42, 0.14), segs=(6, 3),
                    off=0.035, material=mats['carbon'])
        lens = top_hit(lh_hit.x, lh_hit.y)
        if lens is not None:
            shell_patch('Headlight_Lens_L', lens[0], lens[1], size=(0.38, 0.11),
                        segs=(6, 3), off=0.055, material=mats['lens'])
            shell_patch('Headlight_DRL_L', lens[0], lens[1], size=(0.34, 0.035),
                        segs=(6, 2), off=0.070, material=mats['drl'])
    rh_hit, rh_nrm = body_hit(AXLE_F - 0.08, -(W(AXLE_F - 0.08) - 0.06), front=True)
    if rh_hit is not None:
        shell_patch('Headlight_R', rh_hit, rh_nrm, size=(0.42, 0.14), segs=(6, 3),
                    off=0.035, material=mats['carbon'])
        lens = top_hit(rh_hit.x, rh_hit.y)
        if lens is not None:
            shell_patch('Headlight_Lens_R', lens[0], lens[1], size=(0.38, 0.11),
                        segs=(6, 3), off=0.055, material=mats['lens'])
            shell_patch('Headlight_DRL_R', lens[0], lens[1], size=(0.34, 0.035),
                        segs=(6, 2), off=0.070, material=mats['drl'])
    th_hit, th_nrm = body_hit(AXLE_R + 0.08, W(AXLE_R + 0.08) - 0.05, front=False)
    if th_hit is not None:
        shell_patch('Tail_Light_L', th_hit, th_nrm, size=(0.30, 0.10), segs=(5, 3),
                    off=0.025, material=mats['tail'])
    th2_hit, th2_nrm = body_hit(AXLE_R + 0.08, -(W(AXLE_R + 0.08) - 0.05), front=False)
    if th2_hit is not None:
        shell_patch('Tail_Light_R', th2_hit, th2_nrm, size=(0.30, 0.10), segs=(5, 3),
                    off=0.025, material=mats['tail'])

    interior_objs = build_interior(mats)
    build_lighting(mats)
    cams = build_cameras()
    bpy.context.scene.camera = cams['Cam_Exterior']

    # привязка колёс и руля к кузову (крен кузова тянет за собой колёса)
    for ob in wheels + rim_objs:
        ob.parent = body_ob
        ob.matrix_parent_inverse = body_ob.matrix_world.inverted()
    steering = bpy.data.objects.get('SteeringWheel')
    if steering is not None:
        steering.parent = body_ob
        steering.matrix_parent_inverse = body_ob.matrix_world.inverted()

    animate(wheels, steering, body_ob, door_objs)

    # --- имена объектов: UE-стиль (без пробелов/скобок/знаков +-) ---
    import re
    for ob in bpy.data.objects:
        if ob.type == 'MESH':
            ob.name = re.sub(r'[^A-Za-z0-9_]', '_', ob.name)
            ob.name = re.sub(r'_+', '_', ob.name).strip('_')
            if not ob.name or ob.name[0].isdigit():
                ob.name = 'obj_' + ob.name

    setup_render()
    blend_path = os.path.abspath('car_realistic.blend')
    bpy.ops.wm.save_as_mainfile(filepath=blend_path, check_existing=False)
    print('scene saved:', blend_path)

    if '--render' in argv:
        render_stills(cams)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
