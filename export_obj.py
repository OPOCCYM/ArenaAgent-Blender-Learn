#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export the Blender-built vehicle mesh to OBJ/MTL for software preview and
cross-tool testing. Not a replacement for FBX/GLB game export.

Run after building the scene:
    blender -b -noaudio -P car_build.py
    blender -b -noaudio car_realistic.blend -P export_obj.py

Result:
    exports/car_ue.obj
    exports/car_ue.mtl

The OBJ uses the evaluated (modifier-applied) mesh and world-space vertex
coordinates, matching the coordinate convention X=length, Y=width, Z=up.
Materials are written with Kd/Ks/Ns from the Principled BSDF inputs so that
software renderers (render_obj_png.py) and other tools can reuse them.
"""
import os
import re
import bpy


def safe(s):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', s)


def main():
    out_dir = os.path.abspath('exports')
    os.makedirs(out_dir, exist_ok=True)

    obj_path = os.path.join(out_dir, 'car_ue.obj')
    mtl_path = os.path.join(out_dir, 'car_ue.mtl')

    depsgraph = bpy.context.evaluated_depsgraph_get()
    vertices = []
    face_records = []
    materials = {}

    for obj in bpy.data.objects:
        if obj.type != 'MESH':
            continue
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        base = len(vertices) + 1
        vertices.extend(
            tuple(obj.matrix_world @ v.co) for v in mesh.vertices
        )
        slots = list(mesh.materials)
        for slot in slots:
            if slot:
                matname = safe(slot.name)
                bsdf = (
                    slot.node_tree.nodes.get('Principled BSDF')
                    if slot.use_nodes else None
                )
                col = tuple(slot.diffuse_color[:3])
                rough = float(bsdf.inputs['Roughness'].default_value) if bsdf and 'Roughness' in bsdf.inputs else 0.4
                metal = float(bsdf.inputs['Metallic'].default_value) if bsdf and 'Metallic' in bsdf.inputs else 0.0
                materials[matname] = (col, rough, metal)
        for poly in mesh.polygons:
            mi = poly.material_index
            mat = slots[mi] if mi < len(slots) else (slots[0] if slots else None)
            matname = safe(mat.name) if mat else 'DefaultGrey'
            if matname not in materials:
                materials[matname] = ((0.5, 0.5, 0.5), 0.45, 0.0)
            face_records.append((safe(obj.name), matname,
                                 [base + i for i in poly.vertices]))
        evaluated.to_mesh_clear()

    with open(mtl_path, 'w', encoding='utf-8') as f:
        f.write('# Exported from Blender; Kd/Ks/Ns from Principled BSDF\n')
        for name, (rgb, rough, metal) in sorted(materials.items()):
            ns = max(2, int((1.0 - rough) * 1000))
            f.write(f'newmtl {name}\n')
            f.write(f'Kd {rgb[0]:.5f} {rgb[1]:.5f} {rgb[2]:.5f}\n')
            f.write('Ka 0.02 0.02 0.02\n')
            f.write(f'Ks {metal:.3f} {metal:.3f} {metal:.3f}\n')
            f.write(f'Ns {ns}\n\n')

    with open(obj_path, 'w', encoding='utf-8') as f:
        f.write('# Exported from Blender car_build.py; X=length, Y=width, Z=up\n')
        f.write('mtllib car_ue.mtl\n')
        for v in vertices:
            f.write(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n')
        active = None
        for name, mat, face in face_records:
            if name != active:
                f.write(f'o {name}\n')
                active = name
            f.write(f'usemtl {mat}\n')
            f.write('f ' + ' '.join(map(str, face)) + '\n')

    print(f'EXPORTED {len(vertices)} vertices, {len(face_records)} faces, '
          f'{len(set(x[0] for x in face_records))} objects')
    print(f'OBJ: {obj_path}')
    print(f'MTL: {mtl_path}')


if __name__ == '__main__':
    main()
