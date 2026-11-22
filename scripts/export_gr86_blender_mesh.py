"""Export the Blender-built vehicle mesh to OBJ for software-preview/testing.
Not a replacement for FBX/GLB game export. Run via Blender WASM with the .blend as input.
"""
import bpy
import os
import re

SRC='/in/assets/gr86_studio.blend'
OBJ='/out/gr86_blender_mesh.obj'
MTL='/out/gr86_blender_mesh.mtl'
bpy.ops.wm.open_mainfile(filepath=SRC)
collection=bpy.data.collections.get('GR86_Model')
if not collection: raise RuntimeError('GR86_Model collection missing')

def safe(s): return re.sub(r'[^A-Za-z0-9_.-]+','_',s)
vertices=[]; face_records=[]; materials={}
depsgraph=bpy.context.evaluated_depsgraph_get()
for obj in collection.objects:
    if obj.type!='MESH': continue
    evaluated=obj.evaluated_get(depsgraph)
    mesh=evaluated.to_mesh()
    base=len(vertices)+1
    vertices.extend(tuple(evaluated.matrix_world @ v.co) for v in mesh.vertices)
    slots=list(mesh.materials)
    for slot in slots:
        if slot:
            bsdf=slot.node_tree.nodes.get('Principled BSDF') if slot.use_nodes else None
            col=tuple(slot.diffuse_color[:3])
            rough=float(bsdf.inputs['Roughness'].default_value) if bsdf and 'Roughness' in bsdf.inputs else .4
            metal=float(bsdf.inputs['Metallic'].default_value) if bsdf and 'Metallic' in bsdf.inputs else 0.0
            materials[safe(slot.name)]=(col,rough,metal)
    for poly in mesh.polygons:
        mi=poly.material_index
        mat=slots[mi] if mi<len(slots) else (slots[0] if slots else None)
        matname=safe(mat.name) if mat else 'DefaultGrey'
        if matname not in materials: materials[matname]=((.5,.5,.5),.45,0.0)
        face_records.append((safe(obj.name),matname,[base+i for i in poly.vertices]))
    evaluated.to_mesh_clear()
os.makedirs('/out',exist_ok=True)
with open(MTL,'w') as f:
    for name,(rgb,rough,metal) in materials.items():
        f.write(f'newmtl {name}\nKd {rgb[0]:.5f} {rgb[1]:.5f} {rgb[2]:.5f}\nKa 0.02 0.02 0.02\nKs {metal:.3f} {metal:.3f} {metal:.3f}\nNs {max(2,int((1-rough)*1000))}\n\n')
with open(OBJ,'w') as f:
    f.write('# Meshes exported from the validated GR86 Blender scene; X length, Z up\nmtllib gr86_blender_mesh.mtl\n')
    for v in vertices: f.write('v %.6f %.6f %.6f\n'%v)
    active=None
    for name,mat,face in face_records:
        if name!=active: f.write('o '+name+'\n'); active=name
        f.write('usemtl '+mat+'\nf '+' '.join(map(str,face))+'\n')
print('EXPORTED',len(vertices),'vertices',len(face_records),'faces',len(face_records),'objects',len(set(x[0] for x in face_records)))
