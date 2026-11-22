"""Import the generated OBJ into Blender, prepare a studio scene, render views and save .blend.

Run in Blender 4.x:
  blender --background --python scripts/build_gr86_blend.py -- \
    --obj assets/gr86_blockout.obj --output assets/gr86_studio.blend

This script is prepared but has not been executed in this environment: bpy cannot load because
required system X/GL libraries are missing. The mesh remains a procedural draft, not a final replica.
"""
import argparse
import math
import os
import sys
import bpy
from mathutils import Vector

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def arguments():
    values=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
    p=argparse.ArgumentParser()
    p.add_argument('--obj',default=os.path.join(ROOT,'assets','gr86_blockout.obj'))
    p.add_argument('--output',default=os.path.join(ROOT,'assets','gr86_studio.blend'))
    p.add_argument('--render-dir',default=os.path.join(ROOT,'assets','blender_renders'))
    return p.parse_args(values)

def clear_scene():
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    for col in list(bpy.data.collections):
        if col.name!='Collection' and col.users==0: bpy.data.collections.remove(col)

def import_obj(path):
    before=set(bpy.context.scene.objects)
    if hasattr(bpy.ops.wm,'obj_import'):
        # For this authored X-length / Z-up OBJ, Y-forward produces identity conversion.
        bpy.ops.wm.obj_import(filepath=path, forward_axis='Y', up_axis='Z')
    else:
        bpy.ops.import_scene.obj(filepath=path, axis_forward='Y', axis_up='Z')
    imported=[o for o in bpy.context.scene.objects if o not in before]
    if not imported: raise RuntimeError('OBJ import produced no objects')
    return imported

def material(name,color,metallic=0.0,roughness=.35):
    mat=bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.diffuse_color=(*color,1); mat.use_nodes=True
    bsdf=mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value=(*color,1)
    bsdf.inputs['Metallic'].default_value=metallic
    bsdf.inputs['Roughness'].default_value=roughness
    return mat

def tune_vehicle_materials():
    # Car-paint clearcoat and tinted glass, following the supplied Blender reference workflow.
    paint=bpy.data.materials.get('Pearl_White')
    if paint:
        paint.use_nodes=True; paint.diffuse_color=(.68,.70,.73,1)
        nodes=paint.node_tree.nodes; links=paint.node_tree.links
        bsdf=nodes.get('Principled BSDF')
        if bsdf:
            bsdf.inputs['Base Color'].default_value=(.68,.70,.73,1)
            bsdf.inputs['Metallic'].default_value=.22
            bsdf.inputs['Roughness'].default_value=.23
            if 'Coat Weight' in bsdf.inputs:
                bsdf.inputs['Coat Weight'].default_value=.82
                bsdf.inputs['Coat Roughness'].default_value=.035
            noise=nodes.new('ShaderNodeTexNoise'); noise.inputs['Scale'].default_value=1800
            bump=nodes.new('ShaderNodeBump'); bump.inputs['Strength'].default_value=.012
            bump.inputs['Distance'].default_value=.001
            links.new(noise.outputs['Fac'],bump.inputs['Height'])
            links.new(bump.outputs['Normal'],bsdf.inputs['Normal'])
    glass=bpy.data.materials.get('Glass_Dark')
    if glass:
        glass.use_nodes=True; bsdf=glass.node_tree.nodes.get('Principled BSDF')
        if bsdf:
            bsdf.inputs['Base Color'].default_value=(.025,.045,.060,1)
            bsdf.inputs['Metallic'].default_value=.28
            bsdf.inputs['Roughness'].default_value=.12
            if 'Coat Weight' in bsdf.inputs:
                bsdf.inputs['Coat Weight'].default_value=.65
                bsdf.inputs['Coat Roughness'].default_value=.025

def area_light(name,location,power,size,color,target):
    data=bpy.data.lights.new(name,'AREA'); data.energy=power; data.shape='DISK'; data.size=size; data.color=color
    obj=bpy.data.objects.new(name,data); bpy.context.collection.objects.link(obj); obj.location=location
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
    return obj

def aim_camera(camera,target):
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()

def main():
    args=arguments(); obj_path=os.path.abspath(args.obj); out_path=os.path.abspath(args.output)
    if not os.path.isfile(obj_path): raise FileNotFoundError(obj_path)
    os.makedirs(os.path.dirname(out_path),exist_ok=True); os.makedirs(args.render_dir,exist_ok=True)
    clear_scene(); scene=bpy.context.scene
    scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1.0
    imported=import_obj(obj_path)
    tune_vehicle_materials()
    for obj in imported:
        if obj.type=='MESH':
            if obj.name.startswith('Body_Blockout'):
                for poly in obj.data.polygons: poly.use_smooth=True
                bevel=obj.modifiers.new('Soft body edge highlights','BEVEL'); bevel.width=.012; bevel.segments=3
                bevel.profile=.5
                normal=obj.modifiers.new('Body weighted normals','WEIGHTED_NORMAL'); normal.keep_sharp=True
            elif obj.name.startswith(('Tire_','Rim_','Brake_','Hub_','Headlamp','TailLamp','Mirror_')):
                for poly in obj.data.polygons: poly.use_smooth=True
            # Move each mesh origin to its own bounds center so parts are easy to edit.
            bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True); bpy.context.view_layer.objects.active=obj
            bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY',center='BOUNDS')
    # Camera/light staging collection.
    stage=bpy.data.collections.new('Studio_Setup'); scene.collection.children.link(stage)
    floor_mat=material('Studio_Floor',(.055,.075,.105),.05,.52)
    bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.012))
    floor=bpy.context.object; floor.name='Studio_Floor'; floor.data.materials.append(floor_mat)
    for c in list(floor.users_collection): c.objects.unlink(floor)
    stage.objects.link(floor)
    world=bpy.data.worlds.new('Studio_World') if not bpy.data.worlds else bpy.data.worlds[0]
    scene.world=world; world.use_nodes=True
    world.node_tree.nodes['Background'].inputs['Color'].default_value=(.22,.27,.35,1)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value=.35
    aim=(0,0,.62)
    lights=[area_light('Key_Softbox',(-4,-4,7),1200,5,(1,.91,.80),aim),
            area_light('Fill_Softbox',(2,-5,3.8),850,4,(.73,.83,1),aim),
            area_light('Rim_Softbox',(1,4,5.5),1500,3.5,(1,1,1),aim),
            area_light('Front_Strip',(-6,1,2.6),500,3,(.8,.9,1),aim)]
    for obj in lights:
        for c in list(obj.users_collection): c.objects.unlink(obj)
        stage.objects.link(obj)
    cam_data=bpy.data.cameras.new('Studio_Camera'); cam=bpy.data.objects.new('Studio_Camera',cam_data); stage.objects.link(cam)
    cam_data.lens=58; scene.camera=cam
    scene.render.engine='CYCLES'
    scene.cycles.samples=48
    scene.cycles.use_denoising=True
    scene.render.resolution_x=1600; scene.render.resolution_y=1100; scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'; scene.render.film_transparent=False
    scene.view_settings.view_transform='AgX'
    scene.render.image_settings.color_mode='RGBA'
    views=[('front_3_4',(-6.5,-7.2,3.2)),('side',(-.1,-8.9,2.5)),('rear_3_4',(6.5,7.2,3.1))]
    for suffix,pos in views:
        cam.location=pos; aim_camera(cam,aim)
        scene.render.filepath=os.path.join(args.render_dir,'gr86_'+suffix+'.png')
        bpy.ops.render.render(write_still=True)
    cam.location=views[0][1]; aim_camera(cam,aim); scene.render.filepath='//renders/gr86_front_3_4.png'
    # Keep imported vehicle and studio setup neatly grouped.
    vehicle=bpy.data.collections.new('GR86_Imported_Model'); scene.collection.children.link(vehicle)
    for obj in imported:
        for c in list(obj.users_collection): c.objects.unlink(obj)
        vehicle.objects.link(obj)
    bpy.ops.wm.save_as_mainfile(filepath=out_path)
    print('Saved Blender scene:',out_path)

if __name__=='__main__': main()
