"""Run inside the Arena Blender WASM runtime against assets/gr86_studio.blend.
Invoke from repo root:
  python scripts/run_blender_wasm.py tests/validate_gr86_blend.py --input assets/gr86_studio.blend
"""
import bpy
import math

BLEND='/in/assets/gr86_studio.blend'
bpy.ops.wm.open_mainfile(filepath=BLEND)
scene=bpy.context.scene
assert bpy.app.version[0] >= 4
assert scene.render.engine in {'BLENDER_EEVEE','BLENDER_EEVEE_NEXT'}
body=bpy.data.objects.get('Body_Shell')
assert body and body.type=='MESH'
assert abs(body.dimensions.x-4.264)<.02, body.dimensions[:]
for name in ('Hood_Panel','Trunk_Lid','Door_L','Door_R','Front_Fender_L','Front_Fender_R','Rear_Quarter_L','Rear_Quarter_R'):
    panel=bpy.data.objects.get(name)
    assert panel and panel.type=='MESH' and len(panel.data.polygons)>20, f'missing separated body panel: {name}'
assert body.dimensions.z>.8, body.dimensions[:]
assert bpy.data.objects.get('Body_Shell') and any(o.name.startswith('Front_Door_Glass_') for o in scene.objects), 'window aperture/glazing assembly missing'
assert all(any(o.name.startswith(prefix) for o in scene.objects) for prefix in ('A_Pillar_','B_Pillar_','C_Pillar_')), 'painted glazing pillars missing'
for name in ('Windscreen_Glass','Backlight_Glass','Front_Door_Glass_+1','Rear_Quarter_Glass_+1'):
    pane=bpy.data.objects.get(name)
    assert pane and pane.type=='MESH' and len(pane.data.polygons)>=100, f'missing fitted curved glazing: {name}'
    assert any(slot==bpy.data.materials.get('Glass_Dark') for slot in pane.data.materials), f'glass material missing: {name}'
# Structural mesh sanity: catch NaNs and zero-area polygons before export.
mesh_objects=[o for o in scene.objects if o.type=='MESH']
nonfinite=[]; degenerate=[]
for ob in mesh_objects:
    for v in ob.data.vertices:
        if not all(math.isfinite(float(c)) for c in v.co): nonfinite.append(ob.name); break
    for poly in ob.data.polygons:
        if poly.area<1e-10: degenerate.append((ob.name,poly.index))
assert not nonfinite, f'non-finite vertices: {nonfinite[:5]}'
assert not degenerate, f'degenerate polygons: {degenerate[:8]}'
assert len([o for o in bpy.data.objects if o.name.startswith('Tire_')])==4
assert len([o for o in bpy.data.objects if o.name.startswith('WheelRoot_')])==4
for axle,x in [('Front',-1.2875),('Rear',1.2875)]:
    for side,letter in [(-1,'R'),(1,'L')]:
        root=bpy.data.objects[f'WheelRoot_{axle}_{letter}']
        tire=bpy.data.objects[f'Tire_{axle}_{side}']
        assert tire.parent==root, (tire.name,tire.parent)
        assert (root.matrix_world.translation-tire.matrix_world.translation).length<1e-4
        assert abs(root.matrix_world.translation.x-x)<1e-4
        assert abs(root.matrix_world.translation.y-side*.715)<1e-4
        assert abs(root.matrix_world.translation.z-.337)<1e-4
for name in ('Pearl_White','Glass_Dark','Tire_Rubber'):
    assert bpy.data.materials.get(name), name
for name in ('Cam_Exterior','Cam_Side','Cam_Rear','Cam_Cockpit','Key_Softbox','Fill_Softbox','Rim_Softbox','Cockpit_Fill'):
    assert bpy.data.objects.get(name), name
assert bpy.data.objects.get('Front_Grille') and bpy.data.objects.get('TailLamp_1')
for name in ('Interior_Floor','Dashboard','Center_Console','Infotainment_Display','Instrument_Cluster','Rear_Seat_Cushion','Front_Seat_Cushion_1','Front_Seat_Back_1','Front_Seat_Headrest_1','Door_Card_1','Door_Armrest_1','Steering_Wheel','Steering_Spoke_0'):
    ob=bpy.data.objects.get(name)
    assert ob and ob.type=='MESH', f'missing cockpit geometry: {name}'
    assert ob.dimensions.x>0 and ob.dimensions.y>0 and ob.dimensions.z>0
headrest=bpy.data.objects['Front_Seat_Headrest_1']
assert headrest.location.z+headrest.dimensions.z/2 < 1.22, 'front headrests must remain below the roof line'
cockpit=bpy.data.objects['Cam_Cockpit']
assert -.4<cockpit.location.y<-.2 and .9<cockpit.location.z<1.4
assert cockpit.location.x<.6, 'cockpit camera must remain inside passenger cabin'
assert cockpit.data.lens<35, 'cockpit camera should use a gameplay-like wide field of view'
print('PASS: Blender scene',bpy.app.version_string,'objects',len(scene.objects),'body dimensions',tuple(round(v,3) for v in body.dimensions))
print('PASS: 4 wheel pivots and parented tire assemblies aligned')
print('PASS: materials, studio lights/cameras, interior geometry and cockpit camera exist')
print('PASS: mesh geometry finite and non-degenerate across',len(mesh_objects),'mesh objects')
