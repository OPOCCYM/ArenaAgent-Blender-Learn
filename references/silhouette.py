"""Сравнение силуэта модели с эталоном: маска + наложение контуров."""
import bpy, math, numpy as np
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

SC = '/home/user/'
scn = bpy.context.scene

def mask_render(path, res=(1600, 640)):
    m = bpy.data.materials.new("__mask"); m.use_nodes = True
    nt = m.node_tree; nt.nodes.clear()
    em = nt.nodes.new("ShaderNodeEmission"); em.inputs[0].default_value = (0,0,0,1)
    out = nt.nodes.new("ShaderNodeOutputMaterial"); nt.links.new(em.outputs[0], out.inputs[0])
    old_override = scn.view_layers[0].material_override
    scn.view_layers[0].material_override = m
    w = bpy.data.worlds.new("__mw"); w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = (1,1,1,1)
    old_w = scn.world; scn.world = w
    old_engine = scn.render.engine; scn.render.engine = 'CYCLES'
    old_s = scn.cycles.samples; scn.cycles.samples = 4
    old_dn = scn.cycles.use_denoising; scn.cycles.use_denoising = False
    old_vt = scn.view_settings.view_transform; scn.view_settings.view_transform='Standard'
    cd = bpy.data.cameras.new("__oc"); cd.type='ORTHO'; cd.ortho_scale=5.6
    cam = bpy.data.objects.new("__oc", cd); scn.collection.objects.link(cam)
    cam.location = (0, -9, 0.75)
    cam.rotation_euler = Vector((0,1,0)).to_track_quat("-Z","Y").to_euler()
    old_cam = scn.camera; scn.camera = cam
    scn.render.resolution_x, scn.render.resolution_y = res
    scn.render.resolution_percentage = 100
    scn.render.filepath = path
    bpy.ops.render.render(write_still=True)
    # restore
    scn.camera = old_cam; scn.world = old_w; scn.render.engine = old_engine
    scn.cycles.samples = old_s; scn.cycles.use_denoising = old_dn
    scn.view_layers[0].material_override = old_override
    scn.view_settings.view_transform = old_vt
    bpy.data.objects.remove(cam, do_unlink=True)
    return res

res = mask_render(SC+'q/mask_model.png')
h, w = res[1], res[0]
img = bpy.data.images.load(SC+'q/mask_model.png')
px = np.array(img.pixels[:], dtype=np.float32).reshape(img.size[1], img.size[0], 4)
bpy.data.images.remove(img)
m = px[:, :, 0] < 0.5   # строки идут снизу вверх
# калибровка: ortho, камера смотрит по -X; u(0.05)= -2.5 ... u(0.95)=+2.5 (как в эталоне)
def px2world(px_col, px_row_from_bottom):
    u = (px_col+0.5)/w
    x = -2.5 + (u-0.05)*5/0.9
    v = (px_row_from_bottom)/h
    z = 0.1 + (v-0.21)*1.3/0.58
    return x, z
def world2px(x, z):
    u = (x+2.5)/5*0.9+0.05
    v = (z-0.1)/1.3*0.58+0.21
    return int(u*w), int(v*h)
rows=[]
for pcol in range(w):
    col = np.nonzero(m[:, pcol])[0]
    if len(col)==0: continue
    x, zb = px2world(pcol, col.max())
    _, zt = px2world(pcol, col.min())
    rows.append((x, zb, zt))
print("model silhouette: x %.3f..%.3f, pts %d" % (rows[0][0], rows[-1][0], len(rows)))

# эталон
ref = np.load(SC+'ref/sil_ref.npy')   # (n,3): x, zbot, ztop
np.save(SC+'q/sil_model.npy', np.array(rows))
def at(arr, x):
    i = int(np.argmin(np.abs(arr[:,0]-x)))
    return arr[i,1], arr[i,2]
print("   x    model(bot,top)      ref(bot,top)")
for x in [-2.2,-1.8,-1.4,-1.0,-0.6,-0.2,0.2,0.6,1.0,1.4,1.8,2.2]:
    mb, mt = at(np.array(rows), x); rb, rt = at(ref, x)
    print(f"{x:+5.1f}  {mb:.3f} {mt:.3f}    {rb:.3f} {rt:.3f}   dTop={mt-rt:+.3f} dBot={mb-rb:+.3f}")
