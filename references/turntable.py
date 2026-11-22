"""Облёт камеры вокруг машины + анимация колёс/руля -> кадры для видео."""
import bpy, math, mathutils

sc = bpy.context.scene
sc.render.engine = 'CYCLES'
sc.cycles.samples = 20
sc.cycles.use_denoising = True
sc.render.resolution_x, sc.render.resolution_y = 720, 405
sc.render.resolution_percentage = 100
sc.view_settings.exposure = 0.0

cam = bpy.data.objects["Cam_Exterior"]
# риг облёта
rig = bpy.data.objects.new("Turntable_Rig", None)
sc.collection.objects.link(rig)
rig.location = (0, 0, 0)
cam.parent = rig
cam.matrix_parent_inverse = mathutils.Matrix.Identity(4)
cam.location = (5.9, -4.9, 1.75)
d = mathutils.Vector((0.05, 0.0, 0.76)) - cam.location
cam.rotation_mode = 'XYZ'
cam.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
cam.data.lens = 55
cam.data.dof.use_dof = True
cam.data.dof.aperture_fstop = 5.6
cam.data.dof.focus_distance = d.length

rig.rotation_mode = 'XYZ'
rig.rotation_euler = (0, 0, 0)
rig.keyframe_insert("rotation_euler", index=2, frame=1)
rig.rotation_euler = (0, 0, math.radians(-170))
rig.keyframe_insert("rotation_euler", index=2, frame=241)
for fc in rig.animation_data.action.fcurves:
    for kp in fc.keyframe_points:
        kp.interpolation = 'LINEAR'

sc.frame_start, sc.frame_end = 1, 241
sc.render.fps = 24
sc.camera = cam
sc.render.filepath = "/home/user/frames/tt_"
sc.render.image_settings.file_format = 'PNG'
bpy.ops.render.render(animation=True)
