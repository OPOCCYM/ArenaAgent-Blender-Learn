import bpy, math, sys
from mathutils import Vector
scn = bpy.context.scene
scn.render.engine='CYCLES'; scn.cycles.samples=24; scn.cycles.use_denoising=True
scn.render.resolution_x=700; scn.render.resolution_y=520; scn.render.resolution_percentage=100
scn.view_settings.exposure=0.0
cd=bpy.data.cameras.new("zc"); cam=bpy.data.objects.new("zc",cd); scn.collection.objects.link(cam); scn.camera=cam
cd.lens=85; cd.dof.use_dof=False
shots = {
 "nose":   ((5.6,-2.2,1.15),(2.15,0.0,0.80)),
 "corner": ((3.9,-3.2,1.35),(1.55,-0.85,0.72)),
 "tail":   ((-5.4,-2.0,1.25),(-2.10,0.0,0.95)),
 "doors":  ((2.0,-4.4,1.05),(-0.25,-0.9,0.78)),
 "archf":  ((3.05,-2.45,0.80),(1.60,-0.80,0.55)),
 "roof":   ((2.4,-3.4,2.35),(-0.25,-0.35,1.38)),
}
import sys
only=[a for a in sys.argv[sys.argv.index("--")+1:]] if "--" in sys.argv else []
for name,(loc,tgt) in shots.items():
    if only and name not in only:
        continue
    cam.location=loc
    d=Vector(tgt)-Vector(loc)
    cam.rotation_euler=d.to_track_quat('-Z','Y').to_euler()
    cd.dof.focus_distance=d.length
    scn.render.filepath="/home/user/z/%s.png"%name
    bpy.ops.render.render(write_still=True)
print("ZOOM done")
