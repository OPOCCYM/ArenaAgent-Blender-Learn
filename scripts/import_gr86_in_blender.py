"""Build an editable Blender scene from the generated GR86 OBJ.

Works in Blender 4.x+ (including the Arena Blender WebAssembly runtime). The WebAssembly
runtime can build/save the scene but cannot render; open the saved .blend in desktop Blender
for Cycles rendering. Coordinates: X = length, front is -X; Y = width; Z = up.
"""
import os
import re
import math
import bpy
from mathutils import Vector

ROOT=os.getcwd()
OBJ_PATH='/in/assets/gr86_blockout.obj' if os.path.isfile('/in/assets/gr86_blockout.obj') else os.path.join(ROOT,'assets','gr86_blockout.obj')
OUT_PATH='/out/gr86_studio.blend' if os.path.isdir('/out') else os.path.join(ROOT,'assets','gr86_studio.blend')


def material(name, color, metallic=0.0, roughness=.35):
    mat=bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.diffuse_color=(*color,1.0); mat.use_nodes=True
    bsdf=mat.node_tree.nodes.get('Principled BSDF')
    if bsdf:
        bsdf.inputs['Base Color'].default_value=(*color,1.0)
        bsdf.inputs['Metallic'].default_value=metallic
        bsdf.inputs['Roughness'].default_value=roughness
    return mat


def tune_materials():
    paint=bpy.data.materials.get('Pearl_White')
    if paint:
        paint.diffuse_color=(.68,.70,.73,1.0); paint.use_nodes=True
        nodes=paint.node_tree.nodes; links=paint.node_tree.links; bsdf=nodes.get('Principled BSDF')
        if bsdf:
            bsdf.inputs['Base Color'].default_value=(.68,.70,.73,1.0)
            bsdf.inputs['Metallic'].default_value=.22
            bsdf.inputs['Roughness'].default_value=.23
            if 'Coat Weight' in bsdf.inputs:
                bsdf.inputs['Coat Weight'].default_value=.82
                bsdf.inputs['Coat Roughness'].default_value=.035
            noise=nodes.new('ShaderNodeTexNoise'); noise.inputs['Scale'].default_value=1800
            bump=nodes.new('ShaderNodeBump'); bump.inputs['Strength'].default_value=.012
            bump.inputs['Distance'].default_value=.001
            links.new(noise.outputs['Fac'],bump.inputs['Height']); links.new(bump.outputs['Normal'],bsdf.inputs['Normal'])
    glass=bpy.data.materials.get('Glass_Dark')
    if glass:
        glass.diffuse_color=(.025,.045,.06,1); glass.use_nodes=True
        bsdf=glass.node_tree.nodes.get('Principled BSDF')
        if bsdf:
            bsdf.inputs['Base Color'].default_value=(.025,.045,.06,1)
            bsdf.inputs['Metallic'].default_value=.04; bsdf.inputs['Roughness'].default_value=.075
            if 'Transmission Weight' in bsdf.inputs:
                bsdf.inputs['Transmission Weight'].default_value=.78
            elif 'Transmission' in bsdf.inputs:
                bsdf.inputs['Transmission'].default_value=.78
            if 'IOR' in bsdf.inputs:
                bsdf.inputs['IOR'].default_value=1.46
            if 'Coat Weight' in bsdf.inputs:
                bsdf.inputs['Coat Weight'].default_value=.65; bsdf.inputs['Coat Roughness'].default_value=.025


def aim(obj, target):
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()


def make_area_light(name, location, energy, size, color, target, collection):
    data=bpy.data.lights.new(name,'AREA'); data.energy=energy; data.shape='DISK'; data.size=size; data.color=color
    obj=bpy.data.objects.new(name,data); collection.objects.link(obj); obj.location=location; aim(obj,target)
    return obj


# Dense parametric shell adapted from the supplied Blender car-build workflow.
# Front is -X. Key curves are editable proportion controls, not claimed scan measurements.
CURVES={
 'width':[(-2.132,.62),(-1.92,.76),(-1.45,.85),(-1.27,.88),(-.82,.86),(-.25,.84),(.35,.85),(.92,.87),(1.30,.88),(1.55,.86),(1.92,.77),(2.132,.62)],
 'bottom':[(-2.132,.39),(-1.92,.34),(-1.45,.32),(-.8,.34),(0,.35),(.8,.34),(1.45,.32),(1.92,.34),(2.132,.39)],
 'belt':[(-2.132,.76),(-1.92,.86),(-1.42,.92),(-.95,.94),(-.55,.945),(.35,.945),(.62,.925),(1.0,.905),(1.5,.89),(1.92,.84),(2.132,.76)],
 'top':[(-2.132,.70),(-1.92,.82),(-1.55,.90),(-1.20,.925),(-.94,.96),(-.74,1.10),(-.56,1.255),(-.28,1.295),(.12,1.30),(.34,1.275),(.52,1.21),(.72,1.08),(.92,.955),(1.18,.925),(1.58,.895),(1.92,.82),(2.132,.72)],
 'roof_width':[(-2.132,.38),(-1.7,.47),(-1.25,.55),(-.9,.60),(-.55,.64),(-.25,.65),(.15,.64),(.42,.62),(.72,.57),(1.05,.52),(1.5,.48),(2.132,.38)],
 'shoulder':[(-2.132,.64),(-1.8,.72),(-1.3,.76),(-.6,.77),(0,.77),(.7,.76),(1.3,.75),(1.8,.71),(2.132,.63)],
}

def smooth_curve(keys,x):
    if x<=keys[0][0]: return keys[0][1]
    if x>=keys[-1][0]: return keys[-1][1]
    i=next(i for i in range(len(keys)-1) if keys[i][0]<=x<=keys[i+1][0])
    x0,y0=keys[i]; x1,y1=keys[i+1]; h=x1-x0; t=(x-x0)/h
    p0=keys[max(0,i-1)]; p3=keys[min(len(keys)-1,i+2)]
    m0=(y1-p0[1])/(x1-p0[0]) if x1!=p0[0] else 0
    m1=(p3[1]-y0)/(p3[0]-x0) if p3[0]!=x0 else 0
    return (2*t**3-3*t**2+1)*y0+(t**3-2*t**2+t)*m0*h+(-2*t**3+3*t**2)*y1+(t**3-t**2)*m1*h

def wheel_opening(x):
    z=.337; radius=.392; result=None
    for cx in (-1.2875,1.2875):
        dx=abs(x-cx)
        if dx<radius:
            h=z+math.sqrt(max(0,radius*radius-dx*dx))
            result=max(result or 0,h)
    return result

def build_parametric_shell():
    old_prefixes=('Body_Blockout','Cabin_Glazing_Blockout','Side_Glass_','Front_Windshield_',
                  'Rear_Glass_','A_Pillar_','B_Pillar_','C_Pillar_','Roof_Panel')
    for obj in list(bpy.context.scene.objects):
        if obj.type=='MESH' and obj.name.startswith(old_prefixes):
            bpy.data.objects.remove(obj,do_unlink=True)
    # Face bands share one manifold-like outer loft; glass and trim are material regions.
    n_half=19; n_ring=2*n_half-2; body_end=10; roof_start=15
    sections=[-2.132+4.264*i/240 for i in range(241)]
    sections=sorted(set(sections+[-1.2875,1.2875,-.56,-.53,.15,.53,.56]))
    verts=[]; rings=[]
    for x in sections:
        w=smooth_curve(CURVES['width'],x); zb=smooth_curve(CURVES['bottom'],x)
        belt=smooth_curve(CURVES['belt'],x); top=smooth_curve(CURVES['top'],x)
        roofw=min(smooth_curve(CURVES['roof_width'],x),.94*w)
        shoulder=smooth_curve(CURVES['shoulder'],x); dz=max(.015,top-belt)
        # Lower side, shoulder and belt-line points, from underside centre outward.
        body=[(0,zb),(.36*w,zb),(.66*w,zb+.010),(.81*w,zb+.045),
              (.90*w,zb+.125),(.955*w,shoulder-.11),(.975*w,shoulder-.045),(.993*w,shoulder),
              (w,shoulder+.050),(.987*w,belt-.055),(.95*w,belt)]
        # Subtle convex door crowns and wheel-arch shoulder swell, built into the skin.
        for k in range(3,len(body)):
            y,z=body[k]; fraction=max(0,min(1,(z-zb)/max(.03,belt-zb)))
            bulge=0.0
            if -.75<x<.75:
                u=(x+.75)/1.5
                bulge+=.010*math.sin(math.pi*u)*math.sin(math.pi*fraction)
            for wheel_x in (-1.2875,1.2875):
                bulge+=.012*math.exp(-((x-wheel_x)/.43)**2)*math.sin(math.pi*min(1,max(0,(k-2)/9)))
            if 5<=k<=8: bulge+=.003
            body[k]=(y+bulge,z)
        opening=wheel_opening(x)
        if opening is not None:
            for k in (3,4,5):
                y,z=body[k]
                weight=max(0.0,min(1.0,(y/w-.76)/.22))
                sill=zb+(opening-zb)*weight
                if z<sill: body[k]=(y,sill)
        # Tumblehome and greenhouse move inward as they rise to the roof.
        glass=[(.91*w,belt+.22*dz),(.84*w,belt+.50*dz),(.74*w,belt+.80*dz),
               (.68*w,top-.015)]
        roof=[(.63*roofw/0.68,top),(.42*roofw/0.68,top+.012),
              (.20*roofw/0.68,top+.018),(0,top+.020)]
        half=body+glass+roof
        # Softly tuck the nose and tail ends toward their centreline.
        tip=max(max(0,1-(x+2.132)/.11),max(0,1-(2.132-x)/.11))
        if tip:
            half=[(y*(1-.20*tip),z) for y,z in half]
        ring=half+[(-y,z) for y,z in reversed(half[1:-1])]
        rings.append(ring)
        verts.extend((x,y,z) for y,z in ring)
    faces=[]; face_mats=[]; face_parts=[]
    trim_centres=(-.55,.15,.55)
    for i in range(len(sections)-1):
        xm=.5*(sections[i]+sections[i+1])
        for j in range(n_ring):
            j2=(j+1)%n_ring
            h1=j if j<n_half else n_ring-j
            h2=j2 if j2<n_half else n_ring-j2
            lo,hi=min(h1,h2),max(h1,h2)
            in_glass_band=lo>=body_end and hi<=roof_start
            in_window_span=(-.96<xm<1.00)
            # Leave actual apertures in the opaque shell for the separate glazing meshes.
            if in_glass_band and in_window_span:
                continue
            face=(i*n_ring+j,i*n_ring+j2,(i+1)*n_ring+j2,(i+1)*n_ring+j)
            faces.append(face)
            mi=2 if in_glass_band and any(abs(xm-c)<.024 for c in trim_centres) else 0
            face_mats.append(mi)
            part='Body_Shell'
            # Split side skin into independently selectable panel objects. Keep seams on
            # existing loft section boundaries and the door-gap locations.
            side=1 if j<n_half else -1
            if lo>=2 and hi<=11:
                if -1.90<xm<-.75: part=f'Front_Fender_{"L" if side>0 else "R"}'
                elif -.75<xm<.75: part=f'Door_{"L" if side>0 else "R"}'
                elif .75<xm<1.90: part=f'Rear_Quarter_{"L" if side>0 else "R"}'
            # Hood and trunk are separate crown/upper-side skins; the aperture ends
            # align to the front cowl and rear deck break.
            if lo>=10 and hi<=18:
                if xm<-.97: part='Hood_Panel'
                elif xm>1.00: part='Trunk_Lid'
            # Roof is a discrete shell, leaving its perimeter adjacent to the A/C
            # pillars; pillars bridge it to the beltline around the glazing.
            if lo>=15 and hi<=18 and -.56<xm<.53:
                part='Roof_Panel'
            # Full-width front and rear bumper covers are separate stamped fascia shells.
            if xm< -1.82: part='Front_Bumper'
            elif xm>1.82: part='Rear_Bumper'
            face_parts.append(part)
    faces.append(tuple(reversed(range(n_ring)))); face_mats.append(0); face_parts.append('Front_Bumper')
    last=(len(sections)-1)*n_ring
    faces.append(tuple(last+j for j in range(n_ring))); face_mats.append(0); face_parts.append('Rear_Bumper')
    mats=[bpy.data.materials.get('Pearl_White'),bpy.data.materials.get('Glass_Dark'),
          bpy.data.materials.get('Window_Trim') or material('Window_Trim',(.012,.016,.022),.1,.24)]
    components={}
    for idx,part in enumerate(face_parts): components.setdefault(part,[]).append(idx)
    shell=None
    for part,face_ids in components.items():
        part_faces=[faces[k] for k in face_ids]
        used=sorted({v for face in part_faces for v in face})
        remap={old:new for new,old in enumerate(used)}
        mesh=bpy.data.meshes.new(f'{part}_Mesh')
        mesh.from_pydata([verts[k] for k in used],[],[tuple(remap[v] for v in f) for f in part_faces])
        mesh.validate(verbose=False); mesh.update()
        for mat in mats: mesh.materials.append(mat)
        for poly,source_id in zip(mesh.polygons,face_ids):
            # Keep the primary shell white; glazing is independent pane geometry.
            poly.material_index=0 if face_mats[source_id]==1 else face_mats[source_id]
            poly.use_smooth=True
        ob=bpy.data.objects.new(part,mesh); bpy.data.collections['GR86_Model'].objects.link(ob)
        ob['component_type']='main body shell' if part=='Body_Shell' else 'separate exterior panel'
        ob['construction']='Split from dense longitudinal loft; independent mesh object'
        if part in {'Hood_Panel','Trunk_Lid','Roof_Panel','Front_Bumper','Rear_Bumper'} or part.startswith(('Door_','Front_Fender_','Rear_Quarter_')):
            solid=ob.modifiers.new('Panel return / sheet thickness','SOLIDIFY')
            solid.thickness=.008 if part.startswith('Door_') else .010
            solid.offset=-.25
            edge=ob.modifiers.new('Pressed panel edge','BEVEL')
            edge.width=.0025; edge.segments=2
        if part=='Body_Shell': shell=ob
    shell['reference_status']='Proportional draft; verify against GR86 orthographic references'
    # Side glazing: subdivided curved patches fitted to the loft's shoulder and roof
    # coordinates, split into front door glass and rear quarter glass by a B-pillar.
    glass_mat=bpy.data.materials.get('Glass_Dark')
    def pane(name, grid):
        rows=len(grid); cols=len(grid[0]); pv=[p for row in grid for p in row]
        pf=[]
        for r in range(rows-1):
            for c in range(cols-1):
                a=r*cols+c; pf.append((a,a+1,a+1+cols,a+cols))
        pm=bpy.data.meshes.new(name+'_Mesh'); pm.from_pydata(pv,[],pf); pm.materials.append(glass_mat); pm.update()
        po=bpy.data.objects.new(name,pm); bpy.data.collections['GR86_Model'].objects.link(po)
        for poly in pm.polygons: poly.use_smooth=True
        return po
    # Side windows sit just outside the body skin; sampled densely for smooth highlights.
    for side in (-1,1):
        window_specs=[
            # Windshield/A-pillar sweep forward, with a near-vertical rear door edge.
            ('Front_Door_Glass',-.88,.14,-.56,.12),
            # Fastback quarter glass tapers into the short C-pillar.
            ('Rear_Quarter_Glass',.20,.91,.18,.56),
        ]
        for name,x_lower_a,x_lower_b,x_upper_a,x_upper_b in window_specs:
            grid=[]
            for r in range(9):
                v=r/8; row=[]
                x0=x_lower_a+(x_upper_a-x_lower_a)*v
                x1=x_lower_b+(x_upper_b-x_lower_b)*v
                for c in range(25):
                    u=c/24; x=x0+(x1-x0)*u
                    belt=smooth_curve(CURVES['belt'],x); top=smooth_curve(CURVES['top'],x)
                    z=belt+(top-belt)*(.17+.72*v)
                    w=smooth_curve(CURVES['width'],x)
                    # The upper edge tucks toward the roof, while the lower edge follows
                    # the beltline; a shallow crown across the pane avoids a flat-card look.
                    frac=.77-.17*v
                    y=side*(w*frac+.003+.010*math.sin(math.pi*u)*math.sin(math.pi*v))
                    row.append((x,y,z))
                grid.append(row)
            pane(f'{name}_{side:+d}',grid)
        # Painted A/B/C pillars follow the fitted glazing edge paths. Narrow quad strips
        # sit proud of the glass and restore real body-colored breaks between panes.
        def pillar(name, lower_x, upper_x, z0=.945, z1=1.245, width=.026):
            verts=[]
            for v in (0,1):
                x=lower_x+(upper_x-lower_x)*v
                z=z0+(z1-z0)*v
                w=smooth_curve(CURVES['width'],x)
                frac=.77-.17*v
                y=side*(w*frac+.006)
                half=width*(1-.18*v)
                verts.extend([(x-half,y,z),(x+half,y,z)])
            me=bpy.data.meshes.new(name+'_Mesh'); me.from_pydata(verts,[],[(0,1,3,2)])
            me.materials.append(bpy.data.materials['Pearl_White']); me.update()
            ob=bpy.data.objects.new(name,me); bpy.data.collections['GR86_Model'].objects.link(ob)
            sol=ob.modifiers.new('Pillar rolled edge','SOLIDIFY'); sol.thickness=.008
            bev=ob.modifiers.new('Pillar edge highlight','BEVEL'); bev.width=.004; bev.segments=2
        pillar(f'A_Pillar_{side:+d}',-.88,-.56,width=.020)
        pillar(f'B_Pillar_{side:+d}',.12,.12,width=.018)
        pillar(f'C_Pillar_{side:+d}',.91,.55,width=.024)
    # Curved front and rear windscreens: 2-D grids bowed gently across width and height.
    for name,xlo,xhi,zlo,zhi in [('Windscreen',-.93,-.56,.965,1.255),('Backlight',1.04,.55,.965,1.245)]:
        grid=[]
        for r in range(13):
            v=r/12; row=[]
            for c in range(25):
                u=-1+2*c/24
                x=xlo+(xhi-xlo)*v + .045*(1-u*u)*math.sin(math.pi*v)
                z=zlo+(zhi-zlo)*v - .018*u*u
                width=smooth_curve(CURVES['roof_width'],x)*(.94-.08*v)
                y=u*width
                row.append((x,y,z))
            grid.append(row)
        pane(name+'_Glass',grid)
    shell['reference_status']='Proportional draft; verify against GR86 orthographic references'
    # Pressed arch lips and inner wheelhouse liners make the fender openings read as
    # formed sheet metal rather than a bare cut through the parametric shell.
    paint=bpy.data.materials.get('Pearl_White')
    liner=material('Wheelwell_Liner',(.028,.034,.042),.02,.78)
    for axle,cx in [('Front',-1.2875),('Rear',1.2875)]:
        for side,letter in [(-1,'R'),(1,'L')]:
            verts=[]; faces=[]; steps=40
            for radius in (.392,.407):
                for i in range(steps+1):
                    a=math.pi*i/steps
                    x=cx+radius*math.cos(a); z=.337+radius*math.sin(a)
                    # Follow the fender outer cross-section: lower arch starts near
                    # the rocker, and the crown reaches the upper wheel-arch shoulder.
                    lateral=.80+.173*math.sin(a)
                    y=side*(smooth_curve(CURVES['width'],x)*lateral+.002)
                    verts.append((x,y,z))
            for i in range(steps): faces.append((i,i+1,steps+2+i,steps+1+i))
            me=bpy.data.meshes.new(f'Arch_Lip_{axle}_{letter}_Mesh'); me.from_pydata(verts,[],faces); me.update()
            me.materials.append(paint); lip=bpy.data.objects.new(f'Arch_Lip_{axle}_{letter}',me)
            bpy.data.collections['GR86_Model'].objects.link(lip)
            for p in me.polygons: p.use_smooth=True
            solid=lip.modifiers.new('Rolled wheel-arch return','SOLIDIFY'); solid.thickness=.003
            bevel=lip.modifiers.new('Arch lip edge','BEVEL'); bevel.width=.0015; bevel.segments=2
            # Full-width dark arch inner surface closes the wheelhouse cavity behind the tire.
            lv=[]; lf=[]
            for y in (.51,.82):
                for i in range(steps+1):
                    a=math.pi*i/steps
                    lv.append((cx+.392*math.cos(a),side*y,.337+.392*math.sin(a)))
            for i in range(steps): lf.append((i,i+1,steps+2+i,steps+1+i))
            lm=bpy.data.meshes.new(f'Wheelhouse_{axle}_{letter}_Mesh'); lm.from_pydata(lv,[],lf); lm.update()
            lm.materials.append(liner); well=bpy.data.objects.new(f'Wheelhouse_Liner_{axle}_{letter}',lm)
            bpy.data.collections['GR86_Model'].objects.link(well)
            for p in lm.polygons: p.use_smooth=True
    # Subtle stamped character beads: paired hood crowns and a shoulder break on each door.
    for side,letter in [(-1,'R'),(1,'L')]:
        verts=[]; faces=[]; count=32
        for i in range(count+1):
            x=-1.84+(.78*i/count); w=smooth_curve(CURVES['width'],x)
            belt=smooth_curve(CURVES['belt'],x); top=smooth_curve(CURVES['top'],x)
            y0=side*.315; q=min(1,abs(y0)/max(.01,w))
            z=top+(belt-top)*q*q+.004
            for dy in (-.009,.009): verts.append((x,y0+dy,z))
        for i in range(count): faces.append((2*i,2*i+1,2*i+3,2*i+2))
        me=bpy.data.meshes.new(f'Hood_Crease_{letter}_Mesh'); me.from_pydata(verts,[],faces); me.update()
        me.materials.append(paint); ob=bpy.data.objects.new(f'Hood_Crease_{letter}',me)
        bpy.data.collections['GR86_Model'].objects.link(ob)
        for p in me.polygons: p.use_smooth=True
        sol=ob.modifiers.new('Pressed hood crown','SOLIDIFY'); sol.thickness=.002
        dverts=[]; dfaces=[]
        for i in range(count+1):
            x=-.54+1.08*i/count; w=smooth_curve(CURVES['width'],x)
            # Door skin shoulder bead, offset just outside the curved side panel.
            z=.690+.006*math.sin(math.pi*i/count)
            y=side*(w*.960+.002)
            dverts.extend([(x,y-side*.0025,z),(x,y+side*.0025,z)])
        for i in range(count): dfaces.append((2*i,2*i+1,2*i+3,2*i+2))
        dm=bpy.data.meshes.new(f'Door_Crease_{letter}_Mesh'); dm.from_pydata(dverts,[],dfaces); dm.update()
        dm.materials.append(paint); doorline=bpy.data.objects.new(f'Door_Crease_{letter}',dm)
        bpy.data.collections['GR86_Model'].objects.link(doorline)
        for p in dm.polygons: p.use_smooth=True
        edge=doorline.modifiers.new('Door character ridge','SOLIDIFY'); edge.thickness=.001
    return shell

def build_interior(vehicle):
    """Low-poly cockpit core placed behind the dark glazing for interior-camera use."""
    leather=material('Cabin_Charcoal',(.12,.13,.145),.03,.62)
    trim=material('Cabin_Trim',(.32,.34,.36),.22,.38)
    screen_mat=material('Cabin_Display',(.045,.28,.42),.18,.20)
    def box(name,loc,dim,mat,bevel=.025):
        bpy.ops.mesh.primitive_cube_add(size=1,location=loc)
        ob=bpy.context.object; ob.name=name; ob.dimensions=dim
        bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
        if mat: ob.data.materials.append(mat)
        if bevel:
            mod=ob.modifiers.new('Soft interior edges','BEVEL'); mod.width=bevel; mod.segments=2
        for c in list(ob.users_collection): c.objects.unlink(ob)
        vehicle.objects.link(ob)
        return ob
    box('Interior_Floor',(.18,0,.65),(1.35,1.22,.08),leather,.04)
    box('Dashboard',(-.47,0,.86),(.28,1.18,.18),trim,.055)
    box('Center_Console',(-.01,0,.76),(.68,.22,.13),leather,.045)
    box('Infotainment_Display',(-.60,0,.925),(.035,.24,.075),screen_mat,.012)
    box('Instrument_Cluster',(-.57,-.30,.985),(.035,.22,.075),screen_mat,.012)
    box('Rear_Seat_Cushion',(.72,0,.75),(.42,1.02,.16),leather,.06)
    box('Rear_Seat_Back',(.91,0,.91),(.16,1.02,.34),leather,.06)
    for side in (-1,1):
        cushion=box(f'Front_Seat_Cushion_{side}',(.13,side*.31,.77),(.48,.43,.14),leather,.055)
        back=box(f'Front_Seat_Back_{side}',(.43,side*.31,.94),(.17,.43,.40),leather,.055)
        back.rotation_euler[1]=math.radians(10)
        headrest=box(f'Front_Seat_Headrest_{side}',(.48,side*.31,1.17),(.12,.22,.075),leather,.035)
        headrest.rotation_euler[1]=math.radians(10)
        # These trimmed surfaces sit on the cabin side of their matching door skins;
        # they are reparented to the door hinge roots later in scene assembly.
        box(f'Door_Card_{side}',(.18,side*.785,.715),(.88,.038,.22),leather,.035)
        box(f'Door_Armrest_{side}',(.27,side*.752,.805),(.40,.052,.035),trim,.015)
        box(f'Door_Release_{side}',(-.39,side*.758,.805),(.085,.022,.035),trim,.012)
        bpy.ops.mesh.primitive_torus_add(major_radius=.057,minor_radius=.007,major_segments=24,minor_segments=8,
                                        location=(.48,side*.755,.66),rotation=(math.pi/2,0,0))
        speaker=bpy.context.object; speaker.name=f'Door_Speaker_{side}'; speaker.data.materials.append(trim)
        for c in list(speaker.users_collection): c.objects.unlink(speaker)
        vehicle.objects.link(speaker)
    bpy.ops.mesh.primitive_torus_add(major_radius=.15,minor_radius=.024,major_segments=32,minor_segments=10,
                                    location=(-.22,-.30,.99),rotation=(0,math.pi/2,0))
    wheel=bpy.context.object; wheel.name='Steering_Wheel'; wheel.data.materials.append(trim)
    for c in list(wheel.users_collection): c.objects.unlink(wheel)
    vehicle.objects.link(wheel)
    hub=box('Steering_Hub',(-.22,-.30,.99),(.07,.07,.07),trim,.02)
    hub.rotation_euler[1]=math.pi/2
    for idx,angle in enumerate((0,2*math.pi/3,4*math.pi/3)):
        sy=math.sin(angle)*.075; sz=math.cos(angle)*.075
        spoke=box(f'Steering_Spoke_{idx}',(-.22,-.30+sy,.99+sz),(.025,.018,.16),trim,.008)
        spoke.rotation_euler[0]=-angle


def build_engine_bay(vehicle):
    """Add the visible engine-bay hard points exposed by the hood-open review pose."""
    block_mat=material('Powertrain_Aluminium',(.22,.24,.25),.68,.34)
    cover_mat=material('Powertrain_Cover',(.12,.14,.16),.28,.34)
    bright=material('Powertrain_Metal',(.52,.55,.58),.78,.24)
    battery_mat=material('Powertrain_Battery',(.10,.12,.13),.06,.48)
    def cube(name,loc,size,mat,bevel=.015):
        bpy.ops.mesh.primitive_cube_add(size=1,location=loc)
        ob=bpy.context.object; ob.name=name; ob.dimensions=size
        bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
        ob.data.materials.append(mat)
        if bevel:
            m=ob.modifiers.new('Cast edge radii','BEVEL'); m.width=bevel; m.segments=2
        for c in list(ob.users_collection): c.objects.unlink(ob)
        vehicle.objects.link(ob)
        return ob
    cube('Engine_Block',(-1.35,0,.585),(.48,.52,.25),block_mat,.035)
    cube('Cylinder_Head_L',(-1.36,.145,.725),(.40,.20,.075),bright,.018)
    cube('Cylinder_Head_R',(-1.36,-.145,.725),(.40,.20,.075),bright,.018)
    cube('Intake_Plenum',(-1.17,0,.79),(.25,.40,.095),cover_mat,.028)
    cube('Engine_Cover',(-1.36,0,.815),(.34,.42,.045),cover_mat,.018)
    cube('Radiator_Core',(-1.91,0,.535),(.075,1.02,.32),cover_mat,.012)
    cube('Battery_Box',(-1.68,.43,.63),(.20,.18,.18),battery_mat,.018)
    for side in (-1,1):
        bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=.093,depth=.06,location=(-1.31,side*.49,.65))
        tower=bpy.context.object; tower.name=f'Strut_Tower_EngineBay_{side}'
        tower.data.materials.append(cover_mat)
        for c in list(tower.users_collection): c.objects.unlink(tower)
        vehicle.objects.link(tower)
    bpy.ops.mesh.primitive_cylinder_add(vertices=24,radius=.014,depth=.92,location=(-1.43,0,.735),rotation=(math.pi/2,0,0))
    brace=bpy.context.object; brace.name='Strut_Brace_EngineBay'; brace.data.materials.append(bright)
    for c in list(brace.users_collection): c.objects.unlink(brace)
    vehicle.objects.link(brace)


def build_crash_structure(vehicle):
    """Curved extruded bumper beams and tapered crush cans remain on the chassis."""
    beam_mat=material('Crash_Structure',(.25,.29,.32),.78,.28)
    def add_mesh(name,verts,faces,bevel=.006):
        me=bpy.data.meshes.new(name+'_Mesh'); me.from_pydata(verts,[],faces); me.validate(verbose=False); me.update()
        me.materials.append(beam_mat); ob=bpy.data.objects.new(name,me); vehicle.objects.link(ob)
        for p in me.polygons: p.use_smooth=True
        if bevel:
            mod=ob.modifiers.new('Stamped/extruded radii','BEVEL'); mod.width=bevel; mod.segments=2
        return ob
    # Broad shallow aluminum impact beam follows the bumper curvature in plan view.
    def beam(name,xbase,yspan,zcenter,depth,height,curve_sign):
        stations=20; profile=[(-.50,-.30),(-.38,-.50),(.38,-.50),(.50,-.30),
                              (.50,.30),(.38,.50),(-.38,.50),(-.50,.30)]
        verts=[]; faces=[]
        for i in range(stations+1):
            y=-yspan/2+yspan*i/stations; q=(2*y/yspan)**2
            x=xbase+curve_sign*.060*q
            for dx,dz in profile: verts.append((x+dx*depth,y,zcenter+dz*height))
        n=len(profile)
        for i in range(stations):
            for j in range(n):
                a=i*n+j; b=i*n+(j+1)%n
                faces.append((a,b,b+n,a+n))
        faces.extend([tuple(reversed(range(n))),tuple(stations*n+j for j in range(n))])
        return add_mesh(name,verts,faces,.006)
    beam('Front_Crash_Beam',-1.765,1.38,.505,.11,.125,1)
    beam('Rear_Crash_Beam',1.765,1.32,.505,.105,.12,-1)
    # Eight-sided stepped crush cans taper toward the chassis rails instead of being boxes.
    def can(name,x_inner,x_outer,y,z,wy_inner,wy_outer,hz):
        sides=8; verts=[]; faces=[]
        for x,wy in ((x_inner,wy_inner),(x_outer,wy_outer)):
            for i in range(sides):
                a=2*math.pi*i/sides+math.pi/8
                verts.append((x,y+wy*math.cos(a),z+hz*math.sin(a)))
        for i in range(sides): faces.append((i,(i+1)%sides,sides+(i+1)%sides,sides+i))
        faces.extend([tuple(reversed(range(sides))),tuple(sides+i for i in range(sides))])
        return add_mesh(name,verts,faces,.004)
    for side in (-1,1):
        can(f'Front_Crush_Can_{side}',-1.61,-1.72,side*.48,.505,.060,.074,.064)
        can(f'Rear_Crush_Can_{side}',1.61,1.72,side*.46,.505,.058,.070,.061)


def build_lamp_optics(vehicle):
    """Give the angular front and rear lamp panels genuine depth and optical modules."""
    led_white=material('LED_White',(.72,.86,1.0),.08,.18)
    led_red=material('LED_Red',(.72,.018,.012),.05,.2)
    for mat,color in ((led_white,(.55,.74,1.0,1)),(led_red,(.8,.008,.004,1))):
        bsdf=mat.node_tree.nodes.get('Principled BSDF') if mat.use_nodes else None
        if bsdf and 'Emission Color' in bsdf.inputs:
            bsdf.inputs['Emission Color'].default_value=color
            bsdf.inputs['Emission Strength'].default_value=.35
    alloy=material('Lamp_Reflector',(.72,.75,.78),.78,.2)
    def ellipsoid(name,loc,scale,mat):
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,location=loc)
        ob=bpy.context.object; ob.name=name; ob.scale=scale
        bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
        ob.data.materials.append(mat)
        for p in ob.data.polygons: p.use_smooth=True
        for c in list(ob.users_collection): c.objects.unlink(ob)
        vehicle.objects.link(ob)
        return ob
    for side in (-1,1):
        ellipsoid(f'Headlamp_Projector_{side}',(-2.163,side*.475,.752),(.020,.050,.024),alloy)
        ellipsoid(f'Headlamp_Outer_Optic_{side}',(-2.165,side*.565,.752),(.012,.022,.012),led_white)
        ellipsoid(f'TailLamp_3D_Lens_{side}',(2.154,side*.575,.795),(.020,.15,.040),led_red)
        # Bevelled 3-D light guides conform to the existing angular headlamp aperture.
        coords=[(.36,.739),(.39,.775),(.44,.783),(.555,.776),(.598,.755),(.575,.735)]
        data=bpy.data.curves.new(f'Headlamp_DRL_{side}_Curve','CURVE'); data.dimensions='3D'
        data.bevel_depth=.0045; data.bevel_resolution=3; data.resolution_u=2
        spline=data.splines.new('POLY'); spline.points.add(len(coords)-1)
        for p,(y,z) in zip(spline.points,coords): p.co=(-2.173,side*y,z,1)
        drl=bpy.data.objects.new(f'Headlamp_DRL_{side}',data); vehicle.objects.link(drl); data.materials.append(led_white)
        # Two slim red optical bars sit behind the raised rear lens volume.
        for k,z in enumerate((.781,.803)):
            ellipsoid(f'TailLamp_Bar_{side}_{k}',(2.173,side*.575,z),(.009,.105,.0045),led_red)


def build_bumper_assemblies(vehicle):
    """Keep bumper-mounted optics, grille, plate and diffuser attached to their fascia."""
    bpy.context.view_layer.update()
    groups={
        'Front_Bumper':('Front_Grille','Front_Grille_Honeycomb','Grille_Slat_','Front_Badge','Headlamp_Bezel_',
                        'Headlamp_Lens_','Headlamp_Projector_','Headlamp_Outer_Optic_','Headlamp_DRL_','Front_Splitter'),
        'Rear_Bumper':('TailLamp_','Rear_Plate','Plate_Glyph_','Exhaust_Tip_','Rear_Diffuser','Diffuser_Fin_'),
    }
    for bumper_name,prefixes in groups.items():
        bumper=bpy.data.objects[bumper_name]
        bumper['assembly']='Independent bumper cover with fascia-mounted components'
        for obj in list(vehicle.objects):
            if obj==bumper or obj.parent: continue
            if any(obj.name.startswith(prefix) for prefix in prefixes):
                obj.parent=bumper
                obj.matrix_parent_inverse=bumper.matrix_world.inverted()
    # Side markers and fender garnish belong to the removable front fender skins.
    for side,letter in ((1,'L'),(-1,'R')):
        fender=bpy.data.objects[f'Front_Fender_{letter}']
        for obj in list(vehicle.objects):
            if obj.parent is None and any(obj.name.startswith(prefix) for prefix in (
                f'Fender_Garnish_Base_{side}', f'Fender_Garnish_Rib_{side}_',
                f'Side_Marker_Bezel_{side}', f'Side_Marker_Lens_{side}', f'Front_Fender_Vent_{side}')):
                obj.parent=fender; obj.matrix_parent_inverse=fender.matrix_world.inverted()
    # The left-rear fuel flap remains attached to its independently removable quarter panel.
    quarter=bpy.data.objects['Rear_Quarter_L']
    for name in ('Fuel_Filler_Recess','Fuel_Filler_Door','Fuel_Filler_Hinge','Fuel_Filler_Notch'):
        obj=bpy.data.objects.get(name)
        if obj:
            obj.parent=quarter; obj.matrix_parent_inverse=quarter.matrix_world.inverted()
    gap_mat=material('Bumper_Panel_Gap',(.008,.012,.016),.04,.58)
    seam_paths={
        'Front_Bumper_Gap':('Front_Bumper',[(-1.824,.79,.48),(-1.824,.80,.64),(-1.824,.72,.79),
                                              (-1.824,.42,.84),(-1.824,0,.855),(-1.824,-.42,.84),
                                              (-1.824,-.72,.79),(-1.824,-.80,.64),(-1.824,-.79,.48)]),
        'Rear_Bumper_Gap':('Rear_Bumper',[(1.824,.78,.46),(1.824,.80,.61),(1.824,.72,.75),
                                         (1.824,.40,.79),(1.824,0,.80),(1.824,-.40,.79),
                                         (1.824,-.72,.75),(1.824,-.80,.61),(1.824,-.78,.46)]),
    }
    for name,(parent_name,coords) in seam_paths.items():
        data=bpy.data.curves.new(name+'_Curve','CURVE'); data.dimensions='3D'
        data.bevel_depth=.0022; data.bevel_resolution=2; data.resolution_u=1
        spline=data.splines.new('POLY'); spline.points.add(len(coords)-1)
        for point,co in zip(spline.points,coords): point.co=(*co,1)
        data.materials.append(gap_mat)
        seam=bpy.data.objects.new(name,data); vehicle.objects.link(seam)
        parent=bpy.data.objects[parent_name]; seam.parent=parent
        seam.matrix_parent_inverse=parent.matrix_world.inverted()
    bpy.context.view_layer.update()


def main():
    if not os.path.isfile(OBJ_PATH): raise FileNotFoundError(OBJ_PATH)
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    scene=bpy.context.scene
    scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1.0
    # OBJ coordinates already have X length and Z up; Y is the importer's forward axis
    # that gives the identity conversion for this authored coordinate frame.
    before=set(scene.objects)
    if hasattr(bpy.ops.wm,'obj_import'):
        bpy.ops.wm.obj_import(filepath=OBJ_PATH,forward_axis='Y',up_axis='Z',use_split_objects=True)
    else:
        bpy.ops.import_scene.obj(filepath=OBJ_PATH,axis_forward='Y',axis_up='Z')
    imported=[obj for obj in scene.objects if obj not in before]
    if not imported: raise RuntimeError('OBJ import returned no objects')
    tune_materials()

    vehicle=bpy.data.collections.new('GR86_Model'); scene.collection.children.link(vehicle)
    for obj in imported:
        for coll in list(obj.users_collection): coll.objects.unlink(obj)
        vehicle.objects.link(obj)
        if obj.type!='MESH': continue
        if obj.name.startswith('Body_Blockout'):
            for poly in obj.data.polygons: poly.use_smooth=True
            obj.data.validate(verbose=False)
        elif obj.name.startswith(('Tire_','Rim_','Brake_','Hub_','Headlamp','TailLamp','Mirror_')):
            for poly in obj.data.polygons: poly.use_smooth=True
        # Helpful pivots on separate assets before wheel-assembly parenting.
        bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True); bpy.context.view_layer.objects.active=obj
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY',center='BOUNDS')
        if obj.name.startswith(('Side_Skirt_','Front_Splitter','Rear_Diffuser','Trunk_Lip','Front_Fender_Vent_','Door_Handle_','Rear_Plate')):
            bevel=obj.modifiers.new('Soft manufactured edges','BEVEL')
            bevel.width=.003; bevel.segments=2
        if obj.name.startswith(('Headlamp_Bezel_','TailLamp_Bezel_','TailLamp_')):
            solid=obj.modifiers.new('Lamp housing depth','SOLIDIFY'); solid.thickness=.010; solid.offset=-.35
            bevel=obj.modifiers.new('Lens perimeter radius','BEVEL'); bevel.width=.004; bevel.segments=3
        elif obj.name.startswith('Headlamp_Lens_'):
            solid=obj.modifiers.new('Optical lens thickness','SOLIDIFY'); solid.thickness=.006; solid.offset=-.4
            bevel=obj.modifiers.new('Lens edge radius','BEVEL'); bevel.width=.0025; bevel.segments=3

    build_parametric_shell()
    build_interior(vehicle)
    build_engine_bay(vehicle)
    build_crash_structure(vehicle)
    build_lamp_optics(vehicle)
    build_bumper_assemblies(vehicle)
    imported=[obj for obj in vehicle.objects]
    rig=bpy.data.collections.new('GR86_Rig'); scene.collection.children.link(rig)
    # Independent transform roots let animators swing panels about plausible hinge axes.
    for panel,root_name,loc in (
        ('Hood_Panel','Hinge_Hood',(-.96,0,.98)),
        ('Trunk_Lid','Hinge_Trunk',(1.02,0,.94)),
        ('Door_L','Hinge_Door_L',(-.75,.82,.76)),
        ('Door_R','Hinge_Door_R',(-.75,-.82,.76)),
    ):
        root=bpy.data.objects.new(root_name,None); rig.objects.link(root)
        root.empty_display_type='CIRCLE'; root.empty_display_size=.10; root.location=loc
        bpy.context.view_layer.update()
        panel_obj=bpy.data.objects.get(panel)
        if panel_obj:
            panel_obj.parent=root
            panel_obj.matrix_parent_inverse=root.matrix_world.inverted()
            if panel.startswith('Door_'):
                side='1' if panel.endswith('_L') else '-1'
                for child_name in (f'Door_Card_{side}',f'Door_Armrest_{side}',f'Door_Release_{side}',f'Door_Speaker_{side}'):
                    child=bpy.data.objects.get(child_name)
                    if child:
                        child.parent=root; child.matrix_parent_inverse=root.matrix_world.inverted()
            bpy.context.view_layer.update()
    wheel_roots={}
    for axle,x in [('Front',-1.2875),('Rear',1.2875)]:
        for side in (-1,1):
            root=bpy.data.objects.new(f'WheelRoot_{axle}_{"L" if side>0 else "R"}',None)
            rig.objects.link(root); root.empty_display_type='CIRCLE'; root.empty_display_size=.16
            root.location=(x,side*.715,.337); wheel_roots[(axle,side)]=root
    bpy.context.view_layer.update()
    pattern=re.compile(r'^(?:Tire|Rim|Brake|Hub|Spoke)_(Front|Rear)_(-?1)(?:_|$)')
    for obj in imported:
        match=pattern.match(obj.name)
        if match and obj.type=='MESH':
            axle,side=match.group(1),int(match.group(2)); world=obj.matrix_world.copy()
            root=wheel_roots[(axle,side)]
            obj.parent=root
            obj.matrix_parent_inverse=root.matrix_world.inverted()
            obj.matrix_world=world
    # Front wheel assemblies steer around vertical Z; rear assemblies remain fixed.
    for (axle,side),root in wheel_roots.items():
        root['role']='steering wheel assembly' if axle=='Front' else 'rear wheel assembly'
        root['side']='left' if side>0 else 'right'

    studio=bpy.data.collections.new('Studio_Setup'); scene.collection.children.link(studio)
    floor_mat=material('Studio_Floor',(.055,.075,.105),.05,.52)
    bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.012))
    floor=bpy.context.object; floor.name='Studio_Floor'; floor.data.materials.append(floor_mat)
    for coll in list(floor.users_collection): coll.objects.unlink(floor)
    studio.objects.link(floor)
    world=bpy.data.worlds.get('GR86_Studio_World') or bpy.data.worlds.new('GR86_Studio_World')
    scene.world=world; world.use_nodes=True
    world.node_tree.nodes['Background'].inputs['Color'].default_value=(.22,.27,.35,1)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value=.35
    target=(0,0,.62)
    make_area_light('Key_Softbox',(-4,-4,7),1200,5,(1,.91,.80),target,studio)
    make_area_light('Fill_Softbox',(2,-5,3.8),850,4,(.73,.83,1),target,studio)
    make_area_light('Rim_Softbox',(1,4,5.5),1500,3.5,(1,1,1),target,studio)
    make_area_light('Front_Strip',(-6,1,2.6),500,3,(.8,.9,1),target,studio)
    make_area_light('Cockpit_Fill',(-.48,0,1.20),45,1.0,(.78,.86,1),(.24,0,.78),studio)
    cam_data=bpy.data.cameras.new('Cam_Exterior'); cam=bpy.data.objects.new('Cam_Exterior',cam_data); studio.objects.link(cam)
    cam_data.lens=64; cam.location=(-6.5,-7.2,3.2); aim(cam,target); scene.camera=cam
    for name,loc,tgt in [('Cam_Side',(-.1,-8.9,2.5),target),('Cam_Rear',(6.5,7.2,3.1),target)]:
        cd=bpy.data.cameras.new(name); ob=bpy.data.objects.new(name,cd); studio.objects.link(ob); ob.location=loc; aim(ob,tgt); cd.lens=62
    cockpit_data=bpy.data.cameras.new('Cam_Cockpit')
    cockpit=bpy.data.objects.new('Cam_Cockpit',cockpit_data); studio.objects.link(cockpit)
    cockpit.location=(.34,-.28,1.12); aim(cockpit,(-1.2,-.28,.96)); cockpit_data.lens=24
    scene.render.engine='BLENDER_EEVEE'
    scene.render.resolution_x=1600; scene.render.resolution_y=1100; scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'; scene.render.image_settings.color_mode='RGBA'
    try: scene.view_settings.view_transform='AgX'
    except Exception: pass
    # Add an editable Bevel modifier for real Blender rendering (WASM runtime cannot render).
    body=next((o for o in imported if o.name.startswith('Body_Shell') and o.type=='MESH'),None)
    if body:
        bevel=body.modifiers.new('Micro edge highlights','BEVEL'); bevel.width=.006; bevel.segments=2
        body['note']='Dense parametric shell; refine curves against GR86 orthographic refs.'
    scene['asset']='Toyota GR86 second generation procedural draft'
    scene['note']='Built in Blender WebAssembly; open in desktop Blender 4.x for Cycles rendering. Geometry still needs reference-based refinement.'
    os.makedirs(os.path.dirname(OUT_PATH),exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=OUT_PATH)
    print('BLENDER_VERSION',bpy.app.version_string)
    print('IMPORTED_OBJECTS',len(imported))
    print('BODY_BOUNDS',tuple(round(x,4) for x in body.dimensions) if body else None)
    print('SAVED',OUT_PATH)

if __name__=='__main__': main()
