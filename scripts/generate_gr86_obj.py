"""Create a dependency-free OBJ/MTL GR86-style blockout for Blender/UE import.

Usage: python scripts/generate_gr86_obj.py [output.obj]
Coordinates: X length (front is -X), Y width, Z up. Approximate blockout only.
"""
from __future__ import annotations
import math
import os
import sys

class Obj:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.materials = {}
        self.current = None
    def material(self, name, rgb, metallic=0.0, roughness=.4):
        self.materials[name] = (rgb, metallic, roughness)
    def use(self, name):
        self.current = name
    def mesh(self, name, verts, faces):
        start = len(self.vertices) + 1
        self.vertices.extend(verts)
        self.faces.append((name, self.current, [tuple(start+i for i in face) for face in faces]))
    def box(self, name, center, size):
        x,y,z=center; a,b,c=(v/2 for v in size)
        v=[(x-a,y-b,z-c),(x+a,y-b,z-c),(x+a,y+b,z-c),(x-a,y+b,z-c),
           (x-a,y-b,z+c),(x+a,y-b,z+c),(x+a,y+b,z+c),(x-a,y+b,z+c)]
        self.mesh(name,v,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
    def ellipsoid(self,name,center,radii,segments=20,rings=12):
        cx,cy,cz=center; rx,ry,rz=radii; verts=[]
        for j in range(rings+1):
            phi=math.pi*j/rings
            for i in range(segments):
                theta=2*math.pi*i/segments
                verts.append((cx+rx*math.sin(phi)*math.cos(theta),cy+ry*math.sin(phi)*math.sin(theta),cz+rz*math.cos(phi)))
        faces=[]
        for j in range(rings):
            for i in range(segments):
                a=j*segments+i; b=j*segments+(i+1)%segments
                faces.append((a,b,b+segments,a+segments))
        self.mesh(name,verts,faces)
    def arch_band(self,name,x,y,z,r_inner,r_outer,theta_start=.62,theta_end=math.pi-.62,steps=24):
        verts=[]
        for r in (r_inner,r_outer):
            for i in range(steps+1):
                a=theta_start+(theta_end-theta_start)*i/steps
                verts.append((x+r*math.cos(a), y, z+r*math.sin(a)))
        faces=[(i,i+1,steps+1+i+1,steps+1+i) for i in range(steps)]
        self.mesh(name,verts,faces)
    def cylinder_y(self,name,x,y,z,radius,width,segments=32):
        verts=[]
        for yy in (y-width/2,y+width/2):
            verts += [(x+radius*math.cos(2*math.pi*i/segments),yy,z+radius*math.sin(2*math.pi*i/segments)) for i in range(segments)]
        faces=[]
        for i in range(segments):
            j=(i+1)%segments; faces.append((i,j,segments+j,segments+i))
        faces += [tuple(reversed(range(segments))),tuple(range(segments,2*segments))]
        self.mesh(name,verts,faces)
    def cylinder_x(self,name,x,y,z,radius,length,segments=24):
        verts=[]
        for xx in (x-length/2,x+length/2):
            verts += [(xx,y+radius*math.cos(2*math.pi*i/segments),z+radius*math.sin(2*math.pi*i/segments)) for i in range(segments)]
        faces=[]
        for i in range(segments):
            j=(i+1)%segments; faces.append((i,j,segments+j,segments+i))
        faces += [tuple(reversed(range(segments))),tuple(range(segments,2*segments))]
        self.mesh(name,verts,faces)
    def torus_y(self,name,x,y,z,major,minor_radial,minor_axial=None,width_segments=48,radial_segments=12):
        if minor_axial is None: minor_axial=minor_radial
        verts=[]
        for i in range(width_segments):
            a=2*math.pi*i/width_segments
            for j in range(radial_segments):
                b=2*math.pi*j/radial_segments
                rr=major+minor_radial*math.cos(b)
                verts.append((x+rr*math.cos(a), y+(minor_axial*math.sin(b)), z+rr*math.sin(a)))
        faces=[]
        for i in range(width_segments):
            for j in range(radial_segments):
                a=i*radial_segments+j; b=i*radial_segments+(j+1)%radial_segments
                c=((i+1)%width_segments)*radial_segments+(j+1)%radial_segments
                d=((i+1)%width_segments)*radial_segments+j
                faces.append((a,b,c,d))
        self.mesh(name,verts,faces)
    def loft(self,name,sections,profile):
        if name=='Body_Blockout':
            smooth=[]
            for i in range(len(sections)-1):
                p0=sections[max(0,i-1)]; p1=sections[i]; p2=sections[i+1]; p3=sections[min(len(sections)-1,i+2)]
                for step in range(10):
                    t=step/10
                    row=[]
                    for k in range(3):
                        row.append(.5*((2*p1[k])+(-p0[k]+p2[k])*t+(2*p0[k]-5*p1[k]+4*p2[k]-p3[k])*t*t+(-p0[k]+3*p1[k]-3*p2[k]+p3[k])*t*t*t))
                    smooth.append(tuple(row))
            smooth.append(sections[-1]); sections=smooth
        verts=[]
        for x,w,dz in sections:
            for yf,z in profile:
                t=max(0.0,min(1.0,(z-.43)/.52))**2
                zz=z+dz*t
                # A restrained OEM shoulder crown adds the GR86's taut side surfacing
                # without turning the body into a widebody or disturbing panel splits.
                side_crown=0.0
                if abs(yf)>.78 and .58<=z<=.84:
                    shoulder=math.exp(-((z-.755)/.075)**2)
                    longitudinal=0.82+0.18*math.cos(min(1.0,abs(x)/2.2)*math.pi/2)
                    side_crown=0.014*shoulder*longitudinal
                # Very shallow paired hood character lines, confined to the bonnet
                # and fading before the cowl and nose (OEM, not raised aftermarket vents).
                hood_line=0.0
                if -1.92<x<-.78 and z>.90 and abs(yf)<.72:
                    across=math.exp(-((abs(yf)-.57)/.055)**2)
                    along=math.sin(math.pi*(x+1.92)/1.14)**0.5
                    hood_line=.007*across*along
                zz += hood_line
                # Raise the outer lower body edge over each tyre to form open wheel arches.
                if abs(yf)>.92 and z<=.51:
                    for wheel_x in (-1.2875,1.2875):
                        dx=abs(x-wheel_x); arch_r=.385
                        if dx<arch_r:
                            arch_z=.337+math.sqrt(max(0,arch_r*arch_r-dx*dx))
                            zz=max(zz,arch_z)
                verts.append((x,yf*(w+side_crown),zz))
        n=len(profile); faces=[]
        for i in range(len(sections)-1):
            for j in range(n): faces.append((i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j))
        faces.extend([tuple(reversed(range(n))),tuple((len(sections)-1)*n+j for j in range(n))])
        self.mesh(name,verts,faces)
    def write(self,path):
        mtl=os.path.splitext(path)[0]+'.mtl'
        with open(mtl,'w') as f:
            for name,(rgb,metal,rough) in self.materials.items():
                f.write(f'newmtl {name}\nKd {rgb[0]} {rgb[1]} {rgb[2]}\nKa 0.02 0.02 0.02\nKs {metal:.3f} {metal:.3f} {metal:.3f}\nNs {max(2, int((1-rough)*1000))}\n\n')
        with open(path,'w') as f:
            f.write('# Approximate GR86-style blockout; metres; front is -X; Z up\nmtllib '+os.path.basename(mtl)+'\n')
            for v in self.vertices: f.write('v %.6f %.6f %.6f\n'%v)
            for name,mat,faces in self.faces:
                f.write('o '+name+'\nusemtl '+mat+'\n')
                for face in faces: f.write('f '+' '.join(map(str,face))+'\n')

def generate(path):
    o=Obj()
    o.material('Pearl_White',(.82,.84,.87),.55,.24); o.material('Glass_Dark',(.055,.105,.135),.42,.17)
    o.material('Tire_Rubber',(.018,.02,.023),0,.84); o.material('Graphite_Alloy',(.68,.70,.73),.82,.24)
    o.material('Brake_Metal',(.24,.24,.23),.65,.38); o.material('Lamp_White',(.94,.97,1.0),.1,.15)
    o.material('Lamp_Red',(.6,.015,.012),.1,.2)
    o.material('Grille_Mesh',(.075,.09,.105),.72,.36); o.material('Marker_Amber',(.92,.24,.018),.18,.26)
    sections=[(-2.132,.62,-.11),(-1.88,.76,-.04),(-1.42,.85,.015),(-.90,.88,.035),(-.38,.82,0),(.30,.82,0),(.88,.87,-.015),(1.45,.86,.005),(1.90,.77,-.015),(2.132,.62,-.075)]
    profile=[(-.96,.43),(-.995,.50),(-1.0,.67),(-.975,.77),(-.93,.87),(-.84,.915),(-.63,.94),(0,.95),(.63,.94),(.84,.915),(.93,.87),(.975,.77),(1.0,.67),(.995,.50),(.96,.43),(.55,.43),(0,.43),(-.55,.43)]
    o.use('Pearl_White'); o.loft('Body_Blockout',sections,profile)
    # Greenhouse trapezoid loft.
    cabin=[(-.74,.65,.91),(-.22,.61,1.285),(.49,.60,1.285),(1.06,.67,.91)]
    prof=[(-1,.91),(1,.91),(1,1.0),(-1,1.0)]
    verts=[]
    for x,w,z in cabin: verts.extend([(x,-w,.91),(x,w,.91),(x,w*.78,z),(x,-w*.78,z)])
    faces=[]
    for i in range(3):
        for j in range(4): faces.append((i*4+j,i*4+(j+1)%4,(i+1)*4+(j+1)%4,(i+1)*4+j))
    faces.extend([(0,1,2,3),(12,15,14,13)])
    o.use('Glass_Dark'); o.mesh('Cabin_Glazing_Blockout',verts,faces)
    o.use('Pearl_White'); o.box('Roof_Panel',(0.16,0,1.29),(.60,.92,.025))
    for side in (-1,1):
        o.use('Glass_Dark')
        o.mesh('Side_Glass_'+str(side),[(-.55,side*.65,.945),(.83,side*.65,.945),(.56,side*.65,1.205),(-.20,side*.65,1.205)],[(0,1,2,3)])
        o.mesh('Front_Windshield_'+str(side),[(-.70,side*.59,.95),(-.22,side*.48,1.255),(-.22,0,1.255),(-.70,0,.95)],[(0,1,2,3)])
        o.mesh('Rear_Glass_'+str(side),[(.50,side*.48,1.255),(1.00,side*.62,.95),(1.00,0,.95),(.50,0,1.255)],[(0,1,2,3)])
        o.use('Pearl_White')
        # Slim painted pillars frame the side glass rather than leaving a solid dark cabin.
        o.mesh('A_Pillar_'+str(side),[(-.70,side*.66,.94),(-.63,side*.66,.94),(-.18,side*.50,1.255),(-.24,side*.47,1.255)],[(0,1,2,3)])
        o.mesh('B_Pillar_'+str(side),[(.10,side*.66,.94),(.16,side*.66,.94),(.16,side*.61,1.205),(.10,side*.61,1.205)],[(0,1,2,3)])
        o.mesh('C_Pillar_'+str(side),[(.80,side*.65,.94),(.87,side*.65,.94),(.58,side*.51,1.205),(.52,side*.49,1.205)],[(0,1,2,3)])
        # Compact GR86-style mirror with a short mount, painted shell, and dark glass inset.
        o.use('Tire_Rubber'); o.box('Mirror_Stem_'+str(side),(-.625,side*.806,.925),(.105,.045,.035))
        o.use('Pearl_White'); o.ellipsoid('Mirror_'+str(side),(-.625,side*.868,.948),(.105,.060,.047),24,12)
        o.use('Glass_Dark'); o.ellipsoid('Mirror_Glass_'+str(side),(-.625,side*.919,.949),(.067,.012,.030),20,10)
        o.use('Graphite_Alloy'); o.box('Door_Handle_'+str(side),(.57,side*.858,.79),(.095,.004,.012))
        o.use('Tire_Rubber')
        # Fine panel gaps sit close to the body instead of reading as protruding black bars.
        o.box('Door_Gap_Front_'+str(side),(-.75,side*.857,.70),(.004,.004,.31))
        o.box('Door_Gap_Rear_'+str(side),(.75,side*.851,.70),(.004,.004,.29))
        o.box('Door_Gap_Bottom_'+str(side),(0,side*.855,.535),(1.40,.004,.004))
        o.use('Pearl_White')
        o.box('Side_Skirt_'+str(side),(.12,side*.855,.49),(1.42,.018,.05))
        # Small separate OEM fender garnish: dark inset, three machined ribs and amber repeater.
        o.use('Tire_Rubber'); o.box('Fender_Garnish_Base_'+str(side),(-1.035,side*.860,.735),(.180,.014,.062))
        o.use('Graphite_Alloy')
        for rib in range(3):
            o.box(f'Fender_Garnish_Rib_{side}_{rib}',(-1.035,side*.870,.716+rib*.018),(.142,.007,.005))
        o.use('Tire_Rubber'); o.box('Side_Marker_Bezel_'+str(side),(-1.035,side*.862,.682),(.132,.008,.028))
        o.use('Marker_Amber'); o.box('Side_Marker_Lens_'+str(side),(-1.035,side*.868,.682),(.108,.004,.016))
    # Broad, inset hexagonal GR86-style lower grille with a dark mesh backing.
    o.use('Tire_Rubber')
    grille=[(-.43,.69),(-.34,.755),(.34,.755),(.43,.69),(.37,.545),(-.37,.545)]
    gv=[(-2.153,y,z) for y,z in grille]+[(-2.126,y,z) for y,z in grille]
    gf=[tuple(range(6)),tuple(reversed(range(6,12)))]+[(i,(i+1)%6,(i+1)%6+6,i+6) for i in range(6)]
    o.mesh('Front_Grille',gv,gf)
    # OEM-style open honeycomb mesh replaces the blockout's horizontal grille bars.
    o.use('Grille_Mesh'); hv=[]; hf=[]
    for row in range(5):
        zc=.574+row*.036
        for col in range(15):
            yc=-.318+col*.045+(row%2)*.0225
            if abs(yc)>.345: continue
            base=len(hv); radius=.022; inner=.017
            for r in (radius,inner):
                for k in range(6):
                    a=math.pi/6+k*math.pi/3
                    hv.append((-2.164,yc+r*math.cos(a),zc+r*math.sin(a)))
            for k in range(6): hf.append((base+k,base+(k+1)%6,base+6+(k+1)%6,base+6+k))
    o.mesh('Front_Grille_Honeycomb',hv,hf)
    o.use('Brake_Metal'); o.ellipsoid('Front_Badge',(-2.167,0,.675),(.012,.036,.026),24,12)
    # Slim angular headlamp housings and cool-white lens panels, rather than ellipsoids.
    for side in (-1,1):
        o.use('Tire_Rubber')
        outer=[(.32,.745),(.37,.795),(.58,.785),(.63,.752),(.59,.720),(.35,.718)]
        vv=[(-2.145,side*y,z) for y,z in outer]
        o.mesh('Headlamp_Bezel_'+str(side),vv,[tuple(range(6))])
        o.use('Lamp_White')
        inner=[(.36,.745),(.40,.778),(.56,.770),(.595,.750),(.565,.730),(.39,.729)]
        vv=[(-2.155,side*y,z) for y,z in inner]
        o.mesh('Headlamp_Lens_'+str(side),vv,[tuple(range(6))])
    o.use('Graphite_Alloy'); o.box('Front_Splitter',(-2.08,0,.435),(.085,1.48,.018))
    o.use('Tire_Rubber'); o.box('Rear_Diffuser',(2.075,0,.385),(.10,1.22,.045))
    for idx,y in enumerate((-.38,0,.38)):
        o.box(f'Diffuser_Fin_{idx}',(2.04,y,.405),(.16,.018,.055))
    o.use('Pearl_White'); o.box('Trunk_Lip', (1.90,0,.912),(.075,.94,.018))
    o.use('Tire_Rubber'); o.box('Rear_Plate_Recess',(2.143,0,.67),(.012,.48,.12))
    o.use('Lamp_White'); o.box('Rear_Plate',(2.151,0,.67),(.008,.40,.082))
    o.use('Tire_Rubber')
    for idx,y in enumerate((-.13,-.07,0,.07,.13)):
        o.box(f'Plate_Glyph_{idx}',(2.156,y,.67),(.004,.025,.035))
    o.use('Brake_Metal')
    for side in (-1,1): o.cylinder_x('Exhaust_Tip_'+str(side),2.085,side*.57,.47,.062,.090,28)
    for side in (-1,1):
        # Angular red outer lenses with a slimmer inner illuminated strip.
        outer=[(.34,.815),(.40,.855),(.70,.835),(.79,.790),(.75,.746),(.43,.755)]
        o.use('Tire_Rubber'); o.mesh('TailLamp_Bezel_'+str(side),[(2.139,side*y,z) for y,z in outer],[tuple(range(6))])
        inner=[(.40,.813),(.46,.840),(.69,.824),(.745,.790),(.71,.762),(.46,.769)]
        o.use('Lamp_Red'); o.mesh('TailLamp_'+str(side),[(2.150,side*y,z) for y,z in inner],[tuple(range(6))])
    # Separate left-rear fuel-filler surround and painted door, as on the road car.
    o.use('Tire_Rubber'); o.box('Fuel_Filler_Recess', (1.48,.872,.735),(.168,.012,.142))
    o.use('Pearl_White'); o.box('Fuel_Filler_Door', (1.48,.881,.735),(.142,.010,.116))
    o.use('Graphite_Alloy'); o.box('Fuel_Filler_Hinge', (1.425,.887,.783),(.035,.006,.006))
    o.use('Tire_Rubber'); o.box('Fuel_Filler_Notch', (1.535,.888,.735),(.012,.006,.028))
    # Approximate four wheels; independent named groups and axle-aligned geometry.
    for axle,x in [('Front',-1.2875),('Rear',1.2875)]:
        for side in (-1,1):
            y=side*.715; z=.337
            o.use('Tire_Rubber'); o.torus_y(f'Tire_{axle}_{side}',x,y,z,.257,.080,.1125)
            o.use('Pearl_White'); o.arch_band(f'Fender_Arch_{axle}_{side}',x,side*.895,z,.390,.406)
            # Open graphite wheel barrel, ventilated rotor and ten slim motorsport spokes.
            o.use('Graphite_Alloy'); o.torus_y(f'Rim_{axle}_{side}',x,y+side*.087,z,.197,.018,.034,48,10)
            o.use('Brake_Metal'); o.cylinder_y(f'Brake_{axle}_{side}',x,y+side*.025,z,.162,.026,40)
            o.use('Graphite_Alloy'); o.cylinder_y(f'Hub_{axle}_{side}',x,y+side*.118,z,.038,.025,24)
            # Ten slender radial spokes give a light, open five-pair wheel face.
            for k in range(10):
                a=2*math.pi*k/10
                dx,dz=math.sin(a),math.cos(a)
                cx=x+dx*.112; cz=z+dz*.112
                px,pz=math.cos(a)*.009,-math.sin(a)*.009
                ya=y+side*.075; yb=y+side*.119
                v=[(cx+px,ya,cz+pz),(cx-px,ya,cz-pz),
                   (cx-px,yb,cz-pz),(cx+px,yb,cz+pz),
                   (x+dx*.190+px,ya,z+dz*.190+pz),(x+dx*.190-px,ya,z+dz*.190-pz),
                   (x+dx*.190-px,yb,z+dz*.190-pz),(x+dx*.190+px,yb,z+dz*.190+pz)]
                o.mesh(f'Spoke_{axle}_{side}_{k}',v,[(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)])
    os.makedirs(os.path.dirname(os.path.abspath(path)),exist_ok=True)
    o.write(path)
    return len(o.vertices),len(o.faces)

if __name__=='__main__':
    dest=sys.argv[1] if len(sys.argv)>1 else 'assets/gr86_blockout.obj'
    v,f=generate(dest)
    print(f'Wrote {dest}: {v} vertices, {f} object meshes')
