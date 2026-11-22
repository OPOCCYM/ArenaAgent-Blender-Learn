"""Render OBJ/MTL views to PNG with a small NumPy/Pillow software rasterizer.
Usage: .venv/bin/python scripts/render_obj_png.py
This preview renders the actual generated mesh; it is not a Blender/Cycles render.
"""
from pathlib import Path
import math
import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT=Path(__file__).resolve().parents[1]
OBJ=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/'assets/gr86_blockout.obj'
OUT=ROOT/'assets/renders'
W,H=1280,960


def read_mtl(path):
    mats={}; cur=None
    for line in path.read_text().splitlines():
        p=line.split()
        if not p: continue
        if p[0]=='newmtl': cur=p[1]; mats[cur]={'Kd':(.7,.7,.7),'Ks':(.1,.1,.1),'Ns':80}
        elif cur and p[0]=='Kd': mats[cur]['Kd']=tuple(map(float,p[1:4]))
        elif cur and p[0]=='Ks': mats[cur]['Ks']=tuple(map(float,p[1:4]))
        elif cur and p[0]=='Ns': mats[cur]['Ns']=float(p[1])
    return mats


def load_obj(path):
    verts=[]; mats=read_mtl(path.with_suffix('.mtl')); tris=[]; curmat='Pearl_White'; group=''
    for line in path.read_text().splitlines():
        p=line.split()
        if not p: continue
        if p[0]=='v': verts.append(tuple(map(float,p[1:4])))
        elif p[0]=='usemtl': curmat=p[1]
        elif p[0]=='o': group=p[1]
        elif p[0]=='f':
            ids=[int(x.split('/')[0])-1 for x in p[1:]]
            for i in range(1,len(ids)-1): tris.append((ids[0],ids[i],ids[i+1],curmat,group))
    return np.asarray(verts,dtype=float),tris,mats


def normalize(v): return v/np.linalg.norm(v)


def camera_project(points, camera, target, fov=35):
    forward=normalize(target-camera); right=normalize(np.cross(forward,np.array([0.,0.,1.])))
    up=normalize(np.cross(right,forward)); rel=points-camera
    xyz=np.stack([rel@right,rel@up,rel@forward],axis=1)
    focal=.5*H/math.tan(math.radians(fov/2))
    sx=W/2+xyz[:,0]*focal/xyz[:,2]; sy=H/2-xyz[:,1]*focal/xyz[:,2]
    return np.stack([sx,sy,xyz[:,2]],axis=1),forward


def background():
    yy=np.linspace(0,1,H)[:,None,None]
    top=np.array([35,45,62])[None,None,:]; bottom=np.array([190,201,212])[None,None,:]
    grad=(top*(1-yy)+bottom*yy).astype(np.uint8)
    return Image.fromarray(np.repeat(grad,W,axis=1),'RGB').convert('RGBA')


def draw_shadow(img, cx, cy, rx, ry):
    layer=Image.new('RGBA',(W,H),(0,0,0,0)); d=ImageDraw.Draw(layer)
    d.ellipse((cx-rx,cy-ry,cx+rx,cy+ry),fill=(16,20,28,125))
    layer=layer.filter(ImageFilter.GaussianBlur(28)); img.alpha_composite(layer)


def render_view(verts,tris,mats,name,camera,target,fov=35):
    smooth_normals=np.zeros_like(verts)
    for ia,ib,ic,matname,group in tris:
        if group in ('Body_Blockout','Body_Shell'):
            n=np.cross(verts[ib]-verts[ia],verts[ic]-verts[ia])
            smooth_normals[ia]+=n; smooth_normals[ib]+=n; smooth_normals[ic]+=n
    lengths=np.linalg.norm(smooth_normals,axis=1); valid=lengths>1e-10
    smooth_normals[valid]/=lengths[valid,None]
    proj,_=camera_project(verts,np.array(camera,dtype=float),np.array(target,dtype=float),fov=fov)
    img=background()
    if name!='Cockpit': draw_shadow(img,W*.51,H*.69,330,52)
    zbuf=np.zeros((H,W),dtype=np.float32); rgb=np.zeros((H,W,3),dtype=np.uint8)
    light=normalize(np.array([-.45,-.55,.95])); view=normalize(np.array(camera)-np.array(target))
    for ia,ib,ic,matname,group in tris:
        q=proj[[ia,ib,ic]]
        if np.any(q[:,2]<=.05): continue
        xy=q[:,:2]; x0=max(0,int(np.floor(xy[:,0].min()))); x1=min(W-1,int(np.ceil(xy[:,0].max())))
        y0=max(0,int(np.floor(xy[:,1].min()))); y1=min(H-1,int(np.ceil(xy[:,1].max())))
        if x1<=x0 or y1<=y0: continue
        v0,v1,v2=xy; den=(v1[1]-v2[1])*(v0[0]-v2[0])+(v2[0]-v1[0])*(v0[1]-v2[1])
        if abs(den)<1e-8: continue
        xs=np.arange(x0,x1+1,dtype=np.float32)[None,:]+.5; ys=np.arange(y0,y1+1,dtype=np.float32)[:,None]+.5
        a=((v1[1]-v2[1])*(xs-v2[0])+(v2[0]-v1[0])*(ys-v2[1]))/den
        b=((v2[1]-v0[1])*(xs-v2[0])+(v0[0]-v2[0])*(ys-v2[1]))/den
        c=1-a-b; inside=(a>=-1e-5)&(b>=-1e-5)&(c>=-1e-5)
        invz=a/q[0,2]+b/q[1,2]+c/q[2,2]
        depth=1/np.maximum(invz,1e-8)
        zs=zbuf[y0:y1+1,x0:x1+1]
        mask=inside&(invz>zs)
        if not mask.any(): continue
        p=verts[[ia,ib,ic]]; n=np.cross(p[1]-p[0],p[2]-p[0]); nl=np.linalg.norm(n)
        if nl<1e-10: continue
        n=n/nl
        m=mats.get(matname,{'Kd':(.7,.7,.7),'Ks':(.1,.1,.1),'Ns':80})
        base=np.array(m['Kd']); ks=np.mean(m['Ks'])
        if group in ('Body_Blockout','Body_Shell'):
            ns=smooth_normals[[ia,ib,ic]]
            nn=a[...,None]*ns[0]+b[...,None]*ns[1]+c[...,None]*ns[2]
            nn/=np.maximum(np.linalg.norm(nn,axis=2,keepdims=True),1e-8)
            flip=(nn@view)<0; nn[flip]*=-1
            nd=np.maximum(0,nn@light)
            hv=normalize(light+view); spec=np.maximum(0,nn@hv)**24
            shade=.34+.64*nd
            colors=np.clip(shade[...,None]*base+ks*spec[...,None]*1.35,0,1)
            patch=rgb[y0:y1+1,x0:x1+1]; patch[mask]=(colors[mask]*255).astype(np.uint8)
        else:
            if np.dot(n,view)<0: n=-n
            nd=max(0.,float(np.dot(n,light)))
            spec=max(0.,float(np.dot(n,normalize(light+view))))**20
            shade=.30+.68*nd
            color=np.clip(base*shade+ks*spec*1.5,0,1)
            patch=rgb[y0:y1+1,x0:x1+1]; patch[mask]=(color*255).astype(np.uint8)
        zs[mask]=invz[mask]
    model=Image.fromarray(rgb,'RGB').convert('RGBA')
    alpha=Image.fromarray((zbuf>0).astype(np.uint8)*255,'L')
    img.alpha_composite(Image.composite(model,Image.new('RGBA',(W,H)),alpha))
    # Editorial title/footer for a useful review sheet.
    draw=ImageDraw.Draw(img)
    draw.rounded_rectangle((44,38,390,105),radius=14,fill=(10,18,30,190))
    draw.text((66,57),'GR86  /  '+name,fill=(246,248,250,255),stroke_width=0)
    draw.text((48,H-44),'PROCEDURAL MESH PREVIEW  •  NOT A CYCLES RENDER',fill=(30,40,52,230))
    slug=name.lower().replace(' ','_').replace('/','_')
    out=OUT/('gr86_'+slug+'.png'); img.convert('RGB').save(out)
    print(out)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    verts,tris,mats=load_obj(OBJ)
    target=np.array([0.,0.,.63])
    render_view(verts,tris,mats,'Front 3/4',[-6.4,-7.0,3.2],target)
    render_view(verts,tris,mats,'Side',[-.1,-8.7,2.35],target)
    render_view(verts,tris,mats,'Rear 3/4',[6.5,7.0,3.1],target)
    cockpit_names=('Interior_','Dashboard','Center_Console','Rear_Seat_','Front_Seat_','Door_Card_','Door_Armrest_','Steering_')
    cockpit_tris=[t for t in tris if t[4].startswith(cockpit_names)]
    render_view(verts,cockpit_tris,mats,'Cockpit',[.34,-.28,1.12],[-1.2,-.28,.96],fov=68)

if __name__=='__main__': main()
