#!/usr/bin/env node
/** Render four GR86 camera views with the Cycles-enabled threaded Blender WASM build.
 *
 * First run installs npm runtime/browser packages under .cache/gr86-cycles (ignored).
 * Requires a headless Chromium-compatible host. Output PNGs are lossless conversions
 * of Cycles-rendered 24-bit BMP frames; the Cycles WASM build's PNG writer emits
 * invalid tEXt metadata, so BMP is used as the intermediate.
 */
import fs from 'node:fs';
import path from 'node:path';
import http from 'node:http';
import zlib from 'node:zlib';
import { spawnSync } from 'node:child_process';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const ROOT=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const WORK=path.join(ROOT,'.cache','gr86-cycles');
const PUBLIC=path.join(WORK,'public');
const RUNTIME=path.join(PUBLIC,'blender-runtime');
const OUT=path.join(ROOT,'assets','renders','cycles');
const PORT=4179;
const SAMPLES=Number(process.env.GR86_CYCLES_SAMPLES||64);
const WIDTH=Number(process.env.GR86_CYCLES_WIDTH||800);
const HEIGHT=Number(process.env.GR86_CYCLES_HEIGHT||550);
const ALL_SHOTS=[['front_3_4','Cam_Exterior'],['side','Cam_Side'],['rear_3_4','Cam_Rear'],['cockpit','Cam_Cockpit'],['panels_open','Cam_Exterior']];
const requested=process.env.GR86_CYCLES_SHOT?.split(',').map(x=>x.trim()).filter(Boolean);
const SHOTS=requested?ALL_SHOTS.filter(([slug])=>requested.includes(slug)):ALL_SHOTS.filter(([slug])=>slug!=='panels_open');
if(!SHOTS.length||requested&&SHOTS.length!==requested.length)throw new Error(`Unknown GR86_CYCLES_SHOT: ${requested}`);
const pyShots=SHOTS.map(([slug,cam])=>`('${slug}','${cam}')`).join(',');
const pkgs=['@knervous/blender-web@0.0.5','@sparticuz/chromium@153.0.0','playwright-core@1.65.1'];
const requireFromWork=createRequire(path.join(WORK,'package.json'));

function ensurePackages(){
  let ready=true;
  for(const name of ['@knervous/blender-web','@sparticuz/chromium','playwright-core']){
    try{requireFromWork.resolve(name);}catch{ready=false;}
  }
  if(!ready){
    fs.mkdirSync(WORK,{recursive:true});
    const r=spawnSync('npm',['install','--prefix',WORK,'--no-save',...pkgs],{cwd:ROOT,stdio:'inherit'});
    if(r.status!==0)throw new Error('Failed installing Cycles browser runtime packages');
  }
}
function ensureRuntime(){
  if(fs.existsSync(path.join(RUNTIME,'manifest.json')))return;
  fs.mkdirSync(PUBLIC,{recursive:true});
  const cli=requireFromWork.resolve('@knervous/blender-web/package.json').replace(/package\.json$/,'bin/blender-web.js');
  const r=spawnSync(process.execPath,[cli,'install',RUNTIME],{cwd:ROOT,stdio:'inherit'});
  if(r.status!==0)throw new Error('Failed installing Blender WASM runtime');
}
function crc32(buf){
  let c=0xffffffff;
  for(const byte of buf){c^=byte;for(let k=0;k<8;k++)c=(c>>>1)^((c&1)?0xedb88320:0);}
  return (c^0xffffffff)>>>0;
}
function pngChunk(type,data){
  const name=Buffer.from(type);const len=Buffer.alloc(4);len.writeUInt32BE(data.length);
  const crc=Buffer.alloc(4);crc.writeUInt32BE(crc32(Buffer.concat([name,data])));
  return Buffer.concat([len,name,data,crc]);
}
function bmpToPng(bmp){
  if(bmp.toString('ascii',0,2)!=='BM')throw new Error('Cycles output is not a BMP');
  const offset=bmp.readUInt32LE(10),width=bmp.readInt32LE(18),signedHeight=bmp.readInt32LE(22);
  const bpp=bmp.readUInt16LE(28),compression=bmp.readUInt32LE(30);
  if(!width||!signedHeight||![24,32].includes(bpp)||compression!==0)throw new Error(`Unsupported BMP: ${width}x${signedHeight}, ${bpp}bpp, compression=${compression}`);
  const height=Math.abs(signedHeight),channels=bpp/8,rowSize=Math.ceil(width*channels/4)*4;
  const scanlines=Buffer.alloc((width*3+1)*height);
  for(let y=0;y<height;y++){
    const sourceY=signedHeight>0?height-1-y:y;
    const src=offset+sourceY*rowSize,dst=y*(width*3+1);
    scanlines[dst]=0;
    for(let x=0;x<width;x++){
      const si=src+x*channels,di=dst+1+x*3;
      scanlines[di]=bmp[si+2];scanlines[di+1]=bmp[si+1];scanlines[di+2]=bmp[si];
    }
  }
  const ihdr=Buffer.alloc(13);ihdr.writeUInt32BE(width,0);ihdr.writeUInt32BE(height,4);
  ihdr[8]=8;ihdr[9]=2;ihdr[10]=0;ihdr[11]=0;ihdr[12]=0;
  return Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]),pngChunk('IHDR',ihdr),pngChunk('IDAT',zlib.deflateSync(scanlines,{level:6})),pngChunk('IEND',Buffer.alloc(0))]);
}
function writeInputs(){
  fs.mkdirSync(path.join(PUBLIC,'in','assets'),{recursive:true});
  fs.mkdirSync(path.join(PUBLIC,'in','scripts'),{recursive:true});
  for(const file of ['gr86_blockout.obj','gr86_blockout.mtl'])fs.copyFileSync(path.join(ROOT,'assets',file),path.join(PUBLIC,'in','assets',file));
  fs.copyFileSync(path.join(ROOT,'scripts','import_gr86_in_blender.py'),path.join(PUBLIC,'in','scripts','import_gr86_in_blender.py'));
  fs.writeFileSync(path.join(PUBLIC,'index.html'),`<!doctype html><meta charset="utf-8"><title>GR86 Cycles render</title><pre id="log">starting</pre><script type="module" src="/public/client.js"></script>`);
  const client=`import {createBlenderRunner} from '/node_modules/@knervous/blender-web/dist/browser/index.js';
const log=s=>{document.querySelector('#log').textContent+='\\n'+s;console.log(s)};
window.renderReady=true;
window.render=async()=>{
 const get=async p=>new Uint8Array(await(await fetch(p)).arrayBuffer());
 const files={'/in/assets/gr86_blockout.obj':await get('/public/in/assets/gr86_blockout.obj'),'/in/assets/gr86_blockout.mtl':await get('/public/in/assets/gr86_blockout.mtl'),'/in/scripts/import_gr86_in_blender.py':await get('/public/in/scripts/import_gr86_in_blender.py')};
 const runner=createBlenderRunner({threads:true,runtimeUrl:'/public/blender-runtime/'});
 const code=${JSON.stringify(`${IMPORT_SCRIPT}\n${RENDER_SETUP}\n${RENDER_LOOP}`)};
 const result=await runner.run({code,files,onLine:log});
 window.frames=Object.fromEntries(Object.entries(result.files).filter(([k])=>k.endsWith('.bmp')));
 log('CYCLES_RENDER_COMPLETE '+Object.keys(window.frames).join(','));
};`;
  fs.writeFileSync(path.join(PUBLIC,'client.js'),client);
}
const IMPORT_SCRIPT="exec(compile(open('/in/scripts/import_gr86_in_blender.py').read(), 'import_gr86_in_blender.py', 'exec'), {'__name__':'__main__'})";
const RENDER_SETUP=`import bpy, os
s=bpy.context.scene
if not bpy.app.build_options.cycles: raise RuntimeError('Cycles support absent')
s.render.engine='CYCLES'; s.cycles.samples=${SAMPLES}; s.cycles.use_denoising=True
s.render.resolution_x=${WIDTH}; s.render.resolution_y=${HEIGHT}; s.render.resolution_percentage=100
s.render.image_settings.file_format='BMP'; s.render.image_settings.color_mode='RGB'
os.makedirs('/out/cycles',exist_ok=True)`;
const RENDER_LOOP=`import math
for slug,cam in [${pyShots}]:
 if slug=='panels_open':
  bpy.data.objects['Hinge_Hood'].rotation_euler[1]=math.radians(38)
  bpy.data.objects['Hinge_Trunk'].rotation_euler[1]=math.radians(-34)
  bpy.data.objects['Hinge_Door_L'].rotation_euler[2]=math.radians(48)
  bpy.data.objects['Hinge_Door_R'].rotation_euler[2]=math.radians(-48)
  bpy.context.view_layer.update()
 s.camera=bpy.data.objects[cam]; s.render.filepath='/out/cycles/gr86_'+slug+'.bmp'
 bpy.ops.render.render(write_still=True); print('CYCLES_FRAME',slug)`;
function serve(){
 const root=WORK;
 const mime={'.html':'text/html','.js':'application/javascript','.json':'application/json','.wasm':'application/wasm','.br':'application/octet-stream','.obj':'text/plain','.mtl':'text/plain','.py':'text/plain'};
 return http.createServer((req,res)=>{
   res.setHeader('Cross-Origin-Opener-Policy','same-origin');res.setHeader('Cross-Origin-Embedder-Policy','require-corp');res.setHeader('Cross-Origin-Resource-Policy','same-origin');
   let url=decodeURIComponent(new URL(req.url,'http://localhost').pathname);if(url==='/')url='/public/index.html';
   const file=path.resolve(root,'.'+url);if(!file.startsWith(root)){res.writeHead(403);return res.end();}
   fs.readFile(file,(err,data)=>{if(err){res.writeHead(404);return res.end('not found');}res.setHeader('Content-Type',mime[path.extname(file)]||'application/octet-stream');res.end(data);});
 });
}
async function main(){
  ensurePackages();ensureRuntime();writeInputs();fs.mkdirSync(OUT,{recursive:true});
  const chromium=requireFromWork('@sparticuz/chromium').default;
  const {inflate,setupLambdaEnvironment}=requireFromWork('@sparticuz/chromium');
  const {chromium:playwright}=requireFromWork('playwright-core');
  const al=path.join(WORK,'node_modules/@sparticuz/chromium/bin/al2023.tar.br');
  await inflate(al);setupLambdaEnvironment('/tmp/al2023/lib');
  const exe=await chromium.executablePath();
  const args=chromium.args.filter(a=>!['--disable-web-security','--disable-site-isolation-trials','--single-process'].includes(a));
  args.push('--no-sandbox','--enable-features=SharedArrayBufferOnDesktop');
  const server=serve();await new Promise((resolve,reject)=>server.listen(PORT,'127.0.0.1',resolve).once('error',reject));
  let browser;
  try{
    browser=await playwright.launch({headless:true,executablePath:exe,args});
    const page=await browser.newPage();page.on('console',m=>{if(m.type()==='error')console.error('BROWSER',m.text());else if(m.text().startsWith('CYCLES_'))console.log('BROWSER',m.text());});page.on('pageerror',e=>console.error('PAGE',e.message));
    await page.goto(`http://127.0.0.1:${PORT}/`,{waitUntil:'load',timeout:120000});
    const isolated=await page.evaluate(()=>crossOriginIsolated&&typeof SharedArrayBuffer==='function');
    if(!isolated)throw new Error('Chromium page is not cross-origin isolated; threaded Cycles needs SharedArrayBuffer');
    await page.waitForFunction(()=>window.renderReady,{timeout:120000});await page.evaluate(()=>window.render());
    const encoded=await page.evaluate(()=>Object.fromEntries(Object.entries(window.frames).map(([k,a])=>{let s='';for(let i=0;i<a.length;i+=0x8000)s+=String.fromCharCode(...a.subarray(i,i+0x8000));return[k,btoa(s)]})));
    for(const [name,b64] of Object.entries(encoded)){
      const bmp=Buffer.from(b64,'base64');
      const dest=path.join(OUT,path.basename(name).replace(/\.bmp$/,'.png'));
      fs.writeFileSync(dest,bmpToPng(bmp));console.log('CYCLES_PNG',dest,fs.statSync(dest).size);
    }
  }finally{if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));}
}
main().catch(e=>{console.error(e.stack||e);process.exitCode=1;});
