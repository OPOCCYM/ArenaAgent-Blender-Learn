"""Run a bpy script through the headless Blender WebAssembly runtime on npm.

Example from repo root:
  python scripts/run_blender_wasm.py scripts/import_gr86_in_blender.py \
    --input assets/gr86_blockout.obj --input assets/gr86_blockout.mtl --output-dir assets

Requires Node 24+ (installed transiently from npm if needed) and about 1.2 GB RAM.
The runtime is Blender WebAssembly 5.3 alpha, EEVEE-only and cannot render images.
"""
from __future__ import annotations
import argparse
import glob
import json
import os
import select
import subprocess
import sys
from pathlib import Path

PACKAGE='@blackbelt-technology/pi-dashboard-blender'
NODE_PACKAGE='node@24.22.3'


def ensure_runtime(root: Path) -> tuple[str,str]:
    cmd=['npx','--yes','--package='+NODE_PACKAGE,'--package='+PACKAGE,'pi-blender','ensure','--json']
    proc=subprocess.run(cmd,cwd=root,text=True,capture_output=True,timeout=240)
    if proc.returncode:
        raise RuntimeError('Blender WebAssembly provisioning failed:\n'+proc.stderr+'\n'+proc.stdout)
    print(proc.stdout.strip())
    candidates=glob.glob(os.path.expanduser('~/.npm/_npx/*/node_modules/node/bin/node'))
    node=None
    for candidate in candidates:
        v=subprocess.run([candidate,'--version'],text=True,capture_output=True,timeout=5)
        if v.returncode==0 and v.stdout.strip().startswith('v24.'):
            node=candidate; break
    if not node:
        raise RuntimeError('Node 24 binary not found in npm cache after ensure')
    packages=glob.glob(os.path.expanduser('~/.npm/_npx/*/node_modules/'+PACKAGE+'/dist/blender.js'))
    if not packages:
        raise RuntimeError('Blender WebAssembly MCP server was not found in npm cache')
    return node,packages[-1]


class McpClient:
    def __init__(self, node: str, server: str, root: Path):
        self.proc=subprocess.Popen([node,server,'mcp','--root',str(root)],stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        self.next_id=1; self.pending=b''
    def send(self,payload):
        self.proc.stdin.write((json.dumps(payload,separators=(',',':'))+'\n').encode()); self.proc.stdin.flush()
    def receive(self,timeout):
        while b'\n' not in self.pending:
            ready,_,_=select.select([self.proc.stdout],[],[],timeout)
            if not ready:
                err=b''
                if select.select([self.proc.stderr],[],[],0)[0]: err=os.read(self.proc.stderr.fileno(),8192)
                raise TimeoutError('Blender WASM response timeout: '+err.decode(errors='replace'))
            chunk=os.read(self.proc.stdout.fileno(),65536)
            if not chunk: raise RuntimeError('Blender WASM server ended unexpectedly')
            self.pending+=chunk
        line,self.pending=self.pending.split(b'\n',1)
        return json.loads(line)
    def request(self,method,params,timeout=60):
        ident=self.next_id; self.next_id+=1
        self.send({'jsonrpc':'2.0','id':ident,'method':method,'params':params})
        while True:
            response=self.receive(timeout)
            if response.get('id')==ident:
                if 'error' in response: raise RuntimeError(str(response['error']))
                return response.get('result',{})
    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:self.proc.kill()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('script',help='Blender Python script to execute')
    p.add_argument('--input',action='append',default=[],help='Root-relative file to stage in Blender')
    p.add_argument('--output-dir',default='blender-out',help='Root-relative output directory')
    p.add_argument('--timeout',type=int,default=300,help='Blender tool timeout, seconds')
    a=p.parse_args(); root=Path.cwd().resolve(); source=Path(a.script).resolve()
    if not source.is_file(): p.error('script not found: '+str(source))
    for item in a.input:
        candidate=(root/item).resolve()
        if root not in candidate.parents or not candidate.is_file(): p.error('input must be an existing file inside project root: '+item)
    node,server=ensure_runtime(root); client=McpClient(node,server,root)
    try:
        init=client.request('initialize',{'protocolVersion':'2024-11-05','capabilities':{},'clientInfo':{'name':'Arena Blender runner','version':'1.0'}})
        client.send({'jsonrpc':'2.0','method':'notifications/initialized','params':{}})
        code=source.read_text(encoding='utf-8')
        result=client.request('tools/call',{'name':'blender_python','arguments':{
            'code':code,'inputs':a.input,'outputDir':a.output_dir,'timeoutSec':a.timeout}},timeout=a.timeout+60)
        print('Blender runtime:',init.get('serverInfo',{}).get('version'))
        for item in result.get('content',[]):
            if item.get('type')=='text': print(item.get('text',''))
        if result.get('isError'): raise RuntimeError('Blender Python script returned an error')
    finally: client.close()

if __name__=='__main__': main()
