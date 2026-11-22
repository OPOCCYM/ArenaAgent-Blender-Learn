#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_gallery.py — HTTP-сервер галереи промежуточных рендеров.

Запуск после получения рендеров из Blender:

    python3 render_gallery.py

Затем открыть http://localhost:8080 (или превью-порт Arena).

Требует папку renders/ с PNG-файрами, полученными через Blender:
  blender -b -noaudio car_realistic.blend -P render_stills.py
"""
import http.server
import os
import socketserver
from urllib.parse import unquote

RENDERS_DIR = os.path.abspath('renders')
PORT = 8080

CARDS = [
    ("Cam_Exterior.png", "Cam_Exterior", "Передний ¾-вид (кадр 1, закрытые двери)"),
    ("Cam_Rear.png", "Cam_Rear", "Задний ¾-вид (фонари, корма, чистый ракурс)"),
    ("Cam_Side.png", "Cam_Side", "Боковой профиль (клиренс, колёса на Z=0)"),
    ("Cam_Exterior_DoorsOpen.png", "Cam_Exterior — кадр 60", "Анимация: все 4 двери открыты наружу на 70°"),
    ("Cam_Interior.png", "Cam_Interior", "Салон сзади: сиденья, тоннель, панель, руль"),
    ("Cam_Interior2.png", "Cam_Interior2", "Крупный план руля, панели и центрального экрана"),
    ("preview_all.png", "Сводная сетка 3×2", "Все 6 промежуточных рендеров на одном листе"),
]

HTML = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
title>Промежуточные рендеры — ArenaAgent-Blender-Learn</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: system-ui, -apple-system, sans-serif; background: #0f1218; color: #e6eaf2; padding: 20px; }
  header { max-width: 1440px; margin: 0 auto 20px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: baseline; gap: 12px; border-bottom: 1px solid #242b38; padding-bottom: 12px; }
  h1 { margin: 0; font-size: 20px; }
  .meta { font-size: 13px; color: #9aa6bc; }
  .grid { max-width: 1440px; margin: 0 auto; display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 16px; }
  .card { background: #161b24; border: 1px solid #262f40; border-radius: 10px; overflow: hidden; cursor: zoom-in; transition: transform .12s ease, border-color .12s ease; }
  .card:hover { transform: translateY(-2px); border-color: #4f7cff; }
  .card.wide { grid-column: 1 / -1; }
  .card img { width: 100%; display: block; background: #0b0d12; }
  .cap { padding: 10px 14px; }
  .cap strong { display: block; font-size: 14px; }
  .cap span { font-size: 12px; color: #9aa6bc; }
  #modal { position: fixed; inset: 0; background: rgba(6,8,12,.92); display: none; align-items: center; justify-content: center; padding: 24px; z-index: 100; cursor: zoom-out; }
  #modal.open { display: flex; }
  #modal img { max-width: 96vw; max-height: 92vh; border-radius: 8px; box-shadow: 0 20px 60px rgba(0,0,0,.7); }
</style>
</head>
<body>
<header>
  <h1>Промежуточные рендеры — Blender 4.2 (Cycles CPU, AgX)</h1>
  <div class="meta">Рендеры из Blender · клик по кадру — увеличить</div>
</header>
<main class="grid">
__CARDS__
</main>
<div id="modal" onclick="this.classList.remove('open')"><img id="mimg" src="" alt=""></div>
<script>
function zoom(src) {
  document.getElementById('mimg').src = src;
  document.getElementById('modal').classList.add('open');
}
document.addEventListener('keydown', e => { if (e.key === 'Escape') document.getElementById('modal').classList.remove('open'); });
</script>
</body>
</html>
"""


class GalleryHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        path = unquote(self.path.split("?", 1)[0])
        if path in ("/", "/index.html"):
            parts = []
            for fname, title, desc in CARDS:
                if os.path.exists(os.path.join(RENDERS_DIR, fname)):
                    cls = "card wide" if fname == "preview_all.png" else "card"
                    parts.append(
                        f'<article class="{cls}" onclick="zoom(\'/{fname}\')">'
                        f'<img src="/{fname}" alt="{title}" loading="lazy">'
                        f'<div class="cap"><strong>{title}</strong><span>{desc}</span></div>'
                        f'</article>'
                    )
            body = HTML.replace("__CARDS__", "\n".join(parts)).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        fname = os.path.basename(path)
        fpath = os.path.join(RENDERS_DIR, fname)
        if fname.endswith(".png") and os.path.isfile(fpath):
            with open(fpath, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_error(404, "Not found")

    def log_message(self, fmt, *args):
        pass


class ReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


if __name__ == "__main__":
    with ReusableTCPServer(("0.0.0.0", PORT), GalleryHandler) as httpd:
        print(f"Render gallery listening on http://0.0.0.0:{PORT}", flush=True)
        httpd.serve_forever()
