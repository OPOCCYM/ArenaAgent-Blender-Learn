#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_viewer.py — интерактивный просмотрщик промежуточных рендеров автомобиля.

Запуск:
    python3 render_viewer.py

Открыть: http://localhost:8090 (или превью-порт Arena)

Что показывает:
  * Все PNG из renders/  (software-превью, рендеры Blender 5 кадров)
  * Все PNG из renders/cycles/  (настоящие Cycles-рендеры desktop Blender)
  * Галерея с карточками, метаданными (размер, дата, папка)
  * Режим просмотра одного кадра на весь экран с увеличением
  * Режим сравнения двух кадров side-by-side (например, software vs Cycles)
  * Автообновление при появлении новых файлов (отключён кэш)

Не требует Blender — только Python 3 + стандартная библиотека.
"""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import json
import os
import re
import mimetypes

ROOT = Path(__file__).resolve().parents[1]
PORT = 8090

# Папки с рендерами (относительно корня проекта)
RENDER_DIRS = {
    'renders': ROOT / 'renders',
    'renders/cycles': ROOT / 'renders' / 'cycles',
}

HTML_TEMPLATE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Просмотрщик рендеров — ArenaAgent-Blender-Learn</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: system-ui, -apple-system, sans-serif; background: #0b0d12; color: #e6eaf2; }
  header { max-width: 1400px; margin: 0 auto 16px; padding: 16px 20px; display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 12px; border-bottom: 1px solid #242b38; background: #121824; position: sticky; top: 0; z-index: 50; }
  h1 { margin: 0; font-size: 18px; }
  .meta { font-size: 12px; color: #9aa6bc; }
  .toolbar { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
  button { background: #1f2a3a; border: 1px solid #334055; color: #e6eaf2; padding: 6px 12px; border-radius: 6px; font-size: 13px; cursor: pointer; }
  button:hover { border-color: #4f7cff; }
  button.active { border-color: #4f7cff; background: #2a3a55; }
  button.secondary { background: #121824; }
  .empty { text-align: center; padding: 60px 20px; color: #6b788c; font-size: 15px; }
  .empty b { color: #e6eaf2; }

  /* Галерея */
  .grid { max-width: 1400px; margin: 0 auto 30px; padding: 0 20px; display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 16px; }
  .card { background: #161b24; border: 1px solid #262f40; border-radius: 10px; overflow: hidden; cursor: pointer; transition: transform .12s ease, border-color .12s ease, box-shadow .12s ease; display: flex; flex-direction: column; }
  .card:hover { transform: translateY(-3px); border-color: #4f7cff; box-shadow: 0 8px 24px rgba(0,0,0,.35); }
  .card.selected { border-color: #4f7cff; background: #1e2838; }
  .card img { width: 100%; height: 200px; object-fit: cover; background: #0b0d12; display: block; }
  .card .cap { padding: 10px 14px; display: flex; flex-direction: column; gap: 4px; }
  .card .cap strong { display: block; font-size: 14px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .card .cap span { font-size: 11px; color: #9aa6bc; }
  .badge { display: inline-block; font-size: 10px; padding: 1px 6px; border-radius: 4px; margin-right: 6px; text-transform: uppercase; letter-spacing: .05em; }
  .badge.cycles { background: #1b3a2e; color: #7fd08c; border: 1px solid #2e5d46; }
  .badge.software { background: #3a2e1b; color: #d0b07f; border: 1px solid #5d462e; }

  /* Модалка */
  #modal { position: fixed; inset: 0; background: rgba(6,8,12,.97); display: none; align-items: center; justify-content: center; padding: 24px; z-index: 200; cursor: zoom-out; }
  #modal.open { display: flex; flex-direction: column; }
  #modal .close { position: absolute; top: 16px; right: 24px; font-size: 28px; color: #9aa6bc; cursor: pointer; z-index: 210; }
  #modal .close:hover { color: #fff; }
  #modal .toolbar { justify-content: center; margin-bottom: 12px; flex-wrap: wrap; }
  #modal img { max-width: 92vw; max-height: 78vh; border-radius: 8px; box-shadow: 0 20px 80px rgba(0,0,0,.8); cursor: zoom-in; }
  #modal .info { margin-top: 10px; text-align: center; font-size: 13px; color: #9aa6bc; }
  #modal .info strong { color: #e6eaf2; }

  /* Сравнение */
  #compare-wrap { display: none; margin-top: 10px; gap: 10px; }
  #compare-wrap.active { display: flex; }
  .compare-pane { flex: 1; background: #121824; border: 1px solid #242b38; border-radius: 8px; overflow: hidden; display: flex; flex-direction: column; }
  .compare-pane .cap { padding: 8px 12px; font-size: 12px; color: #9aa6bc; border-bottom: 1px solid #242b38; text-align: center; }
  .compare-pane img { width: 100%; height: 300px; object-fit: contain; background: #0b0d12; }

  footer { max-width: 1400px; margin: 0 auto 30px; padding: 0 20px 40px; font-size: 12px; color: #6b788c; border-top: 1px solid #242b38; }
</style>
</head>
<body>
<header>
  <h1>🚗 Просмотрщик рендеров автомобиля</h1>
  <div class="meta">Каталог папки <b>renders/</b> · клик по кадру — полный экран · <b>C</b> — сравнение</div>
  <div class="toolbar">
    <button id="btn-refresh" class="secondary">🔄 Обновить список</button>
    <button id="btn-compare" class="secondary">⚖️ Сравнить 2 кадра</button>
    <button id="btn-close-compare" class="secondary" style="display:none">✖ Скрыть сравнение</button>
  </div>
</header>

<div id="empty" class="empty">
  <b>Рендеров не найдено.</b><br>
  Сначала сгенерируйте их:<br>
  <code>blender -b -noaudio -P car_build.py</code><br>
  <code>blender -b -noaudio car_realistic.blend --python render_blender_cycles.py</code><br>
  или software-превью:<br>
  <code>python3 render_obj_png.py exports/car_ue.obj</code>
</div>

<div class="grid" id="grid"></div>

<div id="compare-wrap">
  <div class="compare-pane">
    <div class="cap" id="cap-a">Кадр A</div>
    <img id="img-a" src="" alt="">
  </div>
  <div class="compare-pane">
    <div class="cap" id="cap-b">Кадр B</div>
    <img id="img-b" src="" alt="">
  </div>
</div>

<div id="modal" onclick="closeModal(event)">
  <div class="close" onclick="event.stopPropagation(); closeModal()">✕</div>
  <div class="toolbar">
    <button class="secondary" onclick="zoomIn()">🔍 Увеличить</button>
    <button class="secondary" onclick="zoomOut()">🔎 Уменьшить</button>
    <button class="secondary" onclick="resetZoom()">↻ Сброс</button>
    <button class="secondary" onclick="toggleCompareFromModal()">⚖️ Добавить к сравнению</button>
    <button class="secondary" onclick="closeModal()">Закрыть</button>
  </div>
  <img id="mimg" src="" alt="" onclick="event.stopPropagation()">
  <div class="info" id="minfo"></div>
</div>

<footer>
  Режимы: <b>галерея</b> → клик кадр → <b>полный экран</b> ·两张 карты в <b>сравнении</b> для differences software vs Cycles.
</footer>

<script>
const API = '/api/renders';
let allRenders = [];
let selected = null;
let compareSelection = []; // массив из 2 путей
let scale = 1;
let currentPath = '';

async function loadRenders() {
  const res = await fetch(API);
  allRenders = await res.json();
  renderGrid();
  document.getElementById('empty').style.display = allRenders.length ? 'none' : 'block';
}

function renderGrid() {
  const grid = document.getElementById('grid');
  grid.innerHTML = '';
  // Группируем по папкам
  const byFolder = {};
  for (const r of allRenders) {
    (byFolder[r.folder] = byFolder[r.folder] || []).push(r);
  }
  const order = ['renders', 'renders/cycles'];
  const folders = order.filter(f => byFolder[f]).concat(Object.keys(byFolder).filter(f => !order.includes(f)));
  for (const folder of folders) {
    const renders = byFolder[folder];
    const badgeCls = folder === 'renders/cycles' ? 'cycles' : 'software';
    const badgeText = folder === 'renders/cycles' ? 'CYCLES' : 'PREVIEW';
    // Заголовок папки
    const h = document.createElement('div');
    h.style.cssText = 'max-width:1400px;margin:24px auto 8px;padding:0 20px;font-size:12px;color:#6b788c;text-transform:uppercase;letter-spacing:.1em;border-bottom:1px solid #242b38;';
    h.textContent = badgeText + ' · ' + renders.length + ' кадров';
    grid.appendChild(h);
    for (const r of renders) {
      grid.appendChild(makeCard(r, badgeCls, badgeText));
    }
  }
}

function makeCard(r, badgeCls, badgeText) {
  const card = document.createElement('article');
  card.className = 'card';
  card.onclick = (e) => { e.stopPropagation(); openModal(r); };
  const name = r.name.replace(/\.[^/.]+$/, '');
  const date = new Date(r.mtime * 1000).toLocaleString('ru-RU', { day: '2-digit', month: '2-digit', year: 'numeric', hour: '2-digit', minute: '2-digit' });
  const kb = Math.round(r.size / 1024);
  card.innerHTML = `
    <img src="/file?path=${encodeURIComponent(r.path)}" alt="${r.name}" loading="lazy">
    <div class="cap">
      <span class="badge ${badgeCls}">${badgeText}</span>
      <strong>${escapeHtml(name)}</strong>
      <span>${r.size_x}×${r.size_y} · ${kb} KB · ${date}</span>
    </div>
  `;
  return card;
}

function escapeHtml(s) { return s.replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":"&#39;"}[c])); }

let modalImg = document.getElementById('mimg');
let modalInfo = document.getElementById('minfo');
let modalScale = 1;

function openModal(r) {
  selected = r;
  currentPath = r.path;
  modalScale = 1;
  modalImg.src = '/file?path=' + encodeURIComponent(r.path) + '?t=' + Date.now();
  modalImg.style.transform = 'scale(1)';
  modalImg.style.cursor = 'zoom-in';
  const date = new Date(r.mtime * 1000).toLocaleString('ru-RU');
  const kb = Math.round(r.size / 1024);
  modalInfo.innerHTML = `<strong>${escapeHtml(r.name)}</strong><br>${r.size_x}×${r.size_y} · ${kb} KB · ${date}<br>${r.folder}`;
  document.getElementById('modal').classList.add('open');
  document.body.style.overflow = 'hidden';
}

function closeModal(ev) {
  if (ev && ev.target !== document.getElementById('modal')) return;
  document.getElementById('modal').classList.remove('open');
  document.body.style.overflow = '';
  modalImg.src = '';
}

function zoomIn() { modalScale = Math.min(modalScale + 0.5, 4); applyScale(); }
function zoomOut() { modalScale = Math.max(modalScale - 0.5, 0.5); applyScale(); }
function resetZoom() { modalScale = 1; applyScale(); }
function applyScale() { modalImg.style.transform = 'scale(' + modalScale + ')'; }

/* Сравнение */
const btnCompare = document.getElementById('btnCompare');
const btnCloseCompare = document.getElementById('btn-close-compare');
const compareWrap = document.getElementById('compare-wrap');

btnCompare.onclick = () => {
  compareSelection = [];
  compareWrap.classList.add('active');
  btnCloseCompare.style.display = 'inline-block';
  updateCompareUI();
};
btnCloseCompare.onclick = () => {
  compareSelection = [];
  compareWrap.classList.remove('active');
  btnCloseCompare.style.display = 'none';
  updateCompareUI();
};

function addToCompare(path) {
  if (compareSelection.includes(path)) return;
  if (compareSelection.length >= 2) compareSelection.shift();
  compareSelection.push(path);
  updateCompareUI();
}

function updateCompareUI() {
  const ca = document.getElementById('cap-a');
  const ia = document.getElementById('img-a');
  const cb = document.getElementById('cap-b');
  const ib = document.getElementById('img-b');
  if (compareSelection.length >= 1) {
    const r = allRenders.find(x => x.path === compareSelection[0]);
    ca.textContent = 'A: ' + r.name;
    ia.src = '/file?path=' + encodeURIComponent(r.path) + '?t=' + Date.now();
  }
  if (compareSelection.length >= 2) {
    const r = allRenders.find(x => x.path === compareSelection[1]);
    cb.textContent = 'B: ' + r.name;
    ib.src = '/file?path=' + encodeURIComponent(r.path) + '?t=' + Date.now();
  }
}

function toggleCompareFromModal() {
  if (!selected) return;
  addToCompare(selected.path);
  closeModal();
}

/* Горячие клавиши */
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') { closeModal(); compareWrap.classList.remove('active'); btnCloseCompare.style.display='none'; compareSelection=[]; }
  if (e.key === 'c' || e.key === 'C') { if (document.getElementById('modal').classList.contains('open')) toggleCompareFromModal(); else btnCompare.click(); }
  if (e.key === '+' || e.key === '=') { if (document.getElementById('modal').classList.contains('open')) zoomIn(); }
  if (e.key === '-') { if (document.getElementById('modal').classList.contains('open')) zoomOut(); }
  if (e.key === '0') { if (document.getElementById('modal').classList.contains('open')) resetZoom(); }
});

document.getElementById('btn-refresh').onclick = loadRenders;

loadRenders();
</script>
</body>
</html>
"""


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def do_GET(self):
        from urllib.parse import unquote
        path = unquote(self.path.split('?', 1)[0])

        if path in ('/', '/index.html'):
            body = HTML_TEMPLATE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/api/renders':
            renders = scan_renders()
            data = json.dumps(renders, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)
            return

        if path == '/file':
            from urllib.parse import parse_qs
            qs = parse_qs(self.path.split('?', 1)[1] if '?' in self.path else '')
            rel = qs.get('path', [''])[0]
            if not rel:
                self.send_error(400, 'missing path')
                return
            # Безопасность: только внутри корня проекта
            fpath = (ROOT / rel).resolve()
            if ROOT not in fpath.parents and fpath != ROOT:
                self.send_error(403, 'forbidden')
                return
            if not fpath.is_file() or not fpath.suffix.lower() == '.png':
                self.send_error(404, 'not found')
                return
            with open(fpath, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)
            return

        # Fallback: раздача статики из корня (для разработки)
        return super().do_GET()

    def log_message(self, fmt, *args):
        pass


def scan_renders():
    """Сканирует папки с рендерами и возвращает список файлов с метаданными."""
    results = []
    seen = set()
    for folder, dpath in RENDER_DIRS.items():
        if not dpath.exists():
            continue
        for p in sorted(dpath.rglob('*.png')):
            try:
                st = p.stat()
                rel = p.resolve().relative_to(ROOT.resolve()).as_posix()
                if rel in seen:
                    continue
                seen.add(rel)
                # Размер изображения (через PIL если есть, иначе 0×0)
                sx = sy = 0
                try:
                    from PIL import Image
                    with Image.open(p) as im:
                        sx, sy = im.size
                except Exception:
                    pass
                results.append({
                    'name': p.name,
                    'path': rel,
                    'folder': folder,
                    'size': st.st_size,
                    'mtime': st.st_mtime,
                    'size_x': sx,
                    'size_y': sy,
                })
            except Exception:
                continue
    results.sort(key=lambda r: (r['folder'], r['name']))
    return results


def main():
    server = ThreadingHTTPServer(('0.0.0.0', PORT), ViewerHandler)
    print(f'Render viewer listening on http://0.0.0.0:{PORT}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
