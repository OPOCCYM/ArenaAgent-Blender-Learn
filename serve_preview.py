#!/usr/bin/env python3
"""Serve the render review and project assets read-only.

Usage:
    python3 serve_preview.py

Then open http://localhost:4173 (or the Arena preview port).

Serves the project root, falling back to a preview index if present.
All cache headers are disabled so updated PNGs are picked up immediately.
"""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PORT = 4173


class PreviewHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            preview = ROOT / "preview" / "index.html"
            if preview.exists():
                self.path = "/preview/index.html"
        return super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()


if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", PORT), PreviewHandler)
    print(f"Render preview listening on http://0.0.0.0:{PORT}", flush=True)
    server.serve_forever()
