#!/usr/bin/env python3
"""Serve the GR86 render review on port 4173 and the project assets read-only."""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class PreviewHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.path = "/preview/index.html"
        return super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", 4173), PreviewHandler)
    print("GR86 render review listening on http://0.0.0.0:4173", flush=True)
    server.serve_forever()
