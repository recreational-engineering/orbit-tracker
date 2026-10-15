#!/usr/bin/env python3
"""Local dev server for the orbit tracker prototype.

Serves ./web as static files and proxies orbital elements from CelesTrak:
    GET /api/gp?catnr=100614   -> CelesTrak OMM JSON (cached on disk for 2 h)

CelesTrak asks clients not to re-download the same element set more often
than its update cadence, so responses are cached in ./cache.
"""
import http.server
import json
import os
import socketserver
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

ROOT = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(ROOT, "web")
CACHE_DIR = os.path.join(ROOT, "cache")
CACHE_TTL_S = 2 * 3600
CELESTRAK_URL = "https://celestrak.org/NORAD/elements/gp.php?CATNR={}&FORMAT=JSON"


def download(url: str) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "orbit-tracker-prototype"})
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            return resp.read()
    except urllib.error.URLError as e:
        # python.org builds on macOS ship without CA certs; the system curl has them.
        if not isinstance(e.reason, ssl.SSLError):
            raise
        return subprocess.run(
            ["curl", "-sfL", "-m", "20", "-A", "orbit-tracker-prototype", url],
            check=True, capture_output=True,
        ).stdout


def fetch_gp(catnr: int) -> bytes:
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, f"gp_{catnr}.json")
    if os.path.exists(path) and time.time() - os.path.getmtime(path) < CACHE_TTL_S:
        with open(path, "rb") as f:
            return f.read()
    try:
        data = download(CELESTRAK_URL.format(catnr))
        json.loads(data)  # CelesTrak returns plain text on errors
        with open(path, "wb") as f:
            f.write(data)
        return data
    except Exception:
        # Offline or CelesTrak error: fall back to a stale cache if there is one.
        if os.path.exists(path):
            with open(path, "rb") as f:
                return f.read()
        raise


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if url.path == "/api/gp":
            qs = urllib.parse.parse_qs(url.query)
            try:
                catnr = int(qs.get("catnr", [""])[0])
                body = fetch_gp(catnr)
                status = 200
            except Exception as e:
                body = json.dumps({"error": str(e)}).encode()
                status = 502
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler) as httpd:
        print(f"Orbit tracker prototype: http://localhost:{port}", flush=True)
        httpd.serve_forever()


if __name__ == "__main__":
    main()
