#!/usr/bin/env python3
"""
EA3121 Robotics Lab 02 - Activity 3
2D ToF map server.

Receives polar scan points from the ESP32 over Wi-Fi and serves a
canvas map of them at http://<host>:8080/

Standard library only - no pip install, no requirements.txt, which
keeps the container image small and the build offline-friendly.

API (all JSON):

    POST /api/scan/start    {"scan":123,"steps":72,"step_deg":5.0,
                             "max_range_mm":2000}
    POST /api/scan/points   {"scan":123,"points":[[deg,mm],...]}
    POST /api/scan/end      {"scan":123}
    POST /api/scan          a whole scan in one shot (same fields as
                            start plus "points") - handy for curl
    POST /api/clear         forget everything

    GET  /api/map           the scan the browser should draw
    GET  /api/health        liveness probe

A distance of 0 means "no return at that bearing" - open space, not
a wall at zero range. Those bearings are kept in the payload (so the
UI can tell 'looked and saw nothing' apart from 'never looked') but
are not drawn as obstacles.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST = os.environ.get("MAP_HOST", "0.0.0.0")
PORT = int(os.environ.get("MAP_PORT", "8080"))
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

# Largest body accepted, in bytes. A 72 point scan is well under 2 KB;
# this is just a guard against a stuck client streaming forever.
MAX_BODY = 64 * 1024


class MapStore:
    """Holds the scan in progress and the last completed one.

    The ESP32 streams points as it turns, so at any moment there may
    be a partial scan worth showing. Keeping the previous complete
    scan as well means the map never blanks out while a new one is
    being collected - the browser falls back to the last good map.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._current = None
        self._latest = None

    @staticmethod
    def _new_scan(scan_id, steps, step_deg, max_range_mm):
        return {
            "scan": scan_id,
            "steps": steps,
            "step_deg": step_deg,
            "max_range_mm": max_range_mm,
            # bearing (float, 1 dp) -> distance mm. A dict, not a list,
            # so a re-sent batch overwrites cleanly instead of
            # duplicating points on the map.
            "points": {},
            "complete": False,
            "started": time.time(),
            "updated": time.time(),
        }

    def start(self, scan_id, steps=72, step_deg=5.0, max_range_mm=2000):
        with self._lock:
            self._current = self._new_scan(scan_id, steps, step_deg, max_range_mm)

    def add_points(self, scan_id, points):
        """Add a batch. Starts a scan implicitly if /start was missed -
        a dropped first packet should not cost the whole map."""
        with self._lock:
            if self._current is None or self._current["scan"] != scan_id:
                self._current = self._new_scan(scan_id, 72, 5.0, 2000)

            for pt in points:
                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                    continue
                try:
                    bearing = round(float(pt[0]) % 360.0, 1)
                    distance = int(pt[1])
                except (TypeError, ValueError):
                    continue
                self._current["points"][bearing] = max(0, distance)

            self._current["updated"] = time.time()
            return len(self._current["points"])

    def end(self, scan_id):
        with self._lock:
            if self._current is not None and self._current["scan"] == scan_id:
                self._current["complete"] = True
                self._current["updated"] = time.time()
                self._latest = self._current
                self._current = None
                return True
            return False

    def clear(self):
        with self._lock:
            self._current = None
            self._latest = None

    def snapshot(self):
        """What the browser should draw: the scan in progress if there
        is one, otherwise the last completed scan."""
        with self._lock:
            scan = self._current or self._latest
            if scan is None:
                return {"scan": None, "points": [], "complete": False}

            points = sorted(
                ([bearing, distance] for bearing, distance in scan["points"].items()),
                key=lambda p: p[0],
            )
            return {
                "scan": scan["scan"],
                "steps": scan["steps"],
                "step_deg": scan["step_deg"],
                "max_range_mm": scan["max_range_mm"],
                "complete": scan["complete"],
                "started": scan["started"],
                "updated": scan["updated"],
                "age_s": round(time.time() - scan["updated"], 1),
                "hits": sum(1 for _, d in points if d > 0),
                "points": points,
            }


STORE = MapStore()


class Handler(BaseHTTPRequestHandler):
    server_version = "ToFMapServer/1.0"

    # ----------------------------------------------------------
    # helpers
    # ----------------------------------------------------------

    def _send(self, code, body, content_type="application/json"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        # The map page may be opened from a different origin during
        # development (or fetched by a phone on the same Wi-Fi).
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _send_json(self, code, obj):
        self._send(code, json.dumps(obj))

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return None
        if length <= 0 or length > MAX_BODY:
            return None
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return None

    def _serve_static(self, filename):
        # Flatten the path so a request cannot climb out of static/
        safe = os.path.basename(filename)
        path = os.path.join(STATIC_DIR, safe)

        if not os.path.isfile(path):
            self._send(404, "not found", "text/plain")
            return

        types = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".svg": "image/svg+xml",
        }
        ctype = types.get(os.path.splitext(safe)[1], "application/octet-stream")

        with open(path, "rb") as handle:
            self._send(200, handle.read(), ctype)

    # ----------------------------------------------------------
    # routes
    # ----------------------------------------------------------

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"

        if path == "/":
            self._serve_static("index.html")
        elif path == "/api/map":
            self._send_json(200, STORE.snapshot())
        elif path == "/api/health":
            self._send_json(200, {"ok": True, "time": time.time()})
        elif path.startswith("/static/"):
            self._serve_static(path[len("/static/"):])
        else:
            self._send(404, "not found", "text/plain")

    do_HEAD = do_GET

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        data = self._read_json()

        if path == "/api/clear":
            STORE.clear()
            self._send_json(200, {"ok": True})
            return

        if data is None:
            self._send_json(400, {"ok": False, "error": "bad or missing JSON body"})
            return

        scan_id = data.get("scan", 0)

        if path == "/api/scan/start":
            STORE.start(
                scan_id,
                int(data.get("steps", 72)),
                float(data.get("step_deg", 5.0)),
                int(data.get("max_range_mm", 2000)),
            )
            self.log_message("scan %s started", scan_id)
            self._send_json(200, {"ok": True, "scan": scan_id})

        elif path == "/api/scan/points":
            points = data.get("points") or []
            total = STORE.add_points(scan_id, points)
            self._send_json(200, {"ok": True, "scan": scan_id, "total": total})

        elif path == "/api/scan/end":
            STORE.end(scan_id)
            self.log_message("scan %s complete", scan_id)
            self._send_json(200, {"ok": True, "scan": scan_id})

        elif path == "/api/scan":
            # Whole scan in one request
            STORE.start(
                scan_id,
                int(data.get("steps", 72)),
                float(data.get("step_deg", 5.0)),
                int(data.get("max_range_mm", 2000)),
            )
            STORE.add_points(scan_id, data.get("points") or [])
            STORE.end(scan_id)
            self._send_json(200, {"ok": True, "scan": scan_id})

        else:
            self._send(404, "not found", "text/plain")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def log_message(self, fmt, *args):
        # Default logging writes to stderr with a noisy timestamp;
        # docker logs already timestamps, so keep it short.
        print("[%s] %s" % (self.address_string(), fmt % args), flush=True)


def main():
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print("2D ToF map server on http://%s:%d" % (HOST, PORT), flush=True)
    print("Put this machine on the same Wi-Fi as the robot (SSID GA25LM).",
          flush=True)
    print("The robot finds it by resolving hp-pavilion.local over mDNS, "
          "then GET /api/health.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("shutting down", flush=True)
        server.server_close()


if __name__ == "__main__":
    main()
