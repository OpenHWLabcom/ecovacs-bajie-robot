#!/usr/bin/env python3
# relay8793.py — 上板转发：/chassis?d=&a= → Jetson 8792 /cmd（下板→10.10.10.12:8793）
import json, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
JETSON = "http://127.0.0.1:8792/cmd"
class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code); self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Content-Length", str(len(b)))
        self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        p = self.path.split("?")[0]
        if p != "/chassis":
            return self._send({"err": "unknown"}, 404)
        import urllib.parse
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        d = float(q.get("d", ["0"])[0]); a = float(q.get("a", ["0"])[0])
        try:
            body = json.dumps({"action": "chassis", "d": d, "a": a}).encode()
            req = urllib.request.Request(JETSON, data=body, headers={"Content-Type": "application/json"}, method="POST")
            out = urllib.request.urlopen(req, timeout=12).read().decode()
            return self._send({"ok": True, "resp": out[:120]})
        except Exception as e:
            return self._send({"ok": False, "err": str(e)[:90]})
if __name__ == "__main__":
    print("relay :8793", flush=True)
    ThreadingHTTPServer(("0.0.0.0", 8793), H).serve_forever()
