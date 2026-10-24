#!/usr/bin/env python3
# graspproxy.py — 把机器人下板 picksvc 的抓取日志转给模拟界面(:8794)
# GET /log → {running, ok, dt, log[]}   GET /start → 触发一次自动抓取
import json, time, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
sys.path.insert(0, '/opt/openhwlab/robot-viz')
import subprocess, base64

ADB = "adb"
DEV = "192.168.1.100:5555"
HOST = "10.0.0.2:9100"


def _sh(remote, timeout=20):
    remote = remote.replace('&', '\\&')
    outer = ('chroot /linux /bin/bash -lc "export PATH=/usr/bin:/bin:/usr/sbin:/sbin; %s"' % remote.replace('"', '\\"'))
    try:
        r = subprocess.run([ADB, "-s", DEV, "shell", outer], capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception as e:
        return ""


_cache = {"t": 0, "v": {"running": False, "ok": None, "dt": 0, "log": [], "err": "no data"}}


def fetch(path):
    out = _sh('curl -s --max-time 6 http://%s%s' % (HOST, path))
    try:
        return json.loads(out)
    except Exception:
        return {"ok": False, "raw": out[-80:]}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/start":
            return self._send(fetch("/autograsp"))
        if p == "/log":
            now = time.time()
            if now - _cache["t"] < 0.8:
                return self._send(_cache["v"])
            v = fetch("/grasplog")
            if isinstance(v, dict) and ("log" in v):
                _cache.update({"t": now, "v": v})
            return self._send(_cache["v"])
        return self._send({"err": "unknown"}, 404)


if __name__ == "__main__":
    print("graspproxy :8794", flush=True)
    ThreadingHTTPServer(("0.0.0.0", 8794), H).serve_forever()
