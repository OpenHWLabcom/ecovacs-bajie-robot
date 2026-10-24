#!/usr/bin/env python3
# robotclient.py — 下板本机客户端（直连 127.0.0.1:9100，与 Jetson 的 pk.py 同接口）
import json, urllib.request, base64
H = "http://127.0.0.1:9100"
def get(path, timeout=40):
    try:
        out = urllib.request.urlopen(H + path, timeout=timeout).read().decode()
        return json.loads(out)
    except Exception as e:
        return {"ok": False, "raw": str(e)[:120]}
def post(path, obj, timeout=90):
    try:
        b64 = base64.urlsafe_b64encode(json.dumps(obj).encode()).decode()
        out = urllib.request.urlopen("%s/do?p=%s&d=%s" % (H, path, b64), timeout=timeout).read().decode()
        return json.loads(out)
    except Exception as e:
        return {"ok": False, "raw": str(e)[:120]}
def state(): return get("/state")
def scan(): return get("/scan")
def _f(a): return [float(x) for x in a]
def move(rad, vmax=0.8, timeout=90): return post("/move", {"rad": _f(rad), "vmax": vmax}, timeout)
def chain(rads, vmax=0.8, timeout=180): return post("/chain", {"rads": [_f(r) for r in rads], "vmax": vmax}, timeout)
def grip(v): return post("/grip", {"v": v})
def finger():
    r = get("/finger"); return r.get("finger")

def chassis(d, a):
    return get("/chassis?d=%.3f&a=%.3f" % (float(d), float(a)), timeout=20)

def traj(rads, vmax=0.5):
    return post("/traj", {"rads": [_f(r) for r in rads], "vmax": vmax}, timeout=120)
def reset():
    return get("/reset", timeout=20)
