#!/usr/bin/env python3
# 自适应方向校准（闭环）：向机器人下发若干"相对自身"的方向点，量它转向后的实际朝向，
# 反推方向偏差，写入 calib.json（供 8790 控制台自动载入 trimDeg）。
import os, socket, base64, struct, json, time, math

HOST, PORT, PATH = "192.168.1.100", 17890, "/robot-ws/"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "calib.json")
RAW = os.path.join(HERE, "calib_raw.jsonl")

class WS:
    def __init__(s):
        s.s = socket.create_connection((HOST, PORT), 6)
        k = base64.b64encode(os.urandom(16)).decode()
        s.s.sendall((f"GET {PATH} HTTP/1.1\r\nHost: {HOST}:{PORT}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {k}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        b = b""
        while b"\r\n\r\n" not in b: b += s.s.recv(4096)
        s.pos = None; s.alarm = []
    def _ex(s, n, to=6.0):
        s.s.settimeout(to); b = b""
        while len(b) < n:
            c = s.s.recv(n - len(b))
            if not c: raise IOError
            b += c
        return b
    def send(s, o):
        d = json.dumps(o, ensure_ascii=False).encode(); m = os.urandom(4); n = len(d); h = bytearray([0x81])
        if n < 126: h.append(0x80 | n)
        elif n < 65536: h.append(0x80 | 126); h += struct.pack(">H", n)
        else: h.append(0x80 | 127); h += struct.pack(">Q", n)
        h += m + bytes(d[i] ^ m[i % 4] for i in range(n)); s.s.sendall(bytes(h))
    def recv(s, to=2.0):
        try: hd = s._ex(2, to)
        except (socket.timeout, TimeoutError): return None
        op = hd[0] & 0xf; ln = hd[1] & 0x7f
        if op == 9:
            pl = s._ex(ln, to) if ln else b""
            m = os.urandom(4); s.s.sendall(bytes([0x8A, 0x80 | len(pl)]) + m + bytes(pl[i]^m[i%4] for i in range(len(pl)))); return None
        if ln == 126: ln = struct.unpack(">H", s._ex(2, to))[0]
        elif ln == 127: ln = struct.unpack(">Q", s._ex(8, to))[0]
        pl = s._ex(ln, to) if ln else b""
        if op == 8: return {"__close__": 1}
        try: return json.loads(pl.decode("utf-8", "replace"))
        except Exception: return None

def sub(ws):
    for cmd in ("oneshot", "subscribe"):
        ws.send({"header": {"mode": "event", "type": "request", "cmd": cmd, "ts": int(time.time()), "uuid": "sw", "host": ""},
                 "body": {"name": "robot_info", "task_id": "sw", "data": {"topics": ["pos", "alarm"]}}})

def bstop(ws, why):
    print("!! 中止(%s)：下发 stop_all" % why, flush=True)
    ws.send({"header": {"mode": "mission", "type": "request", "cmd": "start", "ts": int(time.time()), "uuid": "stop", "host": ""},
             "body": {"name": "stop_all", "task_id": "stop-%d" % int(time.time()*1000), "data": {}}})

def guard(ws):
    """返回中止原因字符串，或 None。"""
    p = ws.pos
    if START.get("x") is not None and p and "x" in p:
        d = math.hypot(p["x"] - START["x"], p["y"] - START["y"])
        if d > MAX_DRIFT:
            return "漂移 %.2fm>%.2fm" % (d, MAX_DRIFT)
    if ws.alarm:
        return "告警 %s" % ws.alarm
    return None
def pump(ws, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        o = ws.recv(1.0)
        if o and o.get("body", {}).get("name") == "robot_info":
            d = o["body"].get("data", {})
            if "pos" in d: ws.pos = d["pos"]
            if "alarm" in d: ws.alarm = [c for c in (d.get("alarm") or []) if c not in (0,)]
def observe(ws, mx, my, mz, wait=60.0):
    tid = "acal-%d" % int(time.time()*1000)
    ws.send({"header": {"mode": "mission", "type": "request", "cmd": "start", "ts": int(time.time()), "uuid": tid, "host": ""},
             "body": {"name": "hand_observe", "task_id": tid, "data": {"frame_id": "map",
                      "position": {"x": mx, "y": my, "z": mz}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                      "box_length": {"x": 0.02, "y": 0.02, "z": 0.005}}}})
    t0 = time.time(); code = None; last = None
    while time.time() - t0 < wait:
        o = ws.recv(1.5)
        if not o: continue
        b = o.get("body", {}); d = b.get("data", {}) or {}
        if "pos" in d: ws.pos = d["pos"]
        if "alarm" in d: ws.alarm = [c for c in (d.get("alarm") or []) if c not in (0,)]
        g = guard(ws)
        if g:
            bstop(ws, g); return (990, None)
        if b.get("name") == "hand_observe" and b.get("task_id") == tid and o.get("header", {}).get("cmd") == "finish":
            code = (d.get("error_code") or {}).get("code"); last = dict(ws.pos or {}); break
    return code, last

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

DIRS = [(-70, "右70"), (-60, "右60"), (-50, "右50"), (-40, "右40"), (-30, "右30"),
        (-20, "右20"), (-10, "右10"), (0, "正前"), (10, "左10"), (20, "左20"),
        (30, "左30"), (40, "左40"), (50, "左50"), (60, "左60"), (70, "左70")]
DIST, Z = 0.45, 0.45
# ⭐ 安全护栏：机器人离起始点漂移超过 MAX_DRIFT 或收到告警 → 立即 stop_all 中止
MAX_DRIFT = float(os.environ.get("AUTOCAL_MAX_DRIFT", "0.6"))
START = {}

def main():
    ws = WS(); sub(ws); pump(ws, 2.0)
    if not (ws.pos and "x" in ws.pos): print("no pos, abort"); return 1
    START.update({"x": ws.pos["x"], "y": ws.pos["y"]})
    print("start=(%.2f,%.2f)  漂移上限 %.2fm" % (START["x"], START["y"], MAX_DRIFT), flush=True)
    samples = []
    for bdeg, name in DIRS:
        p0 = dict(ws.pos or {})
        if "x" not in p0: print("no pos, abort"); return 1
        sx, sy, syaw = p0["x"], p0["y"], p0["yaw"]
        b = math.radians(bdeg)
        lx, ly = DIST*math.cos(b), DIST*math.sin(b)
        c, s = math.cos(syaw), math.sin(syaw)
        mx, my = sx + lx*c - ly*s, sy + lx*s + ly*c
        intended = wrap(syaw + b)
        code, last = observe(ws, mx, my, Z)
        if last is None:
            time.sleep(3); pump(ws, 1.5)
            code, last = observe(ws, mx, my, Z)   # 重试一次
        if last is None:
            print("%s: finish=%s (no pos) 跳过" % (name, code), flush=True); continue
        err = math.degrees(wrap(last["yaw"] - intended))
        samples.append({"name": name, "bearing": bdeg, "err": round(err, 1)})
        print("%-4s 方位%+4d°  startYaw=%7.1f°  目标map=(%.2f,%.2f)  结束Yaw=%7.1f°  偏差=%+.1f°" % (
            name, bdeg, math.degrees(syaw), mx, my, math.degrees(last["yaw"]), err), flush=True)
        pump(ws, 1.0)
    if samples:
        # 1) 累积到 calib_raw.jsonl（跨多轮）
        rows = []
        if os.path.exists(RAW):
            for line in open(RAW):
                line = line.strip()
                if line:
                    try: rows.append(json.loads(line))
                    except Exception: pass
        tnow = int(time.time())
        for s in samples: rows.append({"ts": tnow, "bearing": s["bearing"], "err": s["err"]})
        with open(RAW, "w") as f:
            for r in rows: f.write(json.dumps(r, ensure_ascii=False) + "\n")
        # 2) 按 bearing 分组取中位
        from collections import defaultdict
        g = defaultdict(list)
        for r in rows: g[r["bearing"]].append(r["err"])
        def _med(v):
            v = sorted(v); n = len(v); return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.0
        bears = sorted(g)
        med = [{"bearing": b, "err": round(_med(g[b]), 1), "n": len(g[b])} for b in bears]
        # 3) 在“中位点”上拟合直线(仅供参考)
        n = len(med); mb = sum(m["bearing"] for m in med) / n; me = sum(m["err"] for m in med) / n
        sxx = sum((m["bearing"] - mb) ** 2 for m in med); sxy = sum((m["bearing"] - mb) * (m["err"] - me) for m in med)
        slope = (sxy / sxx) if sxx > 0 else 0.0; intercept = me - slope * mb
        rounds = max(len(g[b]) for b in bears)
        out = {"trimDeg": round(-me), "offsetDeg": round(intercept, 1), "slope": round(slope, 4),
               "cmdScale": round(1 + slope, 3), "meanErrDeg": round(me, 1),
               "samples": [{"name": "", "bearing": m["bearing"], "err": m["err"], "n": m["n"]} for m in med],
               "rounds": rounds, "ts": int(time.time()),
               "note": "samples = 逐方位中位(跨 %d 轮)；控制台按方位表反解" % rounds}
        with open(OUT, "w") as f: json.dump(out, f, ensure_ascii=False, indent=1)
        print("\n本轮 %d 点已累积(raw %d 条)；逐方位中位(最多 %d 轮) → 写入 %s" % (len(samples), len(rows), rounds, OUT), flush=True)
        print("中位表: " + "  ".join("%.0f:%+.1f(n%d)" % (m["bearing"], m["err"], m["n"]) for m in med), flush=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
