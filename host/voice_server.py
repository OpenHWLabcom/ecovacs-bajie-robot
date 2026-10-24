#!/usr/bin/env python3
"""
八界 手机指挥系统 服务端：
- HTTPS 静态服务器（自签证书），目录 = 本文件所在目录
- POST /cmd  结构化动作 {action,...} 或 语音文本 {text} -> 经 WS 下发机器人
- GET  /state -> 机器人实时状态(电量/位置/任务)，供手机显示
手机打开 https://<host>:8791/cmd.html
"""
import os, json, socket, base64, struct, time, ssl, threading, re, subprocess, urllib.request
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_HOST = os.environ.get("ROBOT_HOST", "192.168.1.100")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "17890"))
PORT = int(os.environ.get("VOICE_PORT", "8791"))

def log(*a): print(time.strftime("%H:%M:%S"), *a, flush=True)

# ---------------- 头部俯仰常驻控制（下板 ROS action /robot/control task_type=2） ----------------
ADB = os.environ.get("ADB", "adb")
ROBOT_DEV = os.environ.get("ROBOT_DEV", ROBOT_HOST + ":5555")
_head = {"p": None, "lock": threading.Lock(), "last": None}

def _head_start():
    inner = ("chroot /linux /bin/bash -lc 'export PATH=/usr/bin:/bin; exec sshpass -p "$ROBOT_PW" ssh "
             "-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null Agi@10.0.0.2 "
             "\"bash /home/Agi/jogd/head_daemon.sh\"'")
    p = subprocess.Popen([ADB, "-s", ROBOT_DEV, "shell", inner], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    def rd():
        try:
            for line in p.stdout:
                log("[head]", line.strip())
        except Exception:
            pass
    threading.Thread(target=rd, daemon=True).start()
    log("head daemon started pid=%s" % p.pid)
    return p

def _box3d(cam, box, img):
    """调用下板 box3d.py：标注框 -> base_link 3D 点"""
    try:
        args = [str(cam)] + [("%.2f" % float(v)) for v in box] + [("%.2f" % float(v)) for v in img]
        inner = ("chroot /linux /bin/bash -lc 'export PATH=/usr/bin:/bin; sshpass -p "$ROBOT_PW" ssh "
                 "-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null Agi@10.0.0.2 "
                 "\"bash /home/Agi/jogd/box3d_one.sh %s\"'") % " ".join(args)
        pr = subprocess.run([ADB, "-s", ROBOT_DEV, "shell", inner], capture_output=True, text=True, timeout=45)
        out = (pr.stdout or "") + "\n" + (pr.stderr or "")
        for line in reversed(out.splitlines()):
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)
        return {"ok": False, "err": "rc=%s %s" % (pr.returncode, out[-200:].replace("\n", " "))}
    except Exception as e:
        return {"ok": False, "err": str(e)[:100]}

def _anno_compute(body):
    res = {"ts": body.get("ts"), "pick3d": None, "place3d": None}
    if body.get("pick"):
        res["pick3d"] = _box3d(body["pick"].get("cam", "grip"), body["pick"].get("box", []), body["pick"].get("img", []))
    if body.get("place"):
        res["place3d"] = _box3d(body["place"].get("cam", "head"), body["place"].get("box", []), body["place"].get("img", []))
    try:
        json.dump(res, open("/tmp/robot_annotation_3d.json", "w"), ensure_ascii=False)
    except Exception as e:
        log("[anno3d] save err", e)
    log("[anno3d]", json.dumps(res, ensure_ascii=False)[:300])

def head_set(v):
    try:
        v = max(0.0, min(1.3, float(v)))
    except Exception:
        return False
    with _head["lock"]:
        p = _head["p"]
        if p is None or p.poll() is not None:
            p = _head_start(); _head["p"] = p; time.sleep(0.2)
        try:
            p.stdin.write("%.3f\n" % v); p.stdin.flush(); _head["last"] = v
            return True
        except Exception as e:
            log("head send err", e); _head["p"] = None; return False

# ---------------- WebSocket 客户端（收+发） ----------------
class WS:
    def __init__(self, host, port, path):
        self.host, self.port, self.path = host, port, path
        self.s = None; self.lock = threading.Lock()
    def _read_exact(self, n, timeout=3.0):
        self.s.settimeout(timeout); buf = b""
        while len(buf) < n:
            c = self.s.recv(n - len(buf))
            if not c: raise IOError("eof")
            buf += c
        return buf
    def connect(self):
        try:
            s = socket.create_connection((self.host, self.port), 5)
            key = base64.b64encode(os.urandom(16)).decode()
            req = (f"GET {self.path} HTTP/1.1\r\nHost: {self.host}:{self.port}\r\n"
                   f"Upgrade: websocket\r\nConnection: Upgrade\r\n"
                   f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
            s.sendall(req.encode()); s.settimeout(4)
            buf = b""
            while b"\r\n\r\n" not in buf:
                c = s.recv(1024)
                if not c: raise IOError("closed")
                buf += c
            if b"101" not in buf.split(b"\r\n")[0]: raise IOError("bad handshake")
            s.settimeout(0); self.s = s
            log("[ws] connected")
        except Exception as e:
            log("[ws] connect fail:", e); self.s = None
    def _send_raw(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode()
        mask = os.urandom(4); n = len(data)
        h = bytearray([0x81])
        if n < 126: h.append(0x80 | n)
        elif n < 65536: h.append(0x80 | 126); h += struct.pack(">H", n)
        else: h.append(0x80 | 127); h += struct.pack(">Q", n)
        h += mask + bytes(data[i] ^ mask[i % 4] for i in range(n))
        self.s.sendall(bytes(h))
    def send(self, obj):
        with self.lock:
            if not self.s: self.connect()
            if not self.s: return False
            try: self._send_raw(obj); return True
            except Exception as e:
                log("[ws] send err:", e); self.s = None
                # 断线重连后再发一次（机器人 WS ~60s 会踢掉不保活的连接）
                try:
                    self.connect()
                    if self.s:
                        self._send_raw(obj); log("[ws] resend ok"); return True
                except Exception as e2:
                    log("[ws] resend err:", e2); self.s = None
                return False
    def _recv_frame(self, timeout=3.0):
        try:
            hdr = self._read_exact(2, timeout)
        except (socket.timeout, TimeoutError):
            return (None, b"")
        b1, b2 = hdr[0], hdr[1]
        op = b1 & 0x0f; ln = b2 & 0x7f
        if ln == 126: ln = struct.unpack(">H", self._read_exact(2, timeout))[0]
        elif ln == 127: ln = struct.unpack(">Q", self._read_exact(8, timeout))[0]
        return op, (self._read_exact(ln, timeout) if ln else b"")
    def request(self, mode, cmd, name, data, timeout=4.0, want=None):
        """发一条请求并读回匹配的响应。want(data)->bool 可要求特定字段(如 image_query 的 result_data)。"""
        with self.lock:
            if not self.s: self.connect()
            if not self.s: return None
            try:
                self._send_raw({"header": {"mode": mode, "type": "request", "cmd": cmd,
                                           "ts": int(time.time()), "uuid": "st", "host": ""},
                                "body": {"name": name, "task_id": "st", "data": data or {}}})
                t0 = time.time()
                while time.time() - t0 < timeout:
                    op, pl = self._recv_frame(1.0)
                    if op is None: continue
                    if op != 1: continue
                    try: o = json.loads(pl.decode("utf-8", "replace"))
                    except Exception: continue
                    if o.get("body", {}).get("name") == name:
                        d = o["body"].get("data", {})
                        if want is None or want(d): return d
            except Exception as e:
                log("[ws] request err:", e); self.s = None
            return None
    def collect_info(self, topics, timeout=1.8):
        """发 oneshot 并合并所有 robot_info 主题(独立连接用)。"""
        with self.lock:
            if not self.s: self.connect()
            if not self.s: return {}
            try:
                self._send_raw({"header": {"mode": "event", "type": "request", "cmd": "oneshot", "ts": int(time.time()), "uuid": "stc", "host": ""},
                                "body": {"name": "robot_info", "task_id": "stc", "data": {"topics": topics}}})
                out = {}; t0 = time.time()
                while time.time() - t0 < timeout:
                    op, pl = self._recv_frame(0.6)
                    if op != 1: continue
                    try: o = json.loads(pl.decode("utf-8", "replace"))
                    except Exception: continue
                    if o.get("body", {}).get("name") == "robot_info":
                        out.update(o["body"].get("data", {}))
                return out
            except Exception as e:
                log("[wsr] collect err:", e); self.s = None
            return {}

ws = WS(ROBOT_HOST, ROBOT_PORT, "/robot-ws/")
wsr = WS(ROBOT_HOST, ROBOT_PORT, "/robot-ws/")   # 专用读连接(状态/取图)，不阻塞命令通道

_id = [0]
def _hdr(mode):
    _id[0] += 1
    return {"header": {"mode": mode, "type": "request", "cmd": "start",
                       "ts": int(time.time()), "uuid": "vc-%d" % _id[0], "host": ""}}
MISSION_MODE_FILE = "/tmp/robot-mission-mode"


def _mode_default():
    try:
        m = open(MISSION_MODE_FILE).read().strip()
        if m:
            return m
    except Exception:
        pass
    return os.environ.get("ROBOT_MISSION_MODE", "mission")


def send_mission(name, data=None, mode=None):
    wp = _hdr(mode or _mode_default())
    wp["body"] = {"name": name, "task_id": "vc-%s-%d" % (name, int(time.time()*1000)), "data": data or {}}
    return ws.send(wp)
def send_app(name, data=None):
    wp = _hdr("app"); wp["body"] = {"name": name, "task_id": "vc-%s-%d" % (name, int(time.time()*1000)), "data": data or {}}
    return ws.send(wp)

def stop_speech():
    """尝试停掉当前播报（app 级 stop）。"""
    wp = _hdr("app")
    wp["cmd"] = "stop"
    wp["body"] = {"name": "speech", "task_id": "vc-stop-%d" % int(time.time()*1000), "data": {}}
    return ws.send(wp)

ARM_MODES = {"home": 2, "grip_open": 6, "grip_close": 7, "unbox": 9, "grip_maint": 40}

SAY_STATE = "/tmp/robot-say.json"
SAY_LOG = "/tmp/robot-said.log"
AMP_VOL = os.environ.get("AMP_VOLUME", "46")
AMP_URL = os.environ.get("ROBOT_CLI_URL", "http://192.168.1.100:17890/api/cli/exec")
_muted = {"m": None}


def amp_vol(v):
    """设机器人功放主音量（0=静音）。经机器人控制台在 Android 里跑 tinymix。"""
    want = (str(v) == "0")
    if _muted["m"] == want:
        return
    try:
        cmd = "nsenter -t 1 -m -- /system/bin/tinymix -D 0 'Master Playback Volume' %s" % v
        data = json.dumps({"command": cmd}).encode()
        req = urllib.request.Request(AMP_URL, data=data, headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=6).read()
        _muted["m"] = want
        log("amp_vol ->", v)
    except Exception as e:
        log("amp_vol err", str(e)[:80])


def speak(text):
    """下发语音播报，并记录「正在说话到什么时候」+ 说过的话（供麦克风半双工/回声门控）。"""
    amp_vol(AMP_VOL)   # 确保没被静音
    send_app("speech", {"text": text, "code": 0})
    try:
        est = min(30.0, max(2.0, len(text) * 0.16 + 1.5))
        with open(SAY_STATE, "w") as f:
            json.dump({"until": time.time() + est, "text": text}, f, ensure_ascii=False)
        with open(SAY_LOG, "a") as f:
            f.write("%.3f\t%s\n" % (time.time(), text.replace("\n", " ").replace("\t", " ")))
    except Exception:
        pass


def do_action(a):
    """结构化动作 -> (ok, detail)"""
    act = (a or {}).get("action", "")
    if act == "chassis":
        d = max(-1.0, min(1.0, float(a.get("d", 0)))); ang = max(-1.2, min(1.2, float(a.get("a", 0))))
        send_mission("chassis_move", {"move_distance": round(d, 3), "move_angle": round(ang, 4)})
        return (True, "行进 d=%.2f a=%.2f" % (d, ang))
    if act == "stop": send_mission("stop_all"); stop_speech(); amp_vol(0); return (True, "停止")
    if act == "arm":
        am = a.get("mode")
        if am in ("grip_open", "grip_close"):
            v = 0.69 if am == "grip_open" else 0.0
            try:
                data = json.dumps({"v": v}).encode()
                req = urllib.request.Request("http://127.0.0.1:8796/grip", data=data,
                                             headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=5).read()
                return (True, "夹爪 %.2f" % v)
            except Exception as e:
                return (False, "夹爪指令失败: %s" % str(e)[:60])
        m = ARM_MODES.get(am, None)
        if m is None: return (False, "未知臂模式")
        send_mission("robot_arm_ctrl", {"mode": m}); return (True, "臂模式 %s" % am)
    if act == "head":
        v = max(0.0, min(1.3, float(a.get("value", 0))))
        ok = head_set(v)
        return (ok, "头部 %.2f rad" % v)
    if act == "height":
        v = max(0.0, min(0.44, float(a.get("value", 0)))); send_mission("robot_height_ctrl", {"value": v}); return (True, "升降 %.2f m" % v)
    if act == "recharge": send_mission("recharge"); return (True, "回充")
    if act == "wave": send_mission("wave_hand_ctrl"); return (True, "挥手")
    if act == "photo": send_mission("image_query", {"camera_type": 2}); return (True, "头部抓拍")
    if act == "speech": speak(str(a.get("text", ""))); return (True, "播报")
    if act == "pose":
        it = a.get("pose") or {}
        send_mission("robot_pose_ctrl", {"object_pose": [it]})
        p = it.get("position", {}) or {}
        return (True, "位姿 x=%s y=%s z=%s" % (p.get("x"), p.get("y"), p.get("z")))
    if act == "hof":   # hand_observe -> map 点 (对点/示教)
        send_mission("hand_observe", {"frame_id": "map", "position": {"x": float(a.get("x", 0)), "y": float(a.get("y", 0)), "z": float(a.get("z", 0))}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "box_length": {"x": 0.02, "y": 0.02, "z": 0.005}})
        return (True, "hand_observe (%.2f,%.2f,%.2f)" % (float(a.get("x", 0)), float(a.get("y", 0)), float(a.get("z", 0))))
    if act == "mission":
        send_mission(str(a.get("name", "")), a.get("data") or {}); return (True, "mission %s" % a.get("name"))
    return (False, "未知动作")

def map_cmd(t):
    s = (t or "").strip()
    if not s: return (False, "空指令")
    def has(*ks): return any(k in s for k in ks)
    m = re.match(r"^(?:说|念|播报|告诉(?:他|它)?)(?:一句|一下)?[：:，,]?\s*(.+)$", s)
    if m: return do_action({"action": "speech", "text": m.group(1)})
    if has("停", "别动", "站住", "不要动", "停下"): return do_action({"action": "stop"})
    if has("前进", "往前", "向前", "过来", "走"): return do_action({"action": "chassis", "d": 0.3, "a": 0})
    if has("后退", "往后", "退后", "倒车", "退一下"): return do_action({"action": "chassis", "d": -0.3, "a": 0})
    if has("左转", "往左", "向左"): return do_action({"action": "chassis", "d": 0, "a": 0.5})
    if has("右转", "往右", "向右"): return do_action({"action": "chassis", "d": 0, "a": -0.5})
    if has("挥手", "招手", "打招呼"): return do_action({"action": "wave"})
    if has("抬头", "向上看"): return do_action({"action": "head", "value": 1.3})
    if has("低头", "向下看"): return do_action({"action": "head", "value": 0})
    if has("升高", "站高", "变高"): return do_action({"action": "height", "value": 0.44})
    if has("降低", "蹲下", "变矮"): return do_action({"action": "height", "value": 0})
    if has("回充", "充电", "回家", "归位"): return do_action({"action": "recharge"})
    if has("拍照", "拍张", "看看"): return do_action({"action": "photo"})
    if has("夹爪", "抓爪") and has("开"): return do_action({"action": "arm", "mode": "grip_open"})
    if has("夹爪", "抓爪") and has("关", "合"): return do_action({"action": "arm", "mode": "grip_close"})
    speak(s); return (True, "未匹配控制指令，已让机器人复述")

class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw): super().__init__(*a, directory=HERE, **kw)
    def log_message(self, *a): pass
    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/state":
            d = wsr.collect_info(["pos", "battery", "workState", "alarm"], timeout=1.6)
            self._json(200, {"ok": True, "data": d}); return
        if p == "/annotation3d":
            try:
                d = json.load(open("/tmp/robot_annotation_3d.json"))
            except Exception:
                d = {"ok": False, "status": "none"}
            self._json(200, d); return
        if p == "/cam.jpg":
            import base64 as _b64
            d = wsr.request("mission", "start", "image_query", {"camera_type": 2},
                           timeout=10.0, want=lambda x: "result_data" in x) or {}
            b64s = (((d.get("result_data") or {}).get("rgb_image") or {}).get("data")) or ""
            if b64s:
                try: raw = _b64.b64decode(b64s)
                except Exception: raw = b""
                if raw:
                    self.send_response(200); self.send_header("Content-Type", "image/png")
                    self.send_header("Content-Length", str(len(raw))); self.send_header("Cache-Control", "no-store")
                    self.end_headers(); self.wfile.write(raw); return
            self.send_error(504, "no image"); return
        if p == "/":
            self.path = "/cmd.html"
        return super().do_GET()
    def do_POST(self):
        p = self.path.split("?")[0]
        log("[post] path=", repr(self.path), "p=", repr(p))
        if p not in ("/cmd", "/annotate"): self.send_error(404); return
        n = int(self.headers.get("Content-Length") or 0)
        try: body = json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception: self._json(400, {"ok": False, "error": "bad json"}); return
        if p == "/annotate":
            try:
                open("/tmp/robot_annotation.json", "w").write(json.dumps(body, ensure_ascii=False))
                open("/tmp/robot_annotation_3d.json", "w").write(json.dumps({"ts": body.get("ts"), "status": "computing"}, ensure_ascii=False))
            except Exception as e:
                log("[anno] save err", e)
            threading.Thread(target=_anno_compute, args=(dict(body),), daemon=True).start()
            log("[anno]", json.dumps(body, ensure_ascii=False)[:300])
            self._json(200, {"ok": True, "saved": "/tmp/robot_annotation.json", "status": "computing"}); return
        try:
            if body.get("action"): ok, detail = do_action(body)
            else: ok, detail = map_cmd(body.get("text", ""))
        except Exception as e: ok, detail = (False, str(e))
        log("[cmd]", body, "->", detail)
        self._json(200, {"ok": ok, "detail": detail})
    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def _json(self, code, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers(); self.wfile.write(b)

PORT_PLAIN = int(os.environ.get("VOICE_PORT_HTTP", "8792"))  # 手机用明文 HTTP(避免自签证书拦截)

def main():
    ws.connect(); wsr.connect()
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(os.path.join(HERE, "cert.pem"), os.path.join(HERE, "key.pem"))
    h1 = ThreadingHTTPServer(("0.0.0.0", PORT), H)
    h1.socket = ctx.wrap_socket(h1.socket, server_side=True)
    threading.Thread(target=h1.serve_forever, daemon=True).start()
    h2 = ThreadingHTTPServer(("0.0.0.0", PORT_PLAIN), H)
    log("[voice] HTTPS :%d  + HTTP :%d  robot=%s:%d" % (PORT, PORT_PLAIN, ROBOT_HOST, ROBOT_PORT))
    h2.serve_forever()

if __name__ == "__main__":
    main()
