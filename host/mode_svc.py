#!/usr/bin/env python3
# mode_svc.py — 真机/本地模式控制服务 (:8795)
# GET  /mode   → {ok, mode:"real"|"local", robot_online, reason, since, bridge_status}
# POST /mode   → {"mode":"local"} / {"mode":"real"} 切换（离线桥/在线桥由 offline.sh/online.sh 负责）
# 自动：真机模式下连续 10s 检测不到机器人 → 自动切本地模式（黄色告警由前端显示）
import os, json, time, subprocess, threading, socket
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import urllib.request

DIR = os.path.dirname(os.path.abspath(__file__))
FLAG = os.path.join(DIR, "robot_offline")
OFFLINE_SH = os.path.join(DIR, "offline.sh")
ONLINE_SH = os.path.join(DIR, "online.sh")
BRIDGE = os.environ.get("BRIDGE_HEALTH", "http://127.0.0.1:8796/health")
ROBOT_HOST = os.environ.get("ROBOT_HOST", "192.168.1.100")
ROBOT_ADB_PORT = int(os.environ.get("ROBOT_ADB_PORT", "5555"))
PORT = int(os.environ.get("MODE_PORT", "8795"))
AUTO_FAILS = int(os.environ.get("MODE_AUTO_FAILS", "5"))   # 2s * 5 = 10s
AUTO_RECOVER = int(os.environ.get("MODE_AUTO_RECOVER", "3"))  # 2s * 3 = 6s 在线后自动回真机
RECOVER_COOLDOWN = float(os.environ.get("MODE_RECOVER_COOLDOWN", "120"))  # 自动回切失败后的冷却
GRACE = float(os.environ.get("MODE_GRACE", "30"))          # 启动/切换后的宽限
START_DEFAULT_REAL = os.environ.get("MODE_START_DEFAULT_REAL", "1") != "0"

lock = threading.Lock()

# 启动默认真机模式（2026-10-24 管理员要求）：清掉上次遗留的离线标记；
# 真掉线时看门狗 10s 内仍会自动切本地。
if START_DEFAULT_REAL:
    try:
        os.remove(FLAG)
    except FileNotFoundError:
        pass

st = {
    "mode": "local" if os.path.exists(FLAG) else "real",
    "since": time.time(),
    "reason": "startup: default real" if START_DEFAULT_REAL else "startup",
    "robot_online": False,
    "bridge_status": "unknown",
    "busy_until": 0.0,
    "auto": False,
}
last_health = {"ok": False, "robot_online": False, "status": "unknown", "ts": 0.0}
last_recover = {"ts": 0.0, "fail": 0}


def set_mode(m, reason, auto=False):
    with lock:
        st["mode"] = m
        st["reason"] = reason
        st["auto"] = bool(auto)
        st["since"] = time.time()


def run_script(path, reason):
    try:
        log = open("/tmp/robotviz-mode.log", "a")
        subprocess.Popen(["bash", path], stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True, close_fds=True)
        with lock:
            st["busy_until"] = time.time() + 8.0
        print(time.strftime("%H:%M:%S"), "run", os.path.basename(path), "-", reason, flush=True)
    except Exception as e:
        print(time.strftime("%H:%M:%S"), "run script err", e, flush=True)


def bridge_health(timeout=1.5):
    try:
        with urllib.request.urlopen(BRIDGE, timeout=timeout) as r:
            return json.load(r)
    except Exception as e:
        return {"ok": False, "robot_online": False, "status": "bridge-down", "error": str(e)[:80]}


def robot_reachable(timeout=1.2):
    """本地模式下探测机器人是否回来：adbd 端口(5555)可连 = 机器人已启动。
    离线桥连不到机器人，所以不能用 /health。"""
    try:
        with socket.create_connection((ROBOT_HOST, ROBOT_ADB_PORT), timeout=timeout):
            return True
    except Exception:
        return False


def watchdog():
    fails = 0
    oks = 0
    while True:
        time.sleep(2.0)
        # 手动 offline.sh / online.sh 也会改标记，这里跟随
        flag = os.path.exists(FLAG)
        want = "local" if flag else "real"
        with lock:
            cur = st["mode"]
            busy = time.time() < st["busy_until"]
        if cur != want and not busy:
            # 外部脚本改的标记 = 人工操作，不参与自动回切
            set_mode(want, "flag sync", auto=False)

        h = bridge_health()
        last_health.update(h if isinstance(h, dict) else {})
        last_health["ts"] = time.time()
        with lock:
            st["robot_online"] = bool(last_health.get("robot_online"))
            st["bridge_status"] = last_health.get("status", "?")
            cur = st["mode"]
            since = st["since"]
            busy = time.time() < st["busy_until"]
            auto = st["auto"]

        if busy:
            fails = 0
            oks = 0
            continue

        if cur == "real":
            oks = 0
            if time.time() - since < GRACE:
                continue
            if st["robot_online"]:
                fails = 0
                continue
            fails += 1
            if fails >= AUTO_FAILS:
                print(time.strftime("%H:%M:%S"), "AUTO → local (robot offline)", flush=True)
                try:
                    open(FLAG, "w").close()
                except Exception:
                    pass
                set_mode("local", "auto: robot offline > %.0fs" % (AUTO_FAILS * 2), auto=True)
                run_script(OFFLINE_SH, "auto offline")
                fails = 0
            continue

        # 本地模式：只有「自动切过来」的才自动回真机；手动切本地保持不动
        fails = 0
        if not auto:
            oks = 0
            continue
        if last_recover["ts"] and time.time() - last_recover["ts"] < RECOVER_COOLDOWN:
            # 刚自动回切过又掉线：别再频繁来回切
            oks = 0
            continue
        reach = robot_reachable()
        if reach:
            oks += 1
            if oks >= AUTO_RECOVER:
                print(time.strftime("%H:%M:%S"), "AUTO → real (robot back online)", flush=True)
                last_recover["ts"] = time.time()
                try:
                    os.remove(FLAG)
                except FileNotFoundError:
                    pass
                set_mode("real", "auto: robot back online > %.0fs" % (AUTO_RECOVER * 2), auto=False)
                run_script(ONLINE_SH, "auto real")
                oks = 0
        else:
            oks = 0


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

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/mode", "/health", "/"):
            with lock:
                out = dict(st)
            out["ok"] = True
            out["service"] = "mode_svc"
            out["flag"] = os.path.exists(FLAG)
            out["health_age"] = round(time.time() - last_health.get("ts", 0), 1)
            self._send(out)
        else:
            self.send_error(404)

    def do_POST(self):
        p = self.path.split("?")[0]
        if p != "/mode":
            self.send_error(404)
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            j = json.loads(self.rfile.read(n) or b"{}")
            target = j.get("mode")
            if target not in ("real", "local"):
                raise ValueError("mode must be real|local")
            with lock:
                cur = st["mode"]
                if time.time() < st["busy_until"]:
                    self._send({"ok": False, "err": "switching, retry later", "mode": cur})
                    return
            if target == cur:
                self._send({"ok": True, "mode": cur, "unchanged": True})
                return
            if target == "local":
                open(FLAG, "w").close()
                set_mode("local", "user: local mode")
                run_script(OFFLINE_SH, "user local")
            else:
                try:
                    os.remove(FLAG)
                except FileNotFoundError:
                    pass
                set_mode("real", "user: real mode")
                run_script(ONLINE_SH, "user real")
            self._send({"ok": True, "mode": target, "switching": True})
        except Exception as e:
            self._send({"ok": False, "err": str(e)}, 400)


if __name__ == "__main__":
    print(time.strftime("%H:%M:%S"), "mode_svc :%d mode=%s" % (PORT, st["mode"]), flush=True)
    threading.Thread(target=watchdog, daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
