#!/usr/bin/env python3
# 八界真机关节角 → 8790 控制台桥
# 路径：Jetson → adb(机器人上层板) → chroot /linux → sshpass ssh Agi@10.0.0.2(密码 `<LOWER_PW>`, 见《八界一代系统操作手册》3.4) →
#       下板 ROS2(domain 16/RMW=cyclone/CYCLONEDDS_URI=/factory/default.xml) → ros2 topic echo --no-daemon /joint_states
# 解析 6 关节角(rad) → HTTP :8796/joints
# 用法: python3 joints_bridge.py [--port 8796] [--robot 192.168.1.100] [--pw robot]
import os, json, time, threading, subprocess, base64
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

ROBOT = os.environ.get("ROBOT_HOST", "192.168.1.100")
PORT = int(os.environ.get("JOINTS_PORT", "8796"))
PW = os.environ.get("ROBOT_PW", "")
LOWER = os.environ.get("LOWER_IP", "10.0.0.2")
# 真机 mvp03 关节限位(rad) —— 取自下板 arm_escape/urdf/mvp0_3_zj.urdf（J3 不对称）
JLO = [-3.12, -3.12, -2.95, -3.12, -3.12, -3.12]
JHI = [ 3.12,  3.12,  3.30,  3.12,  3.12,  3.12]
VMAX = 0.8   # 真机速度上限 1.0 rad/s，留余量，防猛冲/抖动
CLEAR_POSE = [1.1, 1.2, 2.0, 0.0, -0.8, 0.0]  # 必须：大臂远离立柱（侧摆A）
SMART_GUARD = os.environ.get("SMART_GUARD", "1") != "0"   # 路径安全时不再插侧摆A
HERE = os.path.dirname(os.path.abspath(__file__))
OFFLINE_FLAG = os.path.join(HERE, "robot_offline")

def is_offline():
    """本地模式：不连机器人，HTTP/模式/本地页面照常工作。"""
    return os.environ.get("ROBOT_OFFLINE") == "1" or os.path.exists(OFFLINE_FLAG)

state = {"deg": None, "rad": None, "ts": 0, "status": ("offline" if is_offline() else "starting")}
lock = threading.Lock()

# ---- 智能防撞：用真实运动学检查 当前→目标 的直线路径 ----
def _tba_for_lift(lift):
    """arm_base_link → base_link 变换（与 arm3d 链条一致；z = 0.48267 + lift）。"""
    import kin
    return kin.T_of([-0.058, -0.06302, 0.48267 + float(lift)], [0, np.pi / 2, 0])

def path_safe(q0, q1, lift, mcol=0.02, steps=20):
    """返回 (True/False/None, info)。None = 无法判断（调用方沿用旧规则）。"""
    try:
        import guard
        Tba = _tba_for_lift(lift)
        r1 = guard.check(q1, Tba, mcol, 0.02)
        if not r1["ok"]:
            return False, r1["col"]
        ok, bad = guard.path_ok(q0, q1, Tba, steps=steps, mcol=mcol, mch=0.02)
        if ok:
            return True, float(r1["col"])
        return False, min(float(b[1]["col"]) for b in bad)
    except Exception as e:
        return None, str(e)[:80]

# ---- 流式 jog（拖动示教平滑下发）----
JOG_ARGS = os.environ.get("JOG_ARGS", "--vmax 1.2 --amax 4.0 --tau 0.15 --win 1.2 --np 20 --rate 0.45 --lead 2.8 --dead 0.006")
jog = {"tgt": None, "ts": 0.0, "stop": False, "status": "off", "mode": None, "mode_ts": 0.0,
       "grip": None, "grip_ts": 0.0}
# 最近一次下发到真机的关节目标（/jog 或 /set），供训练数据记录动作
lastcmd = {"rad": None, "ts": 0.0, "src": None}
joglog = deque(maxlen=100)
_jog_log_t = [0.0]

def jlog(s):
    with lock:
        joglog.append("%s %s" % (time.strftime("%H:%M:%S"), s))
def log(*a): print(time.strftime("%H:%M:%S"), *a, flush=True)

ADB = "adb"

def adb_root():
    try:
        subprocess.run([ADB, "-s", ROBOT + ":5555", "root"], capture_output=True, timeout=20)
        time.sleep(1.5)
    except Exception as e:
        log("adb_root err:", e)

def remote():
    script = ("export PATH=/usr/bin:/bin\n"
              "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
              "export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml\n"
              "source /opt/ros/humble/setup.bash 2>/dev/null\n"
              "pkill -f 'ros2 topic echo --no-daemon /joint_states' 2>/dev/null\n"
              "pkill -f 'jogd/jtstream[.]py' 2>/dev/null\n"
              "sleep 0.3\n"
              "cd /home/Agi\n"
              "exec python3 -u /home/Agi/jogd/jtstream.py\n")
    b64 = base64.b64encode(script.encode()).decode()
    inner = ("chroot /linux /bin/bash -lc \"export PATH=/usr/bin:/bin:/usr/sbin:/sbin; "
             "sshpass -p %s ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=6 Agi@%s "
             "'echo %s | base64 -d | bash -s'\"" % (PW, LOWER, b64))
    return inner

def reader():
    if is_offline():
        log("offline mode: 不连接机器人，仅提供本地 HTTP 状态")
        with lock: state["status"] = "offline"
        return
    while True:
        try:
            log("connect → 下板 /joint_states (jtstream) …")
            p = subprocess.Popen([ADB, "-s", ROBOT + ":5555", "shell", remote()],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            needRoot = False
            for line in p.stdout:
                s = line.strip()
                if not s:
                    continue
                if "not permitted" in s or "chroot:" in s:
                    needRoot = True
                if s.startswith("F "):
                    try:
                        with lock: state["finger"] = round(float(s[2:]), 5)
                    except Exception:
                        pass
                    continue
                if s.startswith("L "):
                    try:
                        with lock: state["lift"] = round(float(s[2:]), 5)
                    except Exception:
                        pass
                    continue
                if s.startswith("P "):
                    try:
                        with lock: state["pitch"] = round(float(s[2:]), 5)
                    except Exception:
                        pass
                    continue
                if not s.startswith("D "):
                    if ("Error" in s) or ("error" in s) or ("Traceback" in s) or s.startswith("NO "):
                        log("jtstream:", s[:160])
                    continue
                parts = s[2:].split(",")
                if len(parts) < 7:
                    continue
                try:
                    rad = [float(x) for x in parts[:6]]
                    lift = parts[6]
                    with lock:
                        state["rad"] = rad
                        state["deg"] = [round(r * 57.29577951308232, 2) for r in rad]
                        state["ts"] = time.time(); state["status"] = "ok"
                        if len(parts) >= 13:
                            state["vel"] = [round(float(x), 4) for x in parts[7:13]]
                        if len(parts) >= 19:
                            state["eff"] = [round(float(x), 4) for x in parts[13:19]]
                        if lift not in ("x", ""):
                            state["lift"] = round(float(lift), 4)
                except Exception:
                    pass
            rc = p.wait(); log("stream ended rc=%s; 重连" % rc)
            if needRoot:
                log("检测到 chroot 失败 → adb root"); adb_root()
            with lock: state["status"] = "no /joint_states (arm stack down?)"
        except Exception as e:
            log("reader err:", e)
            with lock: state["status"] = "err: %s" % e
        time.sleep(3)

def set_joints_cmd(rad):
    pos = ",".join("%.5f" % x for x in rad)
    with lock:
        cur = state.get("rad")
    maxd = max(abs(rad[i] - cur[i]) for i in range(6)) if (cur and len(cur) == 6) else 0.3
    t = max(1.5, maxd / VMAX + 0.6)      # 按行程算时间，限速，防猛冲
    sec = int(t); nanosec = int((t - sec) * 1e9)
    inner = ("export PATH=/usr/bin:/bin; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml; "
             "source /opt/ros/humble/setup.bash 2>/dev/null; source ~/install/setup.bash 2>/dev/null; "
             "ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
             '"{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: [%s], time_from_start: {sec: %d, nanosec: %d}}]}"' % (pos, sec, nanosec))
    b64 = base64.b64encode(inner.encode()).decode()
    remote = ("chroot /linux /bin/bash -lc \"export PATH=/usr/bin:/bin:/usr/sbin:/sbin; "
              "sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null Agi@%s 'echo %s | base64 -d | bash -s'\"" % (LOWER, b64))
    return [ADB, "-s", ROBOT + ":5555", "shell", remote]

def _run_joints(rad):
    """下发一条关节轨迹；失败自动 adb root 重试一次。"""
    out = subprocess.run(set_joints_cmd(rad), capture_output=True, text=True, timeout=30)
    if out.returncode != 0 or "not permitted" in (out.stdout or "") + (out.stderr or ""):
        log("下发失败 → adb root + 重试"); adb_root()
        out = subprocess.run(set_joints_cmd(rad), capture_output=True, text=True, timeout=30)
    return out

def set_grip(v):
    """夹爪开度 0..0.69（直接走 set_grip.py，最可靠）。"""
    v = max(0.0, min(0.69, float(v)))
    inner = ("export PATH=/usr/bin:/bin; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
             "export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml; "
             "source /opt/ros/humble/setup.bash 2>/dev/null; source /home/Agi/install/setup.bash 2>/dev/null; "
             "python3 /home/Agi/jogd/set_grip.py %.3f" % v)
    b64 = base64.b64encode(inner.encode()).decode()
    remote = ("chroot /linux /bin/bash -lc \"export PATH=/usr/bin:/bin:/usr/sbin:/sbin; "
              "sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null Agi@%s 'echo %s | base64 -d | bash -s'\"" % (LOWER, b64))
    out = subprocess.run([ADB, "-s", ROBOT + ":5555", "shell", remote], capture_output=True, text=True, timeout=30)
    return out

def set_joints_safe(rad):
    """🛡 智能防撞：路径确实会撞立柱才插侧摆A；路径安全就直接走（2026-10-21 管理员要求去掉多余横臂动作）。"""
    with lock:
        cur = state.get("rad")
        lift = state.get("lift")
    if cur and len(cur) == 6:
        dmove = max(abs(rad[i] - cur[i]) for i in range(6))
        if dmove < 0.03:
            return _run_joints(rad)
        if SMART_GUARD:
            safe, info = path_safe(cur, rad, 0.0 if lift is None else lift)
            if safe is True:
                log("🛡 路径安全（离柱 %.1fcm）→ 不插侧摆A" % (float(info) * 100))
                return _run_joints(rad)
            if safe is None:
                log("⚠ 路径检查不可用(%s)，沿用旧规则" % info)
            else:
                log("🛡 路径不安全（离柱 %.1fcm）→ 侧摆A" % (float(info) * 100))
    if rad[1] < 2.35 and cur and len(cur) == 6:
        dclear = max(abs(cur[i] - CLEAR_POSE[i]) for i in range(6))
        if dclear > 0.12:
            log("🛡 必须步骤：大臂远离立柱 → 侧摆A")
            out = _run_joints(CLEAR_POSE)
            if out.returncode != 0:
                return out
            t0 = time.time()
            while time.time() - t0 < 25:
                with lock:
                    c = state.get("rad")
                if c and max(abs(c[i] - CLEAR_POSE[i]) for i in range(6)) < 0.12:
                    break
                time.sleep(0.2)
            else:
                log("⚠ 侧摆A 未在 25s 内到位，仍继续（请检查）")
    return _run_joints(rad)

# ---- jog 流式通道：常驻 adb→chroot→ssh 会话，向下板 jogd 推目标 ----
def jog_remote_cmd():
    inner = "bash /home/Agi/jogd/run.sh " + JOG_ARGS
    return ("chroot /linux /bin/bash -lc \"export PATH=/usr/bin:/bin:/usr/sbin:/sbin; "
            "sshpass -p %s ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "
            "-o ServerAliveInterval=10 Agi@%s '%s'\"" % (PW, LOWER, inner))

def _jog_drain(p):
    try:
        for line in p.stdout:
            s = line.rstrip()
            if s: jlog(s)
    except Exception:
        pass

def jog_streamer():
    p = None; last_ts = 0.0; last_hb = 0.0; last_mode = 0.0; last_grip = 0.0
    while True:
        if is_offline():
            with lock: jog["status"] = "offline"
            time.sleep(1.0)
            continue
        try:
            if p is None or p.poll() is not None:
                if p is not None:
                    jlog("session ended rc=%s; retry in 4s" % p.poll())
                    try: p.kill()
                    except Exception: pass
                    p = None
                    time.sleep(4)
                jlog("connecting → lower jogd ...")
                p = subprocess.Popen([ADB, "-s", ROBOT + ":5555", "shell", jog_remote_cmd()],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1)
                threading.Thread(target=_jog_drain, args=(p,), daemon=True).start()
                last_ts = 0.0; last_mode = 0.0; last_grip = 0.0
                with lock: jog["status"] = "connected"
            now0 = time.time()
            with lock:
                tgt = jog["tgt"]; ts = jog["ts"]; stopf = jog["stop"]; jog["stop"] = False
                if tgt is not None and (now0 - ts) > 20.0:  # 陈旧目标丢弃：防重连时把旧目标重放出去
                    jog["tgt"] = None; tgt = None; ts = 0.0
            if stopf:
                p.stdin.write(json.dumps({"stop": True}) + "\n"); p.stdin.flush()
                jlog("sent: stop")
            with lock:
                mode = jog["mode"]; mts = jog["mode_ts"]
            if mode is not None and mts != last_mode:
                p.stdin.write(json.dumps({"mode": mode}) + "\n"); p.stdin.flush()
                last_mode = mts
                jlog("mode → %s" % mode.get("label", mode.get("id", "?")))
            with lock:
                gv = jog["grip"]; gts = jog["grip_ts"]
            if gv is not None and gts != last_grip:
                p.stdin.write(json.dumps({"grip": gv}) + "\n"); p.stdin.flush()
                last_grip = gts
                jlog("grip → %.3f" % gv)
            if tgt is not None and ts != last_ts and (now0 - ts) < 8.0:
                p.stdin.write(json.dumps({"rad": tgt}) + "\n"); p.stdin.flush()
                last_ts = ts
            now = time.time()
            if now - last_hb > 3.0:
                last_hb = now
                p.stdin.write(json.dumps({"ping": 1}) + "\n"); p.stdin.flush()
            time.sleep(0.05)
        except Exception as e:
            jlog("streamer err: %s" % e)
            try:
                if p: p.kill()
            except Exception: pass
            p = None
            with lock: jog["status"] = "retry"
            time.sleep(2)

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200); self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/joints", "/"):
            with lock:
                fresh = bool(state.get("ts")) and (time.time() - state["ts"] < 5.0)
                self._send(dict(state, age=round(time.time() - state["ts"], 2) if state["ts"] else None,
                                jog_status=jog["status"], jog_age=(round(time.time() - jog["ts"], 2) if jog["ts"] else None),
                                cmd=lastcmd["rad"],
                                cmd_age=(round(time.time() - lastcmd["ts"], 2) if lastcmd["ts"] else None),
                                cmd_src=lastcmd["src"],
                                offline=is_offline(),
                                fresh=fresh,
                                robot_online=(not is_offline()) and state.get("status") == "ok" and fresh))
        elif p == "/health":
            with lock:
                fresh = bool(state.get("ts")) and (time.time() - state["ts"] < 5.0)
                self._send({"ok": True, "service": "joints_bridge", "offline": is_offline(),
                            "robot_online": (not is_offline()) and state.get("status") == "ok" and fresh,
                            "fresh": fresh, "status": state.get("status")})
        elif p == "/joglog":
            with lock: self._send({"status": jog["status"], "tgt": jog["tgt"], "mode": jog.get("mode"), "age": (round(time.time() - jog["ts"], 2) if jog["ts"] else None), "lines": list(joglog)[-60:]})
        else: self.send_error(404)
    def do_OPTIONS(self):
        log("OPTIONS", self.path)
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0"); self.end_headers()
    def do_POST(self):
        path = self.path.split("?")[0]
        if path not in ("/set", "/jog", "/mode", "/grip"):
            self.send_error(404); return
        if path in ("/set", "/jog", "/grip") and is_offline():
            self._send({"ok": False, "offline": True, "err": "robot offline (local mode)"}); return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            j = json.loads(self.rfile.read(n) or b"{}")
            if path == "/grip":
                v = max(0.0, min(0.69, float(j.get("v", 0.185))))
                with lock:
                    jog["grip"] = v; jog["grip_ts"] = time.time()
                    connected = jog["status"] == "connected"
                if connected:
                    log("POST /grip", v, "(stream)")
                    self._send({"ok": True, "v": v, "via": "stream"})
                    return
                try:
                    out = set_grip(v)
                    ok = out.returncode == 0
                    log("POST /grip", v, "rc", out.returncode, "(ssh fallback)")
                    self._send({"ok": ok, "v": v, "rc": out.returncode,
                                "out": (out.stdout or "")[-160:], "err": (out.stderr or "")[-160:]})
                except Exception as e:
                    self._send({"ok": False, "v": v, "err": str(e)})
                return
            if path == "/mode":
                m = j.get("mode") or {}
                if not isinstance(m, dict): raise ValueError("need mode dict")
                with lock:
                    jog["mode"] = m; jog["mode_ts"] = time.time()
                log("POST /mode", m.get("label", m.get("id", "?")))
                try: open("/tmp/jog_posts.log", "a").write("%s MODE %s\n" % (time.strftime("%H:%M:%S"), json.dumps(m, ensure_ascii=False)))
                except Exception: pass
                self._send({"ok": True, "mode": m}); return
            if path == "/jog":
                if j.get("stop"):
                    with lock:
                        jog["stop"] = True; jog["tgt"] = None; jog["ts"] = time.time()
                    log("POST /jog stop")
                    self._send({"ok": True, "stop": True}); return
                rad = [float(x) for x in (j.get("rad") or [])]
                if len(rad) != 6: raise ValueError("need 6 joints")
                rad = [max(JLO[i], min(JHI[i], rad[i])) for i in range(6)]
                with lock:
                    jog["tgt"] = rad; jog["ts"] = time.time()
                    lastcmd["rad"] = rad; lastcmd["ts"] = jog["ts"]; lastcmd["src"] = "jog"
                now = time.time()
                try:
                    open("/tmp/jog_posts.log", "a").write("%s %s\n" % (time.strftime("%H:%M:%S"), ",".join("%.3f" % x for x in rad)))
                except Exception:
                    pass
                if now - _jog_log_t[0] > 2.0:
                    _jog_log_t[0] = now
                    log("POST /jog", [round(x, 3) for x in rad])
                self._send({"ok": True, "jog": True}); return
            rad = [float(x) for x in (j.get("rad") or j.get("joints") or [])]
            if len(rad) != 6: raise ValueError("need 6 joints")
            rad = [max(JLO[i], min(JHI[i], rad[i])) for i in range(6)]
            with lock:
                lastcmd["rad"] = rad; lastcmd["ts"] = time.time(); lastcmd["src"] = "set"
            log("POST /set", [round(x, 3) for x in rad])
        except Exception as e:
            self._send({"ok": False, "err": "bad body: %s" % e}); return
        try:
            out = set_joints_safe(rad)
            self._send({"ok": out.returncode == 0, "rc": out.returncode, "sent": rad, "out": (out.stdout or "")[-160:], "err": (out.stderr or "")[-160:]})
        except Exception as e:
            self._send({"ok": False, "err": str(e)})

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=PORT); ap.add_argument("--robot", default=ROBOT); ap.add_argument("--pw", default=PW)
    a = ap.parse_args(); PORT = a.port; ROBOT = a.robot; PW = a.pw
    if is_offline():
        log("OFFLINE 本地模式：仅启动 HTTP :%d（机器人不参与）" % PORT)
    else:
        threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=jog_streamer, daemon=True).start()
    log("serving http://0.0.0.0:%d/joints  (robot=%s lower=%s)" % (PORT, ROBOT, LOWER))
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
