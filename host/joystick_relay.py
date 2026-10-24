#!/usr/bin/env python3
"""
ESP32-P4 实体摇杆(串口 J,x,y,b @115200) → 八界机器人(robot-ws) 低延迟中继。
- 等上一个底盘动作 finish 立刻发下一个(最小间隔 GAP)，避免被"任务已在运行"拒。
- 每次发送/等待后清空串口缓冲，取最新摇杆值(降延迟)。
- x,y: -127..127 ; b: SW 按下=1 -> 急停。上=前进 下=后退 左右=转向。
环境: ROBOT_HOST ROBOT_PORT SER BAUD DZ MAXD MAXA GAP
"""
import os, time, json, socket, base64, struct, threading

ROBOT_HOST = os.environ.get("ROBOT_HOST", "192.168.1.100")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "17890"))
SER        = os.environ.get("SER", "/dev/ttyACM0")
BAUD       = int(os.environ.get("BAUD", "115200"))
DZ         = int(os.environ.get("DZ", "18"))
MAXD       = float(os.environ.get("MAXD", "0.80"))
MAXA       = float(os.environ.get("MAXA", "1.10"))
GAP        = float(os.environ.get("GAP", "0.2"))
HSTEP      = float(os.environ.get("HSTEP", "0.15"))  # 升降每档(m) 0..0.44

def log(*a): print(time.strftime("%H:%M:%S"), *a, flush=True)

class WS:
    def __init__(self, host, port, path):
        self.host, self.port, self.path = host, port, path
        self.s = None; self.lock = threading.Lock()
        self.alive = False; self.fin = threading.Event()
    def _close_locked(self):
        self.alive = False
        if self.s:
            try: self.s.close()
            except Exception: pass
            self.s = None
    def connect(self):
        with self.lock:
            self._close_locked()
            try:
                s = socket.create_connection((self.host, self.port), 5)
                key = base64.b64encode(os.urandom(16)).decode()
                req = (f"GET {self.path} HTTP/1.1\r\nHost: {self.host}:{self.port}\r\nUpgrade: websocket\r\n"
                       f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
                s.sendall(req.encode()); s.settimeout(4)
                buf = b""
                while b"\r\n\r\n" not in buf:
                    c = s.recv(1024)
                    if not c: raise IOError("closed")
                    buf += c
                if b"101" not in buf.split(b"\r\n")[0]: raise IOError("bad handshake")
                s.settimeout(0.5); self.s = s; self.alive = True
                threading.Thread(target=self._reader, daemon=True).start()
                log("[ws] connected")
            except Exception as e:
                log("[ws] connect fail:", e); self.s = None; self.alive = False
    def _raw(self, op, payload=b""):
        h = bytearray([0x80 | op]); n = len(payload); mask = os.urandom(4)
        if n < 126: h.append(0x80 | n)
        elif n < 65536: h.append(0x80 | 126); h += struct.pack(">H", n)
        else: h.append(0x80 | 127); h += struct.pack(">Q", n)
        h += mask + bytes(payload[i] ^ mask[i % 4] for i in range(n))
        self.s.sendall(bytes(h))
    def _reader(self):
        s = self.s
        try:
            while self.alive and s is self.s:
                try: hdr = s.recv(2)
                except socket.timeout: continue
                except Exception as e: log("[ws] recv err:", e); break
                if len(hdr) < 2: break
                b1, b2 = hdr[0], hdr[1]; ln = b2 & 0x7f
                if ln == 126: ln = struct.unpack(">H", s.recv(2))[0]
                elif ln == 127: ln = struct.unpack(">Q", s.recv(8))[0]
                d = b""
                while len(d) < ln: d += s.recv(ln - len(d))
                op = b1 & 0x0f
                if op == 9:                      # 机器人 ping -> 回 pong
                    with self.lock:
                        if s is self.s:
                            try: self._raw(0xA, d)
                            except Exception: pass
                elif op == 8:
                    log("[ws] close frame"); break
                elif op == 1:
                    try: o = json.loads(d.decode("utf-8", "replace"))
                    except Exception: continue
                    bb = o.get("body", {})
                    if bb.get("name") == "chassis_move":
                        hh = o.get("header", {}); dd = bb.get("data", {}) or {}
                        ec = dd.get("error_code") or {}
                        if (hh.get("type") == "notify" and hh.get("cmd") == "finish") or (isinstance(ec, dict) and ec.get("code")):
                            self.fin.set()
        except Exception as e:
            log("[ws] reader err:", e)
        finally:
            with self.lock:
                if s is self.s:
                    self.alive = False; self.s = None
    def send(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode()
        for attempt in (0, 1):
            if not self.alive: self.connect()
            if not self.alive: return False
            try:
                with self.lock: self._raw(0x1, data)
                return True
            except Exception as e:
                log("[ws] send err:", e)
                with self.lock: self._close_locked()
        return False
    def wait_finish(self, timeout=2.2):
        """读线程收到 chassis_move finish/错误时置位 fin。"""
        self.fin.clear(); self.fin.wait(timeout)

ws = WS(ROBOT_HOST, ROBOT_PORT, "/robot-ws/")
_uid = [0]
def cmd(name, data=None):
    _uid[0] += 1
    return ws.send({"header": {"mode": "mission", "type": "request", "cmd": "start",
                               "ts": int(time.time()), "uuid": "joy-%d" % _uid[0], "host": ""},
                    "body": {"name": name, "task_id": "joy-%s-%d" % (name, int(time.time()*1000)), "data": data or {}}})

def main():
    import serial
    ws.connect(); cmd("stop_all")
    log("[relay] robot=%s:%s <- %s@%d" % (ROBOT_HOST, ROBOT_PORT, SER, BAUD))
    ser = serial.Serial(); ser.port = SER; ser.baudrate = BAUD; ser.timeout = 0.1
    ser.dtr = False; ser.rts = False
    try: ser.open(); ser.reset_input_buffer()
    except Exception as e: log("[serial] open fail:", e); return

    moving = False; last_tx = 0.0; cnt = 0; last_log = time.time(); last_ka = 0.0
    prev_b = 0; last_click = 0.0; pend_single = 0.0; cur_h = 0.0
    def hgt(delta):
        nonlocal cur_h
        cur_h = max(0.0, min(0.44, round(cur_h + delta, 3)))
        cmd("robot_height_ctrl", {"value": cur_h})
        log("[height] %+.2f -> %.2f m" % (delta, cur_h))
    while True:
        try: raw = ser.readline()
        except Exception as e:
            log("[serial]", e, "-> reopen")
            try: ser.close()
            except Exception: pass
            time.sleep(0.5)
            try: ser.open(); ser.reset_input_buffer(); log("[serial] reopened")
            except Exception as e2: log("[serial] reopen fail:", e2); time.sleep(1.0)
            continue
        line = raw.decode(errors="replace").strip()
        if not line.startswith("J,"): continue
        try:
            _, xs, ys, bs = line.split(",")[:4]; x = int(xs); y = int(ys); b = int(bs)
        except Exception: continue
        cnt += 1; now = time.time()

        if now - last_ka > 8.0:
            ws.send({"header": {"mode": "event", "type": "request", "cmd": "oneshot", "ts": int(time.time()), "uuid": "joy-ka", "host": ""},
                     "body": {"name": "robot_info", "task_id": "joy-ka", "data": {"topics": ["pos"]}}}); last_ka = now
        if now - last_log > 3.0:
            log("[in] x=%d y=%d b=%d moving=%s n=%d" % (x, y, b, moving, cnt)); last_log = now

        if b == 1 and prev_b == 0:                 # SW 按下沿
            if now - last_click < 0.45:            # 双击 -> 降低
                pend_single = 0.0; last_click = 0.0; hgt(-HSTEP)
            else:                                  # 可能是单击 -> 等 0.45s
                last_click = now; pend_single = now + 0.45
        if pend_single and now >= pend_single:     # 单击 -> 升高
            pend_single = 0.0; last_click = 0.0; hgt(HSTEP)
        prev_b = b
        if b == 1:
            continue

        fwd = y < -DZ; back = y > DZ; lft = x < -DZ; rgt = x > DZ
        want = fwd or back or lft or rgt
        if want and (now - last_tx >= GAP):
            dist = 0.0; ang = 0.0
            if fwd:  dist = round(MAXD * min(1.0, abs(y) / 127.0), 3)
            if back: dist = -round(MAXD * min(1.0, abs(y) / 127.0), 3)
            if lft:  ang = round(MAXA * min(1.0, abs(x) / 127.0), 4)
            if rgt:  ang = -round(MAXA * min(1.0, abs(x) / 127.0), 4)
            cmd("chassis_move", {"move_distance": dist, "move_angle": ang}); moving = True
            log("[tx] d=%.2f a=%.2f (x=%d y=%d)" % (dist, ang, x, y))
            ws.wait_finish(2.2)
            last_tx = time.time()
            try: ser.reset_input_buffer()        # 丢弃等待期间堆积的旧值 -> 取最新
            except Exception: pass
        elif not want and moving:
            cmd("stop_all"); moving = False; last_tx = now; log("[stop] centered -> stop_all")

if __name__ == "__main__":
    main()
