#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_client.py — 八界机器人最小客户端（仅 Python 标准库，无第三方依赖）

两种接入：
  RobotHTTP   : Jetson 关节桥 (8796) + 指挥服务 (8792)
  RobotMission: 机器人 Mission WebSocket (17890) 直连

示例：
  from robot_client import RobotHTTP, RobotMission
  h = RobotHTTP("127.0.0.1")
  print(h.health(), h.joints()["deg"])
  print(h.jog([0.2,2.6,2.0,-0.3,-1.0,0.0]))
  print(h.stop())

  m = RobotMission("192.168.1.100")
  print(m.oneshot(["battery", "alarm", "pos"]))
  print(m.mission("robot_height_ctrl", {"value": 0.2}))
  m.close()

注意：运动前确认周围安全；stop() 只停 jog 流，整机停止请同时发
RobotHTTP.cmd({"action":"stop"}) 或 RobotMission.mission("stop_all", {})。
"""
import base64
import json
import os
import socket
import struct
import time
import urllib.request


class RobotHTTP:
    """Jetson 桥接 HTTP API（默认 8796）。"""

    def __init__(self, host="127.0.0.1", port=8796, timeout=5):
        self.base = "http://%s:%d" % (host, int(port))
        self.timeout = timeout

    def _req(self, path, method="GET", obj=None, timeout=None):
        data = None
        headers = {}
        if obj is not None:
            data = json.dumps(obj).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(self.base + path, data=data,
                                     headers=headers, method=method)
        with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
            return json.load(r)

    def health(self):
        return self._req("/health")

    def joints(self):
        return self._req("/joints")

    def joglog(self):
        return self._req("/joglog")

    def set_joints(self, rad, timeout=30):
        """一次性 6 轴目标（带桥内路径防撞）。"""
        return self._req("/set", "POST", {"rad": [float(x) for x in rad]}, timeout)

    def jog(self, rad):
        """流式目标；持续更新即可平滑跟随。"""
        return self._req("/jog", "POST", {"rad": [float(x) for x in rad]})

    def stop(self):
        """停止 jog 流并保持当前位置。"""
        return self._req("/jog", "POST", {"stop": True})

    def grip(self, v, timeout=30):
        """夹爪 0.0(闭) ~ 0.69(开)。"""
        return self._req("/grip", "POST", {"v": float(v)}, timeout)

    def mode(self, mode_id="std", **kw):
        """标准/慢速/柔动/波浪；参数可覆盖。"""
        preset = {
            "std":  {"label": "标准", "vmax": 2.0, "amax": 6.0},
            "slow": {"label": "慢速", "vmax": 0.8, "amax": 3.0},
            "dance": {"label": "柔动", "vmax": 2.0, "amax": 6.0, "dance": 1},
            "wave": {"label": "波浪", "vmax": 2.0, "amax": 6.0, "dance": 1, "wave": 1,
                     "damp": 0.7, "dperiod": 7.0},
            "headfast": {"label": "定点头快速", "vmax": 1.6, "amax": 4.5},
        }
        m = {"id": mode_id, "vmin": 0.15, "dead": 0.02, "damp": 1.0, "dperiod": 7.0}
        m.update(preset.get(mode_id, {}))
        m.update(kw)
        return self._req("/mode", "POST", {"mode": m})

    def cmd(self, payload, timeout=10):
        """高层动作服务（:8792）：{"action":"height","value":0.3} 或 {"text":"升高"}。"""
        base = self.base.rsplit(":", 1)[0] + ":8792"
        data = json.dumps(payload).encode()
        req = urllib.request.Request(base + "/cmd", data=data,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)

    def state(self, timeout=5):
        base = self.base.rsplit(":", 1)[0] + ":8792"
        with urllib.request.urlopen(base + "/state", timeout=timeout) as r:
            return json.load(r)


class RobotMission:
    """机器人 Mission WebSocket（默认 ws://192.168.1.100:17890/robot-ws/）。"""

    def __init__(self, host="192.168.1.100", port=17890, path="/robot-ws/", timeout=4):
        self.host, self.port, self.path = host, int(port), path
        self.timeout = timeout
        self.sock = None
        self._id = 0

    def connect(self, timeout=5):
        s = socket.create_connection((self.host, self.port), timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = ("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n" % (self.path, self.host, self.port, key))
        s.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            c = s.recv(4096)
            if not c:
                raise IOError("websocket handshake failed")
            buf += c
        if b" 101 " not in buf.split(b"\r\n", 1)[0]:
            raise IOError("websocket upgrade rejected: " + buf[:120].decode("latin1", "replace"))
        s.settimeout(timeout)
        self.sock = s
        return self

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _send(self, obj):
        if not self.sock:
            self.connect()
        data = json.dumps(obj, ensure_ascii=False).encode()
        mask = os.urandom(4)
        hdr = bytearray([0x81])  # FIN + text
        n = len(data)
        if n < 126:
            hdr.append(0x80 | n)
        elif n < 65536:
            hdr.append(0x80 | 126)
            hdr += struct.pack(">H", n)
        else:
            hdr.append(0x80 | 127)
            hdr += struct.pack(">Q", n)
        payload = bytes(data[i] ^ mask[i % 4] for i in range(n))
        self.sock.sendall(bytes(hdr) + mask + payload)

    def _recv(self, timeout=None):
        if not self.sock:
            return None
        self.sock.settimeout(timeout or self.timeout)
        try:
            h = self.sock.recv(2)
            if not h or len(h) < 2:
                return None
            op = h[0] & 0x0F
            ln = h[1] & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self.sock.recv(2))[0]
            elif ln == 127:
                ln = struct.unpack(">Q", self.sock.recv(8))[0]
            masked = bool(h[1] & 0x80)
            mask = self.sock.recv(4) if masked else b""
            data = b""
            while len(data) < ln:
                c = self.sock.recv(ln - len(data))
                if not c:
                    break
                data += c
            if masked:
                data = bytes(data[i] ^ mask[i % 4] for i in range(len(data)))
            if op == 8:  # close
                self.close()
                return None
            if op in (9, 10):  # ping/pong
                return self._recv(timeout)
            return data.decode("utf-8", "replace") if op == 1 else None
        except socket.timeout:
            return None

    def _header(self, mode, cmd):
        self._id += 1
        return {"mode": mode, "type": "request", "cmd": cmd,
                "ts": int(time.time()), "uuid": "py-%d" % self._id, "host": ""}

    def oneshot(self, topics=("battery", "alarm", "pos"), timeout=3.0):
        """一次性读 robot_info 主题，返回合并后的 dict。"""
        self._send({"header": self._header("event", "oneshot"),
                    "body": {"name": "robot_info", "task_id": "oneshot-%d" % self._id,
                             "data": {"topics": list(topics)}}})
        out = {}
        t0 = time.time()
        while time.time() - t0 < timeout:
            txt = self._recv(min(0.6, timeout))
            if not txt:
                continue
            try:
                o = json.loads(txt)
            except Exception:
                continue
            body = o.get("body") or {}
            if body.get("name") == "robot_info":
                out.update(body.get("data") or {})
        return out

    def mission(self, name, data=None, timeout=4.0, want=None):
        """发 mission/start，等待同名响应；want(data)->bool 可过滤。"""
        self._send({"header": self._header("mission", "start"),
                    "body": {"name": name, "task_id": "mission-%d" % self._id,
                             "data": data or {}}})
        t0 = time.time()
        while time.time() - t0 < timeout:
            txt = self._recv(min(0.6, timeout))
            if not txt:
                continue
            try:
                o = json.loads(txt)
            except Exception:
                continue
            body = o.get("body") or {}
            if body.get("name") == name:
                d = body.get("data") or {}
                if want is None or want(d):
                    return d
        return None

    def speech(self, text, timeout=4.0):
        """App 级语音播报。"""
        self._send({"header": self._header("app", "start"),
                    "body": {"name": "speech", "task_id": "speech-%d" % self._id,
                             "data": {"text": text, "code": 0}}})
        return True


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="八界机器人最小客户端自检（只读）")
    ap.add_argument("--jetson", default="127.0.0.1")
    ap.add_argument("--robot", default="192.168.1.100")
    a = ap.parse_args()
    h = RobotHTTP(a.jetson)
    print("HTTP health:", h.health())
    j = h.joints()
    print("HTTP joints:", j.get("deg"), "lift", j.get("lift"), "online", j.get("robot_online"))
    m = RobotMission(a.robot)
    print("Mission info:", m.oneshot(["battery", "alarm", "pos"]))
    m.close()
