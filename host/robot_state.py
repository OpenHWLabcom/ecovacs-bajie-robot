#!/usr/bin/env python3
# 读八界机器人 battery/alarm/charge/pos（robot-ws oneshot），输出一行 JSON。供监测用。
import os, json, socket, base64, struct, time
HOST = os.environ.get("ROBOT_HOST", "192.168.1.100"); PORT = 17890
def main():
    try:
        s = socket.create_connection((HOST, PORT), 5)
        k = base64.b64encode(os.urandom(16)).decode()
        s.sendall((f"GET /robot-ws/ HTTP/1.1\r\nHost: {HOST}:{PORT}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {k}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        b = b""
        while b"\r\n\r\n" not in b: b += s.recv(4096)
        def tx(o):
            d = json.dumps(o).encode(); m = os.urandom(4); n = len(d); h = bytearray([0x81])
            if n < 126: h.append(0x80 | n)
            elif n < 65536: h.append(0x80 | 126); h += struct.pack(">H", n)
            else: h.append(0x80 | 127); h += struct.pack(">Q", n)
            h += m + bytes(d[i] ^ m[i % 4] for i in range(n)); s.sendall(bytes(h))
        tx({"header": {"mode": "event", "type": "request", "cmd": "oneshot", "ts": int(time.time()), "uuid": "rs", "host": ""},
            "body": {"name": "robot_info", "task_id": "rs", "data": {"topics": ["battery", "alarm", "workState", "pos"]}}})
        s.settimeout(6); out = {}; t = time.time()
        while time.time() - t < 8:
            try:
                h = s.recv(2)
                if not h: break
                ln = h[1] & 0x7f
                if ln == 126: ln = struct.unpack(">H", s.recv(2))[0]
                elif ln == 127: ln = struct.unpack(">Q", s.recv(8))[0]
                p = b""
                while len(p) < ln: p += s.recv(ln - len(p))
                d = json.loads(p.decode("utf-8", "replace")).get("body", {}).get("data", {})
                for kk in ("battery", "alarm", "workState", "pos"):
                    if kk in d: out[kk] = d[kk]
                if "battery" in out and "alarm" in out: break
            except socket.timeout: break
        print(json.dumps(out, ensure_ascii=False))
    except Exception as e:
        print(json.dumps({"err": str(e)}))
if __name__ == "__main__": main()
