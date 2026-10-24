#!/usr/bin/env python3
"""robot-viz 视频转播：下板相机 → adb/ssh 管道 → HTTP MJPEG。
端口/主题/脚本都可用环境变量配置：
  FEED_PORT  端口（默认 8797）
  FEED_TOPIC ROS 图像话题（默认头部相机）
  FEED_SCRIPT 下板脚本（默认 camstream.py；识别可视化用 percd.py）
  /cam.mjpg  MJPEG流 · /snap.jpg 最新帧
帧格式: 4字节大端长度 + JPEG。断线自动重连。
"""
import os, time, struct, subprocess, threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

PORT = int(os.environ.get("FEED_PORT", "8797"))
TOPIC = os.environ.get("FEED_TOPIC", "/berxel_upper/berxel_upper/color/image_raw")
SCRIPT = os.environ.get("FEED_SCRIPT", "camstream.py")
ADB = os.environ.get("ADB", "adb")
DEV = "192.168.1.100:5555"

REMOTE = (
    "chroot /linux /bin/bash -lc 'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; "
    "exec sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "
    "-o ConnectTimeout=8 -o ServerAliveInterval=5 Agi@10.0.0.2 "
    "\"CAM_TOPIC=" + TOPIC + " CAM_SCRIPT=/home/Agi/jogd/" + SCRIPT + " bash /home/Agi/jogd/camfeed_remote.sh\"'"
)
CMD = [ADB, "-s", DEV, "shell", REMOTE]

latest = [None]
tstamp = [0.0]
lock = threading.Lock()


def reader():
    while True:
        try:
            p = subprocess.Popen(CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            while True:
                hdr = p.stdout.read(4)
                if not hdr or len(hdr) < 4:
                    break
                n = struct.unpack(">I", hdr)[0]
                if n <= 0 or n > 2_000_000:
                    break
                data = b""
                while len(data) < n:
                    chunk = p.stdout.read(n - len(data))
                    if not chunk:
                        break
                    data += chunk
                if len(data) < n:
                    break
                with lock:
                    latest[0] = data
                    tstamp[0] = time.time()
            try:
                p.kill()
            except Exception:
                pass
        except Exception:
            pass
        time.sleep(1.0)


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    def log_message(self, *a):
        pass

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/index.html"):
            with lock:
                ok = latest[0] is not None
                age = int((time.time() - tstamp[0]) * 1000) if ok else -1
            body = ("<html><body style='background:#0b1016;color:#bfe9ff;font:14px monospace'>"
                    "<h3>robot-viz feed :%d</h3><p>topic: %s · script: %s</p>"
                    "<img src='/cam.mjpg' width='640' style='max-width:100%%;border:1px solid #234'>"
                    "<p>status: %s · age %dms</p></body></html>"
                    % (PORT, TOPIC, SCRIPT, "OK" if ok else "waiting...", age)).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if p == "/snap.jpg":
            with lock:
                d = latest[0]
                ts = tstamp[0]
            if d is None:
                self.send_response(503)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(d)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Age-Ms", str(int((time.time() - ts) * 1000)))
            self.end_headers()
            self.wfile.write(d)
            return
        if p in ("/cam.mjpg", "/stream.mjpg", "/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    with lock:
                        d = latest[0]
                    if d:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(d)).encode() + b"\r\n\r\n" + d + b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.02)
            except Exception:
                pass
            return
        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=reader, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), H)
    srv.serve_forever()
