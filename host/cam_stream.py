#!/usr/bin/env python3
"""把 USB 摄像头(/dev/video0, 拍无人机遥控屏) 变成 MJPEG 流，供控制台嵌网页。
单线程采集 + 多客户端广播。  GET /stream (或 /) -> multipart MJPEG。
环境: CAM_DEV CAM_PORT CAM_W CAM_H CAM_Q"""
import os, time, threading, subprocess, json
import cv2
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

DEV = os.environ.get("CAM_DEV", "/dev/video0")
PORT = int(os.environ.get("CAM_PORT", "8091"))
W = int(os.environ.get("CAM_W", "1280"))
Hh = int(os.environ.get("CAM_H", "720"))
Q = int(os.environ.get("CAM_Q", "72"))

latest = [None]; lock = threading.Lock()
osd = [{"texts": [], "ts": 0.0}]
OCR_EVERY = float(os.environ.get("OCR_EVERY", "2.5"))

def cap_loop():
    cap = cv2.VideoCapture(DEV)
    try: cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Hh); cap.set(cv2.CAP_PROP_FPS, 20)
    except Exception: pass
    while True:
        ok, fr = cap.read()
        if not ok or fr is None:
            time.sleep(0.2)
            try: cap.release()
            except Exception: pass
            time.sleep(0.5); cap = cv2.VideoCapture(DEV)
            continue
        ok, jpg = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, Q])
        if ok:
            with lock: latest[0] = jpg.tobytes()

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/stream", "/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    with lock: b = latest[0]
                    if b:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(b)).encode() + b"\r\n\r\n" + b + b"\r\n")
                    time.sleep(0.05)
            except Exception: pass
            return
        if p == "/snap.jpg":
            with lock: b = latest[0]
            if b:
                self.send_response(200); self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(b))); self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers(); self.wfile.write(b); return
            self.send_error(503, "no frame"); return
        if p == "/focus":
            q = parse_qs(urlparse(self.path).query)
            try:
                if "auto" in q:
                    if q["auto"][0] == "1":
                        subprocess.run(["v4l2-ctl", "-d", DEV, "-c", "focus_automatic_continuous=1"], timeout=3, capture_output=True)
                    else:
                        v = int(float(q.get("val", ["0"])[0])); v = max(0, min(250, v))
                        subprocess.run(["v4l2-ctl", "-d", DEV, "-c", "focus_automatic_continuous=0"], timeout=3, capture_output=True)
                        subprocess.run(["v4l2-ctl", "-d", DEV, "-c", "focus_absolute=%d" % v], timeout=3, capture_output=True)
                out = subprocess.run(["v4l2-ctl", "-d", DEV, "-C", "focus_automatic_continuous,focus_absolute"], timeout=3, capture_output=True, text=True).stdout
            except Exception as e:
                out = "err: %s" % e
            body = out.encode()
            self.send_response(200); self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
        if p == "/osd.json":
            with lock: o = osd[0]
            body = json.dumps({"ts": o["ts"], "texts": o["texts"]}, ensure_ascii=False).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
        self.send_error(404)

def ocr_loop():
    try:
        from rapidocr_onnxruntime import RapidOCR
        import numpy as np
    except Exception as e:
        print("[ocr] 不可用:", e, flush=True); return
    eng = RapidOCR()
    print("[ocr] ready", flush=True)
    while True:
        with lock: b = latest[0]
        if b:
            try:
                arr = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
                res, _ = eng(arr)
                texts = [t for _, t, s in (res or []) if s is None or s > 0.4]
                with lock: osd[0] = {"texts": texts, "ts": time.time()}
            except Exception as e:
                print("[ocr] err", e, flush=True)
        time.sleep(OCR_EVERY)

if __name__ == "__main__":
    threading.Thread(target=cap_loop, daemon=True).start()
    threading.Thread(target=ocr_loop, daemon=True).start()
    print("[cam] %s -> MJPEG http://0.0.0.0:%d/stream  (%dx%d q%d)" % (DEV, PORT, W, Hh, Q), flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
