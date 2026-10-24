#!/usr/bin/env python3
"""percg.py — 爪子相机识别检测可视化（berxel_arm，相机系近景分割 + 夹爪状态）。
输出：stdout 长度前缀 JPEG（供 gripperfeed :8800 转 MJPEG）。
参数：GRIP_FPS GRIP_Q GRIP_W GRIP_H
"""
import os, sys, time, struct
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("CYCLONEDDS_URI", "file:///factory/default.xml")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
import cv2
import numpy as np

FPS = float(os.environ.get("GRIP_FPS", "6"))
Q = int(os.environ.get("GRIP_Q", "70"))
OW = int(os.environ.get("GRIP_W", "480"))
OH = int(os.environ.get("GRIP_H", "300"))
TOPIC_C = os.environ.get("GRIP_RGB", "/berxel_arm/berxel_arm/color/image_raw")
TOPIC_D = os.environ.get("GRIP_DEPTH", "/berxel_arm/berxel_arm/aligned_depth_to_color/image_raw")
TOPIC_I = os.environ.get("GRIP_INFO", "/berxel_arm/berxel_arm/color/camera_info")

rclpy.init()
n = Node("percg")
got = {}
def _cb(k):
    def f(m):
        got[k] = m
    return f
n.create_subscription(Image, TOPIC_C, _cb("rgb"), 5)
n.create_subscription(Image, TOPIC_D, _cb("depth"), 5)
n.create_subscription(CameraInfo, TOPIC_I, _cb("info"), 5)
fin = {"v": None}
def on_js(m):
    d = dict(zip(m.name, m.position))
    if "arm_finger_joint1" in d:
        fin["v"] = float(d["arm_finger_joint1"])
n.create_subscription(JointState, "/joint_states", on_js, 50)

out = sys.stdout.buffer
t_last = 0.0
log = lambda s: print(s, file=sys.stderr, flush=True)

def ready(t=6.0):
    t0 = time.time()
    while time.time() - t0 < t:
        rclpy.spin_once(n, timeout_sec=0.05)
        if "rgb" in got and "depth" in got and "info" in got:
            return True
    return False

while rclpy.ok():
    if not ready():
        time.sleep(0.4)
        continue
    rgb = got["rgb"]; dep = got["depth"]; info = got["info"]
    H, W = dep.height, dep.width
    img = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(H, W, 3)
    bgr = img.copy() if rgb.encoding.lower() == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    D = np.frombuffer(bytes(dep.data), dtype=np.uint16).reshape(H, W).astype(np.float32) * 0.001
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]

    # 近景分割：0.10~0.65m
    m = (D > 0.10) & (D < 0.65) & np.isfinite(D)
    found = False
    if m.sum() > 200:
        vv, uu = np.nonzero(m)
        zz = D[vv, uu]
        # 只看中心 60% 区域（爪子前方）
        c0x, c1x = int(W * 0.20), int(W * 0.80)
        c0y, c1y = int(H * 0.15), int(H * 0.90)
        k = (uu >= c0x) & (uu <= c1x) & (vv >= c0y) & (vv <= c1y)
        uu, vv, zz = uu[k], vv[k], zz[k]
        if len(zz) > 150:
            # 最近 30% 的点当物体
            zthr = np.percentile(zz, 35)
            sel = zz <= zthr
            su, sv, sz = uu[sel], vv[sel], zz[sel]
            x0, x1 = int(su.min()), int(su.max())
            y0, y1 = int(sv.min()), int(sv.max())
            if (x1 - x0) > 8 and (y1 - y0) > 8:
                found = True
                cv2.rectangle(bgr, (x0, y0), (x1, y1), (80, 220, 160), 2)
                cv2.putText(bgr, "OBJ %.0fcm" % (np.median(sz) * 100), (x0 + 4, max(14, y0 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 240, 180), 2, cv2.LINE_AA)
                # 物体中心十字
                muc, mvc = int(np.median(su)), int(np.median(sv))
                cv2.line(bgr, (muc - 8, mvc), (muc + 8, mvc), (0, 240, 200), 1)
                cv2.line(bgr, (muc, mvc - 8), (muc, mvc + 8), (0, 240, 200), 1)
    cv2.putText(bgr, "GRIPPER CAM · detection", (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 230, 250), 1, cv2.LINE_AA)
    if not found:
        cv2.putText(bgr, "no object in near range", (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 160, 180), 1, cv2.LINE_AA)
    # 夹爪状态条
    f = fin["v"]
    if f is not None:
        ratio = max(0.0, min(1.0, (f - 0.01) / 0.53))
        bx, by, bw, bh = 8, 22, 120, 10
        cv2.rectangle(bgr, (bx, by), (bx + bw, by + bh), (60, 80, 90), -1)
        cv2.rectangle(bgr, (bx, by), (bx + int(bw * ratio), by + bh), (80, 220, 160), -1)
        cv2.putText(bgr, "GRIP %.2f" % f, (bx + bw + 6, by + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 230, 200), 1, cv2.LINE_AA)

    if OW and OH:
        bgr = cv2.resize(bgr, (OW, OH), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, Q])
    if ok:
        d = jpg.tobytes()
        try:
            out.write(struct.pack(">I", len(d)) + d)
            out.flush()
        except (BrokenPipeError, KeyboardInterrupt):
            break
    dt = time.time() - t_last
    if dt < 1.0 / FPS:
        time.sleep(1.0 / FPS - dt)
    t_last = time.time()
