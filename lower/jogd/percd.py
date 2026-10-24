#!/usr/bin/env python3
"""percd.py v2 — 深度识别可视化（合成画面：左侧放大点云俯视图 + 右侧识别画面 + 流程条）。
持续：抓 RGB-D → base 系点云 → 去地面 → 聚类 → minAreaRect → 抓取点 → 合成标注图
输出：stdout 长度前缀 JPEG（供 Jetson percfeed :8799 转 MJPEG）
参数：PERC_FPS PERC_Q
"""
import os, sys, time, struct, math
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("CYCLONEDDS_URI", "file:///home/Agi/jogd/dds_hi.xml")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
import cv2
import numpy as np

FPS = float(os.environ.get("PERC_FPS", "0.9"))
Q = int(os.environ.get("PERC_Q", "78"))
TOPIC_C = os.environ.get("PERC_RGB", "/berxel_upper/berxel_upper/color/image_raw")
TOPIC_D = os.environ.get("PERC_DEPTH", "/berxel_upper/berxel_upper/aligned_depth_to_color/image_raw")
TOPIC_I = os.environ.get("PERC_INFO", "/berxel_upper/berxel_upper/color/camera_info")

rclpy.init()
n = Node("percd")
got = {}
def _cb(k):
    def f(m):
        got[k] = m
    return f
n.create_subscription(Image, TOPIC_C, _cb("rgb"), 5)
n.create_subscription(Image, TOPIC_D, _cb("depth"), 5)
n.create_subscription(CameraInfo, TOPIC_I, _cb("info"), 5)
buf = tf2_ros.Buffer()
lst = tf2_ros.TransformListener(buf, n)

Tcache = {"R": None, "t": None, "ts": 0.0}

def cam_pose():
    now = time.time()
    if Tcache["R"] is not None and now - Tcache["ts"] < 60:
        return Tcache["R"], Tcache["t"]
    rgb = got.get("rgb")
    if rgb is None:
        return None, None
    opt = rgb.header.frame_id.replace("rgb_frame", "rgb_optical_frame")
    for _ in range(8):
        try:
            tr = buf.lookup_transform("base_link", opt, rclpy.time.Time(),
                                      timeout=rclpy.duration.Duration(seconds=0.4))
            tq = tr.transform.translation
            q = tr.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            Tcache.update(R=R, t=np.array([tq.x, tq.y, tq.z]), ts=now)
            return R, Tcache["t"]
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.1)
    return None, None

def spin_until_ready(timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        if "rgb" in got and "depth" in got and "info" in got and got["rgb"] is not None:
            return True
    return False

def project(R, t, K, p):
    fx, fy, cx, cy = K
    pc = R.T @ (np.asarray(p) - t)
    if pc[2] <= 0.05:
        return None
    return (int(round(fx * pc[0] / pc[2] + cx)), int(round(fy * pc[1] / pc[2] + cy)))

# ---- 点云俯视图映射 ----
MAP_W, MAP_H = 400, 400
X0, X1 = 0.10, 1.30        # 前向范围
Y0, Y1 = -0.60, 0.60       # 左右范围
def map_xy(px, py):
    u = (px - X0) / (X1 - X0) * (MAP_W - 1)
    v = (1.0 - (py - Y0) / (Y1 - Y0)) * (MAP_H - 1)
    return u.astype(np.int32), v.astype(np.int32)

def draw_cloud_map(Pb):
    m = np.zeros((MAP_H, MAP_W, 3), np.uint8)
    m[:] = (10, 18, 26)
    # 网格 10cm
    for x in np.arange(0.2, 1.31, 0.1):
        u = int((x - X0) / (X1 - X0) * (MAP_W - 1))
        cv2.line(m, (u, 0), (u, MAP_H - 1), (24, 44, 52), 1)
    for y in np.arange(-0.5, 0.51, 0.1):
        v = int((1.0 - (y - Y0) / (Y1 - Y0)) * (MAP_H - 1))
        cv2.line(m, (0, v), (MAP_W - 1, v), (24, 44, 52), 1)
    if len(Pb):
        z = Pb[:, 2]
        zmin, zmax = -0.06, 0.45
        t = np.clip((z - zmin) / (zmax - zmin), 0, 1)
        col = cv2.applyColorMap((t * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
        u, v = map_xy(Pb[:, 0], Pb[:, 1])
        ok = (u >= 0) & (u < MAP_W) & (v >= 0) & (v < MAP_H)
        for k in range(0, 3):
            m[v[ok], u[ok], k] = col[ok, k]
        # 稍微加粗
        u2 = np.clip(u[ok] + 1, 0, MAP_W - 1); v2 = v[ok]
        for k in range(0, 3):
            m[v2, u2, k] = col[ok, k]
    cv2.putText(m, "POINT CLOUD (top view)", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 230, 250), 1, cv2.LINE_AA)
    cv2.putText(m, "x: 0.1m(bottom) -> 1.3m(top)", (6, MAP_H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (110, 170, 190), 1, cv2.LINE_AA)
    cv2.putText(m, "y: +left", (6, MAP_H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (110, 170, 190), 1, cv2.LINE_AA)
    return m

out = sys.stdout.buffer
t_last = 0.0
log = lambda s: print(s, file=sys.stderr, flush=True)
CW, CH = 740, 420

while rclpy.ok():
    if not spin_until_ready(6.0):
        time.sleep(0.5)
        continue
    rgb = got["rgb"]; dep = got["depth"]; info = got["info"]
    img = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
    bgr = img.copy() if rgb.encoding.lower() == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    D = np.frombuffer(bytes(dep.data), dtype=np.uint16).reshape(dep.height, dep.width).astype(np.float32) * 0.001
    K = (info.k[0], info.k[4], info.k[2], info.k[5])
    R, t = cam_pose()
    H, W = D.shape
    t0 = time.time()
    npts = nfloor = nclus = 0
    ok_obj = False
    info_lines = []
    Pb = np.zeros((0, 3), np.float32)
    rect3d = None
    if R is not None:
        step = 3
        vv, uu = np.mgrid[0:H:step, 0:W:step]
        ZZ = D[0:H:step, 0:W:step]
        m = (ZZ > 0.2) & (ZZ < 3.0) & np.isfinite(ZZ)
        u = uu[m]; v = vv[m]; zz = ZZ[m]
        npts = len(u)
        X = (u - K[2]) * zz / K[0]
        Y = (v - K[3]) * zz / K[1]
        P = np.stack([X, Y, zz], 1)
        Pb = (R @ P.T).T + t
        zb = Pb[:, 2]
        floor = float(np.percentile(zb[zb < -0.02], 60)) if np.any(zb < -0.02) else -0.06
        sel = (zb > floor + 0.004) & (zb < floor + 0.30) & (Pb[:, 0] > 0.25) & (Pb[:, 0] < 1.35) & (np.abs(Pb[:, 1]) < 0.75)
        nfloor = int(sel.sum())
        if nfloor > 40:
            Pc = Pb[sel]; uc = u[sel].astype(int); vc = v[sel].astype(int)
            gx = np.floor(Pc[:, 0] / 0.035).astype(int)
            gy = np.floor(Pc[:, 1] / 0.035).astype(int)
            keys = list(zip(gx.tolist(), gy.tolist()))
            keyset = set(keys)
            from collections import Counter
            seed = Counter(keys).most_common(1)[0][0]
            seen = {seed}; stack = [seed]
            while stack:
                kk = stack.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (kk[0] + dx, kk[1] + dy)
                        if nb in keyset and nb not in seen:
                            seen.add(nb); stack.append(nb)
            m2 = np.array([kk in seen for kk in keys])
            Qc = Pc[m2]; qu = uc[m2]; qv = vc[m2]
            nclus = len(Qc)
            if nclus > 60:
                ok_obj = True
                (cx, cy), (w, h), ang = cv2.minAreaRect(Qc[:, :2].astype(np.float32))
                if w < h:
                    w, h, ang = h, w, ang + 90
                # 分割点高亮
                ov = bgr[v, u]
                ov[sel] = (0.35 * ov[sel] + 0.65 * np.array([80, 220, 160])).astype(np.uint8)
                bgr[v, u] = ov
                rect_px = cv2.minAreaRect(np.stack([qu, qv], 1).astype(np.float32))
                boxp = cv2.boxPoints(rect_px).astype(int)
                cv2.polylines(bgr, [boxp], True, (80, 220, 160), 2)
                bgr = cv2.resize(bgr, (320, 200), interpolation=cv2.INTER_AREA)
                ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
                rect3d = np.array([[cx + (ca * w / 2 + sa * h / 2), cy + (sa * w / 2 - ca * h / 2)],
                                   [cx + (ca * w / 2 - sa * h / 2), cy + (sa * w / 2 + ca * h / 2)],
                                   [cx - (ca * w / 2 - sa * h / 2), cy - (sa * w / 2 + ca * h / 2)],
                                   [cx - (ca * w / 2 + sa * h / 2), cy - (sa * w / 2 + ca * h / 2)]])
                info_lines = [
                    "OBJ %.0fx%.0fcm  yaw %.0f deg" % (w * 100, h * 100, ang),
                    "z %.0f~%.0fcm above floor" % ((Qc[:, 2].min() - floor) * 100, (Qc[:, 2].max() - floor) * 100),
                    "pts %d" % nclus,
                ]
    if not ok_obj:
        bgr = cv2.resize(bgr, (320, 200), interpolation=cv2.INTER_AREA)
        cv2.putText(bgr, "NO OBJECT", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 160, 180), 2, cv2.LINE_AA)

    cmap = draw_cloud_map(Pb)
    if rect3d is not None:
        pts = []
        for pxy in rect3d:
            uu2, vv2 = map_xy(np.array([pxy[0]]), np.array([pxy[1]]))
            pts.append([int(uu2[0]), int(vv2[0])])
        cv2.polylines(cmap, [np.array(pts)], True, (80, 240, 170), 2)
    canvas = np.zeros((CH, CW, 3), np.uint8)
    canvas[:] = (9, 16, 24)
    canvas[0:MAP_H, 0:MAP_W] = cmap
    canvas[0:200, 405:725] = bgr
    cv2.rectangle(canvas, (405, 0), (725, 200), (60, 120, 150), 1)
    cv2.putText(canvas, "DEPTH RECOGNITION", (409, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 230, 250), 1, cv2.LINE_AA)
    for i, ln in enumerate(info_lines):
        cv2.putText(canvas, ln, (409, 228 + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 230, 180), 1, cv2.LINE_AA)
    steps = "cloud %d | floor-removed %d | cluster %d" % (npts, nfloor, nclus)
    cv2.putText(canvas, steps, (409, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 210, 230), 1, cv2.LINE_AA)
    cv2.putText(canvas, "1 RGB-D > 2 cloud > 3 floor > 4 cluster", (409, 342), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (110, 170, 190), 1, cv2.LINE_AA)
    cv2.putText(canvas, "5 rect fit > 6 pose/grasp", (409, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (110, 170, 190), 1, cv2.LINE_AA)
    cv2.putText(canvas, "%.2fs" % (time.time() - t0), (W - 0, 0) if False else (690, 396), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 180, 200), 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (0, MAP_H), (CW - 1, CH - 1), (12, 22, 30), -1)
    cv2.putText(canvas, "PIPELINE: 1 RGB-D -> 2 cloud -> 3 floor removal -> 4 cluster -> 5 rect -> 6 grasp pose", (8, CH - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (150, 220, 240), 1, cv2.LINE_AA)

    ok, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, Q])
    if ok:
        d = jpg.tobytes()
        try:
            out.write(struct.pack(">I", len(d)) + d)
            out.flush()
        except (BrokenPipeError, KeyboardInterrupt):
            break
    log("perc obj=%s pts=%d cluster=%d dt=%.2fs" % (ok_obj, npts, nclus, time.time() - t0))
    dt = time.time() - t_last
    if dt < 1.0 / FPS:
        time.sleep(1.0 / FPS - dt)
    t_last = time.time()
