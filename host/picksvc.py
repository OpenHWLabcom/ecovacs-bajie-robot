#!/usr/bin/env python3
# picksvc.py — 常驻抓取服务（下板跑）
# 功能: 缓存点云 / 快速扫描 / 单轨迹运动 / 夹爪 / 手指读数
# HTTP: 0.0.0.0:9100   GET /state  GET /finger  GET /scan  POST /move {rad,vmax}  POST /grip {v}
import json, math, threading, time, sys, base64, urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, JointState, CameraInfo
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros

NAMES = ["joint%d" % i for i in range(1, 7)]
CLEAR_POSE_SIDE = [1.1, 1.2, 2.0, 0.0, -0.8, 0.0]   # 立柱低位时的侧摆A
CLEAR_POSE_HIGH = None  # ⚠️ 2026-10-18 高位避险位实测导致离线，暂停使用
CLEAR_POSE = CLEAR_POSE_SIDE
g = {"q": None, "finger": None, "depth": None, "info": None, "depth_ts": 0.0, "q_ts": 0.0,
     "adepth": None, "ainfo": None, "adepth_ts": 0.0, "info_last": None, "ainfo_last": None}
# 启动时加载上次已知内参（防止相机 info 订阅偶发丢失 → scan 报 no cloud）
try:
    _ci = json.load(open(INFO_FILE))
    g["info_last"] = tuple(_ci["upper"]) if _ci.get("upper") else None
    g["ainfo_last"] = tuple(_ci["arm"]) if _ci.get("arm") else None
except Exception:
    pass
lk = threading.Lock()
acl = {}
busy = {"move": False}
ag = {"running": False, "ok": None, "log": [], "t0": 0.0}
INFO_FILE = '/home/Agi/jogd/cam_info.json'

class Svc(Node):
    def __init__(self):
        super().__init__("picksvc")
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(Image, "/berxel_upper/berxel_upper/aligned_depth_to_color/image_raw", self.on_depth, 2)
        self.create_subscription(CameraInfo, "/berxel_upper/berxel_upper/color/camera_info", self.on_info, 2)
        self.create_subscription(Image, "/berxel_arm/berxel_arm/aligned_depth_to_color/image_raw", self.on_adepth, 2)
        self.create_subscription(CameraInfo, "/berxel_arm/berxel_arm/color/camera_info", self.on_ainfo, 2)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        acl["ac"] = self.ac
        self.gac = ActionClient(self, GripperCommand, "/hand_controller/gripper_cmd")
        acl["gac"] = self.gac
        try:
            from std_srvs.srv import SetBool
            acl["rst"] = self.create_client(SetBool, "/arm/error/reset")
        except Exception:
            acl["rst"] = None
    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        q = [d.get(n) for n in NAMES]
        with lk:
            if all(x is not None for x in q):
                g["q"] = [float(x) for x in q]; g["q_ts"] = time.time()
            if "arm_finger_joint1" in d:
                g["finger"] = float(d["arm_finger_joint1"])
    def on_depth(self, m):
        with lk:
            g["depth"] = (m.height, m.width, m.encoding, bytes(m.data)); g["depth_ts"] = time.time()
    def on_info(self, m):
        with lk:
            g["info"] = (m.k[0], m.k[4], m.k[2], m.k[5])
            g["info_last"] = g["info"]
            try:
                d = json.load(open(INFO_FILE))
            except Exception:
                d = {}
            d["upper"] = list(g["info"])
            try: json.dump(d, open(INFO_FILE, 'w'))
            except Exception: pass
    def on_adepth(self, m):
        with lk:
            g["adepth"] = (m.height, m.width, m.encoding, bytes(m.data)); g["adepth_ts"] = time.time()
    def on_ainfo(self, m):
        with lk:
            g["ainfo"] = (m.k[0], m.k[4], m.k[2], m.k[5])
            g["ainfo_last"] = g["ainfo"]
            try:
                d = json.load(open(INFO_FILE))
            except Exception:
                d = {}
            d["arm"] = list(g["ainfo"])
            try: json.dump(d, open(INFO_FILE, 'w'))
            except Exception: pass

node = None

def _rss():
    try:
        return int(open('/proc/self/statm').read().split()[1]) * 4096 // 1024
    except Exception:
        return -1

def cloud(timeout=3.0, key="depth", frame_list=("berxel_upper_rgb_optical_frame", "berxel_upper_rgb_frame"), zcam=None):
    """返回 base_link 点云 Nx3（缓存的最新深度帧）"""
    t0 = time.time()
    while True:
        with lk:
            dep = g[key]
        inf = (g["info"] or g.get("info_last")) if key == "depth" else (g["ainfo"] or g.get("ainfo_last"))
            
        ts = g["depth_ts"] if key == "depth" else g["adepth_ts"]
        if dep and inf and time.time() - ts < 2.0: break
        if time.time() - t0 > timeout: return None
        time.sleep(0.05)
    h, w, enc, data = dep
    fx, fy, cx, cy = inf
    print("cloud dbg: h=%s w=%s enc=%s len=%s rss=%sKB" % (h, w, enc, len(data), _rss()), flush=True)
    try:
        a = np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
    except Exception:
        return None
    z = a
    lo_z, hi_z = (0.05, 2.5) if zcam is None else zcam
    vs, us = np.nonzero((z > lo_z) & (z < hi_z))
    if len(us) < 200: return None
    print("cloud dbg: valid=%d rss=%sKB" % (len(us), _rss()), flush=True)
    zz = z[vs, us]
    xc = (us - cx) / fx * zz; yc = (vs - cy) / fy * zz
    pts = np.stack([xc, yc, zz], axis=1)[::2]
    print("cloud dbg: pts=%s rss=%sKB" % (pts.shape, _rss()), flush=True)
    try:
        tr = None
        for fr in frame_list:
            try:
                tr = node.tf_buf.lookup_transform("base_link", fr, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)); break
            except Exception as e:
                print("cloud tf err %s: %s" % (fr, str(e)[:120]), flush=True)
                tr = None
        if tr is None: return None
    except Exception:
        return None
    t = tr.transform.translation; r = tr.transform.rotation
    x, y, zz2, wq = r.x, r.y, r.z, r.w
    R = np.array([
        [1-2*(y*y+zz2*zz2), 2*(x*y-zz2*wq), 2*(x*zz2+y*wq)],
        [2*(x*y+zz2*wq), 1-2*(x*x+zz2*zz2), 2*(y*zz2-x*wq)],
        [2*(x*zz2-y*wq), 2*(y*zz2+x*wq), 1-2*(x*x+y*y)]])
    return pts @ R.T + np.array([t.x, t.y, t.z])

def scan_objects():
    return _objects(cloud(), xr=(0.25, 1.35))

def obj_in_tool(px, py, pz):
    """把 base 坐标点转到 objects_link（夹爪工具系）"""
    try:
        tr = node.tf_buf.lookup_transform("objects_link", "base_link", rclpy.time.Time(),
                                          timeout=rclpy.duration.Duration(seconds=0.5))
        t = tr.transform.translation; r = tr.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        p = R @ np.array([px, py, pz]) + np.array([t.x, t.y, t.z])
        return [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)]
    except Exception:
        return None

def wrist_objects():
    # 手腕视角常被目标占满 → 地面用固定参考值（头部扫描已给出），避免 floor 估计被目标污染
    with lk: q = list(g["q"]) if g["q"] else None
    return _objects(cloud(key="adepth", frame_list=("berxel_arm_rgb_optical_frame", "berxel_arm_rgb_frame"), zcam=(0.12, 0.7)), xr=(-0.6, 0.9), yr=0.9)

def _objects(P, xr=(0.25, 1.35), yr=0.9, floor_fixed=None):
    if P is None: return {"ok": False, "err": "no cloud"}
    print("objects dbg: P=%s rss=%sKB" % (len(P), _rss()), flush=True)
    if len(P) > 1000 and float(P[:, 2].max()) > 0.3:
        hist, edges = np.histogram(P[:, 2], bins=np.arange(0.0, 1.0, 0.01))
        top = np.argsort(hist)[-6:][::-1]
        print("zpeaks: " + ", ".join("%.2fm=%d" % (edges[i], hist[i]) for i in top), flush=True)
    m = (P[:, 0] > xr[0]) & (P[:, 0] < xr[1]) & (np.abs(P[:, 1]) < yr) & (P[:, 2] > -0.15) & (P[:, 2] < 0.9)
    P = P[m]
    if len(P) < 300: return {"ok": False, "err": "too few points", "n": int(len(P))}
    floor = float(floor_fixed) if floor_fixed is not None else float(np.percentile(P[:, 2], 15))
    table_mode = False
    if floor_fixed is None and len(P) > 5000 and float(P[:, 2].max()) > 0.3:
        # 桌面场景：找 z 方向最强水平面（桌面），只保留桌面以上的点
        zmax_all = float(P[:, 2].max())
        hist, edges = np.histogram(P[:, 2], bins=np.arange(0.20, min(1.20, zmax_all + 0.03), 0.01))
        pi = int(np.argmax(hist))
        cand = float(edges[pi] + 0.005)
        if hist[pi] > max(300, int(0.02 * len(P))):
            table_mode = True
            floor = cand
    H = P[P[:, 2] > floor + (0.012 if table_mode else 0.035)]
    if table_mode:
        print("objects dbg: TABLE mode z=%.3f H=%d" % (floor, len(H)), flush=True)
    print("objects dbg: floor=%.3f H=%d rss=%sKB" % (floor, len(H), _rss()), flush=True)
    if len(H):
        print("objects dbg: H x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] nan=%d rss=%sKB" % (
            float(np.nanmin(H[:, 0])), float(np.nanmax(H[:, 0])),
            float(np.nanmin(H[:, 1])), float(np.nanmax(H[:, 1])),
            float(np.nanmin(H[:, 2])), float(np.nanmax(H[:, 2])),
            int(np.isnan(H).sum()), _rss()), flush=True)
    if len(H) < 60: return {"ok": True, "floor": floor, "objects": []}
    # 简单网格聚类
    cell = 0.045
    key = np.floor(H[:, :2] / cell).astype(int)
    print("objects dbg: key=%s rss=%sKB" % (key.shape, _rss()), flush=True)
    from collections import defaultdict
    buckets = defaultdict(list)
    for i, k in enumerate(zip(key[:, 0].tolist(), key[:, 1].tolist())): buckets[k].append(i)
    print("objects dbg: buckets=%d maxb=%d rss=%sKB" % (len(buckets), max(len(v) for v in buckets.values()), _rss()), flush=True)
    seen = set(); groups = []
    for k in buckets:
        if k in seen: continue
        stack = [k]; idx = []
        seen.add(k)
        while stack:
            kk = stack.pop(); idx += buckets[kk]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (kk[0]+dx, kk[1]+dy)
                    if nb in buckets and nb not in seen: seen.add(nb); stack.append(nb)
        if len(idx) >= 60: groups.append(np.array(idx))
    print("objects dbg: groups=%d rss=%sKB" % (len(groups), _rss()), flush=True)
    objs = []
    for idx in sorted(groups, key=lambda a: -len(a))[:4]:
        print("objects dbg: group n=%d rss=%sKB" % (len(idx), _rss()), flush=True)
        Q = H[idx]
        print("objects dbg: Q=%s rss=%sKB" % (Q.shape, _rss()), flush=True)
        # 稳健裁剪：去掉 2% 离群（阴影/毛刺），矩形和壁线都基于裁剪后的点
        if len(Q) > 120:
            x0_, x1_ = np.percentile(Q[:, 0], [2, 98])
            y0_, y1_ = np.percentile(Q[:, 1], [2, 98])
            mQ = (Q[:, 0] >= x0_) & (Q[:, 0] <= x1_) & (Q[:, 1] >= y0_) & (Q[:, 1] <= y1_)
            if mQ.sum() > 80:
                Q = Q[mQ]
        print("objects dbg: Qtrim=%s rss=%sKB" % (Q.shape, _rss()), flush=True)
        xs, ys, zs = Q[:, 0], Q[:, 1], Q[:, 2]
        cx, cy = float(xs.mean()), float(ys.mean())
        zmax = float(zs.max())
        print("objects dbg: before minAreaRect n=%d rss=%sKB" % (len(Q), _rss()), flush=True)
        try:
            (rcx, rcy), (rw, rh), rang = cv2.minAreaRect(Q[:, :2].astype(np.float32))
            box = cv2.boxPoints(((rcx, rcy), (rw, rh), rang))
        except Exception:
            rw = rh = 0.0; rang = 0.0; box = np.zeros((4, 2))
        print("objects dbg: after minAreaRect rss=%sKB" % (_rss(),), flush=True)
        best = None
        for i in range(4):
            a = box[i]; b = box[(i+1) % 4]
            mx, my = (a[0]+b[0])/2.0, (a[1]+b[1])/2.0
            d = math.hypot(mx, my)
            if best is None or d < best[0]: best = (d, mx, my, a, b)
        # 内向法向（指向 rect 中心）
        nx, ny = rcx - best[1], rcy - best[2]
        nl = math.hypot(nx, ny) or 1.0
        # —— 近壁直线拟合（RANSAC 简化版：带内点 + PCA 主方向 + 中段均值）——
        wall = None
        try:
            ux, uy = nx / nl, ny / nl          # 内向法向
            d = (Q[:, 0] - best[1]) * ux + (Q[:, 1] - best[2]) * uy
            band = Q[np.abs(d) < 0.025]
            if len(band) >= 30:
                c0 = band[:, :2].mean(axis=0)
                uu, ss, vt = np.linalg.svd(band[:, :2] - c0, full_matrices=False)
                dirv = vt[0]
                t = (band[:, :2] - c0) @ dirv
                keep = np.abs(t) < np.percentile(np.abs(t), 65)
                c2 = band[keep, :2].mean(axis=0)
                n2 = np.array([-dirv[1], dirv[0]])
                if n2 @ np.array([ux, uy]) < 0: n2 = -n2
                # 壁面中点 = 盒心在壁线上的投影（比带内均值稳）
                ctr = np.array([cx, cy])
                proj = c2 + dirv * ((ctr - c2) @ dirv)
                ln = float(np.percentile(np.abs(t), 95) * 2)
                # 合理性：长度要接近矩形短边，点要够
                if ln >= 0.7 * min(rw, rh) and len(band) >= 40:
                    wall = {"p": [round(float(proj[0]), 3), round(float(proj[1]), 3)],
                            "n": [round(float(n2[0]), 3), round(float(n2[1]), 3)],
                            "len": round(ln, 3), "npts": int(len(band))}
        except Exception:
            wall = None
        objs.append({"n": int(len(idx)), "c": [round(cx, 3), round(cy, 3)], "size": [round(float(xs.max()-xs.min()), 3), round(float(ys.max()-ys.min()), 3)],
                     "zmax": round(zmax, 3), "rect": {"c": [round(float(rcx), 3), round(float(rcy), 3)], "w": round(float(rw), 3), "h": round(float(rh), 3), "ang": round(float(rang), 1)},
                     "near": [round(best[1], 3), round(best[2], 3)], "jn": [round(nx/nl, 3), round(ny/nl, 3)],
                     "wall": wall, "tool_p": obj_in_tool(cx, cy, zmax),
                     "table_z": round(floor, 3) if table_mode else None,
                     "corners": [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in box]})
    return {"ok": True, "floor": round(floor, 3), "objects": objs, "n": int(len(P))}

def move_to(rad, vmax=0.5, tol=0.02, timeout=None, _clear=True):
    if busy["move"]: return {"ok": False, "err": "busy"}
    with lk: q0 = list(g["q"]) if g["q"] else None
    if q0 is None: return {"ok": False, "err": "no joints"}
    # 🛡 必须步骤：向前/大范围运动前，大臂先远离立柱（侧摆A）
    if _clear and rad[1] < 2.35:
        dclear = max(abs(q0[i] - CLEAR_POSE[i]) for i in range(6))
        if dclear > 0.12:
            print("🛡 必须步骤：大臂远离立柱 → 侧摆A", flush=True)
            r = move_to(CLEAR_POSE, 0.35, tol, timeout, _clear=False)
            if not r.get("ok"):
                return {"ok": False, "err": "clear-column", "detail": r}
            with lk: q0 = list(g["q"]) if g["q"] else None
            if q0 is None: return {"ok": False, "err": "no joints"}
    busy["move"] = True
    try:
        d = max(abs(rad[i]-q0[i]) for i in range(6))
        if d < 0.005: return {"ok": True, "err": 0.0, "t": 0.0, "note": "already"}
        T = max(0.8, d / max(0.08, vmax) + 0.45)
        # 采样要足够密：控制器要求相邻点单关节增量 ≤0.1rad，这里做到 ~0.05rad
        N = max(40, int(T*90), int(12*d))
        pts = []
        for k in range(N+1):
            u = k/N; s = u*u*(3-2*u); ds = 6*u*(1-u)
            p = [q0[i] + (rad[i]-q0[i])*s for i in range(6)]
            v = [(rad[i]-q0[i])/T*ds for i in range(6)]
            pts.append((T*u, p, v))
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory(); traj.joint_names = NAMES
        for t, p, v in pts:
            q = JointTrajectoryPoint(); q.positions = p; q.velocities = v
            sec = int(t); q.time_from_start = Duration(sec=sec, nanosec=int((t-sec)*1e9))
            traj.points.append(q)
        goal.trajectory = traj
        ac = acl["ac"]
        if not ac.wait_for_server(timeout_sec=3): return {"ok": False, "err": "no action server"}
        ev = threading.Event(); res = {}
        fut = ac.send_goal_async(goal)
        def _cb(f):
            try: res["gh"] = f.result()
            except Exception as e: res["gh"] = None
            ev.set()
        fut.add_done_callback(_cb)
        if not ev.wait(5): return {"ok": False, "err": "goal timeout"}
        gh = res.get("gh")
        if gh is None or not gh.accepted: return {"ok": False, "err": "rejected"}
        ev2 = threading.Event()
        rf = gh.get_result_async(); rf.add_done_callback(lambda f: (res.__setitem__("r", f.result()), ev2.set()))
        ev2.wait(T + 20)
        t0 = time.time(); err = 9
        while time.time() - t0 < max(2.0, tol*40):
            with lk: q = list(g["q"]) if g["q"] else rad
            err = max(abs(q[i]-rad[i]) for i in range(6))
            if err < tol: break
            time.sleep(0.08)
        return {"ok": err < max(tol, 0.04), "err": round(err, 4), "t": round(T, 2)}
    finally:
        busy["move"] = False

def chassis_move(d, a):
    """底盘移动（转发到 Jetson 的 8792 指挥服务）"""
    try:
        import urllib.request as _u
        body = json.dumps({"action": "chassis", "d": float(d), "a": float(a)}).encode()
        # 经上板 relay(:8793) → Jetson :8792（下板到不了 Jetson 的 192.168 网段）
        url = "http://10.10.10.12:8793/chassis?d=%.3f&a=%.3f" % (float(d), float(a))
        return json.loads(_u.urlopen(url, timeout=14).read())
    except Exception as e:
        return {"ok": False, "err": str(e)[:80]}

def move_multi(rads, vmax=0.5, tol=0.02, _clear=True):
    """一条轨迹穿过多个关节目标（中途不停，更顺）"""
    if busy["move"]: return {"ok": False, "err": "busy"}
    with lk: q0 = list(g["q"]) if g["q"] else None
    if q0 is None: return {"ok": False, "err": "no joints"}
    # 🛡 必须步骤：轨迹里只要有一个向前目标，先让大臂远离立柱
    if _clear and any(r[1] < 2.35 for r in rads):
        dclear = max(abs(q0[i] - CLEAR_POSE[i]) for i in range(6))
        if dclear > 0.12:
            print("🛡 必须步骤：大臂远离立柱 → 侧摆A", flush=True)
            r = move_to(CLEAR_POSE, 0.35, tol, _clear=False)
            if not r.get("ok"):
                return {"ok": False, "err": "clear-column", "detail": r}
            with lk: q0 = list(g["q"]) if g["q"] else None
            if q0 is None: return {"ok": False, "err": "no joints"}
    busy["move"] = True
    try:
        ac = acl["ac"]
        if not ac.wait_for_server(timeout_sec=3): return {"ok": False, "err": "no action server"}
        goal = FollowJointTrajectory.Goal(); traj = JointTrajectory(); traj.joint_names = NAMES
        cur = list(q0); t = 0.0; acc = []
        for rad in rads:
            d = max(abs(rad[i] - cur[i]) for i in range(6))
            T = max(0.6, d / max(0.08, vmax) + 0.25)
            N = max(20, int(T * 90), int(12 * d))
            for k in range(1, N + 1):
                u = k / N; s_ = u * u * (3 - 2 * u); ds = 6 * u * (1 - u)
                p = [cur[i] + (rad[i] - cur[i]) * s_ for i in range(6)]
                v = [(rad[i] - cur[i]) / T * ds for i in range(6)]
                acc.append((t + T * u, p, v))
            t += T; cur = list(rad)
        for tt, p, v in acc:
            q = JointTrajectoryPoint(); q.positions = p; q.velocities = v
            sec = int(tt); q.time_from_start = Duration(sec=sec, nanosec=int((tt - sec) * 1e9))
            traj.points.append(q)
        goal.trajectory = traj
        ev = threading.Event(); res = {}
        def _cb(f):
            try: res["gh"] = f.result()
            except Exception: res["gh"] = None
            ev.set()
        ac.send_goal_async(goal).add_done_callback(_cb)
        if not ev.wait(5): return {"ok": False, "err": "goal timeout"}
        gh = res.get("gh")
        if gh is None or not gh.accepted: return {"ok": False, "err": "rejected"}
        ev2 = threading.Event()
        def _cb2(f):
            try: res["r"] = f.result()
            except Exception: res["r"] = None
            ev2.set()
        gh.get_result_async().add_done_callback(_cb2)
        ev2.wait(t + 15)
        t0 = time.time(); err = 9; last = list(rads[-1])
        while time.time() - t0 < 3:
            with lk: q = list(g["q"]) if g["q"] else last
            err = max(abs(q[i] - last[i]) for i in range(6))
            if err < tol: break
            time.sleep(0.08)
        return {"ok": err < 0.05, "err": round(err, 4), "t": round(t, 2)}
    finally:
        busy["move"] = False

def arm_reset():
    """清保护锁 /arm/error/reset"""
    try:
        from std_srvs.srv import SetBool
        cli = acl.get("rst")
        if cli is None: return {"ok": False, "err": "no client"}
        if not cli.wait_for_service(timeout_sec=3): return {"ok": False, "err": "no service"}
        req = SetBool.Request(); req.data = True
        fut = cli.call_async(req); ev = threading.Event(); out = {}
        fut.add_done_callback(lambda f: (out.__setitem__("r", f.result()), ev.set()))
        ev.wait(5)
        return {"ok": True, "resp": str(out.get("r"))[:80]}
    except Exception as e:
        return {"ok": False, "err": str(e)[:80]}

def gripper_cmd(pos, effort=8.0):
    ac = acl.get("gac")
    if ac is None: return {"ok": False, "err": "no gripper ac"}
    if not ac.wait_for_server(timeout_sec=3): return {"ok": False, "err": "no gripper server"}
    goal = GripperCommand.Goal(); goal.command.position = float(pos); goal.command.max_effort = float(effort)
    ev = threading.Event(); res = {}
    def _cb(f):
        try: res["gh"] = f.result()
        except Exception: res["gh"] = None
        ev.set()
    fut = ac.send_goal_async(goal); fut.add_done_callback(_cb)
    if not ev.wait(4): return {"ok": False, "err": "goal timeout"}
    gh = res.get("gh")
    if gh is None or not gh.accepted: return {"ok": False, "err": "rejected"}
    ev2 = threading.Event()
    def _cb2(f):
        try: res["r"] = f.result()
        except Exception: res["r"] = None
        ev2.set()
    rf = gh.get_result_async(); rf.add_done_callback(_cb2)
    ev2.wait(2.5)
    r = res.get("r")
    if r is None: return {"ok": False, "err": "no result"}
    st = r.result
    return {"ok": True, "pos": round(float(st.position), 4), "effort": round(float(st.effort), 2),
            "stalled": bool(st.stalled), "reached": bool(st.reached_goal)}

def set_grip(v):
    pub = acl.get("pub")
    if pub is None: return {"ok": False, "err": "no pub"}
    with lk: q = list(g["q"]) if g["q"] else None
    if q is None: return {"ok": False, "err": "no joints"}
    for _ in range(4):
        m = JointState(); m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = "new_arm_base_link"
        m.name = NAMES + ["arm_finger_joint1"]
        m.position = list(q) + [float(v)]; m.velocity = [0.0]*7; m.effort = [0.0]*7
        pub.publish(m); time.sleep(0.12)
    time.sleep(0.5)
    with lk: return {"ok": True, "v": float(v), "finger": g["finger"]}

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/state":
            with lk: return self._send({"q": g["q"], "finger": g["finger"], "depth_age": round(time.time()-g["depth_ts"], 2), "q_age": round(time.time()-g["q_ts"], 2)})
        if p == "/finger":
            with lk: return self._send({"finger": g["finger"]})
        if p == "/scan":
            return self._send(scan_objects())
        if p == "/wrist":
            return self._send(wrist_objects())
        if p == "/traj":
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            b64 = qs.get("d", [""])[0]
            try: data = json.loads(base64.urlsafe_b64decode(b64)) if b64 else {}
            except Exception: data = {}
            rads = data.get("rads") or []
            if len(rads) < 1: return self._send({"ok": False, "err": "need rads"})
            return self._send(move_multi([[float(x) for x in r] for r in rads], float(data.get("vmax", 0.5))))
        if p == "/stats":
            try:
                with open('/home/Agi/jogd/grasp_stats.json') as f: d = json.load(f)
            except Exception:
                d = {}
            return self._send({"stats": d})
        if p == "/bias":
            try:
                with open('/home/Agi/jogd/grasp_bias.json') as f: d = json.load(f)
            except Exception:
                d = {}
            return self._send({"bias": d})
        if p == "/reset":
            return self._send(arm_reset())
        if p == "/chassis":
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            d = float(qs.get("d", ["0"])[0]); a = float(qs.get("a", ["0"])[0])
            return self._send(chassis_move(d, a))
        if p == "/autograsp":
            with lk:
                if ag["running"]: return self._send({"running": True})
                ag["running"] = True; ag["ok"] = None; ag["log"] = []; ag["t0"] = time.time()
            def _job():
                try:
                    import sys as _s; 
                    if "/home/Agi/jogd" not in _s.path: _s.path.insert(0, "/home/Agi/jogd")
                    import grasp2
                    ag["mod"] = grasp2
                    ok = grasp2.auto_grasp()
                    ag["ok"] = bool(ok); ag["log"] = list(grasp2.LOG)
                except Exception as e:
                    ag["ok"] = False; ag["log"] = ag["log"] + ["ERR " + str(e)]
                finally:
                    ag["running"] = False
            threading.Thread(target=_job, daemon=True).start()
            return self._send({"started": True})
        if p == "/grasplog":
            mod = ag.get("mod")
            lg = list(getattr(mod, "LOG", [])) if mod is not None else ag["log"]
            return self._send({"running": ag["running"], "ok": ag["ok"], "dt": round(time.time()-ag["t0"],1) if ag["t0"] else 0,
                               "log": lg[-50:]})
        if p == "/do":
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            ep = qs.get("p", [""])[0]; b64 = qs.get("d", [""])[0]
            try: data = json.loads(base64.urlsafe_b64decode(b64)) if b64 else {}
            except Exception: data = {}
            if ep == "/move":
                rad = data.get("rad")
                if not rad or len(rad) != 6: return self._send({"ok": False, "err": "need rad[6]"})
                return self._send(move_to([float(x) for x in rad], float(data.get("vmax", 0.5))))
            if ep == "/grip":
                return self._send(set_grip(float(data.get("v", 0.18))))
            if ep == "/gripper":
                return self._send(gripper_cmd(float(data.get("v", 0.18)), float(data.get("eff", 8.0))))
            if ep == "/traj":
                rads = data.get("rads") or []
                if not rads: return self._send({"ok": False, "err": "need rads"})
                return self._send(move_multi([[float(x) for x in r] for r in rads], float(data.get("vmax", 0.5))))
            if ep == "/reset":
                return self._send(arm_reset())
            if ep == "/chain":
                out = []
                for r in (data.get("rads") or []):
                    res = move_to([float(x) for x in r], float(data.get("vmax", 0.5))); out.append(res)
                    if not res.get("ok"): return self._send({"ok": False, "steps": out})
                return self._send({"ok": True, "steps": out})
            return self._send({"ok": False, "err": "bad ep"})
        return self._send({"err": "unknown"}, 404)
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0)); body = self.rfile.read(n) if n else b"{}"
        try: d = json.loads(body or b"{}")
        except Exception: d = {}
        p = self.path.split("?")[0]
        if p == "/move":
            rad = d.get("rad"); vmax = float(d.get("vmax", 0.5))
            if not rad or len(rad) != 6: return self._send({"ok": False, "err": "need rad[6]"})
            return self._send(move_to([float(x) for x in rad], vmax))
        if p == "/chain":
            rads = d.get("rads") or []; vmax = float(d.get("vmax", 0.5)); out = []
            for r in rads:
                if not (isinstance(r, list) and len(r) == 6): return self._send({"ok": False, "err": "bad rad", "steps": out})
                res = move_to([float(x) for x in r], vmax); out.append(res)
                if not res.get("ok"): return self._send({"ok": False, "steps": out})
            return self._send({"ok": True, "steps": out})
        if p == "/grip":
            return self._send(set_grip(float(d.get("v", 0.18))))
        return self._send({"err": "unknown"}, 404)

def main():
    global node
    rclpy.init()
    node = Svc()
    acl["pub"] = node.create_publisher(JointState, "/actuator/motor/joint/ctrl", 10)
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", 9100), H)
    print("picksvc ready :9100", flush=True)
    # 预热
    t0 = time.time()
    while time.time()-t0 < 6 and (g["q"] is None or g["depth"] is None): time.sleep(0.2)
    srv.serve_forever()

if __name__ == "__main__":
    main()
