#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jogd_act — 流式 jog（ACTION + 多点轨迹版，v7）
- 与 jogd 相同 stdin 协议： {"rad":[6]} / {"stop":true} / {"ping":1}
- 内部维护虚拟参考(速度/加速度受限 + 输入平滑)；每隔 rate 秒或参考变化时，
  从当前参考生成 win 秒 / np 点的平滑轨迹 → 作为 action goal 发到
  /arm_controller/follow_joint_trajectory（新 goal 自然替换旧 goal）。
- 无目标(>5s)时完全被动（跟随实测，不发 goal）。
"""
import sys, os, json, time, threading, argparse

NAMES = ["joint%d" % i for i in range(1, 7)]
JLO = [-3.12, -3.12, -2.95, -3.12, -3.12, -3.12]
JHI = [3.12, 3.12, 3.30, 3.12, 3.12, 3.12]

ap = argparse.ArgumentParser()
ap.add_argument("--vmax", type=float, default=1.2)
ap.add_argument("--amax", type=float, default=4.0)
ap.add_argument("--tau", type=float, default=0.15)
ap.add_argument("--dead", type=float, default=0.006)
ap.add_argument("--win", type=float, default=1.2)
ap.add_argument("--np", type=int, default=20)
ap.add_argument("--rate", type=float, default=0.6)
ap.add_argument("--lead", type=float, default=2.8)
ap.add_argument("--dry", action="store_true")
ap.add_argument("--secs", type=float, default=0.0)
a = ap.parse_args()

def log(*x): print(time.strftime("%H:%M:%S"), *x, flush=True)

lk = threading.Lock()
st = {"tgt": None, "ts": 0.0, "stop": False}

def reader():
    for line in sys.stdin:
        line = line.strip()
        if not line: continue
        try: j = json.loads(line)
        except Exception: log("bad line:", line[:80]); continue
        if j.get("ping"): continue
        with lk:
            if j.get("stop"):
                st["tgt"] = None; st["stop"] = True; st["ts"] = time.time()
                log("STOP (hold)")
            else:
                r = j.get("rad")
                if isinstance(r, list) and len(r) == 6:
                    try:
                        st["tgt"] = [max(JLO[i], min(JHI[i], float(r[i]))) for i in range(6)]
                        st["ts"] = time.time(); st["stop"] = False
                    except Exception as e: log("bad rad:", e)
    log("stdin EOF -> exit"); os._exit(0)

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Duration

    rclpy.init(); node = Node("jogd_act")
    ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    fb = {"p": None, "t": 0.0}
    def on_js(m):
        d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
        if all(x is not None for x in v): fb["p"] = [float(x) for x in v]; fb["t"] = time.time()
    node.create_subscription(JointState, "/joint_states", on_js, 200)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while fb["p"] is None and time.time() - t0 < 20.0: time.sleep(0.1)
    if fb["p"] is None: log("NO /joint_states feedback — abort"); sys.exit(2)
    if not ac.wait_for_server(timeout_sec=10): log("NO action server — abort"); sys.exit(2)

    cmd = list(fb["p"]); hist = []; filt = list(cmd)
    log("READY v7-act vmax=%.2f amax=%.2f tau=%.2f win=%.2f np=%d rate=%.2f dry=%d fb=%s"
        % (a.vmax, a.amax, a.tau, a.win, a.np, a.rate, 1 if a.dry else 0,
           " ".join("%.3f" % x for x in cmd)))
    threading.Thread(target=reader, daemon=True).start()

    dt = 0.05
    last_ts = -1.0
    last_send = 0.0
    last_sent_ref = None
    cur_goal = {"h": None}
    t_start = time.time()

    def sim_path(from_ref, steps, ddt):
        c = list(from_ref); v = [0.0]*6
        pts = []
        for k in range(steps):
            for i in range(6):
                e = filt[i] - c[i]
                vd = max(-a.vmax, min(a.vmax, e * 3.0))
                dv = max(-a.amax*ddt, min(a.amax*ddt, vd - v[i]))
                v[i] += dv
                c[i] = max(JLO[i], min(JHI[i], c[i] + v[i]*ddt))
            pts.append((c[:], v[:]))
        return pts

    def send_goal():
        nonlocal last_send, last_sent_ref
        if a.dry: return
        ddt = a.win / a.np
        pts = sim_path(list(fb["p"]), a.np, ddt)   # 关键: 从实测出发(首点贴近真机) 
        traj = JointTrajectory(); traj.joint_names = NAMES
        for k, (c, v) in enumerate(pts):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in c]
            p.velocities = [float(x) for x in v]
            tf = ddt * (k + 1)
            sec = int(tf)
            p.time_from_start = Duration(sec=sec, nanosec=int((tf - sec) * 1e9))
            traj.points.append(p)
        g = FollowJointTrajectory.Goal(); g.trajectory = traj
        fut = ac.send_goal_async(g)   # 直接发新 goal，由控制器自然替换旧 goal
        def done(f):
            try:
                gh = f.result()
                cur_goal["h"] = gh
            except Exception as e:
                log("goal err:", e)
        fut.add_done_callback(done)
        last_send = time.time()
        last_sent_ref = list(cmd)
        log("goal sent: ref=%s" % " ".join("%.3f" % x for x in cmd))

    while True:
        with lk:
            tgt = list(st["tgt"]) if st["tgt"] else None
            ts = st["ts"]; stopf = st["stop"]; st["stop"] = False
        now = time.time()
        if stopf:
            hist = []; filt = list(fb["p"]); cmd = list(fb["p"])
            log("HOLD (stop; current goal will finish)")
        if tgt is not None and ts != last_ts:
            last_ts = ts
            f = fb["p"]
            if not hist: filt = list(cmd)
            hist.append([f[i] + max(-a.lead, min(a.lead, tgt[i]-f[i])) for i in range(6)])
            if len(hist) > 5: hist.pop(0)
            dm = max(abs(cmd[i]-f[i]) for i in range(6))
            if dm > 1.5:
                cmd = list(f); log("resync cmd->fb (stale %.2f)" % dm)
        if hist:
            if len(hist) >= 3: med = [sorted((hist[-3][i], hist[-2][i], hist[-1][i]))[1] for i in range(6)]
            else: med = list(hist[-1])
            kf = min(1.0, dt / max(0.01, a.tau))
            for i in range(6): filt[i] += (med[i]-filt[i]) * kf
        has_tgt = (tgt is not None) or (last_ts > 0 and (now - last_ts) < 5.0)
        if not has_tgt:
            filt = list(fb["p"])
            cmd = list(fb["p"])
        else:
            for i in range(6):
                e = filt[i] - cmd[i]
                vd = max(-a.vmax, min(a.vmax, e * 3.0))
                dv = max(-a.amax*dt, min(a.amax*dt, vd))
                cmd[i] = max(JLO[i], min(JHI[i], cmd[i] + dv*dt))
            moved = last_sent_ref is None or max(abs(cmd[i]-last_sent_ref[i]) for i in range(6)) > 0.02
            if now - last_send >= a.rate and (moved or (max(abs(filt[i]-fb["p"][i]) for i in range(6)) > a.dead and now - last_send >= a.rate*2)):
                send_goal()
        time.sleep(dt)
        if a.secs and (time.time() - t_start) > a.secs:
            log("secs limit -> exit"); os._exit(0)

if __name__ == "__main__":
    main()
