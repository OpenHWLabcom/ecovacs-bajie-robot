#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jogd — 八界机器人下板「流式关节 jog」服务 v5（分块执行版）
- 执行链特性：每个指令段必须「抵达目标并收敛」才会被执行；大位移/被限幅钉住 = 静默丢弃。
- 方案：把目标切成 ≤chunk 的小块，每块「平滑追达 → 静默 settle 秒 → 下一块」，
  连起来即为连续平滑跟随；任意狂拖都不会丢指令。
- stdin 每行一条 JSON： {"rad":[6 弧度]} / {"stop":true} / {"ping":1}
- 参数: --hz 40 --horizon 0.15 --vmax 0.3 --amax 1.5 --tau 0.3 --tv 0.2 --chunk 0.08 --settle 0.08 --cap 0.12
"""
import sys, os, json, time, threading, argparse

NAMES = ["joint%d" % i for i in range(1, 7)]
JLO = [-3.12, -3.12, -2.95, -3.12, -3.12, -3.12]
JHI = [3.12, 3.12, 3.30, 3.12, 3.12, 3.12]

ap = argparse.ArgumentParser()
ap.add_argument("--hz", type=float, default=40.0)
ap.add_argument("--horizon", type=float, default=0.15)
ap.add_argument("--vmax", type=float, default=0.5)
ap.add_argument("--amax", type=float, default=2.0)
ap.add_argument("--tau", type=float, default=0.3)
ap.add_argument("--tv", type=float, default=0.12)
ap.add_argument("--chunk", type=float, default=0.12)
ap.add_argument("--settle", type=float, default=0.08)
ap.add_argument("--cap", type=float, default=0.15)
ap.add_argument("--dead", type=float, default=0.012)
ap.add_argument("--arr", type=float, default=0.03)
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
    from rclpy.executors import SingleThreadedExecutor
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Duration

    rclpy.init(); node = Node("jogd")
    pub = node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
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

    cmd = list(fb["p"]); vel = [0.0]*6
    log("READY v5 chunk=%.3f settle=%.2f hz=%.0f horizon=%.2f vmax=%.2f tau=%.2f cap=%.2f dry=%d fb=%s"
        % (a.chunk, a.settle, a.hz, a.horizon, a.vmax, a.tau, a.cap, 1 if a.dry else 0,
           " ".join("%.3f" % x for x in cmd)))
    threading.Thread(target=reader, daemon=True).start()

    dt = 1.0 / a.hz
    nxt = time.time()
    last_ts = -1.0
    filt = list(cmd)
    hist = []
    ep = None
    t_start = time.time()
    last_rep = 0.0

    while True:
        with lk:
            tgt = list(st["tgt"]) if st["tgt"] else None
            ts = st["ts"]; stopf = st["stop"]; st["stop"] = False
        if stopf:
            ep = None; tgt = None; vel = [0.0]*6; hist = []
            filt = list(fb["p"])          # v5.2: stop 时重置滤波器, 防残留目标继续追
            if not a.dry:
                try:                      # v5.3: 补发"当前位置"点, 清掉控制器里的残留指令
                    m = JointTrajectory(); m.joint_names = NAMES
                    pp = JointTrajectoryPoint()
                    pp.positions = [float(x) for x in fb["p"]]
                    pp.velocities = [0.0]*6
                    sec = int(a.horizon)
                    pp.time_from_start = Duration(sec=sec, nanosec=int((a.horizon-sec)*1e9))
                    m.points = [pp]; pub.publish(m)
                    log("sent clean point at current pose")
                except Exception as e:
                    log("stop publish err:", e)
            log("HOLD (stop, filt reset)")
        if tgt is not None and ts != last_ts:
            last_ts = ts
            f = fb["p"]
            if not hist: filt = list(cmd)
            hist.append([f[i] + max(-a.lead, min(a.lead, tgt[i]-f[i])) for i in range(6)])
            if len(hist) > 5: hist.pop(0)
            dm = max(abs(cmd[i]-f[i]) for i in range(6))
            if dm > 0.5:
                cmd = list(f); vel = [0.0]*6
                log("resync cmd->fb (stale %.2f)" % dm)
        if hist:
            if len(hist) >= 3: med = [sorted((hist[-3][i], hist[-2][i], hist[-1][i]))[1] for i in range(6)]
            else: med = list(hist[-1])
            kf = min(1.0, dt / max(0.01, a.tau))
            for i in range(6): filt[i] += (med[i]-filt[i]) * kf

        now = time.time()
        has_tgt = (tgt is not None) or (last_ts > 0 and (now - last_ts) < 5.0)
        if not has_tgt:
            filt = list(fb["p"])          # v5.4: 无目标→跟随实测, 绝不追陈旧filt(防幽灵回拽)
        behind = has_tgt and (max(abs(filt[i]-fb["p"][i]) for i in range(6)) > a.dead)
        if ep is None and not behind:
            # 空闲：cmd 缓慢跟实测自愈
            for i in range(6):
                dd = fb["p"][i]-cmd[i]
                if abs(dd) < 0.002: cmd[i] = fb["p"][i]
                else: cmd[i] += max(-2.0*dt, min(2.0*dt, dd))
                vel[i] = 0.0
        else:
            if ep is None:
                pos = [fb["p"][i] + max(-a.chunk, min(a.chunk, filt[i]-fb["p"][i])) for i in range(6)]
                d = max(abs(pos[i]-cmd[i]) for i in range(6))
                if d > 0.02:
                    log("chunk d=%.3f" % d)
                ep = {"pos": pos, "phase": "chase", "t": now, "t0": now}
            if ep["phase"] == "chase":
                kv = min(1.0, dt / max(0.01, a.tv))
                for i in range(6):
                    e = ep["pos"][i]-cmd[i]
                    vd = max(-a.vmax, min(a.vmax, e*3.0))
                    dv = (vd-vel[i])*kv
                    lim = a.amax*dt
                    if dv > lim: dv = lim
                    elif dv < -lim: dv = -lim
                    vel[i] += dv
                    cmd[i] += vel[i]*dt
                    cmd[i] = max(JLO[i], min(JHI[i], cmd[i]))
                for i in range(6):
                    lo = fb["p"][i]-a.cap; hi = fb["p"][i]+a.cap
                    if cmd[i] < lo: cmd[i] = lo
                    elif cmd[i] > hi: cmd[i] = hi
                reached_cmd0 = max(abs(ep["pos"][i]-cmd[i]) for i in range(6)) < 0.003
                if not a.dry and not reached_cmd0:
                    m = JointTrajectory(); m.joint_names = NAMES
                    p = JointTrajectoryPoint()
                    p.positions = [float(x) for x in cmd]
                    p.velocities = [float(x) for x in vel]
                    sec = int(a.horizon)
                    p.time_from_start = Duration(sec=sec, nanosec=int((a.horizon-sec)*1e9))
                    m.points = [p]; pub.publish(m)
                reached_cmd = max(abs(ep["pos"][i]-cmd[i]) for i in range(6)) < 0.003
                more_after = max(abs(filt[i]-ep["pos"][i]) for i in range(6)) > 0.02
                ath = 0.012 if not more_after else a.arr
                arrived = max(abs(ep["pos"][i]-fb["p"][i]) for i in range(6)) < ath
                if reached_cmd and arrived:
                    for i in range(6): cmd[i] = ep["pos"][i]
                    vel = [0.0]*6
                    ep = None                       # 立即准备下一段
                elif reached_cmd:
                    vel = [0.0]*6                   # 已发到位, 静默等实测到达
                    if fb["t"] > 0 and now - ep["t"] > 2.0:
                        ep = None                   # 超时放弃本段(重发下一段再试)
                else:
                    ep["t0"] = ep.get("t0", now)
            else:
                if now - ep["t"] >= a.settle:
                    ep = None
        now = time.time()
        if now - last_rep >= 1.0:
            last_rep = now
            stt = "idle" if ep is None else ep["phase"]
            log("%s filt=%s cmd=%s v=%s" % (stt,
                " ".join("%.3f" % x for x in filt), " ".join("%.3f" % x for x in cmd),
                " ".join("%.2f" % x for x in vel)))
        nxt += dt
        sl = nxt - time.time()
        if sl > 0: time.sleep(sl)
        else: nxt = time.time()
        if a.secs and (time.time() - t_start) > a.secs:
            log("secs limit -> exit"); os._exit(0)

if __name__ == "__main__":
    main()
