#!/usr/bin/env python3
# smoothtrack.py — 拖动流式执行器（速度包络·顺序单发版）
# 协议: {"rad":[6]} / {"stop":true} / {"ping":1} / {"grip":0..0.69}; 每条=从实测出发的钟形轨迹, 等完成后接下一段
import sys, os, json, time, threading, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

NAMES = ["joint%d" % i for i in range(1, 7)]
JLO = [-3.12, -3.12, -2.95, -3.12, -3.12, -3.12]
JHI = [3.12, 3.12, 3.30, 3.12, 3.12, 3.12]
VMAX = 2.0
AMAX = 6.0
VMIN_SEG = 0.15
DEAD = 0.020
# —— 柔动模式参数 ——
DANCE_VMAX = 0.35
DANCE_AMAX = 2.0
DANCE = {"on": 0, "amp": 1.0, "period": 7.0, "wave": 0}
dance_base = None; dance_ph = 0.0; dance_t0 = 0.0; dance_suppress = False; dance_n = 0; wave_th = None; wave_fresh = True; wave_hold_until = 0.0; wave_hold_fb = None; MODE_EP=[0]

def log(*x): print(time.strftime("%H:%M:%S"), *x, flush=True)
lk = threading.Lock()
st = {"tgt": None, "ts": 0.0, "stop": False}
grip = {"v": None, "ts": 0.0, "until": 0.0}
def reader():
    for line in sys.stdin:
        line = line.strip()
        if not line: continue
        try: j = json.loads(line)
        except Exception: continue
        if j.get("ping"): continue
        if j.get("grip") is not None:
            try:
                gv = max(0.0, min(0.69, float(j["grip"])))
                with lk:
                    grip["v"] = gv; grip["ts"] = time.time(); grip["until"] = time.time() + 1.0
                log("GRIP %.3f" % gv)
            except Exception:
                pass
            continue
        if isinstance(j.get("mode"), dict):
            m = j["mode"]
            global VMAX, AMAX, VMIN_SEG, DEAD, DANCE, dance_base, dance_suppress, dance_t0, wave_th, wave_fresh
            try:
                VMAX = max(0.3, min(3.0, float(m.get("vmax", VMAX))))
                AMAX = max(1.0, min(10.0, float(m.get("amax", AMAX))))
                VMIN_SEG = max(0.10, min(0.50, float(m.get("vmin", VMIN_SEG))))
                DEAD = max(0.005, min(0.05, float(m.get("dead", DEAD))))
                DANCE["on"] = 1 if int(m.get("dance", 0)) else 0
                DANCE["amp"] = max(0.2, min(2.0, float(m.get("damp", 1.0))))
                DANCE["period"] = max(3.0, min(15.0, float(m.get("dperiod", 7.0))))
                DANCE["wave"] = 1 if int(m.get("wave", 0)) else 0
                MODE_EP[0] += 1
                b = m.get("base")
                if DANCE["on"]:
                    if isinstance(b, list) and len(b) == 6:
                        dance_base = [float(x) for x in b]
                    else:
                        dance_base = None
                    dance_t0 = time.time(); wave_th = None; wave_fresh = True
                else:
                    dance_base = None
                dance_suppress = False
                log("MODE applied: %s vmax=%.2f amax=%.1f vmin=%.2f dead=%.3f dance=%d" % (m.get("label", m.get("id","?")), VMAX, AMAX, VMIN_SEG, DEAD, DANCE["on"]))
            except Exception as e:
                log("mode err:", e)
            continue
        with lk:
            if j.get("stop"):
                st["tgt"] = None; st["stop"] = True; st["ts"] = time.time()
                log("STOP (hold)")
            else:
                r = j.get("rad")
                if isinstance(r, list) and len(r) == 6:
                    st["tgt"] = [max(JLO[i], min(JHI[i], float(r[i]))) for i in range(6)]
                    st["ts"] = time.time(); st["stop"] = False
    log("stdin EOF -> exit"); os._exit(0)

rclpy.init(); node = Node("smoothtrack")
ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
fb = {"p": None, "t": 0.0}
def on_js(m):
    d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]; fb["t"] = time.time()
node.create_subscription(JointState, "/joint_states", on_js, 200)
grip_pub = node.create_publisher(JointState, "/actuator/motor/joint/ctrl", 10)
def spin(sec):
    t0=time.time()
    while time.time()-t0<sec:
        rclpy.spin_once(node, timeout_sec=min(0.1, sec))
t0=time.time()
while fb["p"] is None and time.time()-t0<20: spin(0.1)
if fb["p"] is None: log("NO feedback"); sys.exit(2)
if not ac.wait_for_server(timeout_sec=10): log("NO server"); sys.exit(2)
threading.Thread(target=reader, daemon=True).start()

def build_seg(cur, tgt, v_max, amax=None):
    # 真S曲线: 首尾速度与加速度均归零 (10x^3-15x^4+6x^5), 加速度上限约束
    a = amax if amax else AMAX
    D=max(abs(tgt[i]-cur[i]) for i in range(6))
    if D < 1e-6: return []
    T=max(VMIN_SEG, 1.875*D/v_max, (5.77*D/a)**0.5)
    N=max(20, int(T/0.025)+1)
    pts=[]
    for k in range(N+1):
        x=float(k)/N
        S=x*x*x*(10-15*x+6*x*x)
        Sp=30*x*x*(1-x)*(1-x)
        pos=[cur[i]+(tgt[i]-cur[i])*S for i in range(6)]
        vel=[(tgt[i]-cur[i])*Sp/T for i in range(6)]
        pts.append((T*x, pos, vel))
    return pts

def send_and_wait(pts, tgt):
    goal=FollowJointTrajectory.Goal(); traj=JointTrajectory(); traj.joint_names=NAMES
    for (t,p,v) in pts:
        q=JointTrajectoryPoint(); q.positions=[float(x) for x in p]; q.velocities=[float(x) for x in v]
        sec=int(t); q.time_from_start=Duration(sec=sec, nanosec=int((t-sec)*1e9))
        traj.points.append(q)
    goal.trajectory=traj
    snd=ac.send_goal_async(goal)
    t0=time.time()
    while not snd.done() and time.time()-t0<8: spin(0.05)
    gh=snd.result()
    if gh is None or not gh.accepted: return "reject"
    res=gh.get_result_async()
    tt=time.time(); dur=pts[-1][0]
    while True:
        spin(0.05)
        if res.done(): return "res"
        f=fb["p"]
        if f is not None and max(abs(tgt[i]-f[i]) for i in range(6))<0.012: return "conv"
        if time.time()-tt>dur+3.0: return "timeout"

log("READY smooth-v2.8 Scurve vmax=%.2f amax=%.1f dead=%.3f modes=on wave+handpause+head" % (VMAX, AMAX, DEAD))
AMP_D=[0.10,0.06,0.07,0.025,0.11,0.04]
PH_D=[0.0,1.1,2.3,0.5,3.1,1.7]
def dance_off(th, base, amp):
    return [base[i]+amp*AMP_D[i]*(0.62*math.sin(th+PH_D[i])+0.38*math.sin(2.42*th+1.7*PH_D[i])) for i in range(6)]
AMP_W=[0.16,0.19,0.22,0.16,0.26,0.30]
LAG_W=0.30
def dance_off_wave(th, base, amp):
    # 紧凑行波 + J6 摇头（快速左右摆）
    out=[]
    for i in range(6):
        v=amp*AMP_W[i]*(0.75*math.sin(th-i*LAG_W)+0.25*math.sin(2*th-2*i*LAG_W))
        if i==5: v+=amp*0.17*math.sin(2.6*th+0.4)
        out.append(base[i]+v)
    return out
def wave_seg(fbp, th0, th1):
    w2=2*math.pi/max(3.0,DANCE["period"])
    dur=(th1-th0)/w2
    N=max(16, int(dur/0.03)+1)
    pts=[]
    for k in range(N+1):
        th=th0+(th1-th0)*k/N
        pa=dance_off_wave(th, dance_base, DANCE["amp"])
        va=[w2*DANCE["amp"]*(AMP_W[i]*(0.75*math.cos(th-i*LAG_W)+0.5*math.cos(2*th-2*i*LAG_W))+(0.17*2.6*math.cos(2.6*th+0.4) if i==5 else 0.0)) for i in range(6)]
        if fbp is not None:
            # 启动段：位置从当前姿态平滑过渡到波浪，速度从 0 爬升（约 0.9s）
            w = max(0.0, 1.0 - k / 30.0)
            p = [pa[i] + (fbp[i] - pa[i]) * w for i in range(6)]
            env = min(1.0, k / 30.0)
            va = [v * env for v in va]
        else:
            p = pa
        pts.append((dur*k/N, p, va))
    return pts
def send_flow(pts):
    goal=FollowJointTrajectory.Goal(); traj=JointTrajectory(); traj.joint_names=NAMES
    for (t,p,v) in pts:
        q=JointTrajectoryPoint(); q.positions=[float(x) for x in p]; q.velocities=[float(x) for x in v]
        sec=int(t); q.time_from_start=Duration(sec=sec, nanosec=int((t-sec)*1e9))
        traj.points.append(q)
    goal.trajectory=traj
    snd=ac.send_goal_async(goal)
    t0=time.time()
    while not snd.done() and time.time()-t0<3: spin(0.05)
    gh=snd.result()
    if gh is None or not gh.accepted: return False
    res=gh.get_result_async()
    dur=pts[-1][0]; tt=time.time(); ts_sent=tt; ep=MODE_EP[0]
    while not res.done() and time.time()-tt<dur-0.12:
        spin(0.04)
        if MODE_EP[0]!=ep: break
        with lk:
            if st["stop"]: break
            if st["tgt"] is not None and st["ts"] > ts_sent: break
    return True
def wave_flow():
    global dance_base, dance_t0, dance_n, wave_th, wave_fresh, wave_hold_until, wave_hold_fb
    if not DANCE["on"] or dance_suppress: return
    f=fb["p"]
    if f is None: return
    now=time.time()
    if dance_base is None:
        if now < wave_hold_until: return
        if wave_hold_fb is not None and max(abs(f[i]-wave_hold_fb[i]) for i in range(6)) > 0.02:
            wave_hold_fb=list(f); wave_hold_until=now+1.5; return
        dance_base=list(f); dance_t0=now; wave_th=None; wave_fresh=True; wave_hold_fb=None; return
    w2=2*math.pi/max(3.0,DANCE["period"])
    dur=6.5
    if wave_th is None:
        wave_th=(time.time()-dance_t0)*w2 + 0.35
    th0=wave_th; th1=th0+w2*dur
    proj=dance_off_wave(th0, dance_base, DANCE["amp"])
    dmax=max(abs(proj[i]-f[i]) for i in range(6))
    if dmax > 0.40:
        log("wave pause: arm moved externally (%.2f rad off path)" % dmax)
        dance_base=None; wave_hold_fb=list(f); wave_hold_until=time.time()+2.0; wave_th=None
        return
    pts=wave_seg(f if wave_fresh else None, th0, th1)
    wave_fresh=False
    send_flow(pts)
    wave_th=th1
    dance_n+=1
    if dance_n%3==1: log("waveflow3 #%d dur=%.1f"%(dance_n,dur))
def dance_go():
    if DANCE.get("wave"): wave_flow()
    else: dance_step()
def build_dance_seg(cur, tgt, T):
    D=max(abs(tgt[i]-cur[i]) for i in range(6))
    if D < 1e-6: return []
    N=max(24, int(T/0.03)+1)
    pts=[]
    for k in range(N+1):
        x=float(k)/N
        S=x*x*x*(10-15*x+6*x*x); Sp=30*x*x*(1-x)*(1-x)
        pos=[cur[i]+(tgt[i]-cur[i])*S for i in range(6)]
        vel=[(tgt[i]-cur[i])*Sp/T for i in range(6)]
        pts.append((T*x,pos,vel))
    return pts
def dance_step():
    global dance_base, dance_ph, dance_n, dance_t0
    if not DANCE["on"] or dance_suppress: return
    f=fb["p"]
    if f is None: return
    if dance_base is None:
        dance_base=list(f); dance_t0=time.time(); return
    off_fn = dance_off_wave if DANCE.get("wave") else dance_off
    w2=2*math.pi/max(3.0, DANCE["period"])
    if DANCE.get("wave"):
        step_s=max(0.35, min(0.9, DANCE["period"]*0.09))
        look=w2*step_s
    else:
        step_s=max(1.4, DANCE["period"]/2.0)
        look=math.pi
    th=(time.time()-dance_t0)*w2 + look
    td=off_fn(th, dance_base, DANCE["amp"])
    dist=max(abs(td[i]-f[i]) for i in range(6))
    if dist < 0.010: spin(0.12); return
    pts=build_dance_seg(f, td, step_s)
    send_and_wait(pts, td)
    dance_n += 1
    if dance_n % 12 == 1:
        log("dance step #%d dist=%.3f T=%.2f" % (dance_n, dist, step_s))

last_log=0.0
last_tgt=None; corr=0
while True:
    spin(0.05)
    nowg = time.time()
    with lk:
        gv = grip["v"]; gu = grip["until"]
    if gv is not None and nowg < gu and fb["p"] is not None:
        gm = JointState()
        gm.header.stamp = node.get_clock().now().to_msg()
        gm.header.frame_id = "new_arm_base_link"
        gm.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "arm_finger_joint1"]
        gm.position = list(fb["p"][:6]) + [gv]
        gm.velocity = [0.0] * 7
        gm.effort = [0.0] * 7
        grip_pub.publish(gm)
    with lk:
        tgt=list(st["tgt"]) if st["tgt"] else None
        ts=st["ts"]; stopf=st["stop"]; st["stop"]=False
    if stopf:
        log("HOLD")
        dance_base = None; dance_suppress = True
    if tgt is None or (time.time()-ts)>8.0:
        if DANCE["on"] and not dance_suppress: dance_go()
        continue
    if last_tgt is None or max(abs(tgt[i]-last_tgt[i]) for i in range(6))>0.002:
        last_tgt=list(tgt); corr=0
    f=fb["p"]
    if f is None: continue
    dist=max(abs(tgt[i]-f[i]) for i in range(6))
    if dist < DEAD:
        if DANCE["on"] and not dance_suppress and (time.time()-ts)>2.5: dance_go()
        continue
    if corr>=2: continue
    if dist<0.06: corr+=1
    dance_base=None
    pts=build_seg(f, tgt, VMAX)
    vpk=max(max(abs(v[j]) for j in range(6)) for (_,_,v) in pts)
    ok=send_and_wait(pts, tgt)
    f2=fb["p"] or f
    err=max(abs(tgt[i]-f2[i]) for i in range(6))
    now=time.time()
    if now-last_log>0.25:
        last_log=now
        log("seg dist=%.3f T=%.2f vpk=%.2f wait=%s err=%.3f" % (dist, pts[-1][0], vpk, ok, err))
