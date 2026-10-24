#!/usr/bin/env python3
# smoothtrack.py — 拖动流式执行器（速度包络·顺序单发版）
# 协议: {"rad":[6]} / {"stop":true} / {"ping":1}; 每条=从实测出发的钟形轨迹, 等完成后接下一段
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
VMAX = 1.0
VMIN_SEG = 0.25
DEAD = 0.004

def log(*x): print(time.strftime("%H:%M:%S"), *x, flush=True)
lk = threading.Lock()
st = {"tgt": None, "ts": 0.0, "stop": False}
def reader():
    for line in sys.stdin:
        line = line.strip()
        if not line: continue
        try: j = json.loads(line)
        except Exception: continue
        if j.get("ping"): continue
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
def spin(sec):
    t0=time.time()
    while time.time()-t0<sec:
        rclpy.spin_once(node, timeout_sec=min(0.1, sec))
t0=time.time()
while fb["p"] is None and time.time()-t0<20: spin(0.1)
if fb["p"] is None: log("NO feedback"); sys.exit(2)
if not ac.wait_for_server(timeout_sec=10): log("NO server"); sys.exit(2)
threading.Thread(target=reader, daemon=True).start()

def build_seg(cur, tgt, v_max):
    dense=[list(cur)]
    dist=max(abs(tgt[i]-cur[i]) for i in range(6))
    n=max(1,int(dist/0.02)+1)
    for k in range(1,n+1):
        s=k/n; dense.append([cur[i]+(tgt[i]-cur[i])*s for i in range(6)])
    steps=[max(abs(dense[k+1][j]-dense[k][j]) for j in range(6)) for k in range(len(dense)-1)]
    L=sum(steps); ts=[0.0]; cum=0.0
    for k in range(len(steps)):
        u=(cum+0.5*steps[k])/L if L>0 else 0.5
        vfac=0.5+0.5*math.sin(math.pi*min(1,max(0,u)))
        v=v_max*max(vfac,0.3)
        ts.append(ts[-1]+steps[k]/max(v,0.05)); cum+=steps[k]
    if ts[-1] < VMIN_SEG:
        sc=VMIN_SEG/ts[-1] if ts[-1]>1e-6 else 1.0
        ts=[x*sc for x in ts]
    pts=[]
    for k in range(len(dense)):
        if k==0: vel=[(dense[1][j]-dense[0][j])/(ts[1]-ts[0]) for j in range(6)]
        elif k==len(dense)-1: vel=[(dense[k][j]-dense[k-1][j])/(ts[k]-ts[k-1]) for j in range(6)]
        else: vel=[(dense[k+1][j]-dense[k-1][j])/(ts[k+1]-ts[k-1]) for j in range(6)]
        pts.append((ts[k],dense[k],vel))
    return pts

def send_and_wait(pts):
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
    if gh is None or not gh.accepted: return False
    res=gh.get_result_async()
    tt=time.time(); dur=pts[-1][0]
    while not res.done() and time.time()-tt<dur+25: spin(0.05)
    return True

log("READY smooth-v1 vmax=%.2f vmin_seg=%.2f" % (VMAX, VMIN_SEG))
last_log=0.0
while True:
    spin(0.05)
    with lk:
        tgt=list(st["tgt"]) if st["tgt"] else None
        ts=st["ts"]; stopf=st["stop"]; st["stop"]=False
    if stopf: log("HOLD")
    if tgt is None or (time.time()-ts)>8.0: continue
    f=fb["p"]
    if f is None: continue
    dist=max(abs(tgt[i]-f[i]) for i in range(6))
    if dist < DEAD: continue
    pts=build_seg(f, tgt, VMAX)
    vpk=max(max(abs(v[j]) for j in range(6)) for (_,_,v) in pts)
    ok=send_and_wait(pts)
    f2=fb["p"] or f
    err=max(abs(tgt[i]-f2[i]) for i in range(6))
    now=time.time()
    if now-last_log>0.25:
        last_log=now
        log("seg dist=%.3f T=%.2f vpk=%.2f ok=%s err=%.3f" % (dist, pts[-1][0], vpk, ok, err))
