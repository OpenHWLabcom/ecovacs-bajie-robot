#!/usr/bin/env python3
# smoothmove.py — 通用顺滑动作执行器（控制算法固化版）
# 用法: smoothmove.py <waypoints.json> <v_max> [goal:j1..j6|-] [fallback:0/1]
import json, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

HOME=[0.0,2.46,3.256,-0.007,-2.543,0.0]
NAMES=["joint%d"%i for i in range(1,7)]
WP=json.load(open(sys.argv[1]))
VMAX=float(sys.argv[2])
GOAL=None
if len(sys.argv)>3 and sys.argv[3] not in ('-',''):
    GOAL=[float(x) for x in sys.argv[3].split(',')]
if GOAL is None: GOAL=list(WP[-1])
FBK=(len(sys.argv)>4 and sys.argv[4]=='1')
ENF=(max(abs(a-b) for a,b in zip(GOAL,HOME))<0.05)

def build_bell(wp, v_max, step=0.02):
    dense=[list(wp[0])]
    for i in range(len(wp)-1):
        a=wp[i]; b=wp[i+1]
        dist=max(abs(b[j]-a[j]) for j in range(6))
        n=max(1,int(dist/step)+1)
        for k in range(1,n+1):
            s=k/n; dense.append([a[j]+(b[j]-a[j])*s for j in range(6)])
    steps=[max(abs(dense[k+1][j]-dense[k][j]) for j in range(6)) for k in range(len(dense)-1)]
    L=sum(steps); ts=[0.0]; cum=0.0
    for k in range(len(steps)):
        u=(cum+0.5*steps[k])/L if L>0 else 0.5
        vfac=0.5+0.5*math.sin(math.pi*min(1,max(0,u)))
        v=v_max*max(vfac,0.35)
        ts.append(ts[-1]+steps[k]/max(v,0.05))
        cum+=steps[k]
    pts=[]
    for k in range(len(dense)):
        if k==0: vel=[(dense[1][j]-dense[0][j])/(ts[1]-ts[0]) for j in range(6)]
        elif k==len(dense)-1: vel=[(dense[k][j]-dense[k-1][j])/(ts[k]-ts[k-1]) for j in range(6)]
        else: vel=[(dense[k+1][j]-dense[k-1][j])/(ts[k+1]-ts[k-1]) for j in range(6)]
        pts.append({"t":ts[k],"pos":dense[k],"vel":vel})
    vpk=max(max(abs(p["vel"][j]) for j in range(6)) for p in pts)
    print("  build: %d pts, T=%.2fs, vpeak=%.2f"%(len(pts),ts[-1],vpk), flush=True)
    return pts,vpk

rclpy.init(); node=Node("smoothmove")
fb={"p":None,"f":None}
def on_js(m):
    d=dict(zip(m.name,m.position)); v=[d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"]=[float(x) for x in v]
    if "arm_finger_joint1" in d: fb["f"]=float(d["arm_finger_joint1"])
node.create_subscription(JointState,"/joint_states",on_js,50)
def spin(sec):
    t0=time.time()
    while time.time()-t0<sec: rclpy.spin_once(node,timeout_sec=0.1)
def cur_pose(sec=0.4):
    t0=time.time()
    while fb["p"] is None and time.time()-t0<8: rclpy.spin_once(node,timeout_sec=0.1)
    last=fb["p"]; t0=time.time()
    while time.time()-t0<sec: rclpy.spin_once(node,timeout_sec=0.1); last=fb["p"]
    return last
def wait_finger(timeout=6):
    t0=time.time()
    while fb["f"] is None and time.time()-t0<timeout: rclpy.spin_once(node,timeout_sec=0.2)
    return fb["f"]

if ENF:
    f=wait_finger(); print("finger=%.3f"%f,flush=True)
    if f is None or not (0.13<=f<=0.26): print("!! 夹子状态不符(%.3f)"%f); sys.exit(9)

ac=ActionClient(node,FollowJointTrajectory,"/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10): print("NO SERVER"); sys.exit(2)
def send(pts,label):
    goal=FollowJointTrajectory.Goal(); traj=JointTrajectory(); traj.joint_names=NAMES
    for r in pts:
        q=JointTrajectoryPoint(); q.positions=list(r["pos"]); q.velocities=list(r["vel"])
        t=r["t"]; sec=int(t); q.time_from_start=Duration(sec=sec,nanosec=int((t-sec)*1e9))
        traj.points.append(q)
    goal.trajectory=traj
    print("== %s: %d pts, %.2fs =="%(label,len(pts),pts[-1]["t"]),flush=True)
    snd=ac.send_goal_async(goal); rclpy.spin_until_future_complete(node,snd,timeout_sec=8)
    gh=snd.result()
    if not (gh and gh.accepted): print("!! REJECTED"); return False
    res=gh.get_result_async(); rclpy.spin_until_future_complete(node,res,timeout_sec=int(pts[-1]["t"])+35)
    return True
def settle(tgt,label,wait=35):
    t0=time.time(); end=cur_pose(0.1); err=max(abs(a-b) for a,b in zip(end,tgt))
    while err>0.02 and time.time()-t0<wait:
        time.sleep(1.2); end=cur_pose(0.1); err=max(abs(a-b) for a,b in zip(end,tgt))
        print("   [%s] err=%.3f"%(label,err),flush=True)
    print("== %s done. err=%.4f end=%s"%(label,err," ".join("%.3f"%x for x in end)),flush=True)
    return err,end

cur=cur_pose()
pts,vpk=build_bell([cur]+WP, VMAX)
if vpk>0.95: print("!! 速度异常 %.2f, 不执行"%vpk); sys.exit(8)
if not send(pts,"顺滑移动"): sys.exit(3)
err,end=settle(GOAL,"到位",35)
if err>0.05 and FBK:
    print("!! 未到位(err=%.3f), 慢速补正"%err,flush=True)
    e=cur_pose(0.1); pts3=[]
    for k in range(1,31):
        d=k/30.0; s=d*d*(3-2*d); ds=6*d*(1-d)
        p=[e[i]+(GOAL[i]-e[i])*s for i in range(6)]
        v=[(GOAL[i]-e[i])/2.5*ds for i in range(6)]
        pts3.append({"t":2.5*d,"pos":p,"vel":v})
    send(pts3,"补正")
    err,end=settle(GOAL,"补正",35)
print("DONE err=%.4f end=%s"%(err," ".join("%.3f"%x for x in end)),flush=True)
sys.exit(0 if err<=0.05 else 4)
