#!/usr/bin/env python3
# circle_play3.py — 完整一圈(顺滑): 归位→开箱位(学习路径) → 开箱位→归位(学习路径)
import json, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

HOME=[0.0,2.46,3.256,-0.007,-2.543,0.0]
OPEN=[0.0,1.766,2.887,0.0,-0.451,0.0]
NAMES=["joint%d"%i for i in range(1,7)]
exit_wp=json.load(open('/home/Agi/exit_wp.json'))
stow_wp=json.load(open('/home/Agi/stow_wp.json'))

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

rclpy.init(); node=Node("circle3")
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

f=wait_finger()
print("finger=%.3f"%(f if f is not None else -1),flush=True)
if f is None: print("!! 夹子读数超时"); sys.exit(9)
if not (0.13<=f<=0.26): print("!! 夹子状态不符(%.3f)"%f); sys.exit(9)

cur0=cur_pose()
e0=max(abs(a-b) for a,b in zip(cur0,HOME))
print("起点距归位: %.3f (臂应在归位位)"%e0, flush=True)

pts1,vpk1=build_bell([cur0]+exit_wp, 0.5)
if vpk1>0.95: print("!! ①速度异常 %.2f, 不执行"%vpk1); sys.exit(8)
if not send(pts1,"①出舱·学习路径·顺滑"): sys.exit(3)
err1,end1=settle(OPEN,"①开箱位",35)
if err1>0.05: print("!! ①未到位, 中止"); sys.exit(4)
time.sleep(1.2)
f=wait_finger()
print("夹子复查 finger=%.3f"%(f if f is not None else -1),flush=True)
if f is None or not (0.13<=f<=0.26): print("!! 夹子状态不符, 停止"); sys.exit(9)

cur2=cur_pose()
pts2,vpk2=build_bell([cur2]+stow_wp, 0.65)
if vpk2>0.95: print("!! ②速度异常 %.2f, 不执行"%vpk2); sys.exit(8)
if not send(pts2,"②收回归位·学习路径·顺滑"): sys.exit(3)
err2,end2=settle(HOME,"②收回归位",35)
if err2>0.05:
    print("!! ②未到位(err=%.3f), 慢速补正"%err2,flush=True)
    e=cur_pose(0.1); K=30; pts3=[]
    for k in range(1,K+1):
        d=float(k)/K; s=d*d*(3-2*d); ds=6*d*(1-d)
        p=[e[i]+(HOME[i]-e[i])*s for i in range(6)]
        v=[(HOME[i]-e[i])/2.5*ds for i in range(6)]
        pts3.append({"t":2.5*d,"pos":p,"vel":v})
    send(pts3,"②补正")
    err2,end2=settle(HOME,"②补正",35)

print("ALL DONE ①err=%.4f ②err=%.4f"%(err1,err2),flush=True)
sys.exit(0 if err2<=0.05 else 4)
