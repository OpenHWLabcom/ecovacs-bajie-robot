#!/usr/bin/env python3
# circle_play.py — 顺滑版：出舱 + 收回归位（均匀时间基轨迹）
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

HOME = [0.0, 2.46, 3.256, -0.007, -2.543, 0.0]
OPEN = [0.0, 1.766, 2.887, 0.0, -0.451, 0.0]
NAMES = ["joint%d" % i for i in range(1, 7)]
exit_pts = json.load(open('/home/Agi/sm_exit.json'))
stow_pts = json.load(open('/home/Agi/sm_stow.json'))

rclpy.init(); node = Node("circleplay")
fb = {"p": None, "f": None}
def on_js(m):
    d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]
    if "arm_finger_joint1" in d: fb["f"] = float(d["arm_finger_joint1"])
node.create_subscription(JointState, "/joint_states", on_js, 50)
def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec: rclpy.spin_once(node, timeout_sec=0.1)
def cur_pose(sec=0.4):
    t0 = time.time()
    while fb["p"] is None and time.time() - t0 < 8: rclpy.spin_once(node, timeout_sec=0.1)
    last = fb["p"]; t0 = time.time()
    while time.time() - t0 < sec: rclpy.spin_once(node, timeout_sec=0.1); last = fb["p"]
    return last

spin(0.5)
f = fb["f"]
print("finger=%.3f" % (f if f is not None else -1), flush=True)
if f is None or not (0.13 <= f <= 0.26):
    print("!! 夹子状态不符(0.13~0.26), 停止"); sys.exit(9)
ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10):
    print("NO ACTION SERVER"); sys.exit(2)

def send_path(pts, label):
    cur = cur_pose()
    d0 = pts[0]["pos"]; N = len(pts)
    off = [cur[i] - d0[i] for i in range(6)]
    goal = FollowJointTrajectory.Goal(); traj = JointTrajectory(); traj.joint_names = NAMES
    for k, r in enumerate(pts):
        dec = max(0.0, 1.0 - k / max(1.0, N * 0.25))
        q = JointTrajectoryPoint()
        q.positions = [r["pos"][i] + off[i] * dec for i in range(6)]
        q.velocities = list(r["vel"])
        t = r["t"]; sec = int(t)
        q.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(q)
    goal.trajectory = traj
    print("== %s: %d pts, %.2fs ==" % (label, len(pts), pts[-1]["t"]), flush=True)
    snd = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, snd, timeout_sec=8)
    gh = snd.result()
    if not (gh and gh.accepted):
        print("!! REJECTED"); return False
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(pts[-1]["t"]) + 30)
    return True

def settle(tgt, label, wait=30):
    t0 = time.time(); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
    while err > 0.02 and time.time() - t0 < wait:
        time.sleep(1.2); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
        print("   [%s] settling err=%.3f" % (label, err), flush=True)
    print("== %s done. err=%.4f end=%s" % (label, err, " ".join("%.3f" % x for x in end)), flush=True)
    return err, end

if not send_path(exit_pts, "①出舱·顺滑版"): sys.exit(3)
err1, end1 = settle(OPEN, "①出舱", 30)
if err1 > 0.05:
    print("!! 出舱未到位, 中止"); sys.exit(4)
time.sleep(1.5)

if not send_path(stow_pts, "②收回归位·顺滑版"): sys.exit(3)
err2, end2 = settle(HOME, "②收回归位", 30)
if err2 > 0.05:
    print("!! 收回归位未到位(err=%.3f), 慢速补正" % err2, flush=True)
    end2 = cur_pose(0.1); pts2 = []; K = 30
    for k in range(1, K + 1):
        d = float(k) / K; s = d * d * (3 - 2 * d); ds = 6 * d * (1 - d)
        p = [end2[i] + (HOME[i] - end2[i]) * s for i in range(6)]
        v = [(HOME[i] - end2[i]) / 2.5 * ds for i in range(6)]
        sec = int(2.5 * d)
        pts2.append({"t": 2.5 * d, "pos": p, "vel": v})
    send_path(pts2, "②补正")
    err2, end2 = settle(HOME, "②补正", 30)

print("ALL DONE exit_err=%.4f stow_err=%.4f" % (err1, err2), flush=True)
sys.exit(0 if err2 <= 0.05 else 4)
