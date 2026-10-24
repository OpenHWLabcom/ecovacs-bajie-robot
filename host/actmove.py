#!/usr/bin/env python3
# actmove.py — 用 ACTION(follow_joint_trajectory) 发送多点轨迹（测试）
# 用法: actmove.py <t1,..,t6> [dur=2.0] [np=40]
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

NAMES = ["joint%d" % i for i in range(1, 7)]
tgt = [float(x) for x in sys.argv[1].split(",")]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
NP = int(sys.argv[3]) if len(sys.argv) > 3 else 40

rclpy.init(); node = Node("actmove")
fb = {"p": None}
def on_js(m):
    d = dict(zip(m.name, m.position))
    v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]
node.create_subscription(JointState, "/joint_states", on_js, 50)
t0 = time.time()
while fb["p"] is None and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.1)
if fb["p"] is None: print("no fb"); sys.exit(2)
cur = fb["p"]; print("cur", ["%.3f" % x for x in cur], flush=True)

ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=8): print("no action server"); sys.exit(2)
goal = FollowJointTrajectory.Goal()
traj = JointTrajectory(); traj.joint_names = NAMES
for k in range(1, NP + 1):
    d = float(k) / NP
    s = d * d * (3 - 2 * d)
    ds = 6 * d * (1 - d)
    p = JointTrajectoryPoint()
    p.positions = [cur[i] + (tgt[i] - cur[i]) * s for i in range(6)]
    p.velocities = [(tgt[i] - cur[i]) / dur * ds for i in range(6)]
    tf = dur * d
    sec = int(tf)
    p.time_from_start = Duration(sec=sec, nanosec=int((tf - sec) * 1e9))
    traj.points.append(p)
goal.trajectory = traj
print("sending: %d pts, %.1fs" % (NP, dur), flush=True)
send = ac.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=8)
gh = send.result()
print("handle?", gh is not None, "accepted:", gh.accepted if gh else None, flush=True)
if gh and gh.accepted:
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(dur) + 20)
    print("result:", res.result(), flush=True)
print("done", flush=True)
