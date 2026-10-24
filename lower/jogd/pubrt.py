#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pubrt.py — 测试向 /arm/rt/joint_angle_cmd_collision 发布关节角
# 用法: pubrt.py <t1,..,t6> [rate_hz=30] [ramp_s=1.0] [hold_s=0.5]
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
NAMES = ["joint%d" % i for i in range(1, 7)]
tgt = [float(x) for x in sys.argv[1].split(",")]
rate = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
ramp = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
hold = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5

rclpy.init()
node = Node("pubrt")
pub = node.create_publisher(JointState, "/arm/rt/joint_angle_cmd_collision", 10)
fb = {"p": None}
def cb(m):
    d = dict(zip(m.name, m.position))
    v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v):
        fb["p"] = [float(x) for x in v]
node.create_subscription(JointState, "/joint_states", cb, 50)
t0 = time.time()
while fb["p"] is None and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
if fb["p"] is None:
    print("no fb"); sys.exit(2)
cur = list(fb["p"]); print("cur", ["%.4f" % x for x in cur], flush=True)
dt = 1.0 / rate
t0 = time.time()
n = 0
while True:
    x = time.time() - t0
    a = min(1.0, x / ramp)
    pos = [cur[i] + (tgt[i] - cur[i]) * a for i in range(6)]
    m = JointState()
    m.header.frame_id = "e2e_movej"
    m.name = NAMES
    m.position = pos
    pub.publish(m); n += 1
    if x >= ramp + hold:
        break
    time.sleep(dt)
print("sent %d msgs" % n, flush=True)
