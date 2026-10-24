#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# /home/Agi/jogd/samp.py — 采样 /joint_states 到 CSV：时间 6关节(rad)
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

NAMES = ["joint%d" % i for i in range(1, 7)]
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/samp.csv"

rclpy.init()
node = Node("samp")
f = open(out, "w")
n = [0]

def cb(m):
    d = dict(zip(m.name, m.position))
    v = [d.get(x) for x in NAMES]
    if all(x is not None for x in v):
        f.write("%.4f %.5f %.5f %.5f %.5f %.5f %.5f\n" % (time.time(), *[float(x) for x in v]))
        n[0] += 1

node.create_subscription(JointState, "/joint_states", cb, 300)
ex = SingleThreadedExecutor(); ex.add_node(node)
timer = threading.Timer(secs, ex.shutdown); timer.daemon = True; timer.start()
try:
    ex.spin()
except Exception:
    pass
f.flush(); f.close()
print("SAMPDONE %d %s" % (n[0], out))
