#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# jtstream.py — 低开销 /joint_states → 行流（bridge 读取；替代重型 ros2 CLI echo）
# 输出行: "D p1,p2,p3,p4,p5,p6,lift,v1..v6,e1..e6"  (lift 无 → "x")
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

NAMES = ["joint%d" % i for i in range(1, 7)]
rclpy.init(); node = Node("jtstream")
out = sys.stdout
lastv = [None]; lastt = [0.0]

def cb(m):
    pd = dict(zip(m.name, m.position))
    # 夹爪单独一条消息：输出 F 行给 bridge
    if "arm_finger_joint1" in pd:
        out.write("F %.5f\n" % float(pd["arm_finger_joint1"]))
        out.flush()
        return
    if "lift_joint" in pd:
        out.write("L %.5f\n" % float(pd["lift_joint"]))
        out.flush()
        return
    if "pitch_joint" in pd:
        out.write("P %.5f\n" % float(pd["pitch_joint"]))
        out.flush()
        return
    p = [pd.get(n) for n in NAMES]
    if any(x is None for x in p):
        return
    vd = dict(zip(m.name, m.velocity)) if len(m.velocity) == len(m.name) else {}
    ed = dict(zip(m.name, m.effort)) if len(m.effort) == len(m.name) else {}
    lift = pd.get("lift_joint")
    v = [vd.get(n, 0.0) for n in NAMES]
    e = [ed.get(n, 0.0) for n in NAMES]
    key = tuple(round(x, 5) for x in p) + (round(lift, 5) if lift is not None else None,)
    now = time.time()
    changed = key != lastv[0]
    if (not changed and now - lastt[0] < 0.25) or (changed and now - lastt[0] < 0.02):
        return
    lastv[0] = key; lastt[0] = now
    out.write("D " + ",".join("%.5f" % x for x in p)
              + (",%.5f" % lift if lift is not None else ",x")
              + "," + ",".join("%.4f" % x for x in v)
              + "," + ",".join("%.4f" % x for x in e) + "\n")
    out.flush()

node.create_subscription(JointState, "/joint_states", cb, 50)
ex = SingleThreadedExecutor(); ex.add_node(node)
try:
    ex.spin()
except KeyboardInterrupt:
    pass
