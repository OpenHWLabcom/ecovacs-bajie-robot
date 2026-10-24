#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# samp2.py — 采集多路数据到 CSV: S=/joint_states C=/actuator/motor/joint/ctrl R=/arm/rt/joint_angle_cmd_collision A=/arm/rt/joint_angle_cmd
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
NAMES = ["joint%d" % i for i in range(1, 7)]
secs = float(sys.argv[1]); out = sys.argv[2]
rclpy.init(); n = Node("samp2"); f = open(out, "w"); lock = threading.Lock()
def mk(tag):
    def cb(m):
        d = dict(zip(m.name, m.position))
        p = [d.get(x) for x in NAMES]
        if any(x is None for x in p): return
        vd = dict(zip(m.name, m.velocity)) if len(m.velocity) == len(m.name) else {}
        ed = dict(zip(m.name, m.effort)) if len(m.effort) == len(m.name) else {}
        v = [vd.get(x, 0.0) for x in NAMES]; e = [ed.get(x, 0.0) for x in NAMES]
        lift = d.get("lift_joint")
        line = "%s %.4f %s %s %s %s\n" % (tag, time.time(),
            " ".join("%.5f" % x for x in p),
            ("%.5f" % lift if lift is not None else "x"),
            " ".join("%.4f" % x for x in v),
            " ".join("%.4f" % x for x in e))
        with lock: f.write(line)
    return cb
n.create_subscription(JointState, "/joint_states", mk("S"), 200)
n.create_subscription(JointState, "/actuator/motor/joint/ctrl", mk("C"), 100)
n.create_subscription(JointState, "/arm/rt/joint_angle_cmd_collision", mk("R"), 100)
n.create_subscription(JointState, "/arm/rt/joint_angle_cmd", mk("A"), 100)
ex = SingleThreadedExecutor(); ex.add_node(n)
t = threading.Timer(secs, ex.shutdown); t.daemon = True; t.start()
try: ex.spin()
except Exception: pass
f.flush(); f.close(); print("SAMPDONE", out)
