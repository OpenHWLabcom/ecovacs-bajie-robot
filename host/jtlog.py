#!/usr/bin/env python3
# 记录 /arm_controller/joint_trajectory 原始消息（抓输入格式）
import sys, time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
out = open(sys.argv[1] if len(sys.argv) > 1 else "/home/Agi/jtlog.txt", "a")
n = [0]
def cb(m):
    n[0] += 1
    try:
        pts = m.points
        tk = time.time()
        p0 = pts[0]
        tf0 = p0.time_from_start.sec + p0.time_from_start.nanosec / 1e9
        tfl = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec / 1e9
        mx = 0.0
        for k in range(1, len(pts)):
            a = pts[k].positions; b = pts[k-1].positions
            d = max(abs(a[j]-b[j]) for j in range(min(len(a),len(b))))
            if d > mx: mx = d
        v = list(p0.velocities)
        vmax0 = max([abs(x) for x in v]) if v else -1.0
        out.write("MSG %.3f n=%d tf0=%.3f tfN=%.3f maxstep=%.4f vmax0=%.3f\n" % (tk, len(pts), tf0, tfl, mx, vmax0))
        if n[0] % 20 == 1:
            for k in range(min(6, len(pts))):
                pp = pts[k]
                tf = pp.time_from_start.sec + pp.time_from_start.nanosec / 1e9
                pv = list(pp.velocities) or [0.0]*6
                out.write("   pt%d tf=%.3f pos=%s vel=%s\n" % (k, tf,
                    ",".join("%.4f" % x for x in pp.positions[:6]),
                    ",".join("%.3f" % x for x in pv[:6])))
        out.flush()
    except Exception as e:
        try: out.write("ERR %s\n" % e); out.flush()
        except Exception: pass
rclpy.init(); node = Node("jtlog")
node.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory", cb, 200)
from rclpy.executors import SingleThreadedExecutor
ex = SingleThreadedExecutor(); ex.add_node(node)
try: ex.spin()
except KeyboardInterrupt: pass
