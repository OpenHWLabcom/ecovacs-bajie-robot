#!/usr/bin/env python3
# samp3.py — 100Hz 全关节(含夹爪/升降/云台)记录器 (自带ROS环境)
import os
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI','file:///home/Agi/jogd/dds_hi.xml')
import rclpy, time, sys
from rclpy.node import Node
from sensor_msgs.msg import JointState
OUT, DUR = sys.argv[1], float(sys.argv[2])
KEYS = ['joint1','joint2','joint3','joint4','joint5','joint6','arm_finger_joint1','lift_joint','pitch_joint']
rclpy.init(); n = Node('samp3'); f = open(OUT, 'a', buffering=1)
last = [0.0]
def cb(m):
    t = time.time()
    if t - last[0] < 0.009: return
    last[0] = t
    d = dict(zip(m.name, m.position))
    f.write("S %.3f %s\n" % (t, ' '.join('%.4f' % d.get(k, float('nan')) for k in KEYS)))
n.create_subscription(JointState, '/joint_states', cb, 100)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.5)
f.close()
