#!/usr/bin/env python3
# set_grip.py <value> — 设置夹爪开度（发布到 /actuator/motor/joint/ctrl；value 0.0~0.69）
# 用法示例: python3 set_grip.py 0.183
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.183
rclpy.init(); n = Node('set_grip')
latest = {"pos": None}
def on_js(m):
    d = dict(zip(m.name, m.position))
    a = [d.get("joint%d" % i) for i in range(1,7)]
    if all(x is not None for x in a): latest["pos"] = [float(x) for x in a]
n.create_subscription(JointState, '/joint_states', on_js, 50)
pub = n.create_publisher(JointState, '/actuator/motor/joint/ctrl', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
t0 = time.time()
while latest["pos"] is None and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
if latest["pos"] is None:
    print("no joints"); sys.exit(2)
for i in range(6):
    m = JointState(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = "new_arm_base_link"
    m.name = ["joint1","joint2","joint3","joint4","joint5","joint6","arm_finger_joint1"]
    m.position = list(latest["pos"][:6]) + [v]
    m.velocity = [0.0]*7; m.effort = [0.0]*7
    pub.publish(m); spin(0.15)
spin(0.8)
print("grip set to", v)
