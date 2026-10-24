#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init(); n=Node("finger")
d={}
def cb(m):
    dd=dict(zip(m.name,m.position))
    if "arm_finger_joint1" in dd: d["f"]=float(dd["arm_finger_joint1"])
n.create_subscription(JointState,"/joint_states",cb,50)
t0=time.time()
while "f" not in d and time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.2)
print("%.4f"%d.get("f",-1))
