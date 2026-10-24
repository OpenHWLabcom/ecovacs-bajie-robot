#!/usr/bin/env python3
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
rclpy.init(); n=Node('griptest')
fb={"f":None}
def on_js(m):
    d=dict(zip(m.name,m.position))
    if "arm_finger_joint1" in d: fb["f"]=float(d["arm_finger_joint1"])
n.create_subscription(JointState,'/joint_states',on_js,50)
ac=ActionClient(n, GripperCommand, '/hand_controller/gripper_cmd')
if not ac.wait_for_server(timeout_sec=8):
    print("NO SERVER"); sys.exit(2)
def cur():
    t0=time.time()
    while fb["f"] is None and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
    return fb["f"]
def send(pos,eff):
    g=GripperCommand.Goal(); g.command.position=pos; g.command.max_effort=eff
    f=ac.send_goal_async(g)
    t0=time.time()
    while not f.done() and time.time()-t0<6: rclpy.spin_once(n,timeout_sec=0.1)
    gh=f.result()
    acc = gh and gh.accepted
    print("send pos=%.3f eff=%.2f accepted=%s"%(pos,eff,acc), flush=True)
    if acc:
        res=gh.get_result_async()
        t0=time.time()
        while not res.done() and time.time()-t0<8: rclpy.spin_once(n,timeout_sec=0.1)
        r=res.result() if res.done() else None
        if r is not None:
            try: print("  result: pos=%.3f eff=%.3f stalled=%s reached=%s"%(r.result.position, r.result.effort, r.result.stalled, r.result.reached_goal), flush=True)
            except Exception as e: print("  result(raw):", r, e)
print("finger before:", cur(), flush=True)
send(0.30, 1.0); time.sleep(2); print("finger after 0.30:", cur(), flush=True)
send(0.12, 1.0); time.sleep(2); print("finger after 0.12:", cur(), flush=True)
send(0.19, 1.0); time.sleep(2); print("finger after 0.19:", cur(), flush=True)
