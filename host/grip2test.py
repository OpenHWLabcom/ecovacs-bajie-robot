#!/usr/bin/env python3
# grip2test.py — 夹爪官方通道单次验证：开→关→(读回夹爪位置)
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from common_msgs.action import CentaurBodyTask
rclpy.init(); n = Node('grip2test')
fb = {'f': None}
def on_js(m):
    d = dict(zip(m.name, m.position))
    if 'arm_finger_joint1' in d: fb['f'] = float(d['arm_finger_joint1'])
n.create_subscription(JointState, '/joint_states', on_js, 50)
ac = ActionClient(n, CentaurBodyTask, '/centaur_body/task')
def cur():
    t0 = time.time()
    while fb['f'] is None and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
    return fb['f']
def send(tt):
    if not ac.wait_for_server(timeout_sec=8): print('NO SERVER'); return False
    g = CentaurBodyTask.Goal(); g.task_type = tt; g.task_id = 0
    fut = ac.send_goal_async(g)
    t0 = time.time()
    while not fut.done() and time.time()-t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
    gh = fut.result()
    if not (gh and gh.accepted): print('tt=%d rejected' % tt, flush=True); return False
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done() and time.time()-t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
    if rf.done():
        try: print('tt=%d result=%s msg=%s' % (tt, rf.result().result.result, rf.result().result.message), flush=True)
        except Exception as e: print('result raw:', e, flush=True)
        return True
    print('tt=%d timeout' % tt, flush=True); return False
print('finger before:', cur(), flush=True)
send(6); time.sleep(1.0); print('finger after OPEN(6):', cur(), flush=True)
send(7); time.sleep(1.0); print('finger after CLOSE(7):', cur(), flush=True)
