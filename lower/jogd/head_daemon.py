#!/usr/bin/env python3
"""常驻头部俯仰控制：从 stdin 读目标角(rad)，调用 ROS /robot/control (task_type=2)。"""
import os, sys, threading, queue, time
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI','/home/Agi/jogd/dds_hi.xml')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from common_msgs.action import RobotControl

rclpy.init(); node=Node('head_daemon')
ac=ActionClient(node, RobotControl, '/robot/control')
if not ac.wait_for_server(timeout_sec=10):
    print('ERR no server', flush=True); sys.exit(2)
print('READY', flush=True)
q=queue.Queue()
def reader():
    for line in sys.stdin:
        line=line.strip()
        try: q.put(float(line))
        except Exception: pass
    q.put(None)
threading.Thread(target=reader, daemon=True).start()
st={'busy':False,'goal':None}

def on_result(fut):
    st['busy']=False; st['goal']=None
    try:
        r=fut.result()
        print('DONE %s %s %s'%(r.status, r.result.error_code, r.result.message), flush=True)
    except Exception as e:
        print('ERR', str(e)[:80], flush=True)

def on_goal(fut):
    gh=fut.result()
    if not gh or not gh.accepted:
        st['busy']=False; print('REJECT', flush=True); return
    st['goal']=gh
    gh.get_result_async().add_done_callback(on_result)

def tick():
    if st['busy']: return
    try: v=q.get_nowait()
    except queue.Empty: return
    if v is None:
        rclpy.shutdown(); return
    while not q.empty():
        nv=q.get_nowait()
        if nv is None: break
        v=nv
    v=max(0.0,min(1.3,v))
    st['busy']=True
    g=RobotControl.Goal(); g.task_type=2; g.pitch_angle=v; g.lift_dis=0.0
    ac.send_goal_async(g).add_done_callback(on_goal)

node.create_timer(0.05, tick)
rclpy.spin(node)
