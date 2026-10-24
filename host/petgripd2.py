#!/usr/bin/env python3
# petgripd2.py — 夹爪开合循环（宠物模式）· 官方任务通道版（2026-10-17）
# 通道：/centaur_body/task (common_msgs/action/CentaurBodyTask) —— 与 App「夹爪开/关」同一条路
#   task_type: 6 = 夹爪打开(0.690) / 7 = 夹爪关闭(0.000)
# 开关：petgrip.on（/home/Agi/jogd 或 /tmp）存在 → 循环（内容 "开秒数 关秒数"）；不存在 → 空转
# 注意：本进程不向 /actuator/motor/joint/ctrl 写任何东西，不碰 6 轴控制流。
import time, os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from common_msgs.action import CentaurBodyTask

FLAGS = ['/home/Agi/jogd/petgrip.on', '/tmp/petgrip.on']
OPEN, CLOSE = 6, 7

def flag_on():
    for f in FLAGS:
        if os.path.exists(f): return True
    return False

rclpy.init(); n = Node('petgripd2')
ac = ActionClient(n, CentaurBodyTask, '/centaur_body/task')

def spin(t):
    e = time.time() + t
    while time.time() < e:
        try: rclpy.spin_once(n, timeout_sec=0.02)
        except Exception: return

def send_tt(tt, timeout=8.0):
    if not ac.wait_for_server(timeout_sec=5):
        print('no server', flush=True); return False
    g = CentaurBodyTask.Goal(); g.task_type = tt; g.task_id = 0
    fut = ac.send_goal_async(g)
    t0 = time.time()
    while not fut.done() and time.time()-t0 < timeout: rclpy.spin_once(n, timeout_sec=0.05)
    if not fut.done(): print('tt=%d send timeout' % tt, flush=True); return False
    gh = fut.result()
    if not (gh and gh.accepted): print('tt=%d rejected' % tt, flush=True); return False
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done() and time.time()-t0 < timeout: rclpy.spin_once(n, timeout_sec=0.05)
    if not rf.done(): print('tt=%d result timeout' % tt, flush=True); return False
    r = rf.result()
    res = None; msg = ''
    try:
        res = int(r.result.result); msg = str(r.result.message)
    except Exception:
        pass
    ok = (res == 0)
    print('tt=%d result=%s msg=%s ok=%s' % (tt, res, msg, ok), flush=True)
    return ok

print('petgripd2 (official action channel) started', flush=True)
while rclpy.ok():
    if not flag_on():
        time.sleep(0.4); continue
    try:
        src = [f for f in FLAGS if os.path.exists(f)][0]
        txt = open(src).read().split()
        o = float(txt[0]) if len(txt) > 0 else 2.2
        c = float(txt[1]) if len(txt) > 1 else 2.6
    except Exception:
        o, c = 2.2, 2.6
    send_tt(OPEN)
    t0 = time.time()
    while time.time()-t0 < o:
        if not flag_on(): break
        spin(0.2)
    if not flag_on(): continue
    send_tt(CLOSE)
    t0 = time.time()
    while time.time()-t0 < c:
        if not flag_on(): break
        spin(0.2)
