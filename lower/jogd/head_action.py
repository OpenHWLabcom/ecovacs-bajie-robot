#!/usr/bin/env python3
import os, sys
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI','/home/Agi/jogd/dds_hi.xml')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from common_msgs.action import RobotControl
val=float(sys.argv[1]) if len(sys.argv)>1 else 0.0
rclpy.init(); n=Node('head_action_once')
ac=ActionClient(n, RobotControl, '/robot/control')
if not ac.wait_for_server(timeout_sec=4):
    print('ERR no server'); sys.exit(2)
g=RobotControl.Goal(); g.task_type=2; g.pitch_angle=val; g.lift_dis=0.0
fut=ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=4)
gh=fut.result()
if not (gh and gh.accepted): print('ERR rejected'); sys.exit(3)
rf=gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
res=rf.result()
print('OK' if (res and res.result.error_code==0) else 'FAIL', res.result.error_code if res else -1, res.result.message if res else '')
