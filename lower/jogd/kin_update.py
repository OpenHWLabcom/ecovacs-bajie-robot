#!/usr/bin/env python3
import os, json, time, math
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI','/home/Agi/jogd/dds_hi.xml')
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
rclpy.init(); n=Node('kin_update')
buf=tf2_ros.Buffer(); lst=tf2_ros.TransformListener(buf,n)
js={'q':[0.0]*6}
def cb(m):
    try: js['q']=[float(x) for x in m.position[:6]]
    except Exception: pass
n.create_subscription(JointState,'/joint_states',cb,10)
t=time.time()
while time.time()-t<3: rclpy.spin_once(n, timeout_sec=0.1)
def rpy(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1,min(1,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return [roll,pitch,yaw]
def get(f):
    tr=buf.lookup_transform('base_link',f,rclpy.time.Time())
    p=tr.transform.translation; return [[p.x,p.y,p.z],rpy(tr.transform.rotation)]
st={'joints':js['q'],'arm_base':get('arm_base_link'),'objects':get('objects_link')}
for p in ('/home/Agi/jogd/kin_state.json','/tmp/kin_state.json'):
    json.dump(st,open(p,'w'))
print(json.dumps(st))
