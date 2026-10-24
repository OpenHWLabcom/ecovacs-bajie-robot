import time, os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
FLAGS=['/home/Agi/jogd/petgrip.on','/tmp/petgrip.on']
def flag_on():
    for f in FLAGS:
        if os.path.exists(f): return True
    return False
rclpy.init(); n=Node('petgripd')
latest={"pos":None}
def on_js(m):
    d=dict(zip(m.name,m.position))
    a=[d.get("joint%d"%i) for i in range(1,7)]
    if all(v is not None for v in a):
        latest["pos"]=[float(v) for v in a]
n.create_subscription(JointState,'/joint_states',on_js,10)
pub=n.create_publisher(JointState,'/actuator/motor/joint/ctrl',10)
def spin(t):
    e=time.time()+t
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.02)
def make(v):
    m=JointState(); m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id="new_arm_base_link"
    m.name=["joint1","joint2","joint3","joint4","joint5","joint6","arm_finger_joint1"]
    m.position=list(latest["pos"][:6])+[float(v)]
    m.velocity=[0.0]*7; m.effort=[0.0]*7
    return m
def setf(v,reps=3):
    if latest["pos"] is None: return False
    for i in range(reps):
        pub.publish(make(v)); spin(0.12)
    return True
def dwell(sec, v):
    t0=time.time(); ra=False
    while time.time()-t0<sec:
        rclpy.spin_once(n,timeout_sec=0.05)
        if not flag_on(): return False
        if not ra and time.time()-t0>sec*0.5:
            ra=True
            if latest["pos"] is not None:
                pub.publish(make(v)); spin(0.05)
    return True
print('petgripd v3 dual-flag',flush=True)
while True:
    if not flag_on(): time.sleep(0.4); continue
    try:
        src=[f for f in FLAGS if os.path.exists(f)][0]
        txt=open(src).read().split()
        o=float(txt[0]) if len(txt)>0 else 2.2
        c=float(txt[1]) if len(txt)>1 else 2.6
    except Exception: o,c=2.2,2.6
    ok=setf(0.69); print('open %s'%ok,flush=True)
    if not dwell(o,0.69): continue
    ok=setf(0.0); print('close %s'%ok,flush=True)
    if not dwell(c,0.0): continue
