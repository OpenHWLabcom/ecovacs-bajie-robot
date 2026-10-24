#!/usr/bin/env python3
"""box3d.py — 标注框 → base_link 3D 点
用法: python3 box3d.py head|grip x1 y1 x2 y2 imgw imgh
"""
import os, sys, time, json, math
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
os.environ.setdefault('CYCLONEDDS_URI','/home/Agi/jogd/dds_hi.xml')
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros

cam=sys.argv[1] if len(sys.argv)>1 else 'head'
x1,y1,x2,y2=[float(v) for v in sys.argv[2:6]]
imgw,imgh=[float(v) for v in sys.argv[6:8]]
HEAD='head'
TOPIC_D='/berxel_upper/berxel_upper/aligned_depth_to_color/image_raw' if cam==HEAD else '/berxel_arm/berxel_arm/aligned_depth_to_color/image_raw'
TOPIC_I='/berxel_upper/berxel_upper/color/camera_info' if cam==HEAD else '/berxel_arm/berxel_arm/color/camera_info'
FRAME='berxel_upper_rgb_optical_frame' if cam==HEAD else 'berxel_arm_rgb_optical_frame'
rclpy.init(); n=Node('box3d')
got={}
n.create_subscription(Image,TOPIC_D,lambda m: got.__setitem__('d',m),2)
n.create_subscription(CameraInfo,TOPIC_I,lambda m: got.__setitem__('i',m),2)
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
t=time.time()
while time.time()-t<4 and not ('d' in got and 'i' in got):
    rclpy.spin_once(n,timeout_sec=0.1)
if not ('d' in got and 'i' in got):
    print(json.dumps({'ok':False,'err':'no depth/info'})); sys.exit(2)
m=got['d']; H,W=m.height,m.width
info=got['i']; fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
try:
    a=np.frombuffer(m.data,dtype=np.uint16).reshape(H,W).astype(np.float32)/1000.0
except Exception as e:
    print(json.dumps({'ok':False,'err':'depth decode '+str(e)})); sys.exit(3)
sx=W/imgw; sy=H/imgh
u1=max(0,int(x1*sx)); u2=min(W,int(math.ceil(x2*sx)))
v1=max(0,int(y1*sy)); v2=min(H,int(math.ceil(y2*sy)))
if u2<=u1 or v2<=v1:
    print(json.dumps({'ok':False,'err':'empty box','W':W,'H':H})); sys.exit(4)
sub=a[v1:v2,u1:u2]
vs,us=np.nonzero((sub>0.05)&(sub<2.5))
if len(us)<20:
    print(json.dumps({'ok':False,'err':'too few depth pts','n':int(len(us)),'box':[u1,v1,u2,v2],'W':W,'H':H})); sys.exit(5)
z=sub[vs,us]; uu=us+u1; vv=vs+v1
pc=np.stack([(uu-cx)/fx*z,(vv-cy)/fy*z,z],axis=1)
tr=None; last=None
for fr in (FRAME, FRAME.replace('_optical_frame','_frame')):
    for _ in range(30):
        try: rclpy.spin_once(n, timeout_sec=0.05)
        except Exception: pass
        try:
            tr=buf.lookup_transform('base_link',fr,rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.5)); break
        except Exception as e: last=e
    if tr is not None: break
if tr is None:
    print(json.dumps({'ok':False,'err':'tf '+str(last)[:100]})); sys.exit(6)
q=tr.transform.rotation; tx,ty,tz=tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z
R=np.array([[1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w)],
            [2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w)],
            [2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)]])
pb=pc@R.T+np.array([tx,ty,tz])
z70=float(np.percentile(pb[:,2],70)); sel=pb[:,2]>=z70-0.012
ps=pb[sel] if int(sel.sum())>=10 else pb
med=np.median(ps,axis=0)
print(json.dumps({'ok':True,'cam':cam,'n':int(len(pb)),'n_top':int(sel.sum()),'xyz':[round(float(v),4) for v in med],
                  'min':[round(float(v),4) for v in pb.min(axis=0)],'max':[round(float(v),4) for v in pb.max(axis=0)],
                  'z70':round(z70,4),'depth_box':[u1,v1,u2,v2],'depth_size':[W,H],'patch_size':[imgw,imgh]}))
