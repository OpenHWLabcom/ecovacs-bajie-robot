#!/usr/bin/env python3
import os, sys, time, struct
os.environ.setdefault("RMW_IMPLEMENTATION","rmw_cyclonedds_cpp")
os.environ.setdefault("CYCLONEDDS_URI","file:///home/Agi/jogd/dds_hi.xml")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2, numpy as np
FPS=float(os.environ.get("CAM_FPS","8"))
Q=int(os.environ.get("CAM_Q","55"))
W=int(os.environ.get("CAM_W","480"))
H=int(os.environ.get("CAM_H","300"))
TOPIC=os.environ.get("CAM_TOPIC","/berxel_upper/berxel_upper/color/image_raw")
rclpy.init(); n=Node('jetson_camfeed')
latest={'img':None}
def cb(m): latest['img']=m
n.create_subscription(Image,TOPIC,cb,5)
out=sys.stdout.buffer
t_last=0.0
try:
    while rclpy.ok():
        rclpy.spin_once(n,timeout_sec=0.03)
        m=latest['img']
        if m is None: continue
        now=time.time()
        if now-t_last < 1.0/FPS: continue
        t_last=now
        buf=np.frombuffer(bytes(m.data),dtype=np.uint8)
        if m.encoding.lower() in ("rgb8","bgr8"):
            img=buf.reshape(m.height,m.width,3)
            bgr=img if m.encoding.lower()=="bgr8" else cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        else:
            bgr=buf.reshape(m.height,m.width,-1)[:,:,:3]
            bgr=cv2.cvtColor(bgr,cv2.COLOR_RGB2BGR)
        if W>0 and (bgr.shape[1]!=W or bgr.shape[0]!=H):
            bgr=cv2.resize(bgr,(W,H),interpolation=cv2.INTER_AREA)
        ok,jpg=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,Q])
        if not ok: continue
        data=jpg.tobytes()
        out.write(struct.pack(">I",len(data))+data); out.flush()
except (BrokenPipeError, KeyboardInterrupt):
    pass
