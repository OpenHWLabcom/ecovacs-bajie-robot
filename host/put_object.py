#!/usr/bin/env python3
# put_object.py name cx cy cz sx sy sz yaw [conf]
# 把识别到的物体写进 objects.json（arm3d v54 会周期读取并画到三维场景里）
import json, sys, time, os
d = os.path.dirname(os.path.abspath(__file__))
if len(sys.argv) >= 9:
    name = sys.argv[1]
    c = [float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
    s = [float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])]
    yaw = float(sys.argv[8])
    conf = float(sys.argv[9]) if len(sys.argv) > 9 else 0.9
    o = {"name": name, "center": c, "size": s, "yaw": yaw, "conf": conf, "frame": "base_link"}
    data = {"ts": time.time(), "objects": [o]}
else:
    data = {"ts": time.time(), "objects": []}
json.dump(data, open(os.path.join(d, "objects.json"), "w"), ensure_ascii=False)
print(json.dumps(data, ensure_ascii=False))
