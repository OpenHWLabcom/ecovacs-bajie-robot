#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_headfix_point.py — 记录定点头/爪头临界角标定点
用法: python3 record_headfix_point.py --id 1 --label critical_1 [--note "..."]
读取 :8796/joints，计算 FK、夹爪/爪尖/爪子相机位姿与相对桌面（水平面）的角度，
保存 headfix_calib/points.json + pointNN.json + 头/腕快照。
"""
import argparse, json, math, os, time, urllib.request
import struct
import numpy as np
import kin

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'headfix_calib')
# gripper_link -> berxel_arm_rgb_optical_frame（2026-10-21 tf2_echo 实测）
CAM_XYZ = [-0.088, 0.000, -0.001]
CAM_R = np.array([[0.029, 0.966, 0.256],
                  [-1.000, 0.031, -0.004],
                  [-0.012, -0.255, 0.967]])
TIP_LEN = 0.1295

ap = argparse.ArgumentParser()
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--label', default='')
ap.add_argument('--note', default='')
ap.add_argument('--table-z', type=float, default=None, help='桌面在 base_link 的 z（可选）')
ap.add_argument('--side', default='', help='接触侧，如 left/right')
ap.add_argument('--contact-camera', action='store_true', help='相机已接触桌面')
ap.add_argument('--contact-forearm', action='store_true', help='小臂已接触桌面')
ap.add_argument('--contact-gripper', action='store_true', help='夹爪本体/正下方已接触桌面')
ap.add_argument('--contact-motor', action='store_true', help='爪根电机已接触桌面')
A = ap.parse_args()
os.makedirs(OUT, exist_ok=True)


def elev(v):
    v = np.array(v, float); n = np.linalg.norm(v) or 1.0
    return math.degrees(math.asin(max(-1.0, min(1.0, float(v[2] / n)))))


def ang(a, b):
    a = np.array(a, float); b = np.array(b, float)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b)))))


def snap(url, path):
    try:
        with urllib.request.urlopen(url, timeout=4) as r:
            data = r.read()
        if data:
            open(path, 'wb').write(data)
            return os.path.relpath(path, HERE)
    except Exception:
        pass
    return None


j = json.load(urllib.request.urlopen('http://127.0.0.1:8796/joints', timeout=4))
q = [float(x) for x in j['rad'][:6]]
lift = float(j.get('lift') or 0.0)
Tba = kin.T_of([-0.058, -0.06302, 0.48267 + lift], [0, math.pi / 2, 0])
T, grip, frames = kin.fk(q, ret_all=True)
G = Tba @ grip
Tgc = np.eye(4); Tgc[:3, :3] = CAM_R; Tgc[:3, 3] = CAM_XYZ
C = G @ Tgc
zax, yax, xax = G[:3, 2], G[:3, 1], G[:3, 0]
tip = G[:3, 3] + zax * TIP_LEN
camx, camy, camz = C[:3, 0], C[:3, 1], C[:3, 2]


def _load_stl(name):
    try:
        d = open(os.path.join(HERE, 'meshes', name), 'rb').read()
        n = struct.unpack('<I', d[80:84])[0]
        arr = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
        return arr[:, 12:48].copy().view('<f4').reshape(n, 3, 3).reshape(-1, 3)
    except Exception:
        return None


def _mesh_min_z(name, Tf):
    v = _load_stl(name)
    if v is None:
        return None
    w = v @ Tf[:3, :3].T + Tf[:3, 3]
    return float(w[:, 2].min())


gripper_mesh_min = _mesh_min_z('gripper_link.STL', G)
arm6_mesh_min = None
try:
    arm6_mesh_min = _mesh_min_z('arm6_link.STL', Tba @ frames[5])
except Exception:
    pass
# 小臂（joint3→joint4 连杆）中心线最低点；减连杆半径 0.045 估计桌面高度
fore_min = None
try:
    a = (Tba @ frames[2])[:3, 3]
    b = (Tba @ frames[3])[:3, 3]
    fore_min = min(float((a + (b - a) * (k / 20.0))[2]) for k in range(21))
except Exception:
    pass
table_z = A.table_z
table_src = 'given' if table_z is not None else None
if table_z is None and fore_min is not None:
    table_z = fore_min - 0.045
    table_src = 'estimated from forearm centerline min z - 0.045'
stamp = time.strftime('%Y%m%d-%H%M%S')
tag = 'point%02d' % A.id
img_h = snap('http://127.0.0.1:8799/snap.jpg', os.path.join(OUT, '%s_head_%s.jpg' % (tag, stamp)))
img_w = snap('http://127.0.0.1:8800/snap.jpg', os.path.join(OUT, '%s_wrist_%s.jpg' % (tag, stamp)))
tool_elev = elev(zax)
gy_elev = elev(yax)
cy_elev = elev(camy)
rec = {
    'id': A.id, 'label': A.label or tag, 'note': A.note,
    'timestamp': time.strftime('%F %T'), 'q_rad': [round(x, 5) for x in q],
    'q_deg': [round(x * 180 / math.pi, 2) for x in q],
    'lift': round(lift, 4), 'finger': j.get('finger'),
    'gripper_pos': [round(float(x), 4) for x in G[:3, 3]],
    'tip_pos': [round(float(x), 4) for x in tip],
    'camera_pos': [round(float(x), 4) for x in C[:3, 3]],
    'tool_axis': [round(float(x), 4) for x in zax],
    'tool_elev_deg': round(tool_elev, 2),
    'tool_vs_vertical_deg': round(ang(zax, [0, 0, 1]), 2),
    'gripper_axes_elev_deg': {'x': round(elev(xax), 2), 'y': round(gy_elev, 2), 'z': round(tool_elev, 2)},
    'camera_axes': {'x': [round(float(x), 3) for x in camx], 'y': [round(float(x), 3) for x in camy], 'z': [round(float(x), 3) for x in camz]},
    'camera_axes_elev_deg': {'x': round(elev(camx), 2), 'y': round(cy_elev, 2), 'z': round(elev(camz), 2)},
    'camera_opt_elev_deg': round(elev(camz), 2),
    'gripper_face_angle_to_table_deg': round(90 - abs(gy_elev), 2),
    'camera_face_angle_to_table_deg': round(90 - abs(cy_elev), 2),
    'camera_height_above_gripper': round(float(C[2, 3] - G[2, 3]), 4),
    'contact': {'side': A.side, 'camera': bool(A.contact_camera), 'forearm': bool(A.contact_forearm),
                'gripper': bool(A.contact_gripper), 'motor': bool(A.contact_motor)},
    'forearm_min_z': round(fore_min, 4) if fore_min is not None else None,
    'gripper_mesh_min_z': round(gripper_mesh_min, 4) if gripper_mesh_min is not None else None,
    'arm6_mesh_min_z': round(arm6_mesh_min, 4) if arm6_mesh_min is not None else None,
    'gripper_x_face_angle_to_table_deg': round(90 - abs(elev(xax)), 2),
    'gripper_y_face_angle_to_table_deg': round(90 - abs(gy_elev), 2),
    'table_z': round(float(table_z), 4) if table_z is not None else None,
    'table_z_source': table_src,
    'tip_height_above_table': round(float(tip[2] - table_z), 4) if table_z is not None else None,
    'camera_origin_height_above_table': round(float(C[2, 3] - table_z), 4) if table_z is not None else None,
    'camera_tf_gripper': {'xyz': CAM_XYZ, 'rpy_deg': [-14.801, 0.693, -88.331]},
    'images': {'head': img_h, 'wrist': img_w},
}
json.dump(rec, open(os.path.join(OUT, '%s_%s.json' % (tag, stamp)), 'w'), ensure_ascii=False, indent=2)
pfile = os.path.join(OUT, 'points.json')
try:
    pts = json.load(open(pfile))
    if not isinstance(pts, list): pts = []
except Exception:
    pts = []
pts = [p for p in pts if p.get('id') != A.id]
pts.append(rec)
pts.sort(key=lambda p: p.get('id', 0))
json.dump(pts, open(pfile, 'w'), ensure_ascii=False, indent=2)
try:
    import limits_db
    limits_db.upsert(rec)
except Exception as e:
    print('limits db upsert failed:', e)
print(json.dumps(rec, ensure_ascii=False, indent=2))
