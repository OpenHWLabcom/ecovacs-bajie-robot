#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""limits_db.py — 定点头/爪头限位数据库（SQLite）
数据源：headfix_calib/points.json（record_headfix_point.py 追加）
用法：python3 limits_db.py sync|list|stats
"""
import json, os, sqlite3, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(HERE, 'headfix_calib')
DB = os.path.join(CAL, 'limits.db')
POINTS = os.path.join(CAL, 'points.json')

SCHEMA = """
CREATE TABLE IF NOT EXISTS limit_points (
  id INTEGER PRIMARY KEY,
  label TEXT, ts TEXT, side TEXT,
  contact_camera INTEGER, contact_forearm INTEGER, contact_gripper INTEGER, contact_motor INTEGER,
  q_rad TEXT, q_deg TEXT, lift REAL, finger REAL,
  tool_elev REAL, grip_x_face REAL, grip_y_face REAL, cam_opt_elev REAL,
  limit_angle REAL,
  tip_z REAL, cam_z REAL, tip_above_table REAL, cam_above_table REAL, table_z REAL,
  note TEXT, images TEXT, raw TEXT, updated TEXT
);
"""

COLS = ['id','label','ts','side','contact_camera','contact_forearm','contact_gripper','contact_motor',
        'q_rad','q_deg','lift','finger','tool_elev','grip_x_face','grip_y_face','cam_opt_elev',
        'limit_angle',
        'tip_z','cam_z','tip_above_table','cam_above_table','table_z','note','images','raw','updated']


def conn():
    os.makedirs(CAL, exist_ok=True)
    c = sqlite3.connect(DB, timeout=10)
    c.row_factory = sqlite3.Row
    c.execute(SCHEMA)
    try:
        c.execute('ALTER TABLE limit_points ADD COLUMN limit_angle REAL')
    except sqlite3.OperationalError:
        pass
    c.commit()
    return c


def _f(v):
    try:
        return float(v) if v is not None else None
    except Exception:
        return None


def _row(rec):
    ct = rec.get('contact') or {}
    imgs = rec.get('images') or {}
    qr = rec.get('q_rad') or []
    qd = rec.get('q_deg') or []
    tip = rec.get('tip_pos') or [None, None, None]
    cam = rec.get('camera_pos') or [None, None, None]
    gx = _f(rec.get('gripper_x_face_angle_to_table_deg'))
    ax = rec.get('gripper_axes_elev_deg') or {}
    if gx is None and ax.get('x') is not None:
        gx = 90 - abs(float(ax['x']))
    gy = _f(rec.get('gripper_y_face_angle_to_table_deg') or rec.get('gripper_face_angle_to_table_deg'))
    if gy is None and ax.get('y') is not None:
        gy = 90 - abs(float(ax['y']))
    if gx is not None:
        gx = round(gx, 2)
    if gy is not None:
        gy = round(gy, 2)
    lim = min([v for v in (gx, gy) if v is not None], default=None)
    return {
        'id': int(rec.get('id') or 0),
        'label': str(rec.get('label') or ''),
        'ts': str(rec.get('timestamp') or ''),
        'side': str(rec.get('contact', {}).get('side') or rec.get('side') or ''),
        'contact_camera': 1 if ct.get('camera') else 0,
        'contact_forearm': 1 if ct.get('forearm') else 0,
        'contact_gripper': 1 if ct.get('gripper') else 0,
        'contact_motor': 1 if ct.get('motor') else 0,
        'q_rad': json.dumps(qr, ensure_ascii=False),
        'q_deg': json.dumps(qd, ensure_ascii=False),
        'lift': _f(rec.get('lift')),
        'finger': _f(rec.get('finger')),
        'tool_elev': _f(rec.get('tool_elev_deg')),
        'grip_x_face': gx,
        'grip_y_face': gy,
        'cam_opt_elev': _f(rec.get('camera_opt_elev_deg')),
        'limit_angle': lim,
        'tip_z': _f(tip[2] if len(tip) > 2 else None),
        'cam_z': _f(cam[2] if len(cam) > 2 else None),
        'tip_above_table': _f(rec.get('tip_height_above_table')),
        'cam_above_table': _f(rec.get('camera_origin_height_above_table')),
        'table_z': _f(rec.get('table_z')),
        'note': str(rec.get('note') or ''),
        'images': json.dumps(imgs, ensure_ascii=False),
        'raw': json.dumps(rec, ensure_ascii=False),
        'updated': time.strftime('%F %T'),
    }


def upsert(rec):
    r = _row(rec)
    if not r['id']:
        return False
    c = conn()
    try:
        c.execute('INSERT OR REPLACE INTO limit_points (%s) VALUES (%s)' % (','.join(COLS), ','.join(['?'] * len(COLS))),
                  [r[k] for k in COLS])
        c.commit()
        return True
    finally:
        c.close()


def sync():
    try:
        pts = json.load(open(POINTS))
    except Exception:
        pts = []
    n = 0
    for p in pts:
        if upsert(p):
            n += 1
    return n


def _out(row):
    d = dict(row)
    for k in ('q_rad', 'q_deg', 'images', 'raw'):
        try:
            d[k] = json.loads(d[k]) if d.get(k) else ([] if k in ('q_rad', 'q_deg') else {})
        except Exception:
            pass
    for k in ('contact_camera', 'contact_forearm', 'contact_gripper', 'contact_motor'):
        d[k] = bool(d.get(k))
    return d


def list_points():
    c = conn()
    try:
        return [_out(r) for r in c.execute('SELECT * FROM limit_points ORDER BY id ASC')]
    finally:
        c.close()


def stats():
    pts = list_points()
    ang = [p.get('limit_angle') for p in pts if p.get('limit_angle') is not None]
    sides = {}
    for p in pts:
        sides[p.get('side') or '?'] = sides.get(p.get('side') or '?', 0) + 1
    return {'points': len(pts), 'sides': sides,
            'angle_min': min(ang) if ang else None, 'angle_max': max(ang) if ang else None,
            'last_update': max([p.get('updated') or '' for p in pts], default='')}


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'sync'
    if cmd == 'sync':
        print('synced', sync(), 'points ->', DB)
    elif cmd == 'list':
        print(json.dumps(list_points(), ensure_ascii=False, indent=2))
    elif cmd == 'stats':
        print(json.dumps(stats(), ensure_ascii=False, indent=2))
    else:
        print('usage: limits_db.py sync|list|stats')
