#!/usr/bin/env python3
"""book_place_tray.py — 把夹住的书放到前置托盘（分两步，先到托盘上方）
用法（下板）:
  python3 book_place_tray.py pre [x y z]     # 到托盘上方（默认 0.34,0,0.52），打印手腕相机平面
  python3 book_place_tray.py drop [x y z]    # 下降到放置位并松爪
"""
import sys, time
sys.path.insert(0, '/home/Agi/jogd')
from grasp2 import Grasper
import robotclient as pk

def log(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)

def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else 'pre'
    x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.34
    y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    z = float(sys.argv[4]) if len(sys.argv) > 4 else (0.52 if stage == 'pre' else 0.40)
    g = Grasper()
    jaw = (0, 1)
    q, info = g.ik(x, y, z, jaw, (0, 0, -1))
    if q is None:
        log('❌ IK 失败', info); return 4
    log('%s 目标 (%.2f,%.2f,%.2f) IK ok col=%.3f tilt=%.2f' % (stage, x, y, z, info.get('col', 9), info.get('tilt', 0)))
    r = g.go(q, 0.18 if stage == 'pre' else 0.08)
    if not r.get('ok'):
        log('❌ 运动失败', r); return 5
    log('✓ 到位')
    if stage == 'pre':
        try:
            w = pk.get('/wrist')
            log('手腕 floor=%.4f' % w.get('floor', -9))
            for o in w.get('objects', [])[:5]:
                log('  腕部候选 c=(%.2f,%.2f) zmax=%.3f size=%s' % (o['c'][0], o['c'][1], o['zmax'], [round(v,3) for v in o['size']]))
        except Exception as e:
            log('腕部读取失败', str(e)[:80])
        return 0
    # drop
    g.grip(0.52, settle=1.2)
    log('✓ 已松爪')
    q2, _ = g.ik(x, y, z + 0.12, jaw, (0, 0, -1))
    if q2 is not None:
        g.go(q2, 0.12)
    return 0

if __name__ == '__main__':
    sys.exit(main())
