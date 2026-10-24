#!/usr/bin/env python3
import sys, time, math
sys.path.insert(0, '/home/Agi/jogd')
from grasp2 import Grasper
import robotclient as pk
def log(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
X,Y,ZMAX = 0.65, -0.13, 0.637
JAW_ANGLE = 170.0
def main():
    g = Grasper()
    jaw = (math.cos(math.radians(JAW_ANGLE)), math.sin(math.radians(JAW_ANGLE)))
    r = g.clear_column(0.35)
    if not r.get('ok'):
        log('❌ 侧摆A失败', r); return 3
    log('✓ 侧摆A')
    z_pre = ZMAX + 0.12
    q_pre, info = g.ik(X, Y, z_pre, jaw, (0,0,-1))
    if q_pre is None:
        z_pre = ZMAX + 0.08
        q_pre, info = g.ik(X, Y, z_pre, jaw, (0,0,-1))
    if q_pre is None:
        log('❌ 预抓取 IK 失败', info); return 4
    log('预抓取 z=%.3f col=%.3f tilt=%.2f' % (z_pre, info.get('col',9), info.get('tilt',0)))
    r = g.go(q_pre, 0.30)
    if not r.get('ok'):
        log('❌ 到预抓取位失败', r); return 5
    log('✓ 到预抓取位')
    z_g = 0.620
    q_g, info = g.ik(X, Y, z_g, jaw, (0,0,-1))
    if q_g is None:
        log('❌ 抓取位 IK 失败', info); return 6
    log('抓取位 z=%.3f col=%.3f' % (z_g, info.get('col',9)))
    g.grip(0.52, settle=1.0)
    r = g.go(q_g, 0.12)
    if not r.get('ok'):
        log('❌ 下降失败', r); return 7
    time.sleep(0.4)
    close = max(0.02, min(0.4, (0.013 - 0.003)/0.111))
    rr = g.grip(close, eff=22.0)
    f1 = rr.get('pos', -1); log('闭合 %.3f finger=%.3f stalled=%s' % (close, f1, rr.get('stalled')))
    q_l, _ = g.ik(X, Y, z_g+0.08, jaw, (0,0,-1))
    if q_l is not None:
        g.go(q_l, 0.10); time.sleep(0.5)
        f2 = g.grip(close, eff=22.0).get('pos', -1)
        log('微抬 finger=%.3f 滑脱=%s' % (f2, abs(f1-f2)>0.03))
        if abs(f1-f2) <= 0.03:
            log('✓✓ 抓取成功（已夹住，待放置）'); return 0
    log('✗ 抓取失败'); return 8
if __name__ == '__main__':
    sys.exit(main())
