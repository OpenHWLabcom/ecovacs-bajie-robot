#!/usr/bin/env python3
"""book_grasp_once.py — 单本桌面书：侧摆A → 预抓取 → (可选)夹取 → 报告
用法（下板）:
  python3 book_grasp_once.py dry      # 只到预抓取位，不下降
  python3 book_grasp_once.py grasp    # 全流程抓取并抬起（不放置）
  python3 book_grasp_once.py status   # 只看扫描候选
"""
import sys, time, math, json
sys.path.insert(0, '/home/Agi/jogd')
from grasp2 import Grasper
import robotclient as pk

def log(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)

def candidates():
    d = pk.get('/scan')
    fl = d.get('floor', -0.06)
    out = []
    for o in d.get('objects', []):
        c = o.get('c'); r = o.get('rect') or {}
        if not c or not r: continue
        w, h = r.get('w', 0), r.get('h', 0)
        if min(w, h) <= 0: continue
        long_, short_ = max(w, h), min(w, h)
        zmax = o.get('zmax', -9)
        # 桌面高度范围（相对 base_link）：书顶 0.40~0.60，长边 8~30cm，短边 1~12cm
        if not (0.40 < zmax < 0.60): continue
        if not (0.08 < long_ < 0.30): continue
        if not (0.010 < short_ < 0.12): continue
        if long_ / short_ < 1.4: continue
        out.append(o)
    out.sort(key=lambda o: -o.get('n', 0))
    return fl, out

def jaw_dir(o):
    r = o['rect']; w, h, ang = r['w'], r['h'], r.get('ang', 0)
    a = ang + (90 if w > h else 0)   # 短边方向
    return (math.cos(math.radians(a)), math.sin(math.radians(a))), a


def cached_descend(g, x, y, zmax, w, h, ang):
    """从当前位置直接下降到缓存目标抓取（用于已到预抓取位后）"""
    jaw = (math.cos(math.radians(ang + (90 if w > h else 0))), math.sin(math.radians(ang + (90 if w > h else 0))))
    thick = max(0.012, min(0.045, min(w, h) * 0.6))
    table_z = zmax - thick
    z_g = table_z + max(0.008, min(0.020, thick * 0.5))
    log('缓存目标 x=%.3f y=%.3f zmax=%.3f w=%.3f h=%.3f ang=%.1f -> z_g=%.3f' % (x, y, zmax, w, h, ang, z_g))
    q_g, info = g.ik(x, y, z_g, jaw, (0, 0, -1))
    if q_g is None:
        log('❌ 抓取位 IK 失败', info); return 6
    g.grip(0.52, settle=1.0)
    r = g.go(q_g, 0.10)
    if not r.get('ok'):
        log('❌ 下降失败', r); return 7
    time.sleep(0.4)
    close = min(0.5, max(0.02, (min(w, h) - 0.003) / 0.111))
    rr = g.grip(close, eff=25.0)
    f1 = rr.get('pos', -1)
    log('闭合到 %.3f finger=%.3f stalled=%s' % (close, f1, rr.get('stalled')))
    q_l, _ = g.ik(x, y, z_g + 0.08, jaw, (0, 0, -1))
    if q_l is not None:
        g.go(q_l, 0.10); time.sleep(0.5)
        f2 = g.grip(close, eff=25.0).get('pos', -1)
        log('微抬 finger=%.3f 滑脱=%s' % (f2, abs(f1-f2) > 0.03))
        if abs(f1-f2) <= 0.03:
            log('✓✓ 抓取成功（书已夹住）'); return 0
    return 8

def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else 'dry'
    if stage == 'descend' and len(sys.argv) >= 8:
        g = Grasper()
        return cached_descend(g, *[float(v) for v in sys.argv[2:8]])
    g = Grasper()
    fl, objs = candidates()
    log('floor=%.3f 桌面书候选=%d' % (fl, len(objs)))
    for o in objs[:5]:
        log('  c=(%.2f,%.2f) zmax=%.3f rect=%.3fx%.3f ang=%.1f n=%d' % (
            o['c'][0], o['c'][1], o['zmax'], o['rect']['w'], o['rect']['h'], o['rect']['ang'], o.get('n', 0)))
    if stage == 'status' or not objs:
        return 0 if objs else 2
    o = objs[0]
    cx, cy = o['c']; zmax = o['zmax']
    jaw, ang = jaw_dir(o)
    # 桌面高度：书顶 zmax - 估计厚度(取短边 60%，夹 1-3cm 书)
    thick = max(0.012, min(0.045, min(o['rect']['w'], o['rect']['h']) * 0.6))
    table_z = zmax - thick
    log('目标 c=(%.2f,%.2f) zmax=%.3f 估计厚 %.1fmm 桌面 z=%.3f 夹爪角 %.0f°' % (
        cx, cy, zmax, thick*1000, table_z, ang))
    # 1) 侧摆A：必须
    r = g.clear_column(0.35)
    if not r.get('ok'):
        log('❌ 侧摆A失败/离柱不足', r); return 3
    log('✓ 侧摆A 到位')
    # 2) 预抓取位（书上方 12cm）
    z_pre = zmax + 0.12
    q_pre, info = g.ik(cx, cy, z_pre, jaw, (0, 0, -1))
    if q_pre is None:
        # 换 90° 再试
        jaw2 = (-jaw[1], jaw[0])
        q_pre, info = g.ik(cx, cy, z_pre, jaw2, (0, 0, -1)); jaw = jaw2
    if q_pre is None:
        log('❌ 预抓取 IK 失败', info); return 4
    log('预抓取 IK ok col=%.3f tilt=%.2f' % (info.get('col', 9), info.get('tilt', 0)))
    r = g.go(q_pre, 0.35)
    if not r.get('ok'):
        log('❌ 到预抓取位失败', r); return 5
    log('✓ 到达预抓取位，等待确认')
    if stage == 'dry':
        return 0
    # 3) 下降 + 夹取
    z_g = table_z + max(0.008, min(0.020, thick * 0.5))
    q_g, info = g.ik(cx, cy, z_g, jaw, (0, 0, -1))
    if q_g is None:
        log('❌ 抓取位 IK 失败', info); return 6
    log('抓取位 z=%.3f IK ok' % z_g)
    g.grip(0.52, settle=1.0)
    r = g.go(q_g, 0.12)
    if not r.get('ok'):
        log('❌ 下降失败', r); return 7
    time.sleep(0.4)
    close = min(0.5, max(0.02, (min(o['rect']['w'], o['rect']['h']) - 0.003) / 0.111))
    log('闭合到 %.3f' % close)
    rr = g.grip(close, eff=25.0)
    f1 = rr.get('pos', -1); log('闭合 finger=%.3f stalled=%s' % (f1, rr.get('stalled')))
    if not (0.05 <= f1 <= 0.35):
        log('⚠ 闭合读数异常/夹空')
    # 4) 微抬 + 滑脱检测
    q_l, _ = g.ik(cx, cy, z_g + 0.08, jaw, (0, 0, -1))
    if q_l is not None:
        g.go(q_l, 0.12); time.sleep(0.5)
        f2 = g.grip(close, eff=25.0).get('pos', -1)
        log('微抬 finger=%.3f 滑脱=%s' % (f2, abs(f1-f2) > 0.03))
        if abs(f1-f2) <= 0.03:
            log('✓✓ 抓取成功（书已夹住）')
            return 0
    log('✗ 抓取失败')
    return 8

if __name__ == '__main__':
    sys.exit(main())
