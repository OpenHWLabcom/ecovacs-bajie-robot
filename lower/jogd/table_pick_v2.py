#!/usr/bin/env python3
"""table_pick_v2.py — 桌面抓取（遵守 2026-10-19 铁律）
1) 先升立柱到安全高度；2) 夹爪到桌面以上预备位；3) 头部扫描 → 手腕精定位；4) 只从上往下抓。
用法: python3 table_pick_v2.py [lift_m]
"""
import sys, time, math, subprocess
sys.path.insert(0, '/home/Agi/jogd')
from grasp2 import Grasper
import robotclient as pk
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from common_msgs.action import RobotControl

def log(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)

def set_lift(h):
    rclpy.init(args=None); n=Node('table_pick_lift')
    ac=ActionClient(n, RobotControl, '/robot/control')
    if not ac.wait_for_server(timeout_sec=5): log('❌ no robot/control'); return False
    g=RobotControl.Goal(); g.task_type=3; g.lift_dis=float(h); g.pitch_angle=0.0
    fut=ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    gh=fut.result()
    if not (gh and gh.accepted): log('❌ lift rejected'); return False
    rf=gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=20)
    r=rf.result(); log('升立柱 %.2fm -> %s'%(h, r.result.message if r else '?'))
    return bool(r and r.result.error_code==0)

def main():
    lift=float(sys.argv[1]) if len(sys.argv)>1 else 0.15
    if not set_lift(lift): return 3
    subprocess.run(['python3','/home/Agi/jogd/kin_update.py'], check=False)
    g=Grasper()
    g.grip(0.52, settle=1.0)
    if not g.clear_column(0.35).get('ok'):
        log('❌ 侧摆A失败'); return 6
    log('✓ 侧摆A（视野清空）')
    d=pk.get('/scan'); fl=float(d.get('floor',-9)); log('桌面 z=%.3f'%fl)
    cands=[]
    for o in d.get('objects',[]):
        r=o.get('rect') or {}; w=r.get('w',0); h=r.get('h',0); z=o.get('zmax',-9)
        if w<=0 or h<=0: continue
        lo,sh=min(w,h),max(w,h)
        if not (fl+0.004 < z < fl+0.18): continue
        if not (0.08 < sh < 0.32): continue
        if not (0.008 < lo < 0.09): continue
        if sh/lo < 1.25: continue
        x,y=o['c']
        if not (0.32 < x < 0.72): continue
        cands.append(o)
    log('候选 %d'%len(cands))
    for o in cands[:5]:
        log('  c=(%.2f,%.2f) zmax=%.3f rect=%.3fx%.3f'%(o['c'][0],o['c'][1],o['zmax'],o['rect']['w'],o['rect']['h']))
    if not cands: return 4
    # 多夹爪角尝试，选立柱余量最大的安全解
    chosen=None; bestcol=-9
    for o in cands:
        w,h=o['rect']['w'],o['rect']['h']; base_a=o['rect'].get('ang',0)+(90 if w>h else 0)
        for a in sorted(set([base_a, base_a+90, 0, 90, 180, 270])):
            jaw=(math.cos(math.radians(a)),math.sin(math.radians(a)))
            z_pre=max(fl+0.15, o['zmax']+0.12)
            z_g=fl+0.020
            qp,ip=g.ik(o['c'][0],o['c'][1],z_pre,jaw,(0,0,-1))
            qg,ig=g.ik(o['c'][0],o['c'][1],z_g,jaw,(0,0,-1))
            if qp is not None and qg is not None and ig.get('col',0)>=0.06 and ig.get('col',0)>bestcol:
                bestcol=ig['col']; chosen=(o,jaw,z_pre,z_g,qp,qg)
                log('  安全解 jaw=%.0f col=%.3f z_g=%.3f'%(a,bestcol,z_g))
    if not chosen: log('❌ 无安全 IK 候选'); return 5
    o,jaw,z_pre,z_g,qp,qg=chosen
    log('选中 c=(%.2f,%.2f) zmax=%.3f -> 预抓 z=%.3f 抓取 z=%.3f'%(o['c'][0],o['c'][1],o['zmax'],z_pre,z_g))
    if not g.go(qp,0.30).get('ok'): log('❌ 预抓位失败'); return 7
    log('✓ 已到桌面以上预备位')
    # 手腕精定位
    try:
        oo,err=g.servo_align((o['c'][0],o['c'][1]), z_pre, jaw, iters=4)
        if oo and oo.get('c'):
            o=dict(o); o['c']=[oo['c'][0],oo['c'][1]]
            qg2,ig2=g.ik(o['c'][0],o['c'][1],z_g,jaw,(0,0,-1))
            if qg2 is not None: qg=qg2; log('✓ 手腕精定位 (err=%s)'%err)
    except Exception as e:
        log('手腕精定位跳过:',str(e)[:60])
    g.grip(0.52,settle=1.0)
    if not g.go(qg,0.10).get('ok'): log('❌ 下降失败'); return 8
    time.sleep(0.4)
    close=max(0.02,min(0.4,(min(o['rect']['w'],o['rect']['h'])-0.003)/0.111))
    rr=g.grip(close,eff=22.0); f1=rr.get('pos',-1)
    log('闭合 %.3f finger=%.3f stalled=%s'%(close,f1,rr.get('stalled')))
    ql,_=g.ik(o['c'][0],o['c'][1],z_g+0.08,jaw,(0,0,-1))
    if ql is not None:
        g.go(ql,0.10); time.sleep(0.5)
        f2=g.grip(close,eff=22.0).get('pos',-1)
        log('微抬 finger=%.3f 滑脱=%s'%(f2,abs(f1-f2)>0.03))
        if abs(f1-f2)<=0.03: log('✓✓ 抓取成功（待放置）'); return 0
    return 9

if __name__=='__main__': sys.exit(main())
