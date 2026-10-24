#!/usr/bin/env python3
import sys, time, math
sys.path.insert(0,'/home/Agi/jogd')
from grasp2 import Grasper
import robotclient as pk
def log(*a): print(time.strftime('%H:%M:%S'),*a,flush=True)
g=Grasper(); jaw=(math.cos(math.radians(170)),math.sin(math.radians(170)))
f0=g.grip(0.087,eff=22.0).get('pos',-1); log('起始 finger=%.3f'%f0)
q,info=g.ik(0.65,-0.13,0.78,jaw,(0,0,-1))
if q is None: log('IK fail',info); sys.exit(4)
log('抬升 IK z=0.78 col=%.3f'%info.get('col',9))
r=g.go(q,0.08)
log('go ok=%s'%r.get('ok'))
time.sleep(0.5)
f1=g.grip(0.087,eff=22.0).get('pos',-1); log('抬升后 finger=%.3f 滑脱=%s'%(f1,abs(f1-f0)>0.03))
