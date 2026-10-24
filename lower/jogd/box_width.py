import sys,time,math
sys.path.insert(0,'/home/Agi/jogd')
import robotclient as pk, grasp2
g=grasp2.Grasper()
w=g.scan("wrist")
if not w: print("no target"); sys.exit(2)
o=w[0]
rw,rh,ra=o["rect"]["w"],o["rect"]["h"],o["rect"]["ang"]
if rw>rh: ra+=90.0
jaw=(math.cos(math.radians(ra)),math.sin(math.radians(ra)))
cx,cy=o["c"]; z=max(-0.03,o["zmax"]-0.045)
short=min(rw,rh)
close=max(0.30,g.jaw_value_for(short,0.004))
print("target",o["c"],"rect",rw,rh,"short",short,"jaw轴",round(ra,0),"close",round(close,2),"z",round(z,3))
qp,_=g.ik(cx,cy,z+0.10,jaw); qg,_=g.ik(cx,cy,z,jaw)
ql,_=g.ik(cx,cy,z+0.10,jaw); qu,_=g.ik(cx,cy,z+0.22,jaw)
if any(q is None for q in (qp,qg,ql,qu)): print("IK FAIL"); sys.exit(3)
pk.post("/grip",{"v":0.52}); time.sleep(1.0)
print(" pre:",g.go(qp,0.5)); print(" down:",g.go(qg,0.25))
pk.post("/grip",{"v":close}); time.sleep(1.8)
r=pk.post("/gripper",{"v":close,"eff":25}); f1=r.get("pos",-1)
print(" close pos=%.3f (cmd %.2f) 接触=%s"%(f1,close,f1>close+0.02))
if f1<=close+0.02: print("RESULT EMPTY"); sys.exit(4)
print(" lift:",g.go(ql,0.12)); time.sleep(0.5)
r=pk.post("/gripper",{"v":close,"eff":25}); f2=r.get("pos",-1); print(" lifted %.3f"%f2)
if abs(f1-f2)>0.03: print("RESULT SLIP"); sys.exit(5)
print(" up:",g.go(qu,0.15)); time.sleep(0.4)
r=pk.post("/gripper",{"v":close,"eff":25}); print(" up %.3f"%r.get("pos",-1))
print("RESULT OK")
