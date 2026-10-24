import sys,time
sys.path.insert(0,'/home/Agi/jogd')
import robotclient as pk, grasp2, guard
g=grasp2.Grasper()
X,Y=-0.116,0.0  # 占位
X,Y=0.57,-0.12
SIDE=[1.1,1.2,2.0,0.0,-0.8,0.0]
cur=[float(v) for v in pk.state()["q"]]
sols=g.ik_all(X,Y,0.22,(0.0,1.0))
print("sols",len(sols))
qh=None
for q,ep,er,c in sols:
    ok,_=guard.path_ok(cur,q,g.Tba)
    print(" cand col=%.3f path=%s"%(c["col"],ok))
    if ok: qh=q; break
if qh is None:
    print("via SIDE"); print(" go:",g.go(SIDE,0.6)); time.sleep(0.4)
    cur=[float(v) for v in pk.state()["q"]]
    for q,ep,er,c in sols:
        ok,_=guard.path_ok(cur,q,g.Tba)
        if ok: qh=q; break
if qh is None: print("NO SAFE HOVER"); sys.exit(2)
print(" hover:",g.go(qh,0.7)); time.sleep(0.6)
w=g.scan("wrist")
if not w: print("wrist none"); sys.exit(3)
w.sort(key=lambda o:(o["c"][0]-X)**2+(o["c"][1]-Y)**2)
o=w[0]; print(" target",o["c"],o["zmax"],o["size"])
cx,cy=o["c"]; tz=max(-0.03,o["zmax"]-0.055)
close=max(0.38,g.jaw_value_for(max(0.045,min(o["size"])),0.006))
print(" z=%.3f close=%.2f"%(tz,close))
qp,_=g.ik(cx,cy,tz+0.10,(0.0,1.0)); qg,_=g.ik(cx,cy,tz,(0.0,1.0))
ql,_=g.ik(cx,cy,tz+0.10,(0.0,1.0)); qu,_=g.ik(cx,cy,tz+0.22,(0.0,1.0))
if any(q is None for q in (qp,qg,ql,qu)): print("IK FAIL"); sys.exit(4)
pk.post("/grip",{"v":0.52}); time.sleep(1.0)
print(" pre:",g.go(qp,0.5)); print(" down:",g.go(qg,0.25))
pk.post("/grip",{"v":close}); time.sleep(1.8)
r=pk.post("/gripper",{"v":close,"eff":20}); f1=r.get("pos",-1)
print(" close pos=%.3f (cmd %.2f)"%(f1,close))
if f1<=close+0.02: print("RESULT EMPTY"); sys.exit(5)
print(" lift:",g.go(ql,0.12)); time.sleep(0.5)
r=pk.post("/gripper",{"v":close,"eff":20}); f2=r.get("pos",-1)
print(" after lift %.3f"%f2)
if abs(f1-f2)>0.03: print("RESULT SLIP"); sys.exit(6)
print(" up:",g.go(qu,0.15)); time.sleep(0.4)
r=pk.post("/gripper",{"v":close,"eff":20}); print(" after up %.3f"%r.get("pos",-1))
print("RESULT OK")
