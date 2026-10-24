import sys,time
sys.path.insert(0,'/home/Agi/jogd')
import robotclient as pk, grasp2
g=grasp2.Grasper()
X,Y=float(sys.argv[1]),float(sys.argv[2])
Z=float(sys.argv[3]) if len(sys.argv)>3 else -0.030
CL=0.38
best=None
for dx,dy in ((0,0),(0.015,0),(-0.015,0),(0,0.015),(0,-0.015),(0.015,0.015),(-0.015,-0.015),(0.015,-0.015),(-0.015,0.015)):
    x,y=X+dx,Y+dy
    qp,_=g.ik(x,y,Z+0.10,(0.0,1.0)); qg,_=g.ik(x,y,Z,(0.0,1.0))
    if qp is None or qg is None:
        print(" (%+.3f,%+.3f) IK fail"%(dx,dy)); continue
    pk.post("/grip",{"v":0.52}); time.sleep(0.8)
    r1=g.go(qp,0.5); r2=g.go(qg,0.25)
    if not (r1.get("ok") and r2.get("ok")):
        print(" (%+.3f,%+.3f) move fail"%(dx,dy)); continue
    pk.post("/grip",{"v":CL}); time.sleep(1.5)
    r=pk.post("/gripper",{"v":CL,"eff":20}); pos=r.get("pos",-1)
    d=pos-CL
    print(" (%+.3f,%+.3f) pos=%.3f depth=%+.3f %s"%(dx,dy,pos,d,"★" if d>0.02 else ""))
    if best is None or d>best[0]: best=(d,dx,dy,pos)
    # 抬起，若夹住则直接抬升验证
    if d>0.02:
        g.go(qp,0.3); time.sleep(0.4)
        r=pk.post("/gripper",{"v":CL,"eff":20}); print("   抬升后 %.3f"%r.get("pos",-1))
        if abs(r.get("pos",-1)-pos)<0.03:
            qu,_=g.ik(x,y,Z+0.22,(0.0,1.0))
            if qu is not None:
                g.go(qu,0.15); print("RESULT OK")
                sys.exit(0)
    g.go(qp,0.5); pk.post("/grip",{"v":0.52}); time.sleep(0.4)
print("best:",best)
print("RESULT FAIL")
