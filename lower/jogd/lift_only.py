import sys,time,numpy as np
sys.path.insert(0,'/home/Agi/jogd')
import robotclient as pk, grasp2, kin
g=grasp2.Grasper()
q=[float(x) for x in pk.state()["q"]]
T=g.Tba@kin.fk(q)
print("TCP now",np.round(T[:3,3],3))
tgt=T.copy(); tgt[2,3]+=0.10
Ta=np.linalg.inv(g.Tba)@tgt
# 用当前姿态做种子求 IK（保持夹爪朝向）
q2,ep,er=g._solve(Ta,q)
print("lift IK ep=%.4f er=%.3f"%(ep,er))
if ep<0.02:
    print("move:",g.go(q2,0.15)); time.sleep(0.6)
    r=pk.post("/gripper",{"v":0.05,"eff":25}); print("lifted pos=%.3f"%r.get("pos",-1))
    tgt2=tgt.copy(); tgt2[2,3]+=0.12
    q3,ep3,_=g._solve(np.linalg.inv(g.Tba)@tgt2,q2)
    if ep3<0.02:
        g.go(q3,0.18); time.sleep(0.5)
        r=pk.post("/gripper",{"v":0.05,"eff":25}); print("up pos=%.3f"%r.get("pos",-1))
