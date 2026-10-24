import sys,time
sys.path.insert(0,'/home/Agi/jogd')
import robotclient as pk, grasp2
g=grasp2.Grasper()
w=g.scan("wrist")
if not w: print("no target"); sys.exit(2)
o=w[0]
print("target",o["c"],o["zmax"],o["size"],o["rect"])
ok,info=g.grasp_target(o)
print("RESULT",ok,info.get("err") or info.get("pos"),info.get("strat"))
