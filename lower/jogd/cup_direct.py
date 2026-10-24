#!/usr/bin/env python3
# cup_direct.py x y z close — 直接垂直抓（绕开策略分类）
import sys, json, time, math
sys.path.insert(0, '/home/Agi/jogd')
import robotclient as pk, grasp2
x, y, z, close = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
g = grasp2.Grasper()
q_pre, i1 = g.ik(x, y, z + 0.14, (0.0, 1.0))
q_g, i2 = g.ik(x, y, z, (0.0, 1.0))
q_l, i3 = g.ik(x, y, z + 0.10, (0.0, 1.0))
q_u, i4 = g.ik(x, y, z + 0.22, (0.0, 1.0))
print("IK:", [(q is not None, round(i.get("tilt", 0), 2)) for q, i in ((q_pre, i1), (q_g, i2), (q_l, i3), (q_u, i4))])
if any(q is None for q in (q_pre, q_g, q_l, q_u)):
    print("IK FAIL", i2); sys.exit(2)
pk.post("/grip", {"v": 0.52}); time.sleep(1.2)
print("pre:", g.go(q_pre, 0.5))
print("descend:", g.go(q_g, 0.28))
pk.post("/grip", {"v": close}); time.sleep(1.8)
r = pk.post("/gripper", {"v": close, "eff": 20}); f1 = r.get("pos", -1)
print("close %.2f → pos=%.3f" % (close, f1))
print("lift:", g.go(q_l, 0.12)); time.sleep(0.5)
r = pk.post("/gripper", {"v": close, "eff": 20}); f2 = r.get("pos", -1)
print("after lift pos=%.3f" % f2)
if 0.10 <= f1 <= 0.48 and abs(f1 - f2) < 0.03:
    print("up:", g.go(q_u, 0.15)); time.sleep(0.4)
    r = pk.post("/gripper", {"v": close, "eff": 20}); print("after up pos=%.3f" % r.get("pos", -1))
    print("RESULT OK")
else:
    print("RESULT FAIL")
