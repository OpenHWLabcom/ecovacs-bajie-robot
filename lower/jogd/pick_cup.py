#!/usr/bin/env python3
# pick_cup.py [x y] — 捡杯子（手腕精测 + 垂直夹杯身 + 接触验证）
import sys, json, time, math
sys.path.insert(0, '/home/Agi/jogd')
import robotclient as pk, grasp2
x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.664
y = float(sys.argv[2]) if len(sys.argv) > 2 else -0.116
g = grasp2.Grasper()

qh, ie = g.ik(x, y, 0.22, (0.0, 1.0))
if qh is None:
    print("悬停 IK 失败", ie); sys.exit(2)
r = g.go(qh, 0.7); print("悬停:", r)
time.sleep(0.6)
w = g.scan("wrist")
if not w:
    print("手腕看不到目标"); sys.exit(2)
w.sort(key=lambda o: (o["c"][0]-x)**2 + (o["c"][1]-y)**2)
o = w[0]
print("手腕目标: c=%s zmax=%.3f size=%s rect=%.3f×%.3f" % (o["c"], o["zmax"], o["size"], o["rect"]["w"], o["rect"]["h"]))
cx, cy = o["c"]
tz = max(-0.03, o["zmax"] - 0.055)
body = max(0.045, min(o["size"]))
close = max(0.38, g.jaw_value_for(body, 0.006))
print("抓取: (%.3f,%.3f) z=%.3f close=%.2f" % (cx, cy, tz, close))
qp, _ = g.ik(cx, cy, tz + 0.10, (0.0, 1.0))
qg, _ = g.ik(cx, cy, tz, (0.0, 1.0))
ql, _ = g.ik(cx, cy, tz + 0.10, (0.0, 1.0))
qu, _ = g.ik(cx, cy, tz + 0.22, (0.0, 1.0))
if any(q is None for q in (qp, qg, ql, qu)):
    print("IK 失败"); sys.exit(2)
pk.post("/grip", {"v": 0.52}); time.sleep(1.1)
print("接近:", g.go(qp, 0.5))
print("下探:", g.go(qg, 0.25))
pk.post("/grip", {"v": close}); time.sleep(1.8)
r = pk.post("/gripper", {"v": close, "eff": 20}); f1 = r.get("pos", -1)
contact = f1 > close + 0.02
print("闭合 pos=%.3f (命令 %.2f) → %s" % (f1, close, "★接触" if contact else "空"))
if not contact:
    print("RESULT FAIL(空)"); sys.exit(3)
print("微抬:", g.go(ql, 0.12)); time.sleep(0.5)
r = pk.post("/gripper", {"v": close, "eff": 20}); f2 = r.get("pos", -1)
print("微抬后 pos=%.3f" % f2)
if abs(f1 - f2) > 0.03:
    print("RESULT FAIL(滑脱)"); sys.exit(4)
print("举起:", g.go(qu, 0.15)); time.sleep(0.5)
r = pk.post("/gripper", {"v": close, "eff": 20}); f3 = r.get("pos", -1)
print("举起后 pos=%.3f" % f3)
print("RESULT OK" if abs(f2 - f3) < 0.03 else "RESULT FAIL")
