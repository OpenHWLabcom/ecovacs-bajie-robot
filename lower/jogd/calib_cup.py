#!/usr/bin/env python3
# calib_cup.py — 用「接触法」标定手腕相机到夹爪的固定偏差
import sys, json, time, math
sys.path.insert(0, '/home/Agi/jogd')
import robotclient as pk, grasp2
g = grasp2.Grasper()

# 先用大致坐标悬停（头部扫描或人工给的 x y）
if len(sys.argv) >= 3:
    hx, hy = float(sys.argv[1]), float(sys.argv[2])
else:
    hh = g.scan("head")
    if not hh:
        print("头部也无候选"); sys.exit(2)
    hx, hy = hh[0]["c"][0], hh[0]["c"][1]
print("悬停到 (%.3f, %.3f)" % (hx, hy))
qh, _ = g.ik(hx, hy, 0.22, (0.0, 1.0))
if qh is None:
    print("悬停 IK 失败"); sys.exit(2)
rh = pk.move(qh, 0.8); time.sleep(0.6)
print("  悬停结果:", rh)
if not rh.get("ok"):
    print("悬停移动失败"); sys.exit(2)
w = g.scan("wrist")
if not w:
    print("手腕无目标"); sys.exit(2)
w.sort(key=lambda o: (o["c"][0]-hx)**2 + (o["c"][1]-hy)**2)
o = w[0]
cx, cy = o["c"]; ztop = o["zmax"]
print("手腕测杯子: (%.3f, %.3f) zmax=%.3f size=%s" % (cx, cy, ztop, o["size"]))
tz = max(-0.03, ztop - 0.055)
cmd = 0.30
results = []
offs = [(0,0),(1.5,0),(-1.5,0),(0,1.5),(0,-1.5),(1.5,1.5),(-1.5,-1.5),(1.5,-1.5),(-1.5,1.5)]
for dx, dy in offs:
    x, y = cx + dx/100.0, cy + dy/100.0
    qp, _ = g.ik(x, y, tz + 0.12, (0.0, 1.0))
    qg, _ = g.ik(x, y, tz, (0.0, 1.0))
    if qp is None or qg is None:
        print("  (%+.1f,%+.1f) IK 失败" % (dx, dy)); continue
    pk.post("/grip", {"v": 0.52}); time.sleep(0.9)
    pk.move(qp, 0.5); pk.move(qg, 0.28)
    pk.post("/grip", {"v": cmd}); time.sleep(1.5)
    r = pk.post("/gripper", {"v": cmd, "eff": 20})
    pos = r.get("pos", -1)
    depth = pos - cmd
    print("  (%+.1f,%+.1f) → pos=%.3f 夹入深度=%.3f %s" % (dx, dy, pos, depth, "★接触" if depth > 0.02 else ""))
    results.append(((dx, dy), depth, pos))
    pk.move(qp, 0.5)       # 抬回悬停
    pk.post("/grip", {"v": 0.52}); time.sleep(0.5)
best = max(results, key=lambda t: t[1]) if results else None
if best and best[1] > 0.02:
    (bx, by), dpt, pos = best
    print("✅ 最佳偏移 (%+.1fmm, %+.1fmm) 夹入深度 %.3f" % (bx, by, dpt))
    json.dump({"dx": bx/1000.0, "dy": by/1000.0, "depth": dpt, "ts": time.time()},
              open('/home/Agi/jogd/wrist_bias.json', 'w'))
else:
    print("⚠️ 所有点都没接触（杯子太小/位置差太多）")
print("DONE")
