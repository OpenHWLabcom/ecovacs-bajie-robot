#!/usr/bin/env python3
# cup_cycle.py N — 放回→重抓 循环（纸杯垂直夹杯身）
import sys, json, time, math
sys.path.insert(0, '/home/Agi/jogd')
import robotclient as pk, grasp2
N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
g = grasp2.Grasper()

def find_cup():
    for src in ("wrist", "head"):
        os_ = g.scan(src)
        if os_:
            # 杯类优先：尺寸 3~9cm、高度>4cm
            cups = [o for o in os_ if 0.025 < min(o["size"]) < 0.09 and 0.03 < max(o["size"]) < 0.12
                    and (o["zmax"] + 0.06) > 0.04]
            if cups:
                cups.sort(key=lambda o: max(o["size"]))
                print("  [%s] 找到杯类候选: %s zmax=%.3f size=%s" % (src, cups[0]["c"], cups[0]["zmax"], cups[0]["size"]))
                return cups[0]
    return None

cur_grasp = None
for it in range(N):
    print("==== 第 %d 轮 ====" % (it + 1))
    if cur_grasp:
        print(" 放回…"); g.go(cur_grasp, 0.3); time.sleep(0.4)
        for v in (0.34, 0.46, 0.52):
            pk.post("/grip", {"v": v}); time.sleep(1.0)
        print("  开爪 %.3f" % (pk.finger() or -1))
        g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.5); time.sleep(0.6)
    o = find_cup()
    if not o:
        print("  未找到杯子"); break
    x, y = o["c"]; z = max(-0.03, o["zmax"] - 0.06)
    close = g.jaw_value_for(min(o["size"]), 0.008)
    print("  目标 (%.3f,%.3f) z=%.3f close=%.2f" % (x, y, z, close))
    qp, _ = g.ik(x, y, z + 0.14, (0.0, 1.0))
    qg, _ = g.ik(x, y, z, (0.0, 1.0))
    ql, _ = g.ik(x, y, z + 0.10, (0.0, 1.0))
    qu, _ = g.ik(x, y, z + 0.22, (0.0, 1.0))
    if any(q is None for q in (qp, qg, ql, qu)):
        print("  IK 失败（大臂放平被禁/超距）"); break
    pk.post("/grip", {"v": 0.52}); time.sleep(1.2)
    r1 = g.go(qp, 0.5); r2 = g.go(qg, 0.28)
    print("  接近 ok=%s/%s" % (r1.get("ok"), r2.get("ok")))
    pk.post("/grip", {"v": close}); time.sleep(1.8)
    r = pk.post("/gripper", {"v": close, "eff": 20}); f1 = r.get("pos", -1)
    print("  闭合 pos=%.3f" % f1)
    if not (0.10 <= f1 <= 0.48):
        print("  ✗ 夹空"); break
    g.go(ql, 0.12); time.sleep(0.5)
    r = pk.post("/gripper", {"v": close, "eff": 20}); f2 = r.get("pos", -1)
    if abs(f1 - f2) > 0.03:
        print("  ✗ 滑脱 %.3f" % f2); break
    g.go(qu, 0.15); time.sleep(0.4)
    r = pk.post("/gripper", {"v": close, "eff": 20}); f3 = r.get("pos", -1)
    print("  ✅ 举起 pos=%.3f" % f3)
    cur_grasp = qg
    print("  本轮完成")
print("DONE")
