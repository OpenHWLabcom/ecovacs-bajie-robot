#!/usr/bin/env python3
# pick_one.py <x> <y> [strategy] — 抓指定坐标附近的目标（下板运行）
import sys, json, time, math
sys.path.insert(0, '/home/Agi/jogd')
import robotclient as pk
import grasp2

x = float(sys.argv[1]); y = float(sys.argv[2])
strat = sys.argv[3] if len(sys.argv) > 3 else None
g = grasp2.Grasper()
w = g.scan("head")
if not w:
    print("head 无候选 → 改用手腕相机")
    ww = g.scan("wrist")
    if not ww:
        print("no candidates"); sys.exit(2)
    w = ww
w.sort(key=lambda o: (o["c"][0]-x)**2 + (o["c"][1]-y)**2)
o = w[0]
print("target:", o["c"], "zmax", o["zmax"], "rect", o["rect"])
# 悬停后手腕精测
qh, _ = g.ik(o["c"][0], o["c"][1], max(0.18, o["zmax"]+0.26), (-o["jn"][0], -o["jn"][1]))
if qh is not None:
    g.go(qh, 0.7); time.sleep(0.5)
    ww = g.scan("wrist")
    if ww:
        ww.sort(key=lambda q: (q["c"][0]-x)**2 + (q["c"][1]-y)**2)
        o2 = ww[0]
        if math.hypot(o2["c"][0]-o["c"][0], o2["c"][1]-o["c"][1]) < 0.15:
            print("wrist refine:", o2["c"], o2["zmax"], o2["rect"])
            o = o2
ok, info = g.grasp_target(o, strategy=strat)
if (not ok) and isinstance(info.get("g"), dict) and info["g"].get("err") == "flat-arm-forbidden":
    print("大臂放平被禁 → 前进 0.15m 重试")
    pk.chassis(0.15, 0); time.sleep(5)
    g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9); time.sleep(0.4)
    w2 = g.scan("wrist") or g.scan("head")
    if w2:
        w2.sort(key=lambda q: (q["c"][0]-x)**2 + (q["c"][1]-y)**2)
        o = w2[0]
        ok, info = g.grasp_target(o, strategy=strat)
print("RESULT", ok, info.get("err") or info.get("pos"), info.get("strat"))
if ok and info.get("grasp"):
    g.place_back(info["grasp"], info.get("up"))
print("DONE")
