#!/usr/bin/env python3
"""grasp2.py — 抓取流程 v2（安全运动 + 多解选优 + 绕柱 + 策略库 + 视觉验证 + 重试）

优化点：
 1) 所有运动走 guard（立柱/底盘/地面）限位，越界自动找绕行中间点
 2) IK 多解 + 按「立柱余量」择优（不是只看误差）
 3) 策略库：立着→夹壁；平躺→夹侧壁/先翻立；杯状→斜 45° 夹下部
 4) 夹爪用官方 action，双读消除滞后；抬升微验证滑脱
 5) 失败自动 ±1cm 网格重试
 6) 每步记录耗时/余量/读数，方便复盘
"""
import sys, time, json, math
sys.path.insert(0, '/opt/openhwlab/robot-viz')
sys.path.insert(0, '/tmp')
import numpy as np
import kin, pk, guard

LOG = []
T0 = time.time()


def log(msg):
    s = "[%5.1fs] %s" % (time.time() - T0, msg)
    print(s, flush=True)
    LOG.append(s)


class Grasper:
    SEEDS = [
        [1.2, -0.24, 1.46, -1.61, -1.2, -0.3],
        [1.2, -0.24, 1.46, -1.61, -1.2, 0.0],
        [0.6, 0.2, 2.2, 0.2, -0.4, 0.2],
        [0.3, 0.9, 2.6, 0.3, -0.9, 0.3],
        [1.5, 0.0, 1.7, -1.6, -1.5, 0.0],
        [0.186, 2.694, 2.15, -0.285, -1.065, 0.0],
    ]

    def __init__(self):
        st = json.load(open('/tmp/kin_state.json'))
        self.Tba = kin.T_of(*st['arm_base'])
        self.lo = np.array([j[4][0] for j in kin.JOINTS]) + 1e-4
        self.hi = np.array([j[4][1] for j in kin.JOINTS]) - 1e-4

    # ---------- IK ----------
    def _R(self, ap, jaw):
        ap = np.array(ap, float); ap /= np.linalg.norm(ap)
        jaw = np.array(jaw, float); jaw -= (jaw @ ap) * ap; jaw /= np.linalg.norm(jaw)
        z = np.cross(ap, jaw)
        return np.array([[ap[0], jaw[0], z[0]], [ap[1], jaw[1], z[1]], [ap[2], jaw[2], z[2]]])

    def _solve(self, Ta, seed, iters=600):
        seed = np.array(seed, float)

        def err(q):
            ep, er = kin.pose_err(kin.fk(q), Ta)
            return np.concatenate([ep, 0.35 * er, 0.02 * (q - seed)])
        q = np.clip(seed.copy(), self.lo, self.hi); e = err(q); lam = 1e-3
        for _ in range(iters):
            J = np.zeros((12, 6))
            for k in range(6):
                dq = np.zeros(6); dq[k] = 1e-6
                J[:, k] = (err(np.clip(q + dq, self.lo, self.hi)) - e) / 1e-6
            try:
                d = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ e)
            except Exception:
                break
            qn = np.clip(q + d, self.lo, self.hi); en = err(qn)
            if np.linalg.norm(en) < np.linalg.norm(e):
                q, e = qn, en; lam = max(lam * 0.5, 1e-9)
            else:
                lam = min(lam * 4, 1e3)
                if lam >= 1e3:
                    break
        ep, er = kin.pose_err(kin.fk(q), Ta)
        return q, float(np.linalg.norm(ep)), float(np.linalg.norm(er))

    def ik(self, x, y, z, jaw, ap=(0, 0, -1)):
        """多解 + 立柱余量择优；返回 (q, info) 或 (None, err)"""
        Tt = np.eye(4); Tt[:3, :3] = self._R(ap, (jaw[0], jaw[1], 0))
        Tt[:3, 3] = [x, y, z]
        Ta = np.linalg.inv(self.Tba) @ Tt
        sols = []
        for s in self.SEEDS:
            q, ep, er = self._solve(Ta, s)
            if ep < 0.012 and er < 0.10:
                key = tuple(np.round(q, 2))
                if key not in [k for k, _ in sols]:
                    sols.append((key, (q, ep, er)))
        if not sols:
            return None, {"err": 9}
        best = None
        for _, (q, ep, er) in sols:
            c = guard.check(q, self.Tba)
            score = ep + 0.05 * er - 0.001 * min(0.05, c["col"])   # 余量越大越好（上限 5cm 影响）
            if best is None or score < best[0]:
                best = (score, q, ep, er, c)
        return best[1], {"ep": round(best[2], 4), "er": round(best[3], 3), "col": best[4]["col"], "n": len(sols)}

    # ---------- 安全运动 ----------
    def go(self, rad, vmax=0.5, allow_detour=True):
        cur = [float(x) for x in pk.state()["q"]]
        ok, bad = guard.path_ok(cur, rad, self.Tba)
        if ok:
            r = pk.move(rad, vmax)
            return r
        if not allow_detour:
            return {"ok": False, "err": "collision", "bad": bad[:3]}
        detours = [
            [rad[0], 1.2, 2.4, 0.0, -0.9, 0.0],
            [0.0, 1.5, 2.8, 0.0, -1.5, 0.0],
            [cur[0], 2.2, 3.0, cur[3], -1.6, 0.0],
        ]
        for d in detours:
            ok1, b1 = guard.path_ok(cur, d, self.Tba)
            ok2, b2 = guard.path_ok(d, rad, self.Tba)
            if ok1 and ok2:
                log("  绕行: 经 %s" % [round(x, 2) for x in d])
                r = pk.move(d, vmax)
                if not r.get("ok"):
                    continue
                return pk.move(rad, vmax)
        return {"ok": False, "err": "collision(no detour)", "bad": bad[:3]}

    def go_chain(self, rads, vmax=0.5):
        out = []
        for r in rads:
            res = self.go(r, vmax)
            out.append(res)
            if not res.get("ok"):
                return {"ok": False, "steps": out}
        return {"ok": True, "steps": out}

    # ---------- 夹爪 ----------
    def grip(self, v, eff=20.0, settle=1.3):
        """控制走发布式(/grip 可靠)，读数走 action 的“上一状态”(/gripper)"""
        pk.post("/grip", {"v": v})
        time.sleep(settle)
        r = pk.post("/gripper", {"v": v, "eff": eff})
        return r

    # ---------- 感知 ----------
    def scan(self, which="head"):
        d = pk.get("/scan" if which == "head" else "/wrist")
        fl = d.get("floor", -0.06)
        objs = [o for o in d.get("objects", [])
                if fl + 0.02 < o["zmax"] < 0.35 and max(o["size"]) < 0.35 and o.get("n", 0) > 120]
        objs.sort(key=lambda o: math.hypot(*o["c"]))
        return objs

    def classify(self, o):
        w, h = o["rect"]["w"], o["rect"]["h"]
        long_, short_ = max(w, h), min(w, h)
        top = o["zmax"] - (-0.058)      # 离地高度
        if top > 0.10 and long_ > 2.0 * short_:
            return "standing"           # 立着的盒子：夹长壁
        if top > 0.06 and long_ < 0.10 and short_ < 0.08 and abs(w - h) < 0.03:
            return "cup"                # 杯状
        if top < 0.09:
            return "flat"               # 平躺的盒子
        return "generic"

    # ---------- 抓取 ----------
    def grasp_target(self, o, close=0.05, strategy=None):
        """对单个目标执行抓取；返回 (ok, info)"""
        strat = strategy or self.classify(o)
        log("策略=%s 目标 c=%s rect=%.3f×%.3f zmax=%.3f" % (strat, o["c"], o["rect"]["w"], o["rect"]["h"], o["zmax"]))
        cx, cy = o["c"]
        jn = o["jn"]
        jaw = (-jn[0], -jn[1])
        if strat == "cup":
            tz = max(-0.02, o["zmax"] - 0.03)
            ap = (0.72, 0.0, -0.70)     # 斜 45° 避开杯口
        else:
            tz = max(-0.03, min(0.09, o["zmax"] - 0.045))
            ap = (0, 0, -1)
        q_g, i_g = self.ik(cx, cy, tz, jaw, ap)
        q_p, i_p = self.ik(cx, cy, tz + 0.14, jaw, ap)
        if q_g is None or q_p is None:
            return False, {"err": "IK fail", "g": i_g, "p": i_p}
        log("IK ok grasp z=%.3f (ep %.4f, 立柱余量 %.3f, 解数 %d)" % (tz, i_g.get("ep", 9), i_g.get("col", 9), i_g.get("n", 0)))
        q_l, _ = self.ik(cx, cy, tz + 0.10, jaw, ap)
        q_u, _ = self.ik(cx, cy, tz + 0.22, jaw, ap)
        # 夹爪张开（用 action）
        r = self.grip(0.52, settle=1.2)
        log("开爪 pos=%.3f" % r.get("pos", -1))
        r = self.go_chain([q_p, q_g], vmax=0.35)
        if not r.get("ok"):
            return False, {"err": "move", "steps": r.get("steps")}
        # 闭合 + 双读
        r1 = self.grip(close, eff=25.0)
        f1 = r1.get("pos", -1)
        log("闭合 pos=%.3f stalled=%s" % (f1, r1.get("stalled")))
        if not (0.04 <= f1 <= 0.48):
            return False, {"err": "empty", "pos": f1, "strat": strat}
        # 微抬验证
        self.go(q_l, 0.12)
        time.sleep(0.5)
        r2 = self.grip(close, eff=25.0)
        f2 = r2.get("pos", -1)
        slip = abs(f1 - f2) > 0.03
        log("微抬 pos=%.3f 滑脱=%s" % (f2, slip))
        if slip:
            return False, {"err": "slip", "pos": f1, "strat": strat}
        self.go(q_u, 0.2)
        time.sleep(0.4)
        r3 = self.grip(close, eff=25.0)
        f3 = r3.get("pos", -1)
        log("举起 pos=%.3f" % f3)
        return (0.04 <= f3 <= 0.48), {"pos": f3, "strat": strat}


def auto_grasp(retries=True, verbose=True):
    g = Grasper()
    # 0) 扫描（先收回到准备位清视野）
    g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9)
    time.sleep(0.4)
    head = g.scan("head")
    if not head:
        log("头部无候选"); return False
    hc = head[0]
    log("头部候选 c=%s zmax=%.3f" % (hc["c"], hc["zmax"]))
    # 1) 悬停到候选上方 → 手腕精测
    qh, _ = g.ik(hc["c"][0], hc["c"][1], max(0.12, hc["zmax"] + 0.16), (-hc["jn"][0], -hc["jn"][1]))
    if qh is None:
        log("悬停 IK 失败"); return False
    g.go(qh, 0.8); time.sleep(0.5)
    wobjs = g.scan("wrist")
    if not wobjs:
        log("手腕无目标"); return False
    o = wobjs[0]
    log("手腕精测 c=%s zmax=%.3f rect=%.3f×%.3f" % (o["c"], o["zmax"], o["rect"]["w"], o["rect"]["h"]))
    # 2) 抓取（带 ±1cm 网格重试）
    offsets = [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]
    for dx, dy in offsets:
        o2 = dict(o); o2["c"] = [o["c"][0] + dx, o["c"][1] + dy]
        ok, info = g.grasp_target(o2)
        if ok:
            log("✅ 抓取成功 %s" % info); return True
        log("✗ 失败: %s" % info)
        if not retries:
            break
        g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.8)
        time.sleep(0.3)
    return False


if __name__ == "__main__":
    ok = auto_grasp()
    print("RESULT", "OK" if ok else "FAIL")
