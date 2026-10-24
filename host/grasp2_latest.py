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
sys.path.insert(0, '/home/Agi/jogd')
import numpy as np
try:
    import robotclient as pk
except Exception:
    import pk
import kin, guard

LOG = []
BIAS_FILE = '/home/Agi/jogd/grasp_bias.json'
STATS_FILE = '/home/Agi/jogd/grasp_stats.json'
T0 = time.time()


def log(msg):
    s = "[%5.1fs] %s" % (time.time() - T0, msg)
    print(s, flush=True)
    LOG.append(s)


class Grasper:
    # 必须步骤：大臂远离立柱（侧摆A，2026-10-17 验证全骨段离柱 ≥6cm）
    CLEAR_POSE = [1.1, 1.2, 2.0, 0.0, -0.8, 0.0]
    # ⚠️ 2026-10-18 11:31 高位避险位 [-0.6,2.4,2.0,0,-1.0,0] 实测导致机器人离线，暂停使用；
    #    在重新标定前只用侧摆A，离柱余量不足则中止，不再自动尝试高位位。
    CLEAR_POSE_HIGH = None
    SEEDS = [
        [1.2, -0.24, 1.46, -1.61, -1.2, -0.3],
        [1.2, -0.24, 1.46, -1.61, -1.2, 0.0],
        [0.6, 0.2, 2.2, 0.2, -0.4, 0.2],
        [0.3, 0.9, 2.6, 0.3, -0.9, 0.3],
        [1.5, 0.0, 1.7, -1.6, -1.5, 0.0],
        [0.186, 2.694, 2.15, -0.285, -1.065, 0.0],
    ]

    def stat(self, key, ok):
        try:
            d = json.load(open(STATS_FILE))
        except Exception:
            d = {}
        e = d.get(key, {"ok": 0, "fail": 0})
        e["ok" if ok else "fail"] = e.get("ok" if ok else "fail", 0) + 1
        d[key] = e
        try: json.dump(d, open(STATS_FILE, 'w'))
        except Exception: pass

    def load_bias(self, key):
        try:
            d = json.load(open(BIAS_FILE))
            return d.get(key, [0.0, 0.0])
        except Exception:
            return [0.0, 0.0]

    def save_bias(self, key, dx, dy, ok):
        try:
            d = json.load(open(BIAS_FILE))
        except Exception:
            d = {}
        cur = d.get(key, [0.0, 0.0, 0])
        if ok:
            n = min(9, cur[2] if len(cur) > 2 else 0)
            cur = [(cur[0]*n + dx)/(n+1), (cur[1]*n + dy)/(n+1), n+1]
            log("  学习偏置 %s → (%.3f, %.3f) 样本 %d" % (key, cur[0], cur[1], cur[2]))
        else:
            cur = [cur[0], cur[1], cur[2] if len(cur) > 2 else 0]
        d[key] = cur
        try:
            json.dump(d, open(BIAS_FILE, 'w'))
        except Exception:
            pass

    def __init__(self):
        st = json.load(open('/home/Agi/jogd/kin_state.json'))
        self.Tba = kin.T_of(*st['arm_base'])
        self.lo = np.array([j[4][0] for j in kin.JOINTS]) + 1e-4
        self.hi = np.array([j[4][1] for j in kin.JOINTS]) - 1e-4

    # ---------- IK ----------
    def _R(self, ap, jaw):
        ap = np.array(ap, float); ap /= np.linalg.norm(ap)
        jaw = np.array(jaw, float); jaw -= (jaw @ ap) * ap; jaw /= np.linalg.norm(jaw)
        z = np.cross(ap, jaw)
        return np.array([[ap[0], jaw[0], z[0]], [ap[1], jaw[1], z[1]], [ap[2], jaw[2], z[2]]])

    def _chain_axes(self, q):
        T = np.eye(4); out = []
        for (name, xyz, rpy, axis, lim), qi in zip(kin.JOINTS, q):
            Tb = T @ kin.T_of(xyz, rpy)
            ax = Tb[:3, :3] @ np.array(axis, float); ax /= np.linalg.norm(ax)
            out.append((Tb[:3, 3].copy(), ax))
            A = np.eye(4); A[:3, :3] = kin.axis_rot(axis, qi); T = Tb @ A
        return out, T

    def _ev(self, q, Tt):
        T = kin.fk(q); ep, er = kin.pose_err(T, Tt)
        return np.concatenate([-ep, -(T[:3, :3] @ er)]), T

    def _solve(self, Ta, seed, iters=80, tol=1e-5):
        """解析雅可比 LM（约 0.01s/解，比数值差分快 ~300 倍）"""
        q = np.clip(np.array(seed, float), self.lo, self.hi)
        e, T = self._ev(q, Ta); lam = 1e-3
        k = 0
        for k in range(iters):
            if np.linalg.norm(e) < tol:
                break
            p = T[:3, 3]; J = np.zeros((6, 6))
            for i, (o, ax) in enumerate(self._chain_axes(q)[0]):
                J[0:3, i] = np.cross(ax, p - o); J[3:6, i] = ax
            try:
                d = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ e)
            except Exception:
                break
            qn = np.clip(q + d, self.lo, self.hi); en, Tn = self._ev(qn, Ta)
            if np.linalg.norm(en) < np.linalg.norm(e):
                q, e, T = qn, en, Tn; lam = max(lam * 0.5, 1e-9)
            else:
                lam = min(lam * 4, 1e3)
                if lam >= 1e3:
                    break
        ep, er = kin.pose_err(kin.fk(q), Ta)
        return q, float(np.linalg.norm(ep)), float(np.linalg.norm(er))

    def ik_all(self, x, y, z, jaw, ap=(0, 0, -1)):
        """返回全部可行解（按误差排序），用于“选一条路径安全的解”"""
        Tt = np.eye(4); Tt[:3, :3] = self._R(ap, (jaw[0], jaw[1], 0))
        Tt[:3, 3] = [x, y, z]
        Ta = np.linalg.inv(self.Tba) @ Tt
        out = []
        seen = set()
        for sd in self.SEEDS:
            q, ep, er = self._solve(Ta, sd)
            if ep < 0.012 and er < 0.10 and self._arm_tilt(q) >= 0.30:
                k = tuple(np.round(q, 2))
                if k in seen: continue
                seen.add(k)
                c = guard.check(q, self.Tba)
                out.append((q, ep, er, c))
        out.sort(key=lambda t: t[1])
        return out

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
        n_flat = 0
        for _, (q, ep, er) in sols:
            c = guard.check(q, self.Tba)
            tilt = self._arm_tilt(q)
            if tilt < 0.30:            # 🔴 硬规则（管理员 2026-10-17）：大臂不许放平（≥17°）
                n_flat += 1
                continue
            # 大臂不要放平（放平扫掠圆周大、易碰立柱）：倾角 <0.35 时扣分
            flat_pen = 0.030 * (1.0 - min(1.0, tilt / 0.35))
            # 长臂尽量一条线（J3 越弯扣分）；少用 J1 横扫（减小横向扫掠面）
            straight_pen = 0.030 * min(1.0, abs(q[2]) / 1.2)
            j1_pen = 0.012 * min(1.0, abs(q[0]) / 1.0)
            score = ep + 0.05 * er - 0.001 * min(0.05, c["col"]) + flat_pen + straight_pen + j1_pen
            if best is None or score < best[0]:
                best = (score, q, ep, er, c)
        if best is None:
            return None, {"err": "flat-arm-forbidden", "n_flat": n_flat, "n": len(sols)}
        return best[1], {"ep": round(best[2], 4), "er": round(best[3], 3), "col": best[4]["col"], "n": len(sols), "tilt": round(self._arm_tilt(best[1]), 2)}

    def _arm_tilt(self, q):
        """大臂倾角：|z分量|/长度（0=水平，1=竖直）"""
        try:
            Tcp, grip, frames = kin.fk(q, ret_all=True)
            p_sh = (self.Tba @ frames[1])[:3, 3]
            p_el = (self.Tba @ frames[2])[:3, 3]
            v = p_el - p_sh
            L = float(np.linalg.norm(v)) or 1.0
            return abs(float(v[2])) / L
        except Exception:
            return 0.0

    # ---------- 安全运动 ----------
    JAW_MM_PER_UNIT = 111.0   # 实测标定：开口(mm) ≈ 111 × finger 值（0.506→56mm）

    def jaw_value_for(self, width_m, squeeze=0.004):
        """按目标宽度求夹爪命令值（mm 模型），留 squeeze 压紧量"""
        mm = max(2.0, width_m * 1000.0 - squeeze * 1000.0)
        return max(0.005, min(0.50, mm / self.JAW_MM_PER_UNIT))

    def go1(self, rad, vmax=0.5, allow_detour=True):
        """单段移动（内部用）"""
        cur = [float(x) for x in pk.state()["q"]]
        ok, bad = guard.path_ok(cur, rad, self.Tba)
        if ok:
            r = pk.move(rad, vmax)
            if r.get("ok"):
                return r
            # 控制器拒绝/未收敛（如 out-of-max-internal）→ 复位后重试一次，再不行走绕行
            log("  移动未到位 err=%s → 复位重试" % r.get("err"))
            try: pk.reset()
            except Exception: pass
            time.sleep(0.4)
            r = pk.move(rad, vmax)
            if r.get("ok"):
                return r
        if not allow_detour:
            return {"ok": False, "err": "collision", "bad": bad[:3]}
        detours = [
            [1.1, 1.2, 2.0, 0.0, -0.8, 0.0],     # 侧摆A：全骨段离柱 ≥6cm（最安全中转位）
            [rad[0], 1.2, 2.4, 0.0, -0.9, 0.0],
            [0.0, 1.5, 2.8, 0.0, -1.5, 0.0],
        ]
        for d in detours:
            ok1, b1 = guard.path_ok(cur, d, self.Tba)
            ok2, b2 = guard.path_ok(d, rad, self.Tba)
            if ok1 and ok2:
                log("  绕行: 经 %s" % [round(x, 2) for x in d])
                r = pk.move(d, vmax)
                if not r.get("ok"):
                    try: pk.reset()
                    except Exception: pass
                    continue
                return pk.move(rad, vmax)
        return {"ok": False, "err": "collision(no detour)", "bad": bad[:3]}

    def go_traj(self, rads, vmax=0.5):
        """多点一条轨迹（更顺）；仍做限位检查"""
        cur = [float(x) for x in pk.state()["q"]]
        q_prev = cur
        for r in rads:
            ok, bad = guard.path_ok(q_prev, r, self.Tba)
            if not ok:
                log("  轨迹越界，退回分段: %s" % str(bad[:1]))
                return self.go_chain(rads, vmax)
            q_prev = r
        res = pk.traj(rads, vmax)
        if not res.get("ok") and "err" in res:
            # 可能是保护锁 → 自动复位后重试一次
            log("  轨迹失败(%s) → 自动复位重试" % res.get("err"))
            try: pk.reset()
            except Exception: pass
            time.sleep(0.5)
            res = pk.traj(rads, vmax)
        return res

    def go(self, rad, vmax=0.5, allow_detour=True):
        cur = [float(x) for x in pk.state()["q"]]
        delta = max(abs(rad[i]-cur[i]) for i in range(6))
        if delta > 0.7:
            n = int(math.ceil(delta / 0.55))
            log("  拆分为 %d 段（总跨度 %.2f rad）" % (n, delta))
            last = {"ok": True}
            for k in range(1, n+1):
                t = [cur[i] + (rad[i]-cur[i])*k/n for i in range(6)]
                last = self.go1(t, vmax, allow_detour)
                if not last.get("ok"):
                    return last
            return last
        return self.go1(rad, vmax, allow_detour)

    def go_chain(self, rads, vmax=0.5):
        out = []
        for r in rads:
            res = self.go(r, vmax)
            out.append(res)
            if not res.get("ok"):
                return {"ok": False, "steps": out}
        return {"ok": True, "steps": out}

    def clear_column(self, vmax=0.35):
        """必须步骤：先让大臂远离立柱（侧摆A），再做向前/大范围运动。"""
        cur = [float(x) for x in pk.state()["q"]]
        lift_z = float(self.Tba[2, 3])
        pose = self.CLEAR_POSE
        name = "侧摆A"
        d = max(abs(cur[i] - pose[i]) for i in range(6))
        if d < 0.08:
            return {"ok": True, "note": "already-clear", "d": round(d, 3)}
        log("🛡 必须步骤：大臂远离立柱 → %s %s（lift_z=%.3f）" % (name, [round(x, 2) for x in pose], lift_z))
        r = self.go1(pose, vmax, allow_detour=True)
        if not r.get("ok"):
            log("❌ 大臂远离立柱失败: %s" % r)
            return r
        c = guard.check(pose, self.Tba)
        if not c.get("ok") or c.get("col", 0) < 0.06:
            log("❌ %s 离柱余量不足（实测 %.3fm < 0.06m）: %s" % (name, c.get("col", -1), c))
            return {"ok": False, "err": "clear-margin", "check": c}
        log("✓ %s 已到位，离柱余量 %.3f m" % (name, c["col"]))
        return r

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
        table = fl > 0.2
        objs = [o for o in d.get("objects", [])
                if fl + (0.006 if table else 0.035) < o["zmax"] < (fl + 0.30 if table else 0.35)
                and max(o["size"]) < 0.35 and o.get("n", 0) > (50 if table else 80)
                and min(o.get("rect", {}).get("w", 0), o.get("rect", {}).get("h", 0)) > (0.015 if table else 0.025)]
        if table:
            for o in objs:
                o["table_z"] = fl
        # 按“体积感”排序：高度 × 水平尺寸（避免抓到小碎屑）
        objs.sort(key=lambda o: -((o["zmax"] - fl) * max(o["size"])))
        return objs

    def classify(self, o):
        w, h = o["rect"]["w"], o["rect"]["h"]
        long_, short_ = max(w, h), min(w, h)
        base = float(o.get("table_z") if o.get("table_z") is not None else -0.058)
        top = o["zmax"] - base          # 离支撑面高度
        if base > 0.2:
            return "flat_table" if top < 0.06 else "generic_table"
        if top > 0.10 and long_ > 2.0 * short_:
            return "standing"           # 立着的盒子：夹长壁
        if top > 0.05 and long_ < 0.10 and short_ < 0.085 and abs(w - h) < 0.045:
            return "cup"                # 杯状
        if top < 0.09:
            return "flat"               # 平躺的盒子
        return "generic"

    def servo_align(self, goal_xy, z_hover, jaw, iters=6, tol=0.005):
        """闭环视觉伺服：用手腕相机的‘工具系偏移’把目标对准夹爪轴线"""
        gx, gy = goal_xy
        for it in range(iters):
            w = pk.get("/wrist")
            fl = w.get("floor", -0.06)
            table = fl > 0.2
            objs = [o for o in w.get("objects", []) if o.get("n", 0) > (50 if table else 80)
                    and o["zmax"] < (fl + 0.20 if table else 0.35) and o.get("tool_p")]
            if not objs:
                log("  伺服: 手腕看不到目标"); return None, None
            objs.sort(key=lambda o: (o["c"][0]-gx)**2 + (o["c"][1]-gy)**2)
            o = objs[0]
            tp = o["tool_p"]                      # 目标在工具系的位置
            # 工具系: X=approach(向下), Y=爪轴, Z=横向 → 平面偏移 (Y,Z)
            dy, dz = tp[1], tp[2]
            err = math.hypot(dy, dz)
            log("  伺服%d: 工具系偏移 (%.3f, %.3f) 误差 %.1fmm" % (it, dy, dz, err*1000))
            if err < tol:
                return o, err
            # 把工具系偏移转成 base 方向：用当前 TCP 姿态
            q = [float(x) for x in pk.state()["q"]]
            T = self.Tba @ kin.fk(q)
            R = T[:3, :3]
            delta_base = R @ np.array([0.0, dy, dz])
            gx += delta_base[0]; gy += delta_base[1]   # 朝目标移动（原来符号反了 → 来回震荡）
            qq, info = self.ik(gx, gy, z_hover, jaw)
            if qq is None:
                log("  伺服: 修正 IK 失败"); return o, err
            self.go(qq, 0.35)
            time.sleep(0.3)
        return o, err

    # ---------- 远距离接近 ----------
    def approach_target(self, max_steps=8, reach=0.72):
        """边走边扫：转向 → 前进 → 重扫，直到目标进入臂展"""
        for i in range(max_steps):
            head = self.scan("head")
            if not head:
                log("接近: 无候选"); return False
            o = head[0]
            x, y = o["c"]; dist = math.hypot(x, y); bear = math.atan2(y, x)
            log("接近 %d: 目标 (%.2f,%.2f) 距离 %.2fm 方位 %.0f°" % (i, x, y, dist, math.degrees(bear)))
            if dist <= reach:
                log("已进入臂展 (%.2fm)" % dist); return True
            if abs(bear) > 0.12:
                a = max(-0.9, min(0.9, bear))
                # 弧线：小幅偏差时边走边转（比"先转再走"少一次停顿）
                d = 0.16 if abs(bear) < 0.5 else 0.0
                log("  弧线 d=%.2fm a=%.2frad" % (d, a)); pk.chassis(d, a); time.sleep(5.5)
            else:
                d = max(0.12, min(0.35, dist - reach + 0.05))
                log("  前进 %.2fm" % d); pk.chassis(d, 0); time.sleep(5.0)
        head = self.scan("head")
        if head and math.hypot(*head[0]["c"]) <= reach + 0.12:
            return True
        log("接近失败"); return False

    def place_back(self, q_grasp, q_up=None):
        """把夹住的物体轻轻放回：降到抓取位 → 分步开爪 → 竖直退开"""
        log("放回…")
        self.go(q_grasp, 0.35); time.sleep(0.5)
        for v in (0.34, 0.46, 0.52):
            pk.post("/grip", {"v": v}); time.sleep(1.0)
        log("  开爪 %.3f" % (pk.finger() or -1))
        # 必须步骤：收回前先让大臂远离立柱
        self.clear_column(0.35)
        self.go(q_up if q_up else [0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.5)
        time.sleep(0.8)
        try:
            w = self.scan("wrist")
            if w:
                log("  放置后复扫: zmax=%.3f (%.0fcm)" % (w[0]["zmax"], (w[0]["zmax"]+0.06)*100))
            else:
                log("  放置后复扫: 未见物体（可能被遮挡）")
        except Exception:
            pass

    # ---------- 翻立（平躺 → 立起）----------
    def flip_up(self, o):
        """夹住近壁 → 向上+向后拉 → 松开，让盒子绕远边翻起"""
        cx, cy = o["c"]; jn = o["jn"]; jaw = (-jn[0], -jn[1])
        tz = max(-0.03, min(0.09, o["zmax"] - 0.04))
        q_g, i_g = self.ik(cx, cy, tz, jaw)
        q_p, _ = self.ik(cx, cy, tz + 0.14, jaw)
        if q_g is None or q_p is None:
            return False
        self.grip(0.52, settle=1.0)
        if not self.go_chain([q_p, q_g], 0.35).get("ok"):
            return False
        r = self.grip(0.05, eff=25.0); f = r.get("pos", -1)
        log("翻立: 夹近壁 pos=%.3f" % f)
        if not (0.04 <= f <= 0.48):
            self.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.8)
            return False
        # 向上 + 朝机器人拉，制造绕远边的翻转力矩
        q_t, _ = self.ik(cx - 0.06, cy, tz + 0.09, jaw)
        if q_t:
            self.go(q_t, 0.12)
        self.grip(0.52, settle=1.2)          # 松开
        self.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9)
        time.sleep(0.6)
        w = self.scan("wrist")
        if w:
            log("翻立后: c=%s zmax=%.3f" % (w[0]["c"], w[0]["zmax"]))
        return bool(w) and w[0]["zmax"] > 0.09

    # ---------- 抓取 ----------
    def grasp_target(self, o, close=0.05, strategy=None):
        """对单个目标执行抓取；返回 (ok, info)"""
        strat = strategy or self.classify(o)
        table = float(o.get("table_z") if o.get("table_z") is not None else -1) > 0.2
        log("策略=%s 目标 c=%s rect=%.3f×%.3f zmax=%.3f" % (strat, o["c"], o["rect"]["w"], o["rect"]["h"], o["zmax"]))
        # 优先用拟合出的壁线（更准），否则退回 near/jn
        w = o.get("wall")
        if w and w.get("len", 0) > 0.04:
            cx, cy = w["p"]; jn = w["n"]
        else:
            cx, cy = o["c"]; jn = o["jn"]
        jaw = (-jn[0], -jn[1])
        # 🔑 有壁线拟合时：夹爪轴必须**垂直于壁**（夹壁/夹薄边），不能用短边方向
        try:
            if not (w and w.get("len", 0) > 0.04):
                raise RuntimeError("no wall")
            rw, rh, ra = o["rect"]["w"], o["rect"]["h"], o["rect"]["ang"]
            if rw > rh:
                ra += 90.0          # 让 ra 指向短边
            jax = (math.cos(math.radians(ra)), math.sin(math.radians(ra)))
            if jax[0]*jaw[0] + jax[1]*jaw[1] < 0:
                jax = (-jax[0], -jax[1])   # 保持与壁线法向同侧
            jaw = jax
            log("  夹爪轴=短边方向 %.0f° (短边 %.1fcm)" % (ra % 180, min(rw, rh)*100))
        except Exception:
            pass
        if table:
            tz = max(o["table_z"] + 0.004, min(o["table_z"] + 0.10, o["zmax"] - 0.015))
            ap = (0, 0, -1)
            log("  桌面目标 table_z=%.3f → tz=%.3f" % (o["table_z"], tz))
        elif strat == "cup":
            # 实战：斜 45° 在这个臂型下常无解 → 改为**垂直夹杯身下部**（Ø 比杯口小，5.7cm 开口够）
            tz = max(-0.03, o["zmax"] - 0.06)
            ap = (0, 0, -1)
            try:
                body_w = max(0.045, min(o.get("size", [0.05, 0.05])))
                close = max(0.38, self.jaw_value_for(body_w, 0.008))
                log("  杯身宽 %.1fcm → 闭合 %.2f" % (body_w*100, close))
            except Exception:
                pass
        elif strat == "standing":
            tz = max(-0.035, o["zmax"] - 0.09)   # 立着：夹更深（指尖插入 9cm）
            ap = (0, 0, -1)
        else:
            tz = max(-0.03, min(0.09, o["zmax"] - 0.045))
            ap = (0, 0, -1)
        # 壁线三点搜索：中点优先，失败换左/右 3cm
        wall_dir = (jn[1], -jn[0]) if jn else (1.0, 0.0)
        cand = [(cx, cy)]
        cand.append((cx + wall_dir[0] * 0.03, cy + wall_dir[1] * 0.03))
        cand.append((cx - wall_dir[0] * 0.03, cy - wall_dir[1] * 0.03))
        q_g, i_g = self.ik(cx, cy, tz, jaw, ap)
        q_p, i_p = self.ik(cx, cy, tz + 0.14, jaw, ap)
        if q_g is None or q_p is None:
            self.stat("IKfail", False)
            return False, {"err": "IK fail", "g": i_g, "p": i_p}
        log("IK ok grasp z=%.3f (ep %.4f, 立柱余量 %.3f, 大臂倾角 %.2f, 解数 %d)" % (tz, i_g.get("ep", 9), i_g.get("col", 9), i_g.get("tilt", 0), i_g.get("n", 0)))
        q_l, _ = self.ik(cx, cy, tz + 0.10, jaw, ap)
        q_u, _ = self.ik(cx, cy, tz + 0.22, jaw, ap)
        # 夹爪张开
        r = self.grip(0.52, settle=1.2)
        log("开爪 pos=%.3f" % r.get("pos", -1))
        small = table or (o.get("zmax", 0) - (-0.06)) < 0.12 or strat == "cup"
        if small:
            # 先悬停 → 闭环伺服对齐 → 再算抓取位
            r = self.go(q_p, 0.55)
            if r.get("ok"):
                oo, serr = self.servo_align((cx, cy), tz + 0.14, jaw)
                if oo is not None and oo.get("c"):
                    cx, cy = oo["c"][0], oo["c"][1]
                    o = dict(o); o["c"] = [cx, cy]
                    q_g2, i_g2 = self.ik(cx, cy, tz, jaw, ap)
                    if q_g2 is not None:
                        q_g = q_g2
                        log("  伺服后重算抓取位 (误差 %s)" % ("%.1fmm" % (serr*1000) if serr else "?"))
        else:
            d_total = max(abs(q_g[i]-q_p[i]) for i in range(6))
            v_ap = 0.55 if d_total > 0.8 else 0.35
            r = self.go_traj([q_p, q_g], vmax=v_ap)
        if not r.get("ok"):
            self.stat("movefail", False)
            return False, {"err": "move", "steps": r.get("steps")}
        # 闭合 + 双读
        eff = 25.0 if strat in ("standing", "flat") else (12.0 if strat == "cup" else 18.0)
        if strat not in ("standing", "flat"):
            try:
                w_est = min(o.get("size", [0.05, 0.05]))
                close = min(close, self.jaw_value_for(w_est, 0.004))
            except Exception:
                pass
        r1 = self.grip(close, eff=eff)
        f1 = r1.get("pos", -1)
        log("闭合 pos=%.3f stalled=%s" % (f1, r1.get("stalled")))
        if strat in ("standing", "flat"):
            # 夹壁应有 2-3mm 壁厚 → 期望停在 0.01~0.12
            if not (0.003 <= f1 <= 0.035):   # 3mm 壁 → 期望 0.003~0.035（之前 0.048 是夹空）
                self.stat("wall-miss", False)
                return False, {"err": "not-on-wall", "pos": f1, "strat": strat}
        else:
            if not (0.06 <= f1 <= 0.48) or abs(f1 - close) < 0.015:
                self.stat("empty", False)
                return False, {"err": "empty", "pos": f1, "close": close, "strat": strat}
        # 微抬验证
        self.go(q_l, 0.12)
        time.sleep(0.5)
        r2 = self.grip(close, eff=25.0)
        f2 = r2.get("pos", -1)
        slip = abs(f1 - f2) > 0.03
        log("微抬 pos=%.3f 滑脱=%s" % (f2, slip))
        if slip:
            self.stat("slip", False)
            return False, {"err": "slip", "pos": f1, "strat": strat}
        self.go(q_u, 0.2)
        time.sleep(0.4)
        r3 = self.grip(close, eff=25.0)
        f3 = r3.get("pos", -1)
        log("举起 pos=%.3f" % f3)
        ok3 = (0.08 <= f3 <= 0.48) and abs(f2 - f3) < 0.03
        self.stat(strat + (":ok" if ok3 else ":fail"), ok3)
        return ok3, {"pos": f3, "strat": strat, "slip_top": abs(f2 - f3) >= 0.03, "grasp": q_g, "up": q_u}


def auto_grasp(retries=True, verbose=True):
    g = Grasper()
    # 0) 扫描（先收回到准备位清视野）
    g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9)
    time.sleep(0.4)
    head = g.scan("head")
    if not head:
        log("头部无候选"); return False
    hc = head[0]
    if math.hypot(*hc["c"]) > 0.75:
        log("目标较远 (%.2fm) → 自己走过去" % math.hypot(*hc["c"]))
        if not g.clear_column(0.35).get("ok"):
            log("❌ 大臂远离立柱失败，中止接近"); return False
        if not g.approach_target():
            log("接近失败，仍尝试就近抓取")
        g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9); time.sleep(0.4)
        head = g.scan("head")
        if not head:
            log("接近后无候选"); return False
        hc = head[0]
    log("头部候选 c=%s zmax=%.3f" % (hc["c"], hc["zmax"]))
    # 必须步骤：向前伸手前，先让大臂远离立柱
    if not g.clear_column(0.35).get("ok"):
        log("❌ 大臂远离立柱失败，中止抓取"); return False
    # 1) 悬停到候选上方 → 手腕精测
    qh, _ = g.ik(hc["c"][0], hc["c"][1], max(0.18, hc["zmax"] + 0.26), (-hc["jn"][0], -hc["jn"][1]))
    if qh is None:
        log("悬停 IK 失败"); return False
    g.go(qh, 0.8); time.sleep(0.5)
    wobjs = g.scan("wrist")
    if not wobjs:
        log("手腕无目标"); return False
    o = wobjs[0]
    bkey = g.classify(o)
    bx, by = g.load_bias(bkey)
    if abs(bx) + abs(by) > 1e-4:
        log("应用学习偏置 %s: (%.3f, %.3f)" % (bkey, bx, by))
        o["c"] = [o["c"][0] + bx, o["c"][1] + by]
        if o.get("wall"):
            o["wall"] = dict(o["wall"]); o["wall"]["p"] = [o["wall"]["p"][0] + bx, o["wall"]["p"][1] + by]
    log("手腕精测 c=%s zmax=%.3f rect=%.3f×%.3f wall=%s" % (o["c"], o["zmax"], o["rect"]["w"], o["rect"]["h"], (o.get("wall") or {}).get("p")))
    hw, hh = hc.get("rect", {}).get("w", 9), hc.get("rect", {}).get("h", 9)
    ww, wh = o["rect"]["w"], o["rect"]["h"]
    if hw < 5 and hh < 5 and max(ww, wh) > 1.35 * max(hw, hh):
        log("⚠ 手腕矩形偏大(%.2f×%.2f vs 头 %.2f×%.2f)，以头部为准修正" % (ww, wh, hw, hh))
        o = dict(o); o["rect"] = dict(o["rect"])
        sc = max(hw, hh) / max(ww, wh)
        o["rect"]["w"] = ww * sc; o["rect"]["h"] = wh * sc
    # 2) 抓取（带 ±1cm 网格重试）
    offsets = [(0, 0), (0.015, 0), (-0.015, 0), (0, 0.015), (0, -0.015), (0.03, 0), (-0.03, 0)]
    for dx, dy in offsets:
        o2 = dict(o); o2["c"] = [o["c"][0] + dx, o["c"][1] + dy]
        ok, info = g.grasp_target(o2)
        if (not ok) and info.get("g") and isinstance(info.get("g"), dict) and info["g"].get("err") == "flat-arm-forbidden" and dx == 0 and dy == 0:
            log("⚠ 大臂放平被规则禁止 → 前进 0.2m 再试")
            pk.chassis(0.20, 0); time.sleep(5)
            g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.9); time.sleep(0.4)
            w4 = g.scan("head")
            if w4:
                o5 = w4[0]
                ok5, info5 = g.grasp_target(o5)
                log("靠近后重试: ok=%s %s" % (ok5, info5.get("err") or info5.get("pos")))
                if ok5:
                    try: g.save_bias(bkey, 0, 0, True)
                    except Exception: pass
                    if info5.get("grasp"): g.place_back(info5["grasp"], info5.get("up"))
                    return True
        if ok:
            try: g.save_bias(bkey, dx, dy, True)
            except Exception: pass
            log("✅ 抓取成功 %s" % info)
            try:
                if info.get("grasp"): g.place_back(info["grasp"], info.get("up"))
            except Exception as e:
                log("放回失败 %s" % str(e)[:60])
            return True
        log("✗ 失败: %s" % info)
        if info.get("strat") == "flat" and info.get("err") in ("slip", "empty", "not-on-wall") and dx == 0 and dy == 0:
            log("→ 尝试翻立")
            g2 = g
            if g2.flip_up(o):
                g2.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.8)
                w3 = g2.scan("wrist")
                if w3 and w3[0]["zmax"] > 0.09:
                    o3 = w3[0]
                    for ddx, ddy in ((0,0),(0.01,0),(-0.01,0)):
                        o4 = dict(o3); o4["c"] = [o3["c"][0]+ddx, o3["c"][1]+ddy]
                        ok4, info4 = g2.grasp_target(o4, strategy="standing")
                        if ok4:
                            log("✅ 翻立后抓取成功 %s" % info4); return True
        if not retries:
            break
        g.clear_column(0.30)
        g.go([0.186, 2.694, 2.15, -0.285, -1.065, 0.0], 0.8)
        time.sleep(0.3)
    return False


if __name__ == "__main__":
    ok = auto_grasp()
    print("RESULT", "OK" if ok else "FAIL")
