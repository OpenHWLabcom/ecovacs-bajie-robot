#!/usr/bin/env python3
"""八界机械臂运动学：FK + 数值 IK（基于 URDF 参数，本地计算，不依赖 moveit python）。

链（arm_base_link 内）：
  joint1: xyz(0,0,0.141802)  rpy(0.001289,-1.570768,-0.001289)  axis(1,0,0)  lim ±3.12
  joint2: xyz(0.0427,-0.06,0) rpy 0                           axis(0,-1,0) lim ±3.12
  joint3: xyz(0.3245,0.0097,0)                                axis(0,1,0)  lim -2.77..3.47
  joint4: xyz(0.0754,0.0503,0)                                axis(1,0,0)  lim ±3.12
  joint5: xyz(0.3251,0.0428,0)                                axis(0,-1,0) lim -3.12..2.61
  joint6: xyz(0.056,-0.0428,0.0765) rpy(0,1.5708,0)           axis(0,0,1)  lim ±3.12
  gripper(固定): xyz(0,0,0.0735)
  objects(固定): xyz(0,0,0.12236) rpy(0,-1.5708,0)

用法：
  python3 kin.py check            # 用当前 /joint_states + TF 校验 FK
  python3 kin.py ik x y z qx qy qz qw   # 求 objects_link 位姿的 IK（base_link 系）
"""
import sys, math, numpy as np

JOINTS = [
    # name, xyz, rpy, axis, (lo,hi)
    ("joint1", (0, 0, 0.141802), (0.001289, -1.570768, -0.001289), (1, 0, 0), (-3.12, 3.12)),
    ("joint2", (0.0427, -0.06, 0), (0, 0, 0), (0, -1, 0), (-3.12, 3.12)),
    ("joint3", (0.3245, 0.0097, 0), (0, 0, 0), (0, 1, 0), (-2.77, 3.47)),
    ("joint4", (0.0754, 0.0503, 0), (0, 0, 0), (1, 0, 0), (-3.12, 3.12)),
    ("joint5", (0.3251, 0.0428, 0), (0, 0, 0), (0, -1, 0), (-3.12, 2.61)),
    ("joint6", (0.056, -0.0428, 0.0765), (0, 1.5708, 0), (0, 0, 1), (-3.12, 3.12)),
]
GRIPPER = ((0, 0, 0.0735), (0, 0, 0))
OBJECT = ((0, 0, 0.12236), (0, -1.5708, 0))


def rpy2R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def T_of(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy2R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rot(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def fk(q, ret_all=False):
    T = np.eye(4)
    frames = []
    for (name, xyz, rpy, axis, lim), qi in zip(JOINTS, q):
        T = T @ T_of(xyz, rpy)
        A = np.eye(4)
        A[:3, :3] = axis_rot(axis, qi)
        T = T @ A
        frames.append(T.copy())
    T = T @ T_of(*GRIPPER)
    grip = T.copy()
    T = T @ T_of(*OBJECT)
    if ret_all:
        return T, grip, frames
    return T


def pose_err(T, Tt):
    ep = Tt[:3, 3] - T[:3, 3]
    R1, R2 = T[:3, :3], Tt[:3, :3]
    # 旋转误差（轴角）
    dR = R1.T @ R2
    ang = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
    if ang < 1e-9:
        er = np.zeros(3)
    else:
        ax = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        er = ax / (2 * math.sin(ang)) * ang
    return ep, er


def _err_vec(q, Tt):
    ep, er = pose_err(fk(q), Tt)
    return np.concatenate([ep, 0.35 * er])


def _lm(q0, Tt, lo, hi, iters=300):
    """纯 numpy Levenberg-Marquardt（数值雅可比）。"""
    q = np.clip(np.array(q0, float), lo, hi)
    e = _err_vec(q, Tt)
    lam = 1e-3
    for _ in range(iters):
        if np.linalg.norm(e) < 1e-6:
            break
        J = np.zeros((6, 6))
        for k in range(6):
            dq = np.zeros(6)
            dq[k] = 1e-6
            J[:, k] = (_err_vec(np.clip(q + dq, lo, hi), Tt) - e) / 1e-6
        A = J.T @ J + lam * np.eye(6)
        try:
            delta = np.linalg.solve(A, -J.T @ e)
        except Exception:
            break
        qn = np.clip(q + delta, lo, hi)
        en = _err_vec(qn, Tt)
        if np.linalg.norm(en) < np.linalg.norm(e):
            q, e = qn, en
            lam = max(lam * 0.5, 1e-9)
        else:
            lam = min(lam * 4.0, 1e3)
            if lam >= 1e3:
                break
    return q, e


def ik(Tt, q0=None, tries=24):
    lo = np.array([j[4][0] for j in JOINTS]) + 1e-4
    hi = np.array([j[4][1] for j in JOINTS]) - 1e-4
    rng = np.random.default_rng(0)
    starts = []
    if q0 is not None:
        starts.append(np.clip(np.array(q0, float), lo, hi))
    for _ in range(tries):
        starts.append(rng.uniform(lo, hi))
    best = None
    for s in starts:
        q, e = _lm(s, Tt, lo, hi)
        ep, er = pose_err(fk(q), Tt)
        score = np.linalg.norm(ep) + 0.35 * np.linalg.norm(er)
        if best is None or score < best[0]:
            best = (score, q, np.linalg.norm(ep), np.linalg.norm(er))
    return best


def _q2R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd = sys.argv[1]
    if cmd == "check":
        import json, urllib.request
        # 读本地缓存的当前状态（由调用方准备 state.json: joints[6], arm_base pose, objects pose）
        st = json.load(open("/tmp/kin_state.json"))
        q = st["joints"]
        T = fk(q)
        T_base_arm = T_of(st["arm_base"][0], st["arm_base"][1])  # xyz+rpy
        Tt_armbase = np.linalg.inv(T_base_arm) @ T_of(st["objects"][0], st["objects"][1])
        ep, er = pose_err(T, Tt_armbase)
        print("FK pos:", np.round(T[:3, 3], 4), " TF pos:", np.round(Tt_armbase[:3, 3], 4))
        print("pos err(m):", round(float(np.linalg.norm(ep)), 4), " rot err(rad):", round(float(np.linalg.norm(er)), 4))
    elif cmd == "ik":
        x, y, z, qx, qy, qz, qw = map(float, sys.argv[2:9])
        st = json.load(open("/tmp/kin_state.json"))
        T_base_arm = T_of(st["arm_base"][0], st["arm_base"][1])
        Tt = np.eye(4)
        Tt[:3, :3] = _q2R(qx, qy, qz, qw)
        Tt[:3, 3] = (x, y, z)
        Tt_ab = np.linalg.inv(T_base_arm) @ Tt
        best = ik(Tt_ab, q0=st.get("joints"))
        score, q, ep, er = best
        print("score=%.5f pos_err=%.4fm rot_err=%.4frad" % (score, ep, er))
        print("joints:", [round(float(v), 4) for v in q])
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
