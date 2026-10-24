#!/usr/bin/env python3
# guard.py — 抓取/运动碰撞限位（立柱 / 底盘 / 地面）
import numpy as np, json, sys
sys.path.insert(0,'/opt/openhwlab/robot-viz')
import kin

# 立柱：分两段（2026-10-17 21:48 实测 J3 弯头蹭到立柱头部 → 头部加粗 + 覆盖到 z0.93）
# r 已包含连杆半径(约1.5cm)，等效“中心线到柱面”的安全距离
# ⚠️ 2026-10-18 管理员确认：立柱与机械臂一起升降，永远连在一起 → z 范围随 arm_base 升降平移
LIFT0_Z = 0.48267   # lift=0 时 arm_base z
COL_SEGS = [
    {"c": (-0.014, 0.099), "r": 0.125, "z": (0.10, 0.60)},   # 立柱主体
    {"c": (-0.014, 0.099), "r": 0.130, "z": (0.60, 0.93)},   # 头部+摄像头总成（大臂必须远离）
]
# 各段连杆的等效半径（大臂最粗）
BONE_R = [0.045, 0.050, 0.045, 0.040, 0.036, 0.033, 0.030]
CH  = {"c": (0.0, 0.0),    "r": 0.34,  "z": (-0.12, 0.14)}   # 底盘（保守）
FLOOR = -0.043   # 真实地面在 base_link 下 -0.058；留 1.5cm 余量
KIN_STATE = '/tmp/kin_state.json'

def _Tba():
    st=json.load(open(KIN_STATE)); return kin.T_of(*st['arm_base'])

def _segs_for(Tba):
    """立柱与机械臂一起升降：把 lift=0 的 z 范围平移到当前 arm_base 高度。"""
    if Tba is None:
        Tba = _Tba()
    dz = float(Tba[2, 3]) - LIFT0_Z
    return [{"c": s["c"], "r": s["r"], "z": (s["z"][0] + dz, s["z"][1] + dz)} for s in COL_SEGS]

def points(q, Tba=None, n=12):
    """返回 [(x,y,z, bone_index)]，bone_index 用于取该段连杆半径"""
    Tba = Tba if Tba is not None else _Tba()
    Tcp, grip, frames = kin.fk(q, ret_all=True)
    pts=[(Tba@T)[:3,3] for T in frames] + [(Tba@grip)[:3,3], (Tba@Tcp)[:3,3]]
    out=[]
    for bi,(a,b) in enumerate(zip(pts, pts[1:])):
        for k in range(n+1):
            p=a+(b-a)*(k/n); out.append((p[0],p[1],p[2],bi))
    return out

def check(q, Tba=None, mcol=0.02, mch=0.02, m_tilt=0.25):
    P = points(q, Tba)
    segs = _segs_for(Tba)
    dz = (float(Tba[2, 3]) - LIFT0_Z) if Tba is not None else 0.0
    col = {"clear": 1e9}
    ch  = {"clear": 1e9}
    fl  = min(p[2] for p in P)
    col_sh = 9.9   # 肩部（bone0）单独记录（结构上贴柱，不算风险）
    # 大臂（bone 1）倾角
    b1 = [p for p in P if p[3] == 1]
    tilt = 1.0
    if len(b1) >= 2:
        a = np.array(b1[0][:3]); b = np.array(b1[-1][:3]); v = b - a
        L = float(np.linalg.norm(v)) or 1.0
        tilt = abs(float(v[2])) / L
    for (x, y, z, bi) in P:
        rb = BONE_R[bi] if bi < len(BONE_R) else 0.03
        if bi == 1 and z > 0.58 + dz: rb += 0.02      # 大臂在立柱头部区域额外留 2cm（随升降平移）
        for seg in segs:
            if seg["z"][0]-0.03 <= z <= seg["z"][1]+0.02:
                d = float(np.hypot(x-seg["c"][0], y-seg["c"][1]) - seg["r"] - rb)
                if bi == 0:
                    col_sh = min(col_sh, d)          # 肩部仅记录
                else:
                    col["clear"] = min(col["clear"], d)   # 运动连杆（bone1+）才算风险
        if CH["z"][0] <= z <= CH["z"][1]:
            d = float(np.hypot(x-CH["c"][0], y-CH["c"][1]) - CH["r"] - rb)
            ch["clear"] = min(ch["clear"], d)
    ok = bool(col["clear"] >= mcol and ch["clear"] >= mch and fl >= FLOOR and tilt >= m_tilt)
    return {"ok": ok, "col": round(float(col["clear"]),3), "col_sh": round(float(col_sh),3),
            "chassis": round(float(ch["clear"]),3), "floor": round(float(fl),3), "tilt": round(float(tilt),2)}

def path_ok(q0, q1, Tba=None, steps=16, mcol=0.0, mch=0.02, m_tilt=0.25):
    bad=[]
    for k in range(1, steps+1):   # 跳过 s=0（当前姿态），允许脱困
        s=k/steps; q=[q0[i]+(q1[i]-q0[i])*s for i in range(6)]
        r=check(q,Tba,mcol,mch,m_tilt)
        if not r["ok"]: bad.append((round(s,2),r))
    return (len(bad)==0), bad

def safe_move(rad, vmax=0.5, steps=12):
    import pk
    s=pk.state(); q0=[float(x) for x in s["q"]]
    ok,bad=path_ok(q0,rad,steps=steps)
    if not ok: return {"ok":False,"err":"collision","bad":bad}
    return pk.move(rad,vmax)

if __name__=="__main__":
    Tba=_Tba()
    for name,q in (("归位",[0,2.459,3.257,-0.006,-2.543,0]),
                   ("抓取准备位",[0.186,2.694,2.15,-0.285,-1.065,0]),
                   ("开箱",[0,1.767,2.887,0,-0.451,0])):
        print(name, check(q,Tba))
