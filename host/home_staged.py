#!/usr/bin/env python3
# home_staged.py — 三段式收臂归位（避开下方收纳盒）
#   S1 抬大臂（J2→2.90, J3→2.82, J1→0, J4→-0.01；J5 保持不动）
#   S2 高处折腕（J5→-2.543，其余不动）
#   S3 下降到归位（J2→2.46, J3→3.257, J4→-0.006 → 归位位）
# 前置：执行引擎(:8796)就绪；脚本会自动把夹爪设为 0.183（归位规程）。
# 用法: python3 home_staged.py [--dry] [--vmax 0.8]
import json, time, urllib.request, sys, subprocess, os
B = "http://127.0.0.1:8796"
HOME = [0.0, 2.46, 3.257, -0.006, -2.543, 0.0]
S1 = [0.0, 2.72, 2.98, -0.01, None, 0.0]   # 2026-10-17 11:39 管理员: 减小抬起幅度(J2 166->156, J3 162->171)，避免折叠爪碰大臂电机
S2 = [0.0, 2.72, 2.98, -0.01, -2.543, 0.0]
ADB = "adb"
DEV = "192.168.1.100:5555"
GRIP = 0.183
def get(p): return json.loads(urllib.request.urlopen(B+p, timeout=4).read())
def post(p, d):
    req = urllib.request.Request(B+p, data=json.dumps(d).encode(), headers={"Content-Type":"application/json"}, method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=5).read())
def set_grip():
    script = "export PATH=/usr/bin:/bin; source /opt/ros/humble/setup.bash 2>/dev/null; source /home/Agi/install/setup.bash 2>/dev/null; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///factory/default.xml; python3 /home/Agi/jogd/set_grip.py %s" % GRIP
    outer = "chroot /linux /bin/bash -lc \"export PATH=/usr/bin:/bin:/usr/sbin:/sbin; timeout 40 sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 Agi@10.0.0.2 '%s'\"" % script.replace("'","'\\''")
    r = subprocess.run([ADB, "-s", DEV, "shell", outer], capture_output=True, text=True, timeout=70)
    print("   set_grip:", (r.stdout or "").strip()[-80:], flush=True)
def wait_arrive(tgt, tag, timeout=70):
    t0 = time.time(); last_err = None; stall = 0; stable = 0
    while time.time()-t0 < timeout:
        time.sleep(0.9)
        j = get("/joints"); rad = j.get("rad")
        if not rad: continue
        err = max(abs(rad[i]-tgt[i]) for i in range(6))
        mv = max([abs(v) for v in (j.get("vel") or [0])])
        print("   [%s] t=%4.1fs deg:" % (tag, time.time()-t0), [round(x,1) for x in j.get("deg",[])], "maxErr=%.3f maxVel=%.2f" % (err, mv), flush=True)
        if err < 0.03 and mv < 0.06: stable += 1
        else: stable = 0
        if stable >= 2:
            print("   [%s] ARRIVED" % tag, flush=True); return True
        if last_err is not None and err > 0.03 and err >= last_err - 0.004: stall += 1
        else: stall = 0
        last_err = err
        if stall >= 6:
            print("   [%s] STALL → stop" % tag, flush=True)
            post("/jog", {"stop": True})
            # 今天的经验：stop 后队列可能仍在执行，等 3 秒再看
            time.sleep(3)
            j2 = get("/joints"); rad2 = j2.get("rad") or []
            if rad2:
                err2 = max(abs(rad2[i]-tgt[i]) for i in range(6))
                print("   [%s] post-stop err=%.3f" % (tag, err2), flush=True)
                if err2 < 0.05: return True
            return False
    print("   [%s] TIMEOUT" % tag, flush=True)
    return False
def main():
    dry = "--dry" in sys.argv
    vmax = 0.8
    for i,a in enumerate(sys.argv):
        if a == "--vmax": vmax = float(sys.argv[i+1])
    j = get("/joints"); cur = j.get("rad")
    print("current rad:", [round(x,3) for x in cur], "deg:", [round(x,1) for x in j.get("deg",[])], flush=True)
    s1 = list(S1); s1[4] = cur[4] if cur else -0.36
    plan = [("S1 抬大臂", s1), ("S2 高处折腕", S2), ("S3 下降到归位", HOME)]
    print("plan:", [(t, [round(x,3) for x in q]) for t,q in plan], flush=True)
    if dry: print("DRY — 不执行"); return
    post("/mode", {"mode":{"id":"slow","label":"慢速","vmax":vmax,"amax":3.0,"vmin":0.15,"dead":0.02}})
    set_grip()
    okall = True
    for tag, tgt in plan:
        print("== %s → %s" % (tag, [round(x,3) for x in tgt]), flush=True)
        post("/jog", {"rad": tgt})
        if not wait_arrive(tgt, tag): okall = False; break
    post("/mode", {"mode":{"id":"std","label":"标准","vmax":2,"amax":6,"vmin":0.15,"dead":0.02}})
    j = get("/joints")
    print("FINAL:", [round(x,3) for x in (j.get("rad") or [])], "OK" if okall else "FAILED", flush=True)
    sys.exit(0 if okall else 1)
if __name__ == "__main__": main()
