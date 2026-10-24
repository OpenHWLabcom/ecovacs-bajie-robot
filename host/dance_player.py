#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dance_player.py — 机械臂关键帧舞蹈播放器。

默认只 dry-run（不动机器人）。确认仿真/空载后加 --execute。
安全规则见 MOTION_RULES.md：慢速 → 侧摆A → 等到位 → 逐帧 → 恢复标准。

用法：
  python3 dance_player.py --dance dances/example.json --dry-run
  python3 dance_player.py --dance dances/example.json --execute --speed 0.6

JSON：
{
  "name": "hello",
  "keyframes": [
    {"t": 0.0, "q": [0.186,2.694,2.150,-0.285,-1.065,0.0], "dwell": 0.4},
    {"t": 1.2, "q": [...], "lift": 0.1, "grip": 0.3, "dwell": 0.2}
  ]
}
"""
import argparse, json, math, os, sys, time, urllib.request

CLEAR = [1.1, 1.2, 2.0, 0.0, -0.8, 0.0]
BASE = os.environ.get('ROBOT_BRIDGE', 'http://127.0.0.1:8796')
CMD = os.environ.get('ROBOT_CMD', 'http://127.0.0.1:8792')

ap = argparse.ArgumentParser()
ap.add_argument('--dance', required=True)
ap.add_argument('--execute', action='store_true', help='真的动真机；默认 dry-run')
ap.add_argument('--dry-run', action='store_true', help='只打印，不动机器人（默认）')
ap.add_argument('--speed', type=float, default=0.6, help='0.3~1.0，越小越慢')
ap.add_argument('--timeout', type=float, default=25.0, help='每帧等待上限秒')
ap.add_argument('--no-clear', action='store_true', help='跳过侧摆A（不建议）')
a = ap.parse_args()
if a.dry_run:
    a.execute = False

dance = json.load(open(a.dance))
frames = dance.get('keyframes') or []
if len(frames) < 2:
    sys.exit('keyframes 至少 2 帧')
for i, f in enumerate(frames):
    if not isinstance(f.get('q'), list) or len(f['q']) != 6:
        sys.exit('第 %d 帧 q 必须是 6 个值' % (i + 1))


def req(url, body=None, timeout=8):
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    return urllib.request.urlopen(r, timeout=timeout).read().decode('utf-8', 'replace')


def joints():
    try:
        return json.loads(req(BASE + '/joints', timeout=3))
    except Exception as e:
        print('  ! 读取 /joints 失败:', e)
        return None


def maxdiff(a, b):
    return max(abs(float(a[i]) - float(b[i])) for i in range(6))


def wait_near(target, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        j = joints()
        if j and j.get('rad') and maxdiff(j['rad'], target) < 0.15:
            return True
        time.sleep(0.25)
    return False


def mode(vmax, amax, label):
    try:
        req(BASE + '/mode', {'mode': {'id': 'dance_' + label, 'label': label, 'vmax': vmax, 'amax': amax, 'vmin': 0.15, 'dead': 0.02}})
    except Exception as e:
        print('  ! mode 失败:', e)


def jog(q):
    try:
        return json.loads(req(BASE + '/jog', {'rad': q}))
    except Exception as e:
        print('  ! /jog 失败:', e)
        return None


def lift(v):
    try:
        req(CMD + '/cmd', {'action': 'height', 'value': float(v)})
    except Exception as e:
        print('  ! lift 失败:', e)


def grip(v):
    try:
        req(BASE + '/grip', {'v': float(v)})
    except Exception as e:
        print('  ! grip 失败:', e)


print('舞蹈:', dance.get('name', os.path.basename(a.dance)), '·', len(frames), '帧 ·', 'EXECUTE' if a.execute else 'DRY-RUN')
cur = joints()
if cur:
    print('当前:', [round(x, 3) for x in (cur.get('rad') or [])], 'lift=', cur.get('lift'))

if not a.execute:
    for i, f in enumerate(frames):
        print('  %02d t=%.2fs q=%s lift=%s grip=%s dwell=%s' % (
            i + 1, f.get('t', 0), [round(float(x), 3) for x in f['q']],
            f.get('lift', '-'), f.get('grip', '-'), f.get('dwell', 0)))
    print('dry-run 结束；确认后在仿真/空载验证，再加 --execute')
    sys.exit(0)

# ---- 真机执行 ----
if cur is None or cur.get('status') != 'ok':
    sys.exit('机器人状态不正常，中止')
vmax = max(0.3, min(1.0, a.speed))
amax = max(1.0, min(6.0, a.speed * 5.0))
mode(vmax, amax, '慢速 %.2f' % vmax)
first_q = [float(x) for x in frames[0]['q']]
need_far_move = maxdiff(cur['rad'], first_q) > 0.15
if not a.no_clear and need_far_move and maxdiff(cur['rad'], CLEAR) > 0.15:
    print('→ 侧摆A')
    jog(CLEAR)
    if not wait_near(CLEAR, a.timeout):
        sys.exit('侧摆A 未到位，中止')
for i, f in enumerate(frames):
    q = [float(x) for x in f['q']]
    print('→ 第 %d/%d 帧 t=%.2fs' % (i + 1, len(frames), f.get('t', 0)))
    jog(q)
    if not wait_near(q, a.timeout):
        print('  ! 未到位，中止后续'); break
    if f.get('lift') is not None:
        lift(f['lift'])
    if f.get('grip') is not None:
        grip(f['grip'])
    time.sleep(max(0.0, float(f.get('dwell', 0))))
mode(2.0, 6.0, '标准')
print('完成')
