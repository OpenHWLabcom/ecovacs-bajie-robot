#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collector_svc.py — 训练数据采集服务（定点头/通用 episode）
- 启停 episode_logger_v2.py，管理 headfix episode 元数据
- 每次停止后自动重建 viewer_index.json（综合分析台数据源）
HTTP: 0.0.0.0:8789
  GET  /status        当前采集状态
  POST /start         {task,label,operator,duration,mode,note,tip,limits}
  POST /stop          停止并落 meta
  GET  /episodes?task=headfix&limit=50
  POST /meta          {dir,success,note}
  GET  /health
"""
import os, sys, json, time, signal, subprocess, threading, csv, re
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

HERE = os.path.dirname(os.path.abspath(__file__))
BP = os.path.join(HERE, 'book_pick')
LOGGER = os.path.join(BP, 'episode_logger_v2.py')
INDEXER = os.path.join(BP, 'build_viewer_index.py')
ROOT = os.path.join(BP, 'data', 'teleop')
PORT = int(os.environ.get('COLLECTOR_PORT', '8789'))
lock = threading.Lock()
rec = None  # {proc, dir, task, label, start, duration, mode, note, operator, tip, limits}


def _slug(s):
    s = ''.join(c if (c.isalnum() or c in '-_.') else '_' for c in str(s or ''))
    return s.strip('._') or 'episode'


def _rel(p):
    return os.path.relpath(p, HERE).replace(os.sep, '/')


def _count_csv(path):
    try:
        with open(path, 'rb') as f:
            n = sum(1 for _ in f)
        return max(0, n - 1)
    except Exception:
        return 0


def _frame_counts(outdir):
    p = os.path.join(outdir, 'frames.csv')
    h = w = 0
    try:
        for r in csv.DictReader(open(p)):
            if r.get('kind') == 'head':
                h += 1
            elif r.get('kind') == 'wrist':
                w += 1
    except Exception:
        pass
    return h, w


def _finalize(r):
    """把采集会话的附加信息写进 meta.json，并重建索引。"""
    if not r:
        return
    outdir = r.get('dir')
    if not outdir or not os.path.isdir(outdir):
        return
    mp = os.path.join(outdir, 'meta.json')
    meta = {}
    try:
        meta = json.load(open(mp))
    except Exception:
        pass
    meta['task'] = r.get('task', meta.get('task', 'headfix'))
    meta['label'] = r.get('label', meta.get('label', ''))
    meta['motion'] = 'headfix' if meta['task'] == 'headfix' else meta.get('motion', '')
    meta['mode'] = r.get('mode', meta.get('mode', 'manual'))
    if r.get('operator'):
        meta['operator'] = r['operator']
    if r.get('note'):
        meta['note'] = r['note']
    if r.get('tip'):
        meta['tip_fixed'] = r['tip']
    if r.get('limits'):
        meta['limits'] = r['limits']
    if 'success' not in meta:
        meta['success'] = 'unknown'
    if meta.get('duration_s') is None and meta.get('start'):
        meta['duration_s'] = round(time.time() - float(meta['start']), 2)
    try:
        meta['samples'] = _count_csv(os.path.join(outdir, 'samples.csv'))
        h, w = _frame_counts(outdir)
        meta['head_frames'] = h
        meta['wrist_frames'] = w
    except Exception:
        pass
    try:
        json.dump(meta, open(mp, 'w'), ensure_ascii=False, indent=2)
    except Exception as e:
        print('meta write failed', e, flush=True)
    # 自动刷新综合分析台索引
    try:
        subprocess.run([sys.executable, INDEXER], cwd=BP, timeout=40,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def _running(r):
    return bool(r and r.get('proc') and r['proc'].poll() is None)


def _orphans():
    """采集服务被重启后，老的 episode_logger 进程可能还在跑；按 --outdir 找回。"""
    out = []
    try:
        p = subprocess.run(['pgrep', '-af', 'episode_logger_v2.py'], capture_output=True, text=True, timeout=3)
        for line in (p.stdout or '').splitlines():
            parts = line.split(None, 1)
            if not parts or not parts[0].isdigit():
                continue
            cmd = parts[1] if len(parts) > 1 else ''
            if 'pgrep' in cmd:
                continue
            m = re.search(r'--outdir\s+(\S+)', cmd)
            out.append({'pid': int(parts[0]), 'cmd': cmd, 'dir': m.group(1) if m else None})
    except Exception:
        pass
    return out


def _status():
    global rec
    with lock:
        r = rec
    if not r:
        orps = _orphans()
        if orps:
            o = orps[0]
            d = o.get('dir')
            st = {'ok': True, 'running': True, 'orphan': True, 'pid': o['pid'],
                  'dir': _rel(d) if d else '', 'abs_dir': d or '',
                  'samples': _count_csv(os.path.join(d, 'samples.csv')) if d else 0}
            if d:
                h, w = _frame_counts(d)
                st['head_frames'] = h; st['wrist_frames'] = w
                try:
                    st['start'] = os.path.getmtime(os.path.join(d, 'samples.csv'))
                    st['elapsed'] = round(time.time() - st['start'], 1)
                except Exception:
                    pass
            return st
        return {'ok': True, 'running': False}
    if not _running(r):
        with lock:
            if rec is r:
                rec = None
        _finalize(r)
        return {'ok': True, 'running': False, 'last_dir': r.get('dir')}
    outdir = r['dir']
    st = {
        'ok': True, 'running': True, 'task': r.get('task'), 'label': r.get('label'),
        'dir': _rel(outdir),
        'abs_dir': outdir, 'start': r.get('start'),
        'elapsed': round(time.time() - r['start'], 1),
        'duration': r.get('duration') or 0,
        'mode': r.get('mode'), 'operator': r.get('operator'),
        'samples': _count_csv(os.path.join(outdir, 'samples.csv')),
    }
    h, w = _frame_counts(outdir)
    st['head_frames'] = h
    st['wrist_frames'] = w
    try:
        with open(os.path.join(outdir, 'logger.log'), 'rb') as f:
            data = f.read()[-1600:].decode('utf-8', 'replace')
        st['log_tail'] = data
    except Exception:
        st['log_tail'] = ''
    return st


def _sysinfo():
    out = {'ok': True}
    try:
        kv = {}
        for line in open('/proc/meminfo'):
            p = line.split(':')
            if len(p) == 2:
                kv[p[0]] = int(p[1].split()[0])
        out['mem_total_mb'] = round(kv.get('MemTotal', 0) / 1024)
        out['mem_available_mb'] = round(kv.get('MemAvailable', 0) / 1024)
        out['swap_free_mb'] = round(kv.get('SwapFree', 0) / 1024)
        out['load'] = [round(x, 2) for x in os.getloadavg()]
    except Exception as e:
        out['err'] = str(e)[:80]
    return out


def _start(body):
    global rec
    with lock:
        if _running(rec):
            return 409, {'ok': False, 'err': 'already recording'}
        if rec:
            _finalize(rec)
        task = body.get('task') or 'headfix'
        if task not in ('headfix', 'desktop_pick', 'shelf_pick'):
            task = 'headfix'
        label = _slug(body.get('label') or ('headfix_' + time.strftime('%H%M%S')))
        stamp = time.strftime('%Y%m%d-%H%M%S')
        outdir = os.path.join(ROOT, task, '%s_%s' % (stamp, label))
        os.makedirs(outdir, exist_ok=True)
        duration = float(body.get('duration') or 0)
        cmd = [sys.executable, '-u', LOGGER, '--task', task, '--label', label,
               '--outdir', outdir, '--operator', str(body.get('operator') or '')]
        if duration > 0:
            cmd += ['--duration', str(duration)]
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        log = open(os.path.join(outdir, 'logger.log'), 'ab')
        proc = subprocess.Popen(cmd, cwd=BP, stdout=log, stderr=subprocess.STDOUT, env=env)
        rec = {'proc': proc, 'dir': outdir, 'task': task, 'label': label, 'start': time.time(),
               'duration': duration, 'mode': body.get('mode') or 'manual',
               'operator': body.get('operator') or '', 'note': body.get('note') or '',
               'tip': body.get('tip'), 'limits': body.get('limits')}
        print(time.strftime('%H:%M:%S'), 'START', outdir, 'mode', rec['mode'], flush=True)
        return 200, {'ok': True, 'dir': _rel(outdir),
                     'abs_dir': outdir, 'label': label, 'task': task}


def _stop():
    global rec
    with lock:
        r = rec
    if not r:
        # 采集服务重启过：杀掉遗留的 logger（页面停止按钮仍然有效）
        killed = []
        first_dir = None
        for o in _orphans():
            try:
                if not first_dir and o.get('dir'):
                    first_dir = o['dir']
                os.kill(o['pid'], signal.SIGTERM)
                killed.append(o['pid'])
                time.sleep(0.8)
                if o.get('dir'):
                    _finalize({'dir': o['dir']})
            except Exception:
                pass
        return {'ok': True, 'running': False, 'killed': killed,
                'dir': _rel(first_dir) if first_dir else None, 'abs_dir': first_dir}
    p = r.get('proc')
    if p and p.poll() is None:
        try:
            p.send_signal(signal.SIGTERM)
            p.wait(timeout=10)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    with lock:
        if rec is r:
            rec = None
    _finalize(r)
    print(time.strftime('%H:%M:%S'), 'STOP', r.get('dir'), flush=True)
    return {'ok': True, 'running': False, 'dir': r.get('dir')}


def _norm_success(v):
    if v is True or str(v).lower() in ('true', '1', 'yes', '成功'):
        return True
    if v is False or str(v).lower() in ('false', '0', 'no', '失败'):
        return False
    return 'unknown'


def _episodes(task, limit):
    base = os.path.join(ROOT, task)
    out = []
    if not os.path.isdir(base):
        return out
    for name in sorted(os.listdir(base), reverse=True):
        d = os.path.join(base, name)
        if not os.path.isdir(d):
            continue
        mp = os.path.join(d, 'meta.json')
        meta = {}
        try:
            meta = json.load(open(mp))
        except Exception:
            pass
        sp = os.path.join(d, 'samples.csv')
        if not os.path.exists(sp):
            continue
        h, w = _frame_counts(d)
        dur = meta.get('duration_s')
        if dur is None:
            try:
                rows = list(csv.DictReader(open(sp)))
                if len(rows) >= 2:
                    dur = round(float(rows[-1]['t']) - float(rows[0]['t']), 2)
            except Exception:
                dur = None
        out.append({
            'name': name,
            'dir': _rel(d),
            'abs_dir': d,
            'task': task,
            'label': meta.get('label', name),
            'mode': meta.get('mode', ''),
            'motion': meta.get('motion', ''),
            'operator': meta.get('operator', ''),
            'success': _norm_success(meta.get('success', 'unknown')),
            'duration_s': dur,
            'samples': _count_csv(sp),
            'head_frames': h, 'wrist_frames': w,
            'start_iso': meta.get('start_iso', ''),
            'note': meta.get('note', ''),
            'tip_fixed': meta.get('tip_fixed'),
        })
        if len(out) >= max(1, min(200, int(limit))):
            break
    return out


def _meta(body):
    d = os.path.abspath(str(body.get('dir') or ''))
    if not os.path.isabs(str(body.get('dir') or '')):
        d = os.path.abspath(os.path.join(HERE, str(body.get('dir') or '')))
    root = os.path.abspath(ROOT)
    if not (d == root or d.startswith(root + os.sep)):
        return 400, {'ok': False, 'err': 'bad dir'}
    mp = os.path.join(d, 'meta.json')
    try:
        meta = json.load(open(mp))
    except Exception:
        meta = {}
    if 'success' in body:
        meta['success'] = _norm_success(body['success'])
    if 'note' in body:
        meta['note'] = str(body.get('note') or '')
    if not os.path.isdir(d):
        return 404, {'ok': False, 'err': 'no dir'}
    try:
        json.dump(meta, open(mp, 'w'), ensure_ascii=False, indent=2)
    except Exception as e:
        return 500, {'ok': False, 'err': str(e)}
    try:
        subprocess.run([sys.executable, INDEXER], cwd=BP, timeout=40,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    return 200, {'ok': True, 'meta': meta}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        try:
            self.wfile.write(b)
        except BrokenPipeError:
            pass

    def do_OPTIONS(self):
        self._send({'ok': True})

    def do_GET(self):
        p = self.path.split('?')[0]
        if p == '/health':
            return self._send({'ok': True, 'service': 'collector', 'port': PORT})
        if p == '/status':
            return self._send(_status())
        if p == '/sys':
            return self._send(_sysinfo())
        if p == '/episodes':
            q = {}
            try:
                q = dict(x.split('=', 1) for x in self.path.split('?', 1)[1].split('&')) if '?' in self.path else {}
            except Exception:
                q = {}
            task = q.get('task', 'headfix')
            limit = q.get('limit', '50')
            return self._send({'ok': True, 'task': task, 'episodes': _episodes(task, limit)})
        return self._send({'ok': False, 'err': 'not found'}, 404)

    def do_POST(self):
        p = self.path.split('?')[0]
        try:
            n = int(self.headers.get('Content-Length', '0'))
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception as e:
            return self._send({'ok': False, 'err': 'bad json: %s' % e}, 400)
        if p == '/start':
            code, obj = _start(body)
            return self._send(obj, code)
        if p == '/stop':
            return self._send(_stop())
        if p == '/meta':
            code, obj = _meta(body)
            return self._send(obj, code)
        return self._send({'ok': False, 'err': 'not found'}, 404)


if __name__ == '__main__':
    print('collector service :%d  root=%s' % (PORT, ROOT), flush=True)
    ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
