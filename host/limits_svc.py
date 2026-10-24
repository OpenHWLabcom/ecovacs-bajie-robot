#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""limits_svc.py — 限位数据库 HTTP 服务（:8788）
GET  /limits     全部限位点 + 统计
GET  /stats      统计
POST /sync       points.json → SQLite
GET  /export.csv 导出 CSV
GET  /health
"""
import csv, io, json, os
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import limits_db

PORT = int(os.environ.get('LIMITS_PORT', '8788'))


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, obj, code=200, ctype='application/json; charset=utf-8', raw=None):
        b = raw if raw is not None else json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_OPTIONS(self):
        self._send({'ok': True})

    def do_GET(self):
        p = self.path.split('?')[0]
        if p == '/health':
            return self._send({'ok': True, 'service': 'limits', 'port': PORT})
        if p == '/limits':
            return self._send({'ok': True, 'points': limits_db.list_points(), 'stats': limits_db.stats()})
        if p == '/stats':
            return self._send({'ok': True, **limits_db.stats()})
        if p == '/export.csv':
            pts = limits_db.list_points()
            buf = io.StringIO()
            cols = ['id','label','ts','side','contact_camera','contact_forearm','contact_gripper','contact_motor',
                    'tool_elev','grip_x_face','grip_y_face','limit_angle','cam_opt_elev','lift','finger',
                    'tip_above_table','cam_above_table','table_z','note']
            w = csv.DictWriter(buf, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            for r in pts:
                w.writerow(r)
            return self._send(None, ctype='text/csv; charset=utf-8', raw=buf.getvalue().encode())
        return self._send({'ok': False, 'err': 'not found'}, 404)

    def do_POST(self):
        p = self.path.split('?')[0]
        if p == '/sync':
            n = limits_db.sync()
            return self._send({'ok': True, 'synced': n, 'stats': limits_db.stats()})
        return self._send({'ok': False, 'err': 'not found'}, 404)


if __name__ == '__main__':
    limits_db.sync()
    print('limits service :%d  db=%s' % (PORT, limits_db.DB), flush=True)
    ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
