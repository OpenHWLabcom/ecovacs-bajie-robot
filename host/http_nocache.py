#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""http_nocache.py — robot-viz 静态服务器（HTML/JS/CSS/JSON 禁止缓存）
避免改完页面后浏览器仍拿旧 index.html / arm3d.html。
"""
import os, functools
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

PORT = int(os.environ.get('PORT', '8790'))
DIR = os.environ.get('DIR', os.path.dirname(os.path.abspath(__file__)))
NO_CACHE_EXT = ('.html', '.htm', '.js', '.css', '.json')


class H(SimpleHTTPRequestHandler):
    def end_headers(self):
        p = self.path.split('?', 1)[0]
        if p in ('', '/') or p.lower().endswith(NO_CACHE_EXT):
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

    def log_message(self, fmt, *args):
        # 静默；需要排障时看 /tmp/robotviz-8790.log 的启动信息即可
        pass


if __name__ == '__main__':
    handler = functools.partial(H, directory=DIR)
    print('robot-viz static (no-cache) :%d  dir=%s' % (PORT, DIR), flush=True)
    ThreadingHTTPServer(('0.0.0.0', PORT), handler).serve_forever()
