"""HTTP control API of the executor, for the app (through its dev server).

  GET  /status          executor, arm, ghost run and run progress (JSON)
  GET  /replay          the replay of the last run (hardware + ghost run)
  POST /prepare         {"request": "<live-app request text>"}: load and simulate
  POST /move_to_start   move the arm to the request's start pose
  POST /begin           start the hardware run (refused unless safe and at the start)
  POST /abort           stop the arm now
  POST /reset           forget the experiment

Every POST answers {"ok": true} or {"ok": false, "error": "..."}. Bind to
127.0.0.1 only: the API moves a real arm.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def make_server(executor, host='127.0.0.1', port=8765):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, kind='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()

        def do_GET(self):
            if self.path == '/status':
                self._send(200, executor.status())
            elif self.path == '/replay':
                if not executor.replay_path:
                    self._send(404, {'ok': False, 'error': 'no replay yet'})
                    return
                with open(executor.replay_path, 'rb') as file:
                    self._send(200, file.read())
            else:
                self._send(404, {'ok': False, 'error': 'unknown path'})

        def do_POST(self):
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                self._send(400, {'ok': False, 'error': 'the body is not JSON'})
                return
            actions = {
                '/prepare': lambda: executor.prepare(body.get('request') or ''),
                '/move_to_start': executor.move_to_start,
                '/begin': executor.begin,
                '/abort': lambda: executor.abort(body.get('reason') or 'stopped by the operator'),
                '/reset': executor.reset,
            }
            action = actions.get(self.path)
            if not action:
                self._send(404, {'ok': False, 'error': 'unknown path'})
                return
            try:
                action()
                self._send(200, {'ok': True})
            except (RuntimeError, ValueError) as error:
                self._send(409, {'ok': False, 'error': str(error)})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def serve_in_thread(server):
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return thread
