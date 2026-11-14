"""The scate_hardware child process: commands on stdin, JSON lines on stdout.

See examples/example_scate_ur10e/scate_hardware.cpp for the protocol. A
reader thread queues every message; stderr (the planner's diagnostics) goes
to a log file.
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import tempfile
import threading
import time


def fmt(values):
    return ' '.join(repr(float(x)) for x in values)


class PlannerError(RuntimeError):
    pass


class PlannerProcess:

    def __init__(self, binary, request_text, workdir=None, env=None):
        if not os.path.isfile(binary):
            raise PlannerError(f'scate_hardware not found at {binary} '
                               '(build it: ninja scate_hardware in the GTDynamics build folder)')
        self.workdir = workdir or tempfile.mkdtemp(prefix='scate_hw_')
        self.request_path = os.path.join(self.workdir, 'request.txt')
        with open(self.request_path, 'w') as file:
            file.write(request_text)
        self.log_path = os.path.join(self.workdir, 'planner.log')
        self._log = open(self.log_path, 'w')
        self.messages = queue.Queue()
        self.process = subprocess.Popen(
            [binary, 'serve', self.request_path], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self._log, text=True, bufsize=1, env=env)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        self._lock = threading.Lock()

    def _read(self):
        for line in self.process.stdout:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                self.messages.put(json.loads(line))
            except ValueError:
                self.messages.put({'type': 'error', 'message': f'unreadable planner output: {line[:200]}'})
        self.messages.put({'type': 'exit', 'code': self.process.wait()})

    def send(self, line):
        with self._lock:
            if self.process.poll() is not None:
                raise PlannerError('the planner has exited')
            self.process.stdin.write(line + '\n')
            self.process.stdin.flush()

    def receive(self, timeout):
        """The next message, or None after timeout seconds."""
        try:
            return self.messages.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for(self, kind, timeout):
        """The next message of this type; raises on an error, exit or timeout."""
        deadline = time.monotonic() + timeout
        while True:
            message = self.receive(max(0.0, deadline - time.monotonic()))
            if message is None:
                raise PlannerError(f'no {kind} from the planner within {timeout:.0f} s')
            if message['type'] == kind:
                return message
            if message['type'] in ('error', 'exit'):
                raise PlannerError(message.get('message') or f'planner exited ({message.get("code")})')

    def start(self):
        self.send('start')

    def measure(self, k, t, q, v, obstacle_centers):
        centers = ' '.join(fmt(c) for c in obstacle_centers)
        self.send(f'measure {k} {t!r} {fmt(q)} {fmt(v)} {centers}')

    def trace(self, t, q, v):
        self.send(f'trace {t!r} {fmt(q)} {fmt(v)}')

    def finish(self, status, path):
        self.send(f'finish {status} {path}')

    def close(self):
        try:
            if self.process.poll() is None:
                self.send('quit')
                self.process.wait(timeout=5)
        except (PlannerError, OSError, subprocess.TimeoutExpired):
            self.process.kill()
        finally:
            self._log.close()

    def log_tail(self, lines=20):
        try:
            with open(self.log_path) as file:
                return file.read().splitlines()[-lines:]
        except OSError:
            return []
