"""The arm interface the executor drives, and a simulated arm.

Robot: what the executor needs from an arm.
  state()       latest JointState (monotonic time, q, v), or None
  command(v)    stream a joint velocity command (rad/s); the arm stops on
                its own if commands stop (ur_bridge's watchdog, speedj's time)
  stop()        stop now
  status()      'ready', or why commands would not reach the arm

SimRobot imitates ur_bridge + speedj: commands are followed with a short
lag, the arm stops when commands are older than command_timeout, and a
command faster than max_joint_speed faults the arm, as ur_bridge does.
RosRobot (the real arm through ur_bridge) is in ros2/scate_ur10e_hw.
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass


@dataclass
class JointState:
    t: float      # time.monotonic() of the reading
    q: list
    v: list


class Robot:
    kind = 'robot'

    def state(self):
        raise NotImplementedError

    def command(self, velocity):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def status(self):
        return 'ready'

    def close(self):
        pass


class SimRobot(Robot):
    """A simulated UR10e driven by joint velocity commands, in its own thread."""

    kind = 'sim'

    def __init__(self, q0, lag=0.05, rate_hz=250.0, command_timeout=0.25, max_joint_speed=0.6,
                 noise=0.0, seed=0, clock=time.monotonic):
        self.q = [float(x) for x in q0]
        self.v = [0.0] * len(self.q)
        self.lag, self.dt = lag, 1.0 / rate_hz
        self.command_timeout, self.max_joint_speed = command_timeout, max_joint_speed
        self.noise, self.random = noise, random.Random(seed)
        self.clock = clock
        self.target = [0.0] * len(self.q)
        self.commanded_at = -math.inf
        self.fault = None
        self.lock = threading.Lock()
        self.stamp = clock()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        last = self.clock()
        while self.running:
            time.sleep(self.dt)
            now = self.clock()
            self.step(now - last, now)
            last = now

    def step(self, dt, now):
        """Advance the arm by dt (also callable directly in tests)."""
        with self.lock:
            target = self.target if now - self.commanded_at <= self.command_timeout and not self.fault \
                else [0.0] * len(self.q)
            alpha = 1.0 - math.exp(-dt / self.lag) if self.lag > 0 else 1.0
            for j in range(len(self.q)):
                self.v[j] += alpha * (target[j] - self.v[j])
                self.q[j] += self.v[j] * dt
            self.stamp = now

    def state(self):
        with self.lock:
            q = [x + self.random.gauss(0.0, self.noise) for x in self.q] if self.noise else list(self.q)
            return JointState(self.stamp, q, list(self.v))

    def command(self, velocity):
        with self.lock:
            if self.fault:
                return
            if max(abs(x) for x in velocity) > self.max_joint_speed:
                self.fault = (f'arm command {[round(x, 3) for x in velocity]} exceeds '
                              f'{self.max_joint_speed} rad/s')
                return
            self.target = [float(x) for x in velocity]
            self.commanded_at = self.clock()

    def stop(self):
        with self.lock:
            self.target = [0.0] * len(self.q)
            self.commanded_at = -math.inf

    def status(self):
        return f'fault: {self.fault}' if self.fault else 'ready'

    def reset_fault(self):
        with self.lock:
            self.fault = None

    def close(self):
        self.running = False
