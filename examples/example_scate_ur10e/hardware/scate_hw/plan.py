"""A plan from the planner, evaluated at any time of the run.

SCATE's model is a double integrator with the acceleration u_i held
constant over each interval, so between states the plan is exact:
  q(t) = q_i + tau v_i + tau^2 u_i / 2,  v(t) = v_i + tau u_i,  tau = t - i dt.
Before the start the plan holds q_0; after the end it holds q_N at rest.
"""

from __future__ import annotations

import math


class Plan:

    def __init__(self, q, v, u, dt, k=-1, solve_ms=0.0, received_at=None):
        if len(q) != len(v) or len(u) != len(q) - 1 or len(q) < 2:
            raise ValueError('a plan needs N+1 states and N controls')
        if not dt > 0:
            raise ValueError('dt must be positive')
        self.q = [list(map(float, x)) for x in q]
        self.v = [list(map(float, x)) for x in v]
        self.u = [list(map(float, x)) for x in u]
        self.dt = float(dt)
        self.k = k                  # state index of the reading it follows (-1 = offline)
        self.solve_ms = solve_ms
        self.received_at = received_at

    @classmethod
    def from_message(cls, message, received_at=None):
        return cls(message['q'], message['v'], message['u'], message['dt'], message.get('k', -1),
                   message.get('solve_ms', 0.0), received_at)

    @property
    def num_intervals(self):
        return len(self.u)

    @property
    def duration(self):
        return self.num_intervals * self.dt

    @property
    def num_joints(self):
        return len(self.q[0])

    def interval(self, t):
        """Index of the interval holding time t (clamped to the plan)."""
        return min(max(int(math.floor(t / self.dt)), 0), self.num_intervals - 1)

    def evaluate(self, t):
        """(q, v, u) at time t from the start of the run."""
        if t <= 0.0:
            return list(self.q[0]), list(self.v[0]), list(self.u[0])
        if t >= self.duration:
            return list(self.q[-1]), [0.0] * self.num_joints, [0.0] * self.num_joints
        i = self.interval(t)
        tau = t - i * self.dt
        q = [q0 + tau * v0 + 0.5 * tau * tau * a for q0, v0, a in zip(self.q[i], self.v[i], self.u[i])]
        v = [v0 + tau * a for v0, a in zip(self.v[i], self.u[i])]
        return q, v, list(self.u[i])

    def samples(self, t_from, per_interval=20):
        """Times from t_from to the end, per_interval per interval, ending at the end."""
        step = self.dt / per_interval
        t = max(0.0, t_from)
        out = []
        while t < self.duration:
            out.append(t)
            t += step
        out.append(self.duration)
        return out


class TimedPath:
    """A rest-to-rest move from q_a to q_b (smootherstep timing), for moving to the start."""

    def __init__(self, q_a, q_b, duration):
        if not duration > 0:
            raise ValueError('duration must be positive')
        self.q_a, self.q_b, self.duration = list(q_a), list(q_b), float(duration)

    @classmethod
    def at_speed(cls, q_a, q_b, peak_speed, min_duration=2.0):
        """Duration so the fastest joint peaks at peak_speed (smootherstep peaks at 15/8 of the mean)."""
        distance = max(abs(b - a) for a, b in zip(q_a, q_b))
        return cls(q_a, q_b, max(min_duration, 1.875 * distance / peak_speed))

    def evaluate(self, t):
        s = min(max(t / self.duration, 0.0), 1.0)
        shape = s ** 3 * (10 - 15 * s + 6 * s * s)
        rate = 30 * s * s * (1 - s) ** 2 / self.duration
        q = [a + shape * (b - a) for a, b in zip(self.q_a, self.q_b)]
        v = [rate * (b - a) for a, b in zip(self.q_a, self.q_b)]
        return q, v, [0.0] * len(q)

    def samples(self, t_from=0.0, count=100):
        return [self.duration * i / count for i in range(count + 1) if self.duration * i / count >= t_from]
