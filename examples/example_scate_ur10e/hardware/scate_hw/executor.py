"""The experiment state machine and the control loop of a hardware run.

  idle --prepare--> preparing --ghost safe--> ready --begin--> running --> done
                               \\-ghost unsafe-> unsafe          \\-> aborted
  ready --move_to_start--> moving --> ready

prepare(request) starts the planner (scate_hardware), which simulates the
ghost run first; the run may only begin if that ghost run is safe, the arm
is ready and it is at the request's start (move_to_start takes it there).

During the run (running), at each state time k dt the executor reads the
arm and the obstacles and sends them to the planner, then keeps following
the current plan until the planner answers with the new one, which it
checks (safety.check_path, and that it starts where the arm is) before
switching to it. Every control period it commands
  v = v_ref(t) + clip(kp (q_ref(t) - q)),
and it stops the arm if the arm's state goes stale, the arm lags the plan
by more than tracking_abort, a plan fails its check, the planner is late
or fails, the arm reports a fault, or the operator aborts. After the last
state it holds the final pose for settle_time, stops, and has the planner
write the replay (the hardware run beside its ghost run).
"""

from __future__ import annotations

import collections
import os
import threading
import time
import traceback

from . import kinematics
from .plan import Plan, TimedPath
from .planner import PlannerError, PlannerProcess
from .request import Request
from .safety import Limits, Settings, check_path, command, hold


class Abort(Exception):
    """Stop the arm and end the run, with the reason."""


class Executor:

    def __init__(self, robot, planner_binary, description=None, settings=None, workdir=None,
                 planner_env=None, clock=time.monotonic, sleep=time.sleep):
        self.robot = robot
        self.planner_binary = planner_binary
        self.description = kinematics.load_description(description) if isinstance(description, str) \
            else description
        self.settings = settings or Settings()
        self.workdir = workdir
        self.planner_env = planner_env
        self.clock, self.sleep = clock, sleep
        self.lock = threading.RLock()
        self.log_lines = collections.deque(maxlen=200)
        self._reset_fields()

    # ----------------------------------------------------------------- state
    def _reset_fields(self):
        self.state, self.message = 'idle', 'No experiment loaded.'
        self.request = self.planner = self.limits = self.ghost = None
        self.plan = None
        self.run_info = {}
        self.replay_path = None
        self.result = None
        self.abort_requested = None
        self.thread = None

    def log(self, text):
        line = f'{time.strftime("%H:%M:%S")} {text}'
        with self.lock:
            self.log_lines.append(line)
        print(f'[scate_hw] {text}', flush=True)

    def _set(self, state, message):
        with self.lock:
            self.state, self.message = state, message
        self.log(f'{state}: {message}')

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def _start_thread(self, target, *args):
        if self.busy():
            raise RuntimeError(f'busy ({self.state})')
        self.thread = threading.Thread(target=self._guard, args=(target,) + args, daemon=True)
        self.thread.start()

    def _guard(self, target, *args):
        try:
            target(*args)
        except Exception as error:  # never leave the arm moving on a bug
            self.robot.stop()
            self.log(traceback.format_exc())
            self._set('error', f'{type(error).__name__}: {error}')

    def wait(self, timeout=None):
        """Wait for the current step (prepare, move, run) to finish; for tests and scripts."""
        if self.thread:
            self.thread.join(timeout)
        return self.state

    def start_distance(self, state=None):
        state = state or self.robot.state()
        if state is None or self.request is None:
            return None
        return max(abs(a - b) for a, b in zip(state.q, self.request.q_start))

    def status(self):
        state = self.robot.state()
        now = self.clock()
        with self.lock:
            distance = self.start_distance(state)
            return {
                'state': self.state,
                'message': self.message,
                'robot': {
                    'kind': self.robot.kind,
                    'status': self.robot.status(),
                    'q': state.q if state else None,
                    'v': state.v if state else None,
                    'age': (now - state.t) if state else None,
                },
                'start_distance': distance,
                'at_start': distance is not None and distance <= self.settings.start_tolerance,
                'ghost': self.ghost,
                'run': dict(self.run_info),
                'replay_ready': self.replay_path is not None and os.path.isfile(self.replay_path),
                'result': self.result,
                'can_begin': self._can_begin(state, distance)[0],
                'begin_blocked': self._can_begin(state, distance)[1],
                'log': list(self.log_lines)[-30:],
            }

    def _can_begin(self, state, distance):
        if self.state != 'ready':
            return False, f'the executor is {self.state}'
        if not (self.ghost and self.ghost['safety']['safe']):
            return False, 'the ghost run is not safe'
        if self.robot.status() != 'ready':
            return False, f'the arm is not ready: {self.robot.status()}'
        if state is None:
            return False, 'no state from the arm'
        if distance is None or distance > self.settings.start_tolerance:
            return False, f'the arm is {distance:.3f} rad from the start pose: move it there first'
        return True, ''

    # --------------------------------------------------------------- actions
    def prepare(self, request_text):
        """Load a request: start the planner, which simulates and checks the ghost run."""
        if self.busy():
            raise RuntimeError(f'busy ({self.state})')
        request = Request(request_text)  # raises on a bad request
        if self.description and len(request.q_start) != len(self.description['joints']):
            raise ValueError('the request does not match the robot description')
        self._close_planner()
        self._reset_fields()
        self.request = request
        self._set('preparing', 'Simulating the ghost run.')
        self._start_thread(self._prepare)

    def _prepare(self):
        self.planner = PlannerProcess(self.planner_binary, self.request.text, self.workdir,
                                      self.planner_env)
        try:
            message = self.planner.wait_for('ghost', self.settings.ghost_timeout)
        except PlannerError as error:
            self._close_planner()
            self._set('error', f'The planner failed: {error}')
            return
        self.limits = Limits(message)
        safety = dict(message['safety'])
        safety['reasons'] = list(safety['reasons']) + self._pose_problems()
        safety['safe'] = safety['safe'] and not safety['reasons']
        self.ghost = {'safety': safety, 'summary': message['summary'], 'sigma': message['sigma']}
        message['safety'] = safety
        if message['safety']['safe']:
            self._set('ready', 'The ghost run is safe.')
        else:
            self._set('unsafe', 'The ghost run is not safe: ' + '; '.join(message['safety']['reasons']))

    def _pose_problems(self):
        """The start and goal poses must be clear of the walls and floor (the planner's own
        check is soft: it stops short of a goal it cannot reach, and calls that safe)."""
        problems = []
        if not (self.description and self.request.planes):
            return problems
        for label, q in (('start', self.request.q_start), ('goal', self.request.q_goal)):
            gap, name = kinematics.plane_clearance(self.description, q, self.request.planes)
            if gap < self.settings.min_environment_clearance:
                problems.append(f'the {label} pose goes through the {name}')
        return problems

    def move_to_start(self):
        if self.state != 'ready':
            raise RuntimeError(f'cannot move to the start while {self.state}')
        if self.robot.status() != 'ready':
            raise RuntimeError(f'the arm is not ready: {self.robot.status()}')
        self._start_thread(self._move_to_start)

    def _move_to_start(self):
        state = self.robot.state()
        path = TimedPath.at_speed(state.q, self.request.q_start, self.settings.move_speed)
        problems = check_path(path, 0.0, self.limits, self.settings, self.description,
                              self.request.planes)
        if problems:
            self._set('ready', 'Not moving to the start: ' + '; '.join(problems))
            return
        self._set('moving', f'Moving to the start pose ({path.duration:.1f} s).')
        try:
            self._follow(path, path.duration + self.settings.settle_time)
        except Abort as reason:
            self.robot.stop()
            self._set('ready', f'Move to the start stopped: {reason}')
            return
        self.robot.stop()
        self._set('ready', 'At the start pose.')

    def _follow(self, path, until):
        """Track a TimedPath until time `until`; raises Abort on a problem."""
        period = 1.0 / self.settings.rate_hz
        t0 = self.clock()
        while True:
            self._check_abort()
            now = self.clock()
            t = now - t0
            if t >= until:
                return
            state = self._fresh_state(now)
            q_ref, v_ref, _ = path.evaluate(t)
            self._check_tracking(q_ref, state.q)
            self.robot.command(command(q_ref, v_ref, state.q, self.settings))
            self.sleep(max(0.0, t0 + (int(t / period) + 1) * period - self.clock()))

    def begin(self):
        """Start the hardware run, if the ghost run is safe and the arm is ready at the start."""
        ok, why = self._can_begin(self.robot.state(), self.start_distance())
        if not ok:
            raise RuntimeError(why)
        self._set('running', 'Hardware run started.')
        self._start_thread(self._run)

    def abort(self, reason='stopped by the operator'):
        self.robot.stop()
        with self.lock:
            self.abort_requested = reason
        self.log(f'abort requested: {reason}')

    def reset(self):
        if self.busy():
            raise RuntimeError(f'busy ({self.state}); abort first')
        self._close_planner()
        self._reset_fields()
        self.log('reset')

    def close(self):
        self.abort('executor shutting down')
        if self.thread:
            self.thread.join(5)
        self._close_planner()

    def _close_planner(self):
        if self.planner:
            self.planner.close()
            self.planner = None

    # ------------------------------------------------------------ the run
    def _check_abort(self):
        with self.lock:
            reason, self.abort_requested = self.abort_requested, None
        if reason:
            raise Abort(reason)
        status = self.robot.status()
        if status != 'ready':
            raise Abort(f'the arm reports: {status}')

    def _fresh_state(self, now):
        state = self.robot.state()
        if state is None or now - state.t > self.settings.stale_after:
            raise Abort('the arm state is stale')
        return state

    def _check_tracking(self, q_ref, q):
        error = max(abs(a - b) for a, b in zip(q_ref, q))
        if error > self.settings.tracking_abort:
            raise Abort(f'the arm is {error:.3f} rad from the plan (limit {self.settings.tracking_abort})')
        return error

    def _accept(self, plan, t, state):
        problems = check_path(plan, t, self.limits, self.settings, self.description, self.request.planes)
        q_ref, _, _ = plan.evaluate(t)
        jump = max(abs(a - b) for a, b in zip(q_ref, state.q))
        if jump > self.settings.tracking_abort:
            problems.append(f'the plan starts {jump:.3f} rad from the arm')
        if problems:
            raise Abort(f'plan after state {plan.k} refused: ' + '; '.join(problems[:3]))
        return plan

    def _run(self):
        s, planner, request = self.settings, self.planner, self.request
        n_intervals, dt = self.limits.num_intervals, self.limits.dt
        duration = n_intervals * dt
        trace, period = [], 1.0 / s.rate_hz
        status = 'completed'
        try:
            planner.start()
            first = Plan.from_message(planner.wait_for('plan', 60.0))
            start_state = self._fresh_state(self.clock())
            self.plan = self._accept(first, 0.0, start_state)
            t0 = self.clock()
            k, awaiting, sent_at = 0, 0, t0
            planner.measure(0, 0.0, start_state.q, start_state.v, request.obstacle_centers(0.0))
            solve_ms, next_trace, settle_until = [], 0.0, None
            self.run_info = {'t': 0.0, 'k': 0, 'duration': duration, 'dt': dt}
            while True:
                self._check_abort()
                now = self.clock()
                t = now - t0
                state = self._fresh_state(now)
                # New plans from the planner.
                while True:
                    message = planner.receive(0.0)
                    if message is None:
                        break
                    if message['type'] == 'plan':
                        plan = Plan.from_message(message, t)
                        if plan.k != awaiting:
                            raise Abort(f'unexpected plan for state {plan.k} (waiting for {awaiting})')
                        self.plan = self._accept(plan, t, state)
                        solve_ms.append(message['solve_ms'])
                        awaiting = None
                    elif message['type'] in ('error', 'exit'):
                        raise Abort(f'planner: {message.get("message") or "exited"}')
                if awaiting is not None and now - sent_at > s.late_fraction * dt:
                    raise Abort(f're-plan after state {awaiting} took over {s.late_fraction * dt:.2f} s')
                # The next state: read the arm and the obstacles, ask for a new plan.
                if awaiting is None and k < n_intervals and t >= (k + 1) * dt:
                    k += 1
                    planner.measure(k, t, state.q, state.v, request.obstacle_centers(t))
                    awaiting, sent_at = k, now
                # Command.
                if k == n_intervals and awaiting is None and t >= duration:
                    settle_until = settle_until or now + s.settle_time
                    if now >= settle_until:
                        break
                    q_ref, v_ref = self.plan.q[-1], [0.0] * len(state.q)
                else:
                    q_ref, v_ref, _ = self.plan.evaluate(t)
                error = self._check_tracking(q_ref, state.q)
                self.robot.command(command(q_ref, v_ref, state.q, s))
                if t >= next_trace:
                    trace.append((t, state.q, state.v))
                    next_trace = t + 1.0 / s.trace_hz
                with self.lock:
                    self.run_info = {
                        't': t, 'k': k, 'duration': duration, 'dt': dt, 'tracking_error': error,
                        'last_solve_ms': solve_ms[-1] if solve_ms else None,
                        'max_solve_ms': max(solve_ms) if solve_ms else None,
                        'obstacles': request.obstacle_centers(t),
                        'plan_k': self.plan.k, 'plan_q': self.plan.q,
                        'q_ref': q_ref,
                    }
                self.sleep(max(0.0, t0 + (int(t / period) + 1) * period - self.clock()))
            self.robot.stop()
            self._set('finishing', 'Run complete; writing the replay.')
        except Abort as reason:
            self.robot.stop()
            status = 'aborted'
            self._set('finishing', f'Stopped: {reason}; writing the replay.')
            stop_reason = str(reason)
        except PlannerError as error:
            self.robot.stop()
            status = 'aborted'
            stop_reason = f'planner: {error}'
            self._set('finishing', f'Stopped: {stop_reason}; writing the replay.')
        self._finish(status, trace, stop_reason if status == 'aborted' else None)

    def _finish(self, status, trace, reason):
        path = os.path.join(self.planner.workdir, 'replay.json')
        try:
            # Drain stale messages, then send the trace and ask for the replay.
            while self.planner.receive(0.0) is not None:
                pass
            for t, q, v in trace:
                self.planner.trace(t, q, v)
            self.planner.finish(status, path)
            message = self.planner.wait_for('replay', 120.0)
            self.replay_path = path
            self.result = {'status': status, 'reason': reason, 'safety': message['safety'],
                           'summary': message['summary']}
        except PlannerError as error:
            self.result = {'status': status, 'reason': reason, 'replay_error': str(error)}
        if status == 'completed':
            self._set('done', 'Hardware run complete.')
        else:
            self._set('aborted', f'Hardware run stopped: {reason}')
