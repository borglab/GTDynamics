"""Tests of the executor (python3 -m unittest discover -s tests, from hardware/).

The end-to-end tests run the real planner (scate_hardware, found as in
python3 -m scate_hw, or SCATE_HARDWARE_BIN) against the simulated arm in
real time (about 25 s each); they are skipped if it is not built.
"""

import json
import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scate_hw import kinematics  # noqa: E402
from scate_hw.__main__ import default_binary, default_description  # noqa: E402
from scate_hw.executor import Executor  # noqa: E402
from scate_hw.plan import Plan, TimedPath  # noqa: E402
from scate_hw.request import Request  # noqa: E402
from scate_hw.robots import SimRobot  # noqa: E402
from scate_hw.safety import Limits, Settings, check_path, command  # noqa: E402

DESCRIPTION = default_description()
BINARY = default_binary()

# The MK 111 rig experiment (setMk111RigExperiment), as the app sends it for hardware.
REQUEST = """robot ur10e
q_start 0 -1.5708 1.5708 -1.5708 -1.5708 0
q_goal -1.6 -1.2 1.3 -1.9 -1.8 0.4
obstacle 0.72 -0.47 0.78 0 0 0 0.06
obstacle 0.45 -0.75 1.0 0 0 -0.012 0.06
sigmas 3
seed 0
plane wall 0 0 1 -0.05
plane floor 0 -1 0 -0.975
sensing encoders
hardware_sigma 3
"""
START = [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]
RIG = [{'name': 'wall', 'normal': [0, 0, 1], 'offset': -0.05},
       {'name': 'floor', 'normal': [0, -1, 0], 'offset': -0.975}]


def limits(v=0.5, u=1.0, n=6):
    return Limits({'v_limit': [v] * n, 'u_limit': [u] * n, 'q_lower': [-6.3] * n, 'q_upper': [6.3] * n,
                   'dt': 1.0, 'num_intervals': 20})


class KinematicsTest(unittest.TestCase):

    def setUp(self):
        self.robot = kinematics.load_description(DESCRIPTION)

    def test_spheres_match_the_planner(self):
        # Sphere centers the C++ planner wrote for q = START (replay frame 0).
        expected = {0: [0, 0, 0.1807], 1: [0, 0.176, 0.1807], 2: [-5.62643e-07, 0.176, 0.333875],
                    7: [0.142885, 0.043, 0.7934], 10: [0.571548, 0.043, 0.7934]}
        spheres = kinematics.sphere_centers(self.robot, START)
        for index, center in expected.items():
            for a, b in zip(spheres[index][0], center):
                self.assertAlmostEqual(a, b, places=5)

    def test_plane_clearance(self):
        gap, name = kinematics.plane_clearance(self.robot, START, RIG)
        brute = min(kinematics.plane_distance(p, c) - r
                    for c, r in kinematics.sphere_centers(self.robot, START) for p in RIG)
        self.assertAlmostEqual(gap, brute, places=12)
        self.assertIn(name, ('wall', 'floor'))
        # The level, stretched arm over base +y is through the floor.
        gap, name = kinematics.plane_clearance(self.robot, [math.pi / 2, -0.3, 0, -1.5708, -1.5708, 0], RIG)
        self.assertLess(gap, 0)
        self.assertEqual(name, 'floor')
        self.assertEqual(kinematics.plane_clearance(self.robot, START, []), (math.inf, None))


class PlanTest(unittest.TestCase):

    def test_double_integrator(self):
        q = [[0.0], [0.5], [1.5]]
        v = [[0.0], [1.0], [1.0]]
        u = [[1.0], [0.0]]
        plan = Plan(q, v, u, 1.0)
        self.assertEqual(plan.evaluate(0.5)[0:2], ([0.125], [0.5]))
        self.assertEqual(plan.evaluate(1.0)[0:2], ([0.5], [1.0]))
        self.assertEqual(plan.evaluate(1.5)[0], [1.0])
        self.assertEqual(plan.evaluate(5.0)[0:2], ([1.5], [0.0]))   # held at the end, at rest
        self.assertEqual(plan.evaluate(-1.0)[0], [0.0])
        self.assertEqual(plan.samples(1.5, 2), [1.5, 2.0])
        with self.assertRaises(ValueError):
            Plan(q, v, [[1.0]], 1.0)

    def test_timed_path(self):
        path = TimedPath.at_speed([0.0, 0.0], [0.4, -0.2], peak_speed=0.2, min_duration=0.1)
        self.assertAlmostEqual(path.duration, 1.875 * 0.4 / 0.2)
        self.assertEqual(path.evaluate(0)[0], [0.0, 0.0])
        self.assertEqual(path.evaluate(path.duration)[0], [0.4, -0.2])
        peak = max(abs(path.evaluate(path.duration * i / 100)[1][0]) for i in range(101))
        self.assertAlmostEqual(peak, 0.2, places=6)


class SafetyTest(unittest.TestCase):

    def test_command_law(self):
        s = Settings(kp=2.0, max_correction=0.1, command_cap=0.55)
        self.assertEqual(command([1.0], [0.2], [0.98], s), [0.2 + 2.0 * 0.02000000000000002])
        self.assertEqual(command([1.0], [0.2], [0.0], s), [0.30000000000000004])  # correction clipped
        self.assertEqual(command([0.0], [-0.6], [0.0], s), [-0.55])              # hard cap

    def test_check_path_flags_limits(self):
        s, robot = Settings(), kinematics.load_description(DESCRIPTION)
        n = 6
        still = Plan([START, START], [[0] * n] * 2, [[0] * n], 1.0)
        self.assertEqual(check_path(still, 0, limits(), s, robot, RIG), [])
        fast = Plan([START, START], [[0.6] + [0] * 5, [0] * n], [[0] * n], 1.0)
        self.assertTrue(any('speed' in p for p in check_path(fast, 0, limits(), s)))
        hard = Plan([START, START], [[0] * n] * 2, [[0, 1.2, 0, 0, 0, 0]], 1.0)
        self.assertTrue(any('acceleration' in p for p in check_path(hard, 0, limits(), s)))
        tight = Limits({'v_limit': [0.5] * n, 'u_limit': [1] * n, 'q_lower': [-1] * n,
                        'q_upper': [1] * n, 'dt': 1, 'num_intervals': 1})
        self.assertTrue(any('limits' in p for p in check_path(still, 0, tight, s)))
        floor = [math.pi / 2, -0.3, 0, -1.5708, -1.5708, 0]
        through = Plan([floor, floor], [[0] * n] * 2, [[0] * n], 1.0)
        self.assertTrue(any('floor' in p for p in check_path(through, 0, limits(), s, robot, RIG)))


class RequestTest(unittest.TestCase):

    def test_parse(self):
        request = Request(REQUEST)
        self.assertEqual(request.q_start, START)
        self.assertEqual(len(request.obstacles), 2)
        self.assertEqual(request.obstacle_centers(10.0)[1], [0.45, -0.75, 1.0 - 0.12])
        self.assertEqual([p['name'] for p in request.planes], ['wall', 'floor'])
        for bad in (REQUEST.replace('sensing encoders', 'sensing markers'),
                    REQUEST.replace('q_goal -1.6', 'q_goal'), 'q_start 0\nq_goal 1\nsensing encoders\n'):
            with self.assertRaises(ValueError):
                Request(bad)


class SimRobotTest(unittest.TestCase):

    def test_follows_and_stops(self):
        clock = [0.0]
        robot = SimRobot([0.0, 0.0], lag=0.05, clock=lambda: clock[0])
        robot.running = False
        robot.command([0.2, -0.1])
        for _ in range(100):  # 0.2 s at 2 ms
            clock[0] += 0.002
            robot.step(0.002, clock[0])
        state = robot.state()
        self.assertAlmostEqual(state.v[0], 0.2, places=2)
        clock[0] += 0.0  # commands older than the timeout stop the arm
        for _ in range(500):
            clock[0] += 0.002
            robot.step(0.002, clock[0])
        self.assertAlmostEqual(robot.state().v[0], 0.0, places=6)
        robot.command([0.7, 0.0])
        self.assertTrue(robot.status().startswith('fault'))


@unittest.skipUnless(os.path.isfile(BINARY), f'scate_hardware not built ({BINARY})')
class EndToEndTest(unittest.TestCase):
    """The executor with the real planner and the simulated arm, in real time."""

    def make(self, q0=START, request=REQUEST):
        self.robot = SimRobot(q0, lag=0.05)
        self.executor = Executor(self.robot, BINARY, DESCRIPTION)
        self.addCleanup(self.robot.close)
        self.addCleanup(self.executor.close)
        self.executor.prepare(request)
        self.assertEqual(self.executor.wait(120), 'ready', self.executor.message)
        return self.executor

    def test_full_run(self):
        executor = self.make()
        status = executor.status()
        self.assertTrue(status['ghost']['safety']['safe'])
        self.assertTrue(status['can_begin'], status['begin_blocked'])
        executor.begin()
        worst = 0.0
        while executor.busy():
            worst = max(worst, executor.run_info.get('tracking_error') or 0.0)
            time.sleep(0.1)
        self.assertEqual(executor.state, 'done', executor.message)
        result = executor.result
        print(f'\n  [diag] sim arm: worst tracking {worst:.4f} rad, goal error '
              f'{result["summary"]["goal_error"]:.4f} rad, wall/floor '
              f'{100 * result["summary"]["min_environment_clearance"]:.2f} cm, obstacles '
              f'{100 * result["summary"]["min_obstacle_clearance"]:.2f} cm, slowest re-plan '
              f'{result["summary"]["max_solve_ms"]:.0f} ms')
        self.assertTrue(result['safety']['safe'], result['safety']['reasons'])
        self.assertLess(result['summary']['goal_error'], 0.05)
        self.assertLess(worst, 0.03)
        with open(executor.replay_path) as file:
            replay = json.load(file)
        self.assertEqual([run['name'] for run in replay['runs']], ['online', 'ghost'])
        self.assertEqual(replay['runs'][0]['label'], 'Hardware run')
        self.assertEqual(len(replay['runs'][0]['frames']), len(replay['runs'][1]['frames']))
        self.assertEqual(len(replay['runs'][0]['steps']), 21)

    def test_abort_and_move_to_start(self):
        executor = self.make(q0=[x + 0.1 for x in START])
        self.assertFalse(executor.status()['can_begin'])
        with self.assertRaises(RuntimeError):
            executor.begin()
        executor.move_to_start()
        self.assertEqual(executor.wait(30), 'ready', executor.message)
        self.assertTrue(executor.status()['at_start'], executor.status()['start_distance'])
        executor.begin()
        time.sleep(3.0)
        executor.abort('test')
        self.assertEqual(executor.wait(60), 'aborted')
        self.assertIn('test', executor.message)
        self.assertEqual(executor.result['status'], 'aborted')
        time.sleep(0.5)  # the simulated arm decelerates with its 50 ms lag
        self.assertLess(max(abs(v) for v in self.robot.state().v), 1e-3)
        with open(executor.replay_path) as file:
            self.assertIn('stopped early', file.read())

    def test_unsafe_ghost_cannot_begin(self):
        # A goal that ends stretched out through the floor.
        request = REQUEST.replace('q_goal -1.6 -1.2 1.3 -1.9 -1.8 0.4', 'q_goal 1.5708 -0.3 0 -1.5708 -1.5708 0')
        self.robot = SimRobot(START)
        self.executor = Executor(self.robot, BINARY, DESCRIPTION)
        self.addCleanup(self.robot.close)
        self.addCleanup(self.executor.close)
        self.executor.prepare(request)
        self.assertEqual(self.executor.wait(120), 'unsafe')
        self.assertIn('goal pose goes through the floor', self.executor.message)
        with self.assertRaises(RuntimeError):
            self.executor.begin()


if __name__ == '__main__':
    unittest.main()
