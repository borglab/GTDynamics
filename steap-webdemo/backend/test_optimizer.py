#!/usr/bin/env python3
"""Exercise the real native protocol, convergence, frame conventions and replans."""
import copy
import json
import math
import pathlib
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET


EXECUTABLE = pathlib.Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else (
    pathlib.Path(__file__).resolve().parents[2] / "build/steap-webdemo/backend/steap_optimizer"
)
URDF = pathlib.Path(__file__).resolve().parents[2] / "models/urdfs/panda/panda.urdf"


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b)))
             for j in range(len(b[0]))] for i in range(len(a))]


def transform(position=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, position[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, position[1]],
            [-sp, cp*sr, cp*cr, position[2]], [0, 0, 0, 1]]


def forward(q):
    """Independent URDF link-origin FK; no GTDynamics COM assumptions."""
    # The repository URDF contains unused finger xacro tags without declaring
    # their namespace. Bind it for the strict XML parser; joint1..7 are plain URDF.
    xml = ET.fromstring(URDF.read_text().replace(
        "<robot ", '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ', 1))
    result = transform()
    for i, angle in enumerate(q, 1):
        joint = xml.find(f"joint[@name='joint{i}']")
        origin = joint.find("origin")
        result = multiply(result, transform(
            list(map(float, origin.get("xyz", "0 0 0").split())),
            list(map(float, origin.get("rpy", "0 0 0").split()))))
        result = multiply(result, transform(rpy=(0, 0, angle)))
    return multiply(result, transform(position=(0, 0, .2104), rpy=(0, 0, -math.pi/4)))


class NativeDemoTest(unittest.TestCase):
    def setUp(self):
        self.process = subprocess.Popen([str(EXECUTABLE)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1)
        self.request_id = 0

    def tearDown(self):
        self.process.stdin.close()
        self.process.wait(timeout=5)
        self.process.stdout.close()

    def send(self, command, **kwargs):
        self.request_id += 1
        self.process.stdin.write(json.dumps(dict(command=command, request_id=self.request_id, **kwargs)) + "\n")
        self.process.stdin.flush()
        response = json.loads(self.process.stdout.readline())
        self.assertEqual(response["request_id"], self.request_id)
        return response

    def solve(self, initial):
        state = initial
        errors = [state["error"]]
        while not state.get("done", state["converged"]):
            state = self.send("iterate")
            self.assertEqual(state["type"], "iteration")
            self.assertLessEqual(state["iteration"], 80)
            self.assertLessEqual(state["error"], errors[-1] * (1 + 1e-10) + 1e-8)
            errors.append(state["error"])
        self.assertLess(state["error"], errors[0] * .01)
        return state

    def check_solution(self, state):
        self.assertTrue(state["converged"], state["stop_reason"])
        self.assertTrue(state["targets_reached"], state["endpoint_error"])
        for name, error in state["endpoint_error"].items():
            tolerance = 1e-4 if name.endswith("position") else 1e-3
            self.assertLess(error, tolerance, f"{name}={error}")
        for sample in state["states"]:
            self.assertEqual(len(sample["q"]), 7)
            self.assertEqual(len(sample["v"]), 7)
            self.assertEqual(len(sample["links"]), 8)
            self.assertEqual(len(sample["com_poses"]), 8)
            self.assertEqual(len(sample["twists"]), 8)
            self.assertTrue(all(len(twist) == 6 for twist in sample["twists"]))
            actual = forward(sample["q"])
            for i in range(3):
                self.assertAlmostEqual(actual[i][3], sample["ee"]["position"][i], delta=2e-4)
            x, y, z, w = sample["ee"]["quaternion"]
            rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(actual[i][j], rotation[i][j], delta=2e-4)
        self.assertLess(max(abs(v) for v in state["states"][0]["v"]), .005)
        self.assertLess(max(abs(v) for v in state["states"][-1]["v"]), .005)

    def test_convergence_retarget_and_frames(self):
        description = self.send("describe")
        self.assertEqual(description["joint_names"], [f"joint{i}" for i in range(1, 8)])
        request = dict(intermediate_states=10, duration=4,
                       start=description["start"], goal=description["goal"])
        state = self.send("build", **request)
        self.assertEqual(state["type"], "built")
        self.assertEqual(len(state["states"]), 12)
        self.assertEqual(state["graph"]["variables"], 30*12)
        self.assertEqual(state["graph"]["scalar_dimensions"], 110*12)
        state = self.solve(state)
        self.check_solution(state)
        goal = copy.deepcopy(description["goal"])
        goal["position"][0] -= .035
        goal["position"][2] += .025
        # Rotate the interactive goal 0.08 rad about world Z.
        x, y, z, w = goal["quaternion"]
        s, c = math.sin(.04), math.cos(.04)
        goal["quaternion"] = [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z]
        updated = self.send("targets", start=description["start"], goal=goal)
        self.assertEqual(updated["iteration"], 0)
        self.assertEqual(updated["graph"], state["graph"])
        self.assertEqual(updated["states"], state["states"])
        solved = self.solve(updated)
        self.check_solution(solved)
        self.assertNotEqual(solved["states"][-1]["q"], state["states"][-1]["q"])
        finished = self.send("iterate")
        self.assertEqual(finished["iteration"], solved["iteration"])
        self.assertEqual(finished["states"], solved["states"])
        print(f"Default solve {state['iteration']} iterations; live replan {solved['iteration']} iterations.")

    def test_validation_and_recovery(self):
        description = self.send("describe")
        self.assertEqual(self.send("iterate")["type"], "error")
        request = dict(intermediate_states=4, duration=4,
                       start=description["start"], goal=description["goal"])
        for field, value in [("duration", 0), ("duration", 21),
                             ("intermediate_states", 0), ("intermediate_states", 41),
                             ("intermediate_states", 3.5)]:
            self.assertEqual(self.send("build", **dict(request, **{field: value}))["type"], "error")
        bad_pose = dict(position=[0, 0, .5], quaternion=[0, 0, 0, 0])
        self.assertEqual(self.send("build", **dict(request, start=bad_pose))["type"], "error")
        huge_quaternion = dict(position=[0, 0, .5], quaternion=[1e308, 0, 0, 1e308])
        self.assertEqual(self.send("build", **dict(request, start=huge_quaternion))["type"], "error")
        built = self.send("build", **request)
        self.assertEqual(built["type"], "built")
        self.assertEqual(len(built["states"]), 6)
        self.assertEqual(self.send("targets", start=description["start"], goal=bad_pose)["type"], "error")
        # Rejected target updates do not poison the prior or current estimate.
        self.check_solution(self.solve(built))
        self.assertEqual(self.send("unknown")["type"], "error")
        self.assertEqual(self.send("describe")["type"], "description")

    def test_unreachable_target_stops_without_claiming_convergence(self):
        description = self.send("describe")
        goal = copy.deepcopy(description["goal"])
        goal["position"] = [2., 0., 1.]
        state = self.send("build", intermediate_states=4, duration=4,
                          start=description["start"], goal=goal)
        for _ in range(80):
            state = self.send("iterate")
            self.assertEqual(state["type"], "iteration")
            if state["done"]:
                break
        self.assertTrue(state["done"])
        self.assertFalse(state["converged"])
        self.assertFalse(state["targets_reached"])
        self.assertIn(state["stop_reason"], ["stalled", "iteration_limit"])
        self.assertGreater(state["endpoint_error"]["goal_position"], .1)


if __name__ == "__main__":
    unittest.main()
