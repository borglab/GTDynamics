"""Small deterministic checks; no G1 assets, GPU, or timing assertions needed."""

import unittest

import numpy as np
import gtsam
import gtdynamics as gtd

from kinematic_walk import build_problem, contact_slices, solve_cm
from mujoco_io import KinematicRecording, import_robot, contact_phases


class ContactPhaseTests(unittest.TestCase):
    def test_boundary_samples_belong_to_exactly_one_phase(self):
        labels = [
            dict(foot="left", start=1, end=3),
            dict(foot="right", start=3, end=5),
            dict(foot="left", start=5, end=6),
        ]
        phases = contact_phases(labels, 8)
        self.assertEqual(
            [k for a, b, _ in phases for k in range(a, b + 1)], list(range(8))
        )
        self.assertEqual(
            phases,
            [
                (0, 0, ()),
                (1, 2, ("left",)),
                (3, 3, ("left", "right")),
                (4, 4, ("right",)),
                (5, 5, ("left", "right")),
                (6, 6, ("left",)),
                (7, 7, ()),
            ],
        )

    def test_persistent_contact_spans_other_foot_transition(self):
        phases = contact_phases(
            [dict(foot="left", start=0, end=4), dict(foot="right", start=2, end=3)], 5
        )
        self.assertEqual(
            phases, [(0, 1, ("left",)), (2, 3, ("left", "right")), (4, 4, ("left",))]
        )

    def test_continuity_crosses_phase_boundaries_but_not_flight(self):
        left = gtd.Link(1, "left", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        right = gtd.Link(2, "right", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        l = gtd.PointOnLink(left, np.zeros(3))
        r = gtd.PointOnLink(right, np.zeros(3))
        phases = [
            gtd.Phase(a, b, gtd.FootContactConstraintSpec(gtd.PointOnLinks(points)))
            for a, b, points in (
                (0, 1, [l]),
                (2, 3, [r, l]),
                (4, 5, [l]),
                (6, 6, []),
                (7, 7, [l]),
            )
        ]
        trajectory = gtd.Trajectory(gtd.WalkCycle(phases), 1)
        slices = list(contact_slices(trajectory))
        self.assertEqual([k for k, _, _ in slices], list(range(8)))
        self.assertEqual(
            [sorted(p.link.id() for p in continuing) for _, _, continuing in slices],
            [[], [1], [1], [1, 2], [1], [1], [], []],
        )
        graph = gtd.Kinematics().contactEqualityFactors(
            trajectory, gtsam.noiseModel.Isotropic.Sigma(3, 0.005)
        )
        self.assertEqual(graph.size(), 6)
        self.assertTrue(
            all(
                isinstance(graph.at(i), gtd.ContactEqualityFactor)
                for i in range(graph.size())
            )
        )


class NativeTrajectoryTests(unittest.TestCase):
    def test_one_trajectory_two_initial_states(self):
        def body(identifier, parent, name, position, joint_id, free):
            return dict(
                id=identifier,
                parent=parent,
                name=name,
                position=position,
                quaternion=[1.0, 0.0, 0.0, 0.0],
                com_position=[0.0, 0.0, 0.0],
                com_quaternion=[1.0, 0.0, 0.0, 0.0],
                mass=1.0,
                inertia=[1.0, 1.0, 1.0],
                joint=dict(
                    id=joint_id,
                    name="hinge",
                    free=free,
                    position=[0.0, 0.0, 0.0],
                    axis=[0.0, 1.0, 0.0],
                ),
            )

        robot = import_robot(
            [
                body(1, 0, "root", [0.0, 0.0, 0.0], 0, True),
                body(2, 1, "tip", [0.0, 0.0, -1.0], 1, False),
            ]
        )
        measured = gtsam.Values()
        for k in range(4):
            measured.insert(gtd.JointAngleKey(1, k), 0.1 * k)
        spec = gtd.FootContactConstraintSpec(gtd.PointOnLinks([]))
        phases = [gtd.Phase(0, 1, spec), gtd.Phase(2, 3, spec)]
        walk = KinematicRecording(
            robot=robot,
            root=robot.link("root"),
            measured=measured,
            anchor=gtsam.Pose3(),
            trajectory=gtd.Trajectory(gtd.WalkCycle(phases), 1),
            count=4,
            dt=0.01,
            speed=0.0,
            reference_root_positions=np.zeros((4, 3)),
            provenance={},
        )
        costs, constraints, keys = build_problem(walk)
        self.assertEqual(constraints.size(), 4)
        final_costs = []
        for offset in (0.0, 0.5):
            initial = gtsam.Values()
            for k in range(4):
                q = gtsam.Values()
                q.insert(gtd.JointAngleKey(1, k), offset)
                q.insert(gtd.PoseKey(1, k), gtsam.Pose3())
                q.insert(gtd.TwistKey(1, k), np.zeros(6))
                initial.insert(robot.forwardKinematics(q, k, "root"))
            solution, info = solve_cm(costs, constraints, initial.extract(keys), 1, 30)
            history = []
            observed, observed_info = solve_cm(
                costs,
                constraints,
                initial.extract(keys),
                1,
                30,
                lambda iteration, state: history.append(
                    (iteration, gtsam.Values(state))
                ),
            )
            self.assertTrue(observed.equals(solution, 1e-9))
            self.assertEqual(observed_info, info)
            self.assertEqual(
                [i for i, _ in history], list(range(info["iterations"] + 1))
            )
            self.assertTrue(history[-1][1].equals(solution, 1e-9))
            self.assertEqual(info["components"], 4)
            self.assertEqual(info["reduced_dimension"], 28)
            self.assertLess(constraints.violationNorm(solution), 1e-8)
            for k in range(4):
                self.assertAlmostEqual(
                    solution.atDouble(gtd.JointAngleKey(1, k)), 0.1 * k, places=6
                )
            final_costs.append(costs.error(solution))
        self.assertLess(max(final_costs), 1e-10)


if __name__ == "__main__":
    unittest.main()
