# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Exercise public trajectory, initializer and CM-Opt bindings with GTSAM types."""

import unittest

import gtsam
import gtdynamics as gtd
import numpy as np


class KinematicsWrappingTests(unittest.TestCase):
    def setUp(self):
        self.anchor = gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 1.0]))
        root = gtd.Link(1, "root", 1.0, np.eye(3), self.anchor, self.anchor)
        tip = gtd.Link(2, "tip", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        joint = gtd.RevoluteJoint(
            0, "hinge", self.anchor, root, tip, np.array([0.0, 1.0, 0.0])
        )
        root.addJoint(joint)
        tip.addJoint(joint)
        self.robot = gtd.Robot({"root": root, "tip": tip}, {"hinge": joint})
        self.points = gtd.PointOnLinks(
            [
                gtd.PointOnLink(tip, np.array([x, y, 0.0]))
                for x, y in ((-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1))
            ]
        )
        self.measured = gtsam.Values()
        for k in range(4):
            self.measured.insert(gtd.JointAngleKey(0, k), 0.0)

    def test_contact_initializer_defaults_and_output_pointer(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        trajectory = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(0, 3, spec)]), 1)
        init = gtd.Initializer()
        args = (self.robot, "root", self.anchor, self.measured, trajectory)
        options = gtd.contact_init.Options()
        options.recordHistory = False
        result = gtd.contact_init.Result()
        explicit = init.InitializeSolutionContacts(*args, 0.01, options, result)
        default = init.InitializeSolutionContacts(*args, 0.01)
        self.assertIsInstance(explicit, gtsam.Values)
        self.assertTrue(explicit.equals(default, 1e-8))
        self.assertGreater(len(result.poses), 0)
        self.assertEqual(len(result.landmarks), 4)
        self.assertLess(result.residualRms, 1e-8)
        self.assertGreaterEqual(result.timings.totalMs, 0.0)
        for k in range(4):
            self.assertTrue(
                explicit.atPose3(gtd.PoseKey(1, k)).equals(self.anchor, 1e-8)
            )
            self.assertLess(
                gtd.Kinematics()
                .constraints(gtd.Slice(k), self.robot)
                .violationNorm(explicit),
                1e-8,
            )

    def test_inverse_kinematics_with_contact_patch(self):
        # Four points on one link exercise the shared contact-wrench key seed.
        result = gtd.Initializer().InitializeSolutionInverseKinematics(
            self.robot,
            "root",
            self.anchor,
            [self.anchor],
            [0.03],
            0.01,
            0.0,
            self.points,
        )
        for k in range(4):
            self.assertTrue(result.atPose3(gtd.PoseKey(1, k)).equals(self.anchor, 1e-7))
            self.assertAlmostEqual(
                result.atDouble(gtd.JointAngleKey(0, k)), 0.0, places=7
            )

    def test_phase_contact_height_builders(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        phase = gtd.Phase(5, 7, spec)
        self.assertIsInstance(phase.constraintSpec(), gtd.FootContactConstraintSpec)
        self.assertTrue(
            phase.constraintSpec().contactPoints()[0].equals(self.points[0], 0.0)
        )
        kinematics = gtd.Kinematics()
        gravity = np.array([0.0, 0.0, -9.81])
        graph = kinematics.contactHeightObjectives(phase, gravity, 0.2)
        explicit = gtsam.NonlinearFactorGraph()
        for k in range(5, 8):
            explicit.push_back(
                kinematics.contactHeightObjectives(gtd.Slice(k), self.points, gravity, 0.2)
            )
        with self.assertRaises(TypeError):
            kinematics.contactHeightObjectives(phase, self.points, gravity, 0.2)
        self.assertEqual(graph.size(), 12)
        self.assertEqual(
            list(graph.keyVector()), [gtd.PoseKey(2, k) for k in range(5, 8)]
        )
        values = gtsam.Values()
        for k in range(5, 8):
            values.insert(
                gtd.PoseKey(2, k), gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.2]))
            )
        self.assertAlmostEqual(graph.error(values), 0.0)
        self.assertAlmostEqual(explicit.error(values), 0.0)
        singleton = kinematics.contactHeightObjectives(gtd.Phase(9, 9, spec), gravity)
        self.assertEqual(singleton.size(), 4)
        self.assertEqual(list(singleton.keyVector()), [gtd.PoseKey(2, 9)])
        empty_spec = gtd.FootContactConstraintSpec(gtd.PointOnLinks([]))
        empty = kinematics.contactHeightObjectives(gtd.Phase(5, 7, empty_spec), gravity)
        self.assertEqual(empty.size(), 0)
        with self.assertRaisesRegex(ValueError, "FootContactConstraintSpec"):
            kinematics.contactHeightObjectives(gtd.Phase(5, 7, None), gravity)

    def test_trajectory_contact_height_builder(self):
        def phase(first, last, points):
            return gtd.Phase(
                first, last, gtd.FootContactConstraintSpec(gtd.PointOnLinks(points))
            )

        trajectory = gtd.Trajectory(
            gtd.WalkCycle([
                phase(5, 6, [self.points[0], self.points[1]]),
                phase(6, 7, [self.points[2], self.points[1]]),
                phase(8, 9, []),
                phase(10, 10, [self.points[0]]),
            ]), 1
        )
        kinematics = gtd.Kinematics()
        gravity = np.array([0.0, 0.0, -9.81])
        graph = kinematics.contactHeightObjectives(trajectory, gravity, 0.2)
        # Shared point at slice 6 contributes once; distinct points on the same
        # link still contribute separately. Flight contributes no factors.
        self.assertEqual(graph.size(), 8)
        self.assertEqual(
            list(graph.keyVector()), [gtd.PoseKey(2, k) for k in (5, 6, 7, 10)]
        )
        values = gtsam.Values()
        for k in (5, 6, 7, 10):
            values.insert(
                gtd.PoseKey(2, k), gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.2]))
            )
        self.assertAlmostEqual(graph.error(values), 0.0)
        self.assertEqual(
            kinematics.contactHeightObjectives(gtd.Trajectory(), gravity).size(), 0
        )
        invalid = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(5, 7, None)]), 1)
        with self.assertRaisesRegex(ValueError, "FootContactConstraintSpec"):
            kinematics.contactHeightObjectives(invalid, gravity)

    def test_context_graphs_and_cmopt_with_python_basis_callback(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        phase = gtd.Phase(0, 3, spec)
        self.assertIsInstance(phase, gtd.Interval)
        self.assertEqual((phase.k_start, phase.k_end), (0, 3))
        trajectory = gtd.Trajectory(gtd.WalkCycle([phase]), 1)
        self.assertEqual(len(trajectory.phases()), 1)
        kinematics = gtd.Kinematics()
        equalities = kinematics.constraints(trajectory, self.robot)
        self.assertEqual(equalities.size(), 4)
        self.assertEqual(kinematics.constraints(phase, self.robot).size(), 4)
        costs = kinematics.jointAngleObjectives(trajectory, self.robot, self.measured)
        costs.push_back(
            kinematics.contactHeightObjectives(trajectory, np.array([0.0, 0.0, -9.81]))
        )
        initial = gtsam.Values()
        for k in range(4):
            costs.add(
                gtsam.PriorFactorPose3(
                    gtd.PoseKey(1, k),
                    self.anchor,
                    gtsam.noiseModel.Isotropic.Sigma(6, 0.1),
                )
            )
            known = gtsam.Values()
            known.insert(gtd.JointAngleKey(0, k), 0.1)
            known.insert(gtd.PoseKey(1, k), self.anchor)
            known.insert(gtd.TwistKey(1, k), np.zeros(6))
            initial.insert(self.robot.forwardKinematics(known, k, "root"))
        initial = initial.extract(
            sorted(set(costs.keyVector()) | set(equalities.keyVector()))
        )
        calls = []

        def basis(keys):
            calls.append(len(keys))
            return [
                key
                for key in keys
                if gtd.DynamicsSymbol(key).label() == "q"
                or key == gtd.PoseKey(1, gtd.DynamicsSymbol(key).time())
            ]

        parameters = gtd.ManifoldOptimizerParameters()
        basis_params = gtd.TangentSpaceBasisParams(True, True)
        parameters.constraintManifoldParams.basisCreator = gtd.EliminationBasisCreator(
            basis, basis_params
        )
        retraction = gtd.RetractionParams()
        retraction.feasibleThreshold = 1e-20
        retraction.lmParams.setAbsoluteErrorTol(1e-18)
        parameters.constraintManifoldParams.retractorCreator = (
            gtd.BasisRetractorCreator(basis, retraction)
        )
        cm = gtd.NonlinearManifoldOptimizer(
            parameters, gtsam.LevenbergMarquardtParams()
        )
        problem = cm.initializeManifoldOptimizationProblem(costs, equalities, initial)
        self.assertEqual(len(problem.components), 4)
        self.assertEqual(problem.values.dim(), 28)
        optimizer = cm.constructNonlinearOptimizer(problem)
        solution = cm.baseValues(problem, optimizer.optimize())
        self.assertTrue(calls)
        self.assertIsInstance(solution, gtsam.Values)
        self.assertLess(costs.error(solution), 1e-8)
        self.assertLess(equalities.violationNorm(solution), 1e-8)
        self.assertLess(equalities.penaltyGraph().error(solution), 1e-8)
        for k in range(4):
            self.assertAlmostEqual(
                solution.atDouble(gtd.JointAngleKey(0, k)), 0.0, places=6
            )


if __name__ == "__main__":
    unittest.main()
