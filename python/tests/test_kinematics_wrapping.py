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
    def test_runtime_equality_type_and_point_hierarchy(self):
        key = gtsam.symbol("x", 0)
        values = gtsam.Values()
        values.insert(key, 1.0)
        manifold = gtd.EConstraintManifold(gtd.NonlinearEqualityConstraints(), values)
        self.assertIsInstance(manifold, gtd.IEConstraintManifold)
        self.assertIsInstance(manifold, gtd.ConstraintManifold)
        point = manifold.point(values)
        self.assertIsInstance(point, gtd.ECMPoint)
        self.assertIsInstance(point, gtd.IECMPoint)
        self.assertIsInstance(point, gtd.CMPoint)
        moved = point.retract(np.array([0.25]))
        self.assertIsInstance(moved, gtd.ECMPoint)
        self.assertAlmostEqual(moved.values().atDouble(key), 1.25)
        self.assertAlmostEqual(point.values().atDouble(key), 1.0)
        np.testing.assert_allclose(point.localCoordinates(moved), [0.25])
        self.assertTrue(point.equals(point.retract(np.zeros(1))))
        conditioned = manifold.condition(values)
        self.assertIsInstance(conditioned, gtd.EConstraintManifold)
        self.assertEqual(conditioned.dim(), 0)
        self.assertEqual(conditioned.point(gtsam.Values()).dim(), 0)

    def test_point_values_are_an_independent_python_snapshot(self):
        key = gtsam.symbol("x", 0)
        values = gtsam.Values()
        values.insert(key, 1.0)
        manifold = gtd.EConstraintManifold(gtd.NonlinearEqualityConstraints(), values)
        point = manifold.point(values)
        moved = point.retract(np.array([0.25]))

        # Mutating the Python snapshot must not invalidate the point's geometry.
        snapshot = point.values()
        snapshot.update(key, 10.0)
        self.assertAlmostEqual(snapshot.atDouble(key), 10.0)
        self.assertAlmostEqual(point.values().atDouble(key), 1.0)
        np.testing.assert_allclose(point.localCoordinates(moved), [0.25])
        self.assertTrue(moved.equals(point.retract(np.array([0.25]))))

    def test_concrete_point_values_interoperate_with_gtsam(self):
        ambient_key = gtsam.symbol("x", 0)
        values = gtsam.Values()
        values.insert(ambient_key, 1.0)
        manifold = gtd.EConstraintManifold(gtd.NonlinearEqualityConstraints(), values)
        equality_point = manifold.point(values)
        general_point = gtd.IEConstraintManifold.point(manifold, values)
        self.assertIsInstance(general_point, gtd.IECMPoint)
        self.assertNotIsInstance(general_point, gtd.ECMPoint)

        equality_key, general_key = gtsam.symbol("c", 0), gtsam.symbol("c", 1)
        points = gtsam.Values()
        points.insert(equality_key, equality_point)
        points.insert(general_key, general_point)
        self.assertIsInstance(points.atECMPoint(equality_key), gtd.ECMPoint)
        self.assertIsInstance(points.atIECMPoint(general_key), gtd.IECMPoint)
        # Value extraction uses the exact concrete GTSAM value type.
        with self.assertRaises(RuntimeError):
            points.atIECMPoint(equality_key)
        with self.assertRaises(RuntimeError):
            points.atECMPoint(general_key)

        delta = gtsam.VectorValues()
        delta.insert(equality_key, np.array([0.25]))
        delta.insert(general_key, np.array([0.5]))
        moved = points.retract(delta)
        self.assertAlmostEqual(
            moved.atECMPoint(equality_key).values().atDouble(ambient_key), 1.25
        )
        self.assertAlmostEqual(
            moved.atIECMPoint(general_key).values().atDouble(ambient_key), 1.5
        )
        self.assertAlmostEqual(
            points.atECMPoint(equality_key).values().atDouble(ambient_key), 1.0
        )

    def test_galilean_imu_factors(self):
        spec = gtd.FootContactConstraintSpec(gtd.PointOnLinks([]))
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 1, spec), gtd.Phase(1, 2, spec)]), 1
        )
        params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        pim = gtsam.PreintegratedImuMeasurementsG(params, gtsam.imuBias.ConstantBias())
        pim.integrateMeasurement(np.array([0.0, 0.0, 9.81]), np.zeros(3), 0.2)
        bias_key = gtsam.symbol("b", 0)
        graph = gtd.Kinematics().imuFactors(trajectory, 1, [pim, pim], bias_key)
        self.assertEqual(graph.size(), 2)
        self.assertIsInstance(graph.at(0), gtsam.GalileanImuFactor)
        values = gtsam.Values()
        values.insert(bias_key, gtsam.imuBias.ConstantBias())
        for k in range(3):
            values.insert(gtd.PoseKey(1, k), gtsam.Pose3())
            values.insert(gtd.LinearVelocityKey(1, k), np.zeros(3))
        self.assertAlmostEqual(graph.error(values), 0.0)
        with self.assertRaises(ValueError):
            gtd.Kinematics().imuFactors(trajectory, 1, [pim], bias_key)

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

    def test_contact_equality_factors(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 1, spec), gtd.Phase(2, 3, spec)]), 1
        )
        kinematics = gtd.Kinematics()
        graph = kinematics.contactEqualityFactors(
            trajectory, gtsam.noiseModel.Isotropic.Sigma(3, 0.005)
        )
        self.assertEqual(graph.size(), 12)
        values = gtsam.Values()
        for k in range(4):
            values.insert(
                gtd.PoseKey(2, k),
                gtsam.Pose3(gtsam.Rot3(), np.array([0.005 * k, 0.0, 0.0])),
            )
        self.assertAlmostEqual(graph.error(values), 6.0)
        nonground = gtd.FootContactConstraintSpec(
            gtd.PointOnLinks(
                [gtd.PointOnLink(p.link, p.point, False) for p in self.points]
            )
        )
        mixed = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 1, nonground), gtd.Phase(2, 3, spec)]), 1
        )
        ground_only = kinematics.contactEqualityFactors(
            mixed, gtsam.noiseModel.Isotropic.Sigma(3, 0.005)
        )
        self.assertEqual(ground_only.size(), 4)
        with self.assertRaises(ValueError):
            kinematics.contactEqualityFactors(
                trajectory, gtsam.noiseModel.Isotropic.Sigma(6, 0.005)
            )

    def test_contact_initializer_defaults_and_output_pointer(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        trajectory = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(0, 3, spec)]), 1)

        options = gtd.contact_init.Options()
        options.recordHistory = False
        result = gtd.contact_init.Result()
        explicit = gtd.ContactTrajectoryInitializer(
            robot=self.robot, options=options
        ).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
            diagnostics=result,
        )
        default = gtd.ContactTrajectoryInitializer(robot=self.robot).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
        )
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

    def test_initializer_iteration_observers(self):

        trajectory = gtd.Trajectory(
            gtd.WalkCycle(
                [gtd.Phase(0, 3, gtd.FootContactConstraintSpec(self.points))]
            ),
            1,
        )
        options = gtd.contact_init.Options()
        options.recordHistory = True
        result = gtd.contact_init.Result()
        gtd.ContactTrajectoryInitializer(robot=self.robot, options=options).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
            diagnostics=result,
        )
        self.assertEqual(len(result.history), result.iterations + 1)
        self.assertEqual(result.history[0].iteration, 0)
        self.assertEqual([m.slice.k for m in result.graph.measurements], [0, 3])
        self.assertTrue(result.history[-1].poses[-1].equals(result.poses[-1], 1e-10))
        self.assertAlmostEqual(result.history[-1].objective, result.objectiveFinal)

        history = {}

        def observe(sample, iteration, error, values):
            history.setdefault(sample, []).append(
                (iteration, error, gtsam.Values(values))
            )

        observed = gtd.InverseKinematicsInitializer(
            robot=self.robot, gaussianNoise=0.0
        ).run(
            linkName="root",
            initialPose=self.anchor,
            targetPoses=[self.anchor],
            times=[0.03],
            dt=0.01,
            contactPoints=self.points,
            iterationCallback=observe,
        )
        plain = gtd.InverseKinematicsInitializer(
            robot=self.robot, gaussianNoise=0.0
        ).run(
            linkName="root",
            initialPose=self.anchor,
            targetPoses=[self.anchor],
            times=[0.03],
            dt=0.01,
            contactPoints=self.points,
        )
        self.assertTrue(observed.equals(plain, 1e-10))
        self.assertEqual(sorted(history), list(range(4)))
        for sample, snapshots in history.items():
            self.assertEqual([s[0] for s in snapshots], list(range(len(snapshots))))
            for before, after in zip(snapshots, snapshots[1:]):
                self.assertLessEqual(after[1], before[1] + 1e-10)
            key = gtd.PoseKey(1, sample)
            self.assertTrue(
                snapshots[-1][2].atPose3(key).equals(observed.atPose3(key), 1e-10)
            )

    def test_contact_buffer_keyword_and_default(self):
        spec = gtd.FootContactConstraintSpec(self.points)
        trajectory = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(0, 3, spec)]), 1)

        original, trimmed = gtd.contact_init.Result(), gtd.contact_init.Result()
        default = gtd.ContactTrajectoryInitializer(robot=self.robot).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
            diagnostics=original,
        )
        zero = gtd.ContactTrajectoryInitializer(robot=self.robot, contactBuffer=0).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
        )
        self.assertTrue(default.equals(zero, 1e-10))
        buffered = gtd.ContactTrajectoryInitializer(
            robot=self.robot, contactBuffer=1
        ).run(
            rootLinkName="root",
            initialPose=self.anchor,
            measuredJoints=self.measured,
            trajectory=trajectory,
            dt=0.01,
            diagnostics=trimmed,
        )
        self.assertEqual([m.slice.k for m in original.graph.measurements], [0, 3])
        self.assertEqual([m.slice.k for m in trimmed.graph.measurements], [1, 2])
        for k in range(4):
            self.assertTrue(
                buffered.atPose3(gtd.PoseKey(1, k)).equals(self.anchor, 1e-8)
            )
        with self.assertRaisesRegex(ValueError, "fewer than two slices"):
            gtd.ContactTrajectoryInitializer(robot=self.robot, contactBuffer=2).run(
                rootLinkName="root",
                initialPose=self.anchor,
                measuredJoints=self.measured,
                trajectory=trajectory,
                dt=0.01,
            )

    def test_inverse_kinematics_with_contact_patch(self):
        # Four points on one link exercise the shared contact-wrench key seed.
        result = gtd.InverseKinematicsInitializer(
            robot=self.robot, gaussianNoise=0.0
        ).run(
            linkName="root",
            initialPose=self.anchor,
            targetPoses=[self.anchor],
            times=[0.03],
            dt=0.01,
            contactPoints=self.points,
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
                kinematics.contactHeightObjectives(
                    gtd.Slice(k), self.points, gravity, 0.2
                )
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

    def test_ground_contact_selection(self):
        ground = gtd.PointOnLink(self.robot.link("root"), np.zeros(3))
        wall = gtd.PointOnLink(self.robot.link("tip"), np.zeros(3), False)
        self.assertTrue(ground.isGround)
        self.assertFalse(wall.isGround)
        same_point = gtd.PointOnLink(wall.link, wall.point, True)
        self.assertFalse(wall.equals(same_point))
        same_point.isGround = False
        self.assertTrue(wall.equals(same_point))
        points = gtd.PointOnLinks([ground, wall])
        spec = gtd.FootContactConstraintSpec(points)
        phase = gtd.Phase(5, 7, spec)
        self.assertEqual(len(phase.constraintSpec().contactPoints()), 2)
        selected = phase.constraintSpec().groundContactPoints()
        self.assertEqual(len(selected), 1)
        self.assertTrue(selected[0].equals(ground))
        gravity = np.array([0.0, 0.0, -9.81])
        kinematics = gtd.Kinematics()
        graph = kinematics.contactHeightObjectives(phase, gravity)
        self.assertEqual(graph.size(), 3)
        self.assertEqual(
            list(graph.keyVector()), [gtd.PoseKey(1, k) for k in (5, 6, 7)]
        )
        self.assertEqual(
            kinematics.contactHeightObjectives(gtd.Slice(5), points, gravity).size(), 1
        )
        ground.isGround = False
        spec = gtd.FootContactConstraintSpec(gtd.PointOnLinks([ground, wall]))
        self.assertEqual(len(spec.contactPoints()), 2)
        self.assertEqual(len(spec.groundContactPoints()), 0)
        self.assertEqual(
            kinematics.contactHeightObjectives(gtd.Phase(5, 7, spec), gravity).size(), 0
        )

    def test_trajectory_contact_height_builder(self):
        def phase(first, last, points):
            return gtd.Phase(
                first, last, gtd.FootContactConstraintSpec(gtd.PointOnLinks(points))
            )

        trajectory = gtd.Trajectory(
            gtd.WalkCycle(
                [
                    phase(5, 6, [self.points[0], self.points[1]]),
                    phase(6, 7, [self.points[2], self.points[1]]),
                    phase(8, 9, []),
                    phase(10, 10, [self.points[0]]),
                ]
            ),
            1,
        )
        kinematics = gtd.Kinematics()
        gravity = np.array([0.0, 0.0, -9.81])
        graph = kinematics.contactHeightObjectives(trajectory, gravity, 0.2)
        # Each phase contributes independently, including its endpoints.
        self.assertEqual(graph.size(), 9)
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
        parameters.constraintManifoldParams.equalityBasisCreator = gtd.EliminationBasisCreator(
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
        self.assertEqual(len(problem.types), 4)
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
