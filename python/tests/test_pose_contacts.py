# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_pose_contacts.py
# @brief Verify pose-contact bindings and the shared 200 ms sampling default.
# @author GTDynamics contributors
"""Native pose-contact bindings, including defaults and exact buffered slice IDs."""

import unittest
import gtsam
import gtdynamics as gtd
import numpy as np


class PoseContactTests(unittest.TestCase):
    def test_native_pose_initializer_and_ik(self) -> None:
        link = gtd.Link(1, "root", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"root": link}, {})
        contact = gtd.PoseOnLink(
            link, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -1.0]))
        )
        spec = gtd.PoseContactConstraintSpec([contact])
        trajectory = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(0, 20, spec)]), 1)
        options = gtd.pose_contact_init.Options()
        result = gtd.pose_contact_init.Result()
        anchor = gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 1.0]))
        initializer = gtd.PoseContactInitializer(
            robot=robot, options=options, contactBuffer=1
        )
        values = initializer.run(
            rootLinkName="root",
            initialPose=anchor,
            measuredJoints=gtsam.Values(),
            trajectory=trajectory,
            dt=0.1,
            diagnostics=result,
        )
        # A one-sample buffer leaves [1, 19]; the 200 ms default at 10 Hz
        # retains every other source slice, including single-support poses.
        expected_slices = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19]
        self.assertEqual(options.maxSliceInterval, 0.2)
        self.assertEqual([s.k for s in result.slices()], expected_slices)
        np.testing.assert_allclose(result.times, np.asarray(expected_slices) * 0.1)
        self.assertEqual(len(result.contactPoses), 1)
        self.assertEqual(len(result.contactFrames), 1)
        self.assertTrue(result.contactFrames[0].equals(contact))
        self.assertEqual(
            [o.poseIndex for o in result.observations],
            list(range(len(expected_slices))),
        )
        for observation in result.observations:
            self.assertEqual(observation.contactIndex, 0)
            predicted = result.poses[observation.poseIndex] * observation.rootTcontact
            self.assertTrue(predicted.equals(result.contactPoses[0], 1e-8))
        self.assertEqual(len(result.history), result.iterations + 1)
        self.assertTrue(contact.predict(values, 10).equals(gtsam.Pose3(), 1e-8))
        # Construction copies tuning, while each run gets its own gauge and clip.
        options.maxIterations = 0
        shorter = gtd.Trajectory(gtd.WalkCycle([gtd.Phase(0, 5, spec)]), 1)
        shifted_anchor = gtsam.Pose3(
            anchor.rotation(), anchor.translation() + np.array([0.3, 0.2, 0.0])
        )
        shifted = initializer.run(
            rootLinkName="root",
            initialPose=shifted_anchor,
            measuredJoints=gtsam.Values(),
            trajectory=shorter,
            dt=0.05,
        )
        self.assertTrue(shifted.atPose3(gtd.PoseKey(1, 0)).equals(shifted_anchor, 1e-8))
        self.assertTrue(values.atPose3(gtd.PoseKey(1, 0)).equals(anchor, 1e-8))
        options.maxIterations = 100
        selected_slices = [gtd.Slice(3), gtd.Slice(17)]
        options.interpolateOutput = False
        options.initializeUpright = True
        sparse = gtd.PoseContactInitializer(robot=robot, options=options).run(
            rootLinkName="root",
            initialPose=anchor,
            measuredJoints=gtsam.Values(),
            trajectory=trajectory,
            dt=0.1,
            diagnostics=result,
            selectedSlices=selected_slices,
        )
        self.assertEqual([s.k for s in result.slices()], [3, 17])
        np.testing.assert_allclose(result.times, [0.3, 1.7])
        for k in range(21):
            self.assertEqual(sparse.exists(gtd.PoseKey(1, k)), k in (3, 17))
        options.addUprightFactors = True
        options.uprightSigma = float(np.deg2rad(15.0))
        options.addNormalFactors = False
        options.groundHeight = 0.0
        options.groundSigma = 0.1
        gtd.PoseContactInitializer(robot=robot, options=options).run(
            rootLinkName="root",
            initialPose=anchor,
            measuredJoints=gtsam.Values(),
            trajectory=trajectory,
            dt=0.1,
            diagnostics=result,
            selectedSlices=selected_slices,
        )
        factors = [result.graph.at(i) for i in range(result.graph.size())]
        self.assertEqual(
            sum(isinstance(f, gtd.contact_init.UprightFactor) for f in factors), 1
        )
        self.assertEqual(
            sum(isinstance(f, gtd.ContactNormalFactor) for f in factors), 0
        )
        self.assertTrue(result.poses[0].equals(anchor, 1e-12))
        self.assertFalse(result.stalled)
        self.assertTrue(result.converged)
        self.assertEqual(len(spec.contactPoints()), 0)
        self.assertEqual(len(spec.contactPoses()), 1)
        seen = []
        ik = gtd.PoseIKInitializer(robot=robot, maxIterations=2).run(
            rootLinkName="root",
            rootGuesses=[anchor] * 21,
            measuredJoints=gtsam.Values(),
            trajectory=trajectory,
            iterationCallback=lambda k, i, e, v: seen.append((k, i)),
        )
        self.assertEqual(len({k for k, i in seen}), 21)
        self.assertTrue(ik.atPose3(gtd.PoseKey(1, 20)).equals(anchor, 1e-8))


if __name__ == "__main__":
    unittest.main()
