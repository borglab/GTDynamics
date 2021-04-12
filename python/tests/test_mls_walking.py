"""Test for multi-level STEAP walking."""

import unittest

from gtsam import Point3, Pose3, Rot3
from gtsam.utils.test_case import GtsamTestCase


class ExecutedTrajectory:
    """A record of the trajectory that was executed by the robot."""

    def final_pose(self):
        """Return the final pose reached by the robot.

        Returns:
            Pose3: Final pose reached by the robot.
        """
        return Pose3()


class MlsWalker:
    """Multi-level STEAP walker."""

    def __init__(self, start: Pose3, goal: Pose3):
        """Initialize with start and goal.

        Args:
            start(Pose3): Starting pose for the robot.
            goal(Pose3): Goal pose.
        """
        self._executed_trajectory = ExecutedTrajectory()

    def executed_trajectory(self):
        """Return saved trajectory."""
        return self._executed_trajectory


class Simulator:
    """Simple simulator."""

    def add(self, walker: MlsWalker):
        """Add a walker to the simulator.

        Args:
            walker (MlsWalker): The walker to add.
        """

    def run(self, steps: int):
        """Run the simulator for a number of steps.

        Args:
            steps(int): Number of steps.
        """


class TestMlsWalking(GtsamTestCase):
    """Test for multi-level STEAP walking"""

    def test_whole_enchilada(self):
        """Test end-to-end walking."""
        simulator = Simulator()
        start = Pose3()
        goal = Pose3(Rot3(), Point3(0, 10, 0))
        mls = MlsWalker(start, goal)
        simulator.add(mls)
        simulator.run(100)
        trajectory = mls.executed_trajectory()
        self.assertIsInstance(trajectory, ExecutedTrajectory)
        final_pose = trajectory.final_pose()
        self.gtsamAssertEquals(final_pose, goal, tol=0.1)


if __name__ == "__main__":
    unittest.main()
