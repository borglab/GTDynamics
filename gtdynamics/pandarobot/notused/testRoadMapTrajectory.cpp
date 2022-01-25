/**
 * @file  testPandaIKFast.cpp
 * @brief test Roadmap Trajectory optimization
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/pandarobot/roadmap/RoadMapTrajectory.h>
#include <gtdynamics/pandarobot/roadmap/RoadMap.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;

TEST(RoadMapTrajectory, optimizeForTrajectory) {
  Vector7 thetas_start = (Vector7() << 0, 0, 0, 0, 0, 0, 0).finished();
  Rot3 bRe_final(1, 0, 0, 0, 1, 0, 0, 0, 1);
  Point3 bte_final = (Vector7() << 0.5, 0.5, 0.5).finished();
  Pose3 bTe_final(bRe_final, bte_final);

  Roadmap rm_test();
  Values trajectory = RoadMapTrajectory::optimizeForTrajectory(
      rm_test
      thetas_start,
      bTe_final);  // add some component for time discretization :)
  EXPECT(assert_equal(1000, trajectory.size()));

  // need to add expects for the trajectory itself
}