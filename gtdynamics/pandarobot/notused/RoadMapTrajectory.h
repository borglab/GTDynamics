/**
 * @file  testPandaIKFast.cpp
 * @brief test Roadmap Trajectory optimization
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

namespace gtdynamics {

class RoadMapTrajectory {
 public:
  RoadMapTrajectory();
  static gtsam::Values optimizeForTrajectory(RoadMap, gtsam::Vector7 thetas_start, gtsam::Pose3 bTe_final);
};
}  // namespace gtdynamics