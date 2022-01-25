/**
 * @file  testPandaIKFast.cpp
 * @brief test Roadmap Trajectory optimization
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include "RoadMapTrajectory.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

namespace gtdynamics {
using gtsam::Values;
using gtsam::Vector7;
using gtsam::Pose3;

// Not sure if this is the way to go, putting factorgraphs here already?
static Values RoadMap::optimizeForTrajectory(Vector7 thetas_start, Pose3 bTe_final){
    // build roadmap

    // find index of closest node to start

    // find possible indexes of closest nodes to bTe_final

    // shortest path on roadmap (waypoints)

    // optimize with factorgraphs?
}

}  // namespace gtdynamics