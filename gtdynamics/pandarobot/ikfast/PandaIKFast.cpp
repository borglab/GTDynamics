/**
 * @file  PandaIKFast.h
 * @brief Wrapper for the generated IKFast source files for the panda robot.
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include "PandaIKFast.h"

//----------------------------------------------------------------------------//

#define IKFAST_HAS_LIBRARY  // Build IKFast with API functions
#define IKFAST_NAMESPACE panda_internal

#define IK_VERSION 61
#include <gtdynamics/pandarobot/ikfast/ikfast.h>

//----------------------------------------------------------------------------//

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>

namespace gtdynamics {

using gtsam::Matrix3;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector7;

PandaIKFast::PandaIKFast() {}

Pose3 PandaIKFast::forward(const Vector7& joint_values) {
  // Arrays where solution for orientation and position will be stored
  panda_internal::IkReal orientation[9], position[3];
  panda_internal::ComputeFk(joint_values.data(), position, orientation);

  // Transform array solutions to gtsam types.
  // default eigen matrix storage is column major (used in Map), while the
  // ikfast plugin is rowmajor, a transpose is necessary
  Rot3 bRe(Matrix3::Map(&orientation[0]).transpose());
  Point3 bte(Point3::Map(&position[0]));

  return Pose3(bRe, bte);
}

std::vector<Vector7> PandaIKFast::inverse(const Pose3& bTe, double theta7) {
  // default eigen matrix storage is column major, while the ikfast uses a
  // rowmajor one, rotation matrix needs to be transposed before getting the
  // data pointer
  const Matrix3 bRe = bTe.rotation().matrix().transpose();

  // Ikfast class where solutions are stored
  ikfast::IkSolutionList<panda_internal::IkReal> solutions;

  // The inputs (except "solutions") have to be arrays
  bool success = panda_internal::ComputeIk(bTe.translation().data(), bRe.data(),
                                           &theta7, solutions);

  if (!success) {
    fprintf(stderr, "Error: (inverse PandaIKFast) failed to get ik solution\n");
    return std::vector<Vector7>();
  }

  unsigned int num_sols = (unsigned int)solutions.GetNumSolutions();

  std::vector<Vector7> joint_values(num_sols, Vector7());
  for (size_t i = 0, j = 0; i < num_sols; ++i) {
    const ikfast::IkSolutionBase<panda_internal::IkReal>& sol =
        solutions.GetSolution(i);

    if (sol.GetFree().size() == 0) {
      // Just save solution if there is no extra degree of freedom, i.e.,if the
      // resulting joint configurations are not in a singularity
      sol.GetSolution(joint_values[j++].data(), NULL);
    } else {
      // If it is a singularity, do not save it and erase one of the solution
      // "containers", so there are no empty solutions returned
      joint_values.pop_back();
    }
  }

  return joint_values;
}

}  // namespace gtdynamics
