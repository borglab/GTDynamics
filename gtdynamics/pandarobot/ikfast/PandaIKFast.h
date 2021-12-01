/**
 * @file  PandaIKFast.h
 * @brief Wrapper for the generated IKFast source files for the panda robot.
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

#define IKFAST_HAS_LIBRARY  // Build IKFast with API functions
#define IKFAST_NO_MAIN      // Don't include main() from IKFast
#define IKFAST_NAMESPACE panda_internal

#define IK_VERSION 61
#include "ikfast61_panda.cpp"

//----------------------------------------------------------------------------//

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>

using gtsam::Vector7;
using IKFAST_NAMESPACE::IkReal;

namespace gtdynamics {

// Wrapper of IKFast functions for panda robot.
class PandaIKFast {
 private:
  using Pose3 = gtsam::Pose3;
  using Point3 = gtsam::Point3;
  using Rot3 = gtsam::Rot3;
  using Vector7 = gtsam::Vector7;
  using Matrix3 = gtsam::Matrix3;

 public:
  PandaIKFast();

  // The robot's number of joints, for the panda it's 7
  static constexpr size_t kNumJoints = 7;

  static Pose3 forward(const Vector7& joint_values);

  static std::vector<Vector7> inverse(const Pose3& bRe, double theta7);
};

PandaIKFast::PandaIKFast() {}

/**
 * @brief Forward Kinematics on Panda robot using IKFast.
 *
 * @param joint_values -- Vector7 of joint angles' values
 * @return gtsam::Pose3 -- Resulting 3D Pose with the guven joint angles
 */
gtsam::Pose3 PandaIKFast::forward(const Vector7& joint_values) {
  // Arrays where solution for orientation and position will be stored
  IkReal orientation[9], position[3];
  IKFAST_NAMESPACE::ComputeFk(joint_values.data(), position, orientation);

  // Transform array solutions to gtsam types.
  // default eigen matrix storage is column major (used in Map), while the
  // ikfast plugin is rowmajor, a transpose is necessary
  Rot3 bRe(Matrix3::Map(&orientation[0]).transpose());
  Point3 bte(Point3::Map(&position[0]));

  return Pose3(bRe, bte);
}

/**
 * @brief Inverse Kinematics on Panda robot using IKFast. Singularity solutions
 * are not returned.
 *
 * @param bTe -- the desired end-effector pose wrt the base frame
 * @param theta7 -- the value for the 7th joint angle, working as the parameter
 * of the 1D solution
 * @return std::vector<Vector7> -- std vector containing the different solutions
 * for joint angles' values
 */
std::vector<Vector7> PandaIKFast::inverse(const Pose3& bTe, double theta7) {
  // default eigen matrix storage is column major, while the ikfast uses a
  // rowmajor one, rotation matrix needs to be transposed before getting the
  // data pointer
  const Matrix3 bRe = bTe.rotation().matrix().transpose();

  // Ikfast class where solutions are stored
  IkSolutionList<IkReal> solutions;

  // The inputs (except "solutions") have to be arrays
  bool success = IKFAST_NAMESPACE::ComputeIk(bTe.translation().data(),
                                             bRe.data(), &theta7, solutions);

  if (!success) {
    fprintf(stderr, "Error: (inverse PandaIKFast) failed to get ik solution\n");
    return std::vector<Vector7>();
  }

  unsigned int num_sols = (int)solutions.GetNumSolutions();

  std::vector<Vector7> joint_values(num_sols, Vector7());
  for (size_t i = 0, j = 0; i < num_sols; ++i) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);

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
