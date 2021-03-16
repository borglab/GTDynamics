/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ForwardKinematicsFactor.h
 * @brief General forward kinematics factor.
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {

/**
 * ForwardKinematicsFactor is a two-way nonlinear factor which computes the
 * relatvie CoM pose between the specified links.
 *
 * This factor assumes a joint noise model on the entire kinematic chain, rather
 * than individual noise parameters for each joint. The pose of each link is
 * taken in the robot base/model frame.
 */
class ForwardKinematicsFactor : public gtsam::BetweenFactor<gtsam::Pose3> {
 private:
  using This = ForwardKinematicsFactor;
  using Base = gtsam::BetweenFactor<gtsam::Pose3>;

 public:
  /**
   * Construct the factor by computing the end link estimate via forward
   * kinematics.
   *
   * @param bTl1_key Key for CoM pose of start link in the kinematic chain.
   * @param bTl2_key Key for CoM pose of end link in the kinematic chain.
   * @param robot The Robot model to perform forward kinematics.
   * @param start_link_name The name of the robot's base link.
   * @param end_link_name The name of the end link whose pose we wish to
   * compute.
   * @param joint_angles Map of joint names to joint angles used in forward
   * kinematics.
   * @param cost_model The noise model for this factor.
   */
  ForwardKinematicsFactor(gtsam::Key bTl1_key, gtsam::Key bTl2_key,
                          const Robot &robot,
                          const std::string &start_link_name,
                          const std::string &end_link_name,
                          const JointValues &joint_angles,
                          const gtsam::SharedNoiseModel &model)
      : Base(bTl1_key, bTl2_key,
             forwardKinematics(robot, joint_angles, start_link_name,
                               end_link_name),
             model) {}

  virtual ~ForwardKinematicsFactor() {}

  /**
   * @brief Function to compute the relative pose between start and end link via
   * forward kinematics using the joint angles.
   *
   * @param robot Robot model on which to perform forward kinematics.
   * @param joint_angles Joint angles in radians.
   * @param start_link_name String for the start link in the kinematic chain.
   * @param end_link_name String for the end link in the kinematic chain.
   * @return gtsam::Pose3
   */
  gtsam::Pose3 forwardKinematics(const Robot &robot,
                                 const JointValues &joint_angles,
                                 const std::string &start_link_name,
                                 const std::string &end_link_name) {
    JointValues joint_velocities;
    for (auto &&kv : joint_angles) {
      joint_velocities[kv.first] = 0;
    }
    FKResults result =
        robot.forwardKinematics(joint_angles, joint_velocities, start_link_name,
                                robot.link(start_link_name)->lTcom());
    gtsam::Pose3 bTl1 = result.first[start_link_name];
    gtsam::Pose3 bTl2 = result.first[end_link_name];
    return bTl1.between(bTl2);
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ForwardKinematicsFactor" << std::endl;
    Base::print("", keyFormatter);
  }
};

}  // namespace gtdynamics
