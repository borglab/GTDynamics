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
#include "gtdynamics/utils/ContactPoint.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * ForwardKinematicsFactor is a two-way nonlinear factor which computes the
 * relative CoM pose between the specified links.
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
   * @param bTl1_key        Key for pose of start link in the kinematic chain.
   * @param bTl2_key        Key for pose of end link in the kinematic chain.
   * @param robot           The Robot model to perform forward kinematics.
   * @param start_link_name The name of the root link for FK.
   * @param end_link_name   The end link name whose pose to compute via FK.
   * @param joint_angles    gtsam::Values with joint angles for relevant joints.
   * @param model           The noise model for this factor.
   * @param k               The discrete time index.
   */
  ForwardKinematicsFactor(gtsam::Key bTl1_key, gtsam::Key bTl2_key,
                          const Robot &robot,
                          const std::string &start_link_name,
                          const std::string &end_link_name,
                          const gtsam::Values &joint_angles,
                          const gtsam::SharedNoiseModel &model, size_t k = 0)
      : Base(bTl1_key, bTl2_key,
             computeRelativePose(robot, start_link_name, end_link_name,
                                 joint_angles, k),
             model) {}

  /**
   * Construct which conveniently generates keys from the link names.
   *
   * @param robot           The Robot model to perform forward kinematics.
   * @param start_link_name The name of the root link for FK.
   * @param end_link_name   The end link name whose pose to compute via FK.
   * @param joint_angles    gtsam::Values with joint angles for relevant joints.
   * @param model           The noise model for this factor.
   * @param k               The discretized time index.
   */
  ForwardKinematicsFactor(const Robot &robot,
                          const std::string &start_link_name,
                          const std::string &end_link_name,
                          const gtsam::Values &joint_angles,
                          const gtsam::SharedNoiseModel &model, size_t k = 0)
      : Base(internal::PoseKey(robot.link(start_link_name)->id(), k),
             internal::PoseKey(robot.link(end_link_name)->id(), k),
             computeRelativePose(robot, start_link_name, end_link_name,
                                 joint_angles, k),
             model) {}

  virtual ~ForwardKinematicsFactor() {}

  /**
   * @brief Function to compute the relative pose between start and end link via
   * forward kinematics using the joint angles.
   *
   * @param robot Robot model on which to perform forward kinematics.
   * @param start_link_name String for the start link in the kinematic chain.
   * @param end_link_name String for the end link in the kinematic chain.
   * @param joint_angles Values with joint angles in radians.
   * @param k The time index at which to compute the forward kinematics.
   * @return relative pose of the end link in the start link's frame.
   */
  gtsam::Pose3 computeRelativePose(const Robot &robot,
                                   const std::string &start_link_name,
                                   const std::string &end_link_name,
                                   const gtsam::Values &joint_angles,
                                   size_t k) const {
    auto start_link = robot.link(start_link_name);
    auto end_link = robot.link(end_link_name);

    gtsam::Values fk =
        robot.forwardKinematics(joint_angles, k, start_link_name);

    gtsam::Pose3 bTl1 = Pose(fk, start_link->id(), k);
    gtsam::Pose3 bTl2 = Pose(fk, end_link->id(), k);
    return bTl1.between(bTl2);
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ForwardKinematicsFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ")\n";
    gtsam::traits<gtsam::Pose3>::Print(measured(), "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }
};

}  // namespace gtdynamics
