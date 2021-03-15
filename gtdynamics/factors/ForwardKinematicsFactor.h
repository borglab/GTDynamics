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
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"
#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {

/**
 * ForwardKinematicsFactor is a two-way nonlinear factor which computes the pose
 * of the specified link with respect to the robot's base link,
 * and compares it to the estimate.
 */
class ForwardKinematicsFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  using This = ForwardKinematicsFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  Robot robot_;
  std::string base_link_name_, end_link_name_;
  JointValues joint_angles_;

 public:
  /**
   * Create single factor relating specified link's pose (COM) with estimate.
   *
   * Please use the joint based constructor above if possible.
   *
   * @param bTp_key Key for end link's CoM pose in robot base frame.
   * @param robot The Robot model to perform forward kinematics.
   * @param base_link_name The name of the robot's base link.
   * @param end_link_name The name of the end link whose pose we wish to
   * compute.
   * @param joint_angles Map of joint names to joint angles used in forward
   * kinematics.
   * @param cost_model The noise model for this factor.
   */
  ForwardKinematicsFactor(gtsam::Key bTp_key, const Robot &robot,
                          const std::string &base_link_name,
                          const std::string &end_link_name,
                          const JointValues &joint_angles,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, bTp_key),
        robot_(robot),
        base_link_name_(base_link_name),
        end_link_name_(end_link_name),
        joint_angles_(joint_angles) {}

  virtual ~ForwardKinematicsFactor() {}

  /**
   * Evaluate Forward Kinematics errors
   * @param bTp previous (parent) link CoM pose
   * @param wTc this (child) link CoM pose
   * @param q joint angle
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &bTp,
      boost::optional<gtsam::Matrix &> H_bTp = boost::none) const override {
    JointValues joint_velocities;
    for (auto &&kv : joint_angles_) {
      joint_velocities[kv.first] = 0;
    }
    FKResults result = robot_.forwardKinematics(
        joint_angles_, joint_velocities, base_link_name_,
        robot_.link(base_link_name_)->lTcom());
    gtsam::Pose3 measured_ = result.first[end_link_name_];
    return gtsam::traits<gtsam::Pose3>::Local(measured_, bTp, boost::none,
                                              H_bTp);
    ;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ForwardKinematicsFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
