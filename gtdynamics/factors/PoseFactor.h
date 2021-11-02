/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PoseFactor.h
 * @brief Forward kinematics factor.
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/assign/list_of.hpp>
#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

using boost::assign::cref_list_of;

/**
 * PoseFactor is a three-way nonlinear factor between a joint's parent link
 * pose, child link pose, and the joint angle relating the two poses.
 *
 * Given the joint model, this factor optimizes for the underlying joint axis
 * and the corresponding poses of the parent and child links.
 */
class PoseFactor : public gtsam::ExpressionFactor<gtsam::Vector6> {
 private:
  using This = PoseFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector6>;

  int t_;
  JointConstSharedPtr joint_;

 public:
  /**
   * Create single factor relating this link's pose (COM) with previous one.
   *
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses.
   * @param time The timestep at which this factor is defined.
   */
  PoseFactor(const gtsam::SharedNoiseModel &cost_model,
             const JointConstSharedPtr &joint, int time)
      : Base(cost_model, gtsam::Vector6::Zero(), joint->poseConstraint(time)),
        t_(time),
        joint_(joint) {}

  /**
   * Create single factor relating this link's pose (COM) with previous one.
   *
   * Please use the joint based constructor above if possible.
   *
   * @param wTp_key Key for parent link's CoM pose in world frame.
   * @param wTc_key Key for child link's CoM pose in world frame.
   * @param q_key Key for joint value.
   * @param cost_model The noise model for this factor.
   * @param joint The joint connecting the two poses
   */
  PoseFactor(DynamicsSymbol wTp_key, DynamicsSymbol wTc_key,
             DynamicsSymbol q_key,
             const gtsam::noiseModel::Base::shared_ptr &cost_model,
             JointConstSharedPtr joint)
      : Base(cost_model, gtsam::Vector6::Zero(), joint->poseConstraint(wTp_key.time())),
        t_(wTp_key.time()),
        joint_(joint) {}

  virtual ~PoseFactor() {}

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? s : s + " ") << "Pose Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
