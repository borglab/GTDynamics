/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  LinkPoseFactor.h
 * @brief Factor relating a parent link and a child link given joint
 * measurement.
 * @author Varun Agrawal
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "gtdynamics/universal_robot/Joint.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * @brief A 2-way factor to relate the parent and child links of a joint given
 * the joint coordinate as a measurement.
 *
 * @tparam JOINT The type of joint which is being constrained.
 */
template <typename JOINT>
class LinkPoseFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  using This = LinkPoseFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  JointConstSharedPtr joint_;
  typename JOINT::JointCoordinate joint_coordinate_;
  size_t k_;

 public:
  /**
   * @brief Construct a new Link Pose Factor object
   *
   * @param wTp_key Key to the parent link.
   * @param wTc_key Key to the child link.
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  LinkPoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key,
                 const gtsam::noiseModel::Base::shared_ptr& model,
                 const JointConstSharedPtr joint,
                 const typename JOINT::JointCoordinate& joint_coordinate,
                 size_t k)
      : Base(model, wTp_key, wTc_key),
        joint_(joint),
        joint_coordinate_(joint_coordinate),
        k_(k) {}

  /**
   * @brief Convenience constructor
   *
   * @param model The noise model for this factor.
   * @param joint The joint connecting the links.
   * @param joint_coordinate The coordinates of the joint motion.
   * @param k The time index.
   */
  LinkPoseFactor(const gtsam::noiseModel::Base::shared_ptr& model,
                 const JointConstSharedPtr joint,
                 const typename JOINT::JointCoordinate& joint_coordinate,
                 size_t k)
      : Base(model, internal::PoseKey(joint->parent()->id(), k),
             internal::PoseKey(joint->child()->id(), k)),
        joint_(joint),
        joint_coordinate_(joint_coordinate),
        k_(k) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& wTp, const gtsam::Pose3& wTc,
      boost::optional<gtsam::Matrix&> H_wTp = boost::none,
      boost::optional<gtsam::Matrix&> H_wTc = boost::none) const override {
    gtsam::Values joint_angles;
    InsertJointAngle(&joint_angles, joint_->id(), k_, joint_coordinate_);

    gtsam::Matrix H_q;
    gtsam::Matrix6 H;
    gtsam::Pose3 wTc_hat =
        joint_->poseOf(joint_->child(), wTp, joint_angles, k_, H_wTp, H_q);

    gtsam::Vector6 error = wTc.logmap(wTc_hat, H_wTc, H_wTp ? &H : 0);
    if (H_wTp) {
      *H_wTp = H * (*H_wTp);
    }
    return error;
  }

  /// print contents
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "LinkPoseFactor(" << keyFormatter(key1()) << ","
              << keyFormatter(key2()) << ")\n";
    gtsam::traits<typename JOINT::JointCoordinate>::Print(joint_coordinate_,
                                                          "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }
};
}  // namespace gtdynamics