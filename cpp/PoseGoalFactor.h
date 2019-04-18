/**
 * @file  PoseGoalFactor.h
 * @brief end effector pose goal factor
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace manipulator {

/**
 * PoseGoalFactor is a non-linear factor of manipulator end-effector pose goal
 */
class PoseGoalFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  typedef PoseGoalFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

  gtsam::Pose3 goalPose_;
  boost::function<std::vector<gtsam::Pose3>(
      const gtsam::Vector &, boost::optional<std::vector<gtsam::Matrix> &>)>
      forwardKinematics_;

 public:
  /**
   * Construct from joint angle limits
   * Keyword arguments:
      key
      cost_model  -- noise model
      goalPose    -- end effector pose goal
   */
  PoseGoalFactor(
      gtsam::Key key, const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &goal_pose,
      boost::function<std::vector<gtsam::Pose3>(
          const gtsam::Vector &, boost::optional<std::vector<gtsam::Matrix> &>)>
          forward_kinematics)
      : Base(cost_model, key),
        goalPose_(goal_pose),
        forwardKinematics_(forward_kinematics) {}

  virtual ~PoseGoalFactor() {}

  /** error function
      Keyword argument:
          jointCoordinates      -- angles for revolution joint,
                                    distances for prismatic joint
          H                     -- jacobian matrix
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector &joint_coordinates,
      boost::optional<gtsam::Matrix &> H = boost::none) const {
    // forward kinematics
    std::vector<gtsam::Matrix> jacobian;
    auto end_effector_pose =
        forwardKinematics_(joint_coordinates, jacobian).back();
    if (H) {
      *H = jacobian.back();
    }
    return goalPose_.logmap(end_effector_pose);
  }

  // @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "pose goal factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
