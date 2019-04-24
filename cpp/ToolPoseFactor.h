/**
 * @file  ToolPoseFactor.h
 * @brief end effector pose factor
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

/** ToolPoseFactor is a one-way nonlinear factor which enforces the tool pose*/
class ToolPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  typedef ToolPoseFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;
  gtsam::Pose3 tTn_;
  gtsam::Pose3 tool_pose_;

 public:
  /**
   * create a single factor enforing the tool pose
   * Keyword arguments:
      tTn       -- last link's COM frame expressed in tool frame
      toolPose  -- end effector pose goal
   */
  ToolPoseFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const gtsam::Pose3 &tTn, const gtsam::Pose3 &tool_pose)
      : Base(cost_model, pose_key), tTn_(tTn), tool_pose_(tool_pose) {}

  virtual ~ToolPoseFactor() {}

  /** evaluate link pose errors
      Keyword argument:
          pose         -- last link pose
          H_pose       -- jacobian matrix w.r.t. last link pose
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const {
    auto pose_predict = tool_pose_ * tTn_;
    gtsam::Vector6 error = pose.logmap(pose_predict);
    if (H_pose) {
      *H_pose = -gtsam::I_6x6;
    }

    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "tool pose factor" << std::endl;
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
