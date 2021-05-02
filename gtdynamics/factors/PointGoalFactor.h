/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PointGoalFactor.h
 * @brief Link point goal factor.
 * @author Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * PointGoalFactor is a unary factor enforcing that a point on a link
 * reaches a desired goal point.
 */
class PointGoalFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  using This = PointGoalFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  // Point, expressed in link CoM, where this factor is enforced.
  gtsam::Point3 point_com_;
  // Goal point in the spatial frame.
  gtsam::Point3 goal_point_;

 public:
  /**
   * Constructor from goal point.
   * @param pose_key key for COM pose of the link
   * @param cost_model noise model
   * @param point_com point on link, in COM coordinate frame
   * @param goal_point end effector goal point, in world coordinates
   */
  PointGoalFactor(gtsam::Key pose_key,
                  const gtsam::noiseModel::Base::shared_ptr &cost_model,
                  const gtsam::Point3 &point_com,
                  const gtsam::Point3 &goal_point)
      : Base(cost_model, pose_key),
        point_com_(point_com),
        goal_point_(goal_point) {}

  virtual ~PointGoalFactor() {}

  /// Return goal point.
  const gtsam::Point3 &goalPoint() const { return goal_point_; }

  /**
   * Error function
   * @param wTcom -- The link pose.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTcom,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override;

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override;

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};

/**
 * Construct many PointGoalFactors from goal trajectory.
 *
 * This function willl take a key to the first pose and then increment the key
 * by 1, for each time-step in the goal trajectory. If you need more
 * sophisticated behavior then this function is not it ;-).
 *
 * @param first_key key for the COM pose.
 * @param cost_model noise model
 * @param point_com point on link, in COM coordinate frame
 * @param goal_trajectory end effector goal trajectory, in world coordinates
 */
gtsam::NonlinearFactorGraph PointGoalFactors(
    gtsam::Key first_key, const gtsam::noiseModel::Base::shared_ptr &cost_model,
    const gtsam::Point3 &point_com,
    const std::vector<gtsam::Point3> &goal_trajectory);

}  // namespace gtdynamics
