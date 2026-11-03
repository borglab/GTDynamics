/* GTDynamics Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PoseTranslationPrior.h>
namespace g1_example {
/** Choose heading without constraining the gravity-observable tilt. */
class YawGaugeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  double yaw_;

 public:
  YawGaugeFactor(gtsam::Key key, double yaw)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Unit::Create(1), key),
        yaw_(yaw) {}
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix13 rotationJacobian;
    const double yaw = pose.rotation().yaw(rotationJacobian);
    if (H) {
      *H = gtsam::Matrix16::Zero();
      H->leftCols<3>() = rotationJacobian;
    }
    const double difference = yaw - yaw_;
    return gtsam::Vector1(
        std::atan2(std::sin(difference), std::cos(difference)));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<YawGaugeFactor>(*this);
  }
};
inline gtsam::NonlinearFactorGraph gaugeFactors(gtsam::Key key,
                                                const gtsam::Pose3& anchor) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(
      key, anchor.translation(), gtsam::noiseModel::Unit::Create(3));
  graph.emplace_shared<YawGaugeFactor>(key, anchor.rotation().yaw());
  return graph;
}
}  // namespace g1_example
