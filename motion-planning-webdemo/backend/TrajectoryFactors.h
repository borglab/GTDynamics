#pragma once

#include <gtdynamics/factors/CollocationFactors.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace motion_planning_demo {

// q[k+1] - q[k] = dt/2 (v[k] + v[k+1]). The signed residual is
// immaterial; its four Jacobians must match the same convention.
class JointTrapezoidalFactor
    : public gtsam::NoiseModelFactorN<double, double, double, double> {
  using Base = gtsam::NoiseModelFactorN<double, double, double, double>;
  double dt_;
 public:
  JointTrapezoidalFactor(gtsam::Key q0, gtsam::Key q1, gtsam::Key v0,
                        gtsam::Key v1, double dt,
                        const gtsam::SharedNoiseModel& noise)
      : Base(noise, q0, q1, v0, v1), dt_(dt) {}
  gtsam::Vector evaluateError(const double& q0, const double& q1,
                             const double& v0, const double& v1,
                             gtsam::OptionalMatrixType H0 = nullptr,
                             gtsam::OptionalMatrixType H1 = nullptr,
                             gtsam::OptionalMatrixType H2 = nullptr,
                             gtsam::OptionalMatrixType H3 = nullptr) const override {
    if (H0) *H0 = -gtsam::Matrix1::Identity();
    if (H1) *H1 = gtsam::Matrix1::Identity();
    if (H2) *H2 = -0.5 * dt_ * gtsam::Matrix1::Identity();
    if (H3) *H3 = -0.5 * dt_ * gtsam::Matrix1::Identity();
    return gtsam::Vector1(q1 - q0 - 0.5 * dt_ * (v0 + v1));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<JointTrapezoidalFactor>(*this);
  }
};

// Same body-twist trapezoidal residual as GTDynamics' existing factor, with
// the full Logmap chain rule. The upstream factor uses the zero-residual
// approximation for those Jacobians, which is inaccurate during large drags.
class PoseTrapezoidalFactor
    : public gtdynamics::FixTimeTrapezoidalPoseCollocationFactor {
  double dt_;
 public:
  PoseTrapezoidalFactor(gtsam::Key p0, gtsam::Key p1, gtsam::Key v0,
                       gtsam::Key v1, double dt,
                       const gtsam::SharedNoiseModel& noise)
      : FixTimeTrapezoidalPoseCollocationFactor(p0, p1, v0, v1, dt, noise),
        dt_(dt) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& p0, const gtsam::Pose3& p1,
                             const gtsam::Vector6& v0, const gtsam::Vector6& v1,
                             gtsam::OptionalMatrixType H0 = nullptr,
                             gtsam::OptionalMatrixType H1 = nullptr,
                             gtsam::OptionalMatrixType H2 = nullptr,
                             gtsam::OptionalMatrixType H3 = nullptr) const override {
    gtsam::Matrix6 Dp0, Dstep, DlogPrediction, DlogActual;
    const auto predicted = gtdynamics::predictPose(
        p0, 0.5 * dt_ * (v0 + v1), Dp0, Dstep);
    const auto error = p1.logmap(predicted, DlogActual, DlogPrediction);
    if (H0) *H0 = DlogPrediction * Dp0;
    if (H1) *H1 = DlogActual;
    if (H2) *H2 = DlogPrediction * Dstep * (0.5 * dt_);
    if (H3) *H3 = DlogPrediction * Dstep * (0.5 * dt_);
    return error;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<PoseTrapezoidalFactor>(*this);
  }
};
}  // namespace motion_planning_demo
