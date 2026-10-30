#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtdynamics::contact_init {

// All residuals are measured in metres. Pose3 supplies the analytic derivative
// in GTSAM's right-retracted [rotation, translation] tangent convention.
class ContactFactor3 : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;
  gtsam::Point3 local_;
 public:
  ContactFactor3(gtsam::Key pose, gtsam::Key point, const gtsam::Point3& local)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), pose, point), local_(local) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& point,
                            gtsam::OptionalMatrixType Hpose = nullptr,
                            gtsam::OptionalMatrixType Hpoint = nullptr) const override {
    if (Hpoint) *Hpoint = -gtsam::Matrix3::Identity();
    return pose.transformFrom(local_, Hpose) - point;
  }
};

class ContactFactor2 : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point2>;
  gtsam::Point3 local_;
  double height_;
 public:
  ContactFactor2(gtsam::Key pose, gtsam::Key point, const gtsam::Point3& local, double height)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), pose, point), local_(local), height_(height) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point2& point,
                            gtsam::OptionalMatrixType Hpose = nullptr,
                            gtsam::OptionalMatrixType Hpoint = nullptr) const override {
    if (Hpoint) { *Hpoint = gtsam::Matrix::Zero(3, 2); (*Hpoint)(0, 0) = -1; (*Hpoint)(1, 1) = -1; }
    return pose.transformFrom(local_, Hpose) - gtsam::Point3(point.x(), point.y(), height_);
  }
};

// The anchor pose is a constant, not an uncertain prior or an optimized key.
class AnchoredContactFactor3 : public gtsam::NoiseModelFactorN<gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3>;
  gtsam::Point3 target_;
 public:
  AnchoredContactFactor3(gtsam::Key point, const gtsam::Point3& target)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), point), target_(target) {}
  gtsam::Vector evaluateError(const gtsam::Point3& point, gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) *H = -gtsam::Matrix3::Identity();
    return target_ - point;
  }
};

class AnchoredContactFactor2 : public gtsam::NoiseModelFactorN<gtsam::Point2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point2>;
  gtsam::Point3 target_;
  double height_;
 public:
  AnchoredContactFactor2(gtsam::Key point, const gtsam::Point3& target, double height)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), point), target_(target), height_(height) {}
  gtsam::Vector evaluateError(const gtsam::Point2& point, gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) { *H = gtsam::Matrix::Zero(3, 2); (*H)(0, 0) = -1; (*H)(1, 1) = -1; }
    return target_ - gtsam::Point3(point.x(), point.y(), height_);
  }
};
}  // namespace gtdynamics::contact_init
