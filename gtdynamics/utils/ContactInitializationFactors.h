#pragma once

/**
 * @file ContactInitializationFactors.h
 * Analytic contact, attitude, and landmark-height factors for sparse seeding.
 *
 * Contact and height residuals use metres; upright residuals are dimensionless
 * differences of unit directions. Pose Jacobians follow GTSAM's right-retracted
 * [rotation, translation] tangent convention. All world coordinates are Z-up.
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtdynamics::contact_init {

/** Contact residual X.transformFrom(local) - landmark for a free Point3.
 *
 * Local geometry is frozen FK in the root frame. The Pose3 maps root to world;
 * the landmark is one stationary contact point shared across a stance.
 * Contact noise has unit standard deviation in each coordinate (one metre).
 */
class ContactFactor3
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;
  gtsam::Point3 local_;

 public:
  /** Associate a root-pose key and a world-landmark key with fixed local FK.
   *
   * The two keys must be distinct; local is a contact-point position in
   * metres.
   */
  ContactFactor3(gtsam::Key pose, gtsam::Key point, const gtsam::Point3& local)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), pose, point),
        local_(local) {}
  /** Return the 3D displacement in metres and optional analytic Jacobians.
   *
   * Hpose has shape 3x6; Hpoint is the 3x3 negative identity.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point3& point,
      gtsam::OptionalMatrixType Hpose = nullptr,
      gtsam::OptionalMatrixType Hpoint = nullptr) const override {
    if (Hpoint) *Hpoint = -gtsam::Matrix3::Identity();
    return pose.transformFrom(local_, Hpose) - point;
  }
};

/** Soft world-up alignment that leaves position and heading unconstrained.
 *
 * The residual is R * unitZ - unitZ, including for upside-down poses. Sigma is
 * a small-angle scale in radians, rather than the variance of a 3D translation.
 */
class GTSAM_EXPORT UprightFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

 public:
  /// Construct an upright prior with a small-angle standard deviation in
  /// radians.
  UprightFactor(gtsam::Key pose, double sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, sigma), pose) {}

  /** Return the world-space body-up direction minus world up.
   *
   * The optional 3x6 Jacobian has zero translation columns. The residual norm
   * is 2 sin(theta / 2) for a tilt angle theta, so it is not a logarithmic
   * angle.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      gtsam::OptionalMatrixType H = nullptr) const override {
    gtsam::Matrix3 rotationJacobian;
    const gtsam::Point3 direction = pose.rotation().rotate(
        gtsam::Point3::UnitZ(), H ? &rotationJacobian : nullptr);
    if (H) {
      H->setZero(3, 6);
      H->leftCols<3>() = rotationJacobian;
    }
    return direction - gtsam::Point3::UnitZ();
  }
};

/** Soft landmark-height prior that leaves horizontal position unconstrained.
 *
 * Adds one scalar (point.z - height) residual per unique Point3 landmark,
 * rather than per observation. The soft model retains all three landmark
 * coordinates.
 */
class GTSAM_EXPORT LandmarkGroundFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3>;
  double height_;

 public:
  /// Construct a prior at the given world ground height, with sigma in metres.
  LandmarkGroundFactor(gtsam::Key point, double height, double sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, sigma), point),
        height_(height) {}

  /** Return landmark Z minus ground height, in metres.
   *
   * The optional Jacobian is the 1x3 row [0, 0, 1]; XY are unconstrained.
   */
  gtsam::Vector evaluateError(
      const gtsam::Point3& point,
      gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) {
      H->setZero(1, 3);
      (*H)(0, 2) = 1;
    }
    return gtsam::Vector1(point.z() - height_);
  }
};

/** Contact residual to a Point2 landmark lifted onto an exact horizontal plane.
 *
 * Only the landmark's XY coordinates are variables; height is a fixed world Z
 * in metres. The 3D contact residual retains unit noise in each coordinate.
 */
class ContactFactor2
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point2>;
  gtsam::Point3 local_;
  double height_;

 public:
  /// Associate fixed local FK with a root pose and an XY landmark at world
  /// height.
  ContactFactor2(gtsam::Key pose, gtsam::Key point, const gtsam::Point3& local,
                 double height)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), pose, point),
        local_(local),
        height_(height) {}
  /** Return X.transformFrom(local) - [point.x, point.y, height].
   *
   * Optional Jacobians have shapes 3x6 for the pose and 3x2 for the XY
   * landmark; the landmark Jacobian's Z row is zero because height is not
   * optimized.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point2& point,
      gtsam::OptionalMatrixType Hpose = nullptr,
      gtsam::OptionalMatrixType Hpoint = nullptr) const override {
    if (Hpoint) {
      *Hpoint = gtsam::Matrix::Zero(3, 2);
      (*Hpoint)(0, 0) = -1;
      (*Hpoint)(1, 1) = -1;
    }
    return pose.transformFrom(local_, Hpose) -
           gtsam::Point3(point.x(), point.y(), height_);
  }
};

/** Contact to a constant world target from an exactly fixed root pose.
 *
 * This is the anchored form of ContactFactor3. The anchor is substituted into
 * the measurement, so it has no optimized pose key or uncertain pose prior.
 */
class AnchoredContactFactor3 : public gtsam::NoiseModelFactorN<gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3>;
  gtsam::Point3 target_;

 public:
  /// Construct a unit-noise contact from a free landmark to a fixed world
  /// point.
  AnchoredContactFactor3(gtsam::Key point, const gtsam::Point3& target)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), point),
        target_(target) {}
  /// Return target - point in metres, with optional 3x3 negative-identity
  /// Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Point3& point,
      gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) *H = -gtsam::Matrix3::Identity();
    return target_ - point;
  }
};

/** Contact to a constant world target with a landmark on an exact plane.
 *
 * This is the anchored form of ContactFactor2: the pose and landmark Z are
 * constants, and only the two horizontal landmark coordinates are optimized.
 */
class AnchoredContactFactor2 : public gtsam::NoiseModelFactorN<gtsam::Point2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point2>;
  gtsam::Point3 target_;
  double height_;

 public:
  /// Construct a unit-noise contact with a fixed target and world plane height.
  AnchoredContactFactor2(gtsam::Key point, const gtsam::Point3& target,
                         double height)
      : Base(gtsam::noiseModel::Isotropic::Sigma(3, 1), point),
        target_(target),
        height_(height) {}
  /** Return target - [point.x, point.y, height] in metres.
   *
   * The optional 3x2 Jacobian is negative identity in XY and zero in Z.
   */
  gtsam::Vector evaluateError(
      const gtsam::Point2& point,
      gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) {
      *H = gtsam::Matrix::Zero(3, 2);
      (*H)(0, 0) = -1;
      (*H)(1, 1) = -1;
    }
    return target_ - gtsam::Point3(point.x(), point.y(), height_);
  }
};
}  // namespace gtdynamics::contact_init
