/**
 * @file  CableVelocityFactor.h
 * @brief Cable velocity factor: relates cable velocity (or acceleration), two
 * mounting points, and resultant mounting point velocities
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <iostream>
#include <string>

namespace gtdynamics {

/** CableVelocityFactor is a 3-way nonlinear factor which enforces relation
 * amongst cable velocity, end-effector pose, and end-effector twist
 */
class CableVelocityFactor
    : public gtsam::NoiseModelFactorN<double, gtsam::Pose3, gtsam::Vector6> {
 private:
  using Point3 = gtsam::Point3;
  using Vector3 = gtsam::Vector3;
  using Pose3 = gtsam::Pose3;
  using Vector6 = gtsam::Vector6;
  using This = CableVelocityFactor;
  using Base = gtsam::NoiseModelFactorN<double, Pose3, Vector6>;

  Point3 wPa_, xPb_;

 public:
  /** Cable factor
   * @param ldot_key -- key for cable speed
   * @param wTx -- key for end effector pose
   * @param Vx -- key for end effector twist
   * @param cost_model -- noise model (1 dimensional)
   * @param wPa -- cable mounting location on the fixed frame, in world coords
   * @param xPb -- cable mounting location on the end effector, in the
   * end-effector frame (wPb = wTx * xPb)
   */
  CableVelocityFactor(gtsam::Key ldot_key, gtsam::Key wTx_key,
                      gtsam::Key Vx_key,
                      const gtsam::noiseModel::Base::shared_ptr &cost_model,
                      const Point3 &wPa, const Point3 &xPb)
      : Base(cost_model, ldot_key, wTx_key, Vx_key), wPa_(wPa), xPb_(xPb) {}
  virtual ~CableVelocityFactor() {}

 private:
  /** Computes the cable speed that will result from some twist
   * @param wTx the pose of the end effector
   * @param Vx the twist of the end effector in the end effector's frame
   * @return Vector6: calculated wrench
   */
  double computeLdot(const Pose3 &wTx, const Vector6 &Vx,
                     gtsam::OptionalMatrixType H_wTx = nullptr,
                     gtsam::OptionalMatrixType H_Vx = nullptr) const;

 public:
  /** Cable factor
   * @param ldot -- cable speed (ldot)
   * @param wTx -- end effector pose
   * @param Vx -- end effector twist
   * @return given ldot minus expected/calculated cable speed
   */
  gtsam::Vector evaluateError(
      const double &ldot, const Pose3 &wTx, const Vector6 &Vx,
      gtsam::OptionalMatrixType H_ldot = nullptr,
      gtsam::OptionalMatrixType H_wTx = nullptr,
      gtsam::OptionalMatrixType H_Vx = nullptr) const override {
    double expected_ldot = computeLdot(wTx, Vx, H_wTx, H_Vx);
    if (H_ldot) *H_ldot = gtsam::I_1x1;
    if (H_wTx) *H_wTx = -(*H_wTx);
    if (H_Vx) *H_Vx = -(*H_Vx);
    return gtsam::Vector1(ldot - expected_ldot);
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 GTDKeyFormatter) const override {
    std::cout << s << "cable vel factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics
