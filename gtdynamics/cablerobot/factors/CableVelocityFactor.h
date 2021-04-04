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
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

#include <iostream>
#include <string>

namespace gtdynamics {

/** CableVelocityFactor is a 3-way nonlinear factor which enforces relation amongst
 * cable velocity, end-effector pose, and end-effector twist
 */
class CableVelocityFactor
    : public gtsam::NoiseModelFactor3<double, gtsam::Pose3, gtsam::Vector6> {
 private:
  using Point3 = gtsam::Point3;
  using Vector3 = gtsam::Vector3;
  using Pose3 = gtsam::Pose3;
  using Vector6 = gtsam::Vector6;
  using This = CableVelocityFactor;
  using Base = gtsam::NoiseModelFactor3<double, Pose3, Vector6>;

  Point3 wPb_, eePem_;

 public:
  /** Cable factor
   * @param ldot_key -- key for cable speed
   * @param wTee -- key for end effector pose
   * @param Vee -- key for end effector twist
   * @param cost_model -- noise model (1 dimensional)
   * @param wPb -- cable mounting location on the fixed frame, in world coords
   * @param eePem -- cable mounting location on the end effector, in the
   * end-effector frame (wPem = wTee * eePem)
   */
  CableVelocityFactor(gtsam::Key ldot_key, gtsam::Key wTee_key,
                      gtsam::Key Vee_key,
                      const gtsam::noiseModel::Base::shared_ptr &cost_model,
                      const Point3 &wPb, const Point3 &eePem)
      : Base(cost_model, ldot_key, wTee_key, Vee_key),
        wPb_(wPb),
        eePem_(eePem) {}
  virtual ~CableVelocityFactor() {}

 private:
  /** Computes the cable speed that will result from some twist
   * @param wTee the pose of the end effector
   * @param Vee the twist of the end effector in the end effector's frame
   * @return Vector6: calculated wrench
   */
  double computeLdot(
      const Pose3 &wTee, const Vector6 &Vee,
      boost::optional<gtsam::Matrix &> H_wTee = boost::none,
      boost::optional<gtsam::Matrix &> H_Vee = boost::none) const;

 public:
  /** Cable factor
   * @param ldot -- cable speed (ldot)
   * @param wTee -- end effector pose
   * @param Vee -- end effector twist
   * @return given ldot minus expected/calculated cable speed
   */
  gtsam::Vector evaluateError(
      const double &ldot, const Pose3 &wTee, const Vector6 &Vee,
      boost::optional<gtsam::Matrix &> H_ldot = boost::none,
      boost::optional<gtsam::Matrix &> H_wTee = boost::none,
      boost::optional<gtsam::Matrix &> H_Vee = boost::none) const override {
    double expected_ldot = computeLdot(wTee, Vee, H_wTee, H_Vee);
    if (H_ldot) *H_ldot = gtsam::I_1x1;
    if (H_wTee) *H_wTee = -(*H_wTee);
    if (H_Vee) *H_Vee = -(*H_Vee);
    return gtsam::Vector1(ldot - expected_ldot);
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
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
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
