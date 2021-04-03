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

 public:
  /** Cable factor
   * @param ldot -- cable speed (ldot)
   * @param wTee -- end effector pose
   * @param Vee -- end effector twist
   * @return expected/calculated cable speed minus given ldot
   */
  gtsam::Vector evaluateError(
      const double &ldot, const Pose3 &wTee, const Vector6 &Vee,
      boost::optional<gtsam::Matrix &> H_ldot = boost::none,
      boost::optional<gtsam::Matrix &> H_wTee = boost::none,
      boost::optional<gtsam::Matrix &> H_Vee = boost::none) const override {
    // Jacobians: cable direction
    gtsam::Matrix13 H_dir;
    gtsam::Matrix33 dir_H_wPem;
    gtsam::Matrix36 wPem_H_wTee;
    // Jacobians: _E_nd-effector _M_ounting point velocity (in world coords)
    gtsam::Matrix13 H_wPDOTem;
    gtsam::Matrix33 wPDOTem_H_wRee;
    gtsam::Matrix33 wPDOTem_H_eePDOTem;
    gtsam::Matrix36 eePDOTem_H_Vee;
    gtsam::Matrix33 cross_H_omega;

    // cable direction
    Point3 wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
    Vector3 dir = gtsam::normalize(wPem - wPb_, H_wTee ? &dir_H_wPem : 0);

    // velocity aka pdot
    Vector3 eePDOTem = Vee.tail<3>() + gtsam::cross(Vee.head<3>(), eePem_,
                                                    H_Vee ? &cross_H_omega : 0);
    if (H_Vee) eePDOTem_H_Vee << cross_H_omega, gtsam::I_3x3;
    Vector3 wPDOTem = wTee.rotation().rotate(eePDOTem,  //
                                             H_wTee ? &wPDOTem_H_wRee : 0,
                                             H_Vee ? &wPDOTem_H_eePDOTem : 0);

    // ldot = (cable direction) dot (velocity aka pdot)
    double expected_ldot = gtsam::dot(dir, wPDOTem,         //
                                      H_wTee ? &H_dir : 0,  //
                                      H_Vee ? &H_wPDOTem : 0);

    // jacobians
    if (H_ldot) *H_ldot = gtsam::I_1x1;
    if (H_wTee) {
      *H_wTee = -H_dir * dir_H_wPem * wPem_H_wTee;  //
      H_wTee->leftCols<3>() -= H_wPDOTem * wPDOTem_H_wRee;
    }
    if (H_Vee) *H_Vee = -H_wPDOTem * wPDOTem_H_eePDOTem * eePDOTem_H_Vee;

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
