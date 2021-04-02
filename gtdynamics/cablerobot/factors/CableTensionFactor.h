/**
 * @file  CableTensionFactor.h
 * @brief Cable tension factor: relates cable tension, two mounting points, and
 * resultant forces
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtdynamics/cablerobot/utils/cableUtils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

#include <iostream>
#include <string>

namespace gtdynamics {

/** CableTensionFactor is a 3-way nonlinear factor which enforces relation
 * amongst cable tension, end effector pose, and wrench felt by the end effector
 */
class CableTensionFactor
    : public gtsam::NoiseModelFactor3<double, gtsam::Pose3, gtsam::Vector6> {
 private:
  using Pose = gtsam::Pose3;
  using Point = gtsam::Point3;
  using Wrench = gtsam::Vector6;
  using Vector3 = gtsam::Vector3;
  typedef CableTensionFactor This;
  typedef gtsam::NoiseModelFactor3<double, Pose, Wrench> Base;

  Point wPb_, eePem_;

 public:
  /** Cable tension factor
   * @param tension_key -- key for cable tension (scalar)
   * @param eePose_key -- key for end effector pose
   * @param wrench_key -- key for the wrench acting on the end-effector (in the
   end effector's reference frame)
   * @param cost_model -- noise model (6 dimensional)
   * @param wPb -- cable mounting location on the fixed frame, in world coords
   * @param eePem -- cable mounting location on the end effector, in the
   * end-effector frame (wPem = wTee * eePem)
   */
  CableTensionFactor(gtsam::Key tension_key, gtsam::Key eePose_key,
                     gtsam::Key wrench_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const Point &wPb, const Point &eePem)
      : Base(cost_model, tension_key, eePose_key, wrench_key),
        wPb_(wPb),
        eePem_(eePem) {}
  virtual ~CableTensionFactor() {}

 public:
  /** Cable wrench factor
   * @param t cable tension
   * @param wTee end effector pose (in the world frame)
   * @param Fee wrench acting on the end effector (in the end effector frame)
   * @return Vector(6): calculated wrench minus Fee
   */
  gtsam::Vector evaluateError(
      const double &t, const Pose &wTee, const Wrench &Fee,
      boost::optional<gtsam::Matrix &> H_t = boost::none,
      boost::optional<gtsam::Matrix &> H_wTee = boost::none,
      boost::optional<gtsam::Matrix &> H_Fee = boost::none) const override {
    // Jacobians: cable direction
    gtsam::Matrix33 dir_H_wPem;
    gtsam::Matrix36 wPem_H_wTee;
    // Jacobians: force to wrench conversion
    gtsam::Matrix31 wf_H_t;
    gtsam::Matrix33 wf_H_dir;
    gtsam::Matrix33 eef_H_wf;
    gtsam::Matrix33 eef_H_wRee;
    gtsam::Matrix63 H_eef;
    gtsam::Matrix33 eem_H_eef;  // = H_eef.topRows<3>(); TODO(gerry): pointer?

    // cable direction
    Point wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
    Vector3 dir = cablerobot::normalize(wPem - wPb_, H_wTee ? &dir_H_wPem : 0);
    // force->wrench
    Vector3 wf = -t * dir;
    if (H_t) wf_H_t = -dir;
    if (H_wTee) wf_H_dir = -t * gtsam::I_3x3;
    Vector3 eef = wTee.rotation().rotate(wf,  //
                                         H_wTee ? &eef_H_wRee : 0,
                                         (H_t || H_wTee) ? &eef_H_wf : 0);
    Vector3 eem = gtsam::cross(eePem_, eef,    //
                               boost::none,  //
                               (H_t || H_wTee) ? &eem_H_eef : 0);
    Wrench F_expected = (Wrench() << eem, eef).finished();
    if (H_t || H_wTee) H_eef << eem_H_eef, gtsam::I_3x3;

    // error
    Wrench error = (Wrench() << F_expected - Fee).finished();

    if (H_t) *H_t = H_eef * eef_H_wf * wf_H_t;
    if (H_wTee) {
      *(H_wTee) = H_eef * eef_H_wf * wf_H_dir * dir_H_wPem * wPem_H_wTee;
      H_wTee->leftCols<3>() += H_eef * eef_H_wRee;
    }
    if (H_Fee) *H_Fee = -gtsam::I_6x6;

    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "cable tension factor" << std::endl;
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
