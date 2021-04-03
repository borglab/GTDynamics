/**
 * @file  CableLenFactor.h
 * @brief Cable length factor: relates cable length and two mounting points
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

#include <iostream>
#include <string>

namespace gtdynamics {

/** CableLenFactor is a 2-way nonlinear factor which enforces relation for
 * end effector pose and cable length
 */
class CableLenFactor : public gtsam::NoiseModelFactor2<
                           double, gtsam::Pose3> {
 private:
  using Pose = gtsam::Pose3;
  using Point = gtsam::Point3;
  typedef CableLenFactor This;
  typedef gtsam::NoiseModelFactor2<double, Pose> Base;

  Point wPb_, eePem_;

 public:
  /** Cable factor
   * @param l_key -- key for cable length
   * @param wTee -- key for end effector pose
   * @param cost_model -- noise model (1 dimensional)
   * @param wPb -- cable mounting location on the fixed frame, in world coords
   * @param eePem -- cable mounting location on the end effector, in the
   * end-effector frame (wPem = wTee * eePem)
   */
  CableLenFactor(gtsam::Key l_key, gtsam::Key wTee_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const Point &wPb, const Point &eePem)
      : Base(cost_model, l_key, wTee_key), wPb_(wPb), eePem_(eePem) {}
  virtual ~CableLenFactor() {}

 public:
  /** Cable factor
   * @param l -- cable length
   * @param wTee -- end effector pose
   * @return length predicted minus cable length given
   */
  gtsam::Vector evaluateError(
      const double &l, const gtsam::Pose3 &wTee,
      boost::optional<gtsam::Matrix &> H_l = boost::none,
      boost::optional<gtsam::Matrix &> H_wTee = boost::none) const override {
    gtsam::Matrix36 wPem_H_wTee;
    gtsam::Matrix13 H_wPem;
    auto wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
    double expected_l = gtsam::distance3(wPem, wPb_, H_wTee ? &H_wPem : 0);
    if (H_l) *H_l = gtsam::Vector1(-1);
    if (H_wTee) *H_wTee = H_wPem * wPem_H_wTee;
    return gtsam::Vector1(expected_l - l);
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
    std::cout << s << "cable length factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
