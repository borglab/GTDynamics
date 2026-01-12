/**
 * @file  CableLengthFactor.h
 * @brief Cable length factor: relates cable length and two mounting points
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

/** CableLengthFactor is a 2-way nonlinear factor which enforces relation for
 * end effector pose and cable length
 */
class CableLengthFactor
    : public gtsam::NoiseModelFactorN<double, gtsam::Pose3> {
 private:
  using Pose3 = gtsam::Pose3;
  using Point3 = gtsam::Point3;
  using This = CableLengthFactor;
  using Base = gtsam::NoiseModelFactorN<double, Pose3>;

  Point3 wPa_, xPb_;

 public:
  /** Cable factor
   * @param l_key -- key for cable length
   * @param wTx -- key for end effector pose
   * @param cost_model -- noise model (1 dimensional)
   * @param wPa -- cable mounting location on the fixed frame, in world coords
   * @param xPb -- cable mounting location on the end effector, in the
   * end-effector frame (wPb = wTx * xPb)
   */
  CableLengthFactor(gtsam::Key l_key, gtsam::Key wTx_key,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    const Point3 &wPa, const Point3 &xPb)
      : Base(cost_model, l_key, wTx_key), wPa_(wPa), xPb_(xPb) {}
  virtual ~CableLengthFactor() {}

 public:
  /** Cable factor
   * @param l -- cable length
   * @param wTx -- end effector pose
   * @return cable length given minus length predicted
   */
  gtsam::Vector evaluateError(
      const double &l, const gtsam::Pose3 &wTx,
      gtsam::OptionalMatrixType H_l = nullptr,
      gtsam::OptionalMatrixType H_wTx = nullptr) const override {
    gtsam::Matrix36 wPb_H_wTx;
    gtsam::Matrix13 H_wPb;
    auto wPb = wTx.transformFrom(xPb_, H_wTx ? &wPb_H_wTx : 0);
    double expected_l = gtsam::distance3(wPb, wPa_, H_wTx ? &H_wPb : 0);
    if (H_l) *H_l = gtsam::I_1x1;
    if (H_wTx) *H_wTx = -H_wPb * wPb_H_wTx;
    return gtsam::Vector1(l - expected_l);
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
    std::cout << s << "cable length factor" << std::endl;
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
