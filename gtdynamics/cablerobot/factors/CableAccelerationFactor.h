/**
 * @file  CableAccelerationFactor.h
 * @brief Cable acceleration factor: relates cable length acceleration with end
 * effector pose/twist/twist acceleration.  The cable mounting locations on the
 * stationary frame and on the end-effector are passed as parameters.
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

/** CableAccelerationFactor is a 4-way nonlinear factor which enforces relation
 * amongst cable acceleration, end-effector pose, end-effector twist, and
 * end-effector twist acceleration
 */
class CableAccelerationFactor
    : public gtsam::NoiseModelFactor4<double, gtsam::Pose3, gtsam::Vector6,
                                      gtsam::Vector6> {
 private:
  using Point3 = gtsam::Point3;
  using Vector3 = gtsam::Vector3;
  using Pose3 = gtsam::Pose3;
  using Vector6 = gtsam::Vector6;
  using This = CableAccelerationFactor;
  using Base = gtsam::NoiseModelFactor4<double, Pose3, Vector6, Vector6>;

  Point3 wPa_, xPb_;

 public:
  /** Cable factor
   * @param lddot_key -- key for cable speed
   * @param wTx -- key for end effector pose
   * @param Vx -- key for end effector twist
   * @param VAx -- key for end effector twist acceleration
   * @param cost_model -- noise model (1 dimensional)
   * @param wPa -- cable mounting location on the fixed frame, in world coords
   * @param xPb -- cable mounting location on the end effector, in the
   * end-effector frame (wPb = wTx * xPb)
   */
  CableAccelerationFactor(gtsam::Key lddot_key, gtsam::Key wTx_key,
                          gtsam::Key Vx_key, gtsam::Key VAx_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const Point3 &wPa, const Point3 &xPb)
      : Base(cost_model, lddot_key, wTx_key, Vx_key, VAx_key),
        wPa_(wPa),
        xPb_(xPb) {}
  virtual ~CableAccelerationFactor() {}

 private:
  /** Computes the cable acceleration that will result from some
   * twist/twistaccel
   * @param wTx the pose of the end effector
   * @param Vx the twist of the end effector in the end effector's frame
   * @param VAx the twist of the end effector in the end effector's frame
   * @return double: calculated length acceleration change
   */
  double computeLddot(
      const Pose3 &wTx, const Vector6 &Vx, const Vector6 &VAx,
      boost::optional<gtsam::Matrix &> H_wTx = boost::none,
      boost::optional<gtsam::Matrix &> H_Vx = boost::none,
      boost::optional<gtsam::Matrix &> H_VAx = boost::none) const;

 public:
  /** Cable acceleration factor
   * @param lddot -- cable speed (lddot)
   * @param wTx -- end effector pose
   * @param Vx -- end effector twist
   * @param VAx -- end effector twist
   * @return given lddot minus expected/calculated cable speed
   */
  gtsam::Vector evaluateError(
      const double &lddot, const Pose3 &wTx, const Vector6 &Vx,
      const Vector6 &VAx, boost::optional<gtsam::Matrix &> H_lddot = boost::none,
      boost::optional<gtsam::Matrix &> H_wTx = boost::none,
      boost::optional<gtsam::Matrix &> H_Vx = boost::none,
      boost::optional<gtsam::Matrix &> H_VAx = boost::none) const override {
    double expected_lddot = computeLddot(wTx, Vx, VAx, H_wTx, H_Vx, H_VAx);
    if (H_lddot) *H_lddot = gtsam::I_1x1;
    if (H_wTx) *H_wTx = -(*H_wTx);
    if (H_Vx) *H_Vx = -(*H_Vx);
    if (H_VAx) *H_VAx = -(*H_VAx);
    return gtsam::Vector1(lddot - expected_lddot);
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
    std::cout << s << "cable accel factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
