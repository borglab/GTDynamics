/**
 * @file  CableTensionFactor.h
 * @brief Cable tension factor: relates cable tension, two mounting points, and
 * resultant forces
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

/** CableTensionFactor is a 3-way nonlinear factor which enforces relation
 * amongst cable tension, end effector pose, and wrench felt by the end effector
 */
class CableTensionFactor
    : public gtsam::NoiseModelFactor3<double, gtsam::Pose3, gtsam::Vector6> {
 private:
  using Pose3 = gtsam::Pose3;
  using Point3 = gtsam::Point3;
  using Vector6 = gtsam::Vector6;
  using Vector3 = gtsam::Vector3;
  using This = CableTensionFactor;
  using Base = gtsam::NoiseModelFactor3<double, Pose3, Vector6>;

  Point3 wPa_, xPb_;

 public:
  /** Cable tension factor
   * @param tension_key -- key for cable tension (scalar)
   * @param xPose_key -- key for end effector pose
   * @param wrench_key -- key for the wrench acting on the end-effector (in the
   end effector's reference frame)
   * @param cost_model -- noise model (6 dimensional)
   * @param wPa -- cable mounting location on the fixed frame, in world coords
   * @param xPb -- cable mounting location on the end effector, in the
   * end-effector frame (wPb = wTx * xPb)
   */
  CableTensionFactor(gtsam::Key tension_key, gtsam::Key xPose_key,
                     gtsam::Key wrench_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const Point3 &wPa, const Point3 &xPb)
      : Base(cost_model, tension_key, xPose_key, wrench_key),
        wPa_(wPa),
        xPb_(xPb) {}
  virtual ~CableTensionFactor() {}

 private:
  /** Computes the wrench acting on the end-effector due to some cable tension
   * and at some pose.
   * @param tension the tension on the cable
   * @param wTx the pose of the end effector
   * @return Vector6: calculated wrench
   */
  Vector6 computeWrench(
      double tension, const Pose3 &wTx,
      boost::optional<gtsam::Matrix &> H_t = boost::none,
      boost::optional<gtsam::Matrix &> H_wTx = boost::none) const;

  // an alternate version of the above function that uses adjoint; will upgrade
  // to this version once I figure out how to get the jacobian from an Adjoint
  // operation
  Vector6 computeWrench2(
      double tension, const Pose3 &wTx,
      boost::optional<gtsam::Matrix &> H_t = boost::none,
      boost::optional<gtsam::Matrix &> H_wTx = boost::none) const;

 public:
  /** Cable wrench factor
   * @param t cable tension
   * @param wTx end effector pose (in the world frame)
   * @param Fx wrench acting on the end effector (in the end effector frame)
   * @return Vector(6): Fx minus calculated wrench
   */
  gtsam::Vector evaluateError(
      const double &t, const Pose3 &wTx, const Vector6 &Fx,
      boost::optional<gtsam::Matrix &> H_t = boost::none,
      boost::optional<gtsam::Matrix &> H_wTx = boost::none,
      boost::optional<gtsam::Matrix &> H_Fx = boost::none) const override {
    Vector6 error =
        (Vector6() << Fx - computeWrench(t, wTx, H_t, H_wTx)).finished();
    if (H_t) *H_t = -(*H_t);
    if (H_wTx) *H_wTx = -(*H_wTx);
    if (H_Fx) *H_Fx = gtsam::I_6x6;
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
                 GTDKeyFormatter) const override {
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
