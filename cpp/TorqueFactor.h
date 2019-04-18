/**
 * @file  TorqueFactor.h
 * @brief toque factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace manipulator {

/** TorqueFactor is a two-way nonlinear factor which enforces relation between
 * wrench and torque on each link*/
class TorqueFactor : public gtsam::NoiseModelFactor2<gtsam::Vector, double> {
 private:
  typedef TorqueFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector, double> Base;
  gtsam::Vector6 screw_axis_;

 public:
  /** torque factor, common between forward and inverse dynamics.
      Keyword argument:
          screw_axis -- screw axis expressed in this link's COM frame
      Will create factor corresponding to Lynch & Park book:
      Torque is always wrench projected on screw axis.
      Equation 8.49, page 293 can be written as
      screw_axis.transpose() * F.transpose() == torque
   */
  TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const gtsam::Vector6 &screw_axis)
      : Base(cost_model, wrench_key, torque_key), screw_axis_(screw_axis) {}
  virtual ~TorqueFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          wrench       -- wrench on this link
          torque       -- torque on this link joint
          H_wrench     -- jacobian matrix w.r.t. wrench
          H_torque     -- jacobian matrix w.r.t. torque
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector &wrench, const double &torque,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const {
    if (H_wrench) {
      *H_wrench = screw_axis_.transpose();
    }
    if (H_torque) {
      *H_torque = -gtsam::Matrix::Identity(1, 1);
    }

    return screw_axis_.transpose() * wrench - gtsam::Vector1(torque);
  }

  // @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "torque factor" << std::endl;
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
}  // namespace manipulator
