/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TorqueFactor.h
 * @brief Torque factor, common between forward and inverse dynamics.
 * @Author: Frank Dellaert and Mandy Xie
 */

#ifndef GTDYNAMICS_FACTORS_TORQUEFACTOR_H_
#define GTDYNAMICS_FACTORS_TORQUEFACTOR_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

namespace gtdynamics {
// Forward declarations
class NonlinearDynamicsConditional;

/** TorqueFactor is a two-way nonlinear factor which enforces relation between
 * wrench and torque on each link*/
class TorqueFactor : public gtsam::NoiseModelFactor2<gtsam::Vector6, double> {
 private:
  typedef TorqueFactor This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector6, double> Base;
  gtsam::noiseModel::Base::shared_ptr costModel_;
  gtsam::Vector6 screw_axis_;
  gtsam::Key frontalKey_;

 public:
  TorqueFactor() {}

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
               const gtsam::Vector6 &screw_axis,
               const boost::optional<gtsam::Key> &frontalKey = boost::none)
      : Base(cost_model, wrench_key, torque_key),
        costModel_(cost_model),
        screw_axis_(screw_axis) {
    // set the frontal key for nonlinear conditional
    if (frontalKey) {
      frontalKey_ = *frontalKey;
    } else {
      frontalKey_ = wrench_key;
    }
  }
  virtual ~TorqueFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          wrench       -- wrench on this link
          torque       -- torque on this link joint
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &wrench, const double &torque,
      boost::optional<gtsam::Matrix &> H_wrench = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const override {
    if (H_wrench) {
      *H_wrench = screw_axis_.transpose();
    }
    if (H_torque) {
      *H_torque = -gtsam::I_1x1;
    }

    return screw_axis_.transpose() * wrench - gtsam::Vector1(torque);
  }

  gtsam::noiseModel::Base::shared_ptr getCostModel() const {
    return costModel_;
  }
  gtsam::Vector6 getScrewAxis() const { return screw_axis_; }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "torque factor" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Dense elimination function for nonlinear dynamics factors.
   * Keyword argument:
   *   frontalKey  -- frontal key
   *                  if wrench_key is frontal key, then eliminate wrench
   *                  if torque_key is frontal key, then eliminate torque
   * return nonlinear dynaimcs conditional, and the remaining factor 
   */

  std::pair<boost::shared_ptr<NonlinearDynamicsConditional>,
            boost::shared_ptr<TorqueFactor> >
  EliminateNonlinear(const gtsam::Key &frontalKey);

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_TORQUEFACTOR_H_
