/**
 * @file  CableLenFactor.h
 * @brief Cable length factor: relates cable length and two mounting points
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtdynamics/cablerobot/utils/cableUtils.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

#include <iostream>
#include <string>

namespace gtdynamics {
namespace cablerobot {

/** CableLenFactor is a 3-way nonlinear factor which enforces relation for
 * mounting points and cable length
 */
template <class PointSpace>
class CableLenFactor : public gtsam::NoiseModelFactor3<
                           double, PointSpace, PointSpace> {
 private:
  typedef typename gtsam::traits<PointSpace>::TangentVector VelVector;
  enum { N = gtsam::traits<PointSpace>::dimension };
  typedef CableLenFactor This;
  typedef gtsam::NoiseModelFactor3<double, PointSpace, PointSpace>
      Base;

 public:
  /** Cable factor
      Arguments:
          l_key -- key for cable length
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
          cost_model -- noise model (1 dimensional)
   */
  CableLenFactor(gtsam::Key l_key, gtsam::Key point1_key,
              gtsam::Key point2_key,
              const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, l_key, point1_key, point2_key) {}
  virtual ~CableLenFactor() {}

 public:
  /** Cable factor
      Arguments:
          l_key -- key for cable velocity
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
   */
  gtsam::Vector evaluateError(
      const double &l, const PointSpace &point1,
      const PointSpace &point2,
      boost::optional<gtsam::Matrix &> H_l = boost::none,
      boost::optional<gtsam::Matrix &> H_point1 = boost::none,
      boost::optional<gtsam::Matrix &> H_point2 = boost::none) const override {
    double expected_l = distance(point2, point1, H_point2, H_point1);
    if (H_l) *H_l = gtsam::Vector1(-1);
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
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace cablerobot
}  // namespace gtdynamics
