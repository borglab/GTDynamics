/**
 * @file  CableVelFactor.h
 * @brief Cable velocity factor: relates cable velocity (or acceleration), two
 * mounting points, and resultant mounting point velocities
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

/** CableVelFactor is a 5-way nonlinear factor which enforces relation amongst
 * cable velocity, mounting points, and the velocity of the EE mounting point
 */
template <class PointSpace>
class CableVelFactor : public gtsam::NoiseModelFactor5<
                           double, PointSpace, PointSpace,
                           typename gtsam::traits<PointSpace>::TangentVector,
                           typename gtsam::traits<PointSpace>::TangentVector> {
 private:
  typedef typename gtsam::traits<PointSpace>::TangentVector VelVector;
  enum { N = gtsam::traits<PointSpace>::dimension };
  typedef CableVelFactor This;
  typedef gtsam::NoiseModelFactor5<double, PointSpace, PointSpace, VelVector,
                                   VelVector>
      Base;

 public:
  /** Cable factor
      Arguments:
          v_key -- key for cable velocity
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
          vel1_key -- key for velocity attachment point 1
          vel2_key -- key for velocity attachment point 2
          cost_model -- noise model (6 dimensional: 3 force directions for 2
     attachment points)
   */
  CableVelFactor(gtsam::Key v_key, gtsam::Key point1_key,
              gtsam::Key point2_key, gtsam::Key vel1_key, gtsam::Key vel2_key,
              const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, v_key, point1_key, point2_key, vel1_key, vel2_key) {}
  virtual ~CableVelFactor() {}

 public:
  /** Cable factor
      Arguments:
          v_key -- key for cable velocity
          point1_key -- key for attachment point 1
          point2_key -- key for attachment point 2
          vel1_key -- key for force on attachment point 1
          force2_key -- key for force on attachment point 2
   */
  gtsam::Vector evaluateError(
      const double &v, const PointSpace &point1,
      const PointSpace &point2, const VelVector &vel1, const VelVector &vel2,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_point1 = boost::none,
      boost::optional<gtsam::Matrix &> H_point2 = boost::none,
      boost::optional<gtsam::Matrix &> H_vel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_vel2 = boost::none) const override {
    Eigen::Matrix<double, N, N> dir_H_Dir;  // || vel2 - vel1 ||
    Eigen::Matrix<double, 1, N> err_H_Dir;  // v x (vel2 - vel1)
    Eigen::Matrix<double, 1, N> err_H_dir;
    Eigen::Matrix<double, 1, N> err_H_Vel;

    PointSpace dir =
        normalize(point2 - point1, (H_point1 || H_point2) ? &dir_H_Dir : 0);
    double expected_v =
        dot(dir, vel2 - vel1, (H_point1 || H_point2) ? &err_H_dir : 0,
            (H_vel1 || H_vel2) ? &err_H_Vel : 0);

    if (H_v) *H_v = gtsam::Vector1(-1);
    if (H_point1 || H_point2)
      err_H_Dir << err_H_dir * dir_H_Dir;
    if (H_point1) *H_point1 = -err_H_Dir;
    if (H_point2) *H_point2 = err_H_Dir;
    if (H_vel1)
      *H_vel1 = -err_H_Vel;
    if (H_vel2)
      *H_vel2 = err_H_Vel;

    return gtsam::Vector1( expected_v - v );
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
    std::cout << s << "cable factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace cablerobot
}  // namespace gtdynamics
