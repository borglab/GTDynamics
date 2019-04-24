/**
 * @file  ObstacleSDFFactor.h
 * @brief obstacle signed distance field factor used for collision check
 * @Author: Mandy Xie and Frank Dellaert
 */

# pragma once

#include <SignedDistanceField.h>
#include <SphereLink.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace manipulator {

/** hinge loss obstacle cost function */
inline double hingeLossObstacleCost(
    const gtsam::Point3 &point, const SignedDistanceField &sdf, double eps,
    gtsam::OptionalJacobian<1, 3> H_point = boost::none) {
  gtsam::Vector3 field_gradient;
  double dist_signed;
  try {
    dist_signed = sdf.getSignedDistance(point, field_gradient);
  } catch (std::runtime_error) {
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (dist_signed > eps) {
    // faraway no error
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;

  } else {
    // outside but < eps or inside object
    if (H_point) *H_point = -field_gradient.transpose();
    return eps - dist_signed;
  }
}

/** unary factor for collision check */
class ObstacleSDFFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 private:
  // typedefs
  typedef ObstacleSDFFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  // obstacle cost settings
  double epsilon_;  // distance from object that start non-zero cost

  // signed distance field
  const SignedDistanceField &sdf_;

  // link sphere model
  SphereLink sphere_link_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * Keyword arguments:
   *  cost_model -- cost function covariance, should be identity model
   *  field      -- signed distance field
   *  nn_index   -- nearest neighbour index of signed distance field
   */
  ObstacleSDFFactor(gtsam::Key poseKey,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    double epsilon, const SignedDistanceField &sdf,
                    double radius, std::vector<gtsam::Point3> &sphere_centers)
      : Base(cost_model, poseKey),
        epsilon_(epsilon),
        sdf_(sdf),
        sphere_link_(radius, sphere_centers) {}

  ~ObstacleSDFFactor() {}

  /** evaluate link pose errors
    Keyword argument:
      pose         -- this link COM pose
  */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H_pose = boost::none) const override {
    // if Jacobians used, initialize as zeros
    auto num = sphere_link_.num_sphere();
    auto radius = sphere_link_.radius();
    if (H_pose) *H_pose = gtsam::Matrix::Zero(num, 6);
    // allocate cost vector
    gtsam::Vector error = gtsam::Vector::Zero(num);

    for (size_t i = 0; i < num; ++i) {
      if (H_pose) {
        gtsam::Matrix36 H_point;
        auto center = sphere_link_.sphereCenter(i, pose, H_point);
        gtsam::Matrix13 H_hingeLoss;
        error[i] =
            hingeLossObstacleCost(center, sdf_, epsilon_ + radius, H_hingeLoss);
        H_pose->row(i) = H_hingeLoss * H_point;
      } else {
        auto center = sphere_link_.sphereCenter(i, pose);
        error[i] = hingeLossObstacleCost(center, sdf_, epsilon_ + radius);
      }
    }
    return error;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "ObstacleSDFFactor :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
