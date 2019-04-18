/**
 * @file  JointLimitVectorFactor.h
 * @brief apply joint limit
 * @Author: Frank Dellaert and Mandy Xie
 */
#ifndef JOINTLIMITVECTORFACTOR_H
#define JOINTLIMITVECTORFACTOR_H

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace manipulator {

class JointLimitVectorFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  typedef JointLimitVectorFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;
  gtsam::Vector lower_limits_, upper_limits_, limit_thresholds_;

 public:
  /**
   * Construct from joint angle limits
   * Keyword arguments:
      pose_key (key)                   --
      cost_model (noiseModel::Base::shared_ptr)    --
      lower_limit (Vector)             -- joint angle lower limit
      upper_limit (Vector)             -- joint angle upper limit
      limit_threshold (Vector)         -- joint angle limit threshold
   */
  JointLimitVectorFactor(gtsam::Key pose_key,
                         const gtsam::noiseModel::Base::shared_ptr &cost_model,
                         const gtsam::Vector &lower_limits,
                         const gtsam::Vector &upper_limits,
                         const gtsam::Vector &limit_thresholds)
      : Base(cost_model, pose_key),
        lower_limits_(lower_limits),
        upper_limits_(upper_limits),
        limit_thresholds_(limit_thresholds) {
    // check dimensions
    assert(lower_limits.size() == cost_model->dim());
    assert(upper_limits.size() == cost_model->dim());
    assert(limit_thresholds.size() == cost_model->dim());
  }

  virtual ~JointLimitVectorFactor() {}

 private:
  /** calculate joint limit cost
      Keyword argument:
          p --
          lower_limit --
          upper_limit --
          limit_threshold --
          error --
          H_p --
  */
  double jointLimitCost_(double p, double lower_limit, double upper_limit,
                         double limit_threshold,
                         boost::optional<double &> H_p = boost::none) const {
    if (p < lower_limit + limit_threshold) {
      if (H_p) *H_p = -1.0;
      return lower_limit + limit_threshold - p;
    } else if (p <= upper_limit - limit_threshold) {
      if (H_p) *H_p = 0.0;
      return 0.0;
    } else {
      if (H_p) *H_p = 1.0;
      return p - upper_limit + limit_threshold;
    }
  }

 public:
  /** evaluate joint angle limit errors
      Keyword argument:
          conf   --
          H      --
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector &conf,
      boost::optional<gtsam::Matrix &> H = boost::none) const {
    int size = conf.size();
    gtsam::Vector errors = gtsam::Vector::Zero(size);
    if (H) {
      *H = gtsam::Matrix::Zero(size, size);
    }
    for (int i = 0; i < size; ++i) {
      if (H) {
        errors(i) = jointLimitCost_(conf(i), lower_limits_(i), upper_limits_(i),
                                    limit_thresholds_(i), (*H)(i, i));
      } else {
        errors(i) = jointLimitCost_(conf(i), lower_limits_(i), upper_limits_(i),
                                    limit_thresholds_(i));
      }
    }
    return errors;
  }

  // @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
#endif
