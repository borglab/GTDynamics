/**
 * @file  BaseTwistFactor.h
 * @brief Factor enforcing base twist.
 * @Author: Frank Dellaert and Mandy Xie
 */
#ifndef BASETWISTFACTOR_H
#define BASETWISTFACTOR_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace manipulator {

/** BaseTwistFactor is a one-way nonlinear factor which enforces the twist of
 * the base*/
class BaseTwistFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  typedef BaseTwistFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;
  gtsam::Vector base_twist_;

 public:
  /** Construct Factor enforcing base twist.
   *  Keyword argument:
          base_twist  -- twist for base
   */
  BaseTwistFactor(gtsam::Key twist_key_0,
                  const gtsam::noiseModel::Base::shared_ptr &cost_model,
                  const gtsam::Vector &base_twist)
      : Base(cost_model, twist_key_0), base_twist_(base_twist) {}

  virtual ~BaseTwistFactor() {}

 public:
  /** evaluate base twist errors
      Keyword argument:
          twist_0    -- twist on the base
          H_twist_0  -- jacobian matrix w.r.t. twist_0
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector &twist_0,
      boost::optional<gtsam::Matrix &> H_twist_0 = boost::none) const {
    int size = twist_0.size();
    if (H_twist_0) {
      *H_twist_0 = gtsam::Matrix::Identity(size, size);
    }
    return twist_0 - base_twist_;
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
    std::cout << s << "base twist factor" << std::endl;
    Base::print("", keyFormatter);
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
