/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactDynamicsMomentFactor.h
 * @brief Factor to enforce moment at the contact point.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <utils.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>
#include <string>
#include <boost/optional.hpp>

namespace robot {

/** ContactDynamicsMomentFactor is unary nonlinear factor which enforces
 *  zero moment at the contact point for the link. */
class ContactDynamicsMomentFactor
    : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  typedef ContactDynamicsMomentFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector6> Base;
  gtsam::Pose3 cTcom_;
  gtsam::Matrix36 H_contact_wrench_;

 public:
  /** Contact dynamics factor for zero moment at contact.
      Keyword argument:
          contact_wrench_key -- gtsam::LabeledKey corresponding to this link's
            contact wrench.
          cost_model -- gtsam::noiseModel for this factor.
          cTcom      -- Contact frame expressed in com frame.
   */
  ContactDynamicsMomentFactor(
      gtsam::Key contact_wrench_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom)
      : Base(cost_model, contact_wrench_key), cTcom_(cTcom) {
    H_contact_wrench_ = (gtsam::Matrix36() << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                         0, 0, 0, 1, 0, 0, 0)
                            .finished();
  }
  virtual ~ContactDynamicsMomentFactor() {}

 public:
  /** Evaluate contact point moment errors.
      Keyword argument:
          contact_wrench         -- Contact wrench on this link.
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &contact_wrench,
      boost::optional<gtsam::Matrix &> H_contact_wrench =
          boost::none) const override {
    // Transform the twist from the link COM frame to the contact frame.
    gtsam::Vector3 error =
        H_contact_wrench_ * cTcom_.AdjointMap() * contact_wrench;

    if (H_contact_wrench)
      *H_contact_wrench = H_contact_wrench_ * cTcom_.AdjointMap();

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
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "Contact Dynamics Moment Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace robot
