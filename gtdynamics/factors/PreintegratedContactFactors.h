/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PreintegratedContactFactors.h
 * @brief Preintegrated contact factors as defined in Hartley18icra.
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/JointTyped.h"
#include "gtdynamics/universal_robot/Link.h"

namespace gtdynamics {

/**
 * Class to perform preintegration of contact measurements for point foot model.
 */
class PreintegratedPointContactMeasurements {
  gtsam::Matrix3 preintMeasCov_;

 public:
  PreintegratedPointContactMeasurements();

  gtsam::Matrix3 preintMeasCov() const { return preintMeasCov_; }

  //TODO(Varun) Add covariance updates
};

/**
 * Class to perform preintegration of contact measurements for rigid foot model.
 */
class PreintegratedRigidContactMeasurements {
  gtsam::Matrix6 preintMeasCov_;

 public:
  PreintegratedRigidContactMeasurements();

  gtsam::Matrix3 preintMeasCov() const { return preintMeasCov_; }

  //TODO(Varun) Add covariance updates
};

/**
 * The Preintegrated Contact Factor for point foot measurements as defined in
 * Hartley18icra.
 */
class PreintegratedPointContactFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3,
                                      gtsam::Pose3> {
 private:
  using This = PreintegratedPointContactFactor;
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Pose3, gtsam::Pose3>;

 public:
  /**
   * Constructor
   *
   * @param wTbi_key Key for base link pose in world frame at previous step.
   * @param wTci_key Key for contact pose in world frame at previous step.
   * @param wTbi_key Key for base link pose in world frame at current step.
   * @param wTci_key Key for contact pose in world frame at current step.
   * @param pim Preintegrated contact measurements which captures the
   measurement covariance for the point foot model.
   *
   */
  PreintegratedPointContactFactor(
      gtsam::Key wTbi_key, gtsam::Key wTci_key, gtsam::Key wTbj_key,
      gtsam::Key wTcj_key, const PreintegratedPointContactMeasurements &pim)
      : Base(gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov()),
             wTbi_key, wTci_key, wTbj_key, wTcj_key) {}

  /**
   * Convenience constructor using links.
   *
   * @param base_link_i Base link at previous step.
   * @param contact_link_i Link in contact at previous step.
   * @param base_link_j Base link at current step.
   * @param contact_link_j Link in contact at current step.
   * @param pim Preintegrated contact measurements which captures the
   measurement covariance for the point foot model.
   */
  PreintegratedPointContactFactor(
      const LinkSharedPtr &base_link_i, const LinkSharedPtr &contact_link_i,
      const LinkSharedPtr &base_link_j, const LinkSharedPtr &contact_link_j,
      const PreintegratedPointContactMeasurements &pim, size_t time)
      : PreintegratedPointContactFactor(
            internal::PoseKey(base_link_i->id(), time),
            internal::PoseKey(contact_link_i->id(), time),
            internal::PoseKey(base_link_j->id(), time),
            internal::PoseKey(contact_link_j->id(), time), pim) {}

  virtual ~PreintegratedPointContactFactor() {}

  /**
   * @brief
   *
   * @param wTp previous (parent) link CoM pose
   * @param wTc this (child) link CoM pose
   * @param q joint angle
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTb_i, const gtsam::Pose3 &wTc_i,
      const gtsam::Pose3 &wTb_j, const gtsam::Pose3 &wTc_j,
      boost::optional<gtsam::Matrix &> H_wTb_i = boost::none,
      boost::optional<gtsam::Matrix &> H_wTc_i = boost::none,
      boost::optional<gtsam::Matrix &> H_wTb_j = boost::none,
      boost::optional<gtsam::Matrix &> H_wTc_j = boost::none) const override {
    // For Rot3, translation == inverse due to orthogonality
    gtsam::Vector3 error = wTb_i.rotation().inverse() *
                           (wTc_j.translation() - wTc_i.translation());

    // Please refer to the supplementary material for the Jacobian calculations.
    // https://arxiv.org/src/1712.05873v2/anc/icra-supplementary-material.pdf
    if (H_wTb_i) {
      gtsam::Matrix36 H;
      H << gtsam::SO3::Hat(error), gtsam::Z_3x3;
      *H_wTb_i = H;
    }
    if (H_wTc_i) {
      gtsam::Matrix36 H;
      H << gtsam::Z_3x3, -gtsam::I_3x3;
      *H_wTc_i = H;
    }
    if (H_wTb_j) {
      *H_wTb_j = gtsam::Z_6x6;
    }
    if (H_wTc_j) {
      gtsam::Matrix36 H;
      H << gtsam::Z_3x3, wTb_i.rotation().inverse() * wTb_j.rotation();
      *H_wTc_j = H;
    }
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? s : s + " ")
              << "Preintegrated Point Contact Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};

//TODO(Varun) PreintegratedRigidContactFactor

}  // namespace gtdynamics