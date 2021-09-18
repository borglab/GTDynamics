/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Statics.h
 * @brief Wrench calculations for configurations at rest.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include "gtdynamics/kinematics/Kinematics.h"
#include "gtdynamics/utils/Slice.h"

namespace gtdynamics {

/**
 * @fn Calculate gravity wrench
 * @param gravity 3-vector indicating gravity force, typically, [0,0,-g]
 * @param mass link mass, in kg.
 * @param wTcom pose of link center of mass frame
 * @param H_wTcom optional 6x6 Jacobian of wrench wrt COM pose
 * @returns 6x1 gravity wrench in CoM frame
 */
gtsam::Vector6 GravityWrench(
    const gtsam::Vector3& gravity, double mass, const gtsam::Pose3& wTcom,
    gtsam::OptionalJacobian<6, 6> H_wTcom = boost::none);

/**
 * @fn Calculate sum of wrenches with optional Jacobians (all identity!).
 * @param wrenches external wrenches
 * @param H optional Jacobians, if given needs to be same size as wrenches.
 */
///
gtsam::Vector6 ResultantWrench(
    const std::vector<gtsam::Vector6>& wrenches,
    boost::optional<std::vector<gtsam::Matrix>&> H = boost::none);

/**
 * @fn Calculate sum of external wrenches and gravity wrench on a body.
 * @param wrenches external wrenches
 * @param mass mass of the body
 * @param wTcom pose of body center of mass frame
 * @param gravity optional gravity vector
 * @param H optional Jacobians, back H corresponds to wTcom.
 */
gtsam::Vector6 ResultantWrench(const std::vector<gtsam::Vector6>& wrenches,
                               double mass, const gtsam::Pose3& wTcom,
                               boost::optional<gtsam::Vector3> gravity,
                               boost::optional<std::vector<gtsam::Matrix>&> H);

/// Noise models etc specific to Statics class
struct StaticsParameters : public KinematicsParameters {
  boost::optional<gtsam::Vector3> gravity, planar_axis;

  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel fs_cost_model,  // statics cost model
      f_cost_model,                             // wrench equivalence factor
      t_cost_model,                             // torque factor
      planar_cost_model;                        // planar factor

  /// Constructor with default arguments
  StaticsParameters(
      double sigma_dynamics = 1e-5,
      const boost::optional<gtsam::Vector3>& gravity = boost::none,
      const boost::optional<gtsam::Vector3>& planar_axis = boost::none)
      : gravity(gravity),
        planar_axis(planar_axis),
        fs_cost_model(Isotropic::Sigma(6, 1e-4)),
        f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)) {}
};

/// Algorithms for Statics, i.e. kinematics + wrenches at rest
class Statics : public Kinematics {
 protected:
  boost::shared_ptr<const StaticsParameters> p_;  // overrides Base::p_

 public:
  /**
   * @fn Constructor.
   */
  Statics(const boost::shared_ptr<const StaticsParameters>& parameters)
      : Kinematics(parameters), p_(parameters) {}

  /// Graph with a WrenchEquivalenceFactor for each joint
  gtsam::NonlinearFactorGraph wrenchEquivalenceFactors(
      const Slice& slice, const Robot& robot) const;

  /// Graph with a TorqueFactor for each joint
  gtsam::NonlinearFactorGraph torqueFactors(const Slice& slice,
                                            const Robot& robot) const;

  /// Graph with a WrenchPlanarFactor for each joint
  gtsam::NonlinearFactorGraph wrenchPlanarFactors(const Slice& slice,
                                                  const Robot& robot) const;

  /**
   * Create graph with only static balance factors.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   * @param robot Robot specification from URDF/SDF.
   */
  gtsam::NonlinearFactorGraph graph(const Slice& slice,
                                    const Robot& robot) const;

  /**
   * Create keys for unkowns and initial values.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   * @param robot Robot specification from URDF/SDF.
   */
  gtsam::Values initialValues(const Slice& slice, const Robot& robot,
                              double gaussian_noise = 1e-6) const;

  /**
   * Solve for wrenches given kinematics configuration.
   * @param slice Slice instance.
   * @param robot Robot specification from URDF/SDF.
   * @param configuration A known kinematics configuration.
   */
  gtsam::Values solve(const Slice& slice, const Robot& robot,
                      const gtsam::Values& configuration) const;

  /**
   * Solve for wrenches and kinematics configuration.
   * @param slice Slice instance.
   */
  gtsam::Values minimizeTorques(const Slice& slice, const Robot& robot) const;
};
}  // namespace gtdynamics
