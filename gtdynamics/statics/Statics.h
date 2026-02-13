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

#include <gtdynamics/dynamics/MechanicsParameters.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/**
 * @fn Calculate gravity wrench
 * @param gravity 3-vector indicating gravity force, typically, [0,0,-g]
 * @param mass link mass, in kg.
 * @param wTcom pose of link center of mass frame
 * @param H_wTcom optional 6x6 Jacobian of wrench wrt COM pose
 * @returns 6x1 gravity wrench in CoM frame
 */
gtsam::Vector6 GravityWrench(const gtsam::Vector3& gravity, double mass,
                             const gtsam::Pose3& wTcom,
                             gtsam::OptionalJacobian<6, 6> H_wTcom = {});

/**
 * @fn Calculate sum of wrenches with optional Jacobians (all identity!).
 * @param wrenches external wrenches
 * @param H optional Jacobians, if given needs to be same size as wrenches.
 */
///
gtsam::Vector6 ResultantWrench(const std::vector<gtsam::Vector6>& wrenches,
                               gtsam::OptionalMatrixVecType H = nullptr);

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
                               std::optional<gtsam::Vector3> gravity,
                               gtsam::OptionalMatrixVecType H = nullptr);

/// Noise models etc specific to Statics class
struct StaticsParameters : public MechanicsParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  gtsam::SharedNoiseModel fs_cost_model;  // statics cost model

  /// Constructor with default arguments
  StaticsParameters(double sigma_dynamics = 1e-5,
                    const std::optional<gtsam::Vector3>& gravity = {},
                    const std::optional<gtsam::Vector3>& planar_axis = {})
      : MechanicsParameters(sigma_dynamics, gravity, planar_axis),
        fs_cost_model(Isotropic::Sigma(6, 1e-4)) {}

  /// Constructor from shared mechanics parameters.
  explicit StaticsParameters(const MechanicsParameters& mechanics_parameters,
                             const gtsam::SharedNoiseModel& fs_cost_model =
                                 Isotropic::Sigma(6, 1e-4))
      : MechanicsParameters(mechanics_parameters), fs_cost_model(fs_cost_model) {}
};

/// Algorithms for Statics, i.e. kinematics + wrenches at rest
class Statics : public Kinematics {
 protected:
  const StaticsParameters p_;  // overrides Base::p_

 public:
  /**
   * @fn Constructor.
   */
  Statics(const StaticsParameters& parameters = StaticsParameters())
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
   * Create keys for unknowns and initial values.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   * @param robot Robot specification from URDF/SDF.
   * @param gaussian_noise noise (stddev) added to initial values (default 0.0).
   */
  gtsam::Values initialValues(const Slice& slice, const Robot& robot,
                              double gaussian_noise = 0.0) const;

  /**
   * Solve for wrenches given kinematics configuration.
   * @param slice Slice instance.
   * @param robot Robot specification from URDF/SDF.
   * @param configuration A known kinematics configuration.
   * @param gaussian_noise noise (stddev) added to initial values (default 0.0).
   */
  gtsam::Values solve(const Slice& slice, const Robot& robot,
                      const gtsam::Values& configuration,
                      double gaussian_noise = 0.0) const;

  /**
   * Solve for wrenches and kinematics configuration.
   * @param slice Slice instance.
   */
  gtsam::Values minimizeTorques(const Slice& slice, const Robot& robot) const;
};
}  // namespace gtdynamics
