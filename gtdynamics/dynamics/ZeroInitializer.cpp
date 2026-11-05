/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ZeroInitializer.cpp
 * @brief Construct zero/noisy dynamics values over robot slices.
 * @author Alejandro Escontrela, Yetong Zhang, GTDynamics contributors
 */
#include <gtdynamics/dynamics/ZeroInitializer.h>
#include <gtdynamics/kinematics/KinematicInitializer.h>
#include <gtdynamics/utils/values.h>
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Sampler;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;
namespace gtdynamics {
ZeroSliceInitializer::ZeroSliceInitializer(const Robot& robot,
                                           double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values ZeroSliceInitializer::run(
    const int t, const std::optional<PointOnLinks>& contactPoints) const {
  Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussianNoise_);
  Sampler sampler(sampler_noise_model);

  // Initialize link dynamics to 0.
  for (auto&& link : robot_.links()) {
    int i = link->id();
    InsertPose(&values, i, t, addGaussianNoiseToPose(link->bMcom(), sampler));
    InsertTwist(&values, i, t, sampler.sample());
    InsertTwistAccel(&values, i, t, sampler.sample());
  }

  // Initialize joint kinematics/dynamics to 0.
  for (auto&& joint : robot_.joints()) {
    int j = joint->id();
    InsertWrench(&values, joint->parent()->id(), j, t, sampler.sample());
    InsertWrench(&values, joint->child()->id(), j, t, sampler.sample());

    // Joint angles, velocities, accelerations, and torques.
    std::vector<DynamicsSymbol> keys = {TorqueKey(j, t), JointAngleKey(j, t),
                                        JointVelKey(j, t), JointAccelKey(j, t)};
    for (size_t i = 0; i < keys.size(); i++)
      values.insert(keys[i], sampler.sample()[0]);
  }

  if (contactPoints) {
    for (auto&& cp : *contactPoints) {
      // Contact points on one link share its aggregate contact wrench seed.
      // A sole patch can contain several contact points on that same link.
      const auto key = ContactWrenchKey(cp.link->id(), 0, t);
      if (!values.exists(key)) values.insert(key, sampler.sample());
    }
  }

  return values;
}

MultiPhaseZeroInitializer::MultiPhaseZeroInitializer(const Robot& robot,
                                                     const double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values MultiPhaseZeroInitializer::run(
    const std::vector<int>& phaseSteps,
    std::vector<gtsam::Values> transitionValues, double dt,
    const std::optional<std::vector<PointOnLinks>>& phaseContactPoints,
    const ZeroSliceInitializer* sliceInitializer) const {
  const ZeroSliceInitializer defaultInitializer(robot_, gaussianNoise_);
  const auto& initializer =
      sliceInitializer ? *sliceInitializer : defaultInitializer;
  Values values;
  int num_phases = phaseSteps.size();

  // Return either PointOnLinks or None if none specified for phase p
  auto contact_points =
      [&phaseContactPoints](int p) -> std::optional<PointOnLinks> {
    if (phaseContactPoints) return (*phaseContactPoints)[p];
    return {};
  };

  // First slice, k==0
  values.insert(initializer.run(0, contact_points(0)));

  int k = 0;
  for (int p = 0; p < num_phases; p++) {
    // in-phase
    for (int step = 0; step < phaseSteps[p] - 1; step++) {
      values.insert(initializer.run(++k, contact_points(p)));
    }

    if (p == num_phases - 1) {
      values.insert(initializer.run(++k, contact_points(p)));
    } else {
      values.insert(transitionValues[p]);
      k++;
    }
  }

  for (int p = 0; p < num_phases; p++) {
    values.insert(PhaseKey(p), dt);
  }

  return values;
}

ZeroTrajectoryInitializer::ZeroTrajectoryInitializer(const Robot& robot,
                                                     double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values ZeroTrajectoryInitializer::run(
    const int numSteps, const int numPhases,
    const std::optional<PointOnLinks>& contactPoints,
    const ZeroSliceInitializer* sliceInitializer) const {
  const ZeroSliceInitializer defaultInitializer(robot_, gaussianNoise_);
  const auto& initializer =
      sliceInitializer ? *sliceInitializer : defaultInitializer;
  Values z_values;
  for (int t = 0; t <= numSteps; t++)
    z_values.insert(initializer.run(t, contactPoints));
  if (numPhases > 0) {
    for (int phase = 0; phase <= numPhases; phase++)
      z_values.insert(PhaseKey(phase), 0.0);
  }
  return z_values;
}
}  // namespace gtdynamics
