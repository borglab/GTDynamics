/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ChainInitializer.cpp
 * @brief Utility methods for initializing trajectory optimization solutions for
 * chain graph.
 * @author: Dan Barladeanu
 */

#include <gtdynamics/dynamics/ChainInitializer.h>
#include <gtdynamics/kinematics/KinematicInitializer.h>
#include <gtdynamics/utils/values.h>

namespace gtdynamics {

ChainInitializer::ChainInitializer(const Robot& robot, double gaussianNoise)
    : ZeroSliceInitializer(robot, gaussianNoise) {}

gtsam::Values ChainInitializer::run(
    const int t, const std::optional<PointOnLinks>& contactPoints) const {
  gtsam::Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussianNoise_);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize link dynamics to 0.
  for (auto&& link : robot_.links()) {
    const int i = link->id();
    // TODO(Varun): Not sure what's happening here.
    // The link IDs are not guaranteed to be consistent across robots.
    // If these IDs are for the feet, they should be passed in as an argument.
    if (i == 0 || i == 3 || i == 6 || i == 9 || i == 12)
      InsertPose(&values, i, t, addGaussianNoiseToPose(link->bMcom(), sampler));
  }
  InsertTwist(&values, 0, t, sampler.sample());
  InsertTwistAccel(&values, 0, t, sampler.sample());

  // Initialize joint kinematics/dynamics to 0.
  for (auto&& joint : robot_.joints()) {
    const int j = joint->id();
    if (joint->parent()->name().find("trunk") != std::string::npos) {
      InsertWrench(&values, joint->parent()->id(), j, t, sampler.sample());
    }
    std::vector<DynamicsSymbol> keys = {JointAngleKey(j, t), JointVelKey(j, t),
                                        JointAccelKey(j, t)};
    for (size_t i = 0; i < keys.size(); i++)
      values.insert(keys[i], sampler.sample()[0]);
  }

  // TODO(Varun): Why are these commented out?
  // if (contact_points) {
  // for (auto&& cp : *contact_points) {
  // TODO(frank): allow multiple contact points on one link, id = 0,1,2...
  // values.insert(ContactWrenchKey(cp.link->id(), 0, t), sampler.sample());
  // std::cout<<t<<std::endl;
  // InsertPose(&values, cp.link->id(), t,
  // addGaussianNoiseToPose(cp.link->bMcom(), sampler));
  //}
  //}

  return values;
}

}  // namespace gtdynamics