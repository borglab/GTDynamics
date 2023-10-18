/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ChainInitializer.h
 * @brief Utility methods for initializing trajectory optimization solutions for
 * chain graph.
 * @author: Dan Barladeanu
 */

#include <gtdynamics/utils/ChainInitializer.h>

namespace gtdynamics {

gtsam::Values ChainInitializer::ZeroValues(
    const Robot& robot, const int t, double gaussian_noise,
    const std::optional<PointOnLinks>& contact_points) const {
  gtsam::Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize link dynamics to 0.
  for (auto&& link : robot.links()) {
    int i = link->id();
    InsertPose(&values, i, t, AddGaussianNoiseToPose(link->bMcom(), sampler));
    InsertTwist(&values, i, t, sampler.sample());
    InsertTwistAccel(&values, i, t, sampler.sample());
  }

  // Initialize joint kinematics/dynamics to 0.
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    if (joint->parent()->name().find("trunk") < 100) {
      InsertWrench(&values, joint->parent()->id(), j, t, sampler.sample());
    }
    std::vector<DynamicsSymbol> keys = {TorqueKey(j, t), JointAngleKey(j, t),
                                        JointVelKey(j, t), JointAccelKey(j, t)};
    for (size_t i = 0; i < keys.size(); i++)
      values.insert(keys[i], sampler.sample()[0]);
  }

  if (contact_points) {
    for (auto&& cp : *contact_points) {
      // TODO(frank): allow multiple contact points on one link, id = 0,1,2...
      values.insert(ContactWrenchKey(cp.link->id(), 0, t), sampler.sample());
    }
  }

  return values;
}

}  // namespace gtdynamics