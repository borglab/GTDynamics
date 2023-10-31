/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>

using namespace gtdynamics;

namespace gtsam {
/* ************************************************************************* */
IEVision60Robot::Leg::Leg(const gtdynamics::Robot &robot,
                          const std::string &hip_joint_name,
                          const std::string &upper_joint_name,
                          const std::string &lower_joint_name,
                          const std::string &hip_link_name,
                          const std::string &upper_link_name,
                          const std::string &lower_link_name)
    : hip_joint(robot.joint(hip_joint_name)),
      upper_joint(robot.joint(upper_joint_name)),
      lower_joint(robot.joint(lower_joint_name)),
      hip_link(robot.link(hip_link_name)),
      upper_link(robot.link(upper_link_name)),
      lower_link(robot.link(lower_link_name)), hip_joint_id(hip_joint->id()),
      upper_joint_id(upper_joint->id()), lower_joint_id(lower_joint->id()),
      hip_link_id(hip_link->id()), upper_link_id(upper_link->id()),
      lower_link_id(lower_link->id()),
      joints({hip_joint, upper_joint, lower_joint}),
      links({hip_link, upper_link, lower_link}) {}

/* ************************************************************************* */
void IEVision60Robot::Params::set4C() {
  contact_indices = IndexSet{0, 1, 2, 3};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
}

/* ************************************************************************* */
void IEVision60Robot::Params::setBackOnGround() {
  contact_indices = IndexSet{2, 3};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
}

/* ************************************************************************* */
void IEVision60Robot::Params::setFrontOnGround() {
  contact_indices = IndexSet{0, 1};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
}

/* ************************************************************************* */
void IEVision60Robot::Params::setInAir() {
  contact_indices = IndexSet{};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
}

/* ************************************************************************* */
void IEVision60Robot::Params::setBoundaryLeave(const Params &phase0_params,
                                               const Params &phase1_params) {
  contact_indices = IndexSet{};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
  std::set_difference(phase0_params.contact_indices.begin(),
                      phase0_params.contact_indices.end(),
                      phase1_params.contact_indices.begin(),
                      phase1_params.contact_indices.end(),
                      std::inserter(leaving_indices, leaving_indices.begin()));
  std::set_union(phase0_params.contact_indices.begin(),
                 phase0_params.contact_indices.end(),
                 phase1_params.contact_indices.begin(),
                 phase1_params.contact_indices.end(),
                 std::inserter(contact_indices, contact_indices.begin()));
}

/* ************************************************************************* */
void IEVision60Robot::Params::setBoundaryLand(const Params &phase0_params,
                                              const Params &phase1_params) {
  contact_indices = IndexSet{};
  leaving_indices = IndexSet{};
  landing_indices = IndexSet{};
  std::set_difference(phase1_params.contact_indices.begin(),
                      phase1_params.contact_indices.end(),
                      phase0_params.contact_indices.begin(),
                      phase0_params.contact_indices.end(),
                      std::inserter(landing_indices, landing_indices.begin()));
  std::set_union(phase0_params.contact_indices.begin(),
                 phase0_params.contact_indices.end(),
                 phase1_params.contact_indices.begin(),
                 phase1_params.contact_indices.end(),
                 std::inserter(contact_indices, contact_indices.begin()));
}

} // namespace gtsam
