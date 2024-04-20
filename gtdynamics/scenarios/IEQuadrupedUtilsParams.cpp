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

#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

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
void IEVision60Robot::PhaseInfo::print() const {
  contact_indices.print("contact indices: ");
  leaving_indices.print("leaving indices: ");
  landing_indices.print("landing indices: ");
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr IEVision60Robot::PhaseInfo::Ground() {
  return std::make_shared<PhaseInfo>(IndexSet{0, 1, 2, 3}, IndexSet{},
                                     IndexSet{});
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr
IEVision60Robot::PhaseInfo::BackOnGround() {
  return std::make_shared<PhaseInfo>(IndexSet{2, 3}, IndexSet{}, IndexSet{});
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr
IEVision60Robot::PhaseInfo::FrontOnGround() {
  return std::make_shared<PhaseInfo>(IndexSet{0, 1}, IndexSet{}, IndexSet{});
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr IEVision60Robot::PhaseInfo::InAir() {
  return std::make_shared<PhaseInfo>(IndexSet{}, IndexSet{}, IndexSet{});
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr
IEVision60Robot::PhaseInfo::BoundaryLeave(const PhaseInfo &phase0_params,
                                          const PhaseInfo &phase1_params) {

  IndexSet _leaving_indices = IndexSet{};
  IndexSet _contact_indices = IndexSet{};
  std::set_difference(
      phase0_params.contact_indices.begin(),
      phase0_params.contact_indices.end(),
      phase1_params.contact_indices.begin(),
      phase1_params.contact_indices.end(),
      std::inserter(_leaving_indices, _leaving_indices.begin()));
  std::set_union(phase0_params.contact_indices.begin(),
                 phase0_params.contact_indices.end(),
                 phase1_params.contact_indices.begin(),
                 phase1_params.contact_indices.end(),
                 std::inserter(_contact_indices, _contact_indices.begin()));
  return std::make_shared<PhaseInfo>(_contact_indices, _leaving_indices,
                                     IndexSet{});
}

/* ************************************************************************* */
IEVision60Robot::PhaseInfo::shared_ptr
IEVision60Robot::PhaseInfo::BoundaryLand(const PhaseInfo &phase0_params,
                                         const PhaseInfo &phase1_params) {
  IndexSet _contact_indices = IndexSet{};
  IndexSet _landing_indices = IndexSet{};
  std::set_difference(
      phase1_params.contact_indices.begin(),
      phase1_params.contact_indices.end(),
      phase0_params.contact_indices.begin(),
      phase0_params.contact_indices.end(),
      std::inserter(_landing_indices, _landing_indices.begin()));
  std::set_union(phase0_params.contact_indices.begin(),
                 phase0_params.contact_indices.end(),
                 phase1_params.contact_indices.begin(),
                 phase1_params.contact_indices.end(),
                 std::inserter(_contact_indices, _contact_indices.begin()));
  return std::make_shared<PhaseInfo>(_contact_indices, IndexSet{},
                                     _landing_indices);
}

} // namespace gtsam
