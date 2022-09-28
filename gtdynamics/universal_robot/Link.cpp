/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Link.cpp
 * @brief Abstract representation of a robot link.
 */

#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/statics/Statics.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/slam/expressions.h>

#include <iostream>

namespace gtdynamics {

/* ************************************************************************* */
gtsam::Vector6_ Link::wrenchConstraint(
    const std::vector<gtsam::Key>& wrench_keys, uint64_t t,
    const boost::optional<gtsam::Vector3>& gravity) const {
  // Collect wrenches to implement L&P Equation 8.48 (F = ma)
  std::vector<gtsam::Vector6_> wrenches;

  // Coriolis forces.
  gtsam::Vector6_ twist(TwistKey(id(), t));
  // TODO(yetong): make Coriolis a functor and get rid of placeholders.
  gtsam::Vector6_ wrench_coriolis(
      std::bind(Coriolis, inertiaMatrix(), std::placeholders::_1,
                std::placeholders::_2),
      twist);
  wrenches.push_back(wrench_coriolis);

  // Change in generalized momentum.
  const gtsam::Matrix6 neg_inertia = -inertiaMatrix();
  gtsam::Vector6_ twistAccel(TwistAccelKey(id(), t));
  gtsam::Vector6_ wrench_momentum(
      std::bind(MatVecMult<6, 6>, neg_inertia, std::placeholders::_1,
                std::placeholders::_2),
      twistAccel);
  wrenches.push_back(wrench_momentum);

  // External wrenches.
  for (const auto& key : wrench_keys) {
    wrenches.push_back(gtsam::Vector6_(key));
  }

  // Gravity wrench.
  if (gravity) {
    gtsam::Pose3_ pose(PoseKey(id(), t));
    gtsam::Vector6_ wrench_gravity(
        std::bind(GravityWrench, *gravity, mass_, std::placeholders::_1,
                  std::placeholders::_2),
        pose);
    wrenches.push_back(wrench_gravity);
  }

  // Calculate resultant wrench.
  gtsam::Vector6_ error(gtsam::Z_6x1);
  for (const auto& wrench : wrenches) {
    error += wrench;
  }
  return error;
}

/* ************************************************************************* */
gtsam::Matrix6 Link::inertiaMatrix() const {
  std::vector<gtsam::Matrix> gmm;
  gmm.push_back(inertia_);
  gmm.push_back(gtsam::I_3x3 * mass_);
  return gtsam::diag(gmm);
}

/* ************************************************************************* */
bool Link::operator==(const Link& other) const {
  return (this->name_ == other.name_ && this->id_ == other.id_ &&
          this->mass_ == other.mass_ &&
          this->centerOfMass_.equals(other.centerOfMass_) &&
          this->inertia_ == other.inertia_ &&
          this->bMcom_.equals(other.bMcom_) &&
          this->bMlink_.equals(other.bMlink_) &&
          this->is_fixed_ == other.is_fixed_ &&
          this->fixed_pose_.equals(other.fixed_pose_));
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Link& link) {
  std::string fixed = link.isFixed() ? " (fixed)" : "";
  os << link.name() << ", id=" << size_t(link.id()) << fixed << ":\n";
  os << "\tcom pose:  " << link.bMcom().rotation().rpy().transpose() << ", "
     << link.bMcom().translation().transpose() << "\n";
  os << "\tlink pose: " << link.bMlink().rotation().rpy().transpose() << ", "
     << link.bMlink().translation().transpose() << std::endl;
  return os;
}

/* ************************************************************************* */
void Link::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this;
}

}  // namespace gtdynamics
