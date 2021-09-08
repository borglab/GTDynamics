/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Link.h
 * @brief only link part of a robot, does not include joint part
 * @author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <string>
#include <vector>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/factors/WrenchFactor.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

class Link;   // forward declaration
class Joint;  // forward declaration

LINK_TYPEDEF_CLASS_POINTER(Link);
LINK_TYPEDEF_CLASS_POINTER(Joint);

/**
 * @class Base class for links taking different format of parameters.
 */
class Link : public boost::enable_shared_from_this<Link> {
 private:
  uint8_t id_;
  std::string name_;

  /// Inertial elements.
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;

  /// SDF Elements.
  gtsam::Pose3 bMcom_;  // CoM frame defined in the base frame at rest.
  gtsam::Pose3 bMlink_; // link frame defined in the base frame at rest.

  /// Option to fix the link, used for ground link
  bool is_fixed_;
  gtsam::Pose3 fixed_pose_;

  /// Joints connected to the link
  std::vector<JointSharedPtr> joints_;

 public:
  Link() {}

  /**
   * @brief Construct a new Link object.
   * 
   * @param id Link ID
   * @param name The name of the link as defined in the SDF/URDF file.
   * @param mass The mass of the link.
   * @param inertia The inertial matrix of the link.
   * @param bMcom The pose of the link CoM relative to the base frame.
   * @param bMlink The pose of the link frame relative to the base frame.
   * @param is_fixed Flag indicating if the link is fixed.
   */
  Link(uint8_t id, const std::string &name, const double mass,
       const gtsam::Matrix3 &inertia, const gtsam::Pose3 &bMcom, const gtsam::Pose3 &bMlink, bool is_fixed = false)
      : id_(id),
        name_(name),
        mass_(mass),
        inertia_(inertia),
        bMcom_(bMcom),
        bMlink_(bMlink),
        is_fixed_(is_fixed) {}

  /** destructor */
  virtual ~Link() = default;

  bool operator==(const Link &other) const {
    return (this->name_ == other.name_ && this->id_ == other.id_ &&
            this->mass_ == other.mass_ &&
            this->centerOfMass_.equals(other.centerOfMass_) &&
            this->inertia_ == other.inertia_ &&
            this->bMcom_.equals(other.bMcom_) &&
            this->bMlink_.equals(other.bMlink_) &&
            this->is_fixed_ == other.is_fixed_ &&
            this->fixed_pose_.equals(other.fixed_pose_));
  }

  bool operator!=(const Link &other) const { return !(*this == other); }

  /// return a shared pointer of the link
  LinkSharedPtr shared(void) { return shared_from_this(); }

  /// remove the joint
  void removeJoint(const JointSharedPtr &joint) {
    joints_.erase(std::remove(joints_.begin(), joints_.end(), joint));
  }

  /// return ID of the link
  uint8_t id() const { return id_; }

  /// add joint to the link
  void addJoint(const JointSharedPtr &joint) { joints_.push_back(joint); }

  /// Relative pose at rest from link’s COM to the base frame.
  inline const gtsam::Pose3 bMcom() const { return bMcom_; }

    /// Relative pose at rest from link frame to the base frame. mainly for interoperability uses
  inline const gtsam::Pose3 bMlink() const { return bMlink_; }

  /// the fixed pose of the link
  const gtsam::Pose3 &getFixedPose() const { return fixed_pose_; }

  /// whether the link is fixed
  bool isFixed() const { return is_fixed_; }

  /// fix the link to fixed_pose. If fixed_pose is not specified, use bTcom.
  void fix(const boost::optional<gtsam::Pose3 &> fixed_pose = boost::none) {
    is_fixed_ = true;
    fixed_pose_ = fixed_pose ? *fixed_pose : bMcom();
  }

  /// Unfix the link
  void unfix() { is_fixed_ = false; }

  /// return all joints of the link
  const std::vector<JointSharedPtr> &joints() const { return joints_; }

  /// return the number of connected joints
  size_t numJoints() const { return joints_.size(); }

  /// Return link name.
  const std::string &name() const { return name_; }

  /// Return link mass.
  double mass() const { return mass_; }

  /// Return center of mass (gtsam::Pose3)
  const gtsam::Pose3 &centerOfMass() const { return centerOfMass_; }

  /// Return inertia.
  const gtsam::Matrix3 &inertia() const { return inertia_; }

  /// Return general mass gtsam::Matrix
  gtsam::Matrix6 inertiaMatrix() const {
    std::vector<gtsam::Matrix> gmm;
    gmm.push_back(inertia_);
    gmm.push_back(gtsam::I_3x3 * mass_);
    return gtsam::diag(gmm);
  }

  /// Print to ostream
  friend std::ostream &operator<<(std::ostream &os, const Link &l) {
    os << l.name();
    return os;
  }

  /// Helper print function
  void print(const std::string &s = "") const {
    std::cout << (s.empty() ? s : s + " ") << *this;
  }
};
}  // namespace gtdynamics
