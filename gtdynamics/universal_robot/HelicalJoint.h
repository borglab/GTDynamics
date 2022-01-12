/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  HelicalJoint.h
 * @brief Representation of screw joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#pragma once

#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {

/**
 * @class HelicalJoint is an implementation of the Joint class which represents
 * a helical joint and contains all necessary factor construction methods.
 */
class HelicalJoint : public Joint {
 protected:
  /**
   * Returns the screw axis in the joint frame given the joint axis and thread
   * pitch.
   */
  gtsam::Vector6 getScrewAxis(const gtsam::Vector3 &axis, double thread_pitch) {
    gtsam::Vector6 screw_axis;
    screw_axis << axis, axis * thread_pitch / 2 / M_PI;
    return screw_axis;
  }

 public:
  /// Constructor for serialization
  HelicalJoint() {}

  /**
   * @brief Create HelicalJoint using JointParams, joint name, joint pose in
   * world frame, screw axes, and parent and child links.
   *
   * @param[in] id            id for keys
   * @param[in] name          Name of the joint
   * @param[in] bTj           joint pose expressed in base frame
   * @param[in] parent_link   Shared pointer to the parent Link.
   * @param[in] child_link    Shared pointer to the child Link.
   * @param[in] axis          joint axis expressed in joint frame
   * @param[in] thread_pitch  joint's thread pitch in dist per rev
   * @param[in] parameters    JointParams struct.
   */
  HelicalJoint(uint8_t id, const std::string &name, const gtsam::Pose3 &bTj,
               const LinkSharedPtr &parent_link,
               const LinkSharedPtr &child_link, const gtsam::Vector3 &axis,
               double thread_pitch,
               const JointParams &parameters = JointParams())
      : Joint(id, name, bTj, parent_link, child_link,
              getScrewAxis(axis, thread_pitch), parameters) {}

  /// Constructor directly from screwAxis
  using Joint::Joint;

  /// Return joint type for use in reconstructing robot from JointParams.
  Type type() const final override { return Type::Screw; }

 private:
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Joint);
  }

  /// @}
};

}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::HelicalJoint>
    : public Testable<gtdynamics::HelicalJoint> {};

}  // namespace gtsam
