// /* ----------------------------------------------------------------------------
//  * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
//  * Atlanta, Georgia 30332-0415
//  * All Rights Reserved
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file  ScrewJointBase.h
//  * @brief Representation of screw-type robot joints. Revolute, Prismatic, and
//  *  Screw subclasses
//  * @author Frank Dellaert
//  * @author Mandy Xie
//  * @author Alejandro Escontrela
//  * @author Yetong Zhang
//  * @author Stephanie McCormick
//  * @author Gerry Chen
//  * @author Varun Agrawal
//  */

// #pragma once

// #include <gtsam/geometry/Pose3.h>

// #include <cmath>
// #include <map>
// #include <string>

// #include "gtdynamics/factors/JointLimitFactor.h"
// #include "gtdynamics/universal_robot/JointTyped.h"
// #include "gtdynamics/utils/utils.h"
// #include "gtdynamics/utils/values.h"

// namespace gtdynamics {
// /**
//  * @class ScrewJointBase is an implementation of the abstract Joint class
//  *  which represents a screw-type joint and contains all necessary factor
//  *  construction methods.
//  *  It is the base class for RevoluteJoint, PrismaticJoint, and ScrewJoint.
//  */
// class ScrewJointBase : public Joint {
//   using Pose3 = gtsam::Pose3;
//   using Vector6 = gtsam::Vector6;

//  protected:
//   gtsam::Vector3 axis_;

//   // Screw axis in parent and child CoM frames.
//   Vector6 pScrewAxis_;
//   Vector6 cScrewAxis_;

//  public:
//   /// Return transform of child link CoM frame w.r.t parent link CoM frame
//   Pose3 parentTchild(double q, gtsam::OptionalJacobian<6, 1> pMc_H_q =
//                                    boost::none) const override;

//  protected:
//   /// Return transform of parent link CoM frame w.r.t child link CoM frame
//   Pose3 childTparent(double q, gtsam::OptionalJacobian<6, 1> cMp_H_q =
//                                    boost::none) const override;

//   /**
//    * Return the joint axis in the joint frame. Rotational axis for revolute and
//    * translation direction for prismatic in the joint frame.
//    */
//   const gtsam::Vector3 &axis() const { return axis_; }

//  public:
//   /**
//    * Constructor using JointParams, joint name, bTj, screw axes,
//    * and parent and child links.
//    */
//   ScrewJointBase(uint8_t id, const std::string &name, const Pose3 &bTj,
//                  const LinkSharedPtr &parent_link,
//                  const LinkSharedPtr &child_link, const gtsam::Vector3 &axis,
//                  const Vector6 &jScrewAxis,
//                  const JointParams &parameters = JointParams())
//       : JointTyped(id, name, bTj, parent_link, child_link, parameters),
//         axis_(axis),
//         pScrewAxis_(-jMp_.inverse().AdjointMap() * jScrewAxis),
//         cScrewAxis_(jMc_.inverse().AdjointMap() * jScrewAxis) {}

//   /// Return joint type for use in reconstructing robot from JointParams.
//   Type type() const override { return Type::ScrewAxis; }

//   /// Return screw axis expressed in the specified link frame
//   const Vector6 screwAxis(const LinkSharedPtr &link) const {
//     return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
//   }

//   // inherit overloads
//   using JointTyped::poseOf;
//   using JointTyped::relativePoseOf;
//   using JointTyped::transformTwistAccelTo;
//   using JointTyped::transformTwistTo;

//   /**
//    * Return the twist of this link given the other link's twist and joint angle.
//    */
//   Vector6 transformTwistTo(
//       const LinkSharedPtr &link, double q, double q_dot,
//       boost::optional<Vector6> other_twist = boost::none,
//       gtsam::OptionalJacobian<6, 1> H_q = boost::none,
//       gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
//       gtsam::OptionalJacobian<6, 6> H_other_twist = boost::none) const override;

//   /**
//    * Return the twist acceleration of this link given the other link's twist
//    * acceleration, twist, and joint angle and this link's twist.
//    */
//   Vector6 transformTwistAccelTo(
//       const LinkSharedPtr &link, double q, double q_dot, double q_ddot,
//       boost::optional<Vector6> this_twist = boost::none,
//       boost::optional<Vector6> other_twist_accel = boost::none,
//       gtsam::OptionalJacobian<6, 1> H_q = boost::none,
//       gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
//       gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
//       gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
//       gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
//           boost::none) const override;

//   JointTorque transformWrenchToTorque(
//       const LinkSharedPtr &link, boost::optional<Vector6> wrench = boost::none,
//       gtsam::OptionalJacobian<1, 6> H_wrench = boost::none) const override;

//   // TODO(frank): document and possibly eliminate
//   gtsam::Matrix6 AdjointMapJacobianJointAngle(const LinkSharedPtr &link,
//                                               double q) const override {
//     return AdjointMapJacobianQ(q, relativePoseOf(otherLink(link), 0),
//                                screwAxis(link));
//   }

//   /// Return forward dynamics priors on torque.
//   gtsam::GaussianFactorGraph linearFDPriors(
//       size_t t, const gtsam::Values &known_values,
//       const OptimizerSetting &opt) const override;

//   /// Return linearized acceleration factors.
//   gtsam::GaussianFactorGraph linearAFactors(
//       size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
//       const boost::optional<gtsam::Vector3> &planar_axis) const override;

//   /// Return linearized dynamics factors.
//   gtsam::GaussianFactorGraph linearDynamicsFactors(
//       size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
//       const boost::optional<gtsam::Vector3> &planar_axis) const override;

//   /// Return joint limit factors.
//   gtsam::NonlinearFactorGraph jointLimitFactors(
//       size_t t, const OptimizerSetting &opt) const override;

//   /// Joint-induced twist in child frame
//   gtsam::Vector6 childTwist(double q_dot) const override {
//     return cScrewAxis_ * q_dot;
//   }

//   /// Joint-induced twist in parent frame
//   gtsam::Vector6 parentTwist(double q_dot) const override {
//     return pScrewAxis_ * q_dot;
//   }

//   /// Helper function for overloading stream operator
//   virtual std::ostream &to_stream(std::ostream &os) const override;
// };

// }  // namespace gtdynamics
