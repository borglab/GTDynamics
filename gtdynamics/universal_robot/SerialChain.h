/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SerialChain.h
 * @brief Serial chain kinematics as a manifold. By specifying the joint angles,
 * we can compute poses of all the links.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/nonlinear/Expression.h>

namespace gtsam {

template <int P>
/** Kinematic serial chain manifold, with the basis variables as the joint
   angles. */
class SerialChain {
 protected:
  boost::shared_ptr<gtdynamics::Robot> robot_;
  std::string base_name_;  // name of base link
  boost::shared_ptr<Values> values_;

 public:
  using VectorP = Eigen::Matrix<double, P, 1>;
  enum { dimension = P };

  /** Constructor.
   * @param robot robot representing serial chain
   * @param base_name name of base link
   * @param base_pose pose of base link
   * @param joint_angles angles of all the joints.
   */
  SerialChain(const boost::shared_ptr<gtdynamics::Robot>& robot,
              const std::string& base_name, const Pose3& base_pose,
              const Values& joint_angles)
      : robot_(robot),
        base_name_(base_name),
        values_(boost::make_shared<Values>(joint_angles)) {
    values_ = boost::make_shared<Values>(
        robot_->forwardKinematics(*values_, 0, base_name_));
  }

  /** Constructor with new joint angles. */
  SerialChain<P> createWithNewAngles(const Values& new_joint_angles) const {
    return SerialChain(
        robot_, base_name_,
        gtdynamics::Pose(*values_, robot_->link(base_name_)->id()),
        new_joint_angles);
  }

  /** return joint angle with Jacobian given joint id. */
  double joint_by_id(const size_t& id,
                     OptionalJacobian<1, P> H = boost::none) const {
    if (H) {
      H->setZero();
      (*H)(id - 1) = 1;
    }
    return gtdynamics::JointAngle(*values_, id);
  }

  /** return joint angle with Jacobian given joint name. */
  double joint(const std::string& name,
               OptionalJacobian<1, P> H = boost::none) const {
    return joint_by_id(robot_->joint(name)->id(), H);
  }

  /** return link pose with Jacobian given link name. */
  Pose3 link_pose(const std::string& name,
                  OptionalJacobian<6, P> H = boost::none) const {
    Pose3 wTe = gtdynamics::Pose(*values_, robot_->link(name)->id());

    if (H) {
      auto parent_link = robot_->link(base_name_);
      auto joint = parent_link->joints().at(0);
      // find all joints between base and this link
      while (parent_link->name() != name) {
        auto child_link = joint->otherLink(parent_link);
        auto wTp = gtdynamics::Pose(*values_, parent_link->id());
        auto wTc = gtdynamics::Pose(*values_, child_link->id());

        double q = gtdynamics::JointAngle(*values_, joint->id());
        Matrix J_wTc_q;
        joint->poseOf(child_link, wTp, q, boost::none, J_wTc_q);

        if (child_link->name() == name) {
          (*H).col(joint->id() - 1) = J_wTc_q;
          break;
        } else {
          Pose3 cTe = wTc.inverse() * (wTe);
          Matrix H_wTc;
          wTc.compose(cTe, H_wTc, boost::none);
          (*H).col(joint->id() - 1) = H_wTc * J_wTc_q;
        }

        // next link and joint
        parent_link = child_link;
        if (joint == child_link->joints().at(0)) {
          joint = child_link->joints().at(1);
        } else {
          joint = child_link->joints().at(0);
        }
      }
    }

    return wTe;
  }

  /// retraction with optional derivatives.
  SerialChain retract(const VectorP& v,  // TODO: use xi
                      OptionalJacobian<P, P> H1 = boost::none,
                      OptionalJacobian<P, P> H2 = boost::none) const {
    if (H1) {
      *H1 = Matrix::Identity(P, P);
    }
    if (H2) {
      *H2 = Matrix::Identity(P, P);
    }
    Values new_joint_angles;
    for (const auto& joint : robot_->joints()) {
      double q = gtdynamics::JointAngle(*values_, joint->id());
      q += v(joint->id() - 1);
      gtdynamics::InsertJointAngle(&new_joint_angles, joint->id(), q);
    }
    return createWithNewAngles(new_joint_angles);
  }

  /// localCoordinates with optional derivatives.
  VectorP localCoordinates(const SerialChain& g,
                           OptionalJacobian<P, P> H1 = boost::none,
                           OptionalJacobian<P, P> H2 = boost::none) const {
    if (H1) {
      *H1 = -Matrix::Identity(P, P);
    }
    if (H2) {
      *H2 = Matrix::Identity(P, P);
    }
    VectorP v;
    for (const auto& joint : robot_->joints()) {
      double q1 = gtdynamics::JointAngle(*values_, joint->id());
      double q2 = gtdynamics::JointAngle(*(g.values_), joint->id());
      v(joint->id() - 1) = q2 - q1;
    }
    return v;
  }

  /// print
  void print(const std::string& s = "") const {
    for (const auto& joint : robot_->joints()) {
      std::cout << "joint: " << joint->name()
                << "\tangle: " << gtdynamics::JointAngle(*values_, joint->id())
                << "\n";
    }
  }

  /// equals
  bool equals(const SerialChain& other, double tol = 1e-8) const {
    for (const auto& joint : robot_->joints()) {
      double q1 = gtdynamics::JointAngle(*values_, joint->id());
      double q2 = gtdynamics::JointAngle(*(other.values_), joint->id());
      if (abs(q2 - q1) > 1e-8) {
        return false;
      }
    }
    return true;
  }
};

// Specialize SerialChain<P> traits to use a Retract/Local
template <int P>
struct traits<SerialChain<P>> : internal::Manifold<SerialChain<P>> {};
}  // namespace gtsam
