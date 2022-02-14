/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Joint.cpp
 * @brief Absract representation of a robot joint.
 */

#include "gtdynamics/universal_robot/Joint.h"

#include <gtsam/slam/expressions.h>

#include <iostream>

#include "gtdynamics/factors/JointLimitFactor.h"
#include "gtdynamics/universal_robot/Link.h"

using gtsam::Pose3;
using gtsam::Vector6;

namespace gtdynamics {

/* ************************************************************************* */
Joint::Joint(uint8_t id, const std::string &name, const Pose3 &bTj,
             const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
             const Vector6 &jScrewAxis, const JointParams &parameters)
    : id_(id),
      name_(name),
      parent_link_(parent_link),
      child_link_(child_link),
      jMp_(bTj.inverse() * parent_link->bMcom()),
      jMc_(bTj.inverse() * child_link->bMcom()),
      pScrewAxis_(-jMp_.inverse().AdjointMap() * jScrewAxis),
      cScrewAxis_(jMc_.inverse().AdjointMap() * jScrewAxis),
      parameters_(parameters) {}

/* ************************************************************************* */
bool Joint::isChildLink(const LinkSharedPtr &link) const {
  if (link != child_link_ && link != parent_link_)
    throw std::runtime_error("link " + link->name() +
                             " is not connected to this joint " + name_);
  return link == child_link_;
}

/* ************************************************************************* */
Pose3 Joint::parentTchild(double q,
                          gtsam::OptionalJacobian<6, 1> pTc_H_q) const {
  // Multiply screw axis with joint angle to get a finite 6D screw.
  const Vector6 screw = cScrewAxis_ * q;

  // Calculate the actual relative pose taking into account the joint angle.
  // TODO(dellaert): use formula `pMj_ * screw_around_Z * jMc_`.
  gtsam::Matrix6 exp_H_screw;
  const Pose3 exp = Pose3::Expmap(screw, pTc_H_q ? &exp_H_screw : 0);
  if (pTc_H_q) {
    *pTc_H_q = exp_H_screw * cScrewAxis_;
  }
  return pMc() * exp;  // Note: derivative of compose in exp is identity.
}

/* ************************************************************************* */
Pose3 Joint::childTparent(double q,
                          gtsam::OptionalJacobian<6, 1> cTp_H_q) const {
  // TODO(frank): don't go via inverse, specialize in base class
  Vector6 pTc_H_q;
  Pose3 pTc = parentTchild(q, cTp_H_q ? &pTc_H_q : 0);  // pTc(q) ->  pTc_H_q
  gtsam::Matrix6 cTp_H_pTc;
  Pose3 cTp = pTc.inverse(cTp_H_q ? &cTp_H_pTc : 0);  // cTp(pTc) ->  cTp_H_pTc
  if (cTp_H_q) {
    *cTp_H_q = cTp_H_pTc * pTc_H_q;
  }
  return cTp;
}

/* ************************************************************************* */
Vector6 Joint::transformTwistTo(
    const LinkSharedPtr &link, double q, double q_dot,
    boost::optional<Vector6> other_twist, gtsam::OptionalJacobian<6, 1> H_q,
    gtsam::OptionalJacobian<6, 1> H_q_dot,
    gtsam::OptionalJacobian<6, 6> H_other_twist) const {
  Vector6 other_twist_ = other_twist ? *other_twist : Vector6::Zero();

  auto other = otherLink(link);
  auto this_ad_other = relativePoseOf(other, q).AdjointMap();

  if (H_q) {
    // TODO(frank): really, zero below? Check derivatives
    *H_q = AdjointMapJacobianQ(q, relativePoseOf(other, 0.0), screwAxis(link)) *
           other_twist_;
  }
  if (H_q_dot) {
    *H_q_dot = screwAxis(link);
  }
  if (H_other_twist) {
    *H_other_twist = this_ad_other;
  }

  return this_ad_other * other_twist_ + screwAxis(link) * q_dot;
}

/* ************************************************************************* */
Vector6 Joint::transformWrenchCoordinate(
    const LinkSharedPtr &link, double q, const gtsam::Vector6 &wrench,
    gtsam::OptionalJacobian<6, 1> H_q,
    gtsam::OptionalJacobian<6, 6> H_wrench) const {
  auto other = otherLink(link);
  gtsam::Pose3 T_21 = relativePoseOf(other, q);
  gtsam::Matrix6 Ad_21_T = T_21.AdjointMap().transpose();
  gtsam::Vector6 transformed_wrench = Ad_21_T * wrench;

  if (H_wrench) {
    *H_wrench = Ad_21_T;
  }
  if (H_q) {
    // TODO(frank): really, child? Double-check derivatives
    *H_q = AdjointMapJacobianQ(q, relativePoseOf(other, 0.0), screwAxis(link))
               .transpose() *
           wrench;
  }
  return transformed_wrench;
}

/* ************************************************************************* */
double Joint::transformWrenchToTorque(
    const LinkSharedPtr &link, boost::optional<Vector6> wrench,
    gtsam::OptionalJacobian<1, 6> H_wrench) const {
  auto screw_axis_ = screwAxis(link);
  if (H_wrench) {
    *H_wrench = screw_axis_.transpose();
  }
  return screw_axis_.transpose() * (wrench ? *wrench : Vector6::Zero());
}

/* ************************************************************************* */
gtsam::GaussianFactorGraph Joint::linearFDPriors(
    size_t t, const gtsam::Values &known_values,
    const OptimizerSetting &opt) const {
  gtsam::GaussianFactorGraph priors;
  gtsam::Vector1 rhs(Torque(known_values, id(), t));
  // TODO(alej`andro): use optimizer settings
  priors.add(TorqueKey(id(), t), gtsam::I_1x1, rhs,
             gtsam::noiseModel::Constrained::All(1));
  return priors;
}

/* ************************************************************************* */
gtsam::GaussianFactorGraph Joint::linearAFactors(
    size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const {
  gtsam::GaussianFactorGraph graph;

  const Pose3 T_wi1 = Pose(known_values, parent()->id(), t);
  const Pose3 T_wi2 = Pose(known_values, child()->id(), t);
  const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
  const Vector6 V_i2 = Twist(known_values, child()->id(), t);
  const Vector6 S_i2_j = screwAxis(child_link_);
  const double v_j = JointVel(known_values, id(), t);

  // twist acceleration factor
  // A_i2 - Ad(T_21) * A_i1 - S_i2_j * a_j = ad(V_i2) * S_i2_j * v_j
  Vector6 rhs_tw = Pose3::adjointMap(V_i2) * S_i2_j * v_j;
  graph.add(TwistAccelKey(child()->id(), t), gtsam::I_6x6,
            TwistAccelKey(parent()->id(), t), -T_i2i1.AdjointMap(),
            JointAccelKey(id(), t), -S_i2_j, rhs_tw,
            gtsam::noiseModel::Constrained::All(6));

  return graph;
}

/* ************************************************************************* */
gtsam::GaussianFactorGraph Joint::linearDynamicsFactors(
    size_t t, const gtsam::Values &known_values, const OptimizerSetting &opt,
    const boost::optional<gtsam::Vector3> &planar_axis) const {
  gtsam::GaussianFactorGraph graph;

  const Pose3 T_wi1 = Pose(known_values, parent()->id(), t);
  const Pose3 T_wi2 = Pose(known_values, child()->id(), t);
  const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
  const Vector6 S_i2_j = screwAxis(child_link_);

  // torque factor
  // S_i_j^T * F_i_j - tau = 0
  gtsam::Vector1 rhs_torque = gtsam::Vector1::Zero();
  graph.add(WrenchKey(child()->id(), id(), t), S_i2_j.transpose(),
            TorqueKey(id(), t), -gtsam::I_1x1, rhs_torque,
            gtsam::noiseModel::Constrained::All(1));

  // wrench equivalence factor
  // F_i1_j + Ad(T_i2i1)^T F_i2_j = 0
  Vector6 rhs_weq = Vector6::Zero();
  graph.add(WrenchKey(parent()->id(), id(), t), gtsam::I_6x6,
            WrenchKey(child()->id(), id(), t), T_i2i1.AdjointMap().transpose(),
            rhs_weq, gtsam::noiseModel::Constrained::All(6));

  // wrench planar factor
  if (planar_axis) {
    gtsam::Matrix36 J_wrench = getPlanarJacobian(*planar_axis);
    graph.add(WrenchKey(child()->id(), id(), t), J_wrench,
              gtsam::Vector3::Zero(), gtsam::noiseModel::Constrained::All(3));
  }

  return graph;
}

/* ************************************************************************* */
Vector6 Joint::childTwist(double q_dot) const { return cScrewAxis_ * q_dot; }

/* ************************************************************************* */
Vector6 Joint::parentTwist(double q_dot) const { return pScrewAxis_ * q_dot; }

/* ************************************************************************* */
gtsam::NonlinearFactorGraph Joint::jointLimitFactors(
    size_t t, const OptimizerSetting &opt) const {
  gtsam::NonlinearFactorGraph graph;
  auto id = this->id();
  // Add joint angle limit factor.
  graph.emplace_shared<JointLimitFactor>(
      JointAngleKey(id, t), opt.jl_cost_model,
      parameters().scalar_limits.value_lower_limit,
      parameters().scalar_limits.value_upper_limit,
      parameters().scalar_limits.value_limit_threshold);

  // Add joint velocity limit factors.
  graph.emplace_shared<JointLimitFactor>(
      JointVelKey(id, t), opt.jl_cost_model, -parameters().velocity_limit,
      parameters().velocity_limit, parameters().velocity_limit_threshold);

  // Add joint acceleration limit factors.
  graph.emplace_shared<JointLimitFactor>(
      JointAccelKey(id, t), opt.jl_cost_model, -parameters().acceleration_limit,
      parameters().acceleration_limit,
      parameters().acceleration_limit_threshold);

  // Add joint torque limit factors.
  graph.emplace_shared<JointLimitFactor>(
      TorqueKey(id, t), opt.jl_cost_model, -parameters().torque_limit,
      parameters().torque_limit, parameters().torque_limit_threshold);
  return graph;
}

/* ************************************************************************* */
std::ostream &Joint::to_stream(std::ostream &os) const {
  os << name_ << "\n\tid=" << size_t(id_)
     << "\n\tparent link: " << parent()->name()
     << "\n\tchild link: " << child()->name()
     << "\n\tscrew axis (parent): " << screwAxis(parent()).transpose();
  return os;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Joint &j) {
  // Delegate printing responsibility to member function so we can override.
  return j.to_stream(os);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const JointSharedPtr &j) {
  return j->to_stream(os);
}

/* ************************************************************************* */
gtsam::Vector6_ Joint::poseConstraint(uint64_t t) const {
  using gtsam::Pose3_;

  // Get an expression for parent pose.
  Pose3_ wTp(PoseKey(parent()->id(), t));
  Pose3_ wTc(PoseKey(child()->id(), t));
  gtsam::Double_ q(JointAngleKey(id(), t));

  // Compute the expected pose of the child link.
  Pose3_ pTc(std::bind(&Joint::parentTchild, this, std::placeholders::_1,
                       std::placeholders::_2),
             q);
  Pose3_ wTc_hat = wTp * pTc;

  // Return the error in tangent space
  return gtsam::logmap(wTc, wTc_hat);
}

/* ************************************************************************* */
gtsam::Vector6_ Joint::twistConstraint(uint64_t t) const {
  gtsam::Vector6_ twist_p(TwistKey(parent()->id(), t));
  gtsam::Vector6_ twist_c(TwistKey(child()->id(), t));
  gtsam::Double_ q(JointAngleKey(id(), t));
  gtsam::Double_ qVel(JointVelKey(id(), t));

  gtsam::Vector6_ twist_c_hat(
      std::bind(&Joint::transformTwistTo, this, child(), std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5,
                std::placeholders::_6),
      q, qVel, twist_p);

  return twist_c_hat - twist_c;
}

/* ************************************************************************* */
gtsam::Vector6_ Joint::twistAccelConstraint(uint64_t t) const {
  gtsam::Vector6_ twist_c(TwistKey(child()->id(), t));
  gtsam::Vector6_ twistAccel_p(TwistAccelKey(parent()->id(), t));
  gtsam::Vector6_ twistAccel_c(TwistAccelKey(child()->id(), t));
  gtsam::Double_ q(JointAngleKey(id(), t));
  gtsam::Double_ qVel(JointVelKey(id(), t));
  gtsam::Double_ qAccel(JointAccelKey(id(), t));

  /// The following 2 lambda functions computes the expected twist acceleration
  /// of the child link. (Note: we split it into 2 functions because the
  /// expression constructor currently only support up to tenary expression.)
  auto transformTwistAccelTo1 =
      [this](double q, const Vector6 &other_twist_accel,
             gtsam::OptionalJacobian<6, 1> H_q,
             gtsam::OptionalJacobian<6, 6> H_other_twist_accel) {
        Pose3 jTi = relativePoseOf(parent(), q);
        Vector6 this_twist_accel = jTi.AdjointMap() * other_twist_accel;

        if (H_other_twist_accel) {
          *H_other_twist_accel = jTi.AdjointMap();
        }
        if (H_q) {
          // TODO(frank): really, zero below? Check derivatives. Also,
          // copy/pasta from above?
          *H_q = AdjointMapJacobianQ(q, relativePoseOf(parent(), 0.0),
                                     cScrewAxis_) *
                 other_twist_accel;
        }
        return this_twist_accel;
      };
  gtsam::Vector6_ twistAccel_c_hat1(transformTwistAccelTo1, q, twistAccel_p);

  auto transformTwistAccelTo2 =
      [this](double q_dot, double q_ddot, const Vector6 &this_twist,
             gtsam::OptionalJacobian<6, 1> H_q_dot,
             gtsam::OptionalJacobian<6, 1> H_q_ddot,
             gtsam::OptionalJacobian<6, 6> H_this_twist) {
        Vector6 this_twist_accel =
            Pose3::adjoint(this_twist, cScrewAxis_ * q_dot, H_this_twist) +
            cScrewAxis_ * q_ddot;
        if (H_q_dot) {
          *H_q_dot = Pose3::adjointMap(this_twist) * cScrewAxis_;
        }
        if (H_q_ddot) {
          *H_q_ddot = cScrewAxis_;
        }
        return this_twist_accel;
      };
  gtsam::Vector6_ twistAccel_c_hat2(transformTwistAccelTo2, qVel, qAccel,
                                    twist_c);

  return twistAccel_c_hat1 + twistAccel_c_hat2 - twistAccel_c;
}

/* ************************************************************************* */
gtsam::Vector6_ Joint::wrenchEquivalenceConstraint(uint64_t t) const {
  gtsam::Vector6_ wrench_p(WrenchKey(parent()->id(), id(), t));
  gtsam::Vector6_ wrench_c(WrenchKey(child()->id(), id(), t));
  gtsam::Double_ q(JointAngleKey(id(), t));

  gtsam::Vector6_ wrench_c_hat(
      std::bind(&Joint::transformWrenchCoordinate, this, child(),
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4),
      q, wrench_c);

  return wrench_p + wrench_c_hat;
}

/* ************************************************************************* */
gtsam::Double_ Joint::torqueConstraint(uint64_t t) const {
  using gtsam::Pose3_;

  gtsam::Double_ torque(TorqueKey(id(), t));
  gtsam::Vector6_ wrench(WrenchKey(child()->id(), id(), t));
  gtsam::Double_ torque_hat(
      std::bind(&Joint::transformWrenchToTorque, this, child(),
                std::placeholders::_1, std::placeholders::_2),
      wrench);

  return torque_hat - torque;
}

}  // namespace gtdynamics
