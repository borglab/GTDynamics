/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJointBase.h
 * @brief Representation of screw-type robot joints. Revolute, Prismatic, and
 *  Screw subclasses
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINTBASE_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINTBASE_H_

#include <cmath>
#include <map>
#include <string>

#include "gtdynamics/factors/JointLimitFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"
#include "gtdynamics/factors/WrenchPlanarFactor.h"
#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {
/**
 * @class ScrewJointBase is an implementation of the abstract Joint class
 *  which represents a screw-type joint and contains all necessary factor
 *  construction methods.
 *  It is the base class for RevoluteJoint, PrismaticJoint, and ScrewJoint.
 */
class ScrewJointBase : public JointTyped<double, double> {
  using Pose3 = gtsam::Pose3;

 public:
  /**
   * This struct contains all parameters needed to construct a joint.
   * TODO (stephanie): Make sure all parameters have meaningful default values.
   */
  struct Parameters {
    JointEffortType effort_type = JointEffortType::Actuated;

    //TODO (stephanie): replace these three parameters with ScalarLimit struct.
    double joint_lower_limit;
    double joint_upper_limit;
    double joint_limit_threshold = 0.0;

    double velocity_limit;
    double velocity_limit_threshold = 0.0;
    double acceleration_limit = 10000;
    double acceleration_limit_threshold = 0.0;
    double torque_limit;
    double torque_limit_threshold = 0.0;
    double damping_coefficient;
    double spring_coefficient = 0.0;
  };

 protected:
  // Joint parameters struct.
  Parameters parameters_;

  gtsam::Vector3 axis_;

  // Screw axis in parent and child COM frames.
  gtsam::Vector6 pScrewAxis_;
  gtsam::Vector6 cScrewAxis_;

  /// Return transform of child link com frame w.r.t parent link com frame
  Pose3 pMcCom(boost::optional<double> q = boost::none,
               gtsam::OptionalJacobian<6, 1> pMc_H_q = boost::none) const {
    if (pMc_H_q) {
      // For reference:
      // pMc = jMi * exp([S]q)
      // pMc_H_q = pMc_H_exp * exp_H_q
      //         = pMc_H_exp * exp_H_Sq * Sq_pMc_H_q
      //         = pMc_H_exp * exp_H_Sq * [S]
      gtsam::Matrix6 pMc_H_exp, exp_H_Sq;
      gtsam::Vector6 Sq = q ? static_cast<gtsam::Vector6>(cScrewAxis_ * *q) :
                              gtsam::Vector6::Zero();
      Pose3 exp = Pose3::Expmap(Sq, exp_H_Sq);
      Pose3 pMc = pMccom_.compose(exp, boost::none, pMc_H_exp);
      *pMc_H_q = pMc_H_exp * exp_H_Sq * cScrewAxis_;
      return pMc;
    } else {
      return q ? pMccom_ * Pose3::Expmap(cScrewAxis_ * (*q)) : pMccom_;
    }
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  Pose3 cMpCom(boost::optional<double> q = boost::none,
               gtsam::OptionalJacobian<6, 1> cMp_H_q = boost::none) const {
    if (cMp_H_q) {
      // For reference:
      // cMp = inverse(pMc(q))
      // cMp_H_q = cMp_H_pMc * pMc_H_q
      gtsam::Matrix6 cMp_H_pMc;
      gtsam::Vector6 pMc_H_q;
      Pose3 pMc = pMcCom(q, pMc_H_q);      // pMc(q)    ->  pMc_H_q
      Pose3 cMp = pMc.inverse(cMp_H_pMc);  // cMp(pMc)  ->  cMp_H_pMc
      *cMp_H_q = cMp_H_pMc * pMc_H_q;
      return cMp;
    } else {
      return pMcCom(q).inverse();
    }
  }

  /// Return the joint axis in the joint frame. Rotational axis for revolute and
  /// translation direction for prismatic in the joint frame.
  const gtsam::Vector3 &axis() const { return axis_; }

 public:
  /** constructor using Parameters, joint name, wTj, screw axes, and parent
   * and child links. */
  ScrewJointBase(const std::string &name, const gtsam::Pose3 &wTj,
                 const LinkSharedPtr &parent_link,
                 const LinkSharedPtr &child_link, const Parameters &parameters,
                 const gtsam::Vector3 &axis, const gtsam::Vector6 &jScrewAxis)
      : JointTyped(name, wTj, parent_link, child_link),
        parameters_(parameters),
        axis_(axis),
        pScrewAxis_(-jTpcom_.inverse().AdjointMap() * jScrewAxis),
        cScrewAxis_(jTccom_.inverse().AdjointMap() * jScrewAxis) {}

  /// Return joint effort type
  JointEffortType jointEffortType() const { return parameters_.effort_type; }

  /// Return screw axis expressed in the specified link frame
  const gtsam::Vector6 screwAxis(const LinkSharedPtr &link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

  /// Return the transform from the other link com to this link com frame
  Pose3 transformToImpl(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none) const override {
    return isChildLink(link) ? cMpCom(q, H_q) : pMcCom(q, H_q);
  }

  /// Return the twist of this link given the other link's twist and
  /// joint angle.
  gtsam::Vector6 transformTwistToImpl(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      boost::optional<double> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist =
          boost::none) const override {
    double q_ = q ? *q : 0.0;
    double q_dot_ = q_dot ? *q_dot : 0.0;
    gtsam::Vector6 other_twist_ =
        other_twist ? *other_twist : gtsam::Vector6::Zero();

    // i = other link
    // j = this link
    auto this_ad_other = transformTo(link, q_).AdjointMap();

    if (H_q) {
      *H_q = AdjointMapJacobianQ(q_, transformTo(link), screwAxis(link)) *
             other_twist_;
    }
    if (H_q_dot) {
      *H_q_dot = screwAxis(link);
    }
    if (H_other_twist) {
      *H_other_twist = this_ad_other;
    }

    return this_ad_other * other_twist_ +
           screwAxis(link) * q_dot_;
  }

  gtsam::Vector6 transformTwistAccelToImpl(
      const LinkSharedPtr &link, boost::optional<double> q = boost::none,
      boost::optional<double> q_dot = boost::none,
      boost::optional<double> q_ddot = boost::none,
      boost::optional<gtsam::Vector6> this_twist = boost::none,
      boost::optional<gtsam::Vector6> other_twist_accel = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_dot = boost::none,
      gtsam::OptionalJacobian<6, 1> H_q_ddot = boost::none,
      gtsam::OptionalJacobian<6, 6> H_this_twist = boost::none,
      gtsam::OptionalJacobian<6, 6> H_other_twist_accel =
          boost::none) const override {
    double q_dot_ = q_dot ? *q_dot : 0;
    double q_ddot_ = q_ddot ? *q_ddot : 0;
    gtsam::Vector6 this_twist_ =
        this_twist ? *this_twist : gtsam::Vector6::Zero();
    gtsam::Vector6 other_twist_accel_ =
        other_twist_accel ? *other_twist_accel : gtsam::Vector6::Zero();
    gtsam::Vector6 screw_axis_ = isChildLink(link) ? cScrewAxis_ : pScrewAxis_;

    // i = other link
    // j = this link
    gtsam::Pose3 jTi = transformTo(link, q);

    gtsam::Vector6 this_twist_accel =
        jTi.AdjointMap() * other_twist_accel_ +
        gtsam::Pose3::adjoint(this_twist_, screw_axis_ * q_dot_,
                              H_this_twist) +
        screw_axis_ * q_ddot_;

    if (H_other_twist_accel) {
      *H_other_twist_accel = jTi.AdjointMap();
    }
    if (H_q) {
      *H_q = AdjointMapJacobianQ(q ? *q : 0, transformTo(link), screw_axis_) *
             other_twist_accel_;
    }
    if (H_q_dot) {
      *H_q_dot = gtsam::Pose3::adjointMap(this_twist_) * screw_axis_;
    }
    if (H_q_ddot) {
      *H_q_ddot = screw_axis_;
    }

    return this_twist_accel;
  }

  AngleTangentType transformWrenchToTorqueImpl(
      boost::optional<gtsam::Vector6> wrench = boost::none,
      gtsam::OptionalJacobian<6, 1> H_wrench = boost::none) const override {
    return 0;  // TODO
  }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return parameters_.joint_lower_limit; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return parameters_.joint_upper_limit; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const {
    return parameters_.joint_limit_threshold;
  }

  /// Return joint damping coefficient
  double dampCoefficient() const { return parameters_.damping_coefficient; }

  /// Return joint spring coefficient
  double springCoefficient() const { return parameters_.spring_coefficient; }

  /// Return joint velocity limit.
  double velocityLimit() const { return parameters_.velocity_limit; }

  /// Return joint velocity limit threshold.
  double velocityLimitThreshold() const {
    return parameters_.velocity_limit_threshold;
  }

  /// Return joint acceleration limit.
  double accelerationLimit() const { return parameters_.acceleration_limit; }

  /// Return joint acceleration limit threshold.
  double accelerationLimitThreshold() const {
    return parameters_.acceleration_limit_threshold;
  }

  /// Return joint torque limit.
  double torqueLimit() const { return parameters_.torque_limit; }

  /// Return joint torque limit threshold.
  double torqueLimitThreshold() const {
    return parameters_.torque_limit_threshold;
  }

  /// Return forward dynamics priors on torque.
  gtsam::GaussianFactorGraph linearFDPriors(
      size_t t, const JointValues &torques,
      const OptimizerSetting &opt) const override {
    gtsam::GaussianFactorGraph priors;
    gtsam::Vector1 rhs;
    rhs << torques.at<double>(getKey());
    // TODO(alejandro): use optimizer settings
    priors.add(TorqueKey(getID(), t), gtsam::I_1x1, rhs,
               gtsam::noiseModel::Constrained::All(1));
    return priors;
  }

  /// Return linearized acceleration factors.
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const LinkPoses &poses,
      const LinkTwists &twists,
      const JointValues &joint_angles,
      const JointValues &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override {
    gtsam::GaussianFactorGraph graph;

    const Pose3 T_wi1 = poses.at(parent_link_->name());
    const Pose3 T_wi2 = poses.at(child_link_->name());
    const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
    const gtsam::Vector6 V_i2 = twists.at(child_link_->name());
    const gtsam::Vector6 S_i2_j = screwAxis(child_link_);
    const double v_j = joint_vels.at<double>(getKey());

    // twist acceleration factor
    // A_i2 - Ad(T_21) * A_i1 - S_i2_j * a_j = ad(V_i2) * S_i2_j * v_j
    gtsam::Vector6 rhs_tw = Pose3::adjointMap(V_i2) * S_i2_j * v_j;
    graph.add(TwistAccelKey(child_link_->getID(), t), gtsam::I_6x6,
              TwistAccelKey(parent_link_->getID(), t), -T_i2i1.AdjointMap(),
              JointAccelKey(getID(), t), -S_i2_j, rhs_tw,
              gtsam::noiseModel::Constrained::All(6));

    return graph;
  }

  /// Return joint dynamics factors.
  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<WrenchEquivalenceFactor>(
        WrenchKey(parent_link_->getID(), getID(), t),
        WrenchKey(child_link_->getID(), getID(), t), JointAngleKey(getID(), t),
        opt.f_cost_model, transformTo(child_link_), screwAxis(child_link_));
    graph.emplace_shared<TorqueFactor>(
        WrenchKey(child_link_->getID(), getID(), t), TorqueKey(getID(), t),
        opt.t_cost_model, screwAxis(child_link_));
    if (planar_axis)
      graph.emplace_shared<WrenchPlanarFactor>(
          WrenchKey(child_link_->getID(), getID(), t), opt.planar_cost_model,
          *planar_axis);
    return graph;
  }

  /// Return linearized dynamics factors.
  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const LinkPoses &poses,
      const LinkTwists &twists,
      const JointValues &joint_angles,
      const JointValues &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const override {
    gtsam::GaussianFactorGraph graph;

    const Pose3 T_wi1 = poses.at(parent_link_->name());
    const Pose3 T_wi2 = poses.at(child_link_->name());
    const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
    const gtsam::Vector6 S_i2_j = screwAxis(child_link_);

    // torque factor
    // S_i_j^T * F_i_j - tau = 0
    gtsam::Vector1 rhs_torque = gtsam::Vector1::Zero();
    graph.add(WrenchKey(child_link_->getID(), getID(), t), S_i2_j.transpose(),
              TorqueKey(getID(), t), -gtsam::I_1x1, rhs_torque,
              gtsam::noiseModel::Constrained::All(1));

    // wrench equivalence factor
    // F_i1_j + Ad(T_i2i1)^T F_i2_j = 0
    gtsam::Vector6 rhs_weq = gtsam::Vector6::Zero();
    graph.add(WrenchKey(parent_link_->getID(), getID(), t), gtsam::I_6x6,
              WrenchKey(child_link_->getID(), getID(), t),
              T_i2i1.AdjointMap().transpose(), rhs_weq,
              gtsam::noiseModel::Constrained::All(6));

    // wrench planar factor
    if (planar_axis) {
      gtsam::Matrix36 J_wrench = getPlanarJacobian(*planar_axis);
      graph.add(WrenchKey(child_link_->getID(), getID(), t), J_wrench,
                gtsam::Vector3::Zero(), gtsam::noiseModel::Constrained::All(3));
    }

    return graph;
  }

  /// Return joint limit factors.
  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const OptimizerSetting &opt) const override {
    gtsam::NonlinearFactorGraph graph;
    auto id = getID();
    // Add joint angle limit factor.
    graph.emplace_shared<JointLimitFactor>(
        JointAngleKey(id, t), opt.jl_cost_model, jointLowerLimit(),
        jointUpperLimit(), jointLimitThreshold());

    // Add joint velocity limit factors.
    graph.emplace_shared<JointLimitFactor>(
        JointVelKey(id, t), opt.jl_cost_model, -velocityLimit(),
        velocityLimit(), velocityLimitThreshold());

    // Add joint acceleration limit factors.
    graph.emplace_shared<JointLimitFactor>(
        JointAccelKey(id, t), opt.jl_cost_model, -accelerationLimit(),
        accelerationLimit(), accelerationLimitThreshold());

    // Add joint torque limit factors.
    graph.emplace_shared<JointLimitFactor>(TorqueKey(id, t), opt.jl_cost_model,
                                           -torqueLimit(), torqueLimit(),
                                           torqueLimitThreshold());
    return graph;
  }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINTBASE_H_
