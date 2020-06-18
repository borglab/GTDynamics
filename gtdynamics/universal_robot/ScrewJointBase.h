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
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/TwistAccelFactor.h"
#include "gtdynamics/factors/TwistFactor.h"
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
class ScrewJointBase : public Joint {
  using Pose3 = gtsam::Pose3;

 public:
 /**
   * This struct contains all parameters needed to construct a joint.
   * TODO (stephanie): Make sure all parameters have meaningful default values.
   */
  struct Parameters {
    Joint::JointEffortType effort_type = Joint::JointEffortType::Actuated;
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

  static Parameters ParametersFromSDF(const sdf::Joint &sdf_joint)
  {
    Parameters parameters;
    // TODO (stephanie): make this work

    parameters.joint_lower_limit = sdf_joint.Axis()->Lower();
    parameters.joint_upper_limit = sdf_joint.Axis()->Upper();
    parameters.damping_coefficient = sdf_joint.Axis()->Damping();

    // parameters.spring_coefficient = sdf_joint.Axis()->SpringReference() or SpringStiffness()? ; // spring_coeff_(spring_coefficient),
    parameters.velocity_limit = sdf_joint.Axis()->MaxVelocity();
    // acceleration_limit_(acceleration_limit),                     // No matching function? (/usr/include/sdformat-8.7/sdf)
    parameters.torque_limit = sdf_joint.Axis()->Effort();

    return parameters;
  }

 protected:
  // Joint parameters struct.
  Parameters parameters_;

  gtsam::Vector3 axis_;

  // Screw axis in parent and child COM frames.
  gtsam::Vector6 pScrewAxis_;
  gtsam::Vector6 cScrewAxis_;

  /// Return transform of child link com frame w.r.t parent link com frame
  Pose3 pMcCom(boost::optional<double> q = boost::none) const {
    return q ? pMccom_ * Pose3::Expmap(cScrewAxis_ * (*q)) : pMccom_;
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  Pose3 cMpCom(boost::optional<double> q = boost::none) const {
    return pMcCom(q).inverse();
  }

  /// Return the joint axis in the joint frame. Rotational axis for revolute and
  /// translation direction for prismatic in the joint frame.
  const gtsam::Vector3 &axis() const { return axis_; }

 public:
  /** constructor using Parameters, joint name, wTj, screw axes, and parent 
   * and child links. */
  ScrewJointBase(const std::string &name, const gtsam::Pose3 &wTj,
                 const LinkSharedPtr &parent_link, const LinkSharedPtr &child_link,
                 const Parameters &parameters, gtsam::Vector3 axis,
                 gtsam::Vector6 jScrewAxis)
      : Joint(name, wTj, parent_link, child_link),
        parameters_(parameters),
        axis_(axis),
        pScrewAxis_(-jTpcom_.inverse().AdjointMap() * jScrewAxis),
        cScrewAxis_(jTccom_.inverse().AdjointMap() * jScrewAxis) {}

  /** constructor using Parameters, sdf_joint, screw axes, and parent 
   * and child links. */
  ScrewJointBase(const sdf::Joint &sdf_joint, const LinkSharedPtr &parent_link,
                 const LinkSharedPtr &child_link, const Parameters &parameters,
                 gtsam::Vector3 axis, gtsam::Vector6 jScrewAxis)
      : Joint(sdf_joint, parent_link, child_link),
        parameters_(parameters),
        axis_(axis),
        pScrewAxis_(-jTpcom_.inverse().AdjointMap() * jScrewAxis),
        cScrewAxis_(jTccom_.inverse().AdjointMap() * jScrewAxis) {}

  /**
   * @brief Create ScrewJointBase from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                       sdf::Joint object.
   * @param[in] jScrewAxis                      Screw axis in joint frame.
   * @param[in] parent_link                     Shared pointer to the parent Link.
   * @param[in] child_link                      Shared pointer to the child Link.
   */
  ScrewJointBase(const sdf::Joint &sdf_joint, const LinkSharedPtr &parent_link,
                 const LinkSharedPtr &child_link, gtsam::Vector6 jScrewAxis)
      : ScrewJointBase(sdf_joint, parent_link, child_link,
                       ParametersFromSDF(sdf_joint), getSdfAxis(sdf_joint),
                       jScrewAxis) {}

  /** Construct joint using sdf::Joint instance and joint parameters. */
  ScrewJointBase(const sdf::Joint &sdf_joint, LinkSharedPtr parent_link,
                 LinkSharedPtr child_link, const Parameters &parameters,
                 gtsam::Vector6 jScrewAxis)
      : ScrewJointBase(sdf_joint, parent_link, child_link, parameters,
                       getSdfAxis(sdf_joint), jScrewAxis) {}

  /// Return joint effort type
  JointEffortType jointEffortType() const { return parameters_.effort_type; }

  /// Return screw axis expressed in the specified link frame
  const gtsam::Vector6 &screwAxis(const LinkSharedPtr link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

  /// Return the transform from this link com to the other link com frame
  Pose3 transformFrom(const LinkSharedPtr link,
                      boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? pMcCom(q) : cMpCom(q);
  }

  /// Return the twist of the other link given this link's twist and
  /// joint angle.
  gtsam::Vector6 transformTwistFrom(
      const LinkSharedPtr link, boost::optional<double> q,
      boost::optional<double> q_dot,
      boost::optional<gtsam::Vector6> this_twist) const {
    double q_ = q ? *q : 0.0;
    double q_dot_ = q_dot ? *q_dot : 0.0;
    gtsam::Vector6 this_twist_ =
        this_twist ? *this_twist : gtsam::Vector6::Zero();

    return transformFrom(link, q_).AdjointMap() * this_twist_ +
           screwAxis(otherLink(link)) * q_dot_;
  }

  /// Return the transform from the other link com to this link com frame
  Pose3 transformTo(const LinkSharedPtr link,
                    boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? cMpCom(q) : pMcCom(q);
  }

  /// Return the twist of this link given the other link's twist and
  /// joint angle.
  gtsam::Vector6 transformTwistTo(
      const LinkSharedPtr link, boost::optional<double> q = boost::none,
      boost::optional<double> q_dot = boost::none,
      boost::optional<gtsam::Vector6> other_twist = boost::none) const {
    double q_ = q ? *q : 0.0;
    double q_dot_ = q_dot ? *q_dot : 0.0;
    gtsam::Vector6 other_twist_ =
        other_twist ? *other_twist : gtsam::Vector6::Zero();

    return transformTo(link, q_).AdjointMap() * other_twist_ +
           screwAxis(link) * q_dot_;
  }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return parameters_.joint_lower_limit; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return parameters_.joint_upper_limit; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const { return parameters_.joint_limit_threshold; }

  /// Return joint damping coefficient
  double dampCoefficient() const { return parameters_.damping_coefficient; }

  /// Return joint spring coefficient
  double springCoefficient() const { return parameters_.spring_coefficient; }

  /// Return joint velocity limit.
  double velocityLimit() const { return parameters_.velocity_limit; }

  /// Return joint velocity limit threshold.
  double velocityLimitThreshold() const { return parameters_.velocity_limit_threshold; }

  /// Return joint acceleration limit.
  double accelerationLimit() const { return parameters_.acceleration_limit; }

  /// Return joint acceleration limit threshold.
  double accelerationLimitThreshold() const {
    return parameters_.acceleration_limit_threshold;
  }

  /// Return joint torque limit.
  double torqueLimit() const { return parameters_.torque_limit; }

  /// Return joint torque limit threshold.
  double torqueLimitThreshold() const { return parameters_.torque_limit_threshold; }

  /// Return joint angle factors.
  gtsam::NonlinearFactorGraph qFactors(size_t t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<PoseFactor>(
        PoseKey(parent_link_->getID(), t), PoseKey(child_link_->getID(), t),
        JointAngleKey(getID(), t), opt.p_cost_model, transformTo(child_link_),
        screwAxis(child_link_));
    return graph;
  }

  /// Return joint vel factors.
  gtsam::NonlinearFactorGraph vFactors(size_t t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<TwistFactor>(
        TwistKey(parent_link_->getID(), t), TwistKey(child_link_->getID(), t),
        JointAngleKey(getID(), t), JointVelKey(getID(), t), opt.v_cost_model,
        transformTo(child_link_), screwAxis(child_link_));

    return graph;
  }

  /// Return joint accel factors.
  gtsam::NonlinearFactorGraph aFactors(size_t t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<TwistAccelFactor>(
        TwistKey(child_link_->getID(), t),
        TwistAccelKey(parent_link_->getID(), t),
        TwistAccelKey(child_link_->getID(), t), JointAngleKey(getID(), t),
        JointVelKey(getID(), t), JointAccelKey(getID(), t), opt.a_cost_model,
        transformTo(child_link_), screwAxis(child_link_));

    return graph;
  }

  /// Return linearized acceleration factors.
  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const {
    gtsam::GaussianFactorGraph graph;

    const Pose3 T_wi1 = poses.at(parent_link_->name());
    const Pose3 T_wi2 = poses.at(child_link_->name());
    const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
    const gtsam::Vector6 V_i2 = twists.at(child_link_->name());
    const gtsam::Vector6 S_i2_j = screwAxis(child_link_);
    const double v_j = joint_vels.at(name());

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
      const boost::optional<gtsam::Vector3> &planar_axis) const {
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
      size_t t, const std::map<std::string, Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const {
    gtsam::GaussianFactorGraph graph;

    const Pose3 T_wi1 = poses.at(parent_link_->name());
    const Pose3 T_wi2 = poses.at(child_link_->name());
    const Pose3 T_i2i1 = T_wi2.inverse() * T_wi1;
    // const gtsam::Vector6 V_i2 = twists.at(child_link_->name());
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
  gtsam::NonlinearFactorGraph jointLimitFactors(size_t t,
                                                const OptimizerSetting &opt) {
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
