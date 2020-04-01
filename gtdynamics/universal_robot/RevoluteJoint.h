/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RevoluteJoint.h
 * @brief Representation of a revolute robot joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_

#include "gtdynamics/factors/JointLimitFactor.h"
#include "gtdynamics/factors/PoseFactor.h"
#include "gtdynamics/factors/TorqueFactor.h"
#include "gtdynamics/factors/TwistAccelFactor.h"
#include "gtdynamics/factors/TwistFactor.h"
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"
#include "gtdynamics/factors/WrenchPlanarFactor.h"
#include "gtdynamics/universal_robot/Joint.h"

namespace gtdynamics {
class RevoluteJoint : public Joint {
 protected:
  char joint_type_;
  JointEffortType jointEffortType_;
  gtsam::Vector3 axis_;

  // Joint limit parameters.
  double joint_lower_limit_;
  double joint_upper_limit_;
  double joint_limit_threshold_;

  double damping_coeff_;
  double spring_coeff_;

  double velocity_limit_;
  double velocity_limit_threshold_;

  double acceleration_limit_;
  double acceleration_limit_threshold_;

  double torque_limit_;
  double torque_limit_threshold_;

  // Screw axis in parent and child COM frames.
  gtsam::Vector6 pScrewAxis_;
  gtsam::Vector6 cScrewAxis_;

  /// Return transform of child link com frame w.r.t parent link com frame
  gtsam::Pose3 pMcCom(boost::optional<double> q = boost::none) const {
    if (q)
      return pMccom_ * gtsam::Pose3::Expmap(cScrewAxis_ * (*q));
    else
      return pMccom_;
  }

  /// Return transform of parent link com frame w.r.t child link com frame
  gtsam::Pose3 cMpCom(boost::optional<double> q = boost::none) const {
    if (q)
      // return gtsam::Pose3::Expmap(screwAxis_ * (*q)).inverse() *
      //        (pMccom_.inverse());
      return pMccom_.inverse() * gtsam::Pose3::Expmap(pScrewAxis_ * (*q));
    else
      return pMccom_.inverse();
  }

  /// Return the joint axis. Rotational axis for revolute and translation
  /// direction for prismatic in the joint frame.
  const gtsam::Vector3 &axis() const { return axis_; }

  void setScrewAxis() {
    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    pScrewAxis_ = gtdynamics::unit_twist(pcomRj * -axis_,
                                         pcomRj * (-jTpcom_.translation()));
    cScrewAxis_ = gtdynamics::unit_twist(ccomRj * axis_,
                                         ccomRj * (-jTccom_.translation()));
  }

 public:
  /**
   * @brief Create RevoluteJoint from a sdf::Joint instance.
   *
   * @param[in] sdf_joint                  sdf::Joint object.
   * @param[in] joint_effort_type          Joint effort type.
   * @param[in] springCoefficient          Spring coefficient.
   * @param[in] jointLimitThreshold        Joint angle limit threshold.
   * @param[in] velocityLimitThreshold     Joint velocity limit threshold.
   * @param[in] accelerationLimit          Joint acceleration limit.
   * @param[in] accelerationLimitThreshold Joint Acceleration limit threshold.
   * @param[in] torqueLimitThreshold       Joint torque limit threshold.
   * @param[in] parent_link                Shared pointer to the parent Link.
   * @param[in] child_link                 Shared pointer to the child Link.
   */
  RevoluteJoint(const sdf::Joint &sdf_joint, JointEffortType joint_effort_type,
                double springCoefficient, double jointLimitThreshold,
                double velocityLimitThreshold, double accelerationLimit,
                double accelerationLimitThreshold, double torqueLimitThreshold,
                LinkSharedPtr parent_link, LinkSharedPtr child_link)
      : Joint(sdf_joint, parent_link, child_link),
        jointEffortType_(joint_effort_type),
        axis_(gtsam::Vector3(sdf_joint.Axis()->Xyz()[0],
                             sdf_joint.Axis()->Xyz()[1],
                             sdf_joint.Axis()->Xyz()[2])),
        joint_lower_limit_(sdf_joint.Axis()->Lower()),
        joint_upper_limit_(sdf_joint.Axis()->Upper()),
        joint_limit_threshold_(jointLimitThreshold),
        damping_coeff_(sdf_joint.Axis()->Damping()),
        spring_coeff_(springCoefficient),
        velocity_limit_(sdf_joint.Axis()->MaxVelocity()),
        velocity_limit_threshold_(velocityLimitThreshold),
        acceleration_limit_(accelerationLimit),
        acceleration_limit_threshold_(accelerationLimitThreshold),
        torque_limit_(sdf_joint.Axis()->Effort()),
        torque_limit_threshold_(torqueLimitThreshold) {
    setScrewAxis();
  }

  /** constructor using JointParams */
  explicit RevoluteJoint(const Params &params)
      : Joint(params),
        joint_type_(params.joint_type),
        jointEffortType_(params.effort_type),
        axis_(params.axis),
        joint_lower_limit_(params.joint_lower_limit),
        joint_upper_limit_(params.joint_upper_limit),
        joint_limit_threshold_(params.joint_limit_threshold) {
    setScrewAxis();
  }

  /// Return jointType
  char jointType() const { return 'R'; }

  /// Return joint effort type
  JointEffortType jointEffortType() const { return jointEffortType_; }

  /// Return the transform from this link com to the other link com frame
  gtsam::Pose3 transformFrom(const LinkSharedPtr link,
                             boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? pMcCom(q) : cMpCom(q);
  }

  /// Return the transform from the other link com to this link com frame
  gtsam::Pose3 transformTo(const LinkSharedPtr link,
                           boost::optional<double> q = boost::none) const {
    return isChildLink(link) ? cMpCom(q) : pMcCom(q);
  }

  /// Return screw axis expressed in the specified link frame
  const gtsam::Vector6 &screwAxis(const LinkSharedPtr link) const {
    return isChildLink(link) ? cScrewAxis_ : pScrewAxis_;
  }

  /// Return joint angle lower limit.
  double jointLowerLimit() const { return joint_lower_limit_; }

  /// Return joint angle upper limit.
  double jointUpperLimit() const { return joint_upper_limit_; }

  /// Return joint angle limit threshold.
  double jointLimitThreshold() const { return joint_limit_threshold_; }

  /// Return joint damping coefficient
  double dampCoefficient() const { return damping_coeff_; }

  /// Return joint spring coefficient
  double springCoefficient() const { return spring_coeff_; }

  /// Return joint velocity limit.
  double velocityLimit() const { return velocity_limit_; }

  /// Return joint velocity limit threshold.
  double velocityLimitThreshold() const { return velocity_limit_threshold_; }

  /// Return joint acceleration limit.
  double accelerationLimit() const { return acceleration_limit_; }

  /// Return joint acceleration limit threshold.
  double accelerationLimitThreshold() const {
    return acceleration_limit_threshold_;
  }

  /// Return joint torque limit.
  double torqueLimit() const { return torque_limit_; }

  /// Return joint torque limit threshold.
  double torqueLimitThreshold() const { return torque_limit_threshold_; }

  /// Return joint angle factors.
  gtsam::NonlinearFactorGraph qFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.add(PoseFactor(PoseKey(parent_link_->getID(), t),
                         PoseKey(child_link_->getID(), t),
                         JointAngleKey(getID(), t), opt.p_cost_model,
                         transformTo(child_link_), screwAxis(child_link_)));
    return graph;
  }

  /// Return joint vel factors.
  gtsam::NonlinearFactorGraph vFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.add(TwistFactor(
        TwistKey(parent_link_->getID(), t), TwistKey(child_link_->getID(), t),
        JointAngleKey(getID(), t), JointVelKey(getID(), t), opt.v_cost_model,
        transformTo(child_link_), screwAxis(child_link_)));

    return graph;
  }

  /// Return joint accel factors.
  gtsam::NonlinearFactorGraph aFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    graph.add(TwistAccelFactor(
        TwistKey(child_link_->getID(), t),
        TwistAccelKey(parent_link_->getID(), t),
        TwistAccelKey(child_link_->getID(), t), JointAngleKey(getID(), t),
        JointVelKey(getID(), t), JointAccelKey(getID(), t), opt.a_cost_model,
        transformTo(child_link_), screwAxis(child_link_)));

    return graph;
  }

  /// Return joint dynamics factors.
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const int &t, const OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const {
    gtsam::NonlinearFactorGraph graph;
    graph.add(WrenchEquivalenceFactor(
        WrenchKey(parent_link_->getID(), getID(), t),
        WrenchKey(child_link_->getID(), getID(), t), JointAngleKey(getID(), t),
        opt.f_cost_model, transformTo(child_link_), screwAxis(child_link_)));
    graph.add(TorqueFactor(WrenchKey(child_link_->getID(), getID(), t),
                           TorqueKey(getID(), t), opt.t_cost_model,
                           screwAxis(child_link_)));
    if (planar_axis)
      graph.add(WrenchPlanarFactor(WrenchKey(child_link_->getID(), getID(), t),
                                   opt.planar_cost_model, *planar_axis));
    return graph;
  }

  // Return joint limit factors.
  gtsam::NonlinearFactorGraph jointLimitFactors(const int &t,
                                                const OptimizerSetting &opt) {
    gtsam::NonlinearFactorGraph graph;
    // Add joint angle limit factor.
    graph.add(JointLimitFactor(JointAngleKey(getID(), t), opt.jl_cost_model,
                               jointLowerLimit(), jointUpperLimit(),
                               jointLimitThreshold()));

    // Add joint velocity limit factors.
    graph.add(JointLimitFactor(JointVelKey(getID(), t), opt.jl_cost_model,
                               -velocityLimit(), velocityLimit(),
                               velocityLimitThreshold()));

    // Add joint acceleration limit factors.
    graph.add(JointLimitFactor(JointAccelKey(getID(), t), opt.jl_cost_model,
                               -accelerationLimit(), accelerationLimit(),
                               accelerationLimitThreshold()));

    // Add joint torque limit factors.
    graph.add(JointLimitFactor(TorqueKey(getID(), t), opt.jl_cost_model,
                               -torqueLimit(), torqueLimit(),
                               torqueLimitThreshold()));
    return graph;
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_
