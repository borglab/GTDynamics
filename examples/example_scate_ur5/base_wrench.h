/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file base_wrench.h
 * @author Harrison Jenkins
 *
 * The wrench a fixed-base arm puts on its mount because it moves, and a
 * factor that pushes it toward zero.
 *
 * In orbit, the arm is mounted on a spacecraft whose attitude control holds
 * the base still. Whatever wrench the arm exerts on the base is a
 * disturbance the attitude control must cancel. With no gravity, that
 * wrench is exactly the motion-induced part of the inverse dynamics: the
 * rate of change of the arm's momentum (inertial, Coriolis and centrifugal
 * terms). On Earth the mount also carries the arm's weight, but that part
 * depends on q only, does not exist in orbit, and is left out here (gravity
 * is set to zero), so the same cost works in both settings.
 *
 * BaseWrenchModel computes it with the recursive Newton-Euler algorithm on
 * the GTDynamics robot (link CoM frames, joint screw axes, masses and
 * inertias), in GTDynamics' conventions: twists and wrenches are
 * [angular; linear], expressed in link CoM frames. testBaseWrench checks it
 * against GTDynamics' own inverse dynamics (DynamicsGraph::linearSolveID).
 *
 * BaseWrenchFactor evaluates it at time tau inside interval i of the double
 * integrator, where q(tau) = q_i + tau v_i + tau^2 u_i / 2, v(tau) = v_i +
 * tau u_i and the joint acceleration is u_i, and penalizes it with a
 * Gaussian cost (sigmas in N m and N).
 *
 * The wrench is reported in the robot's base frame (the planner's world
 * frame) about its origin, the mounting interface for the UR URDFs. The
 * torque about another point p (e.g. the spacecraft's center of mass) is
 * tau + (origin - p) x f.
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "scate_init.h"
#include "scate_keys.h"

namespace scate {

/// The motion-induced wrench an arm exerts on its fixed base (gravity-free inverse dynamics).
class BaseWrenchModel {
 public:
  /**
   * @param robot fixed-base serial arm (scate_init.h loadRobot); q is
   *        indexed in chain order (jointIdsInChainOrder)
   */
  explicit BaseWrenchModel(const gtdynamics::Robot& robot) {
    const std::string base = baseLinkName(robot);
    const gtsam::Pose3 bMbase_com = robot.link(base)->bMcom();
    for (size_t id : jointIdsInChainOrder(robot)) {
      for (auto&& joint : robot.joints()) {
        if (joint->id() != id) continue;
        ChainJoint link;
        link.joint = joint;
        const auto child = joint->child();
        link.spatial_inertia = gtsam::Matrix6::Zero();
        link.spatial_inertia.topLeftCorner<3, 3>() = child->inertia();
        link.spatial_inertia.bottomRightCorner<3, 3>() =
            child->mass() * gtsam::Matrix3::Identity();
        chain_.push_back(link);
      }
    }
    if (chain_.empty()) throw std::invalid_argument("BaseWrenchModel: robot has no joints");
    // The first joint's parent must be the fixed base.
    if (chain_.front().joint->parent()->name() != base) {
      throw std::invalid_argument("BaseWrenchModel: the first joint is not on the base link");
    }
    worldTbase_com_ = bMbase_com;
  }

  /// Number of joints.
  size_t dof() const { return chain_.size(); }

  /**
   * Wrench [torque; force] the arm exerts on its base, in the base frame
   * about its origin (N m, N), for joint angles q, velocities v and
   * accelerations a, without gravity.
   */
  gtsam::Vector6 wrench(const Vector& q, const Vector& v, const Vector& a) const {
    const size_t n = chain_.size();
    if (static_cast<size_t>(q.size()) != n || static_cast<size_t>(v.size()) != n ||
        static_cast<size_t>(a.size()) != n) {
      throw std::invalid_argument("BaseWrenchModel::wrench: need one value per joint");
    }
    // Forward pass: twist and acceleration of each link CoM frame, in that frame.
    std::vector<gtsam::Pose3> parentTchild(n);
    std::vector<gtsam::Vector6> twist(n), accel(n);
    gtsam::Vector6 parent_twist = gtsam::Vector6::Zero(), parent_accel = gtsam::Vector6::Zero();
    for (size_t p = 0; p < n; ++p) {
      const gtdynamics::Joint& joint = *chain_[p].joint;
      const gtsam::Vector6& screw = joint.cScrewAxis();
      parentTchild[p] = joint.parentTchild(q(p));
      const gtsam::Matrix6 Ad_childTparent = parentTchild[p].inverse().AdjointMap();
      twist[p] = Ad_childTparent * parent_twist + screw * v(p);
      accel[p] = Ad_childTparent * parent_accel +
                 gtsam::Pose3::adjointMap(twist[p]) * screw * v(p) + screw * a(p);
      parent_twist = twist[p];
      parent_accel = accel[p];
    }
    // Backward pass: wrench each joint applies to its child link, in the child frame.
    gtsam::Vector6 transmitted = gtsam::Vector6::Zero();
    for (size_t p = n; p-- > 0;) {
      const gtsam::Matrix6& G = chain_[p].spatial_inertia;
      const gtsam::Vector6 newton_euler =
          G * accel[p] - gtsam::Pose3::adjointMap(twist[p]).transpose() * G * twist[p];
      // Wrench from the next joint on this link, moved from the next link's frame.
      const gtsam::Vector6 from_next =
          p + 1 < n ? gtsam::Vector6(parentTchild[p + 1].inverse().AdjointMap().transpose() *
                                     transmitted)
                    : gtsam::Vector6::Zero();
      transmitted = newton_euler + from_next;
    }
    // The base feels the opposite of what joint 1 applies to link 1, moved
    // from link 1's CoM frame to the base CoM frame, then to the base frame.
    const gtsam::Vector6 on_base_com =
        -parentTchild[0].inverse().AdjointMap().transpose() * transmitted;
    return worldTbase_com_.inverse().AdjointMap().transpose() * on_base_com;
  }

 private:
  /// A joint along the chain and the spatial inertia of its child link (CoM frame).
  struct ChainJoint {
    gtdynamics::JointSharedPtr joint;
    gtsam::Matrix6 spatial_inertia;  ///< diag(inertia, mass I) in [angular; linear] order
  };
  std::vector<ChainJoint> chain_;
  gtsam::Pose3 worldTbase_com_;  ///< base CoM frame in the base (world) frame
};

/**
 * Pushes the motion-induced base wrench toward zero at time tau of interval i:
 * error = W(q(tau), v(tau), u_i), with q(tau) = q_i + tau v_i + tau^2 u_i / 2
 * and v(tau) = v_i + tau u_i (the double integrator). Jacobians are central
 * differences of BaseWrenchModel::wrench.
 */
class BaseWrenchFactor : public gtsam::NoiseModelFactorN<Vector, Vector, Vector> {
  using Base = gtsam::NoiseModelFactorN<Vector, Vector, Vector>;
  std::shared_ptr<const BaseWrenchModel> model_;
  double tau_;

 public:
  /**
   * @param q_key, v_key, u_key state and control of the interval's start
   * @param model the arm's base wrench
   * @param tau time inside the interval (s)
   * @param noise sigmas of [torque (N m) x3; force (N) x3]
   */
  BaseWrenchFactor(gtsam::Key q_key, gtsam::Key v_key, gtsam::Key u_key,
                   std::shared_ptr<const BaseWrenchModel> model, double tau,
                   const gtsam::SharedNoiseModel& noise)
      : Base(noise, q_key, v_key, u_key), model_(std::move(model)), tau_(tau) {
    if (!model_) throw std::invalid_argument("BaseWrenchFactor: null model");
    if (noise->dim() != 6) throw std::invalid_argument("BaseWrenchFactor: noise must be 6D");
  }

  /// Base wrench at time tau of the interval.
  gtsam::Vector evaluateError(const Vector& q, const Vector& v, const Vector& u,
                              gtsam::OptionalMatrixType H_q = nullptr,
                              gtsam::OptionalMatrixType H_v = nullptr,
                              gtsam::OptionalMatrixType H_u = nullptr) const override {
    const gtsam::Vector6 error = wrenchAt(q, v, u);
    if (H_q) *H_q = numericalJacobian(q, v, u, 0);
    if (H_v) *H_v = numericalJacobian(q, v, u, 1);
    if (H_u) *H_u = numericalJacobian(q, v, u, 2);
    return error;
  }

  /// Deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new BaseWrenchFactor(*this));
  }

 private:
  /// The wrench at time tau, from the interval's start state and control.
  gtsam::Vector6 wrenchAt(const Vector& q, const Vector& v, const Vector& u) const {
    return model_->wrench(q + tau_ * v + 0.5 * tau_ * tau_ * u, v + tau_ * u, u);
  }

  /// Central-difference Jacobian with respect to argument 0 (q), 1 (v) or 2 (u).
  gtsam::Matrix numericalJacobian(const Vector& q, const Vector& v, const Vector& u,
                                  int argument) const {
    const double h = 1e-6;
    const long n = q.size();
    gtsam::Matrix jacobian(6, n);
    for (long j = 0; j < n; ++j) {
      Vector args_plus[3] = {q, v, u}, args_minus[3] = {q, v, u};
      args_plus[argument](j) += h;
      args_minus[argument](j) -= h;
      jacobian.col(j) = (wrenchAt(args_plus[0], args_plus[1], args_plus[2]) -
                         wrenchAt(args_minus[0], args_minus[1], args_minus[2])) /
                        (2 * h);
    }
    return jacobian;
  }
};

/// Noise model of the base-wrench cost: sigma_torque (N m) on the torque, sigma_force (N) on the force.
inline gtsam::SharedNoiseModel baseWrenchNoise(double sigma_torque, double sigma_force) {
  gtsam::Vector6 sigmas;
  sigmas << sigma_torque, sigma_torque, sigma_torque, sigma_force, sigma_force, sigma_force;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

}  // namespace scate
