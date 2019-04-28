/**
 * @file  link.h
 * @brief manipulator link
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace manipulator {
/* Shorthand symbol for linear factors */
/* Shorthand for T_j, for twist accelerations on the j-th link. */
extern gtsam::Symbol T(int j);
/* Shorthand for a_j, for joint accelerations on the j-th link. */
extern gtsam::Symbol a(int j);
/* Shorthand for F_j, for wrenches on the j-th link. */
extern gtsam::Symbol F(int j);
/* Shorthand for t_j, for torque on the j-th link. */
extern gtsam::Symbol t(int j);
/* Shorthand for V_j, for 6D link twist vector on the j-th link. */
extern gtsam::Symbol V(int j);
/* Shorthand for J_j, for all joint positions j. */
extern gtsam::Symbol J(int j);

/**
 * Link is the base class for links taking different format of parameters
 */
class Link {
 protected:
  char jointType_;

 private:
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;
  gtsam::Vector6 screwAxis_;
  double jointLowerLimit_;
  double jointUpperLimit_;
  double jointLimitThreshold_;
  double velocityLimit_;
  double velocityLimitThreshold_;
  double accelerationLimit_;
  double accelerationLimitThreshold_;
  double torqueLimit_;
  double torqueLimitThreshold_;

 public:
  /**
   * Construct from joint_type, mass, center_of_mass, and inertia
   * Keyword arguments:
      joint_type              -- 'R': revolute,
                                 'P': prismatic
      mass                    -- mass of link
      center_of_mass          -- center of mass location expressed
                                 in link frame
      inertia                 -- inertia matrix
      screw_axis              -- joint axis expressed in COM frame
      joint_lower_limit       -- joint angle lower limit
      joint_upper_limit       -- joint angle upper limit
      joint_limit_threshold   -- joint angle limit threshold
      velocityLimit           -- joint velocity limit
      velocityLimitThreshold  -- joint velocity limit threshold
      accelerationLimit       -- joint acceleration limit
      accelerationLimitThreshold -- joint acceleration limit threshold
      torqueLimit                -- joint torque limit
      torqueLimitThreshold       -- joint torque limit threshold
    * Note: joint angle limits are given in radians.
   */
  Link(char joint_type, double mass, const gtsam::Pose3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit = -M_PI, double joint_upper_limit = M_PI,
       double joint_limit_threshold = 0.0, double velocity_limit = 10000,
       double velocity_limit_threshold = 0.0, double acceleration_limit = 10000,
       double acceleration_limit_threshold = 0.0, double torque_limit = 10000,
       double torque_limit_threshold = 0.0)
      : jointType_(joint_type),
        mass_(mass),
        centerOfMass_(center_of_mass),
        inertia_(inertia),
        screwAxis_(screwAxis),
        jointLowerLimit_(joint_lower_limit),
        jointUpperLimit_(joint_upper_limit),
        jointLimitThreshold_(joint_limit_threshold),
        velocityLimit_(velocity_limit),
        velocityLimitThreshold_(velocity_limit_threshold),
        accelerationLimit_(acceleration_limit),
        accelerationLimitThreshold_(acceleration_limit_threshold),
        torqueLimit_(torque_limit),
        torqueLimitThreshold_(torque_limit_threshold) {}

  /**
   * Construct from joint_type, mass, center_of_mass, and inertia
   * Keyword arguments:
   * joint_type                     -- 'R': revolute,
   *                                   'P': prismatic
   * mass                           -- mass of link
   * center_of_mass                 -- center of mass location expressed
   *                                   in link frame
   * inertia                        -- principal inertias
   * screw_axis                     -- joint axis expressed in COM frame
   * joint_lower_limit              -- joint angle lower limit
   * joint_upper_limit              -- joint angle upper limit
   * joint_limit_threshold          -- joint angle limit threshold
   * velocityLimit                  -- joint velocity limit
   * velocityLimitThreshold         -- joint velocity limit threshold
   * accelerationLimit              -- joint acceleration limit
   * accelerationLimitThreshold     -- joint acceleration limit threshold
   * torqueLimit                    -- joint torque limit
   * torqueLimitThreshold           -- joint torque limit threshold
   * Note: joint angle limits are given in radians.
   */
  Link(char joint_type, double mass, const gtsam::Point3 &center_of_mass,
       const gtsam::Matrix3 &inertia, const gtsam::Vector6 &screwAxis,
       double joint_lower_limit = -M_PI, double joint_upper_limit = M_PI,
       double joint_limit_threshold = 0.0, double velocity_limit = 10000,
       double velocity_limit_threshold = 0.0, double acceleration_limit = 10000,
       double acceleration_limit_threshold = 0.0, double torque_limit = 10000,
       double torque_limit_threshold = 0.0)
      : jointType_(joint_type),
        mass_(mass),
        centerOfMass_(gtsam::Pose3(gtsam::Rot3(), center_of_mass)),
        inertia_(inertia),
        screwAxis_(screwAxis),
        jointLowerLimit_(joint_lower_limit),
        jointUpperLimit_(joint_upper_limit),
        jointLimitThreshold_(joint_limit_threshold),
        velocityLimit_(velocity_limit),
        velocityLimitThreshold_(velocity_limit_threshold),
        accelerationLimit_(acceleration_limit),
        accelerationLimitThreshold_(acceleration_limit_threshold),
        torqueLimit_(torque_limit),
        torqueLimitThreshold_(torque_limit_threshold) {}

  /* Copy constructor */
  Link(const Link &link)
      : jointType_(link.jointType_),
        mass_(link.mass_),
        centerOfMass_(link.centerOfMass_),
        inertia_(link.inertia_),
        screwAxis_(link.screwAxis_),
        jointLowerLimit_(link.jointLowerLimit_),
        jointUpperLimit_(link.jointUpperLimit_),
        jointLimitThreshold_(link.jointLimitThreshold_),
        velocityLimit_(link.velocityLimit_),
        velocityLimitThreshold_(link.velocityLimitThreshold_),
        accelerationLimit_(link.accelerationLimit_),
        accelerationLimitThreshold_(link.accelerationLimitThreshold_),
        torqueLimit_(link.torqueLimit_),
        torqueLimitThreshold_(link.torqueLimitThreshold_) {}

  /* Return screw axis. */
  const gtsam::Vector6 &screwAxis() const { return screwAxis_; }

  /* Return link mass.*/
  double mass() const { return mass_; }

  /* Return center of mass (gtsam::Pose3) */
  const gtsam::Pose3 &centerOfMass() const { return centerOfMass_; }

  /* Return inertia. */
  const gtsam::Matrix3 &inertia() const { return inertia_; }

  /* Return general mass gtsam::Matrix */
  gtsam::Matrix6 inertiaMatrix() const {
    std::vector<gtsam::Matrix> gmm;
    gmm.push_back(inertia_);
    gmm.push_back(gtsam::I_3x3 * mass_);
    return gtsam::diag(gmm);
  }

  /* Return joint angle lower limit. */
  double jointLowerLimit() const { return jointLowerLimit_; }

  /* Return joint angle upper limit. */
  double jointUpperLimit() const { return jointUpperLimit_; }

  /* Return joint angle limit threshold. */
  double jointLimitThreshold() const { return jointLimitThreshold_; }

  /* Return joint velocity limit. */
  double velocityLimit() const { return velocityLimit_; }

  /* Return joint velocity limit threshold. */
  double velocityLimitThreshold() const { return velocityLimitThreshold_; }

  /* Return joint acceleration limit. */
  double accelerationLimit() const { return accelerationLimit_; }

  /* Return joint acceleration limit threshold. */
  double accelerationLimitThreshold() const {
    return accelerationLimitThreshold_;
  }

  /* Return joint torque limit. */
  double torqueLimit() const { return torqueLimit_; }

  /* Return joint torque limit threshold. */
  double torqueLimitThreshold() const { return torqueLimitThreshold_; }

  /** Return link transform (gtsam::Pose3).
   * Keyword arguments:
   *  q -- optional generalized joint angle (default 0)
   */
  virtual gtsam::Pose3 A(double q = 0) const { return gtsam::Pose3(); }

  /** Factor enforcing base acceleration.
   *  Keyword argument:
          base_twist_accel (np.array) -- optional acceleration for base
      Example: if you wish to model gravity forces, use
      base_twist_accel = gtsam::Vector(0, 0, 0, 0, 0, -9.8)
      which imparts upwards acceleration on the base, which then will be
      propagated to all links, forcing wrenches and torques to generate
      upward forces consistent with gravity compensation.
      Note: we do not recommand using the way
   */
  static boost::shared_ptr<gtsam::JacobianFactor> BaseTwistAccelFactor(
      const gtsam::Vector6 &base_twist_accel);

  /** Factor enforcing external wrench at tool frame.
      Keyword argument:
          N -- number of links, used to create wrench index
          external_wrench (np.array) -- optional external wrench
   */
  static boost::shared_ptr<gtsam::JacobianFactor> ToolWrenchFactor(
      int N, const gtsam::Vector6 &external_wrench);

  /** Create single factor relating this link's twist with previous one.
      Keyword argument:
          j -- index for this joint
          jTi -- previous COM frame, expressed in this link's COM frame
          joint_vel_j -- joint velocity for this link
   */
  boost::shared_ptr<gtsam::JacobianFactor> twistFactor(
      int j, const gtsam::Pose3 &jTi, double joint_vel_j) const;

  /** Create wrench balance factor, common between forward and inverse dynamics.
      Keyword argument:
          j -- index for this joint
          twist_j -- velocity twist for this link, in COM frame
          kTj -- this COM frame, expressed in next link's COM frame
          gravity  -- if given, will create gravity force. In link COM
     frame.
   */
  boost::shared_ptr<gtsam::JacobianFactor> wrenchFactor(
      int j, const gtsam::Vector6 &twist_j, const gtsam::Pose3 &kTj,
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /** Create all factors linking this links dynamics with previous and next
     link.
     Keyword arguments:
        j -- index for this joint
        jTi -- previous COM frame, expressed in this link's COM frame
        joint_vel_j -- joint velocity for this link
        twist_j -- velocity twist for this link, in COM frame
        torque_j -- torque at this link's joint
        kTj -- this COM frame, expressed in next link's COM frame
        gravity -- if given, will create gravity force. In link COM frame.
     Will create several factors corresponding to Lynch & Park book:
          - twist acceleration, Equation 8.47, page 293
          - wrench balance, Equation 8.48, page 293
          - torque-wrench relationship, Equation 8.49, page 293
   */
  gtsam::GaussianFactorGraph forwardFactors(
      int j, const gtsam::Pose3 &jTi, double joint_vel_j,
      const gtsam::Vector6 &twist_j, double torque_j, const gtsam::Pose3 &kTj,
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  /** Create all factors linking this links dynamics with previous and next
     link.
     Keyword arguments:
        j -- index for this joint jTi -- previous COM frame,
                                         expressed in this link's COM frame
        joint_vel_j -- joint velocity for this link
        twist_j -- velocity twist for this link, in COM frame
        acceleration_j - acceleration at this link's joint
        kTj -- this COM frame, expressed in next link's COM frame
        gravity  -- if given, will create gravity force. In link COM frame.
        Will create several factors corresponding to Lynch & Park book:
          - twist acceleration, Equation 8.47, page 293
          - wrench balance, Equation 8.48, page 293
          - torque-wrench relationship, Equation 8.49, page 293
   */
  gtsam::GaussianFactorGraph inverseFactors(
      int j, const gtsam::Pose3 &jTi, double joint_vel_j,
      const gtsam::Vector6 &twist_j, double acceleration_j,
      const gtsam::Pose3 &kTj,
      boost::optional<gtsam::Vector3 &> gravity = boost::none) const;

  virtual ~Link() = default;
};
}  // namespace manipulator
