class gtsam::Vector6;
class gtsam::Vector3;
class gtsam::Matrix6;
class gtsam::Matrix3;
class gtsam::Pose3;
class gtsam::Point3;
class gtsam::JacobianFactor;
class gtsam::GaussianFactorGraph;
class gtsam::VectorValues;
class gtsam::Values;
class gtsam::NonlinearFactorGraph;
class gtsam::noiseModel::Base;
virtual class gtsam::NoiseModelFactor;

namespace manipulator {
#include <Link.h>

virtual class Link {
  Link(char joint_type, double mass, const gtsam::Pose3& center_of_mass,
       const gtsam::Matrix3& inertia, const gtsam::Vector6& screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  Link(char joint_type, double mass, const gtsam::Point3& center_of_mass,
       const gtsam::Matrix3& inertia, const gtsam::Vector6& screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  gtsam::Vector6 screwAxis() const;
  gtsam::Matrix3 inertia() const;
  double mass() const;
  gtsam::Pose3 centerOfMass() const;

  gtsam::Matrix6 inertiaMatrix() const;

  double jointLowerLimit() const;

  double jointUpperLimit() const;

  double jointLimitThreshold() const;

  double velocityLimit() const;

  double velocityLimitThreshold() const;

  double accelerationLimit() const;

  double accelerationLimitThreshold() const;

  double torqueLimit() const;

  double torqueLimitThreshold() const;

  gtsam::Pose3 linkTransform(double q) const;

  static gtsam::JacobianFactor* BaseTwistAccelFactor(
      const gtsam::Vector6& base_twist_accel);

  static gtsam::JacobianFactor* ToolWrenchFactor(
      int N, const gtsam::Vector6& external_wrench);

  gtsam::JacobianFactor* twistFactor(
      int j, const gtsam::Pose3& jTi, double joint_vel_j) const;

  gtsam::JacobianFactor* wrenchFactor(
      int j, const gtsam::Vector6& twist_j, const gtsam::Pose3& kTj) const;
  
  gtsam::JacobianFactor* wrenchFactor(
      int j, const gtsam::Vector6& twist_j, const gtsam::Pose3& kTj,
      gtsam::Vector3& gravity) const;

  gtsam::GaussianFactorGraph forwardFactors(
      int j, const gtsam::Pose3& jTi, double joint_vel_j,
      const gtsam::Vector6& twist_j, double torque_j, const gtsam::Pose3& kTj) const;

    gtsam::GaussianFactorGraph forwardFactors(
      int j, const gtsam::Pose3& jTi, double joint_vel_j,
      const gtsam::Vector6& twist_j, double torque_j, const gtsam::Pose3& kTj,
      gtsam::Vector3& gravity) const;

  gtsam::GaussianFactorGraph inverseFactors(
      int j, const gtsam::Pose3& jTi, double joint_vel_j,
      const gtsam::Vector6& twist_j, double acceleration_j,
      const gtsam::Pose3& kTj) const;

  gtsam::GaussianFactorGraph inverseFactors(
      int j, const gtsam::Pose3& jTi, double joint_vel_j,
      const gtsam::Vector6& twist_j, double acceleration_j,
      const gtsam::Pose3& kTj,
      gtsam::Vector3& gravity) const;

};

#include <DHLink.h>
#include <URDFLink.h>
virtual class DH_Link : manipulator::Link {
  DH_Link(double theta, double d, double a, double alpha, char joint_type,
          double mass, const gtsam::Point3& center_of_mass,
          const gtsam::Matrix3& inertia, double joint_lower_limit,
          double joint_upper_limit, double joint_limit_threshold,
          double velocity_limit, double velocity_limit_threshold,
          double acceleration_limit,
          double acceleration_limit_threshold,
          double torque_limit, double torque_limit_threshold);

  double length() const;
};

virtual class URDF_Link : manipulator::Link {
  URDF_Link(const gtsam::Pose3& origin, const gtsam::Vector3& axis,
            char joint_type, double mass, const gtsam::Pose3& center_of_mass,
            const gtsam::Matrix3& inertia, double joint_lower_limit,
            double joint_upper_limit, double joint_limit_threshold,
            double velocity_limit,
            double velocity_limit_threshold,
            double acceleration_limit,
            double acceleration_limit_threshold,
            double torque_limit, double torque_limit_threshold);

  double length() const;
};

#include <JointLimitVectorFactor.h>
virtual class JointLimitVectorFactor : gtsam::NoiseModelFactor {
  JointLimitVectorFactor(gtsam::Key pose_key,
                         const gtsam::noiseModel::Base* cost_model,
                         const Vector& lower_limits,
                         const Vector& upper_limits,
                         const Vector& limit_thresholds);        
};

// TODO: add constructor to this class after fixing the std::vector problem.
#include <PoseGoalFactor.h>
virtual class PoseGoalFactor : gtsam::NoiseModelFactor {   
};

#include <Arm.h>
template<T = {manipulator::DH_Link, manipulator::URDF_Link}>
class Arm {
//   Arm(const std::vector<T>& links, const gtsam::Pose3& base,
//       const gtsam::Pose3& tool);

  gtsam::Pose3 base() const;
  gtsam::Pose3 tool() const;
  int numLinks() const;
  T link(int i) const;
  Vector jointLowerLimits() const;
  Vector jointUpperLimits() const;
  Vector jointLimitThresholds() const;

// //   std::vector<gtsam::Pose3> linkTransforms(
// //       const Vector& q = Vector::Zero(1)) const;

// //   std::vector<gtsam::Pose3> forwardKinematics(
// //       const Vector& q,
// //       boost::optional<std::vector<gtsam::Matrix>& > J = boost::none) const;

// //   std::vector<gtsam::Pose3> linkFrames(
// //       const Vector& q = Vector::Zero(1)) const;

// //   std::vector<gtsam::Pose3> comFrames(
// //       const Vector& q = Vector::Zero(1)) const;

// //   std::vector<gtsam::Pose3> transformPOE(
// //       const Vector& q = Vector::Zero(1)) const;

// //   std::vector<gtsam::Vector6> screwAxes() const { return screwAxes_; }

// //   std::vector<gtsam::Vector6> spatialScrewAxes() const;

// //   std::vector<gtsam::Matrix> spatialManipulatorJacobian(
// //       const Vector& q) const;

// //   std::vector<gtsam::Matrix> bodyManipulatorJacobian(
// //       const Vector& q, const std::vector<gtsam::Pose3>& sTb) const;

// //   std::vector<gtsam::Vector6> twists(
// //       const std::vector<gtsam::Pose3>& Ts,
// //       const Vector& joint_velocities) const;

// //   std::vector<gtsam::Pose3> jTi_list(const Vector& q) const;

  gtsam::GaussianFactorGraph forwardDynamicsFactorGraph(
      const Vector& q, const Vector& joint_velocities,
      const Vector& torques,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench) const;

  gtsam::GaussianFactorGraph forwardDynamicsFactorGraph(
      const Vector& q, const Vector& joint_velocities,
      const Vector& torques,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench,
      gtsam::Vector3& gravity) const;

  gtsam::GaussianFactorGraph inverseDynamicsFactorGraph(
      const Vector& q, const Vector& joint_velocities,
      const Vector& joint_accelerations,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench) const;

  gtsam::GaussianFactorGraph inverseDynamicsFactorGraph(
      const Vector& q, const Vector& joint_velocities,
      const Vector& joint_accelerations,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench,
      gtsam::Vector3& gravity) const;

  Vector extractJointAcceleraions(
      const gtsam::VectorValues& result) const;

  Vector extractTorques(const gtsam::VectorValues& result) const;

  gtsam::VectorValues factorGraphOptimization(
      const gtsam::GaussianFactorGraph& dynamics_factor_graph) const;

  Vector forwardDynamics(
      const Vector& q, const Vector& joint_velocities,
      const Vector& torques,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench) const;

  Vector forwardDynamics(
      const Vector& q, const Vector& joint_velocities,
      const Vector& torques,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench,
      gtsam::Vector3& gravity) const;

  Vector inverseDynamics(
      const Vector& q, const Vector& joint_velocities,
      const Vector& joint_accelerations,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench) const;

  Vector inverseDynamics(
      const Vector& q, const Vector& joint_velocities,
      const Vector& joint_accelerations,
      const gtsam::Vector6& base_twist_accel,
      const gtsam::Vector6& external_wrench,
      gtsam::Vector3& gravity) const;

  manipulator::JointLimitVectorFactor jointLimitVectorFactor() const;

  gtsam::NonlinearFactorGraph jointLimitFactors(
      const gtsam::noiseModel::Base* cost_model, int i) const;

  manipulator::PoseGoalFactor poseGoalFactor(const gtsam::Pose3& goal_pose) const;

  gtsam::NonlinearFactorGraph inverseKinematicsFactorGraph(
      const gtsam::Pose3& goal_pose) const;

  Vector extractJointCooridinates(const gtsam::Values& results) const;

  gtsam::Values factorGraphOptimization(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& init_values) const;

  Vector inverseKinematics(const gtsam::Pose3& goal_pose,
                                  const Vector& init_q) const;
};

}