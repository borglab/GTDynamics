class gtsam::Vector6;
class gtsam::Vector3;
class gtsam::Matrix6;
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
  Link(size_t joint_type, double mass, const gtsam::Pose3& center_of_mass,
       const Matrix& inertia, const gtsam::Vector6& screwAxis);

  Link(size_t joint_type, double mass, const gtsam::Pose3& center_of_mass,
       const Matrix& inertia, const gtsam::Vector6& screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  Link(size_t joint_type, double mass, const gtsam::Point3& center_of_mass,
       const Matrix& inertia, const gtsam::Vector6& screwAxis);

  Link(size_t joint_type, double mass, const gtsam::Point3& center_of_mass,
       const Matrix& inertia, const gtsam::Vector6& screwAxis,
       double joint_lower_limit, double joint_upper_limit,
       double joint_limit_threshold, double velocity_limit,
       double velocity_limit_threshold, double acceleration_limit,
       double acceleration_limit_threshold, double torque_limit,
       double torque_limit_threshold);

  // Testable
  void print(string s) const;

  gtsam::Vector6 screwAxis() const;
  Matrix inertia() const;
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
  DH_Link(double theta, double d, double a, double alpha, size_t joint_type,
          double mass, const gtsam::Point3& center_of_mass);

  DH_Link(double theta, double d, double a, double alpha, size_t joint_type,
          double mass, const gtsam::Point3& center_of_mass,
          const Matrix& inertia, double joint_lower_limit,
          double joint_upper_limit, double joint_limit_threshold,
          double velocity_limit, double velocity_limit_threshold,
          double acceleration_limit,
          double acceleration_limit_threshold,
          double torque_limit, double torque_limit_threshold);

  double length() const;
};

virtual class URDF_Link : manipulator::Link {
  URDF_Link(const gtsam::Pose3& origin, const gtsam::Vector3& axis,
            size_t joint_type, double mass, const gtsam::Pose3& center_of_mass,
            const Matrix& inertia, double joint_lower_limit,
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
virtual class Arm {
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

#include <DhArm.h>
// Actually a vector<DH_Link>
class DH_LinkVector {
  DH_LinkVector();
//   DH_LinkVector(const DH_LinkVector& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  manipulator::DH_Link at(size_t i) const;
  manipulator::DH_Link front() const;
  manipulator::DH_Link back() const;
  void push_back(const manipulator::DH_Link& link) const;

//   void serialize() const;
};

virtual class DhArm : manipulator::ArmDH_Link {
  DhArm(const manipulator::DH_LinkVector& links, const gtsam::Pose3& base,
      const gtsam::Pose3& tool);
};

#include <BasePoseFactor.h>
virtual class BasePoseFactor : gtsam::NoiseModelFactor {   
    BasePoseFactor(gtsam::Key pose_key_0,
                 const gtsam::noiseModel::Base* cost_model,
                 const gtsam::Pose3& base_pose);
};

#include <BaseTwistAccelFactor.h>
virtual class BaseTwistAccelFactor : gtsam::NoiseModelFactor {   
    BaseTwistAccelFactor(gtsam::Key twistAccel_key_0,
                       const gtsam::noiseModel::Base* cost_model,
                       const gtsam::Vector6& base_twistAccel);
};

#include <BaseTwistFactor.h>
virtual class BaseTwistFactor : gtsam::NoiseModelFactor {   
    BaseTwistFactor(gtsam::Key twist_key_0,
                  const gtsam::noiseModel::Base* cost_model,
                  const gtsam::Vector6& base_twist);
};

#include <GaussianProcessPriorFactor.h>
virtual class GaussianProcessPriorFactor : gtsam::NoiseModelFactor {   
    GaussianProcessPriorFactor(gtsam::Key q_key1, gtsam::Key qVel_key1, gtsam::Key qAccel_key1,
      gtsam::Key q_key2, gtsam::Key qVel_key2, gtsam::Key qAccel_key2,
      const gtsam::noiseModel::Gaussian* Qc_model, double delta_t);
};

#include <GaussianProcessPriorPose3Factor.h>
virtual class GaussianProcessPriorPose3Factor : gtsam::NoiseModelFactor {   
    GaussianProcessPriorPose3Factor(gtsam::Key p1_key, gtsam::Key v1_key, gtsam::Key vdot1_key,
      gtsam::Key p2_key, gtsam::Key v2_key, gtsam::Key vdot2_key,
      const gtsam::noiseModel::Gaussian* Qc_model, double delta_t);
};

#include <JointLimitFactor.h>
virtual class JointLimitFactor : gtsam::NoiseModelFactor {   
    JointLimitFactor(gtsam::Key q_key,
                   const gtsam::noiseModel::Base* cost_model,
                   const double& lower_limit, const double& upper_limit,
                   const double& limit_threshold);
};

// #include <ObstacleSDFFactor.h>
// virtual class ObstacleSDFFactor : gtsam::NoiseModelFactor {   
//     ObstacleSDFFactor(gtsam::Key poseKey,
//                     const gtsam::noiseModel::Base* cost_model,
//                     double epsilon, const SignedDistanceField& sdf,
//                     double radius,
//                     const std::vector<gtsam::Point3>& sphere_centers);
// };

#include <PoseFactor.h>
virtual class PoseFactor : gtsam::NoiseModelFactor {   
    PoseFactor(gtsam::Key pose_key_i, gtsam::Key pose_key_j, gtsam::Key q_key,
             const gtsam::noiseModel::Base* cost_model,
             const gtsam::Pose3& jMi, const gtsam::Vector6& screw_axis);
};

#include <SignedDistanceField.h>
 class SignedDistanceField {
    SignedDistanceField();

    SignedDistanceField(const gtsam::Point3& origin, double cell_size,
                        size_t field_rows,
                        size_t field_cols, size_t field_z);
    void print(string s) const;
    void saveSDF(string filename);
    void loadSDF(string filename);                        
 };
#include <SphereLink.h>
 class SphereLink {
    // SphereLink(double radius, const std::vector<gtsam::Point3>& sphere_centers);                  
 };

#include <ToolPoseFactor.h>
virtual class ToolPoseFactor : gtsam::NoiseModelFactor {   
    ToolPoseFactor(gtsam::Key pose_key,
                 const gtsam::noiseModel::Base* cost_model,
                 const gtsam::Pose3& tTn, const gtsam::Pose3& tool_pose);
};
#include <ToolWrenchFactor.h>
virtual class ToolWrenchFactor : gtsam::NoiseModelFactor {   
    ToolWrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                   gtsam::Key wrench_key_j, gtsam::Key pose_key,
                   const gtsam::noiseModel::Base* cost_model,
                   const gtsam::Pose3& tTn, const gtsam::Matrix6& inertia,
                   const gtsam::Vector6& external_wrench);

    ToolWrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                   gtsam::Key wrench_key_j, gtsam::Key pose_key,
                   const gtsam::noiseModel::Base* cost_model,
                   const gtsam::Pose3& tTn, const gtsam::Matrix6& inertia,
                   const gtsam::Vector6& external_wrench,
                   gtsam::Vector3& gravity);
};

#include <TorqueFactor.h>
virtual class TorqueFactor : gtsam::NoiseModelFactor {   
    TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
               const gtsam::noiseModel::Base* cost_model,
               const gtsam::Vector6& screw_axis);
};

#include <TwistAccelFactor.h>
virtual class TwistAccelFactor : gtsam::NoiseModelFactor {   
    TwistAccelFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key_i,
                   gtsam::Key twistAccel_key_j, gtsam::Key q_key,
                   gtsam::Key qVel_key, gtsam::Key qAccel_key,
                   const gtsam::noiseModel::Base* cost_model,
                   const gtsam::Pose3& jMi, const gtsam::Vector6& screw_axis);
};

#include <TwistFactor.h>
virtual class TwistFactor : gtsam::NoiseModelFactor {   
    TwistFactor(gtsam::Key twistI_key, gtsam::Key twistJ_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtsam::Pose3& jMi, const gtsam::Vector6& screw_axis);
};

#include <WrenchFactor.h>
virtual class WrenchFactor : gtsam::NoiseModelFactor {   
    WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
               gtsam::Key wrench_key_j, gtsam::Key wrench_key_k,
               gtsam::Key pose_key, gtsam::Key q_key,
               const gtsam::noiseModel::Base* cost_model,
               const gtsam::Pose3& kMj, const gtsam::Matrix6& inertia,
               const gtsam::Vector6& screw_axis);

    WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
               gtsam::Key wrench_key_j, gtsam::Key wrench_key_k,
               gtsam::Key pose_key, gtsam::Key q_key,
               const gtsam::noiseModel::Base* cost_model,
               const gtsam::Pose3& kMj, const gtsam::Matrix6& inertia,
               const gtsam::Vector6& screw_axis,
               gtsam::Vector3& gravity);
};

}