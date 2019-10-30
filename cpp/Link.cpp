/**
 * @file  link.cpp
 * @brief manipulator link
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <Link.h>

using namespace std;
using namespace gtsam;

namespace manipulator {

Symbol T(int j) { return Symbol('T', j); }
Symbol a(int j) { return Symbol('a', j); }
Symbol F(int j) { return Symbol('F', j); }
Symbol t(int j) { return Symbol('t', j); }
Symbol V(int j) { return Symbol('V', j); }
Symbol J(int j) { return Symbol('J', j); }

/// @name Testable
/// @{

/** print with optional string */
void Link::print(const std::string &s) const {
  std::cout << s << " Link joint type is " << jointType_ << std::endl;
  std::cout << " Link mass is " << mass_ << std::endl;
}

/// @}

boost::shared_ptr<JacobianFactor> Link::BaseTwistAccelFactor(
    const gtsam::Vector6 &base_twist_accel) {
  return boost::make_shared<gtsam::JacobianFactor>(
      T(0), gtsam::I_6x6, base_twist_accel,
      gtsam::noiseModel::Constrained::All(6));
}

boost::shared_ptr<JacobianFactor> Link::firstTwistAccelFactor(
    const gtsam::Vector6 &base_twist_accel, const Pose3 &jTi,
    double joint_vel_j, const Vector6 &twist_j, double acceleration_j) const {
  gttic_(Link_FirstTwistAccelFactor);
  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  // Given the above Equation 8.47 can be written as
  // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j *
  // acceleration_j
  Vector6 rhs = ad_j * A_j * joint_vel_j + A_j * acceleration_j +
                jTi.AdjointMap() * base_twist_accel;
  return boost::make_shared<gtsam::JacobianFactor>(
      T(1), gtsam::I_6x6, rhs, gtsam::noiseModel::Constrained::All(6));
}

boost::shared_ptr<JacobianFactor> Link::firstTwistAccelFactor(
    const gtsam::Vector6 &base_twist_accel, const Pose3 &jTi,
    double joint_vel_j, const Vector6 &twist_j) const {
  gttic_(Link_FirstTwistAccelFactor);
  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  // Given the above Equation 8.47 can be written as
  // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j *
  // acceleration_j
  Vector6 rhs = ad_j * A_j * joint_vel_j +
                jTi.AdjointMap() * base_twist_accel;
  return boost::make_shared<gtsam::JacobianFactor>(
      T(1), gtsam::I_6x6, a(1), -A_j, rhs, gtsam::noiseModel::Constrained::All(6));
}

boost::shared_ptr<JacobianFactor> Link::ToolWrenchFactor(
    int N, const gtsam::Vector6 &external_wrench) {
  gttic_(Link_ToolWrenchFactor);
  return boost::make_shared<gtsam::JacobianFactor>(
      F(N + 1), gtsam::I_6x6, -external_wrench,
      gtsam::noiseModel::Constrained::All(6));
}

boost::shared_ptr<JacobianFactor> Link::lastWrenchFactor(
    const gtsam::Vector6 &external_wrench, int j, const Vector6 &twist_j,
    const Pose3 &kTj, const boost::optional<Vector3> &gravity) const {
  gttic_(Link_lastWrenchFactor);
  // Wrench on this link is due to acceleration and reaction to next link.
  // We need inertia, coriolis forces, gravity, and an Adjoint map:
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  Matrix6 G_j = inertiaMatrix();
  auto jAk = kTj.AdjointMap().transpose();
  Vector6 rhs = ad_j.transpose() * G_j * twist_j + jAk * external_wrench;

  if (gravity) {
    Vector gravitational_force = *gravity * mass();
    for (int i = 3; i < 6; ++i) {
      rhs[i] += gravitational_force[i - 3];
    }
  }
  // Given the above Equation 8.48 can be written as
  // G_j * T(j) - F(j) == jAk * external_wrench + coriolis_j [+ gravity]
  return boost::make_shared<gtsam::JacobianFactor>(
      T(j), G_j, F(j), -I_6x6, rhs, noiseModel::Constrained::All(6));
}

boost::shared_ptr<JacobianFactor> Link::twistFactor(int j, const Pose3 &jTi,
                                                    double joint_vel_j) const {
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Vector6 joint_twist = A_j * joint_vel_j;

  if (j == 1) {
    return boost::make_shared<gtsam::JacobianFactor>(
        V(j), I_6x6, joint_twist, noiseModel::Constrained::All(6));
  } else {
    // Equation 8.45 in MR, page 292
    // V(j) - np.dot(jTi.AdjointMap(), V(j-1)) == joint_twist
    return boost::make_shared<gtsam::JacobianFactor>(
        V(j), I_6x6, V(j - 1), -jTi.AdjointMap(), joint_twist,
        noiseModel::Constrained::All(6));
  }
}

boost::shared_ptr<JacobianFactor> Link::wrenchFactor(
    int j, const Vector6 &twist_j, const Pose3 &kTj,
    const boost::optional<Vector3> &gravity) const {
  // Wrench on this link is due to acceleration and reaction to next link.
  // We need inertia, coriolis forces, gravity, and an Adjoint map:
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  Matrix6 G_j = inertiaMatrix();
  Vector6 rhs = ad_j.transpose() * G_j * twist_j;

  if (gravity) {
    Vector gravitational_force = *gravity * mass();
    for (int i = 3; i < 6; ++i) {
      rhs[i] += gravitational_force[i - 3];
    }
  }

  auto jAk = kTj.AdjointMap().transpose();

  // Given the above Equation 8.48 can be written as
  // G_j * T(j) - F(j) + jAk * F(j + 1) == coriolis_j [+ gravity]
  return boost::make_shared<gtsam::JacobianFactor>(
      T(j), G_j, F(j), -I_6x6, F(j + 1), jAk, rhs,
      noiseModel::Constrained::All(6));
}

gtsam::GaussianFactorGraph Link::forwardLoopFactor(
    int j, const gtsam::Vector6 &screw_axis, const gtsam::Pose3 &jTi,
    double joint_vel_j, double torque_j) {
  GaussianFactorGraph factors = GaussianFactorGraph();
  // Twist of base link is zero
  auto twist_j = Vector6::Zero();
  Vector6 A_j = screw_axis;  // joint axis expressed in base frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);

  // add twist acceleration factor
  // Given the above Equation 8.47 can be written as
  // T(j) - A_j * a(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j
  Vector6 rhs = ad_j * A_j * joint_vel_j;
  factors.add(T(0), I_6x6, a(j), -A_j, T(j - 1),
              -jTi.AdjointMap(), rhs, noiseModel::Constrained::All(6));

  // add planar wrench factor to fix the indeterminate issue
  Matrix36 J_wrench;
  J_wrench << 1, 0, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0, 0,          //
      0, 0, 0, 0, 0, 1;
  factors.add(F(j), J_wrench, gtsam::Vector3::Zero(), noiseModel::Constrained::All(3));

  // add torque factor
  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  factors.add(F(j), A_j.transpose(), Vector1(torque_j),
              noiseModel::Constrained::All(1));
  return factors;
}

gtsam::GaussianFactorGraph Link::inverseLoopFactor(
    int j, const gtsam::Vector6 &screw_axis, const gtsam::Pose3 &jTi,
    double joint_vel_j, double acceleration_j, double internal_torque) {
  GaussianFactorGraph factors = GaussianFactorGraph();
  // Twist of base link is zero
  auto twist_j = Vector6::Zero();
  Vector6 A_j = screw_axis;  // joint axis expressed in base frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);

  // add twist acceleration factor
  // Given the above Equation 8.47 can be written as
  // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j + A_j * a(j)
  Vector6 rhs = ad_j * A_j * joint_vel_j + A_j * acceleration_j;
  factors.add(T(0), I_6x6, T(j-1), -jTi.AdjointMap(), rhs,
              noiseModel::Constrained::All(6));

  // add planar wrench factor to fix the indeterminate issue
  Matrix36 J_wrench;
  J_wrench << 1, 0, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0, 0,          //
      0, 0, 0, 0, 0, 1;
  factors.add(F(j), J_wrench, gtsam::Vector3::Zero(), noiseModel::Constrained::All(3));

  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  // internal_torque includes spring torque and damping torque
  factors.add(F(j), A_j.transpose(), t(j), -I_1x1, Vector1(internal_torque),
              noiseModel::Constrained::All(1));

  return factors;
}

GaussianFactorGraph Link::forwardFactors(
    int j, const Pose3 &jTi, double joint_vel_j, const Vector6 &twist_j,
    double torque_j, const Pose3 &kTj,
    const boost::optional<Vector3> &gravity) const {
  GaussianFactorGraph factors = GaussianFactorGraph();

  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);

  // Given the above Equation 8.47 can be written as
  // T(j) - A_j * a(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j
  Vector6 rhs = ad_j * A_j * joint_vel_j;
  factors.add(T(j), I_6x6, a(j), -A_j, T(j - 1), -jTi.AdjointMap(), rhs,
              noiseModel::Constrained::All(6));

  // Wrench on this link is due to acceleration and reaction to next link.
  factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));

  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  factors.add(F(j), A_j.transpose(), Vector1(torque_j),
              noiseModel::Constrained::All(1));

  return factors;
}

GaussianFactorGraph Link::reducedForwardFactors(
    int j, int N, const Pose3 &jTi, double joint_vel_j, const Vector6 &twist_j,
    double torque_j, const Pose3 &kTj,
    const boost::optional<Vector3> &gravity) const {
  gttic_(Link_forwardFactors);
  GaussianFactorGraph factors = GaussianFactorGraph();

  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);

  // Given the above Equation 8.47 can be written as
  // T(j) - A_j * a(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j
  if (j > 1) {
    Vector6 rhs = ad_j * A_j * joint_vel_j;
    factors.add(T(j), I_6x6, a(j), -A_j, T(j - 1), -jTi.AdjointMap(), rhs,
                noiseModel::Constrained::All(6));
  }

  // Wrench on this link is due to acceleration and reaction to next link.
  if (j < N) {
    factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));
  }

  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  factors.add(F(j), A_j.transpose(), Vector1(torque_j),
              noiseModel::Constrained::All(1));

  return factors;
}

GaussianFactorGraph Link::inverseFactors(
    int j, const Pose3 &jTi, double joint_vel_j, const Vector6 &twist_j,
    double acceleration_j, const Pose3 &kTj,
    const boost::optional<Vector3> &gravity, double internal_torque) const {
  GaussianFactorGraph factors = GaussianFactorGraph();

  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  // Given the above Equation 8.47 can be written as
  // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j *
  // acceleration_j
  Vector6 rhs = ad_j * A_j * joint_vel_j + A_j * acceleration_j;
  factors.add(T(j), I_6x6, T(j - 1), -jTi.AdjointMap(), rhs,
              noiseModel::Constrained::All(6));

  // Wrench on this link is due to acceleration and reaction to next link.
  factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));

  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  factors.add(F(j), A_j.transpose(), t(j), -I_1x1, Vector1(internal_torque),
              noiseModel::Constrained::All(1));

  return factors;
}

GaussianFactorGraph Link::reducedInverseFactors(
    int j, int N, const Pose3 &jTi, double joint_vel_j, const Vector6 &twist_j,
    double acceleration_j, const Pose3 &kTj,
    const boost::optional<Vector3> &gravity, double internal_torque) const {
  gttic_(Link_inverseFactors);
  GaussianFactorGraph factors = GaussianFactorGraph();

  // Twist acceleration in this link as a function of previous and joint accel.
  // We need to know our screw axis, and an adjoint map:
  Vector6 A_j = screwAxis_;  // joint axis expressed in COM frame
  Matrix6 ad_j = Pose3::adjointMap(twist_j);
  // Given the above Equation 8.47 can be written as
  // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j *
  // acceleration_j
  if (j > 1) {
    Vector6 rhs = ad_j * A_j * joint_vel_j + A_j * acceleration_j;
    factors.add(T(j), I_6x6, T(j - 1), -jTi.AdjointMap(), rhs,
                noiseModel::Constrained::All(6));
  }

  // Wrench on this link is due to acceleration and reaction to next link.
  if (j < N) {
    factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));
  }

  // Torque is always wrench projected on screw axis.
  // Equation 8.49 can be written as
  // A_j.transpose() * F(j).transpose() == torque_j
  factors.add(F(j), A_j.transpose(), t(j), -I_1x1, Vector1(internal_torque),
              noiseModel::Constrained::All(1));

  return factors;
}
}  // namespace manipulator
