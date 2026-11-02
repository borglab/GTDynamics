/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file trajectory_init.h
 * @author Harrison Jenkins
 *
 * Joint-space initial guess for manipulator SCATE (GPMP2-style formulation).
 *
 *   state   x_i = [q_i; v_i]   (joint angles, joint velocities)
 *   control u_i = qddot_i      (joint accelerations, unit "mass" double integrator)
 *
 * This header depends only on Eigen (gtsam::Vector == Eigen::VectorXd), so it
 * can be compiled and tested without GTDynamics.
 */

#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace scate {

/// Dynamic-size vector; the same type as gtsam::Vector.
using Vector = Eigen::VectorXd;
/// Dynamic-size matrix; the same type as gtsam::Matrix.
using Matrix = Eigen::MatrixXd;

constexpr double kPi = 3.14159265358979323846;

/// Settings of the initial joint-space trajectory.
struct TrajectoryConfig {
  int num_poses = 21;           ///< N+1 states (x_0 ... x_N)
  double dt = 1.0;              ///< time between states
  double bump_amplitude = 0.0;  ///< rad; sideways joint-space bump (see makeInitialTrajectory)
  Vector bump_direction;        ///< size n; empty => no bump
};

/**
 * Discrete double integrator per joint, exact for piecewise-constant qddot:
 *   q_{i+1} = q_i + dt v_i + 0.5 dt^2 u_i
 *   v_{i+1} = v_i + dt u_i
 * i.e. x_{i+1} = Fx x_i + Fu u_i with x = [q; v]. This is the manipulator
 * analogue of the LTI dynamics (paper eqs. 13-15) used in the 2D version.
 */
struct DoubleIntegrator {
  Matrix Fx;  ///< 2n x 2n state transition
  Matrix Fu;  ///< 2n x n control input
};

/// Build Fx and Fu of the double integrator for n joints and step dt.
inline DoubleIntegrator makeDoubleIntegrator(int n, double dt) {
  DoubleIntegrator dynamics;
  const Matrix identity = Matrix::Identity(n, n);
  dynamics.Fx = Matrix::Identity(2 * n, 2 * n);
  dynamics.Fx.topRightCorner(n, n) = dt * identity;
  dynamics.Fu = Matrix::Zero(2 * n, n);
  dynamics.Fu.topRows(n) = 0.5 * dt * dt * identity;
  dynamics.Fu.bottomRows(n) = dt * identity;
  return dynamics;
}

/// 0 -> 1 with zero velocity and acceleration at both ends (rest-to-rest).
inline double smootherstep(double t) {
  return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t);
}

/// Stack joint angles and velocities into the state x = [q; v].
inline Vector stack(const Vector& q, const Vector& v) {
  Vector x(q.size() + v.size());
  x << q, v;
  return x;
}

/**
 * Least-squares control for one step: argmin_u || x_next - Fx x - Fu u ||^2.
 * Per joint this has the closed form
 *   u = (0.5 dt^2 r_q + dt r_v) / (0.25 dt^4 + dt^2),
 * with r = x_next - Fx x = [r_q; r_v]. QR is used instead so it keeps working
 * if Fu changes later (e.g. scaled by inertia).
 */
inline Vector fitControl(const DoubleIntegrator& dynamics, const Vector& x,
                         const Vector& x_next) {
  const Vector residual = x_next - dynamics.Fx * x;
  return dynamics.Fu.colPivHouseholderQr().solve(residual);
}

/// A joint-space trajectory: states x_0..x_N and controls u_0..u_{N-1}.
struct JointTrajectory {
  std::vector<Vector> q;  ///< N+1 joint-angle vectors
  std::vector<Vector> v;  ///< N+1 joint-velocity vectors
  std::vector<Vector> u;  ///< N   joint-acceleration vectors
};

/**
 * Build the initial guess:
 *  - path: straight line in joint space from q_start to q_goal, plus an
 *    optional sin() bump along `bump_direction` (use it to commit to one side
 *    of an obstacle once there is one),
 *  - timing: smootherstep, so the arm starts and ends at rest,
 *  - velocities: central finite differences (zero at both ends),
 *  - controls: least-squares fit to x_{i+1} ~ Fx x_i + Fu u_i, so the guess
 *    is as consistent as possible with the dynamics factors to come.
 */
inline JointTrajectory makeInitialTrajectory(const Vector& q_start,
                                             const Vector& q_goal,
                                             const TrajectoryConfig& config) {
  const int n = static_cast<int>(q_start.size());
  const int num_poses = config.num_poses;
  if (q_goal.size() != n) throw std::invalid_argument("q_start/q_goal size mismatch");
  if (num_poses < 3) throw std::invalid_argument("need at least 3 poses");
  if (config.bump_direction.size() != 0 && config.bump_direction.size() != n)
    throw std::invalid_argument("bump_direction must be empty or size n");

  const Vector displacement = q_goal - q_start;
  const Vector direction = config.bump_direction.size() == n ? config.bump_direction
                                                             : Vector::Zero(n);

  JointTrajectory trajectory;
  trajectory.q.resize(num_poses);
  trajectory.v.assign(num_poses, Vector::Zero(n));
  trajectory.u.resize(num_poses - 1);

  for (int i = 0; i < num_poses; ++i) {
    const double s = smootherstep(static_cast<double>(i) / (num_poses - 1));
    trajectory.q[i] = q_start + s * displacement +
                      config.bump_amplitude * std::sin(kPi * s) * direction;
  }
  for (int i = 1; i < num_poses - 1; ++i) {
    trajectory.v[i] = (trajectory.q[i + 1] - trajectory.q[i - 1]) / (2.0 * config.dt);
  }

  const DoubleIntegrator dynamics = makeDoubleIntegrator(n, config.dt);
  for (int i = 0; i < num_poses - 1; ++i) {
    trajectory.u[i] = fitControl(dynamics, stack(trajectory.q[i], trajectory.v[i]),
                                 stack(trajectory.q[i + 1], trajectory.v[i + 1]));
  }
  return trajectory;
}

/// max_i || x_{i+1} - Fx x_i - Fu u_i ||_inf over the whole trajectory.
inline double maxDynamicsResidual(const JointTrajectory& trajectory,
                                  const DoubleIntegrator& dynamics) {
  double worst = 0.0;
  for (size_t i = 0; i + 1 < trajectory.q.size(); ++i) {
    const Vector residual = stack(trajectory.q[i + 1], trajectory.v[i + 1]) -
                            dynamics.Fx * stack(trajectory.q[i], trajectory.v[i]) -
                            dynamics.Fu * trajectory.u[i];
    worst = std::max(worst, residual.cwiseAbs().maxCoeff());
  }
  return worst;
}

}  // namespace scate
