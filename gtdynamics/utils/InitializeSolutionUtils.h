/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  InitializeSolutionUtils.h
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @Author: Alejandro Escontrela and Yetong Zhang
 */

#ifndef GTDYNAMICS_UTILS_INITIALIZESOLUTIONUTILS_H_
#define GTDYNAMICS_UTILS_INITIALIZESOLUTIONUTILS_H_

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>

#include <random>
#include <string>
#include <vector>

#include <boost/optional.hpp>

namespace gtdynamics {

/** @fn Initialize solution via linear interpolation of initial and final pose.
 *
 * @param[in] robot           A gtdynamics::Robot object.
 * @param[in] link_name       The name of the link whose pose to interpolate.
 * @param[in] wTl_i           The initial pose of the link.
 * @param[in] wTl_f           The final pose of the link.
 * @param[in] T_s             Time at which to start interpolation.
 * @param[in] T_f             Time at which to end interpolation.
 * @param[in] dt              The duration of a single timestep.
 * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
 *      Noise drawn from a zero-mean gaussian distribution with a standard
 *      deviation of gaussian_noise.
 * @param[in] contact_points  ContactPoint objects.
 * @return Initial solution stored in gtsam::Values object.
 */
gtsam::Values InitializeSolutionInterpolation(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const gtsam::Pose3& wTl_f, const double& T_s, const double& T_f,
    const double& dt, const double& gaussian_noise = 0.0,
    const boost::optional<std::vector<ContactPoint>>& contact_points =
        boost::none);

/** @fn Initialize interpolated solution for multiple phases.
 *
 * @param[in] robot           A gtdynamics::Robot object.
 * @param[in] link_name       The name of the link whose pose to interpolate.
 * @param[in] wTl_i           The initial pose of the link.
 * @param[in] wTl_t           A vector of desired poses.
 * @param[in] ts              Times at which poses start and end.
 * @param[in] dt              The duration of a single timestep.
 * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
 *      Noise drawn from a zero-mean gaussian distribution with a standard
 *      deviation of gaussian_noise.
 * @param[in] contact_points  ContactPoint objects.
 * @return Initial solution stored in gtsam::Values object.
 */
gtsam::Values InitializeSolutionInterpolationMultiPhase(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise = 0.0,
    const boost::optional<std::vector<ContactPoint>>& contact_points =
        boost::none);

/** @fn Iteratively solve for the robot kinematics with contacts.
 *
 * @param[in] robot           A gtdynamics::Robot object.
 * @param[in] link_name       The name of the link whose pose to interpolate.
 * @param[in] wTl_i           The initial pose of the link.
 * @param[in] wTl_t           A vector of desired poses.
 * @param[in] ts              Times at which poses start and end.
 * @param[in] dt              The duration of a single timestep.
 * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
 *      Noise drawn from a zero-mean gaussian distribution with a standard
 *      deviation of gaussian_noise.
 * @param[in] contact_points  ContactPoint objects.
 * @return Initial solution stored in gtsam::Values object.
 */
gtsam::Values InitializeSolutionInverseKinematics(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise = 1e-8,
    const boost::optional<std::vector<ContactPoint>>& contact_points =
        boost::none);

/** @fn Return zero values for all variables for initial value of optimization.
 *
 * @param[in] robot          A gtdynamics::Robot object.
 * @param[in] t              Timestep to return zero initial values for.
 * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
 *      Noise drawn from a zero-mean gaussian distribution with a standard
 *      deviation of gaussian_noise.
 * @param[in] contact_points Contact points for timestep t.
 */
gtsam::Values ZeroValues(
    const Robot& robot, const int t, const double& gaussian_noise = 0.0,
    const boost::optional<ContactPoints>& contact_points = boost::none);

/** @fn Return zero values of the trajectory for initial value of optimization.
 *
 * @param[in] robot          A gtdynamics::Robot object.
 * @param[in] num_steps      Total number of time steps.
 * @param[in] num_phases     Number of phases, -1 if not using.
 * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
 *      Noise drawn from a zero-mean gaussian distribution with a standard
 *      deviation of gaussian_noise.
 * @param[in] contact_points Contact points along the trajectory.
 * @return Initial solution stored in a gtsam::Values object.
 */
gtsam::Values ZeroValuesTrajectory(
    const Robot& robot, const int num_steps, const int num_phases = -1,
    const double& gaussian_noise = 0.0,
    const boost::optional<ContactPoints>& contact_points = boost::none);

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UTILS_INITIALIZESOLUTIONUTILS_H_
