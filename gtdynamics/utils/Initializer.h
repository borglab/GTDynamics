/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  initialize_solution_utils.h
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @author: Alejandro Escontrela, Yetong Zhang
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/optional.hpp>
#include <random>
#include <string>
#include <vector>

namespace gtdynamics {

class Initializer {

 public:
    
    // Default Constructor
    Initializer() {}

    /**
     * Add zero-mean gaussian noise to a Pose3.
     *
     * @param T Pose3 which to add noise to.
     * @param sampler Helper to sample values from a gaussian distribution.
     */
    gtsam::Pose3 AddGaussianNoiseToPose(const gtsam::Pose3& T,
                                            const gtsam::Sampler& sampler) const;

    /**
     * Linearly interpolate between initial pose and desired poses at each
     * specified, discrete timestep.
     *
     * @param wTl_i Initial pose of the link.
     * @param wTl_t Vector of desired poses.
     * @param t_i Initial time.
     * @param timesteps Times at which poses start and end.
     * @param dt The duration of a single timestep.
     */
    std::vector<gtsam::Pose3> InterpolatePoses(
        const gtsam::Pose3& wTl_i, const std::vector<gtsam::Pose3>& wTl_t,
        double t_i, const std::vector<double>& timesteps, double dt);

    /**
     * Initialize the poses and joints needed to perform trajectory optimization.
     *
     * @param robot The robot model.
     * @param wTl_i Initial pose of the link.
     * @param wTl_t Vector of poses at each desired timestep.
     * @param link_name The name of the link whose pose to initialize.
     * @param t_i The initial time of the trajectory.
     * @param timesteps A vector of desired timesteps.
     * @param dt The duration of a single timestep.
     * @param sampler Helper to sample values from a gaussian distribution.
     */
    gtsam::Values InitializePosesAndJoints(const Robot& robot,
                                        const gtsam::Pose3& wTl_i,
                                        const std::vector<gtsam::Pose3>& wTl_t,
                                        const std::string& link_name, double t_i,
                                        const std::vector<double>& timesteps,
                                        double dt, const gtsam::Sampler& sampler,
                                        std::vector<gtsam::Pose3>* wTl_dt);

    /**
     * @fn Initialize solution via linear interpolation of initial and final pose.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_f           The final pose of the link.
     * @param[in] T_s             Time at which to start interpolation.
     * @param[in] T_f             Time at which to end interpolation.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInterpolation(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const gtsam::Pose3& wTl_f, double T_s, double T_f, double dt,
        double gaussian_noise = 0.0,
        const boost::optional<PointOnLinks>& contact_points = boost::none);

    /**
     * @fn Initialize interpolated solution for multiple phases.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_t           A vector of desired poses.
     * @param[in] ts              Times at which poses start and end.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInterpolationMultiPhase(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        double dt, double gaussian_noise = 0.0,
        const boost::optional<PointOnLinks>& contact_points = boost::none);

    /**
     * @fn Iteratively solve for the robot kinematics with contacts.
     *
     * @param[in] robot           A Robot object.
     * @param[in] link_name       The name of the link whose pose to interpolate.
     * @param[in] wTl_i           The initial pose of the link.
     * @param[in] wTl_t           A vector of desired poses.
     * @param[in] ts              Times at which poses start and end.
     * @param[in] dt              The duration of a single timestep.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points  PointOnLink objects.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values InitializeSolutionInverseKinematics(
        const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        double dt, double gaussian_noise = 1e-8,
        const boost::optional<PointOnLinks>& contact_points = boost::none);

    /**
     * @fn Initialize solution for multi-phase trajectory to nominal pose.
     *
     * @param[in] robots                The Robot object to use for each phase.
     * @param[in] phase_steps           Number of steps for each phase.
     * @param[in] transition_graph_init Initial values for transition graphs.
     * @param[in] dt_i                  Initial phase duration,
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     * @param[in] phase_contact_points  Contact points at each phase.
     */
    gtsam::Values MultiPhaseZeroValuesTrajectory(
        const Robot& robot, const std::vector<int>& phase_steps,
        std::vector<gtsam::Values> transition_graph_init, double dt_i = 1. / 240,
        const double gaussian_noise = 1e-8,
        const boost::optional<std::vector<PointOnLinks>>& phase_contact_points =
            boost::none) const;

    /**
     * @fn Multi-phase initialize solution inverse kinematics.
     *
     * @param[in] robots                A Robot object for each phase.
     * @param[in] link_name             The name of the link whose pose to
     * interpolate.
     * @param[in] phase_steps           Number of steps for each phase.
     * @param[in] wTl_i                 The initial pose of the link.
     * @param[in] wTl_t                 A vector of desired poses.
     * @param[in] ts                    Discretized times at which poses start and
     * end.
     * @param[in] transition_graph_init Initial values for transition graphs.
     * @param[in] dt_i                  Initial value for phase duration.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] phase_contact_points  Contact points at each phase.
     * @return Initial solution stored in gtsam::Values object.
     */
    gtsam::Values MultiPhaseInverseKinematicsTrajectory(
        const Robot& robot, const std::string& link_name,
        const std::vector<int>& phase_steps, const gtsam::Pose3& wTl_i,
        const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
        std::vector<gtsam::Values> transition_graph_init, double dt_i = 1. / 240,
        double gaussian_noise = 1e-8,
        const boost::optional<std::vector<PointOnLinks>>& phase_contact_points =
            boost::none);

    /**
     * @fn Return zero values for all variables for initial value of optimization.
     *
     * @param[in] robot          A Robot object.
     * @param[in] t              Timestep to return zero initial values for.
     * @param[in] gaussian_noise  Optional gaussian noise to add to initial values.
     *      Noise drawn from a zero-mean gaussian distribution with a standard
     *      deviation of gaussian_noise.
     * @param[in] contact_points Contact points for timestep t.
     */
    virtual gtsam::Values ZeroValues(
        const Robot& robot, const int t, double gaussian_noise = 0.0,
        const boost::optional<PointOnLinks>& contact_points = boost::none) const;

    /**
     * @fn Return zero values of the trajectory for initial value of optimization.
     *
     * @param[in] robot          A Robot object.
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
        double gaussian_noise = 0.0,
        const boost::optional<PointOnLinks>& contact_points = boost::none);

};

}  // namespace gtdynamics
