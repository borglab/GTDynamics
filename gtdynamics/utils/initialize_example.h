/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file initialize_example.h
 * @brief Set up factor graph, optimization, and trajectory file generation 
 * for a given robot example.
 * @Author: Alejandro Escontrela
 * @Author: Stephanie McCormick
 * @Author: Disha Das
 */

#pragma once

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>

// TODO (disha + stephanie): move to GTDynamics/examples/utils?
// (may be more convenient to do so if examples' compilation process
// remains separate from the rest of GTDynamics')

namespace gtdynamics {

using gtdynamics::ContactPoints, gtdynamics::ContactPoint,
      gtdynamics::Robot, gtsam::noiseModel::Isotropic, gtsam::Values,
      gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
      std::string, std::vector;

typedef ContactPoints CPs;

/** Initialize graph builder with desired dynamics constraint stds.
 * @param[in] dynamics_model_6      -- dynamics noise model
 * @param[in] dynamics_model_3      -- dynamics noise model
 * @param[in] dynamics_model_1      -- dynamics noise model
 * @param[in] objectives_model_6    -- objectives noise model
 * @param[in] objectives_model_3    -- objectives noise model
 * @param[in] objectives_model_1    -- objectives noise model
 */
gtdynamics::DynamicsGraph InitializeGraphBuilder(boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_3,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_1,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_3,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_1);

/** Create factor graph for a robot model given env and contact information.
 * @param[in] gravity               -- env parameter
 * @param[in] mu                    -- env parameter
 * @param[in] ground_height         -- env parameter
 * @param[in] sigma_objectives      -- std of additional objectives
 * @param[in] dynamics_model_6      -- dynamics noise model
 * @param[in] dynamics_model_1      -- dynamics noise model
 * @param[in] objectives_model_6    -- objectives noise model
 * @param[in] objectives_model_3    -- objectives noise model
 * @param[in] objectives_model_1    -- objectives noise model
 * @param[in] graph_builder         -- initialized graph builder
 * @param[in] transition_graph_init -- phase transition initial values
 * @param[in] robot                 -- robot model
 * @param[in] links                 -- vector of link name strings
 * @param[in] dt_des                -- desired timestep duration
 * @param[in] gaussian_noise        -- noise to be added to initial values
 * @param[in] robots                -- robot model for each phase
 * @param[in] phase_cps             -- contact points for each phase
 * @param[in] trans_cps             -- transition contact points
 * @param[in] phase_steps           -- vector of integer phase durations
 * @param[in] contact_point         -- reference contact pt for graph generation
 */
gtsam::NonlinearFactorGraph CreateFactorGraph(Vector3 gravity, double mu,
                             double ground_height, double sigma_objectives,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> dynamics_model_1,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_6,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_3,
                             boost::shared_ptr<gtsam::noiseModel::Base> objectives_model_1,
                             gtdynamics::DynamicsGraph graph_builder,
                             vector<Values> &transition_graph_init,
                             gtdynamics::Robot robot, vector<string> links,
                             double dt_des, double gaussian_noise,
                             vector<Robot> robots, vector<CPs> phase_cps,
                             vector<CPs> trans_cps, vector<int> phase_steps,
                             gtsam::Point3 contact_point);

/** Initialize trajectory solution and optimize.
 * @param[in] graph                 -- multi-phase trajectory factor graph
 * @param[in] transition_graph_init -- phase transition initial values
 * @param[in] dt_des                -- desired timestep duration
 * @param[in] gaussian_noise        -- noise to be added to initial values
 * @param[in] robots                -- robot model for each phase
 * @param[in] phase_cps             -- contact points for each phase
 * @param[in] phase_steps           -- vector of integer phase durations
 */
gtsam::Values Optimize(gtsam::NonlinearFactorGraph graph,
                       vector<Values> &transition_graph_init,
                       double dt_des, double gaussian_noise,
                       vector<Robot> robots,
                       vector<CPs> phase_cps,
                       vector<int> phase_steps);

/** Log the joint angles, velocities, accels, torques, and current goal pose 
 *  in a traj.csv file.
 * @param[in] robot                 -- robot model
 * @param[in] phase_steps           -- vector of integer phase durations
 * @param[in] results               -- optimization result
 */
void CreateTrajFile(gtdynamics::Robot robot, 
                    vector<int> phase_steps, gtsam::Values results);

}  // namespace gtdynamics
