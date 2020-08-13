/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Spider trajectory optimization with pre-specified footholds.
 * @Author: Alejandro Escontrela
 * @Author: Stephanie McCormick
 * @Author: Disha Das
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/initialize_example.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <algorithm>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#define GROUND_HEIGHT -0.04

using gtdynamics::PoseKey, gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey, gtsam::Point3,
    gtsam::Rot3, gtsam::Pose3, gtsam::Values,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string, gtsam::noiseModel::Isotropic;

int main(int argc, char** argv) {

  // Load Stephanie's spider robot.
  auto spider = gtdynamics::CreateRobotFromFile("../spider.sdf", "spider");
  spider.printRobot();

  double sigma_dynamics = 1e-6;    // std of dynamics constraints.
  double sigma_objectives = 1e-4;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_3 = Isotropic::Sigma(3, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_3 = Isotropic::Sigma(3, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Env parameters.
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;

  // All contacts.
  typedef ContactPoint CP;
  auto c1 = CP{"tarsus_1", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front left.
  auto c2 = CP{"tarsus_2", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind left.
  auto c3 = CP{"tarsus_3", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front right.
  auto c4 = CP{"tarsus_4", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind right.
  auto c5 = CP{"tarsus_5", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front left.
  auto c6 = CP{"tarsus_6", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind left.
  auto c7 = CP{"tarsus_7", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front right.
  auto c8 = CP{"tarsus_8", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind right.

  // Contact points for each phase. 
  // This gait moves one leg at a time.
  typedef ContactPoints CPs;
  CPs p0  = {c1, c2, c3, c4, c5, c6, c7, c8};  // Initially stationary.
  CPs t01 = {    c2, c3, c4, c5, c6, c7, c8};
  CPs p1  = {    c2, c3, c4, c5, c6, c7, c8};
  CPs t12 = {        c3, c4, c5, c6, c7, c8};
  CPs p2  = {c1,     c3, c4, c5, c6, c7, c8};
  CPs t23 = {c1,         c4, c5, c6, c7, c8};
  CPs p3  = {c1, c2,     c4, c5, c6, c7, c8};
  CPs t34 = {c1, c2,         c5, c6, c7, c8};
  CPs p4  = {c1, c2, c3,     c5, c6, c7, c8};
  CPs t45 = {c1, c2, c3,         c6, c7, c8};
  CPs p5  = {c1, c2, c3, c4,     c6, c7, c8};
  CPs t56 = {c1, c2, c3, c4,         c7, c8};
  CPs p6  = {c1, c2, c3, c4, c5,     c7, c8};
  CPs t67 = {c1, c2, c3, c4, c5,         c8};
  CPs p7  = {c1, c2, c3, c4, c5, c6,     c8};
  CPs t78 = {c1, c2, c3, c4, c5, c6,       };
  CPs p8  = {c1, c2, c3, c4, c5, c6, c7    };
  CPs t89 = {c1, c2, c3, c4, c5, c6, c7    };

  // This gait moves four legs at a time (alternating tetrapod).
  CPs t0a = {    c2,     c4,     c6,     c8};
  CPs pa  = {    c2,     c4,     c6,     c8};
  CPs tab = {                              };
  CPs pb  = {c1,     c3,     c5,     c7    };
  CPs tb0 = {c1,     c3,     c5,     c7    };

  // Define contact points for each phase, transition contact points,
  // and phase durations.
  // Gait one leg at a time: 
  vector<CPs> phase_cps =   {p0, p1, p2, p3, p4, p5, p6, p7, p8, p0};
  vector<CPs> trans_cps =   {t01, t12, t23, t34, t45, t56, t67, t78, t89};
  vector<int> phase_steps = {50, 60, 50, 60, 50, 60, 50, 60, 50, 60};

  // Alternating Tetrapod:
  // vector<CPs> phase_cps =   {p0, pa, pb, pa, pb, pa, pb, p0};
  // vector<CPs> trans_cps =   {t0a, tab, tab, tab, tab, tab, tb0};
  // vector<int> phase_steps = {50, 60, 50, 60, 50, 60, 50, 60};

  // Define noise to be added to initial values, desired timestep duration,
  // vector of link name strings, robot model for each phase, and
  // phase transition initial values.
  double gaussian_noise = 1e-5;
  double dt_des = 1. / 240.;
  vector<string> links = {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"};
  vector<Robot> robots(phase_cps.size(), spider);
  vector<Values> transition_graph_init;

  // Initialize graph builder with desired dynamics constraint stds.
  auto graph_builder = gtdynamics::InitializeGraphBuilder(dynamics_model_6, dynamics_model_3,
                                         dynamics_model_1, objectives_model_6,
                                         objectives_model_3, objectives_model_1);

  // Create factor graph given env and contact information.
  auto graph = gtdynamics::CreateFactorGraph(gravity, mu, GROUND_HEIGHT, 
                sigma_objectives, dynamics_model_6, dynamics_model_1,
                objectives_model_6, objectives_model_3, 
                objectives_model_1, graph_builder, transition_graph_init,
                spider, links,
                dt_des, gaussian_noise, robots, phase_cps,
                trans_cps, phase_steps, c1.contact_point);

  // Initialize trajectory solution and optimize.
  auto results = gtdynamics::Optimize(graph, transition_graph_init, dt_des, 
                gaussian_noise, robots, phase_cps, phase_steps);

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  gtdynamics::CreateTrajFile(spider, phase_steps, results);

  return 0;
}
