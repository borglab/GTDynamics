/**
 * @file  MotionPlanner.cpp
 * @brief robot arm motion planner using nonlinear factor graph
 * @Author: Mandy Xie
 */

#include <MotionPlanner.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

Values MotionPlanner::factorGraphOptimization(const NonlinearFactorGraph &graph,
                                              const Values &init_values) const {
  std::shared_ptr<gtsam::NonlinearOptimizer> optimizer;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> parameters;

  // set parameters for optimizer
  if (opt_.opt_type == OptimizerSetting::GaussNewton) {
    parameters = std::make_shared<gtsam::GaussNewtonParams>();
  } else if (opt_.opt_type == OptimizerSetting::LM) {
    parameters = std::make_shared<gtsam::LevenbergMarquardtParams>();
    dynamic_cast<LevenbergMarquardtParams *>(parameters.get())
        ->setlambdaInitial(1e-2);
  } else if (opt_.opt_type == OptimizerSetting::Dogleg) {
    parameters = std::make_shared<gtsam::DoglegParams>();
  }

  // common settings for parameters
  parameters->setMaxIterations(opt_.max_iter);
  parameters->setRelativeErrorTol(opt_.rel_thresh);
  if (opt_.opt_verbosity >= OptimizerSetting::Error) {
    parameters->setVerbosity("ERROR");
  }

  if (opt_.opt_type == OptimizerSetting::GaussNewton) {
    optimizer = std::make_shared<gtsam::GaussNewtonOptimizer>(
        graph, init_values,
        *(dynamic_cast<GaussNewtonParams *>(parameters.get())));
  } else if (opt_.opt_type == OptimizerSetting::LM) {
    optimizer = std::make_shared<gtsam::LevenbergMarquardtOptimizer>(
        graph, init_values,
        *(dynamic_cast<LevenbergMarquardtParams *>(parameters.get())));
  } else if (opt_.opt_type == OptimizerSetting::Dogleg) {
    optimizer = std::make_shared<gtsam::DoglegOptimizer>(
        graph, init_values, *(dynamic_cast<DoglegParams *>(parameters.get())));
  }
  optimizer->optimize();
  return optimizer->values();
}

vector<Vector> MotionPlanner::extractTrajectoryQ(const Values &results,
                                                 int dof) const {
  vector<Vector> q_trajectory;

  for (int i = 0; i < opt_.total_step; ++i) {
    Vector joint_coordinates(dof);
    for (int j = 1; j <= dof; ++j) {
      joint_coordinates[j - 1] = results.atDouble(JointAngleKey(j, i));
    }
    q_trajectory.push_back(joint_coordinates);
  }
  return q_trajectory;
}

vector<Vector> MotionPlanner::extractTrajectoryTorque(const Values &results,
                                                      int dof) const {
  vector<Vector> torque_trajectory;

  for (int i = 0; i < opt_.total_step; ++i) {
    Vector torque(dof);
    for (int j = 1; j <= dof; ++j) {
      torque[j - 1] = results.atDouble(TorqueKey(j, i));
    }
    torque_trajectory.push_back(torque);
  }
  return torque_trajectory;
}
