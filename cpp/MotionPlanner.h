/**
 * @file  MotionPlanner.h
 * @brief robot arm motion planner using nonlinear factor graph
 * @Author: Mandy Xie
 */
#pragma once

#include <Arm.h>
#include <NonlinearFactors.h>
#include <OptimizerSetting.h>
#include <utils.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <boost/optional.hpp>

namespace manipulator {

/* Shorthand for F_j_i, for wrenches on the j-th link at time i. */
gtsam::LabeledSymbol WrenchKey(int j, int i) {
  return gtsam::LabeledSymbol('F', j, i);
}

/* Shorthand for T_j_i, for torque on the j-th link at time i. */
gtsam::LabeledSymbol TorqueKey(int j, int i) {
  return gtsam::LabeledSymbol('T', j, i);
}

/* Shorthand for p_j_i, for COM pose on the j-th link at time i. */
gtsam::LabeledSymbol PoseKey(int j, int i) {
  return gtsam::LabeledSymbol('p', j, i);
}

/* Shorthand for V_j_i, for 6D link twist vector on the j-th link. */
gtsam::LabeledSymbol TwistKey(int j, int i) {
  return gtsam::LabeledSymbol('V', j, i);
}

/* Shorthand for A_j_i, for twist accelerations on the j-th link at time i. */
gtsam::LabeledSymbol TwistAccelKey(int j, int i) {
  return gtsam::LabeledSymbol('A', j, i);
}

/* Shorthand for q_j_i, for j-th joint angle at time i. */
gtsam::LabeledSymbol JointAngleKey(int j, int i) {
  return gtsam::LabeledSymbol('q', j, i);
}

/* Shorthand for v_j_i, for j-th joint velocity at time i. */
gtsam::LabeledSymbol JointVelKey(int j, int i) {
  return gtsam::LabeledSymbol('v', j, i);
}

/* Shorthand for a_j_i, for j-th joint acceleration at time i. */
gtsam::LabeledSymbol JointAccelKey(int j, int i) {
  return gtsam::LabeledSymbol('a', j, i);
}

/**
 * MotionPlanner is a class which builds a factor graph to do kinodynamic motion
 * planning
 */
class MotionPlanner {
 private:
  OptimizerSetting opt_;

 public:
  /**Constructor
   * Keyword arguments:
      opt  -- optimizer setting
   */
  MotionPlanner(OptimizerSetting &opt) : opt_(opt) {}
  ~MotionPlanner() {}

  /** return nonlinear factor graph of all factors
      Keyword arguments:
          robot -- robotic arm
          pose goal -- pose goal of manipulator end effector
          q_init -- initial value for joint angles
          cartesian_path (optional) -- cartesian path for end effector to follow
          gravity (optional) -- gravitatianl acceleration
          sdf (optional) -- Signed Distance Field for collision check
   */
  template <typename Type>
  gtsam::NonlinearFactorGraph motionPlanningFactorGraph(
      Arm<Type> &robot, gtsam::Pose3 &pose_goal, const gtsam::Vector &q_init,
      boost::optional<std::vector<gtsam::Pose3> &> cartesian_path = boost::none,
      boost::optional<gtsam::Vector3 &> gravity = boost::none,
      boost::optional<SignedDistanceField &> sdf = boost::none) const {
    using namespace gtsam;
    double delta_t = opt_.total_time / opt_.total_step;

    auto dof = robot.numLinks();
    // get robot jMi list at rest
    auto jMi = robot.jTi_list(Vector::Zero(dof));
    // get base pose in world frame
    auto base_pose = robot.base();
    // get link COM pose at start
    auto poses = robot.comFrames();
    // get robot screw_axes for all links
    auto screw_axes = robot.screwAxes();

    NonlinearFactorGraph graph;

    Vector6 base_twist, base_acceleration, external_wrench;
    base_twist.setZero();
    base_acceleration.setZero();
    external_wrench.setZero();

    for (int j = 1; j <= dof; ++j) {
      graph.add(PriorFactor<double>(JointAngleKey(j, 0), q_init[j - 1],
                                    opt_.q_cost_model));
      graph.add(PriorFactor<double>(JointVelKey(j, 0), 0, opt_.qv_cost_model));
    }

    for (int i = 0; i < opt_.total_step; ++i) {
      // add base pose factor
      graph.add(BasePoseFactor(PoseKey(0, i), opt_.bp_cost_model, base_pose));
      // add base twist factor
      graph.add(
          BaseTwistFactor(TwistKey(0, i), opt_.bv_cost_model, base_twist));
      // add base acceleration factor
      graph.add(BaseTwistAccelFactor(TwistAccelKey(0, i), opt_.ba_cost_model,
                                     base_acceleration));
      for (int j = 1; j <= dof; ++j) {
        // add joint angle limit factors
        graph.add(JointLimitFactor(JointAngleKey(j, i), opt_.jl_cost_model,
                                   robot.link(j - 1).jointLowerLimit(),
                                   robot.link(j - 1).jointUpperLimit(),
                                   robot.link(j - 1).jointLimitThreshold()));

        // add joint velocity limit factors
        graph.add(JointLimitFactor(JointVelKey(j, i), opt_.jl_cost_model,
                                   -robot.link(j - 1).velocityLimit(),
                                   robot.link(j - 1).velocityLimit(),
                                   robot.link(j - 1).velocityLimitThreshold()));

        // add joint acceleration limit factors
        graph.add(
            JointLimitFactor(JointAccelKey(j, i), opt_.jl_cost_model,
                             -robot.link(j - 1).accelerationLimit(),
                             robot.link(j - 1).accelerationLimit(),
                             robot.link(j - 1).accelerationLimitThreshold()));

        // add joint torque limit factors
        graph.add(JointLimitFactor(TorqueKey(j, i), opt_.jl_cost_model,
                                   -robot.link(j - 1).torqueLimit(),
                                   robot.link(j - 1).torqueLimit(),
                                   robot.link(j - 1).torqueLimitThreshold()));
        // add pose factor
        graph.add(PoseFactor(PoseKey(j - 1, i), PoseKey(j, i),
                             JointAngleKey(j, i), opt_.p_cost_model, jMi[j - 1],
                             screw_axes[j - 1]));
        // add twist factor
        graph.add(TwistFactor(TwistKey(j - 1, i), TwistKey(j, i),
                              JointAngleKey(j, i), JointVelKey(j, i),
                              opt_.v_cost_model, jMi[j - 1],
                              screw_axes[j - 1]));
        // add twist acceleration factor
        graph.add(TwistAccelFactor(
            TwistKey(j, i), TwistAccelKey(j - 1, i), TwistAccelKey(j, i),
            JointAngleKey(j, i), JointVelKey(j, i), JointAccelKey(j, i),
            opt_.a_cost_model, jMi[j - 1], screw_axes[j - 1]));
        // add wrench factor
        if (j < dof) {
          graph.add(WrenchFactor(
              TwistKey(j, i), TwistAccelKey(j, i), WrenchKey(j, i),
              WrenchKey(j + 1, i), PoseKey(j, i), JointAngleKey(j + 1, i),
              opt_.f_cost_model, jMi[j], robot.link(j - 1).inertiaMatrix(),
              screw_axes[j], gravity));
        }
        // add torque factor
        graph.add(TorqueFactor(WrenchKey(j, i), TorqueKey(j, i),
                               opt_.t_cost_model, screw_axes[j - 1]));

        // add GP prior factor
        if (i < opt_.total_step - 1) {
          graph.add(GaussianProcessPriorFactor(
              JointAngleKey(j, i), JointVelKey(j, i), JointAccelKey(j, i),
              JointAngleKey(j, i + 1), JointVelKey(j, i + 1),
              JointAccelKey(j, i + 1), opt_.Qc_model, delta_t));
        }

        // add obstacle factor
        if (sdf) {
          auto length = robot.link(j - 1).length();
          if (length > 0) {
            int num = std::min((int)(length / opt_.radius), 1);
            auto obs_cost_model =
                noiseModel::Isotropic::Sigma(num, opt_.obsSigma);
            auto sphere_centers = sphereCenters(length, opt_.radius, num);
            graph.add(ObstacleSDFFactor(PoseKey(j, i), obs_cost_model,
                                        opt_.epsilon, *sdf, opt_.radius,
                                        sphere_centers));
          }
        }
      }
      // add tool wrench factor
      graph.add(ToolWrenchFactor(
          TwistKey(dof, i), TwistAccelKey(dof, i), WrenchKey(dof, i),
          PoseKey(dof, i), opt_.tf_cost_model, jMi[dof],
          robot.link(dof - 1).inertiaMatrix(), external_wrench, gravity));

      // follow cartesian path
      if (cartesian_path) {
        // add tool pose factor (which is pose)
        graph.add(ToolPoseFactor(PoseKey(dof, i), opt_.tp_cost_model, jMi[dof],
                                 (*cartesian_path)[i]));
      }
    }
    // add tool pose factor (which is the pose goal)
    graph.add(ToolPoseFactor(PoseKey(dof, opt_.total_step - 1),
                             opt_.tp_cost_model, jMi[dof], pose_goal));

    return graph;
  }

  /** initialization factor graph, return initial values for optimization
      Keyword arguments:
          robot -- robotic arm
          pose goal -- pose goal of manipulator end effector
          q_init -- initial value for joint angles
          cartesian_path (optional) -- cartesian path end effector needs to
     follow
   */
  template <typename Type>
  gtsam::Values factorGraphInitialization(
      Arm<Type> &robot, gtsam::Pose3 &pose_goal, const gtsam::Vector &q_init,
      boost::optional<std::vector<gtsam::Pose3> &> cartesian_path =
          boost::none) const {
    using namespace gtsam;
    // initial joint angle trajectory is set as a straight line
    Values init_values;

    int dof = robot.numLinks();
    int total_step = opt_.total_step;
    Vector start_q = q_init, end_q = Vector::Zero(dof);
    // call inverse kinematics to get joint angles for pose start and goal
    if (cartesian_path) {
      start_q = robot.inverseKinematics(cartesian_path->front(), q_init);
      end_q = robot.inverseKinematics(cartesian_path->back(), q_init);
    } else {
      end_q = robot.inverseKinematics(pose_goal, q_init);
    }

    Vector start_qVel = Vector::Zero(dof);
    Vector end_qVel = 2 * (end_q - start_q) / opt_.total_time;

    Vector start_qAccel = Vector::Zero(dof);
    Vector end_qAccel = 2 * (end_qVel - start_qVel) / opt_.total_time;

    Vector start_torque = Vector::Zero(dof);
    Vector end_torque = Vector::Zero(dof);

    // get base pose in world frame
    auto base_pose = robot.base();

    std::vector<Vector> twists(dof + 1, Vector::Zero(6));
    std::vector<Vector> accels(dof + 1, Vector::Zero(6));
    std::vector<Vector> wrenches(dof, Vector::Zero(6));

    gtsam::Vector qAngle, qVel, qAccel, torque;
    std::vector<Pose3> poses;
    auto intial_q = q_init;
    for (int i = 0; i < total_step; ++i) {
      init_values.insert(PoseKey(0, i), base_pose);
      init_values.insert(TwistKey(0, i), twists[0]);
      init_values.insert(TwistAccelKey(0, i), accels[0]);
      if (cartesian_path) {
        qAngle = robot.inverseKinematics((*cartesian_path)[i], intial_q);
        intial_q = qAngle;
      } else {
        qAngle = q_trajectory(i, total_step, start_q, end_q);
      }
      qVel = q_trajectory(i, total_step, start_qVel, end_qVel);
      qAccel = q_trajectory(i, total_step, start_qAccel, end_qAccel);
      torque = q_trajectory(i, total_step, start_torque, end_torque);
      poses = robot.comFrames(qAngle);
      for (int j = 1; j <= dof; ++j) {
        init_values.insert(JointAngleKey(j, i), qAngle[j - 1]);
        init_values.insert(JointVelKey(j, i), qVel[j - 1]);
        init_values.insert(JointAccelKey(j, i), qAccel[j - 1]);
        init_values.insert(TorqueKey(j, i), torque[j - 1]);
        init_values.insert(PoseKey(j, i), poses[j - 1]);
        init_values.insert(TwistKey(j, i), twists[j]);
        init_values.insert(TwistAccelKey(j, i), accels[j]);
        init_values.insert(WrenchKey(j, i), wrenches[j - 1]);
      }
    }
    return init_values;
  }

  /** optimize factor graph
      Keyword arguments:
          graph -- nonlinear factor graph for motion planning
          init_values -- initial values for optimization
   */
  gtsam::Values factorGraphOptimization(gtsam::NonlinearFactorGraph &graph,
                                        gtsam::Values &init_values) const;

  /** extract joint cooridinates trajactory*/
  std::vector<gtsam::Vector> extractTrajectoryQ(gtsam::Values &results,
                                                int dof) const;

  /** extract joint torque trajactory*/
  std::vector<gtsam::Vector> extractTrajectoryTorque(gtsam::Values &results,
                                                     int dof) const;
};

}  // namespace manipulator
