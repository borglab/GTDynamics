/**
 * @file  Actuator.h
 * @brief manipulator links
 * @Author: Yetong Zhang
 */

#pragma once

#include <PneumaticActuatorFactors.h>
#include <DynamicsGraph.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

#include <vector>

namespace robot
{

/* Shorthand for ti_j, time to open valve for joint j. */
gtsam::LabeledSymbol StartTimeKey(int j)
{
  return gtsam::LabeledSymbol('t', 2, j);
}

/* Shorthand for P_j_t, pressure for joint j at step t. */
gtsam::LabeledSymbol PressureKey(int j, int t)
{
  return gtsam::LabeledSymbol('P', j, t);
}

gtsam::LabeledSymbol InitPressureKey(int j)
{
  return gtsam::LabeledSymbol('P', j, 1000000);
}

/* Shorthand for x_j_t, contraction for joint j at step t. */
gtsam::LabeledSymbol ContractionKey(int j, int t)
{
  return gtsam::LabeledSymbol('x', j, t);
}

/* Shorthand for f_j_t, contraction for joint j at step t. */
gtsam::LabeledSymbol ForceKey(int j, int t)
{
  return gtsam::LabeledSymbol('f', j, t);
}






/// Pneumatic actuator used in jumping robot
class PneumaticActuator
{
private:
  int j_;
  std::vector<double> pressureCoefficients_;
  std::vector<double> pneumaticCoefficients_;
  double springConstant_;
  double pulleyRadius_;
  double restAngle_;
  bool flipped_;

public:
  /**
   * Construct pneumatic actuator
   * Keyword arguments:
        pressureCoefficients        -- coefficients for PressureFactor
        pneumaticCoefficients       -- coefficients for PneumaticActuatorFactor
        springConstant   -- spring coefficient k, used in ActuatorJointFactor
        pulleyRadius    -- radius of pulley r, used in ActuatorJointFactor
        restAngle       -- angle without contraction, in ActuatorJointFactor
   */
  PneumaticActuator(const int joint_index,
                    const std::vector<double> &pressureCoefficients,
                    const std::vector<double> &pneumaticCoefficients,
                    const double springConstant, const double pulleyRadius,
                    const double restAngle,
                    const double initialPressure = 0,
                    const double startTime = 0, const bool flipped = false)
      : j_(joint_index), pressureCoefficients_(pressureCoefficients),
        pneumaticCoefficients_(pneumaticCoefficients),
        springConstant_(springConstant), pulleyRadius_(pulleyRadius),
        restAngle_(restAngle), flipped_(flipped) {}

  // double multDouble(const double& d1, const double& d2, gtsam::OptionalJacobian<1, 1> H1,
  //                 gtsam::OptionalJacobian<1, 1> H2) {
  //   if (H1)
  //     *H1 = gtsam::I_1x1 * d2;
  //   if (H2)
  //     *H2 = gtsam::I_1x1 * d1;
  //   return d1 * d2;
  // }

  // double subPressureCoeff(const double& d1, gtsam::OptionalJacobian<1, 1> H1) {
  //   if (H1)
  //     *H1 = gtsam::I_1x1;
  //   return d1 - pressureCoefficients_[0];
  // }

  // double sigmoid(const double& d1, gtsam::OptionalJacobian<1, 1> H1) {
  //   double result = 1.0/(1+exp(d1));
  //   if (H1)
  //     *H1 = gtsam::I_1x1 * result * (1-result));
  //   return result;
  // }

  // /** Build factor graph for pneumatic actuator.
  //  * Keyword arguments:
  //  *   j          -- joint index
  //  *   t          -- index of time step
  //  * Return nonlinear factor graph of actuator related factors
  //  */
  // gtsam::NonlinearFactorGraph actuatorFG(const int j, const int t) const
  // {
  //   using namespace gtsam;
  //   ExpressionFactorGraph graph;
  //   Double_ t_c = Double_(TimeKey(t));          // expression for current time
  //   Double_ t_i = Double_(StartTimeKey(j));     // expression for time of opening the valve
  //   Double_ P_i = Double_(InitPressureKey(j));  // expression for initial pressure
  //   Double_ P_c = Double_(PressureKey(j, t));   // expression for current pressure

  //   // add pressure vs. time factor
  //   Double_ delta_t = t_c - t_i;
  //   Double_ x_tmp(subPressureCoeff, delta_t);
  //   Double_ poly3(x_tmp);
  //   Double_ ratio(sigmiod, poly);
  //   graph.addExpressionFactor(P_i * ratio - P_c, double(0), gtsam::noiseModel::Constrained::All(1));

  //   // add force vs. pressure vs. displacement factor
  //   graph.addExpressionFactor(double(0), gtsam::noiseModel::Constrained::All(1));

  //   // add force vs. displacement vs. joint angle factor
  //   graph.addExpressionFactor(double(0), gtsam::noiseModel::Constrained::All(1));

  //   return graph;
  // }

  /** Build factor graph for pneumatic actuator.
   * Keyword arguments:
   *   j          -- joint index
   *   t          -- index of time step
   * Return Gaussian factor graph of actuator related factors
   */
  gtsam::NonlinearFactorGraph
  actuatorFactorGraph(const int t) const
  {
    gtsam::Key t_i_key = StartTimeKey(j_);     // time of opening the valve
    gtsam::Key t_c_key = TimeKey(t);          // current time
    gtsam::Key P_i_key = InitPressureKey(j_);  // initial pressure
    gtsam::Key P_c_key = PressureKey(j_, t);   // current pressure
    gtsam::Key x_key = ContractionKey(j_, t);  // contraction length
    gtsam::Key f_key = ForceKey(j_, t);        // force
    gtsam::Key q_key = JointAngleKey(j_, t);   // joint angle
    gtsam::Key torque_key = TorqueKey(j_, t);  // torque

    gtsam::NonlinearFactorGraph graph;
    graph.add(PressureFactor(t_i_key, t_c_key, P_i_key, P_c_key,
                             gtsam::noiseModel::Constrained::All(1),
                             pressureCoefficients_));

    graph.add(PneumaticActuatorFactor(
        x_key, P_c_key, f_key,
        gtsam::noiseModel::Constrained::All(1), pneumaticCoefficients_));

    graph.add(JointBalanceFactor(x_key, q_key, f_key,
                                  gtsam::noiseModel::Constrained::All(1),
                                  springConstant_, pulleyRadius_, restAngle_, flipped_));

    gtsam::Double_ f_expr = gtsam::Double_(f_key);
    gtsam::Double_ torque_expr = gtsam::Double_(torque_key);
    
    if (flipped_) {
      gtsam::ExpressionFactor<double> torque_factor(gtsam::noiseModel::Constrained::All(1), double(0), pulleyRadius_ * f_expr - torque_expr);
      graph.add(torque_factor);
    }
    else {
      gtsam::ExpressionFactor<double> torque_factor(gtsam::noiseModel::Constrained::All(1), double(0), -pulleyRadius_ * f_expr - torque_expr);
      graph.add(torque_factor);
    }
    return graph;
  }

  gtsam::Values computeResult(const int t, const double angle, const double delta_t, const double init_pressure) const {
    // construct a factor graph
    double start_time = 0;
    double current_time = delta_t;
    auto graph = actuatorFactorGraph(t);
    gtsam::Key t_i_key = StartTimeKey(j_);     // time of opening the valve
    gtsam::Key t_c_key = TimeKey(t);          // current time
    gtsam::Key P_i_key = InitPressureKey(j_);  // initial pressure
    gtsam::Key P_c_key = PressureKey(j_, t);   // current pressure
    gtsam::Key x_key = ContractionKey(j_, t);  // contraction length
    gtsam::Key f_key = ForceKey(j_, t);        // force
    gtsam::Key q_key = JointAngleKey(j_, t);   // joint angle
    gtsam::Key torque_key = TorqueKey(j_, t);  // torque

    graph.add(
        gtsam::PriorFactor<double>(P_i_key, init_pressure,
                                   gtsam::noiseModel::Constrained::All(1)));
    graph.add(
        gtsam::PriorFactor<double>(t_i_key, start_time,
                                   gtsam::noiseModel::Constrained::All(1)));
    graph.add(
        gtsam::PriorFactor<double>(q_key, angle, 
                                   gtsam::noiseModel::Constrained::All(1)));
    graph.add(
        gtsam::PriorFactor<double>(t_c_key, current_time, 
                                   gtsam::noiseModel::Constrained::All(1)));

    // solve the factor graph
    gtsam::Values init_values;
    init_values.insert(P_i_key, init_pressure);
    init_values.insert(t_i_key, start_time);
    init_values.insert(q_key, angle);
    init_values.insert(x_key, double(0));
    init_values.insert(f_key, double(0));
    init_values.insert(P_c_key, init_pressure);
    init_values.insert(t_c_key, current_time);
    init_values.insert(torque_key, double(0));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values);
    optimizer.optimize();

    // extract result
    return optimizer.values();
  }

  int j() const {
    return j_;
  }

  /// calculate torque given joint angle
  double calculateTorque(const double angle, const double delta_t, const double init_pressure) const
  {
    gtsam::Values results = computeResult(1, angle, delta_t, init_pressure);
    auto torque = results.atDouble(TorqueKey(j_, 1));
    return torque;
    // if (flipped_)
    // {
    //   return force * pulleyRadius_;
    // }
    // else
    // {
    //   return -force * pulleyRadius_;
    // }
  }
};
} // namespace robot
