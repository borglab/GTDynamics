/**
 * @file  Actuator.h
 * @brief manipulator links
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

#include "gtdynamics/dynamics/DynamicsGraph.h"
// #include "gtdynamics/utils/JsonSaver.h"

namespace gtdynamics {

/* Shorthand for ti_j, time to open valve for joint j. */
inline DynamicsSymbol StartTimeKey(int j) {
  return DynamicsSymbol::JointSymbol("ti", j, 0);
}

/* Shorthand for P_j_t, pressure for joint j at step t. */
inline DynamicsSymbol PressureKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("P", j, t);
}

/* Shorthand for P_j_t, pressure for joint j at step t. */
inline DynamicsSymbol SourcePressureKey(int t) {
  return DynamicsSymbol::SimpleSymbol("Ps", t);
}

inline DynamicsSymbol InitSourcePressureKey() {
  return DynamicsSymbol::SimpleSymbol("Pi", 0);
}

/* Shorthand for x_j_t, contraction for joint j at step t. */
inline DynamicsSymbol ContractionKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("dx", j, t);
}

/* Shorthand for f_j_t, contraction for joint j at step t. */
inline DynamicsSymbol ForceKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("fr", j, t);
}

inline DynamicsSymbol MassKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("m", j, t);
}

inline DynamicsSymbol SourceMassKey(int t) {
  return DynamicsSymbol::SimpleSymbol("ms", t);
}

inline DynamicsSymbol MassRateOpenKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("mo", j, t);
}

inline DynamicsSymbol MassRateActualKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("md", j, t);
}

inline DynamicsSymbol VolumeKey(int j, int t) {
  return DynamicsSymbol::JointSymbol("vo", j, t);
}

inline DynamicsSymbol SourceVolumeKey() {
  return DynamicsSymbol::SimpleSymbol("vs", 0);
}

inline DynamicsSymbol ValveOpenTimeKey(int j) {
  return DynamicsSymbol::JointSymbol("to", j, 0);
}

inline DynamicsSymbol ValveCloseTimeKey(int j) {
  return DynamicsSymbol::JointSymbol("tc", j, 0);
}

struct PriorValues {
  double q;
  double v;
  double Ps;
  double m;
  double t;
  double to;
  double tc;
};

gtsam::Values optimize_LMQR(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& init_values);

/// Pneumatic actuator used in jumping robot
class PneumaticActuator {
 public:
  struct Params {
    std::string joint_name;         // joint name
    int j;                          // joint index
    // std::vector<double> x0_coeffs;
    // std::vector<double> f0_coeffs;
    // std::vector<double> k_coeffs;
    // std::vector<double> p_coeffs;   // coefficients for PressureFactor
    double kt;                      // spring coefficient for tendon spring
    double ka;                      // spring coefficient for antagonistic spring
    double q_rest;                  // nominal angle without contraction
    double q_anta_limit;            // joint limit to activate antagonistic spring
    double r;                       // radius of pulley r
    double b;                       // damping coefficient
    bool positive;                  // actuator configuration

    double Rs;
    double T;
    double D;
    double L;
    double mu;
    double epsilon;
    double ct;
  };

 private:
  Params params_;

 public:

  gtsam::noiseModel::Base::shared_ptr pressure_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::noiseModel::Base::shared_ptr force_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.01);
  gtsam::noiseModel::Base::shared_ptr balance_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
  gtsam::noiseModel::Base::shared_ptr torque_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.01);
  gtsam::noiseModel::Base::shared_ptr prior_pressure_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.01);
  gtsam::noiseModel::Base::shared_ptr prior_valve_t_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.0001);
  gtsam::noiseModel::Base::shared_ptr prior_q_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
  gtsam::noiseModel::Base::shared_ptr prior_time_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.0001);
  gtsam::noiseModel::Base::shared_ptr prior_v_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.001);  

  gtsam::noiseModel::Base::shared_ptr gass_law_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 0.0001);  
  gtsam::noiseModel::Base::shared_ptr mass_rate_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);  
  gtsam::noiseModel::Base::shared_ptr volume_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 1e-7);  
  gtsam::noiseModel::Base::shared_ptr prior_m_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 1e-7);  
  gtsam::noiseModel::Base::shared_ptr m_col_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, 1e-7);  

  gtsam::noiseModel::Base::shared_ptr mass_rate_obj_model = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

  /**
   * Construct pneumatic actuator
   */

  PneumaticActuator() {
    Params params;
    params.joint_name = "j1";
    params.j = 1;                          // joint index
    params.kt = 8200;                      // spring coefficient for tendon spring
    params.ka = 2.0;                      // spring coefficient for antagonistic spring
    params.q_rest = 0;                  // nominal angle without contraction
    params.q_anta_limit =0;            // joint limit to activate antagonistic spring
    params.r = 0.04;                       // radius of pulley r
    params.b = 0.05;                       // damping coefficient
    params.positive = false;                  // actuator configuration

    params.Rs = 287.0550;
    params.T = 296.15;
    params.D = 0.1575 * 0.0254;
    params.L = 74 * 0.0254;
    params.mu = 1.8377e-5;
    params.epsilon = 1e-5;
    params.ct = 1e-3;
    params_ = params;    
  }

  PneumaticActuator(Params& params)
      : params_(params) {}

  /** Build factor graph for pneumatic actuator.
   * Keyword arguments:
   *   t          -- index of time step
   * Return Gaussian factor graph of actuator related factors
   */
  gtsam::NonlinearFactorGraph actuatorFactorGraph(const int t) const;

  gtsam::NonlinearFactorGraph actuatorPriorGraph(const int t,  const PriorValues& prior_values) const;

  gtsam::Values computeResult(const int t, const PriorValues& prior_values,
        const gtsam::Values& previous_values) const;

  gtsam::NonlinearFactorGraph sourceFactorGraph(const int t) const;

  gtsam::NonlinearFactorGraph sourcePriorGraph(const int t, const double ms, const double vs) const;

  gtsam::Values actuatorInitValues(const int t, const PriorValues& prior_values) const;

  gtsam::Values sourceInitValues(const int t,  const double ms, const double vs) const;

  gtsam::Values computeSourceResult(const int t, const double ms, const double vs) const;

  int j() const { return params_.j; }

  std::string name() const { return params_.joint_name; }

  const Params& params() const {return params_; }

  PriorValues priorValues() const{
    PriorValues prior_values;
    prior_values.q = 0.0;
    prior_values.m = 7.873172488131229e-05;
    prior_values.v = 0.0;
    prior_values.Ps = 65.0 * 6.89476;
    prior_values.t = 0.0;
    prior_values.to = 0.0;
    prior_values.tc = 1.0;
    return prior_values;
  }


  // /// calculate torque given joint angle
  // double calculateTorque(const double angle, const PriorValues prior_values,
  //       boost::optional<gtsam::Values> previous_values = boost::none) const {
  //   gtsam::Values results = computeResult(1, angle, velocity, 0, delta_t, init_pressure);
  //   auto torque = results.atDouble(TorqueKey(params_.j, 1));
  //   return torque;
  // }


  // // TODO: a buggy test for LM/PDL optimizer:
  // //      current_t:  1.4
  // //      start_t:    -1
  // //      init_P:     240
  // //      q:          -4.69781
  // //      k:          5000
  // //      contract:    false
  // //      radius:     0.02
  // //      rest angle  2.44346

  // gtsam::Values computeResult(const int t, const double angle, const double
  // start_time, const double current_time, const double init_pressure) const
  // {
  //   // construct a factor graph
  //   gtsam::NonlinearFactorGraph graph;
  //   gtsam::Key t_i_key = StartTimeKey(j_);    // time of opening the valve
  //   gtsam::Key t_c_key = TimeKey(t);          // current time
  //   gtsam::Key P_i_key = InitPressureKey(j_); // initial pressure
  //   gtsam::Key P_c_key = PressureKey(j_, t);  // current pressure
  //   gtsam::Key x_key = ContractionKey(j_, t); // contraction length
  //   gtsam::Key f_key = ForceKey(j_, t);       // force
  //   gtsam::Key q_key = JointAngleKey(j_, t);  // joint angle
  //   gtsam::Key torque_key = TorqueKey(j_, t); // torque

  //   // calculate current pressure
  //   PressureFactor pressure_factor(t_i_key, t_c_key, P_i_key, P_c_key,
  //                                  pressure_cost_model,
  //                                  p_coeffs_);
  //   double current_pressure = pressure_factor.evaluateError(start_time,
  //   current_time, init_pressure, 0)[0];

  //   // balance factor
  //   gtsam::Double_ x_expr(x_key);
  //   gtsam::Double_ f_expr(f_key);
  //   double cm_to_m = 0.01;
  //   double rhs = kt_ * (angle - q_rest_) * r_;
  //   if (positive_)
  //   {
  //     gtsam::ExpressionFactor<double> balance_factor(balance_cost_model, rhs,
  //                                                    kt_ * cm_to_m * x_expr -
  //                                                    f_expr);
  //     graph.add(balance_factor);
  //   }
  //   else
  //   {
  //     gtsam::ExpressionFactor<double> balance_factor(balance_cost_model,
  //     -rhs,
  //                                                    kt_ * cm_to_m * x_expr -
  //                                                    f_expr);
  //     graph.add(balance_factor);
  //   }

  //   SimpleActuatorFactor simple_acutator_factor(
  //       x_key, f_key,
  //       force_cost_model, current_pressure, pneumatic_coeffs_);
  //   graph.add(simple_acutator_factor);

  //   // solve the factor graph
  //   gtsam::Values init_values;
  //   init_values.insert(x_key, double(2));
  //   init_values.insert(f_key, double(0));

  //   gtsam::LevenbergMarquardtParams params;
  //   // params.setVerbosityLM("SUMMARY");
  //   params.setLinearSolverType("MULTIFRONTAL_QR");
  //   gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  //   gtsam::Values result = optimizer.optimize();

  //   // if (graph.error(result) > 0.0001)
  //   if (true)
  //   {
  //     std::cout << "error:\t" << graph.error(result) << "\n";

  //     std::ofstream json_file;
  //     json_file.open("../../visualization/factor_graph.json");
  //     gtsam::JsonSaver::SaveFactorGraph(graph, json_file, result);
  //     json_file.close();

  //     double total_ext = (angle-q_rest_)*r_;

  //     std::cout << "binary search for gt value...\n";

  //     // xc is the contraction of actuator in meters
  //     std::cout <<
  //     "left_xc\t\tmid_xc\t\tright_cx\tf_spring\t\tf_actuator\terror\n";
  //     double left_xc = -4 * total_ext;
  //     double right_xc = total_ext;
  //     for (int i = 0; i<20; i++)
  //     {
  //       double mid_xc = (left_xc + right_xc) / 2;
  //       double f_spring = kt_ * (total_ext - mid_xc);
  //       double f_actuator = simple_acutator_factor.evaluateError(-mid_xc*100,
  //       0)[0]; std::cout << left_xc << "\t" << mid_xc << "\t" << right_xc <<
  //       "\t" << f_spring
  //       << '\t' << f_actuator << "\t" << f_spring - f_actuator << "\n";
  //       if (f_spring > f_actuator)
  //       {
  //         left_xc = mid_xc;
  //       }
  //       else {
  //         right_xc = mid_xc;
  //       }
  //     }

  //     gtsam::Values gt_values;
  //     double force = kt_ * (total_ext - left_xc);
  //     gt_values.insert(f_key, force);
  //     gt_values.insert(x_key, -left_xc*100);
  //     std::cout << "gt_error:\t" << graph.error(gt_values) << "\n";

  //     std::cout << "further optimization from gt results:\n";
  //     params.setVerbosityLM("SUMMARY");
  //     gtsam::LevenbergMarquardtOptimizer optimizer1(graph, gt_values,
  //     params); result = optimizer1.optimize(); std::cout << "error:\t" <<
  //     graph.error(result) << "\n";

  //     std::cout << "show cost surface:\n";
  //     std::ofstream cost_file, cost1_file, cost2_file;
  //     cost_file.open("../../visualization/joint_angles/cost.csv");
  //     cost1_file.open("../../visualization/joint_angles/cost1.csv");
  //     cost2_file.open("../../visualization/joint_angles/cost2.csv");

  //     double x_min = 0, x_max = 2, x_step = 0.01;
  //     double f_min = -3, f_max = 10, f_step = 0.01;
  //     int x_steps = int((x_max - x_min + 0.0000001) / x_step);
  //     int f_steps = int((f_max - f_min + 0.0000001) / f_step);

  //     for (int x_i = 0; x_i <= x_steps; x_i++)
  //     {
  //       for (int f_i =0; f_i <= f_steps; f_i++)
  //       {
  //         double x = x_min + x_i * x_step;
  //         double f = f_min + f_i * f_step;
  //         gtsam::Values values;
  //         values.insert(x_key, x);
  //         values.insert(f_key, f);
  //         cost_file << graph.error(values);
  //         cost1_file << graph[0]->error(values);
  //         cost2_file << graph[1]->error(values);
  //         if (f_i<f_steps)
  //         {
  //           cost_file << ", ";
  //           cost1_file << ", ";
  //           cost2_file << ", ";
  //         }
  //       }
  //       cost_file << std::endl;
  //       cost1_file << std::endl;
  //       cost2_file << std::endl;
  //     }
  //     cost_file.close();
  //     cost1_file.close();
  //     cost2_file.close();

  //     if (graph.error(result) > 1) {
  //       throw std::runtime_error("optimizing for pneumatic actuator graph
  //       fails");
  //     }

  //   }

  //   double force = result.atDouble(f_key);
  //   double torque = positive_? force * r_ : -force * r_;
  //   result.insert(torque_key, torque);
  //   result.insert(t_i_key, start_time);
  //   result.insert(P_i_key, init_pressure);
  //   result.insert(t_c_key, current_time);
  //   result.insert(q_key, angle);

  //   return result;
  // }


};  // PneumaticActuator
}  // namespace gtdynamics
