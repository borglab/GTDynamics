#include "gtdynamics/jumpingrobot/factors/PneumaticActuator.h"

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

#include "gtdynamics/jumpingrobot/factors/PneumaticActuatorFactors.h"
#include "gtdynamics/jumpingrobot/factors/PneumaticFactors.h"

using namespace gtsam;

namespace gtdynamics {

gtsam::NonlinearFactorGraph PneumaticActuator::actuatorFactorGraph(
    const int t) const {
  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key P_key = PressureKey(params_.j, t);     // current pressure
  gtsam::Key x_key = ContractionKey(params_.j, t);  // contraction length, (cm)
  gtsam::Key f_key = ForceKey(params_.j, t);        // force
  gtsam::Key q_key = internal::JointAngleKey(params_.j, t);   // joint angle
  gtsam::Key v_key = internal::JointVelKey(params_.j, t);     // joint velocity
  gtsam::Key torque_key = internal::TorqueKey(params_.j, t);  // torque
  gtsam::Key m_key = MassKey(params_.j, t);
  gtsam::Key mdoto_key = MassRateOpenKey(params_.j, t);
  gtsam::Key mdota_key = MassRateActualKey(params_.j, t);
  gtsam::Key vo_key = VolumeKey(params_.j, t);
  gtsam::Key t_key = TimeKey(t);
  gtsam::Key to_key = ValveOpenTimeKey(params_.j);
  gtsam::Key tc_key = ValveCloseTimeKey(params_.j);

  gtsam::NonlinearFactorGraph graph;

  graph.add(SmoothActuatorFactor(x_key, P_key, f_key, force_cost_model));

  // joint balance factor
  // double cm_to_m = 0.01;

  // gtsam::Double_ f_expr(f_key);
  // gtsam::Double_ q_expr(q_key);
  // gtsam::Double_ x_expr(x_key);
  // gtsam::Double_ torque_expr(torque_key);

  // double rhs = -params_.kt * params_.q_rest * params_.r;
  // if (params_.positive) {
  //   gtsam::ExpressionFactor<double> balance_factor(
  //       balance_cost_model, rhs,
  //       params_.kt * cm_to_m * x_expr - f_expr - params_.kt * params_.r *
  //       q_expr);
  //   graph.add(balance_factor);
  // } else {
  //   gtsam::ExpressionFactor<double> balance_factor(
  //       balance_cost_model, -rhs,
  //       params_.kt * cm_to_m * x_expr - f_expr + params_.kt * params_.r *
  //       q_expr);
  //   graph.add(balance_factor);
  // }

  graph.add(ForceBalanceFactor(x_key, q_key, f_key, balance_cost_model,
                               params_.kt, params_.r, params_.q_rest,
                               params_.positive));

  graph.add(JointTorqueFactor(
      q_key, v_key, f_key, torque_key, torque_cost_model, params_.q_anta_limit,
      params_.ka, params_.r, params_.b, params_.positive));

  graph.add(GasLawFactor(P_key, vo_key, m_key, gass_law_model,
                         params_.Rs * params_.T));

  graph.add(
      ActuatorVolumeFactor(vo_key, x_key, volume_model, params_.D, params_.L));

  graph.add(MassFlowRateFactor(
      P_key, Ps_key, mdoto_key, mass_rate_model, params_.D, params_.L,
      params_.mu, params_.epsilon, 1.0 / (params_.Rs * params_.T)));

  graph.add(ValveControlFactor(t_key, to_key, tc_key, mdoto_key, mdota_key,
                               mass_rate_model, params_.ct));

  return graph;
}

gtsam::NonlinearFactorGraph PneumaticActuator::actuatorPriorGraph(
    const int t, const PriorValues& prior_values) const {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key q_key = internal::JointAngleKey(params_.j, t);  // joint angle
  gtsam::Key v_key = internal::JointVelKey(params_.j, t);    // joint velocity
  gtsam::Key m_key = MassKey(params_.j, t);
  gtsam::Key t_key = TimeKey(t);
  gtsam::Key to_key = ValveOpenTimeKey(params_.j);
  gtsam::Key tc_key = ValveCloseTimeKey(params_.j);

  graph.add(gtsam::PriorFactor<double>(Ps_key, prior_values.Ps,
                                        prior_pressure_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(q_key, prior_values.q, prior_q_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(v_key, prior_values.v, prior_v_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(m_key, prior_values.m, prior_m_cost_model));

  graph.add(
      gtsam::PriorFactor<double>(t_key, prior_values.t, prior_time_cost_model));
  graph.add(gtsam::PriorFactor<double>(to_key, prior_values.to,
                                       prior_valve_t_cost_model));
  graph.add(gtsam::PriorFactor<double>(tc_key, prior_values.tc,
                                       prior_valve_t_cost_model));
  return graph;
}

gtsam::NonlinearFactorGraph PneumaticActuator::sourceFactorGraph(
    const int t) const {
  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key ms_key = SourceMassKey(t);
  gtsam::Key vs_key = SourceVolumeKey();

  gtsam::NonlinearFactorGraph graph;
  graph.add(GasLawFactor(Ps_key, vs_key, ms_key, gass_law_model,
                         params_.Rs * params_.T));
  return graph;
}

gtsam::NonlinearFactorGraph PneumaticActuator::sourcePriorGraph(
    const int t, const double ms, const double vs) const {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Key ms_key = SourceMassKey(t);
  gtsam::Key vs_key = SourceVolumeKey();
  graph.add(PriorFactor<double>(ms_key, ms, prior_m_cost_model));
  graph.add(PriorFactor<double>(vs_key, vs, volume_model));
  return graph;
}

gtsam::Values PneumaticActuator::actuatorInitValues(
    const int t, const PriorValues& prior_values) const {

  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key P_key = PressureKey(params_.j, t);     // current pressure
  gtsam::Key x_key = ContractionKey(params_.j, t);  // contraction length, (cm)
  gtsam::Key f_key = ForceKey(params_.j, t);        // force
  gtsam::Key q_key = internal::JointAngleKey(params_.j, t);   // joint angle
  gtsam::Key v_key = internal::JointVelKey(params_.j, t);     // joint velocity
  gtsam::Key torque_key = internal::TorqueKey(params_.j, t);  // torque
  gtsam::Key m_key = MassKey(params_.j, t);
  gtsam::Key mdoto_key = MassRateOpenKey(params_.j, t);
  gtsam::Key mdota_key = MassRateActualKey(params_.j, t);
  gtsam::Key vo_key = VolumeKey(params_.j, t);
  gtsam::Key t_key = TimeKey(t);
  gtsam::Key to_key = ValveOpenTimeKey(params_.j);
  gtsam::Key tc_key = ValveCloseTimeKey(params_.j);

  gtsam::Values init_values;
  init_values.insert(Ps_key, prior_values.Ps);
  init_values.insert(q_key, prior_values.q);
  init_values.insert(v_key, prior_values.v);
  init_values.insert(m_key, prior_values.m);
  init_values.insert(t_key, prior_values.t);
  init_values.insert(to_key, prior_values.to);
  init_values.insert(tc_key, prior_values.tc);

  auto factor =
      ActuatorVolumeFactor(vo_key, x_key, volume_model, params_.D, params_.L);
  init_values.insert(P_key, double(10));
  init_values.insert(x_key, double(2));
  init_values.insert(f_key, double(0));
  init_values.insert(torque_key, double(0));
  init_values.insert(mdoto_key, double(1e-2));
  init_values.insert(mdota_key, double(1e-2));
  init_values.insert(vo_key, factor.computeVolume(2));

  return init_values;
}

gtsam::Values PneumaticActuator::sourceInitValues(
    const int t, const double ms, const double vs) const {
  gtsam::Values init_values;
  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key ms_key = SourceMassKey(t);
  gtsam::Key vs_key = SourceVolumeKey();
  init_values.insert(Ps_key, params_.Rs * params_.T * ms / vs / 1e3);
  init_values.insert(ms_key, ms);
  init_values.insert(vs_key, vs);
  return init_values;
}

gtsam::Values PneumaticActuator::computeSourceResult(const int t,
                                                     const double ms,
                                                     const double vs) const {
  auto graph = sourceFactorGraph(t);
  auto init_values = sourceInitValues(t, ms, vs);

  gtsam::LevenbergMarquardtParams params;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gtsam::Values result = optimizer.optimize();
  if (graph.error(result) > 1e-5) {
    throw std::runtime_error("source computation failed");
  }

  return result;
}

gtsam::Values PneumaticActuator::computeResult(
    const int t, const PriorValues& prior_values,
    const Values& previous_values) const {
  // construct a factor graph
  auto graph = actuatorFactorGraph(t);
  gtsam::Key Ps_key = SourcePressureKey(t);
  gtsam::Key P_key = PressureKey(params_.j, t);     // current pressure
  gtsam::Key x_key = ContractionKey(params_.j, t);  // contraction length, (cm)
  gtsam::Key f_key = ForceKey(params_.j, t);        // force
  gtsam::Key q_key = internal::JointAngleKey(params_.j, t);   // joint angle
  gtsam::Key v_key = internal::JointVelKey(params_.j, t);     // joint velocity
  gtsam::Key torque_key = internal::TorqueKey(params_.j, t);  // torque
  gtsam::Key m_key = MassKey(params_.j, t);
  gtsam::Key mdoto_key = MassRateOpenKey(params_.j, t);
  gtsam::Key mdota_key = MassRateActualKey(params_.j, t);
  gtsam::Key vo_key = VolumeKey(params_.j, t);
  gtsam::Key t_key = TimeKey(t);
  gtsam::Key to_key = ValveOpenTimeKey(params_.j);
  gtsam::Key tc_key = ValveCloseTimeKey(params_.j);

  graph.add(gtsam::PriorFactor<double>(Ps_key, prior_values.Ps,
                                       prior_pressure_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(q_key, prior_values.q, prior_q_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(v_key, prior_values.v, prior_v_cost_model));
  graph.add(
      gtsam::PriorFactor<double>(m_key, prior_values.m, prior_m_cost_model));

  graph.add(
      gtsam::PriorFactor<double>(t_key, prior_values.t, prior_time_cost_model));
  graph.add(gtsam::PriorFactor<double>(to_key, prior_values.to,
                                       prior_valve_t_cost_model));
  graph.add(gtsam::PriorFactor<double>(tc_key, prior_values.tc,
                                       prior_valve_t_cost_model));

  // solve the factor graph
  gtsam::Values init_values;
  init_values.insert(Ps_key, prior_values.Ps);
  init_values.insert(q_key, prior_values.q);
  init_values.insert(v_key, prior_values.v);
  init_values.insert(m_key, prior_values.m);
  init_values.insert(t_key, prior_values.t);
  init_values.insert(to_key, prior_values.to);
  init_values.insert(tc_key, prior_values.tc);

  if (previous_values.exists(PressureKey(params_.j, t - 1))) {
    init_values.insert(P_key,
                       previous_values.at(PressureKey(params_.j, t - 1)));
    init_values.insert(x_key,
                       previous_values.at(ContractionKey(params_.j, t - 1)));
    init_values.insert(f_key, previous_values.at(ForceKey(params_.j, t - 1)));
    init_values.insert(
        torque_key, previous_values.at(internal::TorqueKey(params_.j, t - 1)));
    init_values.insert(mdoto_key,
                       previous_values.at(MassRateOpenKey(params_.j, t - 1)));
    init_values.insert(mdota_key,
                       previous_values.at(MassRateActualKey(params_.j, t - 1)));
    init_values.insert(vo_key, previous_values.at(VolumeKey(params_.j, t - 1)));
  } else {
    auto factor =
        ActuatorVolumeFactor(vo_key, x_key, volume_model, params_.D, params_.L);
    init_values.insert(P_key, double(10));
    init_values.insert(x_key, double(2));
    init_values.insert(f_key, double(0));
    init_values.insert(torque_key, double(0));
    init_values.insert(mdoto_key, double(1e-2));
    init_values.insert(mdota_key, double(1e-2));
    init_values.insert(vo_key, factor.computeVolume(2));
  }

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setLinearSolverType("MULTIFRONTAL_QR");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gtsam::Values result = optimizer.optimize();

  // std::cout << "error: " << graph.error(result) << "\n";
  // auto factor = ActuatorVolumeFactor(vo_key, x_key, volume_model);
  // std::cout << "volume error: " << factor.error(result) << "\t" << "x: " <<
  // result.atDouble(x_key) <<"v: " << result.atDouble(vo_key) << "\n";

  if (graph.error(result) > 1e-5) {
    for (auto factor : graph) {
      factor->print();
      std::cout << factor->error(result) << "\n";
    }

    std::cout << "error:\t" << graph.error(result) << "\n";
    // // gtsam::DoglegParams dl_params;
    // // dl_params.setLinearSolverType("MULTIFRONTAL_QR");
    // // gtsam::DoglegOptimizer dl_optimizer(graph, init_values, dl_params);
    // // result = dl_optimizer.optimize();
    params.setVerbosityLM("SUMMARY");
    gtsam::LevenbergMarquardtOptimizer optimizer1(graph, init_values, params);
    optimizer1.optimize();

    // gtsam::NonlinearOptimizerParams param;
    // param.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    // param.maxIterations = 5000;
    // gtsam::NonlinearConjugateGradientOptimizer optimizer_gd(
    //     graph, init_values, param);
    // result = optimizer_gd.optimize();

    // std::cout << "error:\t" << graph.error(result) << "\n";
    // std::cout << "current time:\t" << result.atDouble(t_key) << "\n";
    // std::cout << "current pressure:\t" << result.atDouble(P_key) << "\n";
    // std::cout << "current contraction:\t" << result.atDouble(x_key) << "\n";
    // std::cout << "current force:\t" << result.atDouble(f_key) << "\n";
    // std::cout << "start time:\t" << start_time << "\n";
    // std::cout << "init_pressure:\t" << init_pressure << "\n";
    // std::cout << "angle:\t" << angle << "\n";
    // std::cout << "contract:\t" << params_.positive << "\n";
    // std::cout << "spring constant:\t" << params_.kt << "\n";
    // std::cout << "pulley radius:\t" << params_.r << "\n";
    // std::cout << "rest angle:\t" << params_.q_rest << "\n";
    // std::ofstream json_file;
    // json_file.open("../../visualization/factor_graph.json");
    // JsonSaver::SaveFactorGraph(graph, json_file, result);
    // json_file.close();

    throw std::runtime_error("optimizing for pneumatic actuator graph fails");
  }

  return result;
}

// gtsam::NonlinearFactorGraph PneumaticActuator::actuatorFactorGraph(const int
// t) const {
//   gtsam::Key t_i_key = StartTimeKey(params_.j);     // time of opening the
//   valve gtsam::Key t_key = TimeKey(t);           // current time gtsam::Key
//   P_i_key = InitPressureKey(params_.j);  // initial pressure gtsam::Key P_key
//   = PressureKey(params_.j, t);   // current pressure gtsam::Key x_key =
//   ContractionKey(params_.j, t);  // contraction length gtsam::Key f_key =
//   ForceKey(params_.j, t);        // force gtsam::Key q_key =
//   JointAngleKey(params_.j, t);   // joint angle gtsam::Key v_key =
//   JointVelKey(params_.j, t);     // joint velocity gtsam::Key torque_key =
//   TorqueKey(params_.j, t);  // torque

//   gtsam::NonlinearFactorGraph graph;
//   graph.add(PressureFactor(t_i_key, t_key, P_i_key, P_key,
//                             pressure_cost_model, params_.p_coeffs));

//   // graph.add(PneumaticActuatorFactor(x_key, P_key, f_key, force_cost_model,
//   //                                   pneumatic_coeffs_));

//   graph.add(SmoothActuatorFactor(x_key, P_key, f_key, force_cost_model,
//                                   params_.x0_coeffs, params_.k_coeffs,
//                                   params_.f0_coeffs));

//   // graph.add(JointBalanceFactor(x_key, q_key, f_key,
//   //                              balance_cost_model,
//   //                              kt_, r_, q_rest_, positive_));

//   // joint balance factor
//   double cm_to_m = 0.01;

//   gtsam::Double_ f_expr(f_key);
//   gtsam::Double_ q_expr(q_key);
//   gtsam::Double_ x_expr(x_key);
//   gtsam::Double_ torque_expr(torque_key);

//   double rhs = -params_.kt * params_.q_rest * params_.r;
//   if (params_.positive) {
//     gtsam::ExpressionFactor<double> balance_factor(
//         balance_cost_model, rhs,
//         params_.kt * cm_to_m * x_expr - f_expr - params_.kt * params_.r *
//         q_expr);
//     graph.add(balance_factor);
//   } else {
//     gtsam::ExpressionFactor<double> balance_factor(
//         balance_cost_model, -rhs,
//         params_.kt * cm_to_m * x_expr - f_expr + params_.kt * params_.r *
//         q_expr);
//     graph.add(balance_factor);
//   }

//   graph.add(JointTorqueFactor(q_key, v_key, f_key, torque_key,
//                               torque_cost_model, params_.q_anta_limit,
//                               params_.ka, params_.r, params_.b,
//                               params_.positive));
//   // if (positive_) {
//   //   gtsam::ExpressionFactor<double> torque_factor(
//   //       torque_cost_model, double(0), r_ *f_expr - torque_expr);
//   //   graph.add(torque_factor);
//   // } else {
//   //   gtsam::ExpressionFactor<double> torque_factor(
//   //       torque_cost_model, double(0), -r_ *f_expr - torque_expr);
//   //   graph.add(torque_factor);
//   // }
//   return graph;
// }

// gtsam::Values PneumaticActuator::computeResult(const int t, const double
// angle, const double vel,
//                             const double start_time,
//                             const double current_time,
//                             const double init_pressure) const {
//   // construct a factor graph
//   auto graph = actuatorFactorGraph(t);
//   gtsam::Key t_i_key = StartTimeKey(params_.j);     // time of opening the
//   valve gtsam::Key t_key = TimeKey(t);           // current time gtsam::Key
//   P_i_key = InitPressureKey(params_.j);  // initial pressure gtsam::Key P_key
//   = PressureKey(params_.j, t);   // current pressure gtsam::Key x_key =
//   ContractionKey(params_.j, t);  // contraction length gtsam::Key f_key =
//   ForceKey(params_.j, t);        // force gtsam::Key q_key =
//   JointAngleKey(params_.j, t);   // joint angle gtsam::Key v_key =
//   JointVelKey(params_.j, t);     // joint velocity gtsam::Key torque_key =
//   TorqueKey(params_.j, t);  // torque

//   graph.add(gtsam::PriorFactor<double>(P_i_key, init_pressure,
//                                         prior_pressure_cost_model));
//   graph.add(gtsam::PriorFactor<double>(t_i_key, start_time,
//                                         prior_start_t_cost_model));
//   graph.add(gtsam::PriorFactor<double>(q_key, angle, prior_q_cost_model));
//   graph.add(gtsam::PriorFactor<double>(t_key, current_time,
//                                         prior_time_cost_model));
//   graph.add(gtsam::PriorFactor<double>(v_key, vel,
//                                         prior_v_cost_model));

//   PressureFactor pressure_factor(t_i_key, t_key, P_i_key, P_key,
//                                   pressure_cost_model, params_.p_coeffs);
//   double current_pressure = pressure_factor.evaluateError(
//       start_time, current_time, init_pressure, 0)[0];

//   // solve the factor graph
//   gtsam::Values init_values;
//   init_values.insert(P_i_key, init_pressure);
//   init_values.insert(t_i_key, start_time);
//   init_values.insert(q_key, angle);
//   init_values.insert(v_key, vel);
//   init_values.insert(x_key, double(2));
//   init_values.insert(f_key, double(0));
//   init_values.insert(P_key, current_pressure);
//   init_values.insert(t_key, current_time);
//   init_values.insert(torque_key, double(0));

//   gtsam::LevenbergMarquardtParams params;
//   // params.setVerbosityLM("SUMMARY");
//   params.setLinearSolverType("MULTIFRONTAL_QR");
//   gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
//   gtsam::Values result = optimizer.optimize();

//   if (graph.error(result) > 0.1) {
//     std::cout << "error:\t" << graph.error(result) << "\n";
//     // // gtsam::DoglegParams dl_params;
//     // // dl_params.setLinearSolverType("MULTIFRONTAL_QR");
//     // // gtsam::DoglegOptimizer dl_optimizer(graph, init_values, dl_params);
//     // // result = dl_optimizer.optimize();
//     params.setVerbosityLM("SUMMARY");
//     gtsam::LevenbergMarquardtOptimizer optimizer1(graph, init_values,
//     params); optimizer1.optimize();

//     gtsam::NonlinearOptimizerParams param;
//     param.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
//     param.maxIterations = 5000;
//     gtsam::NonlinearConjugateGradientOptimizer optimizer_gd(
//         graph, init_values, param);
//     result = optimizer_gd.optimize();

//     std::cout << "error:\t" << graph.error(result) << "\n";
//     std::cout << "current time:\t" << result.atDouble(t_key) << "\n";
//     std::cout << "current pressure:\t" << result.atDouble(P_key) << "\n";
//     std::cout << "current contraction:\t" << result.atDouble(x_key) << "\n";
//     std::cout << "current force:\t" << result.atDouble(f_key) << "\n";
//     std::cout << "start time:\t" << start_time << "\n";
//     std::cout << "init_pressure:\t" << init_pressure << "\n";
//     std::cout << "angle:\t" << angle << "\n";
//     std::cout << "contract:\t" << params_.positive << "\n";
//     std::cout << "spring constant:\t" << params_.kt << "\n";
//     std::cout << "pulley radius:\t" << params_.r << "\n";
//     std::cout << "rest angle:\t" << params_.q_rest << "\n";
//     std::ofstream json_file;
//     json_file.open("../../visualization/factor_graph.json");
//     JsonSaver::SaveFactorGraph(graph, json_file, result);
//     json_file.close();

//     throw std::runtime_error("optimizing for pneumatic actuator graph
//     fails");
//   }

//   return result;
// }

}  // namespace gtdynamics