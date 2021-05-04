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


};  // PneumaticActuator
}  // namespace gtdynamics
