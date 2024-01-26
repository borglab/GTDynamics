/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SQPOptimizer.h
 * @brief Trust-region SQP Optimizer.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

struct SQPParams {
  LevenbergMarquardtParams lm_params = LevenbergMarquardtParams();
  double merit_e_l1_mu = 1e0;
  double merit_e_l2_mu = 1e0;
  double merit_i_l1_mu = 1e0;
  double merit_i_l2_mu = 1e0;

  using shared_ptr = std::shared_ptr<SQPParams>;
};

struct Eval {
  double cost;
  double e_violation;
  double i_violation;
  double merit;

  void print() const {
    std::cout << "\tcost:\t" << cost << "\n";
    std::cout << "\te_vio:\t" << e_violation << "\n";
    std::cout << "\ti_vio:\t" << i_violation << "\n";
    std::cout << "\tmerit:\t" << merit << "\n";
  }
};

class SQPIterDetails;

class SQPState {
public:
  double lambda;
  double lambda_factor;
  gtsam::Values values;
  Eval eval;
  size_t iterations;
  size_t totalNumberInnerIterations;
  GaussianFactorGraph linear_cost;
  GaussianFactorGraph linear_e_merit;
  GaussianFactorGraph linear_e_constraints;
  GaussianFactorGraph linear_i_merit;
  GaussianFactorGraph linear_i_constraints;
  VectorValues min_vector; // minimum vector that satisfy linear constraints

  SQPState() {}

  SQPState(const Values &_values, const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &e_constraints,
           const gtdynamics::InequalityConstraints &i_constraints,
           const SQPParams &params, const double _lambda,
           const double _lambda_factor, const size_t _iterations = 0);

  static SQPState
  FromLastIteration(const SQPIterDetails &iter_details,
                    const gtsam::NonlinearFactorGraph &graph,
                    const gtdynamics::EqualityConstraints &e_constraints,
                    const gtdynamics::InequalityConstraints &i_constraints,
                    const SQPParams &params);

protected:
  void computeMinVector();
};

class SQPTrial {
public:
  double lambda;
  VectorValues delta;
  Values new_values;
  Eval eval;
  double nonlinear_merit_change = 0;
  double linear_merit_change = 0;
  double model_fidelity = 0;
  bool solve_successful = false;
  bool step_is_successful = false;
  bool use_merit_system = false;
  double bound_rate = 0;
  double trial_time = 0;

  SQPTrial(const SQPState &state, const double _lambda,
           const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &e_constraints,
           const gtdynamics::InequalityConstraints &i_constraints,
           const SQPParams &params);

  /// Update lambda for the next trial/state.
  void setNextLambda(double &new_lambda, double &new_lambda_factor,
                     const LevenbergMarquardtParams &params) const;

  /// Set lambda as increased values for next trial/state.
  void setIncreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;

  /// Set lambda as decreased values for next trial/state.
  void setDecreasedNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const;

protected:
  /// Construct constrained QP problem.
  GaussianFactorGraph constructConstrainedSystem(const SQPState &state,
                                                 const SQPParams &params) const;

  /// Construct quadratic approximation of merit function.
  GaussianFactorGraph constructMeritSystem(const SQPState &state,
                                           const SQPParams &params) const;

  /// Re-solve linear system, using QP approximation of merit function as
  /// objective function.
  void resolveLinearUsingMeritSystem(const SQPState &state,
                                     const SQPParams &params);

  mutable std::vector<LMCachedModel> noiseModelCache;
  LMCachedModel *getCachedModel(size_t dim) const;

  GaussianFactorGraph
  buildDampedSystemDiagonal(GaussianFactorGraph damped, const SQPState &state,
                            const LevenbergMarquardtParams &lm_params) const;

  GaussianFactorGraph buildDampedSystemUniform(GaussianFactorGraph damped,
                                               const SQPState &state) const;

  GaussianFactorGraph
  buildDampedSystem(const GaussianFactorGraph &damped, const SQPState &state,
                    const LevenbergMarquardtParams &lm_params) const;
};

class SQPIterDetails {
public:
  SQPState state;
  std::vector<SQPTrial> trials;

  SQPIterDetails(const SQPState &_state) : state(_state), trials() {}
};

class SQPItersDetails : public std::vector<SQPIterDetails> {
public:
  using base = std::vector<SQPIterDetails>;
  using base::base;

  // void exportFile(const std::string &state_file_path,
  //                 const std::string &trial_file_path) const;
};

class SQPOptimizer {
protected:
  SQPParams p_;
  std::shared_ptr<SQPItersDetails> details_;

public:
  typedef std::shared_ptr<SQPOptimizer> shared_ptr;

  const SQPItersDetails &details() const { return *details_; }

  SQPOptimizer(const SQPParams &parameters)
      : p_(parameters), details_(std::make_shared<SQPItersDetails>()) {}

  static Eval
  MeritFunction(const gtsam::NonlinearFactorGraph &graph,
                const gtdynamics::EqualityConstraints &e_constraints,
                const gtdynamics::InequalityConstraints &i_constraints,
                const gtsam::Values &values, const SQPParams &params);

  static Eval MeritFunctionApprox(const SQPState &state,
                                  const VectorValues &delta,
                                  const SQPParams &params);

  gtsam::Values optimize(const gtsam::NonlinearFactorGraph &graph,
                         const gtdynamics::EqualityConstraints &e_constraints,
                         const gtdynamics::InequalityConstraints &i_constraints,
                         const gtsam::Values &init_values);

  SQPIterDetails iterate(const gtsam::NonlinearFactorGraph &graph,
                         const gtdynamics::EqualityConstraints &e_constraints,
                         const gtdynamics::InequalityConstraints &i_constraints,
                         const SQPState &state);

  /** Check if lambda is within limits. */
  bool checkLambdaWithinLimits(const double &lambda) const;

  bool checkConvergence(const SQPState &prev_state,
                        const SQPState &state) const;

  bool checkSuccessfulTrial(const SQPIterDetails &iter_details) const;
};

} // namespace gtsam
