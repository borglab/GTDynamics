/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizer.h
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#pragma once
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IELMOptimizerState.h>
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/**
 * This class performs Levenberg-Marquardt nonlinear optimization
 */
class IELMOptimizer : public IEOptimizer {

protected:
  const LevenbergMarquardtParams params_; ///< LM parameters
  std::shared_ptr<std::vector<IELMIterDetails>> details_;

public:
  typedef std::shared_ptr<IELMOptimizer> shared_ptr;

  const std::vector<IELMIterDetails> &details() const { return *details_; }

  void exportDetails(const std::string &folder_path) const;

  /** Constructor */
  IELMOptimizer(
      const LevenbergMarquardtParams &params = LevenbergMarquardtParams())
      : IEOptimizer(), params_(params),
        details_(std::make_shared<std::vector<IELMIterDetails>>()) {}

  /** Virtual destructor */
  ~IELMOptimizer() {}

  /** Read-only access the parameters */
  const LevenbergMarquardtParams &params() const { return params_; }

  Values optimizeManifolds(const NonlinearFactorGraph &graph,
                           const IEManifoldValues &manifolds,
                           gtdynamics::ConstrainedOptResult
                               *intermediate_result = nullptr) const override;

  bool checkConvergence(double relativeErrorTreshold,
                        double absoluteErrorTreshold, double errorThreshold,
                        double currentError, double newError) const;

  IELMIterDetails iterate(const NonlinearFactorGraph &graph,
                          const IELMState &state) const;

  /** Inner loop, changes state, returns true if successful or giving up */
  void tryLambda(const NonlinearFactorGraph &graph,
                              const IELMState &currentState,
                              IELMTrial &trial) const;

  bool checkModeChange(const NonlinearFactorGraph &graph,
                       IELMIterDetails &current_iter_details) const;
};

} // namespace gtsam
