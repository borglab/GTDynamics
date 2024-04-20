/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IPOptOptimizer.h
 * @brief Primal-dual interior point optimizer calling IPOPT.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>

namespace gtsam {

class IFOptTranslator {
public:
  using shared_ptr = std::shared_ptr<IFOptTranslator>;

  IFOptTranslator() {}
  /// Translate a gtsam value into a vector compatible for ifopt
  static Vector valueToVec(const gtsam::Values &values, const Key &key);

  static Values vecToValue(const Vector &vec, const Key &key);

  static std::string keyToName(const Key &key);

  static Key nameToKey(const std::string &name);

  static Pose3 VecToPose(const Vector6& vec, OptionalJacobian<6,6> H=nullptr);

  static Vector6 PoseToVec(const Pose3& pose);

  /// Transform the Jacobian w.r.t. pose into Jacobian w.r.t. vector.
  static Matrix PoseJacobian(const Vector& vec, const Matrix& H_pose);

  static Vector PoseGradient(const Vector& vec, const Vector& g_pose);

  static bool IsPoseKey(const Key key);
};

class IPIterDetails {
  IPIterDetails() {}
};
typedef std::vector<IPIterDetails> IPItersDetails;

/// IPOPT optimizer.
class IPOptimizer : public ConstrainedOptimizer {
protected:
public:
  /// Default constructor.
  IPOptimizer() {}

  /// Run optimization with equality constraints only.
  Values optimize(const NonlinearFactorGraph &cost,
                  const EqualityConstraints &constraints,
                  const Values &initial_values) const override;

  /// Run optimization with equality and inequality constraints.
  Values optimize(const NonlinearFactorGraph &cost,
                  const EqualityConstraints &e_constraints,
                  const InequalityConstraints &i_constraints,
                  const Values &initial_values) const override;
};

/// Variable Set compatible with ifopt
class IFOptVariable : public ifopt::VariableSet {
protected:
  Vector vec_;

public:
  IFOptVariable(const Values &values, const Key key,
                const IFOptTranslator::shared_ptr translator);

  void SetVariables(const VectorXd &x) override { vec_ = x; }

  VectorXd GetValues() const override { return vec_; }

  VecBound GetBounds() const override;
};

/// Equality constraint compatible with ifopt
class IFOptEConstraint : public ifopt::ConstraintSet {
protected:
  EqualityConstraint::shared_ptr constraint_;
  NonlinearFactor::shared_ptr factor_;
  KeySet keys_;
  IFOptTranslator::shared_ptr translator_;

public:
  IFOptEConstraint(EqualityConstraint::shared_ptr constraint,
                   const std::string &name,
                   const IFOptTranslator::shared_ptr translator);

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set,
                         Jacobian &jac_block) const override;

protected:
  Values GetValuesGTSAM() const;
};

/// Inequality constraint compatible with ifopt
class IFOptIConstraint : public ifopt::ConstraintSet {
protected:
  InequalityConstraint::shared_ptr constraint_;
  NonlinearFactor::shared_ptr factor_;
  KeySet keys_;
  IFOptTranslator::shared_ptr translator_;

public:
  IFOptIConstraint(InequalityConstraint::shared_ptr constraint,
                   const std::string &name,
                   const IFOptTranslator::shared_ptr translator);

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set,
                         Jacobian &jac_block) const override;

protected:
  Values GetValuesGTSAM() const;
};

/// Cost terms compatible with ifopt
class IFOptCost : public ifopt::CostTerm {
protected:
  NonlinearFactor::shared_ptr factor_;
  KeySet keys_;
  IFOptTranslator::shared_ptr translator_;

public:
  IFOptCost(NonlinearFactor::shared_ptr factor, const std::string &name,
            const IFOptTranslator::shared_ptr translator);
  double GetCost() const override;
  void FillJacobianBlock(std::string var_set, Jacobian &jac) const override;

protected:
  Values GetValuesGTSAM() const;
};

} // namespace gtsam
