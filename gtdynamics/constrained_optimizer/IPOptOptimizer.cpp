#include <gtdynamics/constrained_optimizer/IPOptOptimizer.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <gtdynamics/utils/DynamicsSymbol.h>

namespace gtsam {

/* ************************************************************************* */
/* ****************************  translator  ******************************* */
/* ************************************************************************* */

/* ************************************************************************* */
Vector IFOptTranslator::valueToVec(const gtsam::Values &values,
                                   const Key &key) {
  size_t dim = values.at(key).dim();
  if (dim == 1) {
    double val = values.atDouble(key);
    return Vector1(val);
  } else if (dim == 3) {
    Vector3 val = values.at<Vector3>(key);
    return val;
  } else if (dim == 6) {
    Vector6 val = values.at<Vector6>(key);
    return val;
  } else {
    throw std::runtime_error("undefined dimension");
  }
}

/* ************************************************************************* */
Values IFOptTranslator::vecToValue(const Vector &vec, const Key &key) {
  Values values;
  size_t dim = vec.size();
  if (dim == 1) {
    double val = vec(0);
    values.insert(key, val);
  } else if (dim == 3) {
    Vector3 val = vec;
    values.insert(key, val);
  } else if (dim == 6) {
    Vector6 val = vec;
    values.insert(key, val);
  } else {
    throw std::runtime_error("undefined dimension");
  }

  return values;
}

/* ************************************************************************* */
std::string IFOptTranslator::keyToName(const Key &key) {
  // return gtdynamics::GTDKeyFormatter(key);
  return std::to_string(key);
}

/* ************************************************************************* */
Key IFOptTranslator::nameToKey(const std::string &name) {
  return std::stoull(name);
}

/* ************************************************************************* */
/* *****************************  optimizer  ******************************* */
/* ************************************************************************* */

/* ************************************************************************* */
Values IPOptimizer::optimize(const NonlinearFactorGraph &cost,
                             const EqualityConstraints &e_constraints,
                             const InequalityConstraints &i_constraints,
                             const Values &initial_values) const {

  // 0. translate cost, constraints, values to ipopt format.
  auto translator = std::make_shared<IFOptTranslator>();

  // 1. define the problem
  ifopt::Problem nlp;
  for (const Key key : initial_values.keys()) {
    nlp.AddVariableSet(
        std::make_shared<IFOptVariable>(initial_values, key, translator));
  }
  for (size_t i = 0; i < e_constraints.size(); i++) {
    const auto &constraint = e_constraints.at(i);
    std::string name = "eConstraint" + std::to_string(i);
    nlp.AddConstraintSet(
        std::make_shared<IFOptEConstraint>(constraint, name, translator));
  }
  for (size_t i = 0; i < i_constraints.size(); i++) {
    const auto &constraint = i_constraints.at(i);
    std::string name = "iConstraint" + std::to_string(i);
    nlp.AddConstraintSet(
        std::make_shared<IFOptIConstraint>(constraint, name, translator));
  }
  for (size_t i = 0; i < cost.size(); i++) {
    const auto &factor = cost.at(i);
    std::string name = "cost" + std::to_string(i);
    nlp.AddConstraintSet(std::make_shared<IFOptCost>(factor, name, translator));
  }
  // nlp.AddVariableSet(std::make_shared<ExVariables>());
  // nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  // nlp.AddCostSet(std::make_shared<ExCost>());
  nlp.PrintCurrent();

  // 2. choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");

  // 3 . solve
  ipopt.Solve(nlp);

  // 4. translate to gtsam Values
  Values values;
  auto ifopt_result = nlp.GetOptVariables();
  for (const Key key : initial_values.keys()) {
    std::string name = translator->keyToName(key);
    Vector x = ifopt_result->GetComponent(name)->GetValues();
    values.insert(translator->vecToValue(x, key));
  }
  return values;
}

/* ************************************************************************* */
Values IPOptimizer::optimize(const NonlinearFactorGraph &cost,
                             const EqualityConstraints &constraints,
                             const Values &initial_values) const {
  InequalityConstraints i_constraints;
  return optimize(cost, constraints, i_constraints, initial_values);
}

/* ************************************************************************* */
/* ******************************  utils  ********************************** */
/* ************************************************************************* */
KeySet ConstructKeySet(const KeyVector &kv) {
  KeySet key_set;
  for (const auto &key : kv) {
    key_set.insert(key);
  }
  return key_set;
}

void FillJacobianHelper(
    const GaussianFactor::shared_ptr &linear_factor, const Key &key,
    Eigen::SparseMatrix<double, Eigen::RowMajor> &jac_block) {
  /// Identify start and end slots
  size_t start_col = 0;
  size_t var_dim = 0;
  for (auto it = linear_factor->begin(); it != linear_factor->end(); it++) {
    if (*it == key) {
      var_dim = linear_factor->getDim(it);
      break;
    }
    start_col += linear_factor->getDim(it);
  }

  /// Compute jacobian
  auto jacobian = linear_factor->jacobian().first;

  /// Fill entries
  for (size_t i = 0; i < jacobian.rows(); i++) {
    for (size_t j = 0; j < var_dim; j++) {
      jac_block.coeffRef(i, j) = jacobian(i, start_col + j);
    }
  }
}

/* ************************************************************************* */
/* *****************************  variables  ******************************* */
/* ************************************************************************* */

/* ************************************************************************* */
IFOptVariable::IFOptVariable(const Values &values, const Key key,
                             const IFOptTranslator::shared_ptr translator)
    : VariableSet(values.at(key).dim(), translator->keyToName(key)),
      vec_(translator->valueToVec(values, key)) {}

/* ************************************************************************* */
/* ****************************  e-constraint  ***************************** */
/* ************************************************************************* */

/* ************************************************************************* */
IFOptEConstraint::IFOptEConstraint(EqualityConstraint::shared_ptr constraint,
                                   const std::string &name,
                                   const IFOptTranslator::shared_ptr translator)
    : ConstraintSet(constraint->dim(), name), constraint_(constraint),
      factor_(constraint->createFactor(1.0)),
      keys_(ConstructKeySet(factor_->keys())), translator_(translator) {}

/* ************************************************************************* */
Values IFOptEConstraint::GetValuesGTSAM() const {
  Values values;
  for (const Key &key : constraint_->keys()) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    values.insert(translator_->vecToValue(x, key));
  }
  return values;
}

/* ************************************************************************* */
IFOptEConstraint::VectorXd IFOptEConstraint::GetValues() const {
  // std::cout << "get values e-constraint\n";
  Values values = GetValuesGTSAM();
  return constraint_->evaluate(values);
}

/* ************************************************************************* */
IFOptEConstraint::VecBound IFOptEConstraint::GetBounds() const {
  VecBound b(GetRows(), ifopt::Bounds(0.0, 0.0));
  return b;
}

/* ************************************************************************* */
void IFOptEConstraint::FillJacobianBlock(std::string var_set,
                                         Jacobian &jac_block) const {
  Key key = translator_->nameToKey(var_set);
  if (!keys_.exists(key)) {
    return;
  }
  Values values = GetValuesGTSAM();
  auto linear_factor = factor_->linearize(values);
  FillJacobianHelper(linear_factor, key, jac_block);
}

/* ************************************************************************* */
/* ****************************  i-constraint  ***************************** */
/* ************************************************************************* */

/* ************************************************************************* */
IFOptIConstraint::IFOptIConstraint(InequalityConstraint::shared_ptr constraint,
                                   const std::string &name,
                                   const IFOptTranslator::shared_ptr translator)
    : ConstraintSet(constraint->dim(), name), constraint_(constraint),
      factor_(constraint->createL2Factor()),
      keys_(ConstructKeySet(factor_->keys())), translator_(translator) {}

/* ************************************************************************* */
Values IFOptIConstraint::GetValuesGTSAM() const {
  Values values;
  for (const Key &key : constraint_->keys()) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    values.insert(translator_->vecToValue(x, key));
  }
  return values;
}

/* ************************************************************************* */
IFOptIConstraint::VectorXd IFOptIConstraint::GetValues() const {
  // std::cout << "get values i-constraint\n";
  Values values = GetValuesGTSAM();
  double eval = constraint_->toleranceScaledEvaluation(values);
  return Vector1(eval);
}

/* ************************************************************************* */
IFOptIConstraint::VecBound IFOptIConstraint::GetBounds() const {
  // std::cout << "get bounds i-constraint\n";
  VecBound b(GetRows(), ifopt::Bounds(0.0, ifopt::inf));
  return b;
}

/* ************************************************************************* */
void IFOptIConstraint::FillJacobianBlock(std::string var_set,
                                         Jacobian &jac_block) const {
  // std::cout << "FillJacobianBlock i-constraint\n";
  Key key = translator_->nameToKey(var_set);
  if (!keys_.exists(key)) {
    return;
  }
  Values values = GetValuesGTSAM();
  auto linear_factor = factor_->linearize(values);
  FillJacobianHelper(linear_factor, key, jac_block);
}

/* ************************************************************************* */
/* ********************************  cost  ********************************* */
/* ************************************************************************* */

/* ************************************************************************* */
IFOptCost::IFOptCost(NonlinearFactor::shared_ptr factor,
                     const std::string &name,
                     const IFOptTranslator::shared_ptr translator)
    : ifopt::CostTerm(name), factor_(factor),
      keys_(ConstructKeySet(factor->keys())), translator_(translator) {}

/* ************************************************************************* */
Values IFOptCost::GetValuesGTSAM() const {
  Values values;
  for (const Key &key : factor_->keys()) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    values.insert(translator_->vecToValue(x, key));
  }
  return values;
}

/* ************************************************************************* */
double IFOptCost::GetCost() const {
  // std::cout << "get values cost\n";
  Values values = GetValuesGTSAM();
  return factor_->error(values);
}

/* ************************************************************************* */
void IFOptCost::FillJacobianBlock(std::string var_set, Jacobian &jac) const {
  // std::cout << "FillJacobianBlock cost\n";
  Key key = translator_->nameToKey(var_set);
  if (!keys_.exists(key)) {
    return;
  }
  Values values = GetValuesGTSAM();
  auto linear_factor = factor_->linearize(values);
  VectorValues gradient_all = linear_factor->gradientAtZero();
  Vector gradient = gradient_all.at(key);
  for (size_t j = 0; j < gradient.size(); j++) {
    jac.coeffRef(0, j) = gradient(j);
  }
  // std::cout << "FillJacobianBlock cost finish\n";
}

} // namespace gtsam
