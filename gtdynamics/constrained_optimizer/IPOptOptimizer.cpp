#include <gtdynamics/constrained_optimizer/IPOptOptimizer.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <gtdynamics/utils/DynamicsSymbol.h>

namespace gtdynamics {

/* ************************************************************************* */
/* ****************************  translator  ******************************* */
/* ************************************************************************* */

/* ************************************************************************* */
Pose3 IFOptTranslator::VecToPose(const Vector6 &vec, OptionalJacobian<6, 6> H) {
  if (H) {
    Vector3 euler_angles(vec(0), vec(1), vec(2));
    Vector3 trans(vec(3), vec(4), vec(5));
    Matrix36 H_angles_vec, H_trans_vec;
    H_angles_vec << I_3x3, Z_3x3;
    H_trans_vec << Z_3x3, I_3x3;
    Matrix33 H_rot_angles;
    Rot3 rot = Rot3::RzRyRx(euler_angles, H_rot_angles);
    Matrix63 H_pose_rot, H_pose_trans;
    Pose3 pose = Pose3::Create(rot, trans, H_pose_rot, H_pose_trans);
    *H = H_pose_rot * H_rot_angles * H_angles_vec + H_pose_trans * H_trans_vec;
    return pose;
  } else {
    Vector3 euler_angles(vec(0), vec(1), vec(2));
    Vector3 trans(vec(3), vec(4), vec(5));
    Rot3 rot = Rot3::RzRyRx(euler_angles);
    Pose3 pose = Pose3::Create(rot, trans);
    return pose;
  }
}

/* ************************************************************************* */
Vector6 IFOptTranslator::PoseToVec(const Pose3 &pose) {
  Vector3 trans = pose.translation();
  Rot3 rot = pose.rotation();
  Vector3 euler_angles = rot.rpy();
  Vector6 vec;
  vec << euler_angles, trans;
  return vec;
}

/* ************************************************************************* */
Matrix IFOptTranslator::PoseJacobian(const Vector &vec, const Matrix &H_pose) {
  Matrix66 H_pose_vec;
  VecToPose(vec, H_pose_vec);
  return H_pose * H_pose_vec;
}

/* ************************************************************************* */
Vector IFOptTranslator::PoseGradient(const Vector& vec, const Vector& g_pose) {
  Matrix66 H_pose_vec;
  VecToPose(vec, H_pose_vec);
  return H_pose_vec.transpose() * g_pose;
}

/* ************************************************************************* */
bool IFOptTranslator::IsPoseKey(const Key key) {
  gtdynamics::DynamicsSymbol symb(key);
  if (symb.label() == "p") {
    return true;
  }
  return false;
}

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
    if (IsPoseKey(key)) {
      Pose3 val = values.at<Pose3>(key);
      return PoseToVec(val);
    } else {
      Vector6 val = values.at<Vector6>(key);
      return val;
    }
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
    if (IsPoseKey(key)) {
      Pose3 pose = VecToPose(vec);
      values.insert(key, pose);
    } else {
      Vector6 val = vec;
      values.insert(key, val);
    }
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
                             const gtsam::NonlinearEqualityConstraints &e_constraints,
                             const InequalityConstraints &i_constraints,
                             const Values &initial_values) const {

  // Ensure that all i-constraints are 1-dimensional
  if (i_constraints.size() != i_constraints.dim()) {
    InequalityConstraints new_i_constraints;
    for (const auto &constraint : i_constraints) {
      if (constraint->dim() == 1) {
        new_i_constraints.push_back(constraint);
      } else {
        auto twin_constraint =
            std::static_pointer_cast<TwinDoubleExpressionInequality>(
                constraint);
        new_i_constraints.push_back(twin_constraint->constraint1());
        new_i_constraints.push_back(twin_constraint->constraint2());
      }
    }
    return optimize(cost, e_constraints, new_i_constraints, initial_values);
  }

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
    nlp.AddCostSet(std::make_shared<IFOptCost>(factor, name, translator));
  }
  // nlp.AddVariableSet(std::make_shared<ExVariables>());
  // nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  // nlp.AddCostSet(std::make_shared<ExCost>());
  nlp.PrintCurrent();

  // 2. choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");
  // ipopt.SetOption("hessian_approximation", "limited-memory");

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
                             const gtsam::NonlinearEqualityConstraints &constraints,
                             const Values &initial_values) const {
  InequalityConstraints i_constraints;
  return optimize(cost, constraints, i_constraints, initial_values);
}

/* ************************************************************************* */
/* ******************************  utils  ********************************** */
/* ************************************************************************* */

/* ************************************************************************* */
KeySet ConstructKeySet(const KeyVector &kv) {
  KeySet key_set;
  for (const auto &key : kv) {
    key_set.insert(key);
  }
  return key_set;
}

/* ************************************************************************* */
Matrix RetrieveVarJacobian(const GaussianFactor::shared_ptr &linear_factor,
                           const Key &key) {
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

  return jacobian.middleCols(start_col, var_dim);
}

/* ************************************************************************* */
void FillJacobianHelper(
    const Matrix &jacobian,
    Eigen::SparseMatrix<double, Eigen::RowMajor> &jac_block) {
  /// Fill entries
  for (size_t i = 0; i < jacobian.rows(); i++) {
    for (size_t j = 0; j < jacobian.cols(); j++) {
      jac_block.coeffRef(i, j) = jacobian(i, j);
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
IFOptVariable::VecBound IFOptVariable::GetBounds() const {
  VecBound bounds(GetRows(), ifopt::NoBound);
  return bounds;
}

/* ************************************************************************* */
/* ****************************  e-constraint  ***************************** */
/* ************************************************************************* */

/* ************************************************************************* */
IFOptEConstraint::IFOptEConstraint(
    gtsam::NonlinearEqualityConstraint::shared_ptr constraint,
                                   const std::string &name,
                                   const IFOptTranslator::shared_ptr translator)
    : ConstraintSet(constraint->dim(), name), constraint_(constraint),
      factor_(constraint->penaltyFactor(1.0)),
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
  return constraint_->whitenedError(values);
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
  auto jacobian = RetrieveVarJacobian(linear_factor, key);
  if (values.at(key).dim() == 6 && translator_->IsPoseKey(key)) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    jacobian = translator_->PoseJacobian(x, jacobian);
  }
  FillJacobianHelper(jacobian, jac_block);
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
  auto jacobian = RetrieveVarJacobian(linear_factor, key);
  if (values.at(key).dim() == 6 && translator_->IsPoseKey(key)) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    jacobian = translator_->PoseJacobian(x, jacobian);
  }
  FillJacobianHelper(jacobian, jac_block);
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
  if (values.at(key).dim() == 6 && translator_->IsPoseKey(key)) {
    std::string name = translator_->keyToName(key);
    auto x = GetVariables()->GetComponent(name)->GetValues();
    gradient = translator_->PoseGradient(x, gradient);
  }
  for (size_t j = 0; j < gradient.size(); j++) {
    jac.coeffRef(0, j) = gradient(j);
  }
  // std::cout << "FillJacobianBlock cost finish\n";
}

} // namespace gtdynamics
