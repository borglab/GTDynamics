/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testManifoldOpt_so2.cpp
 * @brief Test manifold optimizer with SO(2) manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/manifold/ManifoldOptimizerType1.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "ManifoldOptScenarios.h"

namespace gtsam {

/**
 * Manually defined Rot2 manifold that performs retraction with projection.
 */
class Rot2Projection {
  /** we store cos(theta) and sin(theta) */
  double c_, s_;

  /** normalize to make sure cos and sin form unit vector */
  Rot2Projection& normalize() {
    double scale = c_ * c_ + s_ * s_;
    if (std::abs(scale - 1.0) > 1e-10) {
      scale = 1 / sqrt(scale);
      c_ *= scale;
      s_ *= scale;
    }
    return *this;
  }

  /** private constructor from cos/sin */
  inline Rot2Projection(double c, double s) : c_(c), s_(s) {}

  Point2 tangent() const { return Point2(-s_, c_); }

  /** return 2*2 rotation matrix */
  Matrix2 matrix() const {
    Matrix2 rvalue_;
    rvalue_ << c_, -s_, s_, c_;
    return rvalue_;
  }

  /** return 2*2 transpose (inverse) rotation matrix   */
  Matrix2 transpose() const {
    Matrix2 rvalue_;
    rvalue_ << c_, s_, -s_, c_;
    return rvalue_;
  }

 public:
  enum { dimension = 1 };

  /// @name Constructors and named constructors
  /// @{

  /** default constructor, zero rotation */
  Rot2Projection() : c_(1.0), s_(0.0) {}

  /** copy constructor */
  Rot2Projection(const Rot2Projection& r) : Rot2Projection(r.c_, r.s_) {}

  /// Constructor from angle in radians == exponential map at identity
  Rot2Projection(double theta) : c_(cos(theta)), s_(sin(theta)) {}

  /// Named constructor from cos(theta),sin(theta) pair, will *not* normalize!
  static Rot2Projection fromCosSin(double c, double s) {
    Rot2Projection R(c, s);
    return R.normalize();
  }

  /** print */
  void print(const std::string& s = "theta") const {
    std::cout << s << ": " << theta() << std::endl;
  }

  /** equals with an tolerance */
  bool equals(const Rot2Projection& R, double tol = 1e-9) const {
    return std::abs(c_ - R.c_) <= tol && std::abs(s_ - R.s_) <= tol;
  }

  /// retraction with optional derivatives.
  Rot2Projection retract(const gtsam::Vector1& v,  // TODO: use xi
                         gtsam::OptionalJacobian<1, 1> H1 = nullptr,
                         gtsam::OptionalJacobian<1, 1> H2 = nullptr) const {
    if (H1) H1->setZero();
    if (H2) H2->setZero();

    Point2 tangent_v = v(0) * tangent();
    Rot2Projection R(c_ + tangent_v.x(), s_ + tangent_v.y());
    return R.normalize();
  }

  /// localCoordinates with optional derivatives.
  gtsam::Vector1 localCoordinates(
      const Rot2Projection& g, gtsam::OptionalJacobian<1, 1> H1 = nullptr,
      gtsam::OptionalJacobian<1, 1> H2 = nullptr) const {
    if (H1) H1->setZero();
    if (H2) H2->setZero();
    return Vector1(0);
  }

  /**
   * rotate point from rotated coordinate frame to world \f$ p^w = R_c^w p^c \f$
   */
  Point2 rotate(const Point2& p, OptionalJacobian<2, 1> H1 = nullptr,
                OptionalJacobian<2, 2> H2 = nullptr) const {
    const Point2 q = Point2(c_ * p.x() + -s_ * p.y(), s_ * p.x() + c_ * p.y());
    if (H1) *H1 << -q.y(), q.x();
    if (H2) *H2 = matrix();
    return q;
  }

  /**
   * rotate point from world to rotated frame \f$ p^c = (R_c^w)^T p^w \f$
   */
  Point2 unrotate(const Point2& p, OptionalJacobian<2, 1> H1 = nullptr,
                  OptionalJacobian<2, 2> H2 = nullptr) const {
    const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
    if (H1) *H1 << q.y(), -q.x();
    if (H2) *H2 = transpose();
    return q;
  }

  /** return cos */
  inline double c() const { return c_; }

  /** return sin */
  inline double s() const { return s_; }

  /** return angle (RADIANS) */
  double theta() const { return ::atan2(s_, c_); }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(c_);
    ar& BOOST_SERIALIZATION_NVP(s_);
  }
#endif
};

// Specialize Rot2Projection traits to use a Retract/Local
template <>
struct traits<Rot2Projection> : gtsam::internal::Manifold<Rot2Projection> {};

}  // namespace gtsam

using namespace gtsam;

/** Check creation of manifold optimization problem. */
TEST(ManifoldOptProblem, SO2) {
  using namespace so2_scenario;
  auto costs = get_graph(-2, 0);
  auto constraints = get_constraints();

  Values init_values;
  init_values.insert(x1_key, 0.8);
  init_values.insert(x2_key, 0.6);

  LevenbergMarquardtParams nopt_params;
  ManifoldOptimizerParameters mopt_params;
  ManifoldOptimizerType1 optimizer(mopt_params, nopt_params);
  auto mopt_problem =
      optimizer.initializeMoptProblem(*costs, *constraints, init_values);

  EXPECT_LONGS_EQUAL(1, mopt_problem.components_.size());
  EXPECT_LONGS_EQUAL(0, mopt_problem.unconstrained_keys_.size());
  EXPECT_LONGS_EQUAL(1, mopt_problem.manifold_keys_.size());
  EXPECT_LONGS_EQUAL(1, mopt_problem.values_.size());
  EXPECT_LONGS_EQUAL(2, mopt_problem.graph_.size());
  EXPECT_LONGS_EQUAL(2, mopt_problem.problemDimension().first);
  EXPECT_LONGS_EQUAL(1, mopt_problem.problemDimension().second);
}

/** Optimization using Rot2 manifold. */
TEST(ManifoldOptimization, SO2) {
  Key rot_key = 1;

  Expression<Point2> pt_unit(Point2(1, 0));
  Expression<Rot2> rot(rot_key);
  Expression<Point2> rotated_pt(&Rot2::rotate, rot, pt_unit);
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);
  NonlinearFactorGraph graph;
  Point2 goal_pt(-2, 0);
  graph.addExpressionFactor<Point2>(noise, goal_pt, rotated_pt);

  Values init_values;
  init_values.insert(rot_key, Rot2::fromCosSin(0.8, 0.6));

  LevenbergMarquardtParams params;
  params.minModelFidelity = 0.5;
  // params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  auto result = optimizer.optimize();
  // result.print();

  Rot2 expected_rot = Rot2::fromCosSin(-1, 0.0);
  EXPECT(assert_equal(expected_rot, result.at<Rot2>(rot_key), 1e-4));
}

/** Optimization using Type1 manifold optimizer. */
TEST(ManifoldOptimizerType1, SO2) {
  using namespace so2_scenario;
  auto costs = get_graph(-2, 0);
  auto constraints = get_constraints();

  Values init_values;
  init_values.insert(x1_key, 0.8);
  init_values.insert(x2_key, 0.6);

  LevenbergMarquardtParams nopt_params;
  nopt_params.minModelFidelity = 0.5;
  // nopt_params.setVerbosityLM("SUMMARY");
  ManifoldOptimizerParameters mopt_params;
  ManifoldOptimizerType1 optimizer(mopt_params, nopt_params);
  auto result = optimizer.optimize(*costs, *constraints, init_values);
  // result.print();

  EXPECT(assert_equal(-1.0, result.atDouble(x1_key), 1e-5));
  EXPECT(assert_equal(0.0, result.atDouble(x2_key), 1e-5));
}

/** Optimization using Type1 manifold optimizer, infeasible. */
TEST(ManifoldOptimizerType1_infeasible, SO2) {
  using namespace so2_scenario;
  auto costs = get_graph(-2, 0);
  auto constraints = get_constraints();

  Values init_values;
  init_values.insert(x1_key, 0.5);
  init_values.insert(x2_key, 0.5);

  LevenbergMarquardtParams nopt_params;
  nopt_params.minModelFidelity = 0.5;
  // nopt_params.setVerbosityLM("SUMMARY");
  ManifoldOptimizerParameters mopt_params;
  mopt_params.cc_params->retract_params->lm_params.setMaxIterations(4);
  ManifoldOptimizerType1 optimizer(mopt_params, nopt_params);
  auto result = optimizer.optimize(*costs, *constraints, init_values);
  // result.print();

  // TODO: check why this does not converge as good as Type2.
  EXPECT(assert_equal(-1.0, result.atDouble(x1_key), 1e-3));
  EXPECT(assert_equal(0.0, result.atDouble(x2_key), 1e-3));
}

/** For infeasible methods, all 3 methods shall generate the same Gauss-Newton
 * iterations. */
TEST(ManifoldOptimizer, GaussNewtonEquality) {
  // Problem setting.
  using namespace so2_scenario;
  double init_x = 0.8;
  double init_y = 0.6;
  double goal_x = -2;
  double goal_y = 0;

  // rot2 manifold problem
  Key rot_key = 1;
  Expression<Point2> pt_unit(Point2(1, 0));
  Expression<Rot2Projection> rot(rot_key);
  Expression<Point2> rotated_pt(&Rot2Projection::rotate, rot, pt_unit);
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);
  NonlinearFactorGraph graph_rot2;
  Point2 goal_pt(goal_x, goal_y);
  graph_rot2.addExpressionFactor<Point2>(noise, goal_pt, rotated_pt);

  Values init_values_rot2;
  init_values_rot2.insert(rot_key, Rot2Projection::fromCosSin(0.8, 0.6));

  // constraint manifold problem (type1 and type2)
  auto costs_cm = get_graph(goal_x, goal_y);
  auto constraints_cm = get_constraints();

  Values init_values_cm;
  init_values_cm.insert(x1_key, init_x);
  init_values_cm.insert(x2_key, init_y);

  // optimization stages
  GaussNewtonParams nopt_params;
  GaussNewtonOptimizer optimizer_m(graph_rot2, init_values_rot2, nopt_params);
  ManifoldOptimizerParameters mopt_params;
  ManifoldOptimizerType1 optimizer_type1(mopt_params, nopt_params);
  auto mopt_problem = optimizer_type1.initializeMoptProblem(
      *costs_cm, *constraints_cm, init_values_cm);
  auto mopt_noptimizer =
      optimizer_type1.constructNonlinearOptimizer(mopt_problem);

  // check values at each iteration
  for (size_t i = 0; i < 2; i++) {
    // run one iteration
    optimizer_m.iterate();
    mopt_noptimizer->iterate();
    // retrieve values
    Values values_m = optimizer_m.values();
    Values values_type1 =
        optimizer_type1.baseValues(mopt_problem, mopt_noptimizer->values());
    // check values are the same
    auto expected_rot = values_m.at<Rot2Projection>(rot_key);
    double expected_x = expected_rot.c();
    double expected_y = expected_rot.s();
    double type1_x = values_type1.atDouble(x1_key);
    double type1_y = values_type1.atDouble(x2_key);
    double tol = 1e-5;
    EXPECT_DOUBLES_EQUAL(expected_x, type1_x, tol);
    EXPECT_DOUBLES_EQUAL(expected_y, type1_y, tol);
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
