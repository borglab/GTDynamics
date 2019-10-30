/**
 * @file  testObstacleSDFFactor.cpp
 * @brief test obstacle signed distance field factor used for collision check
 * @Origin: GPMP2
 */
#include <ObstacleSDFFactor.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

// convert sdf vector to hinge loss err vector
inline Vector convertSDFtoError(const Vector& sdf, double eps) {
  Vector err_ori = 0.0 - sdf.array() + eps;
  return (err_ori.array() > 0.0)
      .select(err_ori, Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}

// data
SignedDistanceField sdf;

/* ************************************************************************** */
TEST(ObstacleSDFFactorArm, data) {
  double cell_size = 0.1;
  // zero orgin
  Point3 origin(0, 0, 0);
  vector<Matrix> field(3);

  field[0] =
      (Matrix(7, 7) << 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
       0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236, 0.2000, 0.1000,
       -0.1000, -0.1000, -0.1000, 0.1000, 0.2000, 0.2000, 0.1000, -0.1000,
       -0.1000, -0.1000, 0.1000, 0.2000, 0.2000, 0.1000, -0.1000, -0.1000,
       -0.1000, 0.1000, 0.2000, 0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414,
       0.2236, 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828)
          .finished();
  field[1] =
      (Matrix(7, 7) << 0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000,
       0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449, 0.2236, 0.1414,
       0.1000, 0.1000, 0.1000, 0.1414, 0.2236, 0.2236, 0.1414, 0.1000, 0.1000,
       0.1000, 0.1414, 0.2236, 0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414,
       0.2236, 0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449, 0.3000,
       0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000)
          .finished();
  field[2] =
      (Matrix(7, 7) << 0.3464, 0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464,
       0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000, 0.2828, 0.2236,
       0.2000, 0.2000, 0.2000, 0.2236, 0.2828, 0.2828, 0.2236, 0.2000, 0.2000,
       0.2000, 0.2236, 0.2828, 0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236,
       0.2828, 0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000, 0.3464,
       0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464)
          .finished();

  sdf = SignedDistanceField(origin, cell_size, field);
}

TEST(ObstacleSDFFactor, error) {
  vector<Point3> sphere_centers;
  sphere_centers.push_back(Point3(-0.05, 0, 0));
  sphere_centers.push_back(Point3(0.05, 0, 0));
  Pose3 pose(Rot3(), Point3(0.5, 0, 0));
  const double radius = 0.05;
  const double epsilon = 0.2;
  // nosie model
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  ObstacleSDFFactor factor(0, cost_model, epsilon, sdf, radius, sphere_centers);

  // call evaluateError
  Matrix actual_H, expected_H;
  auto actual_errors = factor.evaluateError(pose, actual_H);
  auto expected_sdf = (Vector(2) << sdf.getSignedDistance(Point3(0.45, 0, 0)),
                       sdf.getSignedDistance(Point3(0.55, 0, 0)))
                          .finished();
  auto expected_errors = convertSDFtoError(expected_sdf, epsilon + radius);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  //   // when the signed distance is zero or out of range, numericalDerivative
  //   gets wrong result expected_H = numericalDerivative11(
  //       boost::function<Vector(const Pose3&)>(boost::bind(
  //           &ObstacleSDFFactor::evaluateError, factor, _1, boost::none)),
  //       pose, 1e-6);

  //   EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}