/**
 * @file  testSigneDistanceField.cpp
 * @brief test signed distance filed used for collision check
 * @Origin: GPMP2
 */
#include <SignedDistanceField.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace manipulator;

double sdf_wrapper(const SignedDistanceField& field, const Point3& p) {
  return field.getSignedDistance(p);
}

TEST(SignedDistanceField, test) {
  // data
  vector<Matrix> data;
  data.push_back((Matrix(5, 5) << 1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
                  1.4142, 1, 1, 1, 1.4142, 1.4142, 1, 1, 1, 1.4142, 1.4142, 1,
                  1, 1, 1.4142, 1.7321, 1.4142, 1.4142, 1.4142, 1.7321)
                     .finished());
  data.push_back((Matrix(5, 5) << 1.4142, 1, 1, 1, 1.4142, 1, 0, 0, 0, 1, 1, 0,
                  0, 0, 1, 1, 0, 0, 0, 1, 1.4142, 1, 1, 1, 4142)
                     .finished());
  data.push_back((Matrix(5, 5) << 1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
                  1.4142, 1, 1, 1, 1.4142, 1.4142, 1, 1, 1, 1.4142, 1.4142, 1,
                  1, 1, 1.4142, 1.7321, 1.4142, 1.4142, 1.4142, 1.7321)
                     .finished());
  Point3 origin(-0.2, -0.2, -0.1);
  double cell_size = 0.1;

  // constructor
  SignedDistanceField field(origin, cell_size, data);
  EXPECT_LONGS_EQUAL(5, field.xCount());
  EXPECT_LONGS_EQUAL(5, field.yCount());
  EXPECT_LONGS_EQUAL(3, field.zCount());
  EXPECT_DOUBLES_EQUAL(0.1, field.cellSize(), 1e-9);
  EXPECT(assert_equal(origin, field.origin()));

  // access
  SignedDistanceField::float_index index;
  index = field.convertPoint3toCell(Point3(0, 0, 0));
  EXPECT_DOUBLES_EQUAL(2, index.get<0>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(2, index.get<1>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(1, index.get<2>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(0, field.signedDistance(index), 1e-9)
  index = field.convertPoint3toCell(
      Point3(0.18, -0.18, 0.07));  // tri-linear interpolation
  EXPECT_DOUBLES_EQUAL(0.2, index.get<0>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(3.8, index.get<1>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(1.7, index.get<2>(), 1e-9);
  EXPECT_DOUBLES_EQUAL(1.488288, field.signedDistance(index), 1e-9)
  index = boost::make_tuple(1.0, 2.0, 3.0);
  EXPECT(
      assert_equal(Point3(0.0, -0.1, 0.2), field.convertCelltoPoint3(index)));

  // gradient
  Vector3 actual_gradient, expected_gradient;
  Point3 p;
  p = Point3(-0.13, -0.14, 0.06);
  field.getSignedDistance(p, actual_gradient);
  expected_gradient =
      numericalDerivative11(boost::function<double(const Point3&)>(
                                boost::bind(sdf_wrapper, field, _1)),
                            p, 1e-6);
  EXPECT(assert_equal(expected_gradient, actual_gradient, 1e-6));

  p = Point3(0.18, 0.12, 0.01);
  field.getSignedDistance(p, actual_gradient);
  expected_gradient =
      numericalDerivative11(boost::function<double(const Point3&)>(
                                boost::bind(sdf_wrapper, field, _1)),
                            p, 1e-6);
  EXPECT(assert_equal(expected_gradient, actual_gradient, 1e-6));
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
