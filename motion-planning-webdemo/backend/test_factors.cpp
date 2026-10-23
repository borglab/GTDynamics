#include "TrajectoryFactors.h"
#include <gtsam/base/numericalDerivative.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

int main() {
  using namespace gtsam;
  using namespace motion_planning_demo;
  const auto noise = noiseModel::Isotropic::Sigma(6, 1.);
  PoseTrapezoidalFactor factor(0, 1, 2, 3, .3, noise);
  const Pose3 p0(Rot3::RzRyRx(.4, -.3, .8), Point3(.2, -.1, .7));
  const Pose3 p1(Rot3::RzRyRx(-.2, .5, 1.2), Point3(.4, .3, .6));
  Vector6 v0, v1;
  v0 << .2, -.3, .4, -.1, .5, .1;
  v1 << -.1, .1, .3, .2, -.2, .4;
  Matrix H0, H1, H2, H3;
  factor.evaluateError(p0, p1, v0, v1, &H0, &H1, &H2, &H3);
  std::function<Vector(const Pose3&, const Pose3&, const Vector6&, const Vector6&)> f =
      [&](const Pose3& a, const Pose3& b, const Vector6& c, const Vector6& d) {
        return factor.evaluateError(a, b, c, d);
      };
  const double e0 = (H0 - numericalDerivative41(f, p0, p1, v0, v1)).norm();
  const double e1 = (H1 - numericalDerivative42(f, p0, p1, v0, v1)).norm();
  const double e2 = (H2 - numericalDerivative43(f, p0, p1, v0, v1)).norm();
  const double e3 = (H3 - numericalDerivative44(f, p0, p1, v0, v1)).norm();
  JointTrapezoidalFactor joint(0, 1, 2, 3, .3, noiseModel::Isotropic::Sigma(1, 1.));
  const auto residual = joint.evaluateError(.2, .23, .05, .15, &H0, &H1, &H2, &H3);
  if (std::max({e0, e1, e2, e3}) > 1e-6 || residual.norm() > 1e-12 ||
      std::abs(H0(0, 0) + 1.) > 1e-12 || std::abs(H1(0, 0) - 1.) > 1e-12 ||
      std::abs(H2(0, 0) + .15) > 1e-12 || std::abs(H3(0, 0) + .15) > 1e-12) {
    std::cerr << "collocation Jacobian mismatch: " << e0 << ' ' << e1 << ' ' << e2 << ' ' << e3 << '\n';
    return 1;
  }
  std::cout << "Collocation Jacobians agree with numerical derivatives away from zero residual.\n";
}
