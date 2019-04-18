/**
 * @file  utils.cpp
 * @brief a few utilities
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <utils.h>

using namespace std;
using namespace gtsam;

namespace manipulator {

Vector6 unit_twist(const Vector3 &w, const Vector3 &p) {
  Vector6 unit_twist;
  unit_twist << w, p.cross(w);
  return unit_twist;
}

double radians(double degree) { return degree * M_PI / 180; }

Vector radians(const Vector &degree) {
  Vector radian(degree.size());
  for (int i = 0; i < degree.size(); ++i) {
    radian(i) = radians(degree(i));
  }
  return radian;
}

Matrix AdjointMapJacobianQ(double q, const Pose3 &jMi,
                           const Vector6 &screw_axis) {
  // taking opposite value of screw_axis_ is because 
  // jTi = Pose3::Expmap(-screw_axis_ * q) * jMi;
  Vector w =
      (Vector(3) << -screw_axis(0), -screw_axis(1), -screw_axis(2)).finished();
  Vector v =
      (Vector(3) << -screw_axis(3), -screw_axis(4), -screw_axis(5)).finished();
  Pose3 kTj = Pose3::Expmap(-screw_axis * q) * jMi;
  auto w_skew = skewSymmetric(w);
  Matrix H_expo = w_skew * cosf(q) + w_skew * w_skew * sinf(q);
  Matrix H_R = H_expo * jMi.rotation().matrix();
  Vector H_T = H_expo * (jMi.translation().vector() - w_skew * v) +
               w * w.transpose() * v;
  Matrix H_TR = skewSymmetric(H_T) * kTj.rotation().matrix() +
                skewSymmetric(kTj.translation().vector()) * H_R;
  Matrix H = Matrix::Zero(6, 6);
  insertSub(H, H_R, 0, 0);
  insertSub(H, H_TR, 3, 0);
  insertSub(H, H_R, 3, 3);
  return H;
}

Matrix getQc(const SharedNoiseModel Qc_model) {
  noiseModel::Gaussian *Gassian_model =
      dynamic_cast<noiseModel::Gaussian *>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

}  // namespace manipulator
