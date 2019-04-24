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

Matrix6 AdjointMapJacobianQ(double q, const Pose3 &jMi,
                            const Vector6 &screw_axis) {
  // taking opposite value of screw_axis_ is because
  // jTi = Pose3::Expmap(-screw_axis_ * q) * jMi;
  Vector3 w = -screw_axis.head<3>();
  Vector3 v = -screw_axis.tail<3>();
  Pose3 kTj = Pose3::Expmap(-screw_axis * q) * jMi;
  auto w_skew = skewSymmetric(w);
  Matrix3 H_expo = w_skew * cosf(q) + w_skew * w_skew * sinf(q);
  Matrix3 H_R = H_expo * jMi.rotation().matrix();
  Vector3 H_T = H_expo * (jMi.translation().vector() - w_skew * v) +
                w * w.transpose() * v;
  Matrix3 H_TR = skewSymmetric(H_T) * kTj.rotation().matrix() +
                 skewSymmetric(kTj.translation().vector()) * H_R;
  Matrix6 H = Z_6x6;
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

Vector q_trajectory(int i, int total_step, Vector &start_q, Vector &end_q) {
  if (total_step > 1) {
    return start_q + (end_q - start_q) * i / (total_step - 1);
  } else {
    return start_q;
  }
}

vector<Point3> sphereCenters(double length, double radius, int num) {
  vector<Point3> sphere_centers;
  if (num == 1) {
    sphere_centers.push_back(Point3(0, 0, 0));
    return sphere_centers;
  }
  for (int i = 0; i < num; ++i) {
    // get sphere center expressed in link COM frame
    sphere_centers.push_back(Point3((2 * i + 1) * radius - 0.5 * length, 0, 0));
  }
  return sphere_centers;
}

}  // namespace manipulator
