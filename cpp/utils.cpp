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

/** convert angle to radians
 */
double radians(double degree) { return degree * M_PI / 180; }

Vector radians(const Vector &degree) {
  Vector radian(degree.size());
  for (int i = 0; i < degree.size(); ++i) {
    radian(i) = radians(degree(i));
  }
  return radian;
}

Matrix getQc(const SharedNoiseModel Qc_model) {
  noiseModel::Gaussian *Gassian_model =
      dynamic_cast<noiseModel::Gaussian *>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

}  // namespace manipulator