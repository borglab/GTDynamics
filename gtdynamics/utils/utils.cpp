/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.cpp
 * @brief Utility methods.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela
 */

#include <gtdynamics/utils/utils.h>

#include <stdexcept>

namespace gtdynamics {

gtsam::Vector6 unit_twist(const gtsam::Vector3 &w, const gtsam::Vector3 &p) {
  gtsam::Vector6 unit_twist;
  unit_twist << w, p.cross(w);
  return unit_twist;
}

double radians(double degree) { return degree * M_PI / 180; }

gtsam::Vector radians(const gtsam::Vector &degree) {
  gtsam::Vector radian(degree.size());
  for (int i = 0; i < degree.size(); ++i) {
    radian(i) = radians(degree(i));
  }
  return radian;
}

gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model) {
  gtsam::noiseModel::Gaussian *Gassian_model =
      dynamic_cast<gtsam::noiseModel::Gaussian *>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

gtsam::Vector q_trajectory(int i, int total_step,
                           gtsam::Vector &start_q,  // NOLINT
                           gtsam::Vector &end_q) {  // NOLINT
  if (total_step > 1) {
    return start_q + (end_q - start_q) * i / (total_step - 1);
  } else {
    return start_q;
  }
}

std::vector<std::vector<gtsam::Point3>> sphereCenters(
    std::vector<double> lengths, std::vector<double> radii) {
  std::vector<std::vector<gtsam::Point3>> sphere_centers_all;
  int dof = lengths.size();
  for (int j = 0; j < dof; ++j) {
    std::vector<gtsam::Point3> sphere_centers;
    if (lengths[j] == 0) {
      sphere_centers.assign(1, gtsam::Point3(0, 0, 0));
    } else {
      int num = ceil(lengths[j] / radii[j]);
      double distance = lengths[j] / num;
      for (int i = 0; i < num; ++i) {
        // get sphere center expressed in link COM frame
        sphere_centers.push_back(
            gtsam::Point3((i + 0.5) * distance - 0.5 * lengths[j], 0, 0));
      }
    }
    sphere_centers_all.push_back(sphere_centers);
  }
  return sphere_centers_all;
}

std::vector<gtsam::Pose3> circle(int numOfWayPoints, double goalAngle,
                                 double radius) {
  double angle_step = goalAngle / (numOfWayPoints - 1);
  double angle = 0.0;
  std::vector<gtsam::Pose3> path;
  gtsam::Pose3 waypose;
  for (int i = 0; i < numOfWayPoints; ++i) {
    angle = angle_step * i;
    waypose = gtsam::Pose3(
        gtsam::Rot3::Rz(angle),
        gtsam::Point3(radius * cos(angle), radius * sin(angle), 0));
    path.push_back(waypose);
  }
  return path;
}

std::vector<gtsam::Pose3> square(int numOfWayPoints, double goalAngle,
                                 double length) {
  double angle_step = goalAngle / (numOfWayPoints - 1);
  double angle = 0.0;
  std::vector<gtsam::Pose3> path;
  gtsam::Pose3 waypose;
  double x = 0.0, y = 0.0;
  for (int i = 0; i < numOfWayPoints; ++i) {
    angle = angle_step * i;
    if (i <= 0.5 * numOfWayPoints) {
      x = length;
      y = x * tan(angle);
    } else {
      y = length;
      x = y / tan(angle);
    }
    waypose = gtsam::Pose3(gtsam::Rot3::Rz(angle), gtsam::Point3(x, y, 0));
    path.push_back(waypose);
  }
  return path;
}

std::vector<gtsam::Matrix> readFromTxt(std::string mat_dir,
                                       gtsam::Point3 &origin,  // NOLINT
                                       double &cell_size) {    // NOLINT
  std::vector<gtsam::Matrix> data;
  std::ifstream is;
  is.open(mat_dir);
  if (!is.is_open()) {
    std::cout << "failed to open file" << std::endl;
  } else {
    // read origin of sdf
    double x, y, z;
    is >> x >> y >> z;
    origin = gtsam::Point3(x, y, z);

    // read cell size of sdf
    is >> cell_size;

    // read filed rows, cols, and z
    int field_rows, field_cols, field_z;
    is >> field_rows >> field_cols >> field_z;

    // read filed data
    for (int k = 0; k < field_z; ++k) {
      gtsam::Matrix data_slice = gtsam::Matrix::Zero(field_rows, field_cols);
      for (int i = 0; i < field_rows; ++i) {
        for (int j = 0; j < field_cols; ++j) {
          is >> data_slice(i, j);
        }
      }
      data.push_back(data_slice);
    }
  }
  // generate sdf
  return data;
}

gtsam::Matrix36 getPlanarJacobian(const gtsam::Vector3 &planar_axis) {
  gtsam::Matrix36 H_wrench;
  if (planar_axis[0] == 1) {  // x axis
    H_wrench << 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0;
  } else if (planar_axis[1] == 1) {  // y axis
    H_wrench << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;
  } else if (planar_axis[2] == 1) {  // z axis
    H_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  }
  return H_wrench;
}

}  // namespace gtdynamics

namespace gtsam {
double point3_z(const gtsam::Point3 &p, gtsam::OptionalJacobian<1, 3> H) {
  if (H) {
    *H << 0, 0, 1;
  }
  return p.z();
}

double double_division(const double &x1, const double &x2,
                       gtsam::OptionalJacobian<1, 1> H_1,
                       gtsam::OptionalJacobian<1, 1> H_2) {

  double result = x1 / x2;
  if (H_1) {
    H_1->setConstant(1 / x2);
  }
  if (H_2) {
    H_2->setConstant(-result / x2);
  }
  return x1 / x2;
}

double reciprocal(const double &x, gtsam::OptionalJacobian<1, 1> H) {
  if (H) {
    H->setConstant(-pow(x, -2));
  }
  return 1 / x;
}

double clip_by_one(const double &x, gtsam::OptionalJacobian<1, 1> H) {
  if (x < 1) {
    if (H) {
      H->setZero();
    }
    return 1;
  } else {
    if (H) {
      H->setOnes();
    }
    return x;
  }
}
}
