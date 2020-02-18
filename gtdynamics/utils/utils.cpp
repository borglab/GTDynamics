/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  utils.h
 * @brief Utility methods.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#include "gtdynamics/utils/utils.h"

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

gtsam::Matrix6 AdjointMapJacobianQ(double q, const gtsam::Pose3 &jMi,
                                   const gtsam::Vector6 &screw_axis) {
  // taking opposite value of screw_axis_ is because
  // jTi = Pose3::Expmap(-screw_axis_ * q) * jMi;
  gtsam::Vector3 w = -screw_axis.head<3>();
  gtsam::Vector3 v = -screw_axis.tail<3>();
  gtsam::Pose3 kTj = gtsam::Pose3::Expmap(-screw_axis * q) * jMi;
  auto w_skew = gtsam::skewSymmetric(w);
  gtsam::Matrix3 H_expo = w_skew * cosf(q) + w_skew * w_skew * sinf(q);
  gtsam::Matrix3 H_R = H_expo * jMi.rotation().matrix();
  gtsam::Vector3 H_T = H_expo * (jMi.translation().vector() - w_skew * v) +
                       w * w.transpose() * v;
  gtsam::Matrix3 H_TR = gtsam::skewSymmetric(H_T) * kTj.rotation().matrix() +
                        gtsam::skewSymmetric(kTj.translation().vector()) * H_R;
  gtsam::Matrix6 H = gtsam::Z_6x6;
  gtsam::insertSub(H, H_R, 0, 0);
  gtsam::insertSub(H, H_TR, 3, 0);
  gtsam::insertSub(H, H_R, 3, 3);
  return H;
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
  std::vector<std::vector<gtsam::Point3>> shpere_centers_all;
  int dof = lengths.size();
  for (int j = 0; j < dof; ++j) {
    std::vector<gtsam::Point3> sphere_centers;
    if (lengths[j] == 0) {
      sphere_centers.assign(1, gtsam::Point3());
    } else {
      int num = ceil(lengths[j] / radii[j]);
      double distance = lengths[j] / num;
      for (int i = 0; i < num; ++i) {
        // get sphere center expressed in link COM frame
        sphere_centers.push_back(
            gtsam::Point3((i + 0.5) * distance - 0.5 * lengths[j], 0, 0));
      }
    }
    shpere_centers_all.push_back(sphere_centers);
  }
  return shpere_centers_all;
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

}  // namespace gtdynamics

namespace gtdynamics {

sdf::Model get_sdf(std::string sdf_file_path, std::string model_name) {
  auto sdf = sdf::readFile(sdf_file_path);

  sdf::Model model = sdf::Model();
  model.Load(sdf->Root()->GetElement("model"));

  // Check whether this is a world file, in which case we have to first
  // access the world element then check whether one of its child models
  // corresponds to model_name.
  if (model.Name() != "__default__") return model;

  // Load the world element.
  sdf::World world = sdf::World();
  world.Load(sdf->Root()->GetElement("world"));

  for (uint i = 0; i < world.ModelCount(); i++) {
    sdf::Model curr_model = *world.ModelByIndex(i);
    if (curr_model.Name() == model_name) return curr_model;
  }

  // TODO(aescontrela): Make this error message more clear.
  throw std::runtime_error("Model not found.");
}

gtsam::Pose3 parse_ignition_pose(ignition::math::Pose3d ignition_pose) {
  gtsam::Pose3 parsed_pose = gtsam::Pose3(
    gtsam::Rot3(gtsam::Quaternion(
      ignition_pose.Rot().W(), ignition_pose.Rot().X(),
      ignition_pose.Rot().Y(), ignition_pose.Rot().Z())),
    gtsam::Point3(
      ignition_pose.Pos()[0], ignition_pose.Pos()[1], ignition_pose.Pos()[2]));

  return parsed_pose;
}

}  // namespace gtdynamics
