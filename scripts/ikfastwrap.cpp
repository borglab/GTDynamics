/**
 * @file  testRoadMap.cpp
 * @brief test Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

//#include <gtdynamics/pandarobot/roadmap/RoadMap.h>
#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;

bool checkLimits(const Vector7& joint_state) {
  Vector7 lim_inf, lim_sup;
  lim_sup << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  lim_inf << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  for (size_t i = 0; i < 7; i++) {
    // Check if value is within boundaries
    if (lim_inf[i] > joint_state[i] || joint_state[i] > lim_sup[i]){
      return false;
    }
  }
  return true;
}

int main() {
  // Read rotation matrix
  Vector9 rot_v;
  std::cout << "Rot Matrix?" << std::endl;
  for (size_t i = 0; i < 9; i++) {
    std::cin >> rot_v[i];
  }
  Rot3 rot(rot_v[0], rot_v[1], rot_v[2], rot_v[3], rot_v[4], rot_v[5], rot_v[6],
           rot_v[7], rot_v[8]);

  Point3 pos;

  std::string s;
  do {
    // Read pose
    pos << 0, 0, 0;
    std::cout << "Pose position: ";
    for (size_t i = 0; i < 3; i++) {
      std::cin >> pos[i];
    }
    Pose3 pose(rot, pos);

    // Get num theta7 values
    size_t num_theta;
    std::cout << "Number of theta7 samples? ";
    std::cin >> num_theta;
    // Either compute them automatically or specify them
    std::string yesno;
    do {
      std::cout << "Specify values? (y/n): ";
      std::cin >> yesno;
    } while (yesno != "y" && yesno != "n");

    std::vector<double> theta_values(num_theta);
    // specifying them
    if (yesno == "y") {
      std::cout << "Theta7 values: ";
      for (size_t i = 0; i < num_theta; i++) {
        std::cin >> theta_values[i];
      }
      // automatically
    } else {
      double theta7_l_lim = -2.8973, theta7_r_lim = 2.8973;
      double interval = theta7_r_lim - theta7_l_lim;
      for (size_t i = 0; i < num_theta; ++i)
        theta_values[i] =
            theta7_l_lim + double((i + 1)) * interval / double((num_theta + 1));
    }

    std::vector<Vector7> statenodes;
    size_t num_deleted_states = 0;

    for (size_t j = 0; j < num_theta; j++) {
      std::vector<Vector7> solutions =
          PandaIKFast::inverse(pose, theta_values[j]);

      size_t n_sols = solutions.size();
      for (size_t sol_idx = 0; sol_idx < n_sols; ++sol_idx) {
        // check boundaries and add to counter if deleted
        statenodes.push_back(solutions[sol_idx]);
        if (!checkLimits(solutions[sol_idx])) {
          ++num_deleted_states;
        }
        // } else {
      }
    }

    std::cout << statenodes.size() << " (to be deleted: " << num_deleted_states
              << ") " << std::endl;

    for (size_t i = 0; i < statenodes.size(); i++) {
      std::cout << i << ":\t" << checkLimits(statenodes[i]) << "\t";
      for (size_t j = 0; j < 7; j++) {
        std::cout << statenodes[i][j] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  } while (std::cin >> s);
}