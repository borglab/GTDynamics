#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <vector>

using namespace gtdynamics;

int main() {
  gtsam::Vector7 thetas;
  std::string s;
  do {
    std::cout << "thetas: ";
    for (int i = 0; i < 7; ++i) {
      std::cin >> thetas(i);
    }

    gtsam::Pose3 bTe = PandaIKFast::forward(thetas);
    std::cout << bTe << std::endl;

    std::cout << "continue?" << std::endl;
  } while (std::cin >> s);
}