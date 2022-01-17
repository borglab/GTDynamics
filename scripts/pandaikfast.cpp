#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
#include <iostream>
#include <vector>

using namespace gtdynamics;

int main(){
  std::cout << "Beautify answer or not? (y/n)" << std::endl;
  std::string s;
  std::cin >> s;
  bool beauty= (s=="y");

  std::vector<double> r(9,0);
  std::vector<double> p(3,0);
  double theta7;
  std::string rot, pos;
  while(std::cin >> theta7){
    for(int i=0; i<9; ++i){
        std::cin >> r[i];
    }
    for(int i=0; i<3; ++i){
        std::cin >> p[i];
    }
    gtsam::Rot3 bRe(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]);
    gtsam::Point3 bte = (gtsam::Point3() << p[0], p[1], p[2]).finished();
    gtsam::Pose3 bTe(bRe,bte);
    std::vector<gtsam::Vector7> solutions = PandaIKFast::inverse(bTe, theta7);
    int n = solutions.size();
    std::cout << std::endl << "Solutions (" << n << ") are:" << std::endl;
    for(int j=0; j<n; ++j) {
        if (beauty) std::cout << "  Sol#" << j << ": ";
        for(int i=0; i<7; ++i){
          std::cout << solutions[j][i] << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}