/**
 * @file  testRoadMap.cpp
 * @brief test Roadmap
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#include <gtdynamics/pandarobot/roadmap/CanvasSampler.h>
#include <gtdynamics/pandarobot/roadmap/RoadMap.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;

// Relationships will be a list of pairs for each state node. These pairs
// represent each a region of Example: list is [(1,4),(8,10)]. This represents:
// [1,4) u [8,10) = {1,2,3,8,9}

int main() {
  Point3 A, B, C;
  A << 0.3, 0.250, 0.35;
  B << 0.3, 0.250, 0.65;
  C << 0.3, -0.25, 0.35;

  // Create object
  CanvasSampler canvas(A, B, C);

  // Uniform check if it's the same
  size_t numACsamples = 5, numABsamples = 5;
  std::vector<Pose3> poses = canvas.uniformSample(numABsamples, numACsamples);

  RoadMap roadmap;
  size_t theta7_samples = 600;
  auto start = std::chrono::high_resolution_clock::now();
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  {
    std::cout << "poses: " << roadmap.getposes().size() << std::endl
              << "number of theta points: " << theta7_samples << std::endl
              << "poses x theta: " << roadmap.getposes().size() * theta7_samples
              << std::endl
              << "did not find solution of " << roadmap.getnumnotfound()
              << " pose-theta points ("
              << 100.0 * double(roadmap.getnumnotfound()) /
                     double(roadmap.getposes().size() * theta7_samples)
              << "%)" << std::endl
              << "numsolutions: " << roadmap.getstatenodes().size() << std::endl
              << "deleted: " << roadmap.getnumdeleted() << " ("
              << 100 * (double(roadmap.getnumdeleted()) /
                        double(roadmap.getnumdeleted() +
                               roadmap.getstatenodes().size()))
              << "%)" << std::endl
              << "Average numsolutions per point: "
              << double(roadmap.getstatenodes().size()) /
                     double(roadmap.getposes().size() * theta7_samples)
              << " (%/8: "
              << double(roadmap.getstatenodes().size()) /
                     double(roadmap.getposes().size() * theta7_samples) *
                     100.0 / 8.0
              << ")" << std::endl;

    std::cout << "Duration of finding solutions: "
              << double(duration.count()) / 1000000.0 << std::endl;

    std::cout << std::endl << "Distribution:" << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << i << "\t";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << roadmap.getsoldistribution()[i] << "\t";
    }
    std::cout << std::endl;
  }

  // Get poses relationships
  size_t kernel_size = 1;
  size_t theta_kernel_size = 2;
  start = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<size_t>> pose_locality =
      canvas.uniformPoseLocality(numABsamples, numACsamples, kernel_size);

  std::vector<std::vector<std::pair<size_t, size_t>>> states_locality =
      roadmap.computeStateLocality(pose_locality, theta_kernel_size);
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of pre-computing references: "
            << double(duration.count()) / 1000000.0 << std::endl;

  start = std::chrono::high_resolution_clock::now();
  roadmap.set_threshold(0.075);
  roadmap.createGraphFromReference(states_locality);
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of creating graph from references: "
            << double(duration.count()) / 1000000.0 << std::endl;
  size_t count = 0;
  for (const auto& x : roadmap.getadjacencylist())
    for (const auto& y : x) count++;

  std::cout << "Number of edges graph:" << count << std::endl;
  states_locality = std::vector<std::vector<std::pair<size_t, size_t>>>();

  std::string save_path = "/home/jubes/files/graph.txt";

  std::ofstream output(save_path);
  
  output << numABsamples << " " << numACsamples << std::endl;
  for (size_t i = 0; i < roadmap.getstatenodes().size(); i++) {
    output << roadmap.getPoseFromState(i) << " " << roadmap.getstatenodes()[i](6);
    for (RoadMap::Edge child : roadmap.getadjacencylist()[i])
    {
      output << " (" << child.first << "-" << child.second << ")";
    }
    output << std::endl;
    
  }
  
  output.close();
}