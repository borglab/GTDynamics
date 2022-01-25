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
  size_t numACsamples = 100, numABsamples = 100;
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

    std::cout << std::endl << "Distribution before check:" << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << i << "\t";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << roadmap.getbeforecheckdistribution()[i] << "\t";
    }
    std::cout << std::endl;
    std::cout << std::endl << "Distribution after check:" << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << i << "\t";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < 9; i++) {
      std::cout << roadmap.getaftercheckdistribution()[i] << "\t";
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

  Rot3 orientation = poses[0].rotation();

  size_t numStagePoints;
  std::cout << "Number of intermediate stage poses?" << std::endl;
  while (std::cin >> numStagePoints) {
    std::vector<Pose3> stage_poses(numStagePoints + 2);
    for (size_t i = 0; i < numStagePoints + 2; ++i) {
      if (i == 0)
        std::cout << "Start pose:\t";
      else if (i == numStagePoints + 1)
        std::cout << "End pose:\t";
      else
        std::cout << "Intermediate pose #" << i << ":\t";
      Point3 pos = (Point3() << 0, 0, 0).finished();
      std::cin >> pos(0) >> pos(1) >> pos(2);
      stage_poses[i] = Pose3(orientation, pos);
    }

    std::vector<std::vector<size_t>> stage_nodes(stage_poses.size());
    for (size_t i = 0; i < stage_poses.size(); i++) {
      stage_nodes[i] = roadmap.findClosestNodesPose(stage_poses[i]);
    }

    roadmap.set_num_maxpaths(1);
    {
      Heuristic h;

      start = std::chrono::high_resolution_clock::now();
      std::cout << "arribes3" << std::endl;
      std::vector<std::vector<size_t>> paths =
          roadmap.findPath(stage_nodes, &h);
      std::cout << "passes3" << std::endl;
      stop = std::chrono::high_resolution_clock::now();
      duration =
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

      std::cout << "Duration of dijsktra: "
                << double(duration.count()) / 1000000.0 << std::endl;

      std::cout << "nº of solutions: " << paths.size() << std::endl;

      std::vector<double> costs(paths.size());

      for (size_t i = 0; i < paths.size(); ++i) {
        std::ofstream output("/home/jubes/files/jointpath.txt");
        std::vector<size_t>& path = paths[i];
        std::cout << "path #" << i << ": " << path.size() << std::endl;
        for (size_t j = 0; j < path.size(); ++j) {
          std::cout << "{" << path[j] << ", ("
                    << roadmap.getPoseFromState(path[j]) / numACsamples << ", "
                    << roadmap.getPoseFromState(path[j]) % numACsamples << ", "
                    << roadmap.getstatenodes()[path[j]](6) << "), "
                    << (roadmap.getstatenodes()[path[j + 1]] -
                        roadmap.getstatenodes()[path[j]])
                           .norm()
                    << "},  " << std::endl;
          costs[i] += (roadmap.getstatenodes()[path[j + 1]] -
                       roadmap.getstatenodes()[path[j]])
                          .norm();
          output << roadmap.getstatenodes()[path[j]].transpose() << std::endl;

          // Point3 pos = roadmap.getposes()[roadmap.getPoseFromState(path[j])]
          //                 .translation();
          // std::cout << roadmap.getstatenodes()[path[j]](6) << ", " << "(" <<
          // pos(0) << ", " << pos(1) << ", " << pos(2) << ")"
          // std::cout << roadmap.getstatenodes()[path[j]].transpose()
          //          << std::endl;
          // std::cout << path[j] << std::endl;
        }
        output.close();
        std::cout << "{" << path.back() << ", ("
                  << roadmap.getPoseFromState(path.back()) / numACsamples
                  << ", "
                  << roadmap.getPoseFromState(path.back()) % numACsamples
                  << ", " << roadmap.getstatenodes()[path.back()](6) << ")"
                  << "},  " << std::endl;
        std::cout << std::endl << std::endl;
      }

      std::cout << std::endl;
      for (size_t i = 0; i < 10; ++i) std::cout << "-";
      std::cout << std::endl << std::endl << "Costs:" << std::endl;
      for (size_t i = 0; i < costs.size(); ++i) {
        std::cout << i << ":\t " << costs[i] << std::endl;
      }
    }
    std::cout << "Number of intermediate stage poses?" << std::endl;
  }

  /*
  {
    DirectDistance h;
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "arribes4" << std::endl;
    std::vector<std::vector<size_t>> paths = roadmap.findPath(stage_nodes, &h);
    std::cout << "passes4" << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Duration of dijsktra: " << double(duration.count())/1000000.0
  << std::endl;

    bool one = paths.size() == 1;
    std::cout << "nº of solutions: " << one << std::endl;

    if (one) {
      std::vector<size_t>& path = paths[0];
      std::cout << "path: ";
      for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "(" << path[i] << "," << roadmap.getPoseFromState(path[i])
                  << "),  ";
      }
      std::cout << std::endl;
    }
  }*/
}