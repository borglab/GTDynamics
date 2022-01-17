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
#include <iostream>
#include <vector>

using namespace gtdynamics;
using namespace gtsam;

// Relationships will be a list of pairs for each state node. These pairs represent each a region of 
// Example: list is [(1,4),(8,10)]. This represents: [1,4) u [8,10) = {1,2,3,8,9}
std::vector<std::vector<std::pair<size_t, size_t>>> computeRelationship(
    RoadMap& roadmap, std::vector<std::vector<size_t>> poses_relationships) {

  const std::vector<Vector7>& states = roadmap.getstatenodes();
  std::vector<std::vector<std::pair<size_t, size_t>>> nodes_relationships(
      roadmap.getstatenodes().size());

  for (size_t i = 0; i < poses_relationships.size(); i++) {
    std::vector<size_t> nodes = roadmap.getStatesFromPose(i);
    // add possible relations between themselves: between nodes with same theta7
    // value and nodes with very similar theta7 values
    for (auto& parent : nodes) {
      size_t min = parent < 32 ? 0 : parent;
      size_t max = parent < nodes.size() ? parent : nodes.size();
      nodes_relationships[parent].push_back({min, max});
    }

    for (size_t j = 0; j < poses_relationships[i].size(); ++j) {
      // For each pose sufficiently close (poses_relationships), get its
      // corresponding nodes.
      std::vector<size_t> other_nodes =
          roadmap.getStatesFromPose(poses_relationships[i][j]);

      double current_theta_7 = states[nodes[0]](6) - 1;
      // Declare indices of the boundaries of the three sets of nodes: a little
      // smaller theta7 (by just one step), the current theta7 value and a
      // little bigger
      size_t kernel_size = 1;
      std::vector<size_t> less(kernel_size + 1, 0);
      std::vector<size_t> greater(kernel_size + 1, 0);

      // Then, for each node in nodes, we add as possible connections the nodes
      // in other_nodes that have a theta7 value sufficiently close. Since for
      // each pose, their corresponding nodes are ordered increasingly, we only
      // need to keep moving the "bucket" through the vector
      for (const auto& parent : nodes) {
        if (current_theta_7 < states[parent](6)) {
          current_theta_7 = states[parent](6);
          // update methodology for less
          while ((less[0] < other_nodes.size()) &&
                 (states[other_nodes[less[0]]](6) < current_theta_7)) {
            ++less[0];
          }

          for (size_t idx = 1; idx < less.size(); ++idx) {
            less[idx] = less[idx - 1];
            while ((0 < less[idx]) &&
                   (states[other_nodes[less[idx - 1] - 1]](6) <=
                    states[other_nodes[less[idx] - 1]](6))) {
              --less[idx];  // once value changes, we found the index limit
            }
          }

          // update methodology for greater
          while ((greater[0] < other_nodes.size()) &&
                 (states[other_nodes[greater[0]]](6) <= current_theta_7)) {
            ++greater[0];
          }
          for (size_t idx = 1; idx < greater.size(); ++idx) {
            greater[idx] = greater[idx - 1];
            while ((greater[idx] < other_nodes.size()) &&
                   (states[other_nodes[greater[idx]]](6) <=
                    states[other_nodes[greater[idx - 1]]](6)))
              ++greater[idx];
          }
          // std::cout << greater[1] << std::endl;
        }

        // Maybe the nodes in other_nodes are not contiguous
        size_t l, g;
        for (l = less.back(), g = less.back(); g + 1 < greater.back(); ++g) {
          // std::cout << "hey" << g << " " << greater.back() << std::endl;
          if (other_nodes[g] + 1 != other_nodes[g + 1]) {
            std::cout << "Wow" << std::endl;
            nodes_relationships[parent].push_back(
                {other_nodes[l], other_nodes[g] + 1});
            l = g + 1;
          }
          // std::cout << "ho, let's go" << g << " " << greater.back() <<
          // std::endl;
        }
        // std::cout << "less" << l << " " << less.back() << std::endl;
        // std::cout << "great" << g << " " << greater.back() << std::endl;
        if (g > 0)
          nodes_relationships[parent].push_back(
              {other_nodes[l], other_nodes[g] + 1});
      }
    }
  }

  return nodes_relationships;
}

int main() {
  Point3 A, B, C;
  A << 0.2, -0.3, 0.15;
  B << 0.4, 0.0, 0.15;
  C << 0.2, -0.3, 0.5;

  // Create object
  CanvasSampler canvas(A, B, C);

  // Uniform check if it's the same
  size_t numACsamples = 100, numABsamples = 100;
  std::vector<Pose3> poses = canvas.uniformSample(numABsamples, numACsamples);

  RoadMap roadmap;
  size_t theta7_samples = 200;
  std::cout << "arribes1" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of computing solutions: "
            << double(duration.count()) / 1000000.0 << std::endl;
  std::cout << "passes1, solutions: " << roadmap.getstatenodes().size()
            << std::endl;

  std::cout << "Deleted: " << roadmap.getnumdeleted() << std::endl;
  std::cout << "Not found: " << roadmap.getnumnotfound() << " of "
            << poses.size() * theta7_samples << std::endl;

  // Get poses relationships
  std::vector<std::vector<size_t>> poses_relationships(poses.size());
  for (size_t i = 0; i < numABsamples; ++i) {
    for (size_t j = 0; j < numACsamples; ++j) {
      size_t kernel_size = 1;
      for (size_t kernel_i = 0;
           kernel_i < 2 * kernel_size + 1 && kernel_i + i < numABsamples;
           ++kernel_i) {
        if (kernel_size <= i + kernel_i) {
          for (size_t kernel_j = 0;
               kernel_j < 2 * kernel_size + 1 && kernel_j + j < numACsamples;
               ++kernel_j) {
            if ((kernel_size <= j + kernel_j) &&
                (kernel_size <= i + kernel_i) &&
                !((kernel_size == kernel_i) && (kernel_size == kernel_j))) {
              poses_relationships[numACsamples * i + j].push_back(
                  numABsamples * (i - (kernel_size) + kernel_i) +
                  (j - (kernel_size) + kernel_j));
            }
          }
        }
      }
    }
  }

  std::cout << "Getting here!\n";

  start = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<std::pair<size_t, size_t>>> nodes_relationships =
      computeRelationship(roadmap, poses_relationships);
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of pre-computing references: "
            << double(duration.count()) / 1000000.0 << std::endl;

  start = std::chrono::high_resolution_clock::now();
  roadmap.set_threshold(0.075);
  std::cout << "arribes2" << std::endl;
  roadmap.createGraphFromReference(nodes_relationships);
  std::cout << "passes2" << std::endl;
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of creating graph from references: "
            << double(duration.count()) / 1000000.0 << std::endl;
  size_t count = 0;
  for (const auto& x : roadmap.getadjacencylist())
    for (const auto& y : x) count++;

  std::cout << "Number of edges graph:" << count << std::endl;
  nodes_relationships = std::vector<std::vector<std::pair<size_t, size_t>>>();

  Rot3 orientation = poses[0].rotation();
  /*
    std::vector<Pose3> stage_poses = {
        Pose3(orientation, (Point3() << 0.3, -0.1, 0.25).finished()),
        //Pose3(orientation, (Point3() << 0.3, -0.1, 0.33).finished()),
        //Pose3(orientation, (Point3() << 0.3, -0.07, 0.41).finished()),
        Pose3(orientation, (Point3() << 0.3, -0.02, 0.41).finished()),
    };
    */

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
        std::vector<size_t>& path = paths[i];
        std::cout << "path #" << i << ": " << path.size() << std::endl;
        for (size_t j = 0; j < path.size(); ++j) {
          // std::cout << "{" << path[j] << ", ("
          //          << roadmap.getPoseFromState(path[j]) / numACsamples << ",
          //          "
          //          << roadmap.getPoseFromState(path[j]) % numACsamples << ",
          //          "
          //          << roadmap.getstatenodes()[path[j]](6) << "), "
          //          << (roadmap.getstatenodes()[path[j + 1]] -
          //          roadmap.getstatenodes()
          //                                                   [path[j]])
          //                 .norm()
          //          << "},  " << std::endl;
          costs[i] += (roadmap.getstatenodes()[path[j + 1]] -
                       roadmap.getstatenodes()[path[j]])
                          .norm();
          Point3 pos = roadmap.getposes()[roadmap.getPoseFromState(path[j])]
                           .translation();
          // std::cout << roadmap.getstatenodes()[path[j]](6) << ", " << "(" <<
          // pos(0) << ", " << pos(1) << ", " << pos(2) << ")"
          std::cout << roadmap.getstatenodes()[path[j]].transpose()
                    << std::endl;
        }
        // std::cout << "{" << path.back() << ", ("
        //          << roadmap.getPoseFromState(path.back()) / numACsamples
        //          << ", "
        //          << roadmap.getPoseFromState(path.back()) % numACsamples
        //          << ", " << roadmap.getstatenodes()[path.back()](6) << ")"
        //          << "},  " << std::endl;
        std::cout << std::endl << std::endl;
      }

      std::cout << std::endl;
      for (size_t i = 0; i < 10; ++i) std::cout << "-";
      std::cout << std::endl << std::endl << "Costs:" << std::endl;
      for (size_t i = 0; i < costs.size(); ++i) {
        std::cout << i << ":\t " << costs[i] << std::endl;
      }
    }
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