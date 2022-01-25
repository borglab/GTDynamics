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
      size_t max = parent < 32 ? 32 : parent;
      size_t min = parent < nodes.size() ? parent : nodes.size();
      nodes_relationships[parent].push_back({max, min});
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

void print_duration(size_t numABsamples, size_t numACsamples,
                    size_t theta7_samples) {
  Point3 A, B, C;
  A << 0.3, -0.3, 0.1;
  B << 0.3, -0.3, 0.6;
  C << 0.3, 0.3, 0.1;

  // Create object
  CanvasSampler canvas(A, B, C);

  // Uniform check if it's the same
  std::vector<Pose3> poses = canvas.uniformSample(numABsamples, numACsamples);

  RoadMap roadmap;
  std::cout << "arribes1, numposes: " << poses.size() << std::endl;

  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);
  std::cout << "passes1, numsolutions: " << roadmap.getstatenodes().size()
            << ", poses: " << roadmap.getposes().size() << std::endl;

  roadmap.set_threshold(6.0 / double(theta7_samples));
  std::cout << "arribes2" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  roadmap.createGraph();
  std::cout << "passes2" << std::endl;

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of (" << numABsamples << "x" << numACsamples
            << ") and theta7 (" << theta7_samples << "): " << duration.count()
            << std::endl;

  size_t count = 0;
  for (const auto& x : roadmap.getadjacencylist())
    for (const auto& y : x) count++;

  std::cout << "Edges roadmap normal: " << count << std::endl;

  return;
}

void print_duration_chetao(size_t numABsamples, size_t numACsamples,
                           size_t theta7_samples) {
  Point3 A, B, C;
  A << 0.3, -0.3, 0.1;
  B << 0.3, -0.3, 0.6;
  C << 0.3, 0.3, 0.1;

  // Create object
  CanvasSampler canvas(A, B, C);
  std::vector<Pose3> poses = canvas.uniformSample(numABsamples, numACsamples);
  /*
    // Uniform check if it's the same
    std::vector<std::vector<size_t>> poses_relationships(poses.size());
    for (size_t i = 0; i < numABsamples; ++i) {
      for (size_t j = 0; j < numACsamples; ++j) {
        size_t kernel_size = 3;
        for (size_t kernel_i = 0; kernel_i < 2 * kernel_size + 1; ++kernel_i) {
          if (kernel_size <= i + kernel_i) {
            for (size_t kernel_j = 0; kernel_j < 2 * kernel_size + 1;
                 ++kernel_j) {
              if ((kernel_size <= j + kernel_j) &&
                  !((kernel_size == kernel_i) && (kernel_size == kernel_j))) {
                poses_relationships[numABsamples * i + j].push_back(
                    numABsamples * (i - (kernel_size) + kernel_i) +
                    (j - (kernel_size) + kernel_j));
              }
            }
          }
        }
      }
    }
    */// Get poses relationships
  std::vector<std::vector<size_t>> poses_relationships(poses.size());
  for (size_t i = 0; i < numABsamples; ++i) {
    for (size_t j = 0; j < numACsamples; ++j) {
      size_t kernel_size = 3;
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
  /*
    std::cout << "pose relationships:" << std::endl;
    for (size_t i = 0; i < poses_relationships.size(); i++) {
      std::cout << "Pose " << i << ":";
      for (const auto& x : poses_relationships[i]) std::cout << " " << x;
      std::cout << std::endl;
    }
  */
  std::cout << std::endl << std::endl;

  RoadMap roadmap;
  // std::cout << "arribes1, numposes: " << poses.size() << std::endl;

  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);

  std::cout << "passes1, numsolutions: " << roadmap.getstatenodes().size()
            << ", poses: " << roadmap.getposes().size() << std::endl;

  std::vector<std::vector<std::pair<size_t, size_t>>> nodes_relationships =
      computeRelationship(roadmap, poses_relationships);

  /*
  const std::vector<Vector7>& states = roadmap.getstatenodes();
  std::vector<std::vector<std::pair<size_t, size_t>>> nodes_relationships(
      roadmap.getstatenodes().size());

  for (size_t i = 0; i < poses_relationships.size(); i++) {
    std::vector<size_t> nodes = roadmap.getStatesFromPose(i);
    // add possible relations between themselves: between nodes with same
  theta7
    // value and nodes with very similar theta7 values
    for (auto& parent : nodes) {
      size_t max = parent < 32 ? 32 : parent;
      size_t min = parent < nodes.size() ? parent : nodes.size();
      nodes_relationships[parent].push_back({max, min});
    }

    for (size_t j = 0; j < poses_relationships[i].size(); ++j) {
      // For each pose sufficiently close (poses_relationships), get its
      // corresponding nodes.
      std::vector<size_t> other_nodes =
          roadmap.getStatesFromPose(poses_relationships[i][j]);

      double current_theta_7 = states[nodes[0]](6) - 1;
      // Declare indices of the boundaries of the three sets of nodes: a
  little
      // smaller theta7 (by just one step), the current theta7 value and a
      // little bigger
      size_t kernel_size = 1;
      std::vector<size_t> less(kernel_size + 1, 0);
      std::vector<size_t> greater(kernel_size + 1, 0);

      // Then, for each node in nodes, we add as possible connections the
  nodes
      // in other_nodes that have a theta7 value sufficiently close. Since
  for
      // each pose, their corresponding nodes are ordered increasingly, we
  only
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
        for (l = less.back(), g = less.back(); g + 1 < greater.back(); ++g)
  {
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
        //std::cout << "less" << l << " " << less.back() << std::endl;
        //std::cout << "great" << g << " " << greater.back() << std::endl;
        if (g > 0)
          nodes_relationships[parent].push_back(
              {other_nodes[l], other_nodes[g] + 1});
      }
    }
  }

  size_t count = 0;
  for (const auto& x : nodes_relationships)
    for (const auto& y : x) count++;

  std::cout << "Edges naive: " << count << std::endl;
  */
  /*
  for (size_t i = 0; i < poses_relationships.size(); i++) {
    std::vector<size_t> nodes = roadmap.getStatesFromPose(i);
    for (auto& parent : nodes) {
      for (auto& child : nodes) {
        if ((child + 32 > parent) && (child < parent+32) &&(parent !=
  child)) nodes_relationships[parent].push_back(child);
      }
    }
    // also add possible relations between themselves

    for (size_t j = 0; j < poses_relationships[i].size(); ++j) {
      std::vector<size_t> other_nodes =
          roadmap.getStatesFromPose(poses_relationships[i][j]);
      // could be done with concatenate like in other cases in roadmap.cpp
      for (const auto& parent : nodes) {
        for (const auto& child : other_nodes) {
          nodes_relationships[parent].push_back(child);
        }
      }
    }
  }*/
  /*
  std::cout << "nodes relationships:" << std::endl;
  for (size_t i = 0; i < 20; i++) {
    std::cout << "Node " << i << ":";
    for (const auto& x : nodes_relationships[i]) std::cout << " " << x;
    std::cout << std::endl;
  }
  */
  roadmap.set_threshold(6.0 / double(theta7_samples));
  std::cout << "arribes2" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  roadmap.createGraphFromReference(nodes_relationships);
  std::cout << "passes2" << std::endl;

  nodes_relationships = std::vector<std::vector<std::pair<size_t, size_t>>>();

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of (" << numABsamples << "x" << numACsamples
            << ") and theta7 (" << theta7_samples
            << "): " << double(duration.count()) / 1000000.0 << std::endl;

  size_t count = 0;
  for (const auto& x : roadmap.getadjacencylist())
    for (const auto& y : x) count++;

  std::cout << "Edges roadmap: " << count << std::endl;

  return;
}

int main() {
  std::vector<size_t> AB_samples = {5, 10, 15, 20, 25, 40, 60, 70, 80, 90, 100};
  for (size_t i = 0; i < AB_samples.size(); i++) {
    auto start = std::chrono::high_resolution_clock::now();
    print_duration_chetao(AB_samples[i], AB_samples[i], AB_samples[i]);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Chetao:" << AB_samples[i]
              << " time:" << double(duration.count()) / 1000000.0 << std::endl;

    /*
    start = std::chrono::high_resolution_clock::now();
    print_duration(AB_samples[i], AB_samples[i], AB_samples[i]);
    stop = std::chrono::high_resolution_clock::now();
    duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Normal" << AB_samples[i]
              << " time:" << double(duration.count()) / 1000000.0 << std::endl;
              */
  }
  std::cout << ":)" << std::endl;
}