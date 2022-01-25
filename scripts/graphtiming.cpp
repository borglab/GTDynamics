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
                           size_t theta7_samples, size_t kernel_size = 1,
                           size_t theta_kernel_size = 1) {
  Point3 A, B, C;
  A << 0.3, -0.3, 0.1;
  B << 0.3, -0.3, 0.6;
  C << 0.3, 0.3, 0.1;

  // Create object
  CanvasSampler canvas(A, B, C);
  std::vector<Pose3> poses = canvas.uniformSample(numABsamples, numACsamples);

  // Get poses locality
  std::vector<std::vector<size_t>> pose_locality =
      canvas.uniformPoseLocality(numABsamples, numACsamples, kernel_size);

  RoadMap roadmap;

  auto start = std::chrono::high_resolution_clock::now();
  roadmap.addPoseNodes(poses).computeStateSolutions(theta7_samples);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

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
                   double(roadmap.getposes().size() * theta7_samples) * 100.0 /
                   8.0
            << ")" << std::endl;

  std::cout << "Duration of finding solutions: "
            << double(duration.count()) / 1000000.0 << std::endl;

  start = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<std::pair<size_t, size_t>>> state_locality =
      roadmap.computeStateLocality(pose_locality, theta_kernel_size);
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of pre-computing state locality: "
            << double(duration.count()) / 1000000.0 << std::endl;

  roadmap.set_threshold(6.0 / double(theta7_samples));

  start = std::chrono::high_resolution_clock::now();
  roadmap.createGraphFromReference(state_locality);
  stop = std::chrono::high_resolution_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Duration of creating the graph from reference: "
            << double(duration.count()) / 1000000.0 << std::endl;

  state_locality = std::vector<std::vector<std::pair<size_t, size_t>>>();

  size_t count = 0;
  for (const auto& x : roadmap.getadjacencylist())
    for (const auto& y : x) count++;

  std::cout << "Edges roadmap: " << count << std::endl << std::endl;
  std::cout << "Distribution:" << std::endl;
  for (size_t i = 0; i < 9; i++)
  {
    std::cout << i << "\t";
  }
  std::cout << std::endl;
  for (size_t i = 0; i < 9; i++)
  {
    std::cout << roadmap.getsoldistribution()[i] << "\t";
  }
  std::cout << std::endl;
  

  return;
}

int main() {
  std::vector<size_t> AB_samples = {5, 10, 15, 20, 25, 40, 60, 70, 80, 90, 100, 120, 140};
  std::vector<double> timesreference(AB_samples.size());
  for (size_t i = 0; i < AB_samples.size(); i++) {
    auto start = std::chrono::high_resolution_clock::now();
    print_duration_chetao(AB_samples[i], AB_samples[i], AB_samples[i]);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    timesreference[i] = double(duration.count()) / 1000000.0;
    std::cout << std::endl;
    std::cout << "Reference:" << AB_samples[i]
              << " time:" << timesreference[i] << std::endl;
    std::cout << std::endl
              << std::endl
              << "-----------------" << std::endl
              << std::endl;

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

  std::cout << "Create graph by reference times:" << std::endl;
  for (size_t i = 0; i < AB_samples.size(); i++) {
    std::cout << "\t" << AB_samples[i] << ":\t" << timesreference[i]
              << std::endl;
  }
  std::cout << std::endl;

  std::cout << ":)" << std::endl;
}