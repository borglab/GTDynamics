/******************************* Panda Robot Wrapper Interface File *******************************/

namespace gtdynamics {

/****************************************** Nothing ******************************************/
#include <gtdynamics/pandarobot/roadmap/CanvasSampler.h>
class CanvasSampler {
  CanvasSampler(const gtsam::Point3 &A, const gtsam::Point3 &B,
                const gtsam::Point3 &C);
  std::vector<gtsam::Pose3> uniformSample(const size_t numABsamples,
                                          const size_t numACsamples);
  static std::vector<std::vector<size_t>> uniformPoseLocality(
      size_t numABsamples, size_t numACsamples, size_t kernel_size = 1);
};

#include <gtdynamics/pandarobot/roadmap/RoadMap.h>

/*
struct Heuristic {
  virtual void preprocess(const RoadMap& roadmap,
                          const std::vector<std::vector<size_t>>& waypoints);
  virtual double operator()(const RoadMap& roadmap, const Node& node);
};
*/

class RoadMap {
  RoadMap();
  
  const std::vector<gtsam::Pose3>& getposes() const;
  const std::vector<gtsam::Vector7>& getstatenodes() const;
  const std::vector<std::vector<Edge>>& getadjacencylist() const;
  size_t getPoseFromState(const size_t stateindex);
  const std::vector<size_t> getStatesFromPose(const size_t poseindex);

  void set_threshold(double threshold);
  void set_num_maxpaths(size_t num_maxpaths);

  RoadMap& addPoseNodes(const std::vector<gtsam::Pose3>& poses);
  RoadMap& addStateNodes(const std::vector<gtsam::Vector7>& states);
  void computeStateSolutions(const size_t theta7_samples);
  std::vector<std::vector<std::pair<size_t, size_t>>>
  computeStateLocality(
      const std::vector<std::vector<size_t>>& pose_locality,
      size_t theta_kernel_size = 1);

  void createGraphFromReference(
      const std::vector<std::vector<std::pair<size_t, size_t>>>& reference);
        size_t findClosestNodeState(const gtsam::Vector7& state);
  const std::vector<size_t> findClosestNodesPose(const gtsam::Pose3& pose);
  /*
  std::vector<std::vector<size_t>> findPath(
      const std::vector<std::vector<size_t>>& waypoint_sets, Heuristic* h,
      const std::string& save_path = "");
    */  

};

}  // namespace gtdynamics
