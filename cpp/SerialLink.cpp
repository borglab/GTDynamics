/**
 * @file  serial_link.cpp
 * @brief manipulator links
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <DHLink.h>
#include <SerialLink.h>
#include <URDFLink.h>

using namespace std;
using namespace gtsam;

namespace manipulator {

template <typename T>
SerialLink<T>::SerialLink(const std::vector<T> &links, const Pose3 &base,
                          const Pose3 &tool)
    : links_(links), base_(base), tool_(tool) {
  // Calculate screw axes for all joints, expressed in their COM frame.
  for (auto &link : links_) {
    screwAxes_.push_back(link.screwAxis());
  }
}

template <typename T>
Vector SerialLink<T>::jointLowerLimits() const {
  Vector lower_limits = Vector::Zero(numLinks());
  for (int i = 0; i < numLinks(); ++i) {
    lower_limits[i] = links_[i].jointLowerLimit();
  }
  return lower_limits;
}

template <typename T>
Vector SerialLink<T>::jointUpperLimits() const {
  Vector upper_limits = Vector::Zero(numLinks());
  for (int i = 0; i < numLinks(); ++i) {
    upper_limits[i] = links_[i].jointUpperLimit();
  }
  return upper_limits;
}

template <typename T>
Vector SerialLink<T>::jointLimitThresholds() const {
  Vector limit_thresholds = Vector::Zero(numLinks());
  for (int i = 0; i < numLinks(); ++i) {
    limit_thresholds[i] = links_[i].jointLimitThreshold();
  }
  return limit_thresholds;
}

template <typename T>
vector<Pose3> SerialLink<T>::linkTransforms(const Vector &default_q) const {
  Vector q = default_q;
  if (equal(default_q, Vector::Zero(1))) {
    q = Vector::Zero(numLinks());
  }
  vector<Pose3> transforms;
  for (int i = 0; i < numLinks(); ++i) {
    transforms.push_back(links_[i].A(q[i]));
  }
  return transforms;
}

template <typename T>
vector<Pose3> SerialLink<T>::forwardKinematics(
    const Vector &q, boost::optional<std::vector<gtsam::Matrix> &> J) const {
  vector<Pose3> sTb;
  Pose3 t = base_;
  vector<Pose3> transforms = linkTransforms(q);
  for (auto const &A : transforms) {
    t = t * A;
    sTb.push_back(t);
  }
  sTb.push_back(t * tool_);

  if (J) {
    // body_manipulator_jacobian in body frame
    *J = bodyManipulatorJacobian(q, sTb);
  }

  return sTb;
}

template <typename T>
vector<Pose3> SerialLink<T>::linkFrames(const Vector &q) const {
  vector<Pose3> frames;
  Pose3 t = base_;
  vector<Pose3> transforms = linkTransforms(q);
  for (auto const &A : transforms) {
    t = t * A;
    frames.push_back(t);
  }
  return frames;
}

template <typename T>
vector<Pose3> SerialLink<T>::comFrames(const Vector &q) const {
  Pose3 t = base_;
  Pose3 iTcom;
  vector<Pose3> frames;
  vector<Pose3> transforms = linkTransforms(q);
  for (int i = 0; i < numLinks(); ++i) {
    t = t * transforms[i];
    iTcom = links_[i].centerOfMass();
    frames.push_back(t * iTcom);
  }
  return frames;
}

template <typename T>
vector<Pose3> SerialLink<T>::transformPOE(const Vector &default_q) const {
  Vector q = default_q;
  if (equal(default_q, Vector::Zero(1))) {
    q = Vector::Zero(numLinks());
  }
  vector<Pose3> frames;
  Pose3 frame = Pose3();
  frames.push_back(frame);
  vector<Vector6> screw_axes = spatialScrewAxes();
  for (int i = 0; i < numLinks(); ++i) {
    frame = frames.back() * Pose3::Expmap(screw_axes[i] * q[i]);
    frames.push_back(frame);
  }
  vector<Pose3> part(frames.begin() + 1, frames.end());
  return part;
}

template <typename T>
vector<Vector6> SerialLink<T>::spatialScrewAxes() const {
  vector<Vector6> spatial_screwAxes_vec;
  vector<Pose3> frames = comFrames();
  for (int i = 0; i < numLinks(); ++i) {
    spatial_screwAxes_vec.push_back(frames[i].AdjointMap() * screwAxes_[i]);
  }
  return spatial_screwAxes_vec;
}

template <typename T>
vector<Matrix> SerialLink<T>::spatialManipulatorJacobian(
    const Vector &q) const {
  // Calculate spatial_screw_axes at q
  vector<Vector6> screw_axes = spatialScrewAxes();
  // Calculate joint frames expressed in world frame
  vector<Pose3> transforms = transformPOE(q);
  vector<Matrix> Js(numLinks() + 1, Matrix::Zero(6, numLinks()));

  Matrix Jst_eef = Matrix::Zero(6, numLinks());
  Jst_eef.col(0) = screw_axes[0];
  for (int i = 1; i < numLinks(); ++i) {
    Jst_eef.col(i) = transforms[i - 1].AdjointMap() * screw_axes[i];
  }

  for (int i = 0; i < numLinks(); ++i) {
    insertSub(Js[i], sub(Jst_eef, 0, 6, 0, i + 1), 0, 0);
  }
  Js[numLinks()] = Jst_eef;

  return Js;
}

template <typename T>
vector<Matrix> SerialLink<T>::bodyManipulatorJacobian(
    const Vector &q, vector<Pose3> &sTb) const {
  // Calculate spatial_manipulator_jacobian
  vector<Matrix> Js = spatialManipulatorJacobian(q);
  // assign space for manipulator jacobian
  vector<Matrix> Jb(numLinks() + 1, Matrix::Zero(6, numLinks()));
  // Convert spatial_manipulator_jacobian to body_manipulator_jacobian,
  // Lynch & Park's book page 187, equation (5.22)
  for (int i = 0; i <= numLinks(); ++i) {
    Jb[i] = sTb[i].inverse().AdjointMap() * Js[i];
  }
  return Jb;
}

template <typename T>
vector<Vector6> SerialLink<T>::twists(const std::vector<Pose3> &Ts,
                                      const Vector &joint_velocities) const {
  vector<Vector6> twists;
  twists.push_back(screwAxes_[0] * joint_velocities[0]);
  Vector6 twist_i;
  Vector6 twist_j;
  Vector6 Aj;
  double joint_vel_j;
  Pose3 jTi;

  for (int j = 2; j <= numLinks(); ++j) {
    twist_i = twists.back();
    jTi = Ts[j - 1].between(Ts[j - 2]);
    Aj = screwAxes_[j - 1];
    joint_vel_j = joint_velocities[j - 1];
    twist_j = jTi.AdjointMap() * twist_i + Aj * joint_vel_j;
    twists.push_back(twist_j);
  }
  return twists;
}

template <typename T>
vector<Pose3> SerialLink<T>::jTi_list(const Vector &q) const {
  vector<Pose3> Ts = comFrames(q);
  Pose3 bT1 = Ts[0].between(base_);
  Pose3 nTt = tool_;
  Pose3 nTc = links_.back().centerOfMass();
  Pose3 tTnc = nTt.between(nTc);
  vector<Pose3> jTi_com;
  jTi_com.push_back(bT1);
  for (int j = 1; j < numLinks(); ++j) {
    jTi_com.push_back(Ts[j].between(Ts[j - 1]));
  }
  jTi_com.push_back(tTnc);
  return jTi_com;
}

template <typename T>
GaussianFactorGraph SerialLink<T>::forwardDynamicsFactorGraph(
    const Vector &q, const Vector &joint_velocities, const Vector &torques,
    const Vector6 &base_twist_accel, const Vector6 &external_wrench,
    boost::optional<Vector3 &> gravity) const {
  int N = numLinks();
  assert(q.size() == N);
  assert(joint_velocities.size() == N);
  assert(torques.size() == N);

  // Configuration of COM link frames
  vector<Pose3> Ts = comFrames(q);
  // Calculate all twists
  vector<Vector6> twists_vec = twists(Ts, joint_velocities);
  // Set up Gaussian Factors Graph
  GaussianFactorGraph gfg = GaussianFactorGraph();
  // Add factor to enforce base acceleration
  gfg.add(Link::BaseTwistAccelFactor(base_twist_accel));
  // Configuration of link frame j-1 relative to link frame j for arbitrary
  // joint angle
  vector<Pose3> jTis = jTi_list(q);

  int j = 0;
  for (int i = 0; i < N; ++i) {
    j = i + 1;
    auto jRw = Ts[i].rotation().inverse();
    if (gravity) {
      gfg.push_back(links_[i].forwardFactors(j, jTis[i], joint_velocities[i],
                                             twists_vec[i], torques[i], jTis[j],
                                             jRw * (*gravity)));
    } else {
      gfg.push_back(links_[i].forwardFactors(j, jTis[i], joint_velocities[i],
                                             twists_vec[i], torques[i], jTis[j],
                                             gravity));
    }
  }
  // Add factor to enforce external wrench at tool
  gfg.add(Link::ToolWrenchFactor(N, external_wrench));

  return gfg;
}

template <typename T>
GaussianFactorGraph SerialLink<T>::inverseDynamicsFactorGraph(
    const Vector &q, const Vector &joint_velocities,
    const Vector &joint_accelerations, const Vector6 &base_twist_accel,
    const Vector6 &external_wrench, boost::optional<Vector3 &> gravity) const {
  int N = numLinks();
  assert(q.size() == N);
  assert(joint_velocities.size() == N);
  assert(joint_accelerations.size() == N);

  // Configuration of COM link frames
  vector<Pose3> Ts = comFrames(q);
  // Calculate all twists
  vector<Vector6> twists_vec = twists(Ts, joint_velocities);
  // Set up Gaussian Factors Graph
  GaussianFactorGraph gfg = GaussianFactorGraph();
  // Add factor to enforce base acceleration
  gfg.add(Link::BaseTwistAccelFactor(base_twist_accel));
  // Configuration of link frame j-1 relative to link frame j for arbitrary
  // joint angle
  vector<Pose3> jTis = jTi_list(q);

  int j = 0;
  for (int i = 0; i < N; ++i) {
    j = i + 1;
    auto jRw = Ts[i].rotation().inverse();
    if (gravity) {
      gfg.push_back(links_[i].inverseFactors(
          j, jTis[i], joint_velocities[i], twists_vec[i],
          joint_accelerations[i], jTis[j], jRw * (*gravity)));
    } else {
      gfg.push_back(links_[i].inverseFactors(
          j, jTis[i], joint_velocities[i], twists_vec[i],
          joint_accelerations[i], jTis[j], gravity));
    }
  }
  // Add factor to enforce external wrench at tool
  gfg.add(Link::ToolWrenchFactor(N, external_wrench));

  return gfg;
}

template <typename T>
Vector SerialLink<T>::extractJointAcceleraions(
    const VectorValues &result) const {
  int N = numLinks();
  Vector joint_accels(N);
  for (int j = 1; j <= N; ++j) {
    joint_accels[j - 1] = result.at(a(j))(0);
  }
  return joint_accels;
}

template <typename T>
Vector SerialLink<T>::extractTorques(const VectorValues &result) const {
  int N = numLinks();
  Vector torques(N);
  for (int j = 1; j <= N; ++j) {
    torques[j - 1] = result.at(t(j))(0);
  }
  return torques;
}

template <typename T>
VectorValues SerialLink<T>::factorGraphOptimization(
    const GaussianFactorGraph &dynamics_factor_graph) const {
  return dynamics_factor_graph.optimize();
}

template <typename T>
Vector SerialLink<T>::forwardDynamics(
    const Vector &q, const Vector &joint_velocities, const Vector &torques,
    const Vector6 &base_twist_accel, const Vector6 &external_wrench,
    boost::optional<Vector3 &> gravity) const {
  GaussianFactorGraph factor_graph = forwardDynamicsFactorGraph(
      q, joint_velocities, torques, base_twist_accel, external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  return extractJointAcceleraions(result);
}

template <typename T>
Vector SerialLink<T>::inverseDynamics(
    const Vector &q, const Vector &joint_velocities,
    const Vector &joint_accelerations, const Vector6 &base_twist_accel,
    const Vector6 &external_wrench, boost::optional<Vector3 &> gravity) const {
  GaussianFactorGraph factor_graph =
      inverseDynamicsFactorGraph(q, joint_velocities, joint_accelerations,
                                 base_twist_accel, external_wrench, gravity);
  VectorValues result = factor_graph.optimize();
  return extractTorques(result);
}

template <typename T>
JointLimitVectorFactor SerialLink<T>::jointLimitVectorFactor() const {
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(numLinks(), 1.0);
  return JointLimitVectorFactor(J(0), cost_model, jointLowerLimits(),
                                jointUpperLimits(), jointLimitThresholds());
}

template <typename T>
PoseGoalFactor SerialLink<T>::poseGoalFactor(const Pose3 &goal_pose) const {
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(6, 1.0);
  return PoseGoalFactor(
      J(0), cost_model, goal_pose,
      boost::bind(&SerialLink<T>::forwardKinematics, *this, _1, _2));
}

template <typename T>
NonlinearFactorGraph SerialLink<T>::inverseKinematicsFactorGraph(
    const Pose3 &goal_pose) const {
  NonlinearFactorGraph graph;
  graph.add(poseGoalFactor(goal_pose));
  graph.add(jointLimitVectorFactor());
  return graph;
}

template <typename T>
Vector SerialLink<T>::extractJointCooridinates(const Values &results) const {
  return results.at<Vector>(J(0));
}

template <typename T>
Values SerialLink<T>::factorGraphOptimization(const NonlinearFactorGraph &graph,
                                              const Values &init_values) const {
  LevenbergMarquardtOptimizer optimizer(graph, init_values);
  optimizer.optimize();
  return optimizer.values();
}

template <typename T>
Vector SerialLink<T>::inverseKinematics(const Pose3 &goal_pose,
                                        const Vector &init_q) const {
  NonlinearFactorGraph graph = inverseKinematicsFactorGraph(goal_pose);
  Values init_values;
  init_values.insert(J(0), init_q);
  Values results = factorGraphOptimization(graph, init_values);
  return extractJointCooridinates(results);
}

template class SerialLink<DH_Link>;
template class SerialLink<URDF_Link>;
}  // namespace manipulator
