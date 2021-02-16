// GTDynamics Wrapper Interface File

namespace gtdynamics {

/********************** factors **********************/

#include <gtdynamics/factors/PoseFactor.h>
class PoseFactor : gtsam::NonlinearFactor{
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base* cost_model,
             const gtdynamics::JointSharedPtr joint);

  void print(const std::string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/TwistFactor.h>
class TwistFactor {
  TwistFactor(gtsam::Key twistP_key, gtsam::Key twistC_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtdynamics::JointSharedPtr joint);

  void print(const std::string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

// #include <gtdynamics/factors/TwistAccelFactor.h>
// class TwistAccelFactor {
//   TwistAccelFactor(gtsam::Key twist_key_c, gtsam::Key twistAccel_key_p, gtsam::Key twistAccel_key_c,
//               gtsam::Key q_key, gtsam::Key qVel_key, gtsam::Key qAccel_key,
//               const gtsam::noiseModel::Base* cost_model,
//               const gtdynamics::JointSharedPtr joint);

//   void print(const std::string &s,
//              const gtsam::KeyFormatter &keyFormatter);
// };


// #include <gtdynamics/factors/TorqueFactor.h>
// class TorqueFactor {
//   TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
//               const gtsam::noiseModel::Base* cost_model,
//               const gtdynamics::JointSharedPtr joint);

//   void print(const std::string &s,
//              const gtsam::KeyFormatter &keyFormatter);
// };



/********************** robot **********************/

#include <gtdynamics/universal_robot/Robot.h>

class Robot {
  Robot();

  Robot(gtdynamics::LinkJointPair links_and_joints);

  std::vector<gtdynamics::LinkSharedPtr> links() const;

  std::vector<gtdynamics::JointSharedPtr> joints() const;

  void removeLink(gtdynamics::LinkSharedPtr link);

  void removeJoint(gtdynamics::JointSharedPtr joint);

  gtdynamics::LinkSharedPtr getLinkByName(std::string name) const;

  gtdynamics::JointSharedPtr getJointByName(std::string name) const;

//   /// For python wrapper
//   void removeJointByName(const std::string& name);

//   /// For python wrapper
//   void removeLinkByName(const std::string& name);

  int numLinks() const;

  int numJoints() const;

  void print() const;

  FKResults forwardKinematics(
      const gtdynamics::JointValues &joint_angles, const gtdynamics::JointValues &joint_vels,
      const boost::optional<std::string> prior_link_name ,
      const boost::optional<gtsam::Pose3> &prior_link_pose ,
      const boost::optional<gtsam::Vector6> &prior_link_twist) const;

  gtsam::NonlinearFactorGraph qFactors(size_t t,
                                       const gtdynamics::OptimizerSetting &opt) const;


  gtsam::NonlinearFactorGraph vFactors(size_t t,
                                       const gtdynamics::OptimizerSetting &opt) const;

  gtsam::NonlinearFactorGraph aFactors(size_t t,
                                       const gtdynamics::OptimizerSetting &opt) const;

  gtsam::NonlinearFactorGraph dynamicsFactors(
      size_t t, const gtdynamics::OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const;

  gtsam::NonlinearFactorGraph jointLimitFactors(
      size_t t, const gtdynamics::OptimizerSetting &opt) const;

  gtsam::GaussianFactorGraph linearFDPriors(size_t t,
                                            const gtdynamics::JointValues &torques,
                                            const gtdynamics::OptimizerSetting &opt) const;

  gtsam::GaussianFactorGraph linearAFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const gtdynamics::OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const;

  gtsam::GaussianFactorGraph linearDynamicsFactors(
      size_t t, const std::map<std::string, gtsam::Pose3> &poses,
      const std::map<std::string, gtsam::Vector6> &twists,
      const std::map<std::string, double> &joint_angles,
      const std::map<std::string, double> &joint_vels,
      const gtdynamics::OptimizerSetting &opt,
      const boost::optional<gtsam::Vector3> &planar_axis) const;
};

#include <gtdynamics/universal_robot/sdf.h>
gtdynamics::Robot CreateRobotFromFile(const std::string file_path, 
                                    std::string model_name);


/********************** link **********************/
#include <gtdynamics/universal_robot/Link.h>
// class Link  {
//     Link();

    // gtdynamics::LinkSharedPtr getSharedPtr();

    // int getID() const;

    // const gtsam::Pose3 &wTl() const;

//   /// transfrom from link com frame to link frame
//   const gtsam::Pose3 &lTcom() const;

//   /// transform from link com frame to world frame
//   inline const gtsam::Pose3 wTcom() const;

//   /// the fixed pose of the link
//   const gtsam::Pose3 &getFixedPose() const;

//   /// whether the link is fixed
//   bool isFixed() const;

//   /// fix the link to fixed_pose. If fixed_pose is not specified, use wTcom.
//   void fix(const boost::optional<gtsam::Pose3 &> fixed_pose);

//   /// unfix the link
//   void unfix();

//   /// return all joints of the link
//   const std::vector<JointSharedPtr> &getJoints() const;

//   /// Return link name.
//   std::string name() const;

//   /// Return link mass.
//   double mass() const;

//   /// Return center of mass (gtsam::Pose3)
//   const gtsam::Pose3 &centerOfMass();

//   /// Return inertia.
//   const gtsam::Matrix3 &inertia();
// };

/********************** joint **********************/
// #include <gtdynamics/universal_robot/Joint.h>
// class Joint {

// };

/********************** dynamics graph **********************/
#include <gtdynamics/dynamics/OptimizerSetting.h>
class OptimizerSetting {
  OptimizerSetting();

};


#include<gtdynamics/dynamics/DynamicsGraph.h>
class DynamicsGraph {
  DynamicsGraph();

  DynamicsGraph(const gtdynamics::OptimizerSetting &opt);

  // enum CollocationScheme { Euler, RungeKutta, Trapezoidal, HermiteSimpson };

  static gtsam::GaussianFactorGraph linearDynamicsGraph(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_angles,
      const gtdynamics::JointValues &joint_vels, const gtdynamics::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis);

  static gtsam::GaussianFactorGraph linearFDPriors(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &torque_values);

  static gtsam::GaussianFactorGraph linearIDPriors(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_accels);

  static gtsam::Values linearSolveFD(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_angles,
      const gtdynamics::JointValues &joint_vels, const gtdynamics::JointValues &torques,
      const gtdynamics::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis);

  static gtsam::Values linearSolveID(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_angles,
      const gtdynamics::JointValues &joint_vels, const gtdynamics::JointValues &torques,
      const gtdynamics::FKResults &fk_results,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis);

  gtsam::NonlinearFactorGraph qFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return v-level nonlinear factor graph (twist related factors) */
  gtsam::NonlinearFactorGraph vFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return a-level nonlinear factor graph (acceleration related factors) */
  gtsam::NonlinearFactorGraph aFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return dynamics-level nonlinear factor graph (wrench related factors) */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const gtdynamics::Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &torques) const;

  gtsam::NonlinearFactorGraph inverseDynamicsPriors(
      const gtdynamics::Robot &robot, const int t, const gtsam::Vector &joint_angles,
      const gtsam::Vector &joint_vels, const gtsam::Vector &joint_accels) const;

  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_angles,
      const gtdynamics::JointValues &joint_vels,
      const gtdynamics::JointValues &torques) const;

  gtsam::NonlinearFactorGraph trajectoryFDPriors(
      const gtdynamics::Robot &robot, const int num_steps,
      const gtsam::Vector &joint_angles, const gtsam::Vector &joint_vels,
      const std::vector<gtsam::Vector> &torques_seq) const;

  gtsam::NonlinearFactorGraph trajectoryFG(
      const gtdynamics::Robot &robot, const int num_steps, const double dt,
      const gtdynamics::DynamicsGraph::CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const std::vector<gtdynamics::Robot> &robots, const std::vector<int> &phase_steps,
      const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
      const gtdynamics::DynamicsGraph::CollocationScheme collocation,
      const boost::optional<gtsam::Vector3> &gravity,
      const boost::optional<gtsam::Vector3> &planar_axis) const;

  gtsam::NonlinearFactorGraph collocationFactors(
      const gtdynamics::Robot &robot, const int t, const double dt,
      const gtdynamics::DynamicsGraph::CollocationScheme collocation) const;

  gtsam::NonlinearFactorGraph multiPhaseCollocationFactors(
      const gtdynamics::Robot &robot, const int t, const int phase,
      const gtdynamics::DynamicsGraph::CollocationScheme collocation) const;

  gtsam::NonlinearFactorGraph jointLimitFactors(const gtdynamics::Robot &robot,
                                                const int t) const;

  gtsam::NonlinearFactorGraph targetAngleFactors(
      const gtdynamics::Robot &robot, const int t, const std::string &joint_name,
      const double target_angle) const;

  gtsam::NonlinearFactorGraph targetPoseFactors(
      const gtdynamics::Robot &robot, const int t, const std::string &link_name,
      const gtsam::Pose3 &target_pose) const;

  static gtsam::Vector jointAccels(const gtdynamics::Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint velocities. */
  static gtsam::Vector jointVels(const gtdynamics::Robot &robot,
                                 const gtsam::Values &result, const int t);

  /* return joint angles. */
  static gtsam::Vector jointAngles(const gtdynamics::Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint torques. */
  static gtsam::Vector jointTorques(const gtdynamics::Robot &robot,
                                    const gtsam::Values &result, const int t);

  static gtdynamics::JointValues jointAccelsMap(const gtdynamics::Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint velocities as std::map<name, velocity>. */
  static gtdynamics::JointValues jointVelsMap(const gtdynamics::Robot &robot,
                                         const gtsam::Values &result,
                                         const int t);

  /* return joint angles as std::map<name, angle>. */
  static gtdynamics::JointValues jointAnglesMap(const gtdynamics::Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint torques as std::map<name, torque>. */
  static gtdynamics::JointValues jointTorquesMap(const gtdynamics::Robot &robot,
                                            const gtsam::Values &result,
                                            const int t);

  /* print the factors of the factor graph */
  static void printGraph(const gtsam::NonlinearFactorGraph &graph);

  /* print the values */
  static void printValues(const gtsam::Values &values);

  static void saveGraph(const std::string &file_path,
                        const gtsam::NonlinearFactorGraph &graph,
                        const gtsam::Values &values, const gtdynamics::Robot &robot,
                        const int t, bool radial);

  static void saveGraphMultiSteps(const std::string &file_path,
                                  const gtsam::NonlinearFactorGraph &graph,
                                  const gtsam::Values &values,
                                  const gtdynamics::Robot &robot, const int num_steps,
                                  bool radial);

  static void saveGraphTraj(const std::string &file_path,
                            const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &values, const int num_steps);

  /* return the optimizer setting. */
  const gtdynamics::OptimizerSetting &opt() const;
};


/********************** symbols **********************/

#include <gtdynamics/utils/DynamicsSymbol.h>
class DynamicsSymbol {
  DynamicsSymbol();

  DynamicsSymbol(const gtdynamics::DynamicsSymbol& key);

  static DynamicsSymbol LinkJointSymbol(const std::string& s,
                                        unsigned char link_idx,
                                        unsigned char joint_idx,
                                        std::uint64_t t);

  static DynamicsSymbol JointSymbol(const std::string& s,
                                    unsigned char joint_idx, std::uint64_t t);

  static DynamicsSymbol LinkSymbol(const std::string& s, unsigned char link_idx,
                                   std::uint64_t t);

  static DynamicsSymbol SimpleSymbol(const std::string& s, std::uint64_t t);

  DynamicsSymbol(const gtsam::Key& key);

  // operator gtsam::Key() const;

  std::string label() const;

  unsigned char linkIdx() const;

  unsigned char jointIdx() const;

  size_t time() const;

  void print(const std::string& s);

  bool equals(const gtdynamics::DynamicsSymbol& expected, double tol);

  gtsam::Key key() const;

  // operator std::string() const;
};

//TODO Create binding for STL containers

}

