// GTDynamics Wrapper Interface File

namespace gtdynamics {

/********************** factors **********************/

virtual class gtsam::NonlinearFactor;

#include <gtdynamics/factors/PoseFactor.h>
class PoseFactor : gtsam::NonlinearFactor {
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base* cost_model,
             const gtdynamics::Joint* joint);

  void print(const string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/TwistFactor.h>
class TwistFactor {
  TwistFactor(gtsam::Key twistP_key, gtsam::Key twistC_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtdynamics::Joint* joint);

  void print(const string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/TwistAccelFactor.h>
class TwistAccelFactor {
  TwistAccelFactor(gtsam::Key twist_key_c, gtsam::Key twistAccel_key_p, gtsam::Key twistAccel_key_c,
              gtsam::Key q_key, gtsam::Key qVel_key, gtsam::Key qAccel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtdynamics::JointTyped* joint);

  void print(const string &s,
             const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/TorqueFactor.h>
class TorqueFactor {
  TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
               const gtsam::noiseModel::Base *cost_model,
               const gtdynamics::JointTyped *joint);

  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/WrenchFactors.h>
class WrenchFactor0 {
  WrenchFactor0(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key pose_key, const gtsam::noiseModel::Base *cost_model,
                const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

class WrenchFactor1 {
  WrenchFactor1(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key pose_key,
                const gtsam::noiseModel::Base *cost_model, const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

class WrenchFactor2 {
  WrenchFactor2(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key pose_key, const gtsam::noiseModel::Base *cost_model,
                const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

class WrenchFactor3 {
  WrenchFactor3(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key wrench_key_3, gtsam::Key pose_key,
                const gtsam::noiseModel::Base *cost_model, const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

class WrenchFactor4 {
  WrenchFactor4(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                gtsam::Key wrench_key_3, gtsam::Key wrench_key_4,
                gtsam::Key pose_key, const gtsam::noiseModel::Base *cost_model,
                const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
class WrenchEquivalenceFactor {
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const gtsam::noiseModel::Base *cost_model,
                          gtdynamics::JointTyped *joint);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/WrenchPlanarFactor.h>
class WrenchPlanarFactor {
  WrenchPlanarFactor(gtsam::Key wrench_key,
                     const gtsam::noiseModel::Base *cost_model,
                     Vector planar_axis);
  void print(const string &s, const gtsam::KeyFormatter &keyFormatter);
};

#include <gtdynamics/factors/CollocationFactors.h>
class EulerPoseColloFactor{
  EulerPoseColloFactor(gtsam::Key pose_t0_key, gtsam::Key pose_t1_key,
                       gtsam::Key twist_key, gtsam::Key dt_key,
                       const gtsam::noiseModel::Base *cost_model);
};

class TrapezoidalPoseColloFactor{
  TrapezoidalPoseColloFactor(
      gtsam::Key pose_t0_key, gtsam::Key pose_t1_key, gtsam::Key twist_t0_key,
      gtsam::Key twist_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base *cost_model);
};

class EulerTwistColloFactor{
  EulerTwistColloFactor(gtsam::Key twist_t0_key, gtsam::Key twist_t1_key,
                        gtsam::Key accel_key, gtsam::Key dt_key,
                        const gtsam::noiseModel::Base *cost_model);
};

class TrapezoidalTwistColloFactor{
  TrapezoidalTwistColloFactor(
      gtsam::Key twist_t0_key, gtsam::Key twist_t1_key, gtsam::Key accel_t0_key,
      gtsam::Key accel_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base *cost_model);
};

/********************** link **********************/
#include <gtdynamics/universal_robot/Link.h>
class LinkParams {
    LinkParams(const string& _name, const double _mass, 
                const Matrix _inertia, const gtsam::Pose3& _wTl,
                const gtsam::Pose3& _lTcom);
    string name;
    double mass;
    Matrix inertia;
    gtsam::Pose3 wTl;
    gtsam::Pose3 lTcom; 
};

class Link  {
    Link();
    Link(const gtdynamics::LinkParams &params);

    gtdynamics::Link* shared();
    void setID(unsigned char id);
    int id() const;
    void addJoint(gtdynamics::JointSharedPtr joint_ptr);
    const gtsam::Pose3 &wTl() const;
    const gtsam::Pose3 &lTcom() const;
    const gtsam::Pose3 wTcom() const;
    const gtsam::Pose3 &getFixedPose() const;
    bool isFixed() const;
    void fix();
    void fix(gtsam::Pose3 & fixed_pose);
    void unfix();
    const std::vector<Joint> &getJoints() const; // don't need Joint* to get this working
    string name() const;
    double mass() const;
    const gtsam::Pose3 &centerOfMass();
    const Matrix &inertia();
};

/********************** joint **********************/
#include <gtdynamics/universal_robot/Joint.h>
#include <gtdynamics/universal_robot/JointTyped.h>
#include <gtdynamics/universal_robot/ScrewJointBase.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/PrismaticJoint.h>
#include <gtdynamics/universal_robot/ScrewJoint.h>
class JointParams {
  JointParams();
  double velocity_limit;
  double velocity_limit_threshold;
  double acceleration_limit;
  double acceleration_limit_threshold;
  double torque_limit;
  double torque_limit_threshold;
  double damping_coefficient;
  double spring_coefficient;
};

virtual class Joint {
  void setID(unsigned char id);
  int id() const;
  const gtsam::Pose3 &wTj() const;
  const gtsam::Pose3 &jTpcom() const;
  const Pose3 &jTccom() const;
  string name() const;
  gtdynamics::LinkSharedPtr otherLink(const gtdynamics::LinkSharedPtr &link);
  std::vector<gtdynamics::LinkSharedPtr> links() const;
  gtdynamics::LinkSharedPtr parent() const;
  gtdynamics::LinkSharedPtr child() const;
};

virtual class JointTyped : gtdynamics::Joint {
};

virtual class ScrewJointBase : gtdynamics::JointTyped {};

virtual class RevoluteJoint : gtdynamics::ScrewJointBase {
  RevoluteJoint(const string &name, const gtsam::Pose3 &wTj,
                const gtdynamics::LinkSharedPtr &parent_link,
                const gtdynamics::LinkSharedPtr &child_link,
                const gtdynamics::JointParams &parameters, const Vector &axis);
};

virtual class PrismaticJoint : gtdynamics::ScrewJointBase {
  PrismaticJoint(const string &name, const gtsam::Pose3 &wTj,
                 const gtdynamics::LinkSharedPtr &parent_link,
                 const gtdynamics::LinkSharedPtr &child_link,
                 const gtdynamics::JointParams &parameters, const Vector &axis);
};

virtual class ScrewJoint : gtdynamics::ScrewJointBase {
  ScrewJoint(const string &name, const gtsam::Pose3 &wTj,
             const gtdynamics::LinkSharedPtr &parent_link,
             const gtdynamics::LinkSharedPtr &child_link,
             const gtdynamics::JointParams &parameters, const Vector &axis,
             double thread_pitch);
};

/********************** robot **********************/

#include <gtdynamics/universal_robot/Robot.h>

class Robot {
  Robot();

  Robot(std::map<string, gtdynamics::LinkSharedPtr> links, std::map<string, gtdynamics::JointSharedPtr> joints);

  std::vector<gtdynamics::LinkSharedPtr> links() const;

  std::vector<gtdynamics::JointSharedPtr> joints() const;

  void removeLink(gtdynamics::LinkSharedPtr link);

  void removeJoint(gtdynamics::JointSharedPtr joint);

  gtdynamics::Link* link(string name) const;

  gtdynamics::Joint* joint(string name) const;

  int numLinks() const;

  int numJoints() const;

  void print() const;

  FKResults forwardKinematics(
      const gtdynamics::JointValues &joint_angles,
      const boost::optional<gtdynamics::JointValues> &joint_velocities,
      const boost::optional<string> &prior_link_name,
      const gtsam::Pose3 &prior_link_pose,
      const Vector &prior_link_twist) const;
};

#include <gtdynamics/universal_robot/sdf.h>
gtdynamics::Robot CreateRobotFromFile(const string file_path, 
                                    string model_name);


/********************** dynamics graph **********************/
#include <gtdynamics/dynamics/OptimizerSetting.h>
class OptimizerSetting {
  OptimizerSetting();
  gtsam::noiseModel::SharedNoiseModel bv_cost_model;
  gtsam::noiseModel::SharedNoiseModel ba_cost_model;             // acceleration of fixed link
  gtsam::noiseModel::SharedNoiseModel p_cost_model;              // pose factor
  gtsam::noiseModel::SharedNoiseModel v_cost_model;              // twist factor
  gtsam::noiseModel::SharedNoiseModel a_cost_model;              // acceleration factor
  gtsam::noiseModel::SharedNoiseModel linear_a_cost_model;       // linear acceleration factor
  gtsam::noiseModel::SharedNoiseModel f_cost_model;              // wrench equivalence factor
  gtsam::noiseModel::SharedNoiseModel linear_f_cost_model;       // linear wrench equivalence factor
  gtsam::noiseModel::SharedNoiseModel fa_cost_model;             // wrench factor
  gtsam::noiseModel::SharedNoiseModel t_cost_model;              // torque factor
  gtsam::noiseModel::SharedNoiseModel linear_t_cost_model;       // linear torque factor
  gtsam::noiseModel::SharedNoiseModel cp_cost_model;             // contact pose
  gtsam::noiseModel::SharedNoiseModel cfriction_cost_model;      // contact friction cone
  gtsam::noiseModel::SharedNoiseModel cv_cost_model;             // contact twist
  gtsam::noiseModel::SharedNoiseModel ca_cost_model;             // contact acceleration
  gtsam::noiseModel::SharedNoiseModel cm_cost_model;             // contact moment
  gtsam::noiseModel::SharedNoiseModel planar_cost_model;         // planar factor
  gtsam::noiseModel::SharedNoiseModel linear_planar_cost_model;  // linear planar factor
  gtsam::noiseModel::SharedNoiseModel prior_q_cost_model;        // joint angle prior factor
  gtsam::noiseModel::SharedNoiseModel prior_qv_cost_model;       // joint velocity prior factor
  gtsam::noiseModel::SharedNoiseModel prior_qa_cost_model;       // joint acceleration prior factor
  gtsam::noiseModel::SharedNoiseModel prior_t_cost_model;        // joint torque prior factor
  gtsam::noiseModel::SharedNoiseModel q_col_cost_model;          // joint collocation factor
  gtsam::noiseModel::SharedNoiseModel v_col_cost_model;          // joint vel collocation factor
  gtsam::noiseModel::SharedNoiseModel pose_col_cost_model;       // pose collocation factor
  gtsam::noiseModel::SharedNoiseModel twist_col_cost_model;      // twist collocation factor
  gtsam::noiseModel::SharedNoiseModel time_cost_model;           // time prior
  gtsam::noiseModel::SharedNoiseModel jl_cost_model;             // joint limit factor
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
      const gtdynamics::Robot &robot, const int t, const Vector &joint_angles,
      const Vector &joint_vels, const Vector &torques) const;

  gtsam::NonlinearFactorGraph inverseDynamicsPriors(
      const gtdynamics::Robot &robot, const int t, const Vector &joint_angles,
      const Vector &joint_vels, const Vector &joint_accels) const;

  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const gtdynamics::Robot &robot, const int t, const gtdynamics::JointValues &joint_angles,
      const gtdynamics::JointValues &joint_vels,
      const gtdynamics::JointValues &torques) const;

  gtsam::NonlinearFactorGraph trajectoryFDPriors(
      const gtdynamics::Robot &robot, const int num_steps,
      const Vector &joint_angles, const Vector &joint_vels,
      const std::vector<Vector> &torques_seq) const;

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
      const gtdynamics::Robot &robot, const int t, const string &joint_name,
      const double target_angle) const;

  gtsam::NonlinearFactorGraph targetPoseFactors(
      const gtdynamics::Robot &robot, const int t, const string &link_name,
      const gtsam::Pose3 &target_pose) const;

  static Vector jointAccels(const gtdynamics::Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint velocities. */
  static Vector jointVels(const gtdynamics::Robot &robot,
                                 const gtsam::Values &result, const int t);

  /* return joint angles. */
  static Vector jointAngles(const gtdynamics::Robot &robot,
                                   const gtsam::Values &result, const int t);

  /* return joint torques. */
  static Vector jointTorques(const gtdynamics::Robot &robot,
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

  static void saveGraph(const string &file_path,
                        const gtsam::NonlinearFactorGraph &graph,
                        const gtsam::Values &values, const gtdynamics::Robot &robot,
                        const int t, bool radial);

  static void saveGraphMultiSteps(const string &file_path,
                                  const gtsam::NonlinearFactorGraph &graph,
                                  const gtsam::Values &values,
                                  const gtdynamics::Robot &robot, const int num_steps,
                                  bool radial);

  static void saveGraphTraj(const string &file_path,
                            const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &values, const int num_steps);

  /* return the optimizer setting. */
  const gtdynamics::OptimizerSetting &opt() const;
};

/********************** Value Initialization **********************/
#include <gtdynamics/utils/initialize_solution_utils.h>
gtsam::Values ZeroValues(
    const gtdynamics::Robot& robot, const int t, double gaussian_noise);

gtsam::Values ZeroValuesTrajectory(
    const gtdynamics::Robot& robot, const int num_steps, const int num_phases,
    double gaussian_noise,
    const boost::optional<gtdynamics::ContactPoints>& contact_points);

/********************** symbols **********************/

#include <gtdynamics/utils/DynamicsSymbol.h>
class DynamicsSymbol {
  DynamicsSymbol();

  DynamicsSymbol(const gtdynamics::DynamicsSymbol& key);

  static DynamicsSymbol LinkJointSymbol(const string& s,
                                        unsigned char link_idx,
                                        unsigned char joint_idx,
                                        std::uint64_t t);

  static DynamicsSymbol JointSymbol(const string& s,
                                    unsigned char joint_idx, std::uint64_t t);

  static DynamicsSymbol LinkSymbol(const string& s, unsigned char link_idx,
                                   std::uint64_t t);

  static DynamicsSymbol SimpleSymbol(const string& s, std::uint64_t t);

  DynamicsSymbol(const gtsam::Key& key);

  string label() const;

  unsigned char linkIdx() const;

  unsigned char jointIdx() const;

  size_t time() const;

  void print(const string& s);

  bool equals(const gtdynamics::DynamicsSymbol& expected, double tol);

  gtsam::Key key() const;
};

gtdynamics::DynamicsSymbol PoseKey(int i, int t);
gtdynamics::DynamicsSymbol TwistKey(int i, int t);
gtdynamics::DynamicsSymbol TwistAccelKey(int i, int t);
gtdynamics::DynamicsSymbol WrenchKey(int i, int j, int t);
gtdynamics::DynamicsSymbol JointAngleKey(int j, int t);
gtdynamics::DynamicsSymbol JointVelKey(int j, int t);
gtdynamics::DynamicsSymbol JointAccelKey(int j, int t);
gtdynamics::DynamicsSymbol TorqueKey(int j, int t);
gtdynamics::DynamicsSymbol ContactWrenchKey(int i, int k, int t);
gtdynamics::DynamicsSymbol PhaseKey(int k);
gtdynamics::DynamicsSymbol TimeKey(int t);
}

