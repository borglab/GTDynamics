// GTDynamics Wrapper Interface File

virtual class gtsam::NonlinearFactor;
virtual class gtsam::NoiseModelFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::Values;

namespace gtdynamics {

#include <gtdynamics/config.h>
const string URDF_PATH = kUrdfPath;
const string SDF_PATH = kSdfPath;

// Global variable for key formatting
const gtsam::KeyFormatter GTDKeyFormatter;

/********************** factors **********************/
#include <gtdynamics/factors/PoseFactor.h>
class PoseFactor : gtsam::NonlinearFactor {
  PoseFactor(gtsam::Key wTp_key, gtsam::Key wTc_key, gtsam::Key q_key,
             const gtsam::noiseModel::Base* cost_model,
             const gtdynamics::Joint* joint);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/ForwardKinematicsFactor.h>
class ForwardKinematicsFactor : gtsam::NoiseModelFactor {
  ForwardKinematicsFactor(gtsam::Key bTl1_key, gtsam::Key bTl2_key,
                          const gtdynamics::Robot &robot,
                          const string &start_link_name,
                          const string &end_link_name,
                          const gtsam::Values &joint_angles,
                          const gtsam::noiseModel::Base* model,
                          size_t t);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
  const gtsam::Pose3 measured() const;
};

#include <gtdynamics/factors/TwistFactor.h>
class TwistFactor : gtsam::NonlinearFactor {
  TwistFactor(gtsam::Key twistP_key, gtsam::Key twistC_key, gtsam::Key q_key,
              gtsam::Key qVel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtdynamics::Joint* joint);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/TwistAccelFactor.h>
class TwistAccelFactor : gtsam::NonlinearFactor {
  TwistAccelFactor(gtsam::Key twist_key_c, gtsam::Key twistAccel_key_p, gtsam::Key twistAccel_key_c,
              gtsam::Key q_key, gtsam::Key qVel_key, gtsam::Key qAccel_key,
              const gtsam::noiseModel::Base* cost_model,
              const gtdynamics::JointTyped* joint);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/TorqueFactor.h>
class TorqueFactor : gtsam::NonlinearFactor {
  TorqueFactor(gtsam::Key wrench_key, gtsam::Key torque_key,
               const gtsam::noiseModel::Base *cost_model,
               const gtdynamics::JointTyped *joint);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/MinTorqueFactor.h>
class MinTorqueFactor : gtsam::NonlinearFactor {
  MinTorqueFactor(gtsam::Key torque_key,
               const gtsam::noiseModel::Base *cost_model);

  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/WrenchFactor.h>
class WrenchFactor : gtsam::NonlinearFactor {
  WrenchFactor(gtsam::Key twist_key, gtsam::Key twistAccel_key,
                const std::vector<gtdynamics::DynamicsSymbol> wrench_keys, 
                gtsam::Key pose_key,
                const gtsam::noiseModel::Base *cost_model, const Matrix inertia,
                const boost::optional<gtsam::Vector3> &gravity);
  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
class WrenchEquivalenceFactor : gtsam::NonlinearFactor{
  WrenchEquivalenceFactor(gtsam::Key wrench_key_1, gtsam::Key wrench_key_2,
                          gtsam::Key q_key,
                          const gtsam::noiseModel::Base *cost_model,
                          gtdynamics::JointTyped *joint);
  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/WrenchPlanarFactor.h>
class WrenchPlanarFactor : gtsam::NonlinearFactor {
  WrenchPlanarFactor(gtsam::Key wrench_key,
                     const gtsam::noiseModel::Base *cost_model,
                     Vector planar_axis);
  void print(const string &s="",
             const gtsam::KeyFormatter &keyFormatter=gtdynamics::GTDKeyFormatter);
};

#include <gtdynamics/factors/CollocationFactors.h>
class EulerPoseColloFactor : gtsam::NonlinearFactor {
  EulerPoseColloFactor(gtsam::Key pose_t0_key, gtsam::Key pose_t1_key,
                       gtsam::Key twist_key, gtsam::Key dt_key,
                       const gtsam::noiseModel::Base *cost_model);
};

class TrapezoidalPoseColloFactor : gtsam::NonlinearFactor {
  TrapezoidalPoseColloFactor(
      gtsam::Key pose_t0_key, gtsam::Key pose_t1_key, gtsam::Key twist_t0_key,
      gtsam::Key twist_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base *cost_model);
};

class EulerTwistColloFactor : gtsam::NonlinearFactor {
  EulerTwistColloFactor(gtsam::Key twist_t0_key, gtsam::Key twist_t1_key,
                        gtsam::Key accel_key, gtsam::Key dt_key,
                        const gtsam::noiseModel::Base *cost_model);
};

class TrapezoidalTwistColloFactor : gtsam::NonlinearFactor {
  TrapezoidalTwistColloFactor(
      gtsam::Key twist_t0_key, gtsam::Key twist_t1_key, gtsam::Key accel_t0_key,
      gtsam::Key accel_t1_key, gtsam::Key dt_key,
      const gtsam::noiseModel::Base *cost_model);
};

/********************** link **********************/
#include <gtdynamics/universal_robot/Link.h>
class Link  {
    Link();
    Link(int id, const string &name_, const double mass_,
         const Matrix &inertia_, const gtsam::Pose3 &wTl_,
         const gtsam::Pose3 &lTcom_);
    Link(int id, const string &name_, const double mass_,
         const Matrix &inertia_, const gtsam::Pose3 &wTl_,
         const gtsam::Pose3 &lTcom_, bool is_fixed);

    gtdynamics::Link* shared();
    int id() const;
    void addJoint(gtdynamics::Joint* joint_ptr);
    const gtsam::Pose3 &wTl() const;
    const gtsam::Pose3 &lTcom() const;
    const gtsam::Pose3 wTcom() const;
    const gtsam::Pose3 &getFixedPose() const;
    bool isFixed() const;
    void fix();
    void fix(gtsam::Pose3 & fixed_pose);
    void unfix();
    const std::vector<Joint*> &joints() const;
    size_t numJoints() const;
    string name() const;
    double mass() const;
    const gtsam::Pose3 &centerOfMass();
    const Matrix &inertia();
    gtsam::Matrix6 inertiaMatrix();
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
  unsigned char id() const;
  const gtsam::Pose3 &wTj() const;
  const gtsam::Pose3 &jTpcom() const;
  const Pose3 &jTccom() const;
  string name() const;
  gtdynamics::Link* otherLink(const gtdynamics::Link* link);
  std::vector<gtdynamics::Link*> links() const;
  gtdynamics::Link* parent() const;
  gtdynamics::Link* child() const;
};

virtual class JointTyped : gtdynamics::Joint {
};

virtual class ScrewJointBase : gtdynamics::JointTyped {};

virtual class RevoluteJoint : gtdynamics::ScrewJointBase {
  RevoluteJoint(int id, const string &name, const gtsam::Pose3 &wTj,
                const gtdynamics::Link* parent_link,
                const gtdynamics::Link* child_link,
                const gtdynamics::JointParams &parameters, const Vector &axis);
};

virtual class PrismaticJoint : gtdynamics::ScrewJointBase {
  PrismaticJoint(int id, const string &name, const gtsam::Pose3 &wTj,
                 const gtdynamics::Link* parent_link,
                 const gtdynamics::Link* child_link,
                 const gtdynamics::JointParams &parameters, const Vector &axis);
};

virtual class ScrewJoint : gtdynamics::ScrewJointBase {
  ScrewJoint(int id, const string &name, const gtsam::Pose3 &wTj,
             const gtdynamics::Link* parent_link,
             const gtdynamics::Link* child_link,
             const gtdynamics::JointParams &parameters, const Vector &axis,
             double thread_pitch);
};

/********************** robot **********************/

#include <gtdynamics/universal_robot/Robot.h>

class Robot {
  Robot();

  Robot(std::map<string, gtdynamics::Link*> links, std::map<string, gtdynamics::Joint*> joints);

  std::vector<gtdynamics::Link*> links() const;

  std::vector<gtdynamics::Joint*> joints() const;

  void removeLink(gtdynamics::Link* link);

  void removeJoint(gtdynamics::Joint* joint);

  gtdynamics::Link* link(string name) const;

  gtdynamics::Joint* joint(string name) const;

  gtdynamics::Robot fixLink(const string& name);

  int numLinks() const;

  int numJoints() const;

  void print() const;

  gtsam::Values forwardKinematics(
      const gtsam::Values &known_values) const;

  gtsam::Values forwardKinematics(
      const gtsam::Values &known_values, size_t t) const;

  gtsam::Values forwardKinematics(
      const gtsam::Values &known_values, size_t t,
      const boost::optional<std::string> &prior_link_name) const;
};

#include <gtdynamics/universal_robot/sdf.h>
// This version is only for URDF files.
gtdynamics::Robot CreateRobotFromFile(const string& urdf_file_path);
gtdynamics::Robot CreateRobotFromFile(const string& file_path, 
                                    const string& model_name);


/********************** dynamics graph **********************/
#include <gtdynamics/dynamics/OptimizerSetting.h>
class OptimizerSetting {
  OptimizerSetting(double sigma_dynamics, double sigma_linear = 0.001,
                   double sigma_contact = 0.001, double sigma_joint = 0.001,
                   double sigma_collocation = 0.001, double sigma_time = 0.001);
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
enum CollocationScheme { Euler, RungeKutta, Trapezoidal, HermiteSimpson };

class ContactPoint {
  ContactPoint();
  ContactPoint(const gtsam::Point3& point, int id);
  void print(const string &s = "");
};

// ContactPoints defined in specializations.h

class DynamicsGraph {
  DynamicsGraph();
  DynamicsGraph(const boost::optional<gtsam::Vector3> &gravity,
                const boost::optional<gtsam::Vector3> &planar_axis);
  DynamicsGraph(const gtdynamics::OptimizerSetting &opt);
  DynamicsGraph(const gtdynamics::OptimizerSetting &opt,
                const boost::optional<gtsam::Vector3> &gravity,
                const boost::optional<gtsam::Vector3> &planar_axis);

  gtsam::GaussianFactorGraph linearDynamicsGraph(
      const gtdynamics::Robot &robot, const int t,
      const gtsam::Values &known_values);

  gtsam::GaussianFactorGraph linearFDPriors(
      const gtdynamics::Robot &robot, const int t,
      const gtsam::Values &known_values);

  gtsam::GaussianFactorGraph linearIDPriors(
      const gtdynamics::Robot &robot, const int t,
      const gtsam::Values &known_values);

  gtsam::Values linearSolveFD(const gtdynamics::Robot &robot, const int t,
                              const gtsam::Values &known_values);

  gtsam::Values linearSolveID(const gtdynamics::Robot &robot, const int t,
                              const gtsam::Values &known_values);

  gtsam::NonlinearFactorGraph qFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return v-level nonlinear factor graph (twist related factors) */
  gtsam::NonlinearFactorGraph vFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return a-level nonlinear factor graph (acceleration related factors) */
  gtsam::NonlinearFactorGraph aFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtdynamics::ContactPoints> &contact_points) const;

  /* return dynamics-level nonlinear factor graph (wrench related factors) */
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph dynamicsFactorGraph(
      const gtdynamics::Robot &robot, const int t,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph inverseDynamicsPriors(
      const gtdynamics::Robot &robot, const int t,
      const gtsam::Values &known_values) const;

  gtsam::NonlinearFactorGraph forwardDynamicsPriors(
      const gtdynamics::Robot &robot, const int t,
      const gtsam::Values &known_values) const;

  gtsam::NonlinearFactorGraph trajectoryFDPriors(
      const gtdynamics::Robot &robot, const int num_steps,
      const gtsam::Values &known_values) const;

  gtsam::NonlinearFactorGraph trajectoryFG(
      const gtdynamics::Robot &robot, const int num_steps, const double dt) const;

  gtsam::NonlinearFactorGraph trajectoryFG(
      const gtdynamics::Robot &robot, const int num_steps, const double dt,
      const gtdynamics::CollocationScheme collocation,
      const boost::optional<gtdynamics::ContactPoints> &contact_points,
      const boost::optional<double> &mu) const;

  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const gtdynamics::Robot &robot,
      const std::vector<int> &phase_steps,
      const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs) const;

  gtsam::NonlinearFactorGraph multiPhaseTrajectoryFG(
      const gtdynamics::Robot &robot,
      const std::vector<int> &phase_steps,
      const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
      const gtdynamics::CollocationScheme collocation) const;

  static void addCollocationFactorDouble(
      gtsam::NonlinearFactorGraph @graph, const gtsam::Key x0_key,
      const gtsam::Key x1_key, const gtsam::Key v0_key, const gtsam::Key v1_key,
      const double dt, gtsam::noiseModel::Base* cost_model,
      const gtdynamics::CollocationScheme collocation);

  static void addMultiPhaseCollocationFactorDouble(
      gtsam::NonlinearFactorGraph @graph, const gtsam::Key x0_key,
      const gtsam::Key x1_key, const gtsam::Key v0_key, const gtsam::Key v1_key,
      const gtsam::Key phase_key,
      gtsam::noiseModel::Base* cost_model,
      const gtdynamics::CollocationScheme collocation);

  gtsam::NonlinearFactorGraph jointCollocationFactors(
      const int j, const int t, const double dt,
      const gtdynamics::CollocationScheme collocation) const;

  gtsam::NonlinearFactorGraph jointMultiPhaseCollocationFactors(
      const int j, const int t, const int phase,
      const gtdynamics::CollocationScheme collocation) const;

  gtsam::NonlinearFactorGraph collocationFactors(
      const gtdynamics::Robot &robot, const int t, const double dt,
      const gtdynamics::CollocationScheme collocation) const;

  gtsam::NonlinearFactorGraph multiPhaseCollocationFactors(
      const gtdynamics::Robot &robot, const int t, const int phase,
      const gtdynamics::CollocationScheme collocation) const;

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

  static gtdynamics::JointValueMap jointAccelsMap(const gtdynamics::Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint velocities as std::map<name, velocity>. */
  static gtdynamics::JointValueMap jointVelsMap(const gtdynamics::Robot &robot,
                                         const gtsam::Values &result,
                                         const int t);

  /* return joint angles as std::map<name, angle>. */
  static gtdynamics::JointValueMap jointAnglesMap(const gtdynamics::Robot &robot,
                                           const gtsam::Values &result,
                                           const int t);

  /* return joint torques as std::map<name, torque>. */
  static gtdynamics::JointValueMap jointTorquesMap(const gtdynamics::Robot &robot,
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

/********************** Objective Factors **********************/
#include <gtdynamics/factors/ObjectiveFactors.h>
class add_link_objectives {
  add_link_objectives(gtsam::NonlinearFactorGraph @graph, int i, int k = 0);

  add_link_objectives &pose(
      gtsam::Pose3 pose, const gtsam::SharedNoiseModel &pose_model = nullptr);
  add_link_objectives &twist(
      gtsam::Vector6 twist,
      const gtsam::SharedNoiseModel &twist_model = nullptr);
  add_link_objectives &twistAccel(
      gtsam::Vector6 twistAccel,
      const gtsam::SharedNoiseModel &twistAccel_model = nullptr);
};

class add_joint_objectives {
  add_joint_objectives(gtsam::NonlinearFactorGraph @graph, int j, int k = 0);

  add_joint_objectives &angle(
      double angle, const gtsam::SharedNoiseModel &angle_model = nullptr);
  add_joint_objectives &velocity(
      double velocity, const gtsam::SharedNoiseModel &velocity_model = nullptr);
  add_joint_objectives &acceleration(
      double acceleration,
      const gtsam::SharedNoiseModel &acceleration_model = nullptr);
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
  DynamicsSymbol(const gtsam::Key& key);
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

  string label() const;
  unsigned char linkIdx() const;
  unsigned char jointIdx() const;
  size_t time() const;
  gtsam::Key key() const;

  void print(const string& s = "");
  bool equals(const gtdynamics::DynamicsSymbol& expected, double tol);
};

namespace internal {
  gtdynamics::DynamicsSymbol JointAngleKey(int j, int t);
  gtdynamics::DynamicsSymbol JointVelKey(int j, int t);
  gtdynamics::DynamicsSymbol JointAccelKey(int j, int t);
  gtdynamics::DynamicsSymbol TorqueKey(int j, int t);
  gtdynamics::DynamicsSymbol TwistKey(int i, int t);
  gtdynamics::DynamicsSymbol TwistAccelKey(int i, int t);
  gtdynamics::DynamicsSymbol WrenchKey(int i, int j, int t);
  gtdynamics::DynamicsSymbol PoseKey(int i, int t);
}

gtdynamics::DynamicsSymbol ContactWrenchKey(int i, int k, int t);
gtdynamics::DynamicsSymbol PhaseKey(int k);
gtdynamics::DynamicsSymbol TimeKey(int t);

///////////////////// Key Methods /////////////////////
template<T = {double}>
void InsertJointAngle(gtsam::Values@ values, int j, int t, T value);

template<T = {double}>
void InsertJointAngle(gtsam::Values @values, int j, T value);

gtsam::Vector JointAngle(const gtsam::VectorValues &values, int j, int t);

template<T = {double}>
T JointAngle(const gtsam::Values &values, int j, int t);

template<T = {double}>
void InsertJointVel(gtsam::Values @values, int j, int t, T value);

template<T = {double}>
void InsertJointVel(gtsam::Values @values, int j, T value);

gtsam::Vector JointVel(const gtsam::VectorValues &values, int j, int t);

template<T = {double}>
T JointVel(const gtsam::Values &values, int j, int t);

template<T = {double}>
void InsertJointAccel(gtsam::Values @values, int j, int t, T value);

template<T = {double}>
void InsertJointAccel(gtsam::Values @values, int j, T value);

gtsam::Vector JointAccel(const gtsam::VectorValues &values, int j, int t);

template<T = {double}>
T JointAccel(const gtsam::Values &values, int j, int t);

template<T = {double}>
void InsertTorque(gtsam::Values @values, int j, int t, T value);

template<T = {double}>
void InsertTorque(gtsam::Values @values, int j, T value);

gtsam::Vector Torque(const gtsam::VectorValues &values, int j, int t);

template<T = {double}>
T Torque(const gtsam::Values &values, int j, int t);

void InsertPose(gtsam::Values @values, int i, int t, gtsam::Pose3 value);

void InsertPose(gtsam::Values @values, int i, gtsam::Pose3 value);

gtsam::Pose3 Pose(const gtsam::Values &values, int i, int t);

void InsertTwist(gtsam::Values @values, int j, int t, gtsam::Vector6 value);

void InsertTwist(gtsam::Values @values, int j, gtsam::Vector6 value);

gtsam::Vector Twist(const gtsam::VectorValues &values, int j, int t);

gtsam::Vector6 Twist(const gtsam::Values &values, int j, int t);

void InsertTwistAccel(gtsam::Values @values, int j, int t, gtsam::Vector6 value);

void InsertTwistAccel(gtsam::Values @values, int j, gtsam::Vector6 value);

gtsam::Vector TwistAccel(const gtsam::VectorValues &values, int j, int t);

gtsam::Vector6 TwistAccel(const gtsam::Values &values, int j, int t);

void InsertWrench(gtsam::Values @values, int i, int j, int t, gtsam::Vector6 value);

void InsertWrench(gtsam::Values @values, int i, int j, gtsam::Vector6 value);

gtsam::Vector Wrench(const gtsam::VectorValues &values, int i, int j, int t);

gtsam::Vector6 Wrench(const gtsam::Values &values, int i, int j, int t);

/********************** Simulator **********************/
#include <gtdynamics/dynamics/Simulator.h>

class Simulator {
  Simulator(const gtdynamics::Robot &robot, const gtsam::Values &initial_values);
  Simulator(const gtdynamics::Robot &robot, const gtsam::Values &initial_values,
            const boost::optional<gtsam::Vector3> &gravity);
  Simulator(const gtdynamics::Robot &robot, const gtsam::Values &initial_values,
            const gtsam::Vector3 &gravity,
            const gtsam::Vector3 &planar_axis);

  void reset(const double t);
  void forwardDynamics(const gtsam::Values &torques);
  void integration(const double dt);
  void step(const gtsam::Values &torques, const double dt);
  gtsam::Values simulate(const std::vector<gtsam::Values> &torques_seq,
                         const double dt);
  const gtsam::Values &getValues() const;
};

/********************** Trajectory et al  **********************/
#include <gtdynamics/utils/Phase.h>
class Phase {
  Phase(const int &num_time_steps);
  Phase(const size_t &num_time_steps, const std::vector<string> &link_names,
        const gtsam::Point3 &point);
  void addContactPoint(const string &link, const gtsam::Point3 &point);
  void addContactPoints(const std::vector<string> &links,
                        const gtsam::Point3 &point);
  const gtdynamics::ContactPoints &contactPoints() const;
  const gtdynamics::ContactPoint &contactPoint(const string &link) const;
  int numTimeSteps() const;
  void print(const string &s = "");
};

#include <gtdynamics/utils/WalkCycle.h>
class WalkCycle {
  WalkCycle();
  WalkCycle(const std::vector<gtdynamics::Phase>& phases);
  void addPhase(const gtdynamics::Phase& phase);
  const gtdynamics::Phase& phase(size_t p);
  const std::vector<gtdynamics::Phase>& phases() const;
  size_t numPhases() const;
  const ContactPoints& contactPoints() const;
  void print(const string& s = "") const;
  std::map<string, gtsam::Point3>
  initContactPointGoal(const gtdynamics::Robot &robot) const;
  std::vector<string> swingLinks(size_t p) const;
  gtsam::NonlinearFactorGraph
  contactPointObjectives(const gtdynamics::Robot &robot,
                         const gtsam::SharedNoiseModel &cost_model,
                         const gtsam::Point3 &step, size_t k_start,
                         std::map<string, gtsam::Point3> @cp_goals) const;
};

#include <gtdynamics/utils/Trajectory.h>
class Trajectory {
  Trajectory();
  Trajectory(const gtdynamics::Robot &robot,
             const gtdynamics::WalkCycle &walk_cycle, size_t repeat);
  std::vector<gtdynamics::ContactPoints> phaseContactPoints() const;
  std::vector<gtdynamics::ContactPoints> transitionContactPoints() const;
  std::vector<int> phaseDurations() const;
  size_t numPhases() const;
  std::vector<gtsam::NonlinearFactorGraph>
  getTransitionGraphs(gtdynamics::DynamicsGraph &graph_builder,
                      double mu) const;
  gtsam::NonlinearFactorGraph
  multiPhaseFactorGraph(gtdynamics::DynamicsGraph &graph_builder,
                        const gtdynamics::CollocationScheme collocation,
                        double mu) const;
  std::vector<gtsam::Values>
  transitionPhaseInitialValues(double gaussian_noise) const;
  gtsam::Values multiPhaseInitialValues(double gaussian_noise, double dt) const;
  std::vector<int> finalTimeSteps() const;
  size_t phaseIndex(size_t p) const;
  const Phase &phase(size_t p) const;
  size_t getStartTimeStep(size_t p) const;
  size_t getEndTimeStep(size_t p) const;
  const ContactPoints &getPhaseContactLinks(size_t p) const;
  std::vector<std::string> getPhaseSwingLinks(size_t p) const;
  PointGoalFactor pointGoalFactor(const std::string &link_name,
                                  const gtdynamics::ContactPoint &cp, size_t k,
                                  const gtsam::SharedNoiseModel &cost_model,
                                  const gtsam::Point3 &goal_point) const;
  gtsam::NonlinearFactorGraph
  contactPointObjectives(const gtsam::SharedNoiseModel &cost_model,
                         const gtsam::Point3 &step) const;
  void addMinimumTorqueFactors(gtsam::NonlinearFactorGraph @graph,
                               const gtsam::SharedNoiseModel &cost_model) const;
  void addBoundaryConditions(
      gtsam::NonlinearFactorGraph @graph,
      const gtsam::SharedNoiseModel &pose_model,
      const gtsam::SharedNoiseModel &twist_model,
      const gtsam::SharedNoiseModel &twist_acceleration_model,
      const gtsam::SharedNoiseModel &joint_velocity_model,
      const gtsam::SharedNoiseModel &joint_acceleration_model) const;
  void addIntegrationTimeFactors(gtsam::NonlinearFactorGraph @graph,
                                 double desired_dt, double sigma = 0) const;
  void writeToFile(const string &name, const gtsam::Values &results) const;
};

/********************** Utilities  **********************/
#include <gtdynamics/utils/format.h>
string GtdFormat(const gtsam::Values &t, const string &s = "");
string GtdFormat(const gtsam::NonlinearFactorGraph &t, const string &s = "");

}  // namespace gtdynamics
