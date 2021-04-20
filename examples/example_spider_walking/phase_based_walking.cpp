#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/Phase.h>

#include <vector>

using namespace std;


void blahBlah() const {
  // Get phase information
  vector<CPs> phase_cps = this->phaseCPs();
  vector<int> phase_durations = this->phaseDurations();
  vector<Robot> robots = this->phaseRobotModels();

  // Define noise to be added to initial values, desired timestep duration,
  // vector of link name strings, robot model for each phase, and
  // phase transition initial values.
  double gaussian_noise = 1e-5;
  double dt_des = 1. / 240;
  vector<Values> transition_graph_init =
      this->getInitTransitionValues(gaussian_noise);

  // Get final time step.
  int t_f = this->getEndTimeStep(this->numPhases() - 1);  // Final timestep.

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

  // Graphs for transition between phases + their initial values.
  vector<gtsam::NonlinearFactorGraph> transition_graphs =
      this->getTransitionGraphs(graph_builder, gravity, mu);

  // Construct the multi-phase trajectory factor graph.
  // TODO: Pass Trajectory here
  std::cout << "Creating dynamics graph" << std::endl;
  auto graph = graph_builder.multiPhaseTrajectoryFG(
      robots, phase_durations, transition_graphs, collocation, gravity,
      boost::none, phase_cps, mu);

  // Build the objective factors.
  gtsam::NonlinearFactorGraph objective_factors;
  auto base_link = spider.link("body");

  std::map<string, gtdynamics::Link> link_map;
  for (auto &&link_name : links)
    link_map.insert(std::make_pair(link_name, spider.link(link_name)));

  // Previous contact point goal.
  std::map<string, Point3> prev_cp = this->initContactPointGoal();

  // Distance to move contact point per time step during swing.
  auto contact_offset = Point3(0, 0.02, 0);

  // Add contact point objectives to factor graph.
  for (int p = 0; p < this->numPhases(); p++) {
    // if(p <2) contact_offset /=2 ;
    // Phase start and end timesteps.
    int t_p_i = this->getStartTimeStep(p);
    int t_p_f = this->getEndTimeStep(p);

    // Obtain the contact links and swing links for this phase.
    vector<string> phase_contact_links = this->getPhaseContactLinks(p);
    vector<string> phase_swing_links = this->getPhaseSwingLinks(p);

    // Setting the contact point goals for one time
    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

      for (auto &&pcl : phase_contact_links)
        objective_factors.add(this->pointGoalFactor(
            pcl, t, Isotropic::Sigma(3, 1e-7),  // 1e-7
            Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.05)));

      double h =
          GROUND_HEIGHT + std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto &&psl : phase_swing_links)
        objective_factors.add(this->pointGoalFactor(
            psl, t, Isotropic::Sigma(3, 1e-7),
            Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));

      // Update the goal point for the swing links.
      for (auto &&psl : phase_swing_links)
        prev_cp[psl] = prev_cp[psl] + contact_offset;
    }
  }

  // Add base goal objectives to the factor graph.
  for (int t = 0; t <= t_f; t++) {
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        internal::PoseKey(base_link.getID(), t),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.0, 0.5)),  // 0.5
        Isotropic::Sigma(6, 5e-5)));                              // 6.2e-5
    // 5e-5
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        internal::TwistKey(base_link.getID(), t), Vector6::Zero(),
        Isotropic::Sigma(6, 5e-5)));
  }

  // Add link boundary conditions to FG.
  for (auto &&link : spider.links()) {
    // Initial link pose, twists.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        internal::PoseKey(link.getID(), 0), link.wTcom(), dynamics_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        internal::TwistKey(link.getID(), 0), Vector6::Zero(), dynamics_model_6));

    // Final link twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link.getID(), t_f), Vector6::Zero(), objectives_model_6));
    objective_factors.add(
        gtsam::PriorFactor<Vector6>(internal::TwistAccelKey(link.getID(), t_f),
                                    Vector6::Zero(), objectives_model_6));
  }

  // Add joint boundary conditions to FG.
  for (auto &&joint : spider.joints()) {
    // Add priors to joint angles
    for (int t = 0; t <= t_f; t++) {
      if (joint->name().find("hip2") == 0)
        objective_factors.add(gtsam::PriorFactor<double>(
            JointAngleKey(joint->getID(), t), 2.5, dynamics_model_1_2));
    }
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), 0), 0.0, dynamics_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAccelKey(joint->getID(), t_f), 0.0, objectives_model_1));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < this->numPhases(); phase++)
      objective_factors.add(gtsam::PriorFactor<double>(
          PhaseKey(phase), dt_des,
          gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto &&joint : spider.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->getID(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);

  // TODO: Pass Trajectory here
  // Initialize solution.
  gtsam::Values init_vals;
  init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
      robots, phase_durations, transition_graph_init, dt_des, gaussian_noise,
      phase_cps);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e0);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // Write results to traj file
  vector<string> jnames;
  for (auto &&joint : spider.joints()) jnames.push_back(joint->name());
  std::cout << jnames.size() << std::endl;
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;

  // Get current directory to save the generated traj.csv file
  char cwd[PATH_MAX];
  char *fgh = getcwd(cwd, PATH_MAX);
  string example_directory = strcat(cwd, "/..");

  traj_file.open(example_directory + "/forward_traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  for (int phase = 0; phase < this->numPhases(); phase++)
    this->writePhaseToFile(traj_file, results, phase);

  // Write the last 4 phases to disk n times
  for (int i = 0; i < 10; i++) {
    for (int phase = 4; phase < phase_durations.size(); phase++)
      this->writePhaseToFile(traj_file, results, phase);
  }
  traj_file.close();
  return 0;
}