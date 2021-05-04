/**
 * @file  Simulator.cpp
 * @brief dynamics factor graph
 * @Author: Yetong Zhang
 */



#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>

#include "gtdynamics/jumpingrobot/factors/JumpingRobot.h"
#include "gtdynamics/factors/CollocationFactors.h"
#include "gtdynamics/dynamics/Simulator.h"
#include "gtdynamics/universal_robot/sdf.h"

using gtsam::NonlinearFactorGraph, gtsam::Values, gtsam::Vector6, gtsam::Pose3, gtsam::Key, gtsam::Double_, 
      gtsam::PriorFactor, gtsam::BetweenFactor, gtsam::Rot3, gtsam::Point3;
using std::vector;

using gtdynamics::internal::PoseKey, gtdynamics::internal::TwistKey, gtdynamics::internal::TwistAccelKey,
      gtdynamics::internal::WrenchKey, gtdynamics::internal::TorqueKey,
      gtdynamics::internal::JointAngleKey, gtdynamics::internal::JointVelKey, gtdynamics::internal::JointAccelKey;

namespace gtdynamics
{
Robot loadRobot()
{
    Robot jumping_robot = CreateRobotFromFile(kSdfPath + std::string("/test/jumping_robot.sdf"));
    jumping_robot.link("l0")->fix();
    return jumping_robot;
}

Robot robotAir()
{
    Robot robot_air = loadRobot();
    robot_air.removeLink(robot_air.link("l0"));
    return robot_air;
}

Robot robotLeft()
{
    Robot robot_left = loadRobot();
    robot_left.removeJoint(robot_left.joint("j0"));
    return robot_left;
}

Robot robotRight()
{
    Robot robot_right = loadRobot();
    robot_right.removeJoint(robot_right.joint("j5"));
    return robot_right;
}

void JumpingRobot::initializeRobots()
{
    robot_ground_ = loadRobot(),
    robot_air_ = robotAir(),
    robot_left_ = robotLeft(),
    robot_right_ = robotRight(),
    left_i_ = robot_ground_.link("l5")->id();
    right_i_ = robot_ground_.link("l1")->id();
    body_i_ = robot_ground_.link("l3")->id();
    left_j_ = robot_ground_.joint("j5")->id();
    right_j_ = robot_ground_.joint("j0")->id();
}


JumpingRobot::JumpingRobot(const JointValueMap &rest_angles,
                 const JointValueMap &init_angles,
                 const JointValueMap &init_vels,
                 const JointValueMap &init_masses,
                 const Params& params) : init_angles_(init_angles),
                                        init_vels_(init_vels),
                                        init_masses_(init_masses),
                                        params_(params),
                                        gravity_((Vector(3) << 0, 0, -9.8).finished()),
                                        planar_axis_((Vector(3) << 1, 0, 0).finished()),
                                        actuators_(constructActuators(rest_angles))
    {
        OptimizerSetting opt_;
        opt_.bp_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.bv_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.ba_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.linear_a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
        opt_.linear_f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
        opt_.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.01);
        opt_.linear_t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.cfriction_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.cv_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
        opt_.ca_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
        opt_.cm_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
        opt_.planar_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
        opt_.linear_planar_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
        opt_.prior_q_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.prior_qv_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.prior_qa_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.prior_t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.q_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.v_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        opt_.pose_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.twist_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
        opt_.time_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.0001);
        opt_.jl_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.001);
        graph_builder_ = DynamicsGraph(opt_, gravity_, planar_axis_);

        initializeRobots();
    }


const Robot &JumpingRobot::getRobot(const Phase phase) const
{
    if (phase == Phase::Ground)
    {
        return robot_ground_;
    }
    else if (phase == Phase::Air)
    {
        return robot_air_;
    }
    else if (phase == Phase::Left)
    {
        return robot_left_;
    }
    else
    { // Phase::Right
        return robot_right_;
    }
}

vector<PneumaticActuator> JumpingRobot::constructActuators(const JointValueMap &rest_angles)
{
    // const double c1 = -12.05020559, c2 = 8.88481485, c3 = -85.56821655, t0 = 0.224;
    // const vector<double> pressure_coeffs{t0, c1, c2, c3};

    // // coefficients for pneumatic actuator factor
    // // const double p00 = -17.39, p10 = 1.11, p01 = 2.22, p20 = -0.9486,
    // //              p11 = -0.4481, p02 = -0.0003159, p30 = 0.1745, p21 = 0.01601,
    // //              p12 = 0.0001081, p03 = -7.703e-07;
    // // const vector<double> pneumatic_coeffs{p00, p10, p01, p20, p11, p02, p30, p21, p12, p03};
    // const vector<double> x0_coeffs {3.05583930e+00, 7.58361626e-02, -4.91579771e-04, 1.42792618e-06, -1.54817477e-09};
    // const vector<double> f0_coeffs {0, 1.966409};
    // const vector<double> k_coeffs {0, 0.35541599};

    const double r = 0.04; // pully radius

    double q_anta_limit_knee = -0.0/180.0 * M_PI;
    double q_anta_limit_hip = 70.0/180.0 * M_PI - params_.angle_offset;

    Robot robot_ground = loadRobot();

    std::vector<std::string> actuator_names{"j1", "j2", "j3", "j4"};
    std::vector<double> kt_list{8200, 8200, 8200, 8200};
    std::vector<double> ka_list{2.1, 2.5, 2.5, 2.1}; //;{0, 0, 0, 0}
    std::vector<double> b_list {0.03, 0.03, 0.03, 0.03};//{0.6, 0.8, 0.8, 0.6};
    std::vector<double> q_anta_limit_list{q_anta_limit_knee, q_anta_limit_hip, q_anta_limit_hip, q_anta_limit_knee};
    std::vector<bool> positive_list{false, true, true, false};
    vector<PneumaticActuator> actuators;
    actuators.reserve(actuator_names.size());

    double D = 0.1575 * 0.0254;
    double L = 74 * 0.0254;
    double mu = 1.8377e-5;
    double epsilon = 1e-5;
    double ct = 1e-3;

    for (int idx = 0; idx < actuator_names.size(); idx++)
    {
        PneumaticActuator::Params actuator_params;
        std::string name = actuator_names[idx];
        actuator_params.joint_name = name;
        actuator_params.j = robot_ground.joint(name)->id();
        // actuator_params.x0_coeffs = x0_coeffs;
        // actuator_params.f0_coeffs = f0_coeffs;
        // actuator_params.k_coeffs = k_coeffs;
        // actuator_params.p_coeffs = pressure_coeffs;

        actuator_params.kt = kt_list[idx];
        actuator_params.ka = ka_list[idx];
        actuator_params.q_rest = rest_angles.at(name);
        actuator_params.q_anta_limit = q_anta_limit_list[idx];
        actuator_params.r = r;
        actuator_params.b = b_list[idx];
        actuator_params.positive = positive_list[idx];
        
        actuator_params.Rs = params_.Rs;
        actuator_params.T = params_.T;
        actuator_params.D = D;
        actuator_params.L = L;
        actuator_params.mu = mu;
        actuator_params.epsilon = epsilon;
        actuator_params.ct = ct;

        actuators.emplace_back(PneumaticActuator(actuator_params));
    }
    return actuators;
}

JointValueMap JumpingRobot::extractTorques(const Phase phase, const int t, const Values &values) const
{
    JointValueMap torques;
    for (JointSharedPtr joint : getRobot(phase).joints())
    {
        torques[joint->name()] = 0;
    }

    for (auto &actuator : actuators_)
    {
        int j = actuator.j();
        torques[actuator.name()] = values.atDouble(TorqueKey(j, t));
    }
    return torques;
}


JointValueMap JumpingRobot::extractMdots(const Phase phase, const int t, const Values &values) const
{
    JointValueMap mdots;
    for (auto &actuator : actuators_)
    {
        int j = actuator.j();
        mdots[actuator.name()] = values.atDouble(MassRateActualKey(j, t));
    }
    return mdots;
}



Values JumpingRobot::runActuators(const int t, const double time,
                                  const JointValueMap &qs,
                                  const JointValueMap &vs,
                                  const JointValueMap &ms,
                                  const double source_mass,
                                  const Vector &open_times,
                                  const Vector &close_times,
                                  const Values& previous_values) const {
  Values values;

  Values sourceResults = actuators_[0].computeSourceResult(t, source_mass, params_.tank_volume);
  merge(values, sourceResults);
  double source_pressure = sourceResults.atDouble(SourcePressureKey(t));

  for (int actuator_idx = 0; actuator_idx < actuators_.size(); actuator_idx++) {
    // TODO: incorporate open/close times

    auto &actuator = actuators_[actuator_idx];
    std::string name = actuators_[actuator_idx].name();
    PriorValues prior_values;
    prior_values.q = qs.at(name);
    prior_values.v = vs.at(name);
    prior_values.Ps = source_pressure;
    prior_values.m = ms.at(name);
    prior_values.t = time;
    prior_values.to = open_times(actuator_idx);
    prior_values.tc = close_times(actuator_idx);
    merge(values, actuator.computeResult(t, prior_values, previous_values));
  }
  return values;
}

// Values JumpingRobot::simulate(const int num_steps, const double dt, const Vector &init_pressures, const Vector &start_times)
// {
//     Simulator simulator(robot_ground_, init_angles_, init_vels_, gravity_, planar_axis_);
//     simulator.reset();

//     Values values;
//     for (int t = 0; t < num_steps; t++)
//     {
//         Values actuator_values = runActuators(t, t * dt, simulator.getJointAngles(), simulator.getJointVelocities(), init_pressures, start_times);
//         JointValueMap torques = extractTorques(Phase::Ground, t, actuator_values);
//         merge(values, actuator_values);
//         simulator.step(torques, dt);
//     }
//     Values actuator_values = runActuators(num_steps, num_steps * dt, simulator.getJointAngles(), simulator.getJointVelocities(), init_pressures, start_times);
//     JointValueMap torques = extractTorques(Phase::Ground, num_steps, actuator_values);
//     merge(values, actuator_values);
//     simulator.forwardDynamics(torques);
//     merge(values, simulator.getValues());
//     return values;
// }

double getLeftForceZ(const Robot &jumping_robot, const int t, const Values &values)
{
    int i = jumping_robot.link("l5")->id();
    int j = jumping_robot.joint("j5")->id();
    Vector6 wrench_b = values.at<Vector6>(WrenchKey(i, j, t));
    Pose3 T_wb = values.at<Pose3>(PoseKey(i, t));
    Vector6 wrench_w = T_wb.inverse().AdjointMap().transpose() * wrench_b;
    // std::cout << "left wrench: " << wrench_b.transpose() << "\n";
    return wrench_w(5);
}

double getRightForceZ(const Robot &jumping_robot, const int t, const Values &values)
{
    int i = jumping_robot.link("l1")->id();
    int j = jumping_robot.joint("j0")->id();
    Vector6 wrench_b = values.at<Vector6>(WrenchKey(i, j, t));
    Pose3 T_wb = values.at<Pose3>(PoseKey(i, t));
    Vector6 wrench_w = T_wb.inverse().AdjointMap().transpose() * wrench_b;
    // std::cout << "right wrench: " << wrench_b.transpose() << "\n";
    return wrench_w(5);
}

JumpingRobot::Phase JumpingRobot::phaseTransition(const Phase current_phase,
                                                  const int t,
                                                  const Values &values) const
{
    double threshold = 0;
    if (current_phase == Phase::Ground)
    {
        double f_left = getLeftForceZ(robot_ground_, t, values);
        double f_right = getRightForceZ(robot_ground_, t, values);
        std::cout << "\tf_left: " << f_left << "\tf_right: " << f_right << "\n";
        if ((f_left < threshold) && (f_right < threshold))
        {
            return Phase::Air;
        }
        else if (f_left < threshold)
        {
            return Phase::Right;
        }
        else if (f_right < threshold)
        {
            return Phase::Left;
        }
        else
        {
            return Phase::Ground;
        }
    }
    else if (current_phase == Phase::Left)
    {
        double f_left = getLeftForceZ(robot_left_, t, values);
        if (f_left < 0)
        {
            return Phase::Air;
        }
        else
        {
            return Phase::Left;
        }
    }
    else if (current_phase == Phase::Right)
    {
        double f_right = getRightForceZ(robot_right_, t, values);
        if (f_right < 0)
        {
            return Phase::Air;
        }
        else
        {
            return Phase::Right;
        }
    }
    else // Phase::Air
    {
        return Phase::Air;
    }
}

Values JumpingRobot::valuesFromPrev(const Phase phase,
                                    const int t_in, const int t_out,
                                    const Values &values_in,
                                    const bool specify_torque) const
{
    // std::cout << "Interpolate int step " << t_out << " from step " << t_in << "\n";

    Values values_out;
    const Robot &robot = getRobot(phase);
    for (JointSharedPtr joint : robot.joints())
    {
        int j = joint->id();
        int i1 = joint->parent()->id();
        int i2 = joint->child()->id();
        values_out.insert(JointAngleKey(j, t_out), values_in.atDouble(JointAngleKey(j, t_in)));
        values_out.insert(JointVelKey(j, t_out), values_in.atDouble(JointVelKey(j, t_in)));
        values_out.insert(JointAccelKey(j, t_out), values_in.atDouble(JointAccelKey(j, t_in)));
        values_out.insert(TorqueKey(j, t_out), values_in.atDouble(TorqueKey(j, t_in)));
        values_out.insert(WrenchKey(i1, j, t_out), values_in.at<Vector6>(WrenchKey(i1, j, t_in)));
        values_out.insert(WrenchKey(i2, j, t_out), values_in.at<Vector6>(WrenchKey(i2, j, t_in)));
    }
    for (LinkSharedPtr link : robot.links())
    {
        int i = link->id();
        values_out.insert(PoseKey(i, t_out), values_in.at<Pose3>(PoseKey(i, t_in)));
        values_out.insert(TwistKey(i, t_out), values_in.at<Vector6>(TwistKey(i, t_in)));
        values_out.insert(TwistAccelKey(i, t_out), values_in.at<Vector6>(TwistAccelKey(i, t_in)));
    }
    if (!specify_torque)
    {
        for (const PneumaticActuator &actuator : actuators_)
        {
            int j = actuator.j();
            values_out.insert(PressureKey(j, t_out), values_in.atDouble(PressureKey(j, t_in)));
            values_out.insert(ContractionKey(j, t_out), values_in.atDouble(ContractionKey(j, t_in)));
            values_out.insert(ForceKey(j, t_out), values_in.atDouble(ForceKey(j, t_in)));
            values_out.insert(MassKey(j, t_out), values_in.atDouble(MassKey(j, t_in)));
            values_out.insert(MassRateOpenKey(j, t_out), values_in.atDouble(MassRateOpenKey(j, t_in)));
            values_out.insert(MassRateActualKey(j, t_out), values_in.atDouble(MassRateActualKey(j, t_in)));
            values_out.insert(VolumeKey(j, t_out), values_in.atDouble(VolumeKey(j, t_in)));
        }
    }
    values_out.insert(TimeKey(t_out), values_in.atDouble(TimeKey(t_in)));
    values_out.insert(SourceMassKey(t_out), values_in.atDouble(SourceMassKey(t_in)));
    values_out.insert(SourcePressureKey(t_out), values_in.atDouble(SourcePressureKey(t_in)));
    return values_out;
}

Values JumpingRobot::getInterpolateValues(const Phase phase,
                                          const double t_in, const int t_out,
                                          const Values &values_in,
                                          const bool specify_torque) const
{
    // check if t_in is int
    int t_prev = floor(t_in);
    int t_next = ceil(t_in);
    double ratio_next = t_in - t_prev;
    double ratio_prev = t_next - t_in;
    double threshold = 1e-6;
    if (ratio_next < threshold)
    {
        return valuesFromPrev(phase, t_prev, t_out, values_in, specify_torque);
    }
    if (ratio_prev < threshold)
    {
        return valuesFromPrev(phase, t_next, t_out, values_in, specify_torque);
    }

    std::cout << "Interpolate for step " << t_out << " from step " << t_in << "\n";
    gtsam::Values values_out;

    std::cout << "\tt_prev: " << t_prev << "\tt_next: " << t_next << "\tratio_prev: " << ratio_prev << "\tratio_next: " << ratio_next << "\n";
    const Robot &robot = getRobot(phase);
    // joints
    for (JointSharedPtr joint : robot.joints())
    {
        int j = joint->id();
        int i1 = joint->parent()->id();
        int i2 = joint->child()->id();
        // angle
        double q_prev = values_in.atDouble(JointAngleKey(j, t_prev));
        double q_next = values_in.atDouble(JointAngleKey(j, t_next));
        double q = q_prev * ratio_prev + q_next * ratio_next;
        // velocity
        double v_prev = values_in.atDouble(JointVelKey(j, t_prev));
        double v_next = values_in.atDouble(JointVelKey(j, t_next));
        double v = v_prev * ratio_prev + v_next * ratio_next;
        // acceleration
        double a_prev = values_in.atDouble(JointAccelKey(j, t_prev));
        double a_next = values_in.atDouble(JointAccelKey(j, t_next));
        double a = a_prev * ratio_prev + a_next * ratio_next;
        // torque
        double torque_prev = values_in.atDouble(JointAccelKey(j, t_prev));
        double torque_next = values_in.atDouble(JointAccelKey(j, t_next));
        double torque = torque_prev * ratio_prev + torque_next * ratio_next;
        // wrench
        Vector6 wrench1_prev = values_in.at<Vector6>(WrenchKey(i1, j, t_prev));
        Vector6 wrench1_next = values_in.at<Vector6>(WrenchKey(i1, j, t_next));
        Vector6 wrench1 = wrench1_prev * ratio_prev + wrench1_next * ratio_next;
        Vector6 wrench2_prev = values_in.at<Vector6>(WrenchKey(i2, j, t_prev));
        Vector6 wrench2_next = values_in.at<Vector6>(WrenchKey(i2, j, t_next));
        Vector6 wrench2 = wrench2_prev * ratio_prev + wrench2_next * ratio_next;

        values_out.insert(JointAngleKey(j, t_out), q);
        values_out.insert(JointVelKey(j, t_out), v);
        values_out.insert(JointAccelKey(j, t_out), a);
        values_out.insert(TorqueKey(j, t_out), torque);
        values_out.insert(WrenchKey(i1, j, t_out), wrench1);
        values_out.insert(WrenchKey(i2, j, t_out), wrench2);
    }
    // links
    for (LinkSharedPtr link : robot.links())
    {
        int i = link->id();
        // pose
        Pose3 pose_prev = values_in.at<Pose3>(PoseKey(i, t_prev));
        Pose3 pose_next = values_in.at<Pose3>(PoseKey(i, t_next));
        Vector6 log_pose = Pose3::Logmap(pose_prev) * ratio_prev + Pose3::Logmap(pose_next) * ratio_next;
        Pose3 pose = Pose3::Expmap(log_pose);
        // twist
        Vector6 twist_prev = values_in.at<Vector6>(TwistKey(i, t_prev));
        Vector6 twist_next = values_in.at<Vector6>(TwistKey(i, t_next));
        Vector6 twist = twist_prev * ratio_prev + twist_next * ratio_next;
        // twist acceleration
        Vector6 twistAccel_prev = values_in.at<Vector6>(TwistAccelKey(i, t_prev));
        Vector6 twistAccel_next = values_in.at<Vector6>(TwistAccelKey(i, t_next));
        Vector6 twistAccel = twistAccel_prev * ratio_prev + twistAccel_next * ratio_next;
        values_out.insert(PoseKey(i, t_out), pose);
        values_out.insert(TwistKey(i, t_out), twist);
        values_out.insert(TwistAccelKey(i, t_out), twistAccel);
    }
    // actuators
    if (!specify_torque)
    {
        for (const PneumaticActuator &actuator : actuators_)
        {
            int j = actuator.j();
            // pressure
            double p_prev = values_in.atDouble(PressureKey(j, t_prev));
            double p_next = values_in.atDouble(PressureKey(j, t_next));
            double p = p_prev * ratio_prev + p_next * ratio_next;
            // contraction length
            double x_prev = values_in.atDouble(ContractionKey(j, t_prev));
            double x_next = values_in.atDouble(ContractionKey(j, t_next));
            double x = x_prev * ratio_prev + x_next * ratio_next;
            // force
            double f_prev = values_in.atDouble(ForceKey(j, t_prev));
            double f_next = values_in.atDouble(ForceKey(j, t_next));
            double f = f_prev * ratio_prev + f_next * ratio_next;

            double m_prev = values_in.atDouble(MassKey(j, t_prev));
            double m_next = values_in.atDouble(MassKey(j, t_next));
            double m = m_prev * ratio_prev + m_next * ratio_next;

            double mdoto_prev = values_in.atDouble(MassRateOpenKey(j, t_prev));
            double mdoto_next = values_in.atDouble(MassRateOpenKey(j, t_next));
            double mdoto = mdoto_prev * ratio_prev + mdoto_next * ratio_next;

            double mdota_prev = values_in.atDouble(MassRateActualKey(j, t_prev));
            double mdota_next = values_in.atDouble(MassRateActualKey(j, t_next));
            double mdota = mdota_prev * ratio_prev + mdota_next * ratio_next;

            double vo_prev = values_in.atDouble(VolumeKey(j, t_prev));
            double vo_next = values_in.atDouble(VolumeKey(j, t_next));
            double vo = vo_prev * ratio_prev + vo_next * ratio_next;

            values_out.insert(PressureKey(j, t_out), p);
            values_out.insert(ContractionKey(j, t_out), x);
            values_out.insert(ForceKey(j, t_out), f);
            values_out.insert(MassKey(j, t_out), m);
            values_out.insert(MassRateOpenKey(j, t_out), mdoto);
            values_out.insert(MassRateActualKey(j, t_out), mdota);
            values_out.insert(VolumeKey(j, t_out), vo);
        }
    }

    // time
    double time_prev = values_in.atDouble(TimeKey(t_prev));
    double time_next = values_in.atDouble(TimeKey(t_next));
    double time = time_prev * ratio_prev + time_next * ratio_next;
    values_out.insert(TimeKey(t_out), time);

    // for tank
    double ms_prev = values_in.atDouble(SourceMassKey(t_prev));
    double ms_next = values_in.atDouble(SourceMassKey(t_next));
    double ms = ms_prev * ratio_prev + ms_next * ratio_next;
    values_out.insert(SourceMassKey(t_out), ms);

    double ps_prev = values_in.atDouble(SourcePressureKey(t_prev));
    double ps_next = values_in.atDouble(SourcePressureKey(t_next));
    double ps = ps_prev * ratio_prev + ps_next * ratio_next;
    values_out.insert(SourcePressureKey(t_out), ps);

    return values_out;
}

Values JumpingRobot::linearInterpolation(const double dt_in,
                                         const std::vector<Phase> &phases_es_in,
                                         const gtsam::Values &values_in,
                                         const vector<Phase> &phases_out,
                                         const vector<int> &phase_steps_out,
                                         const bool specify_torque) const
{
    gtsam::Values values;
    int t_in_start = 0, // start step for phase
        t_in_end = 0,   // end step for phase
        t_out_start = 0,
        t_out_end = 0;
    for (int phase_idx = 0; phase_idx < phases_out.size(); phase_idx++)
    {
        Phase phase = phases_out[phase_idx];

        //// count the number of input steps for the phase
        t_in_start = t_in_end;
        while (++t_in_end < phases_es_in.size() && phases_es_in[t_in_end] == phase)
        {
        }
        t_in_end--;

        int num_steps_in = t_in_end - t_in_start;
        int num_steps_out = phase_steps_out[phase_idx];
        t_out_start = t_out_end;
        t_out_end = t_out_start + num_steps_out;
        double dt_out = dt_in * double(num_steps_in) / double(num_steps_out);

        std::cout << "Phase " << phase << "\n";
        std::cout << "\t t_in_start:    " << t_in_start << "\n";
        std::cout << "\t t_in_end:      " << t_in_end << "\n";
        std::cout << "\t t_out_start:   " << t_out_start << "\n";
        std::cout << "\t t_out_end:     " << t_out_end << "\n";
        std::cout << "\t num_steps_in:  " << num_steps_in << "\n";
        std::cout << "\t num_steps_out: " << num_steps_out << "\n";
        std::cout << "\t dt_out:        " << dt_out << "\n";

        //// interpolation steps within a phase
        int iter_start = phase_idx == 0 ? t_out_start : t_out_start + 1;
        int iter_end = phase_idx == phases_out.size() - 1 ? t_out_end + 1 : t_out_end;
        for (int t_out = iter_start; t_out < iter_end; t_out++)
        {
            double ratio2 = double(t_out - t_out_start) / double(num_steps_out);
            double ratio1 = double(t_out_end - t_out) / double(num_steps_out);
            double t_in = ratio1 * t_in_start + ratio2 * t_in_end;
            values.insert(getInterpolateValues(phase, t_in, t_out, values_in, specify_torque));
        }

        //// transition step
        if (phase_idx != phases_out.size() - 1)
        {
            Values transition_values = valuesFromPrev(phase, t_in_end, t_out_end, values_in, specify_torque);
            Phase next_phase = phases_out[phase_idx + 1];
            // // TODO: add for other transitions
            // if (next_phase == Phase::Air && phase == Phase::Ground)
            // {
            //     values.insert(valuesFromPrev(t_in_end, curr_step_out, values_in, true));
            //     values.insert(valuesFromPrev(t_in_end, curr_step_out, values_in, false));
            // }
            values.insert(transition_values);
        }

        //// dt variables
        // std::cout << "interpolate for phase " << phase_idx << " as " << dt_out << "\n";
        values.insert(PhaseKey(phase), dt_out);
    }

    if (!specify_torque)
    {
        for (const auto &actuator : actuators_)
        {
            int j = actuator.j();
            values.insert(ValveOpenTimeKey(j), values_in.atDouble(ValveOpenTimeKey(j)));
            values.insert(ValveCloseTimeKey(j), values_in.atDouble(ValveCloseTimeKey(j)));
        }
    }
    values.insert(SourceVolumeKey(), values_in.atDouble(SourceVolumeKey()));

    return values;
}

NonlinearFactorGraph JumpingRobot::actuatorGraphs(const int num_steps) const
{
    NonlinearFactorGraph graph;
    for (int t = 0; t <= num_steps; t++)
    {
        graph.add(actuators_[0].sourceFactorGraph(t));
        for (auto &actuator : actuators_)
        {
            graph.add(actuator.actuatorFactorGraph(t));
        }
    }
    return graph;
}

NonlinearFactorGraph JumpingRobot::zeroTorquePriors(const std::vector<Phase> &phases) const
{
    NonlinearFactorGraph graph;
    for (int t = 0; t < phases.size(); t++)
    {
        Phase phase = phases[t];
        if (leftOnGround(phase))
        {
            graph.add(gtsam::PriorFactor<double>(TorqueKey(left_j_, t), double(0),
                                                 graph_builder_.opt().prior_t_cost_model));
        }
        if (rightOnGround(phase))
        {
            graph.add(gtsam::PriorFactor<double>(TorqueKey(right_j_, t), double(0),
                                                 graph_builder_.opt().prior_t_cost_model));
        }
    }
    return graph;
}

NonlinearFactorGraph JumpingRobot::trajectoryFG(const int num_steps, const double dt) const
{
    NonlinearFactorGraph graph;
    auto collocation = CollocationScheme::Trapezoidal;

    // dynamics trajectory graph
    graph.add(graph_builder_.trajectoryFG(robot_ground_, num_steps, dt, collocation));

    // actuator graph
    graph.add(actuatorGraphs(num_steps));

    // torque priors for unactuated joints
    // auto joints = robot_ground_.joints();
    for (int t = 0; t <= num_steps; t++)
    {
        int j = robot_ground_.joint("j0")->id();
        graph.add(gtsam::PriorFactor<double>(TorqueKey(j, t), double(0),
                                             graph_builder_.opt().prior_t_cost_model));

        j = robot_ground_.joint("j5")->id();
        graph.add(gtsam::PriorFactor<double>(TorqueKey(j, t), double(0),
                                             graph_builder_.opt().prior_t_cost_model));
    }

    // time priors for each step
    for (int t = 0; t <= num_steps; t++)
    {
        graph.add(gtsam::PriorFactor<double>(TimeKey(t), t * dt,
                                             graph_builder_.opt().time_cost_model));
    }
    return graph;
}

double getFz(const gtsam::Vector6 &wrench,
             gtsam::OptionalJacobian<1, 6> H_wrench)
{
    gtsam::Vector6 H_Fz;
    H_Fz << 0, 0, 0, 0, 0, 1;
    if (H_wrench)
        *H_wrench = H_Fz.transpose();
    return wrench(5);
}

NonlinearFactorGraph JumpingRobot::transitionGraph(const int t,
                                                   const JumpingRobot::Phase phase1,
                                                   const JumpingRobot::Phase phase2) const
{
    // decide which phase to use
    bool left_on_ground = leftOnGround(phase1) || leftOnGround(phase2);
    bool right_on_ground = rightOnGround(phase1) || rightOnGround(phase2);
    Phase phase = getPhase(left_on_ground, right_on_ground);
    NonlinearFactorGraph graph = graph_builder_.dynamicsFactorGraph(getRobot(phase), t);

    // add guardian factors (wrenches should be 0)
    if (left_on_ground)
    {
        // graph.add(gtsam::PriorFactor<gtsam::Vector6>(WrenchKey(left_i_, left_j_, t), gtsam::Vector6::Zero(), graph_builder_.opt().f_cost_model));
        int ground_i_ = robot_ground_.link("l0")->id();
        gtsam::Vector6_ wrench_left(WrenchKey(ground_i_, left_j_, t));
        gtsam::Double_ expr(getFz, wrench_left);
        gtsam::ExpressionFactor<double> factor(graph_builder_.opt().time_cost_model, 0., expr);
        graph.add(factor);
    }
    if (right_on_ground)
    {
        // graph.add(gtsam::PriorFactor<gtsam::Vector6>(WrenchKey(right_i_, right_j_, t), gtsam::Vector6::Zero(), graph_builder_.opt().f_cost_model));
        int ground_i_ = robot_ground_.link("l0")->id();
        gtsam::Vector6_ wrench_right(WrenchKey(ground_i_, right_j_, t));
        gtsam::Double_ expr(getFz, wrench_right);
        gtsam::ExpressionFactor<double> factor(graph_builder_.opt().time_cost_model, 0., expr);
        graph.add(factor);
    }
    return graph;
}

NonlinearFactorGraph JumpingRobot::airCollocationFactors(const vector<Phase> &phase_seq,
                                                         const vector<int> &phase_steps,
                                                         const CollocationScheme collocation) const
{
    NonlinearFactorGraph graph;
    for (int phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++)
    {
        int first_t = std::accumulate(phase_steps.begin(), phase_steps.begin() + phase_idx, 0);
        if (true)
        {
            for (int t = first_t; t < first_t + phase_steps[phase_idx]; t++)
            {
                if (collocation == CollocationScheme::Euler)
                {
                    graph.add(EulerPoseColloFactor(PoseKey(body_i_, t), PoseKey(body_i_, t + 1),
                                                   TwistKey(body_i_, t), PhaseKey(phase_seq[phase_idx]), graph_builder_.opt().pose_col_cost_model));
                    graph.add(EulerTwistColloFactor(TwistKey(body_i_, t), TwistKey(body_i_, t + 1),
                                                    TwistAccelKey(body_i_, t), PhaseKey(phase_seq[phase_idx]), graph_builder_.opt().twist_col_cost_model));
                }
                else if (collocation == CollocationScheme::Trapezoidal)
                {
                    graph.add(TrapezoidalPoseColloFactor(PoseKey(body_i_, t), PoseKey(body_i_, t + 1),
                                                         TwistKey(body_i_, t), TwistKey(body_i_, t + 1), PhaseKey(phase_seq[phase_idx]), graph_builder_.opt().pose_col_cost_model));
                    graph.add(TrapezoidalTwistColloFactor(TwistKey(body_i_, t), TwistKey(body_i_, t + 1),
                                                          TwistAccelKey(body_i_, t), TwistAccelKey(body_i_, t + 1), PhaseKey(phase_seq[phase_idx]), graph_builder_.opt().twist_col_cost_model));
                }
            }
        }
    }
    return graph;
}

NonlinearFactorGraph JumpingRobot::costFactors(const size_t num_steps) const {
  NonlinearFactorGraph graph;
  for (size_t t=0; t<num_steps; t++) {
    for (const auto& actuator:actuators_) {
      int j = actuator.j();
      graph.add(PriorFactor<double>(MassRateActualKey(j, t), 0.0, actuator.mass_rate_obj_model));
    }
  }
  return graph;
}


double multDouble1(const double &d1, const double &d2,
                  gtsam::OptionalJacobian<1, 1> H1,
                  gtsam::OptionalJacobian<1, 1> H2) {
  if (H1) *H1 = gtsam::I_1x1 * d2;
  if (H2) *H2 = gtsam::I_1x1 * d1;
  return d1 * d2;
}

gtsam::ExpressionFactorGraph doubleCollocationFactor(Key key1, Key key2, Key rate1_key, Key rate2_key, Key dt_key, 
                       const CollocationScheme collocation, gtsam::noiseModel::Base::shared_ptr model) {
  gtsam::ExpressionFactorGraph graph;
  Double_ expr1 = Double_(key1);
  Double_ expr2 = Double_(key2);
  Double_ expr_rate1 = Double_(rate1_key);
  Double_ expr_rate2 = Double_(rate2_key);
  Double_ expr_dt = Double_(dt_key);
  if (collocation == CollocationScheme::Euler) {
    Double_ ratedt(multDouble1, expr_dt, expr_rate1);
    graph.addExpressionFactor(expr1 + ratedt - expr2, 0.0, model);
  } else if (collocation == CollocationScheme::Trapezoidal) {
    Double_ rate1dt(multDouble1, expr_dt, expr_rate1);
    Double_ rate2dt(multDouble1, expr_dt, expr_rate2);
    graph.addExpressionFactor(expr1 + 0.5 * rate1dt + 0.5 * rate2dt - expr2,
                              0.0, model);
  } else {
    throw std::runtime_error("collocation not implemented yet");
  }
  return graph;
}

NonlinearFactorGraph JumpingRobot::massCollocationFactors(const vector<Phase> &phase_seq,
                                                         const vector<int> &phase_steps,
                                                         const CollocationScheme collocation) const
{
  NonlinearFactorGraph graph;
  int t=0;
  for (int phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++) {
    Key dt_key = PhaseKey(phase_seq[phase_idx]);
    for (int ta = 0; ta<phase_steps[phase_idx]; ta++) {

      std::vector<Double_> mdot1_vec;
      std::vector<Double_> mdot2_vec;
      for (const auto& actuator:actuators_) {
        int j = actuator.j();
        Key m1 = MassKey(j, t);
        Key m2 = MassKey(j, t+1);
        Key mdot1_key = MassRateActualKey(j, t);
        Key mdot2_key = MassRateActualKey(j, t+1);
        
        graph.add(doubleCollocationFactor(m1, m2, mdot1_key, mdot2_key, dt_key, collocation, actuator.m_col_cost_model));
        mdot1_vec.push_back(Double_(mdot1_key));
        mdot2_vec.push_back(Double_(mdot2_key));
      }

      gtsam::ExpressionFactorGraph exp_graph;
      Key ms1_key = SourceMassKey(t);
      Key ms2_key = SourceMassKey(t+1);

      Double_ expr1 = Double_(ms1_key);
      Double_ expr2 = Double_(ms2_key);
      Double_ expr_dt = Double_(dt_key);
      std::vector<Double_> mdot1dt_vec;
      std::vector<Double_> mdot2dt_vec;
      for (int idx=0; idx<mdot1_vec.size(); idx++) {
        mdot1dt_vec.push_back(Double_(multDouble1, expr_dt, mdot1_vec[idx]));
        mdot2dt_vec.push_back(Double_(multDouble1, expr_dt, mdot2_vec[idx]));
      }

      if (collocation == CollocationScheme::Euler) {
        exp_graph.addExpressionFactor(expr1 - mdot1dt_vec[0] - mdot1dt_vec[1] - mdot1dt_vec[2] - mdot1dt_vec[3] - expr2, 0.0, actuators_[0].m_col_cost_model);
      } else if (collocation == CollocationScheme::Trapezoidal) {
        exp_graph.addExpressionFactor(expr1 - 0.5 * mdot1dt_vec[0] - 0.5 * mdot1dt_vec[1] - 0.5 * mdot1dt_vec[2] - 0.5 * mdot1dt_vec[3]
                                        - 0.5 * mdot2dt_vec[0] - 0.5 * mdot2dt_vec[1] - 0.5 * mdot2dt_vec[2] - 0.5 * mdot2dt_vec[3] - expr2,
                                  0.0, actuators_[0].m_col_cost_model);
      } else {
        throw std::runtime_error("collocation not implemented yet");
      }
      graph.add(exp_graph);
      t+=1;
    }
  }
  return graph;
}


NonlinearFactorGraph JumpingRobot::symmetricFactors(const vector<Phase> &phase_seq,
                                                          const vector<int> &phase_steps) const
{
  NonlinearFactorGraph graph;
  int t = 0;
  for (int phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++) {
    if (phase_seq[phase_idx] == Phase::Ground) {
      for (int step = 0; step < phase_steps[phase_idx]; step++) {
        int j_knee1 = getRobot(Phase::Ground).joint("j1")->id();
        int j_knee2 = getRobot(Phase::Ground).joint("j4")->id();
        graph.add(BetweenFactor<double>(JointAngleKey(j_knee1, t+step+1), JointAngleKey(j_knee2, t+step+1), 0.0, graph_builder_.opt().q_col_cost_model));
        graph.add(BetweenFactor<double>(JointVelKey(j_knee1, t+step+1), JointVelKey(j_knee2, t+step+1), 0.0, graph_builder_.opt().v_col_cost_model));
      }
    }
    t += phase_steps[phase_idx];
  }
  return graph;
}


NonlinearFactorGraph JumpingRobot::multiPhaseTrajectoryFG_old(
    const std::vector<Robot> &robots, 
    const std::vector<int> &phase_steps, const vector<Phase> &phase_seq,
    const std::vector<gtsam::NonlinearFactorGraph> &transition_graphs,
    const CollocationScheme collocation,
    const std::vector<std::vector<std::string>> &phase_joint_names) const {
  NonlinearFactorGraph graph;
  int num_phases = robots.size();

  // add dynamcis for each step
  int t = 0;
  graph.add(graph_builder_.dynamicsFactorGraph(robots[0], t));

  for (int phase_idx = 0; phase_idx < num_phases; phase_idx++) {
    // in-phase
    for (int phase_step = 0; phase_step < phase_steps[phase_idx] - 1;
         phase_step++) {
      graph.add(graph_builder_.dynamicsFactorGraph(robots[phase_idx], ++t));
    }
    // transition
    if (phase_idx == num_phases - 1) {
      graph.add(graph_builder_.dynamicsFactorGraph(robots[phase_idx], ++t));
    } else {
      t++;
      graph.add(transition_graphs[phase_idx]);
    }
  }

  // add collocation factors
  t = 0;
  for (int phase_idx = 0; phase_idx < num_phases; phase_idx++) {
    auto phase = phase_seq[phase_idx];
    for (int phase_step = 0; phase_step < phase_steps[phase_idx]; phase_step++) {
        for (const auto& joint_name : phase_joint_names[phase_idx]) {
            int j = robots[phase_idx].joint(joint_name)->id();
            graph.add(graph_builder_.jointMultiPhaseCollocationFactors(j, t, phase, collocation));
        }
        t++;
    //   graph.add(
    //       graph_builder_.multiPhaseCollocationFactors(robots[phase_idx], phase_joint_names[phase_idx], t++, phase_idx, collocation));
    }
  }
  return graph;
}

NonlinearFactorGraph JumpingRobot::multiPhaseTrajectoryFG(const vector<Phase> &phase_seq,
                                                          const vector<int> &phase_steps,
                                                          const CollocationScheme collocation,
                                                          const bool specify_torque) const
{
    vector<Robot> robots;
    for (JumpingRobot::Phase phase : phase_seq)
    {
        robots.emplace_back(getRobot(phase));
    }

    // transition graphs
    vector<NonlinearFactorGraph> transition_graphs;
    int t = 0;
    for (int phase_idx = 0; phase_idx < phase_seq.size() - 1; phase_idx++)
    {
        t += phase_steps[phase_idx];
        transition_graphs.emplace_back(transitionGraph(t, phase_seq[phase_idx], phase_seq[phase_idx + 1]));
    }

    vector<vector<std::string>> phase_joint_names;
    for (int phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++) {
        if (phase_seq[phase_idx] == Phase::Ground) {
            // phase_joint_names.push_back(vector<std::string>{"j2", "j3"});
            phase_joint_names.push_back(vector<std::string>{});
        }
        else if (phase_seq[phase_idx] == Phase::Air) {
            phase_joint_names.push_back(vector<std::string>{"j1", "j2", "j3", "j4"});
        }
        else {
            throw std::runtime_error("phase not considered yet");
        }
    }

    NonlinearFactorGraph graph = multiPhaseTrajectoryFG_old(robots, phase_steps, phase_seq, transition_graphs, collocation, phase_joint_names);

    // integrate base pose, twist for in-air phase
    graph.add(airCollocationFactors(phase_seq, phase_steps, collocation));

    // graph.add(symmetricFactors(phase_seq, phase_steps));
    // graph.add(angleCollocationFactors(phase_seq, phase_steps, collocation));

    graph.add(massCollocationFactors(phase_seq, phase_steps, collocation));

    // add actuator graph
    if (!specify_torque)
    {
        int total_steps = accumulate(phase_steps.begin(), phase_steps.end(), 0);
        graph.add(actuatorGraphs(total_steps));
    }

    // zero torque priors
    auto phases = getPhases(phase_seq, phase_steps);
    graph.add(zeroTorquePriors(phases));

    // add time constraints
    graph.add(timeFactors(phase_seq, phase_steps));

    // prior factor for tank volume
    graph.add(PriorFactor<double>(SourceVolumeKey(), params_.tank_volume, actuators_[0].volume_model));
    return graph;
}

NonlinearFactorGraph JumpingRobot::timeFactors(const std::vector<Phase> &phase_seq, const std::vector<int> phase_steps) const
{
    NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<double>(TimeKey(0), double(0),
                                         graph_builder_.opt().time_cost_model));
    int t = 0;
    for (int phase_idx = 0; phase_idx < phase_seq.size(); phase_idx++)
    {
        gtsam::Double_ dt_expr(PhaseKey(phase_seq[phase_idx]));
        for (int step = 0; step < phase_steps[phase_idx]; step++)
        {
            int next_t = t + 1;
            gtsam::Double_ t_expr(TimeKey(t));
            gtsam::Double_ next_t_expr(TimeKey(next_t));
            gtsam::Double_ expr = t_expr + dt_expr - next_t_expr;
            gtsam::ExpressionFactor<double> factor(graph_builder_.opt().time_cost_model, 0., expr);
            graph.add(factor);
            t = next_t;
        }
    }
    return graph;
}

/** return priors to specify the initial state */
NonlinearFactorGraph JumpingRobot::initialStatePriors() const
{
    NonlinearFactorGraph graph;
    auto joints = robot_ground_.joints();
    for (JointSharedPtr joint : robot_ground_.joints())
    {
        int j = joint->id();
        std::string name = joint->name();
        graph.add(
            gtsam::PriorFactor<double>(JointAngleKey(j, 0), init_angles_.at(name),
                                       graph_builder_.opt().prior_q_cost_model));
        graph.add(
            gtsam::PriorFactor<double>(JointVelKey(j, 0), init_vels_.at(name),
                                       graph_builder_.opt().prior_qv_cost_model));
    }
    for (const auto& actuator: actuators_) {
      int j = actuator.j();
      std::string name =actuator.name();
      graph.add(
            gtsam::PriorFactor<double>(MassKey(j, 0), init_masses_.at(name),
                                actuator.prior_m_cost_model));
    }
    return graph;
}

/** return priors for control variables (init pressure and start time) */
NonlinearFactorGraph JumpingRobot::controlPriors(const double init_tank_pressure, const Vector &open_times, const Vector &close_times) const
{
    NonlinearFactorGraph graph;
    graph.add(PriorFactor<double>(SourcePressureKey(0), init_tank_pressure, actuators_[0].prior_pressure_cost_model));
    for (int actuator_idx = 0; actuator_idx < actuators_.size(); actuator_idx++)
    {
        auto &actuator = actuators_[actuator_idx];
        int j = actuator.j();
        graph.add(gtsam::PriorFactor<double>(ValveOpenTimeKey(j), open_times[actuator_idx],
                                             actuators_[0].prior_valve_t_cost_model));
        graph.add(gtsam::PriorFactor<double>(ValveCloseTimeKey(j), close_times[actuator_idx],
                                             actuators_[0].prior_valve_t_cost_model));
    }
    return graph;
}

void JumpingRobot::exportPoses(double target_height, const int num_steps, const gtsam::Values& results) const
{
  std::ofstream data_file;
  data_file.open("../../visualization/joint_angles/pose_gtsam" + std::to_string(target_height) + ".txt");

  std::vector<Pose3> rel_poses {
    Pose3(Rot3::Rx(-(M_PI_2 - 12.6/180.0*M_PI)), Point3(0, 0, 0)),
    Pose3(Rot3::Rx(-59.1/180.0*M_PI), Point3(0, 0, 0)),
    Pose3(Rot3::Rx(0), Point3(0, 0, 0)),
    Pose3(Rot3::Rx(59.1/180.0*M_PI), Point3(0, 0, 0)),
    Pose3(Rot3::Rx(M_PI_2 - 12.6/180.0*M_PI), Point3(0, 0, 0))
  };

  for (int j = 1; j<=5; j++) {
    for (size_t t =0; t<=num_steps; t++) {
      Pose3 link_pose = results.at<Pose3>(PoseKey(j, t));
      double time = results.atDouble(TimeKey(t));
      data_file << time;
      gtsam::Matrix pose_mat = (link_pose * rel_poses[j-1]).matrix();

      for (int r=0; r<4; r++) {
        for (int c=0; c<4; c++) {
          data_file << ", " << pose_mat(r, c);
        }
      }
      data_file << std::endl;
    }
  }
  data_file.close();


  std::ofstream p_file;
  p_file.open("../../visualization/joint_angles/pressures_gtsam" + std::to_string(target_height) + ".txt");
  for (size_t t =0; t<=num_steps; t++) {
    double time = results.atDouble(TimeKey(t));
    p_file << time;
    for (int j = 1; j<5; j++) {
      double pressure = results.atDouble(PressureKey(j, t));
      p_file << ", " << pressure;
    } 
    p_file << std::endl;
  }
  p_file.close();
}

/** save trajectory to external files */
void JumpingRobot::exportTrajectory(const int num_steps, const double dt, const gtsam::Values &results) const
{
    std::ofstream q, qVel, qAccel, qTorque, data;
    q.open("../../../visualization/joint_angles/q.txt");
    qVel.open("../../../visualization/joint_angles/qVel.txt");
    qAccel.open("../../../visualization/joint_angles/qAccel.txt");
    qTorque.open("../../../visualization/joint_angles/qTorque.txt");
    data.open("../../../visualization/joint_angles/data.txt");

    for (int t = 0; t <= num_steps; t++)
    {
        data << std::setprecision(16) << double(t * dt) << ", 0, 0, ";
        Vector q_t = DynamicsGraph::jointAngles(robot_ground_, results, t);
        Vector v_t = DynamicsGraph::jointVels(robot_ground_, results, t);
        Vector a_t = DynamicsGraph::jointAccels(robot_ground_, results, t);
        Vector torques_t = DynamicsGraph::jointTorques(robot_ground_, results, t);
        for (int j = 0; j < 5; j++)
        {
            if ((j == 2) || (j == 3))
            {
                data << std::setprecision(16) << q_t(j) + M_PI_2 << ", ";
            }
            else
            {
                data << std::setprecision(16) << q_t(j) << ", ";
            }
        }
        data << "0, 0";
        for (int j = 0; j < 5; j++)
        {
            data << std::setprecision(16) << ", " << v_t(j);
        }
        data << std::endl;
        q << std::setprecision(16) << q_t.transpose()
          << std::endl;
        qVel << std::setprecision(16) << v_t.transpose()
             << std::endl;
        qAccel << std::setprecision(16) << a_t.transpose()
               << std::endl;
        qTorque << std::setprecision(16) << torques_t.transpose()
                << std::endl;
    }
    q.close();
    qVel.close();
    qAccel.close();
    qTorque.close();
    data.close();
}

std::vector<double> getAnglesAir(const double angle_offset, const int t, const int right_i, const Robot &robot, const gtsam::Values &results)
{
    std::vector<double> angles;
    gtsam::Pose3 T_wr = results.at<gtsam::Pose3>(PoseKey(right_i, t));
    angles.emplace_back(T_wr.rotation().rpy()[0]);
    std::vector<std::string> joint_names{"j1", "j2", "j3", "j4"};
    for (auto name : joint_names)
    {
        int j = robot.joint(name)->id();
        double angle = results.atDouble(JointAngleKey(j, t));
        if (name == "j2" || name == "j3")
        {
            angle += angle_offset;
        }
        angles.emplace_back(angle);
    }
    return angles;
}

std::vector<double> getVelsAir(const int t, const int right_i, const Robot &robot, const gtsam::Values &results)
{
    std::vector<double> vels;
    gtsam::Pose3 T_wr = results.at<gtsam::Pose3>(PoseKey(right_i, t));
    gtsam::Vector6 V_r = results.at<gtsam::Vector6>(TwistKey(right_i, t));
    vels.emplace_back(V_r[0]);
    std::vector<std::string> joint_names{"j1", "j2", "j3", "j4"};
    for (auto name : joint_names)
    {
        int j = robot.joint(name)->id();
        double vel = results.atDouble(JointVelKey(j, t));
        vels.emplace_back(vel);
    }
    return vels;
}

std::vector<double> JumpingRobot::getActuatorValues(const int t, const std::vector<PneumaticActuator> &actuators, const gtsam::Values &results) const
{
    std::vector<double> data;

    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(MassKey(j, t)));
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(MassRateActualKey(j, t)));
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(VolumeKey(j, t)));
    }

    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(PressureKey(j, t)));
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(ContractionKey(j, t)));
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(0);
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(ForceKey(j, t)));
    }
    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(TorqueKey(j, t)));
    }

    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        double angle = results.atDouble(JointAngleKey(j, t));
        if (actuator.name() == "j2" || actuator.name() == "j3")
        {
            angle += params_.angle_offset;
        }
        data.emplace_back(angle);
    }

    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(JointVelKey(j, t)));
    }

    for (const auto& actuator: actuators)
    {
        int j = actuator.j();
        data.emplace_back(results.atDouble(JointAccelKey(j, t)));
    }

    return data;
}

void JumpingRobot::exportData(const int num_steps, const gtsam::Values &results) const
{
    std::ofstream data_file;
    data_file.open("../../visualization/joint_angles/all_data.csv");
    data_file << "t, mass, , , , mdot, , , , volume, , , , pressure, , , , contraction, , , , spring, , , , force, , , , torque, , , , angle, , , , vel, , , , acc, , , , " << std::endl;
    for (int t = 0; t <= num_steps; t++)
    {
        std::vector<double> data;
        auto actuator_data = getActuatorValues(t, actuators_, results);
        data.insert(data.end(), actuator_data.begin(), actuator_data.end());
        data_file << std::setprecision(16) << results.atDouble(TimeKey(t));
        for (double conf : data)
        {
            data_file << std::setprecision(16) << ", " << conf;
        }
        data_file << std::endl;
    }

    data_file.close();
}

void JumpingRobot::exportTrajectoryMultiPhase(const int num_steps, const std::vector<Phase> &phases, const gtsam::Values &results) const
{
    std::ofstream q, qVel, qAccel, qTorque, data;
    q.open("../../../visualization/joint_angles/q.txt");
    qVel.open("../../../visualization/joint_angles/qVel.txt");
    qAccel.open("../../../visualization/joint_angles/qAccel.txt");
    qTorque.open("../../../visualization/joint_angles/qTorque.txt");
    data.open("../../../visualization/joint_angles/data.txt");

    for (int t = 0; t <= num_steps; t++)
    {
        // std::cout << "t= " << t << "\t phase: " << phases[t] << "\n";
        const Robot &robot = getRobot(phases[t]);
        std::vector<double> config;

        Vector q_t = DynamicsGraph::jointAngles(robot, results, t);
        Vector v_t = DynamicsGraph::jointVels(robot, results, t);
        Vector a_t = DynamicsGraph::jointAccels(robot, results, t);
        Vector torques_t = DynamicsGraph::jointTorques(robot, results, t);

        // time
        data << std::setprecision(16) << double(results.atDouble(TimeKey(t)));
        // end point
        gtsam::Pose3 T_wb = results.at<gtsam::Pose3>(PoseKey(right_i_, t));
        gtsam::Point3 end_point = T_wb * p_end_right_;
        config.emplace_back(end_point.z());
        config.emplace_back(end_point.y());

        // joint angles
        if (phases[t] == Phase::Air || phases[t] == Phase::Left)
        {
            config.emplace_back(T_wb.rotation().rpy()[0]);
        }
        else
        {
            int j = robot.joint("j0")->id();
            config.emplace_back(results.atDouble(JointAngleKey(j, t)));
        }
        std::vector<std::string> joint_names{"j1", "j2", "j3", "j4"};
        for (auto name : joint_names)
        {
            int j = robot.joint(name)->id();
            double angle = results.atDouble(JointAngleKey(j, t));
            if (name == "j2" || name == "j3")
            {
                angle += M_PI_2;
            }
            config.emplace_back(angle);
        }

        // end point vel
        config.emplace_back(0);
        config.emplace_back(0);

        // joint vel
        config.emplace_back(0);
        for (auto name : joint_names)
        {
            int j = robot.joint(name)->id();
            double vel = results.atDouble(JointVelKey(j, t));
            config.emplace_back(vel);
        }

        for (double conf : config)
        {
            data << std::setprecision(16) << ", " << conf;
        }
        data << std::endl;
        q << std::setprecision(16) << q_t.transpose()
          << std::endl;
        qVel << std::setprecision(16) << v_t.transpose()
             << std::endl;
        qAccel << std::setprecision(16) << a_t.transpose()
               << std::endl;
        qTorque << std::setprecision(16) << torques_t.transpose()
                << std::endl;
    }
    q.close();
    qVel.close();
    qAccel.close();
    qTorque.close();
    data.close();
}


void JRSimulator::refineAllJointsGround(JointValueMap& qs, JointValueMap& vs) const {
    const Robot& robot = jr_.getRobot(JumpingRobot::Phase::Ground);
    NonlinearFactorGraph graph;
    int t = 0;
    graph.add(jr_.getGraphBuilder().qFactors(robot, t));
    graph.add(jr_.getGraphBuilder().vFactors(robot, t));

    std::vector<std::string> joint_names {"j2", "j3"};
    for (auto name: joint_names) {
        int j = robot.joint(name)->id();
        graph.add(gtsam::PriorFactor<double>(JointAngleKey(j, t), qs[name],
                                         jr_.getGraphBuilder().opt().prior_q_cost_model));
        graph.add(gtsam::PriorFactor<double>(JointVelKey(j, t), vs[name],
                                        jr_.getGraphBuilder().opt().prior_qv_cost_model));
    }

    Values values;
    for (auto joint: robot.joints()) {
        int j = joint->id();
        values.insert(JointAngleKey(j, t), qs[joint->name()]);
        values.insert(JointVelKey(j, t), vs[joint->name()]);
    }

    for (auto link: robot.links()) {
        int i = link->id();
        gtsam::Key prev_pose_key = PoseKey(i, t_);
        gtsam::Key prev_twist_key = TwistKey(i, t_);
        if (results_.exists(prev_pose_key)) {
            values.insert(PoseKey(i, t), results_.at(prev_pose_key));
        }
        else {
            values.insert(PoseKey(i, t), link->wTcom());
        }
        if (results_.exists(prev_twist_key)) {
            values.insert(TwistKey(i, t), results_.at(prev_twist_key));
        }
        else {
            values.insert(TwistKey(i, t), (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        }
    }

    // graph.print("", GTDKeyFormatter);
    // values.print("", GTDKeyFormatter);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();
    if (graph.error(result) > 1e-5) {
        throw std::runtime_error("cannot refine for ground phase\n");
    }
    qs = DynamicsGraph::jointAnglesMap(robot, result, t);
    vs = DynamicsGraph::jointVelsMap(robot, result, t);
}


void JRSimulator::step(double dt, const JointValueMap &torques)
{
    // std::cout << "robot dynamics\n";

    // std::cout << "torques: ";
    // for (const auto& it:torques) {
    //     std::cout << it.second << "\t";
    // }
    // std::cout << "\n";
    // std::cout << "qs:      ";
    // for (const auto& it:qs_) {
    //     std::cout << it.second << "\t";
    // }
    // std::cout << "\n";
    // std::cout << "vs:      ";
    // for (const auto& it:vs_) {
    //     std::cout << it.second << "\t";
    // }
    // std::cout << "\n";

    // std::cout << "current phase: " << phase_ << "\n";
    //// perform forward dynamics
    const Robot &robot = jr_.getRobot(phase_);
    Values known_values, fk_results;

    // std::cout << "adding qs\n";
    for (JointSharedPtr joint : robot.joints()) {
        const auto& joint_name = joint->name();
        const auto& q = qs_.at(joint_name);
        int j = joint->id();
        InsertJointAngle(&known_values, j, t_, q);
    }
    // std::cout << "adding vs\n";
    for (JointSharedPtr joint : robot.joints()) {
        const auto& joint_name = joint->name();
        const auto& v = vs_.at(joint_name);
        int j = joint->id();
        InsertJointVel(&known_values, j, t_, v);
    }

    // std::cout << "fk\n";
    if (phase_ == JumpingRobot::Phase::Air)
    {
        std::string prior_link_name = "l3";
        int i = robot.link(prior_link_name)->id();
        InsertPose(&known_values, i, t_, body_pose_);
        InsertTwist(&known_values, i, t_, body_twist_);
        fk_results = robot.forwardKinematics(known_values, t_, prior_link_name);
    }
    else
    {
        fk_results = robot.forwardKinematics(known_values, t_);
    }

    // for (JointSharedPtr joint : robot.joints()) {
    //     int j = joint->id();
    //     auto name = joint->name();
    //     auto v_key = JointVelKey(j, t_);
    //     auto q_key = JointAngleKey(j, t_);
    //     std::cout << "\t" << name << "\t" << fk_results.atDouble(q_key) << "\t" << fk_results.atDouble(v_key) <<
    //     "\t" <<  torques.at(name) << "\n";
    // }


    // for (LinkSharedPtr link : robot.links()) {
    //     int i = link->id();
    //     auto name = link->name();
    //     auto pose_key = PoseKey(i, t_);
    //     auto twist_key = TwistKey(i, t_);
    //     std::cout << "\t" << name << "\t" << fk_results.at<gtsam::Pose3>(pose_key).translation().transpose() << "\t" 
    //               << fk_results.at<Vector6>(twist_key).transpose() << "\n";
    // }

    // std::string prior_link_name = "l3";
    // int i = robot.link(prior_link_name)->id();
    // auto pose_key = PoseKey(i, t_);
    // auto pose = fk_results.at<gtsam::Pose3>(pose_key);
    // std::cout << "torso: " << pose.translation().transpose() << "\n";



    // std::cout << "adding torques\n";
    for (JointSharedPtr joint : robot.joints()) {
        const auto& joint_name = joint->name();
        const auto& torque = torques.at(joint_name);
        int j = joint->id();
        InsertTorque(&fk_results, j, t_, torque);
    }

    // std::cout << "linear solve fd\n";
    Values dynamics_values = jr_.graph_builder_.linearSolveFD(robot, t_, fk_results);

    // extract accelerations
    as_ = DynamicsGraph::jointAccelsMap(robot, dynamics_values, t_);
    body_twist_accel_ = dynamics_values.at<Vector6>(TwistAccelKey(jr_.body_i_, t_));

    //// store values
    phases_.emplace_back(phase_);
    if (!results_.exists(TimeKey(t_)))
    {
        results_.insert(TimeKey(t_), time_);
    }
    merge(results_, dynamics_values);

    //// check phase transition
    JumpingRobot::Phase new_phase = jr_.phaseTransition(phase_, t_, dynamics_values);
    if (new_phase == JumpingRobot::Phase::Air && phase_ != JumpingRobot::Phase::Air)
    {
        body_pose_ = dynamics_values.at<Pose3>(PoseKey(jr_.body_i_, t_));
        body_twist_ = dynamics_values.at<Vector6>(TwistKey(jr_.body_i_, t_));
    }
    // std::cout << "new_phase: " << new_phase << "\n";

    //// integration
    JointValueMap vs_new, qs_new;
    for (JointSharedPtr joint : robot.joints())
    {
        std::string name = joint->name();
        vs_new[name] = vs_.at(name) + dt * as_.at(name);
        qs_new[name] = qs_.at(name) + dt * vs_.at(name); // + 0.5 * as_.at(name) * std::pow(dt, 2);
    }
    vs_ = vs_new;
    qs_ = qs_new;
    if (new_phase == JumpingRobot::Phase::Ground) {
        refineAllJointsGround(qs_, vs_);
    }

    if (phase_ == JumpingRobot::Phase::Air)
    {
        // std::cout << "body_twist: " << body_twist_.transpose() << "\n";
        // std::cout << "body_pose: \n" << body_pose_ << "\n";
        Vector6 body_twist_new = (body_twist_ + dt * body_twist_accel_).eval();
        Pose3 body_pose_new = body_pose_ * Pose3::Expmap(dt * body_twist_);// + 0.5 * body_twist_accel_ * std::pow(dt, 2)); // check this equation
        body_twist_ = body_twist_new;
        body_pose_ = body_pose_new;
    }

    //// prepare for next time step
    phase_ = new_phase;
    time_ += dt;
    t_ += 1;
}

void JRSimulator::step(double dt, const Vector &open_times, const Vector &close_times)
{
    std::cout << "t= " << t_ << "\n";
    // std::cout << "step actuators\n";
    //// run actuators
    Values actuator_values = jr_.runActuators(t_, time_, qs_, vs_, ms_, m_t_, open_times, close_times, results_);
    JointValueMap torques = jr_.extractTorques(phase_, t_, actuator_values);
    JointValueMap mdots = jr_.extractMdots(phase_, t_, actuator_values);
    merge(results_, actuator_values);

    //// integration on mass
    JointValueMap ms_new;
    size_t idx = 0;
    for (auto it = ms_.begin(); it!=ms_.end(); ++it)
    {
        std::string name = it->first;
        ms_new[name] = ms_.at(name) + dt * mdots.at(name);
        m_t_ -= dt * mdots.at(name);
        idx++;
    }
    ms_ = ms_new;

    //// compute kinodynamics
    step(dt, torques);
}

} // namespace gtdynamics