/**
 * @file  DynamicsGraph.cpp
 * @brief dynamics factor graph
 * @Author: Yetong Zhang
 */

#include "DynamicsGraph.h"

using namespace std;
using namespace gtsam;

using namespace std;
using namespace gtsam;

namespace robot
{

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::dynamicsFactorGraph(
    const UniversalRobot &robot, const int t,
    const boost::optional<gtsam::Vector3> &gravity,
    const boost::optional<gtsam::Vector3> &plannar_axis,
    const boost::optional<std::vector<uint>> &contacts) const
{

    NonlinearFactorGraph graph;

    std::vector<uint> contacts_;
    if (contacts)
        contacts_ = contacts.get();
    else
        contacts_ = std::vector<uint>(robot.numLinks(), 0);

    // add factors corresponding to links
    // for (auto &&link : robot.links())
    for (int idx = 0; idx < robot.numLinks(); idx++)
    {
        auto link = robot.links()[idx];

        int i = link->getID();
        if (link->isFixed())
        {
            graph.add(PriorFactor<Pose3>(PoseKey(i, t), link->getFixedPose(),
                                         noiseModel::Constrained::All(6)));
            graph.add(PriorFactor<Vector6>(TwistKey(i, t), Vector6::Zero(),
                                           noiseModel::Constrained::All(6)));
            graph.add(PriorFactor<Vector6>(TwistAccelKey(i, t), Vector6::Zero(),
                                           noiseModel::Constrained::All(6)));
        }
        else
        {
            // Get all wrench keys associated with this link.
            const auto &connected_joints = link->getJoints();
            std::vector<gtsam::LabeledSymbol> wrenches;
            for (auto &&joint : connected_joints)
                wrenches.push_back(WrenchKey(i, joint->getID(), t));
            if (contacts_[idx])
                wrenches.push_back(ContactWrenchKey(i, t));

            // Create wrench factor for this link.
            if (wrenches.size() == 0)
                graph.add(WrenchFactor0(TwistKey(i, t), TwistAccelKey(i, t),
                                        PoseKey(i, t), opt_.f_cost_model,
                                        link->inertiaMatrix(), gravity));
            else if (wrenches.size() == 1)
                graph.add(WrenchFactor1(TwistKey(i, t), TwistAccelKey(i, t),
                                        wrenches[0],
                                        PoseKey(i, t), opt_.f_cost_model,
                                        link->inertiaMatrix(), gravity));
            else if (wrenches.size() == 2)
                graph.add(WrenchFactor2(TwistKey(i, t), TwistAccelKey(i, t),
                                        wrenches[0], wrenches[1],
                                        PoseKey(i, t), opt_.f_cost_model,
                                        link->inertiaMatrix(), gravity));
            else if (wrenches.size() == 3)
                graph.add(WrenchFactor3(TwistKey(i, t), TwistAccelKey(i, t),
                                        wrenches[0], wrenches[1], wrenches[2],
                                        PoseKey(i, t), opt_.f_cost_model,
                                        link->inertiaMatrix(), gravity));
            else if (wrenches.size() == 4)
                graph.add(WrenchFactor4(TwistKey(i, t), TwistAccelKey(i, t),
                                        wrenches[0], wrenches[1],
                                        wrenches[2], wrenches[3],
                                        PoseKey(i, t), opt_.f_cost_model,
                                        link->inertiaMatrix(), gravity));
            else
                throw std::runtime_error("Wrench factor not defined");

            // Enforce contact kinematics and contact dynamics for link in
            // contact.
            if (contacts_[idx]) {
                ContactKinematicsTwistFactor(TwistKey(i, t),
                    gtsam::noiseModel::Constrained::All(3),
                    link->leTl_com());
                ContactKinematicsAccelFactor(TwistAccelKey(i, t),
                    gtsam::noiseModel::Constrained::All(3),
                    link->leTl_com());
                ContactKinematicsPoseFactor(PoseKey(i, t),
                    gtsam::noiseModel::Constrained::All(1),
                    link->leTl_com(),
                    (gtsam::Vector(3) << 0, 0, -9.8).finished());
                ContactDynamicsMomentFactor(ContactWrenchKey(i, t),
                    gtsam::noiseModel::Constrained::All(3),
                    link->leTl_com());
            }
        }
    }

    // add factors corresponding to joints
    for (auto &&joint : robot.joints())
    {
        const auto &link_1 = joint->parentLink();
        const auto &link_2 = joint->childLink().lock();
        int i1 = link_1->getID();
        int i2 = link_2->getID(); // cannot use methods for a weak ptr?
        int j = joint->getID();
        // add pose factor
        graph.add(manipulator::PoseFactor(PoseKey(i1, t), PoseKey(i2, t),
                                          JointAngleKey(j, t), opt_.p_cost_model,
                                          joint->McpCom(), joint->screwAxis()));

        // add twist factor
        graph.add(manipulator::TwistFactor(TwistKey(i1, t), TwistKey(i2, t),
                                           JointAngleKey(j, t), JointVelKey(j, t),
                                           opt_.v_cost_model, joint->McpCom(),
                                           joint->screwAxis()));

        // add twist acceleration factor
        graph.add(manipulator::TwistAccelFactor(
            TwistKey(i2, t), TwistAccelKey(i1, t), TwistAccelKey(i2, t),
            JointAngleKey(j, t), JointVelKey(j, t), JointAccelKey(j, t),
            opt_.a_cost_model, joint->McpCom(), joint->screwAxis()));

        // add wrench equivalence factor
        if (!link_1->isFixed() && !link_2->isFixed())
        {
            graph.add(WrenchEquivalenceFactor(
                WrenchKey(i1, j, t), WrenchKey(i2, j, t), JointAngleKey(j, t),
                opt_.f_cost_model, joint->McpCom(), joint->screwAxis()));
        }

        // add torque factor
        graph.add(manipulator::TorqueFactor(WrenchKey(i2, j, t), TorqueKey(j, t),
                                            opt_.t_cost_model,
                                            joint->screwAxis()));

        // add planar wrench factor
        if (plannar_axis)
        {
            graph.add(WrenchPlanarFactor(WrenchKey(i2, j, t),
                                         gtsam::noiseModel::Constrained::All(3),
                                         *plannar_axis));
        }
    }
    return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::trajectoryFG(
    const UniversalRobot &robot, const int num_steps, const double dt,
    const DynamicsGraphBuilder::CollocationScheme collocation,
    const boost::optional<gtsam::Vector3> &gravity,
    const boost::optional<gtsam::Vector3> &plannar_axis) const
{
    NonlinearFactorGraph graph;
    for (int t = 0; t < num_steps + 1; t++)
    {
        graph.add(dynamicsFactorGraph(robot, t, gravity, plannar_axis));
        if (t < num_steps)
        {
            graph.add(collocationFactors(robot, t, dt, collocation));
        }
    }
    return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::multiPhaseTrajectoryFG(
    const UniversalRobot &robot, const std::vector<int> &phase_steps,
    const DynamicsGraphBuilder::CollocationScheme collocation,
    const boost::optional<gtsam::Vector3> &gravity,
    const boost::optional<gtsam::Vector3> &plannar_axis) const
{
    NonlinearFactorGraph graph;
    int t = 0;
    graph.add(dynamicsFactorGraph(robot, t, gravity, plannar_axis));

    for (uint phase = 0; phase < phase_steps.size(); phase++) {
        for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
            graph.add(multiPhaseCollocationFactors(robot, t, phase, collocation));
            t++;
            graph.add(dynamicsFactorGraph(robot, t, gravity, plannar_axis));
        }
    }
    return graph;
}

gtsam::ExpressionFactorGraph DynamicsGraphBuilder::collocationFactors(
    const UniversalRobot &robot, const int t, const double dt,
    const CollocationScheme collocation) const
{
    ExpressionFactorGraph graph;
    for (auto &&joint : robot.joints())
    {
        int j = joint->getID();
        Double_ q0_expr = Double_(JointAngleKey(j, t));
        Double_ q1_expr = Double_(JointAngleKey(j, t + 1));
        Double_ v0_expr = Double_(JointVelKey(j, t));
        Double_ v1_expr = Double_(JointVelKey(j, t + 1));
        Double_ a0_expr = Double_(JointAccelKey(j, t));
        Double_ a1_expr = Double_(JointAccelKey(j, t + 1));
        switch (collocation)
        {
        case CollocationScheme::Euler:
            graph.addExpressionFactor(q0_expr + dt * v0_expr - q1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            graph.addExpressionFactor(v0_expr + dt * a0_expr - v1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            break;
        case CollocationScheme::Trapezoidal:
            graph.addExpressionFactor(q0_expr + 0.5 * dt * v0_expr + 0.5 * dt * v1_expr - q1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            graph.addExpressionFactor(v0_expr + 0.5 * dt * a0_expr + 0.5 * dt * a1_expr - v1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            break;
        default:
            throw std::runtime_error("runge-kutta and hermite-simpson not implemented yet");
            break;
        }
    }
    return graph;
}

// the * operator for doubles in expression factor does not work well yet
double multDouble(const double& d1, const double& d2, OptionalJacobian<1, 1> H1,
                  OptionalJacobian<1, 1> H2) {
  if (H1)
    *H1 = I_1x1 * d2;
  if (H2)
    *H2 = I_1x1 * d1;
  return d1 * d2;
}

gtsam::ExpressionFactorGraph DynamicsGraphBuilder::multiPhaseCollocationFactors(const UniversalRobot &robot,
                                                                            const int t, const int phase,
                                                                            const CollocationScheme collocation) const
{
    ExpressionFactorGraph graph;
    Double_ phase_expr = Double_(PhaseKey(phase));
    for (auto &&joint : robot.joints())
    {
        int j = joint->getID();
        Double_ q0_expr = Double_(JointAngleKey(j, t));
        Double_ q1_expr = Double_(JointAngleKey(j, t + 1));
        Double_ v0_expr = Double_(JointVelKey(j, t));
        Double_ v1_expr = Double_(JointVelKey(j, t + 1));
        Double_ a0_expr = Double_(JointAccelKey(j, t));
        Double_ a1_expr = Double_(JointAccelKey(j, t + 1));

        if (collocation == CollocationScheme::Euler) {
            Double_ v0dt(multDouble, phase_expr, v0_expr);
            Double_ a0dt(multDouble, phase_expr, a0_expr);
            graph.addExpressionFactor(q0_expr + v0dt - q1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            graph.addExpressionFactor(v0_expr + a0dt - v1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
        }
        else if (collocation == CollocationScheme::Trapezoidal) {
            Double_ v0dt(multDouble, phase_expr, v0_expr);
            Double_ a0dt(multDouble, phase_expr, a0_expr);
            Double_ v1dt(multDouble, phase_expr, v1_expr);
            Double_ a1dt(multDouble, phase_expr, a1_expr);
            graph.addExpressionFactor(q0_expr + 0.5 * v0dt + 0.5 * v1dt - q1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
            graph.addExpressionFactor(v0_expr + 0.5 * a0dt + 0.5 * a1dt - v1_expr, double(0), gtsam::noiseModel::Constrained::All(1));
        }
        else {
            throw std::runtime_error("runge-kutta and hermite-simpson not implemented yet");
        }

    }
    return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::jointLimitFactors(const UniversalRobot &robot,
                                                                    const int t) const
{

    gtsam::NonlinearFactorGraph graph;
    for (auto &&joint : robot.joints())
    {
        int j = joint->getID();
        // Add joint angle limit factor.
        graph.add(manipulator::JointLimitFactor(
            gtsam::LabeledSymbol('q', j, t), opt_.jl_cost_model,
            joint->jointLowerLimit(), joint->jointUpperLimit(),
            joint->jointLimitThreshold()));

        // Add joint velocity limit factors.
        graph.add(manipulator::JointLimitFactor(
            gtsam::LabeledSymbol('v', j, t), opt_.jl_cost_model,
            -joint->velocityLimit(), joint->velocityLimit(),
            joint->velocityLimitThreshold()));

        // Add joint acceleration limit factors.
        graph.add(manipulator::JointLimitFactor(
            gtsam::LabeledSymbol('a', j, t), opt_.jl_cost_model,
            -joint->accelerationLimit(), joint->accelerationLimit(),
            joint->accelerationLimitThreshold()));

        // Add joint torque limit factors.
        graph.add(manipulator::JointLimitFactor(
            gtsam::LabeledSymbol('T', j, t), opt_.jl_cost_model,
            -joint->torqueLimit(), joint->torqueLimit(),
            joint->torqueLimitThreshold()));
    }
    return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraphBuilder::forwardDynamicsPriors(const UniversalRobot &robot, const int t,
                                            const gtsam::Vector &joint_angles,
                                            const gtsam::Vector &joint_vels,
                                            const gtsam::Vector &torques) const
{
    gtsam::NonlinearFactorGraph graph;
    auto joints = robot.joints();
    for (int idx = 0; idx < robot.numJoints(); idx++)
    {
        auto joint = joints[idx];
        int j = joint->getID();
        graph.add(
            gtsam::PriorFactor<double>(JointAngleKey(j, t), joint_angles[idx],
                                       gtsam::noiseModel::Constrained::All(1)));
        graph.add(
            gtsam::PriorFactor<double>(JointVelKey(j, t), joint_vels[idx],
                                       gtsam::noiseModel::Constrained::All(1)));
        graph.add(
            gtsam::PriorFactor<double>(TorqueKey(j, t), torques[idx],
                                       gtsam::noiseModel::Constrained::All(1)));
    }
    return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraphBuilder::trajectoryFDPriors(const UniversalRobot &robot, const int num_steps,
                                         const gtsam::Vector &joint_angles,
                                         const gtsam::Vector &joint_vels,
                                         const std::vector<gtsam::Vector> &torques_seq) const
{
    gtsam::NonlinearFactorGraph graph;
    auto joints = robot.joints();
    for (int idx = 0; idx < robot.numJoints(); idx++)
    {
        int j = joints[idx]->getID();
        graph.add(
            gtsam::PriorFactor<double>(JointAngleKey(j, 0), joint_angles[idx],
                                       gtsam::noiseModel::Constrained::All(1)));
        graph.add(
            gtsam::PriorFactor<double>(JointVelKey(j, 0), joint_vels[idx],
                                       gtsam::noiseModel::Constrained::All(1)));
    }
    for (int t = 0; t <= num_steps; t++)
    {
        for (int idx = 0; idx < robot.numJoints(); idx++)
        {
            int j = joints[idx]->getID();
            graph.add(
                gtsam::PriorFactor<double>(TorqueKey(j, t), torques_seq[t][idx],
                                           gtsam::noiseModel::Constrained::All(1)));
        }
    }

    return graph;
}

gtsam::Vector DynamicsGraphBuilder::jointAccels(const UniversalRobot &robot,
                                                const gtsam::Values &result, const int t)
{
    gtsam::Vector joint_accels = gtsam::Vector::Zero(robot.numJoints());
    auto joints = robot.joints();
    for (int idx = 0; idx < robot.numJoints(); idx++)
    {
        auto joint = joints[idx];
        int j = joint->getID();
        joint_accels[idx] = result.atDouble(JointAccelKey(j, t));
    }
    return joint_accels;
}

gtsam::Vector DynamicsGraphBuilder::jointVels(const UniversalRobot &robot,
                                              const gtsam::Values &result, const int t)
{
    gtsam::Vector joint_vels = gtsam::Vector::Zero(robot.numJoints());
    auto joints = robot.joints();
    for (int idx = 0; idx < robot.numJoints(); idx++)
    {
        auto joint = joints[idx];
        int j = joint->getID();
        joint_vels[idx] = result.atDouble(JointVelKey(j, t));
    }
    return joint_vels;
}

gtsam::Vector DynamicsGraphBuilder::jointAngles(const UniversalRobot &robot,
                                                const gtsam::Values &result, const int t)
{
    gtsam::Vector joint_angles = gtsam::Vector::Zero(robot.numJoints());
    auto joints = robot.joints();
    for (int idx = 0; idx < robot.numJoints(); idx++)
    {
        auto joint = joints[idx];
        int j = joint->getID();
        joint_angles[idx] = result.atDouble(JointAngleKey(j, t));
    }
    return joint_angles;
}

gtsam::Values DynamicsGraphBuilder::zeroValues(const UniversalRobot &robot, const int t)
{
    gtsam::Vector zero_twists = gtsam::Vector6::Zero(),
                  zero_accels = gtsam::Vector6::Zero(),
                  zero_wrenches = gtsam::Vector6::Zero(),
                  zero_torque = gtsam::Vector1::Zero(),
                  zero_q = gtsam::Vector1::Zero(),
                  zero_v = gtsam::Vector1::Zero(),
                  zero_a = gtsam::Vector1::Zero();
    gtsam::Values zero_values;
    for (auto &link : robot.links())
    {
        int i = link->getID();
        zero_values.insert(PoseKey(i, t), link->getComPose());
        zero_values.insert(TwistKey(i, t), zero_twists);
        zero_values.insert(TwistAccelKey(i, t), zero_accels);
    }
    for (auto &joint : robot.joints())
    {
        int j = joint->getID();
        auto parent_link = joint->parentLink();
        auto child_link = joint->childLink().lock();
        if (!parent_link->isFixed())
        {
            zero_values.insert(WrenchKey(parent_link->getID(), j, t),
                               zero_wrenches);
        }
        if (!child_link->isFixed())
        {
            zero_values.insert(WrenchKey(child_link->getID(), j, t), zero_wrenches);
        }
        zero_values.insert(TorqueKey(j, t), zero_torque[0]);
        zero_values.insert(JointAngleKey(j, t), zero_q[0]);
        zero_values.insert(JointVelKey(j, t), zero_v[0]);
        zero_values.insert(JointAccelKey(j, t), zero_a[0]);
    }
    return zero_values;
}

gtsam::Values DynamicsGraphBuilder::zeroValuesTrajectory(const UniversalRobot &robot, const int num_steps, const int num_phases)
{
    Values zero_values;
    for (int t = 0; t <= num_steps; t++)
    {
        zero_values.insert(zeroValues(robot, t));
    }
    if (num_phases > 0) {
        for (int phase =0; phase <= num_phases; phase++) {
            zero_values.insert(PhaseKey(phase), double(0));
        }
    }
    return zero_values;
}

gtsam::Values DynamicsGraphBuilder::optimize(
    const gtsam::NonlinearFactorGraph &graph,
    const gtsam::Values &init_values, OptimizerType optim_type)
{
    if (optim_type == OptimizerType::GaussNewton) {
        GaussNewtonOptimizer optimizer(graph, init_values);
        optimizer.optimize();
        return optimizer.values();
    }
    else if (optim_type == OptimizerType::LM) {
        LevenbergMarquardtOptimizer optimizer(graph, init_values);
        optimizer.optimize();
        return optimizer.values();
    }
    else if (optim_type == OptimizerType::PDL) {
        DoglegOptimizer optimizer(graph, init_values);
        optimizer.optimize();
        return optimizer.values();
    }
    else {
        throw std::runtime_error("optimizer not implemented yet");
    }
}

void print_key(const gtsam::Key &key)
{
    auto symb = gtsam::LabeledSymbol(key);
    char ch = symb.chr();
    int index = symb.label();
    int t = symb.index();
    if (ch == 'F')
    {
        std::cout << ch << int(index / 16) << index % 16 << "_" << t;
    }
    else if (ch == 't')
    {
        if (index == 0) { // phase key
            std::cout << "dt" << t;
        }
        else if (index == 1) { // time key
            std::cout << "t" << t;
        }
        else { // time to open valve
            std::cout << "ti" << t;
        }
    }
    else
    {
        std::cout << ch << index << "_" << t;
    }
    std::cout << "\t";
}

// print the factors of the factor graph
void DynamicsGraphBuilder::print_values(const gtsam::Values &values)
{
    for (auto &key : values.keys())
    {
        print_key(key);
        std::cout << "\n";
        // values.at(key).print();
        std::cout << "\n";
    }
}

// print the factors of the factor graph
void DynamicsGraphBuilder::print_graph(const gtsam::NonlinearFactorGraph &graph)
{
    for (auto &factor : graph)
    {
        for (auto &key : factor->keys())
        {
            print_key(key);
        }
        std::cout << "\n";
    }
}

// using radial location to locate the variables
gtsam::Vector3 radial_location(double r, double i, int n)
{
    double theta = M_PI * 2 / n * i;
    double x = r * cos(theta);
    double y = r * sin(theta);
    return (gtsam::Vector(3) << x, y, 0).finished();
}

// using radial location to locate the variables
gtsam::Vector3 corner_location(double r, double j, int n)
{
    double theta = M_PI * 2 / n * (j + 0.5);
    double x = r * cos(theta);
    double y = r * sin(theta);
    return (gtsam::Vector(3) << x, y, 0).finished();
}

void DynamicsGraphBuilder::saveGraph(const std::string &file_path,
                                     const gtsam::NonlinearFactorGraph &graph,
                                     const gtsam::Values &values,
                                     const UniversalRobot &robot, const int t,
                                     bool radial)
{
    std::ofstream json_file;
    json_file.open(file_path);

    gtsam::JsonSaver::LocationType locations;

    if (radial)
    {
        int n = robot.numLinks();
        for (auto &link : robot.links())
        {
            int i = link->getID();
            locations[PoseKey(i, t)] = radial_location(2, i, n);
            locations[TwistKey(i, t)] = radial_location(3, i, n);
            locations[TwistAccelKey(i, t)] = radial_location(4, i, n);
        }

        for (auto &joint : robot.joints())
        {
            int j = joint->getID();
            locations[JointAngleKey(j, t)] = corner_location(2.5, j, n);
            locations[JointVelKey(j, t)] = corner_location(3.5, j, n);
            locations[JointAccelKey(j, t)] = corner_location(4.5, j, n);
            locations[TorqueKey(j, t)] = corner_location(6, j, n);
            int i1 = joint->parentLink()->getID();
            int i2 = joint->childLink().lock()->getID();
            locations[WrenchKey(i1, j, t)] = corner_location(5.5, j - 0.25, n);
            locations[WrenchKey(i2, j, t)] = corner_location(5.5, j + 0.25, n);
        }
    }
    else
    {
        for (auto &link : robot.links())
        {
            int i = link->getID();
            locations[PoseKey(i, t)] = (gtsam::Vector(3) << i, 0, 0).finished();
            locations[TwistKey(i, t)] = (gtsam::Vector(3) << i, 1, 0).finished();
            locations[TwistAccelKey(i, t)] =
                (gtsam::Vector(3) << i, 2, 0).finished();
        }

        for (auto &joint : robot.joints())
        {
            int j = joint->getID();
            locations[JointAngleKey(j, t)] =
                (gtsam::Vector(3) << j + 0.5, 0.5, 0).finished();
            locations[JointVelKey(j, t)] =
                (gtsam::Vector(3) << j + 0.5, 1.5, 0).finished();
            locations[JointAccelKey(j, t)] =
                (gtsam::Vector(3) << j + 0.5, 2.5, 0).finished();
            int i1 = joint->parentLink()->getID();
            int i2 = joint->childLink().lock()->getID();
            locations[WrenchKey(i1, j, t)] =
                (gtsam::Vector(3) << j + 0.25, 3.5, 0).finished();
            locations[WrenchKey(i2, j, t)] =
                (gtsam::Vector(3) << j + 0.75, 3.5, 0).finished();
            locations[TorqueKey(j, t)] =
                (gtsam::Vector(3) << j + 0.5, 4.5, 0).finished();
        }
    }

    gtsam::JsonSaver::SaveFactorGraph(graph, json_file, values, locations);
    json_file.close();
}

} // namespace robot