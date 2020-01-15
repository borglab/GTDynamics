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

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::integrationFactors(const UniversalRobot &robot,
                                                                     const int t, const double dt)
{
    gtsam::NonlinearFactorGraph graph;
    for (auto &&joint : robot.joints())
    {
        int j = joint->getID();
        graph.add(IntegrationFactor(JointAngleKey(j, t), JointVelKey(j, t),
                                    JointAccelKey(j, t), JointAngleKey(j, t + 1),
                                    JointVelKey(j, t + 1),
                                    gtsam::noiseModel::Constrained::All(2), dt));
    }
    return graph;
}

gtsam::NonlinearFactorGraph
DynamicsGraphBuilder::softIntegrationFactors(const UniversalRobot &robot, const int t)
{
    gtsam::NonlinearFactorGraph graph;
    for (auto &&joint : robot.joints())
    {
        int j = joint->getID();
        graph.add(SoftIntegrationFactor(
            JointAngleKey(j, t), JointVelKey(j, t), JointAccelKey(j, t),
            JointAngleKey(j, t + 1), JointVelKey(j, t + 1), TimeKey(t),
            gtsam::noiseModel::Constrained::All(2)));
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
    gtsam::Values init_values;
    for (auto &link : robot.links())
    {
        int i = link->getID();
        init_values.insert(PoseKey(i, t), link->getComPose());
        init_values.insert(TwistKey(i, t), zero_twists);
        init_values.insert(TwistAccelKey(i, t), zero_accels);
    }
    for (auto &joint : robot.joints())
    {
        int j = joint->getID();
        auto parent_link = joint->parentLink();
        auto child_link = joint->childLink().lock();
        if (!parent_link->isFixed())
        {
            init_values.insert(WrenchKey(parent_link->getID(), j, t),
                               zero_wrenches);
        }
        if (!child_link->isFixed())
        {
            init_values.insert(WrenchKey(child_link->getID(), j, t), zero_wrenches);
        }
        init_values.insert(TorqueKey(j, t), zero_torque[0]);
        init_values.insert(JointAngleKey(j, t), zero_q[0]);
        init_values.insert(JointVelKey(j, t), zero_v[0]);
        init_values.insert(JointAccelKey(j, t), zero_a[0]);
    }
    return init_values;
}

void print_key(const gtsam::Key& key) {
    auto symb = gtsam::LabeledSymbol(key);
    char ch = symb.chr();
    int index = symb.label();
    int t = symb.index();
    if (ch == 'F')
    {
        std::cout << ch << int(index / 16) << index % 16 << "_" << t << "\t";
    }
    else
    {
        std::cout << ch << index << "_" << t << "\t";
    }
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