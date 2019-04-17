/**
 * @file  link.cpp
 * @brief manipulator link
 * @Author: Frank Dellaert and Mandy Xie
 */

#include <Link.h>

using namespace std;
using namespace gtsam;

namespace manipulator {

/* Shorthand for T_j, for twist accelerations on the j-th link. */
Symbol T(int j) {
    return Symbol('T', j);
}

/* Shorthand for a_j, for joint accelerations on the j-th link. */
Symbol a(int j) {
    return Symbol('a', j);
}

/* Shorthand for F_j, for wrenches on the j-th link. */
Symbol F(int j) {
    return Symbol('F', j);
}

/* Shorthand for t_j, for torque on the j-th link. */
Symbol t(int j) {
    return Symbol('t', j);
}

/* Shorthand for V_j, for 6D link twist vector on the j-th link. */
Symbol V(int j) {
    return Symbol('V', j);
}

JacobianFactor Link::twistFactor(int j, const Pose3 &jTi, double joint_vel_j) const {
    Vector6 A_j = screwAxis_; //joint axis expressed in COM frame
    Vector6 joint_twist = A_j * joint_vel_j;

    if (j == 1) {
        return JacobianFactor(V(j), Matrix::Identity(6, 6), joint_twist, noiseModel::Constrained::All(6));
    } else {
        // Equation 8.45 in MR, page 292
        // V(j) - np.dot(jTi.AdjointMap(), V(j-1)) == joint_twist
        return JacobianFactor(V(j), Matrix::Identity(6, 6), V(j-1), -jTi.AdjointMap(), joint_twist, noiseModel::Constrained::All(6));
    }
}

JacobianFactor Link::wrenchFactor(int j, const Vector6 &twist_j, const Pose3 &kTj, 
        boost::optional<Vector3&> gravity) const {
    // Wrench on this link is due to acceleration and reaction to next link.
    // We need inertia, coriolis forces, and an Adjoint map: 
    Matrix6 ad_j = Pose3::adjointMap(twist_j);     
    Matrix6 G_j = inertiaMatrix();
    Vector6 rhs = ad_j.transpose() * G_j * twist_j;
    
    if (gravity) {
        Vector gravitational_force = *gravity * mass();
        for (int i = 3; i < 6; ++i) {
            rhs[i] += gravitational_force[i-3];
        }
    }

    auto jAk = kTj.AdjointMap().transpose();

    // Given the above Equation 8.48 can be written as
    // G_j * T(j) - F(j) + jAk * F(j + 1) == coriolis_j [+ gravity] 
    return  JacobianFactor(T(j), G_j,
                           F(j), -Matrix::Identity(6, 6),
                           F(j + 1), jAk,
                           rhs, noiseModel::Constrained::All(6));
}

GaussianFactorGraph Link::forwardFactors(int j, const Pose3 &jTi,
        double joint_vel_j, const Vector6 &twist_j, double torque_j, 
        const Pose3 &kTj, boost::optional<Vector3&> gravity) const {

    GaussianFactorGraph factors = GaussianFactorGraph();

    // Twist acceleration in this link as a function of previous and joint accel.
    // We need to know our screw axis, and an adjoint map:    
    Vector6 A_j = screwAxis_; //joint axis expressed in COM frame
    Matrix6 ad_j = Pose3::adjointMap(twist_j);

    // Given the above Equation 8.47 can be written as
    // T(j) - A_j * a(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j   
    Vector6 rhs = ad_j * A_j * joint_vel_j;
    factors.add(T(j), Matrix::Identity(6, 6),
                a(j), -A_j,
                T(j - 1), -jTi.AdjointMap(),
                rhs, noiseModel::Constrained::All(6));

    // Wrench on this link is due to acceleration and reaction to next link.     
    factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));

    // Torque is always wrench projected on screw axis. 
    // Equation 8.49 can be written as
    // A_j.transpose() * F(j).transpose() == torque_j
    factors.add(F(j), A_j.transpose(), Vector1(torque_j), noiseModel::Constrained::All(1));      

    return factors;
}

GaussianFactorGraph Link::inverseFactors(int j, const Pose3 &jTi,
        double joint_vel_j, const Vector6 &twist_j, double acceleration_j, 
        const Pose3 &kTj, boost::optional<Vector3&> gravity) const {

    GaussianFactorGraph factors = GaussianFactorGraph();

    // Twist acceleration in this link as a function of previous and joint accel.
    // We need to know our screw axis, and an adjoint map:
    Vector6 A_j = screwAxis_; //joint axis expressed in COM frame
    Matrix6 ad_j = Pose3::adjointMap(twist_j);
    // Given the above Equation 8.47 can be written as
    // T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j * acceleration_j
    Vector6 rhs = ad_j * A_j * joint_vel_j + A_j * acceleration_j;
    factors.add(T(j), Matrix::Identity(6, 6),
                T(j - 1), -jTi.AdjointMap(),
                rhs, noiseModel::Constrained::All(6));
    
    // Wrench on this link is due to acceleration and reaction to next link.
    factors.push_back(wrenchFactor(j, twist_j, kTj, gravity));

    // Torque is always wrench projected on screw axis. 
    // Equation 8.49 can be written as
    // A_j.transpose() * F(j).transpose() == torque_j
    factors.add(F(j), A_j.transpose(),
                t(j), -Matrix::Identity(1, 1), 
                Vector1(0), noiseModel::Constrained::All(1));      

    return factors;
}
}
