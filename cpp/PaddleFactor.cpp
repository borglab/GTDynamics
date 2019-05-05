/**
 *  @file  testPaddleFactor.cpp
 *  @brief A factor that evaluates forward movements on a simple turtle model.
 *  @author Stephen Eick
 *  @author Frank Dellaert
 *  @date April 2019
 */

#include <gtsam_unstable/nonlinear/PaddleFactor.h>

#include <cmath>
#include <iostream>
#include <boost/optional/optional_io.hpp>

using namespace std;

namespace gtsam

{
Vector PaddleFactor::evaluateError(const Vector3 &c, boost::optional<Matrix &> error_H_c) const
{
    // This uses standard Cartesian coordinates.
    Matrix13 theta_H_c;
    const double tau = omega_ * t_;
    theta_H_c << 1, sin(tau), cos(tau);
    const double theta = theta_H_c * c;
    //const double vx = length_ * sin(theta);
    const double angular_velocity = (c[1] * cos(tau) - c[2] * sin(tau)) * omega_;
    const double vx = length_ * angular_velocity * sin(theta);
    const double z = -length_ * sin(theta); 
    const double objective = alpha_ * vx * exp(beta_ * z);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    const double error = -objective;

    // Calculate derivative
    if (error_H_c)
    {
        // Calculate partial derivatives.
        const double obj_H_vx = alpha_ * exp(beta_ * z);
        const double obj_H_z = alpha_ * beta_ * vx * exp(beta_ * z);
        const double z_H_theta = -length_ * cos(theta);
        const double vx_H_theta = -omega_ * length_ * cos(theta);
        const double vx_H_omega = -length_ * sin(theta);
        Matrix13 theta_H_c;
        theta_H_c << 1, sin(tau), cos(tau);
        Matrix13 omega_H_c;
        omega_H_c << 0, omega_ * cos(tau), -omega_ * sin(tau);

        // Combine partial derivatives to get full derivative.
        Matrix13 vx_H_c_omega;
        vx_H_c_omega << vx_H_omega * omega_H_c;
        Matrix13 vx_H_c_theta;
        vx_H_c_theta << vx_H_theta * theta_H_c;
        Matrix13 vx_H_c;
        vx_H_c << vx_H_c_omega + vx_H_c_theta;

        Matrix13 z_H_c;
        z_H_c << z_H_theta * theta_H_c;

        Matrix13 obj_H_c_z;
        obj_H_c_z << obj_H_z * z_H_c;
        Matrix13 obj_H_c_vx;
        obj_H_c_vx << obj_H_vx * vx_H_c;
        Matrix13 obj_H_c;
        obj_H_c << obj_H_c_z + obj_H_c_vx;
        *error_H_c = Matrix(1, 3);
        *error_H_c << -obj_H_c;

        // const double vx_H_theta = length_ * cos(theta);
        // const double z_H_theta = -length_ * sin(theta);
        // Matrix23 vx_z_H_c;
        // vx_z_H_c << vx_H_theta * theta_H_c, z_H_theta * theta_H_c;
        // Matrix12 error_H_vx_z;
        // error_H_vx_z << alpha_ * exp(-beta_ * z), alpha_ * vx * -beta_ * exp(-beta_ * z);
        // *error_H_c = Matrix(1, 3);
        // *error_H_c << error_H_vx_z * vx_z_H_c;

        // Debugging statements
        // cout << "alpha: " << alpha_;
        // cout << ", beta: " << beta_;
        cout << ", t: " << t_;
        cout << ", omega: " << omega_;
        cout << ", angular_velocity: " << angular_velocity;
        cout << ", theta: " << theta;
        cout << ", vx: " << vx;
        cout << ", z: " << z;
        cout << ", error: " << error << endl << endl;
        // cout << "=========" << endl;
        // cout << "vx_H_theta: " << vx_H_theta << endl;
        // cout << "z_H_theta: " << z_H_theta << endl;
        // cout << "=========" << endl;
        // cout << "vx_z_H_c: " << vx_z_H_c << endl;
        // cout << "=========" << endl;
        // cout << "error_H_vx_z: " << error_H_vx_z << endl;
        // cout << "=========" << endl;
        cout << "error_H_c: " << error_H_c << endl;
    }

    return Vector1(error);
    //return Vector1(1.23);
}
} // namespace gtsam
