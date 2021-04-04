/**
 * @file  CableVelocityFactor.h
 * @brief Cable velocity factor: relates cable velocity (or acceleration), two
 * mounting points, and resultant mounting point velocities
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "CableVelocityFactor.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

using namespace gtsam;

namespace gtdynamics {

double CableVelocityFactor::computeLdot(const Pose3 &wTee, const Vector6 &Vee,
                                        boost::optional<Matrix &> H_wTee,
                                        boost::optional<Matrix &> H_Vee) const {
  // Jacobians: cable direction
  Matrix13 H_dir;
  Matrix33 dir_H_wPem;
  Matrix36 wPem_H_wTee;
  // Jacobians: _E_nd-effector _M_ounting point velocity (in world coords)
  Matrix13 H_wPDOTem;
  Matrix33 wPDOTem_H_wRee;
  Matrix33 wPDOTem_H_eePDOTem;
  Matrix36 eePDOTem_H_Vee;
  Matrix33 cross_H_omega;

  // cable direction
  Point3 wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
  Vector3 dir = normalize(wPem - wPb_, H_wTee ? &dir_H_wPem : 0);

  // velocity aka pdot
  // TODO(gerry): use Adjoint
  Vector3 eePDOTem =
      Vee.tail<3>() + cross(Vee.head<3>(), eePem_, H_Vee ? &cross_H_omega : 0);
  if (H_Vee) eePDOTem_H_Vee << cross_H_omega, I_3x3;
  Vector3 wPDOTem = wTee.rotation().rotate(eePDOTem,  //
                                           H_wTee ? &wPDOTem_H_wRee : 0,
                                           H_Vee ? &wPDOTem_H_eePDOTem : 0);

  // ldot = (cable direction) dot (velocity aka pdot)
  double ldot = dot(dir, wPDOTem,         //
                    H_wTee ? &H_dir : 0,  //
                    H_Vee ? &H_wPDOTem : 0);

  // jacobians
  if (H_wTee) {
    *H_wTee = H_dir * dir_H_wPem * wPem_H_wTee;  //
    H_wTee->leftCols<3>() += H_wPDOTem * wPDOTem_H_wRee;
  }
  if (H_Vee) *H_Vee = H_wPDOTem * wPDOTem_H_eePDOTem * eePDOTem_H_Vee;

  return ldot;
}

}  // namespace gtdynamics
