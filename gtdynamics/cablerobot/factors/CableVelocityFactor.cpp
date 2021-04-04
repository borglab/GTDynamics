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

double CableVelocityFactor::computeLdot(const Pose3 &wTx, const Vector6 &Vx,
                                        boost::optional<Matrix &> H_wTx,
                                        boost::optional<Matrix &> H_Vx) const {
  // Jacobians: cable direction
  Matrix13 H_dir;
  Matrix33 dir_H_wPb;
  Matrix36 wPb_H_wTx;
  // Jacobians: End-effector Mounting point velocity (in world coords)
  Matrix13 H_wPDOTb;
  Matrix33 wPDOTb_H_wRx;
  Matrix33 wPDOTb_H_xPDOTb;
  Matrix36 xPDOTb_H_Vx;
  Matrix33 cross_H_omega;

  // cable direction
  Point3 wPb = wTx.transformFrom(xPb_, H_wTx ? &wPb_H_wTx : 0);
  Vector3 dir = normalize(wPb - wPa_, H_wTx ? &dir_H_wPb : 0);

  // velocity aka pdot
  // TODO(gerry): use Adjoint
  Vector3 eePDOTem =
      Vx.tail<3>() + cross(Vx.head<3>(), xPb_, H_Vx ? &cross_H_omega : 0);
  if (H_Vx) xPDOTb_H_Vx << cross_H_omega, I_3x3;
  Vector3 wPDOTem = wTx.rotation().rotate(eePDOTem,  //
                                          H_wTx ? &wPDOTb_H_wRx : 0,
                                          H_Vx ? &wPDOTb_H_xPDOTb : 0);

  // ldot = (cable direction) dot (velocity aka pdot)
  double ldot = dot(dir, wPDOTem,        //
                    H_wTx ? &H_dir : 0,  //
                    H_Vx ? &H_wPDOTb : 0);

  // jacobians
  if (H_wTx) {
    *H_wTx = H_dir * dir_H_wPb * wPb_H_wTx;  //
    H_wTx->leftCols<3>() += H_wPDOTb * wPDOTb_H_wRx;
  }
  if (H_Vx) *H_Vx = H_wPDOTb * wPDOTb_H_xPDOTb * xPDOTb_H_Vx;

  return ldot;
}

}  // namespace gtdynamics
