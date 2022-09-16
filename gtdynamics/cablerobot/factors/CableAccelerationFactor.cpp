/**
 * @file  CableAccelerationFactor.cpp
 * @brief Cable acceleration factor: relates cable length acceleration with end
 * effector pose/twist/twist acceleration.  The cable mounting locations on the
 * stationary frame and on the end-effector are passed as parameters.
 * @author Gerry Chen
 */

#include "CableAccelerationFactor.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

using namespace gtsam;

namespace gtdynamics {

// Attention: This version currently ignores all rotational effects since
// they're relatively insignificant and it's just a pain to deal with them.
double CableAccelerationFactor::computeLddot(
    const Pose3 &wTx, const Vector6 &Vx, const Vector6 &VAx,
    boost::optional<Matrix &> H_wTx, boost::optional<Matrix &> H_Vx,
    boost::optional<Matrix &> H_VAx) const {
  Matrix33 wAb_H_wRx;
  Matrix33 wAb_H_xAb;
  Matrix36 wAb_H_wTx;
  Matrix36 wAb_H_VAx;
  Matrix36 wPb_H_wTx;
  Matrix33 dir_H_wPb;
  Matrix13 H_dir;
  Matrix13 H_wAb;

  // linear acceleration
  Vector3 xAb = VAx.bottomRows<3>();  // acceleration of point b in x's frame
  Vector3 wAb = wTx.rotation().rotate(xAb, H_wTx ? &wAb_H_wRx : 0,
                                      H_VAx ? &wAb_H_xAb : 0);
  if (H_wTx) wAb_H_wTx << wAb_H_wRx, Z_3x3;
  if (H_VAx) wAb_H_VAx << Z_3x3, wAb_H_xAb;
  // cable direction
  Point3 wPb = wTx.transformFrom(xPb_, H_wTx ? &wPb_H_wTx : 0);
  Vector3 dir = normalize(wPb - wPa_, H_wTx ? &dir_H_wPb : 0);
  // lddot
  double lddot = dot(dir, wAb,            //
                     H_wTx ? &H_dir : 0,  //
                     (H_wTx || H_VAx) ? &H_wAb : 0);
  if (H_wTx) *H_wTx = H_dir * dir_H_wPb * wPb_H_wTx + H_wAb * wAb_H_wTx;
  if (H_Vx) *H_Vx = Matrix16::Zero();
  if (H_VAx) *H_VAx = H_wAb * wAb_H_VAx;
  return lddot;
}

// below: pseudocode for full version
// static Vector3 compute_linear_acceleration_b(Pose3 x, Point3 xPe, Vector6 Vx, Vector6 VAx) {
//   Vector6 VAe1 = xPe.inverse().AdjointMap() * VAx;
//   Vector3 ae1 = VAe.bottom<3>();

//   Vector6 Ve = xPe.inverse().AdjointMap() * Vx;
//   Vector6 VAe2 = -Pose3::adjointMap(Ve, Ve);
//   Vector3 ae2 = VAe2.bottom<3>();

//   return ae1 + ae2;
// }

}  // namespace gtdynamics
