/**
 * @file  CableTensionFactor.cpp
 * @brief Cable tension factor: relates cable tension, two mounting points, and
 * resultant forces
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include "CableTensionFactor.h"

#include <gtdynamics/utils/DynamicsSymbol.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

using namespace gtsam;

namespace gtdynamics {

/******************************************************************************/
Vector6 CableTensionFactor::computeWrench(
    double t, const Pose3 &wTx, boost::optional<Matrix &> H_t,
    boost::optional<Matrix &> H_wTx) const {
  // Jacobians: cable direction
  Matrix33 dir_H_wPb;
  Matrix36 wPb_H_wTx;
  // Jacobians: force to wrench conversion
  Matrix31 wf_H_t;
  Matrix33 wf_H_dir;
  Matrix33 xf_H_wf;
  Matrix33 xf_H_wRx;
  Matrix63 H_xf;
  Matrix33 xm_H_xf;  // = H_xf.topRows<3>(); TODO(gerry): pointer?

  // cable direction
  Point3 wPb = wTx.transformFrom(xPb_, H_wTx ? &wPb_H_wTx : 0);
  Vector3 dir = normalize(wPb - wPa_, H_wTx ? &dir_H_wPb : 0);
  // force->wrench
  Vector3 wf = -t * dir;
  if (H_t) wf_H_t = -dir;
  if (H_wTx) wf_H_dir = -t * I_3x3;
  Vector3 xf = wTx.rotation().unrotate(wf,  // force in the EE frame
                                       H_wTx ? &xf_H_wRx : 0,
                                       (H_t || H_wTx) ? &xf_H_wf : 0);
  Vector3 xm = cross(xPb_, xf,     // moment in the EE frame
                     boost::none,  //
                     (H_t || H_wTx) ? &xm_H_xf : 0);

  Vector6 F = (Vector6() << xm, xf).finished();
  if (H_t || H_wTx) H_xf << xm_H_xf, I_3x3;
  if (H_t) *H_t = H_xf * xf_H_wf * wf_H_t;
  if (H_wTx) {
    *(H_wTx) = H_xf * xf_H_wf * wf_H_dir * dir_H_wPb * wPb_H_wTx;
    H_wTx->leftCols<3>() += H_xf * xf_H_wRx;
  }
  return F;
}

/******************************************************************************/
Vector6 CableTensionFactor::computeWrench2(
    double t, const Pose3 &wTx, boost::optional<Matrix &> H_t,
    boost::optional<Matrix &> H_wTx) const {
  // Jacobians: cable direction
  Matrix33 dir_H_wPb;
  Matrix36 wPb_H_wTx;
  // Jacobians: force to wrench conversion
  Matrix61 wF_H_t;
  Matrix63 wF_H_dir;

  // cable direction
  Point3 wPb = wTx.transformFrom(xPb_, H_wTx ? &wPb_H_wTx : 0);
  Vector3 dir = normalize(wPb - wPa_, H_wTx ? &dir_H_wPb : 0);
  // force->wrench
  Pose3 bTx = Pose3(wTx.rotation().inverse(), xPb_).inverse();
  Vector6 wF = (Vector6() << 0, 0, 0, -t * dir).finished();
  if (H_t) wF_H_t = (Matrix61() << 0, 0, 0, -dir).finished();
  if (H_wTx) wF_H_dir = (Matrix63() << Z_3x3, -t * I_3x3).finished();
  Vector6 F = bTx.AdjointMap().transpose() * wF;
  // TODO(gerry): find jacobian of adjoint map
  return F;
}

}  // namespace gtdynamics
