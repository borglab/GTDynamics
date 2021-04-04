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
    double t, const Pose3 &wTee, boost::optional<Matrix &> H_t,
    boost::optional<Matrix &> H_wTee) const {
  // Jacobians: cable direction
  Matrix33 dir_H_wPem;
  Matrix36 wPem_H_wTee;
  // Jacobians: force to wrench conversion
  Matrix31 wf_H_t;
  Matrix33 wf_H_dir;
  Matrix33 eef_H_wf;
  Matrix33 eef_H_wRee;
  Matrix63 H_eef;
  Matrix33 eem_H_eef;  // = H_eef.topRows<3>(); TODO(gerry): pointer?

  // cable direction
  Point3 wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
  Vector3 dir = normalize(wPem - wPb_, H_wTee ? &dir_H_wPem : 0);
  // force->wrench
  Vector3 wf = -t * dir;
  if (H_t) wf_H_t = -dir;
  if (H_wTee) wf_H_dir = -t * I_3x3;
  Vector3 eef = wTee.rotation().unrotate(wf,  //
                                         H_wTee ? &eef_H_wRee : 0,
                                         (H_t || H_wTee) ? &eef_H_wf : 0);
  Vector3 eem = cross(eePem_, eef,  //
                      boost::none,  //
                      (H_t || H_wTee) ? &eem_H_eef : 0);

  Vector6 F = (Vector6() << eem, eef).finished();
  if (H_t || H_wTee) H_eef << eem_H_eef, I_3x3;
  if (H_t) *H_t = H_eef * eef_H_wf * wf_H_t;
  if (H_wTee) {
    *(H_wTee) = H_eef * eef_H_wf * wf_H_dir * dir_H_wPem * wPem_H_wTee;
    H_wTee->leftCols<3>() += H_eef * eef_H_wRee;
  }
  return F;
}

/******************************************************************************/
Vector6 CableTensionFactor::computeWrench2(
    double t, const Pose3 &wTee, boost::optional<Matrix &> H_t,
    boost::optional<Matrix &> H_wTee) const {
  // Jacobians: cable direction
  Matrix33 dir_H_wPem;
  Matrix36 wPem_H_wTee;
  // Jacobians: force to wrench conversion
  Matrix61 wF_H_t;
  Matrix63 wF_H_dir;

  // cable direction
  Point3 wPem = wTee.transformFrom(eePem_, H_wTee ? &wPem_H_wTee : 0);
  Vector3 dir = normalize(wPem - wPb_, H_wTee ? &dir_H_wPem : 0);
  // force->wrench
  Pose3 emTee = Pose3(wTee.rotation().inverse(), eePem_).inverse();
  Vector6 wF = (Vector6() << 0, 0, 0, -t * dir).finished();
  if (H_t) wF_H_t = (Matrix61() << 0, 0, 0, -dir).finished();
  if (H_wTee) wF_H_dir = (Matrix63() << Z_3x3, -t * I_3x3).finished();
  Vector6 F = emTee.AdjointMap().transpose() * wF;
  // TODO(gerry): find jacobian of adjoint map
  return F;
}

}  // namespace gtdynamics
