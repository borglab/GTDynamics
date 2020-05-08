/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJoint.h
 * @brief Representation of revolute joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_

#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @class ScrewJoint is an implementation of the ScrewJointBase class
 *  which represents a screw joint and contains all necessary factor
 *  construction methods.
 *  This uses the Curiously Recurring Template Pattern (CRTP) for static
 *  polymorphism and implements the setScrewAxis() and jointType() functions.
 */
class ScrewJoint : public ScrewJointBase<ScrewJoint> {
 protected:
  /// Sets screw axis according to screw type joint
  void setScrewAxis() {
    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    gtsam::Vector6 pScrewAxisRot, pScrewAxisTrans,
                   cScrewAxisRot, cScrewAxisTrans;
    pScrewAxisRot = gtdynamics::unit_twist(pcomRj * -axis_,
                                         pcomRj * (-jTpcom_.translation()));
    cScrewAxisRot = gtdynamics::unit_twist(ccomRj * axis_,
                                         ccomRj * (-jTccom_.translation()));
    pScrewAxisTrans << 0, 0, 0,
        pcomRj * -axis_ / axis_.norm() * thread_pitch_ / 2 / M_PI;
    cScrewAxisTrans << 0, 0, 0,      
        ccomRj * axis_ / axis_.norm() * thread_pitch_ / 2 / M_PI;

    pScrewAxis_ = pScrewAxisRot + pScrewAxisTrans;
    cScrewAxis_ = cScrewAxisRot + cScrewAxisTrans;
  }
 public:
  using ScrewJointBase::ScrewJointBase;

  // give ScrewJointBase access to setScrewAxis()
  friend class ScrewJointBase<ScrewJoint>;

  /// Return jointType
  char jointType() const { return 'C'; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_SCREWJOINT_H_