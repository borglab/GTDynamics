/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  RevoluteJoint.h
 * @brief Representation of revolute joint.
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_

#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @class RevoluteJoint is an implementation of the ScrewJointBase class
 *  which represents a revolute joint and contains all necessary factor
 *  construction methods.
 *  This uses the Curiously Recurring Template Pattern (CRTP) for static
 *  polymorphism and implements the setScrewAxis() and jointType() functions.
 */
class RevoluteJoint : public ScrewJointBase<RevoluteJoint> {
 protected:
  /// Sets screw axis according to revolute type joint
  void setScrewAxis() {
    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    pScrewAxis_ = gtdynamics::unit_twist(pcomRj * -axis_,
                                         pcomRj * (-jTpcom_.translation()));
    cScrewAxis_ = gtdynamics::unit_twist(ccomRj * axis_,
                                         ccomRj * (-jTccom_.translation()));
  }
 public:
  using ScrewJointBase::ScrewJointBase;

  // give ScrewJointBase access to setScrewAxis()
  friend class ScrewJointBase<RevoluteJoint>;

  /// Return jointType
  char jointType() const { return 'R'; }
};

} // namespace gtdynamics

#endif // GTDYNAMICS_UNIVERSAL_ROBOT_REVOLUTEJOINT_H_