/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ScrewJointBase.h
 * @brief Representation of screw-type robot joints. Revolute, Prismatic, and
 *  Screw subclasses
 * @author Frank Dellaert
 * @author Mandy Xie
 * @author Alejandro Escontrela
 * @author Yetong Zhang
 * @author Stephanie McCormick
 * @author Gerry Chen
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_

#include "gtdynamics/universal_robot/ScrewJointBase.h"

namespace gtdynamics {

/**
 * @class PrismaticJoint is an implementation of the ScrewJointBase class
 *  which represents a prismatic joint and contains all necessary factor
 *  construction methods.
 *  This uses the Curiously Recurring Template Pattern (CRTP) for static
 *  polymorphism and implements the setScrewAxis() and jointType() functions.
 */
class PrismaticJoint : public ScrewJointBase<PrismaticJoint> {
 protected:
  /// Sets screw axis according to prismatic type joint
  void setScrewAxis() {
    gtsam::Rot3 pcomRj = jTpcom_.rotation().inverse();
    gtsam::Rot3 ccomRj = jTccom_.rotation().inverse();

    pScrewAxis_ << 0, 0, 0, pcomRj * -axis_;
    cScrewAxis_ << 0, 0, 0, ccomRj * axis_;
  }
 public:
  using ScrewJointBase::ScrewJointBase;

  // give ScrewJointBase access to setScrewAxis()
  friend class ScrewJointBase<PrismaticJoint>;

  /// Return jointType
  char jointType() const { return 'P'; }
};

}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_PRISMATICJOINT_H_