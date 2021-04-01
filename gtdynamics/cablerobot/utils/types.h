/**
 * @file RobotTypes.h
 * @brief common constants and typedefs
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtdynamics {
namespace cablerobot {

#define NEXTENUM(ENUM) (static_cast<unsigned char>(ENUM) + 1)
/// keys for parameters
enum struct PKeys : unsigned char {
  motorF = 'A',  // motor friction
  motorI,        // motor inertia
  winchR,        // winch radius
  cableF,        // cable friction
  cableM,        // cable mass
  anchorP,       // anchor position (per cable)
  eeP,           // end effector mounting position (per cable)
  eeG,           // end effector inertia
  MAX
};

/// keys for control and intermediate variables
enum struct VKeys : unsigned char {
  current = NEXTENUM(PKeys::MAX),  // current (per cable)
  theta,                           // motor angle (per cable)
  omega,                           // motor angular velocity (per cable)
  alpha,                           // motor angular acceleration (per cable)
  torque,                          // torque (per cable)
  l,                               // cable length (per cable)
  v,                               // cable velocity (per cable)
  a,                               // cable acceleration (per cable)
  tension,                         // tension (per cable)
  eeMP,                            // EE mount point position (per cable)
  eeMV,                            // EE mount point velocity (per cable)
  eeMA,                            // EE mount point acceleration (per cable)
  anchorF,                         // wrench on anchor (per cable)
  eeF,                             // wrench on end effector (per cable)
  MAX
};

/// keys for state variables
enum struct XKeys : unsigned char {
  eeT = NEXTENUM(VKeys::MAX),  // end effector pose
  eeV,                   // end effector twist
  eeA,                   // end effector twist acceleration
  eeFT,                  // end effector TOTAL wrench
  MAX
};

/// returns a gtsam::Key for the given parameter, cable index, and time step
static inline gtsam::Key key(PKeys param, size_t cable, size_t t) {
  return gtsam::LabeledSymbol(static_cast<unsigned char>(param), cable, t);
}
/// returns a gtsam::Key for the given variable, cable index, and time step
static inline gtsam::Key key(VKeys var, size_t cable, size_t t) {
  return gtsam::LabeledSymbol(static_cast<unsigned char>(var), cable, t);
}
/// returns a gtsam::Key for the given state variable and time step
static inline gtsam::Key key(XKeys state, size_t t) {
  return gtsam::LabeledSymbol(static_cast<unsigned char>(state), 0, t);
}

/// Map the translational components of the euclidean spaces
template <class Space>
struct SpaceTraits {};

template <> struct SpaceTraits<gtsam::Pose3> {
  typedef gtsam::Point3 PointSpace;
};
template <> struct SpaceTraits<gtsam::Pose2> {
  typedef gtsam::Point2 PointSpace;
};
template <> struct SpaceTraits<gtsam::Point3> {
  typedef gtsam::Point3 PointSpace;
};
template <> struct SpaceTraits<gtsam::Point2> {
  typedef gtsam::Point2 PointSpace;
};

}  // namespace cablerobot
}  // namespace gtdynamics
