/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_keys.h
 * @author Harrison Jenkins
 *
 * Variable keys shared by the initialization and the factor graph. GTSAM only.
 *
 *   q_i : joint angles at state i        (gtsam::Vector, n)
 *   v_i : joint velocities at state i    (gtsam::Vector, n)
 *   u_i : joint accelerations (control)  (gtsam::Vector, n), applied from i to i+1
 *
 * The GPMP2 factors take stacked gtsam::Vector keys for q and v, so we use our
 * own symbols instead of GTDynamics' per-joint JointAngleKey. The key functions
 * are capitalized like GTDynamics' own key functions (JointAngleKey, PoseKey).
 */

#pragma once

#include <gtsam/inference/Symbol.h>

namespace scate {

/// Key of the joint angles q_i at state i.
inline gtsam::Key QKey(size_t i) { return gtsam::Symbol('q', i); }

/// Key of the joint velocities v_i at state i.
inline gtsam::Key VKey(size_t i) { return gtsam::Symbol('v', i); }

/// Key of the control (joint accelerations) u_i applied from state i to i+1.
inline gtsam::Key UKey(size_t i) { return gtsam::Symbol('u', i); }

}  // namespace scate
