/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Link.h
 * @brief only link part of a robot, does not include joint part
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
 */

#ifndef GTDYNAMICS_UNIVERSAL_ROBOT_LINK_H_
#define GTDYNAMICS_UNIVERSAL_ROBOT_LINK_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <sdf/sdf.hh>
#include <memory>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

#include "gtdynamics/dynamics/OptimizerSetting.h"
#include "gtdynamics/factors/WrenchFactors.h"
#include "gtdynamics/universal_robot/RobotTypes.h"
#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/* Shorthand for p_i_t, for COM pose on the i-th link at time t. */
inline gtsam::LabeledSymbol PoseKey(int i, int t) {
  return gtsam::LabeledSymbol('p', i, t);
}

/* Shorthand for V_i_t, for 6D link twist vector on the i-th link. */
inline gtsam::LabeledSymbol TwistKey(int i, int t) {
  return gtsam::LabeledSymbol('V', i, t);
}

/* Shorthand for A_i_t, for twist accelerations on the i-th link at time t. */
inline gtsam::LabeledSymbol TwistAccelKey(int i, int t) {
  return gtsam::LabeledSymbol('A', i, t);
}

/* Shorthand for F_i_j_t, for wrenches at j-th joint on the i-th link at time t.
 */
inline gtsam::LabeledSymbol WrenchKey(int i, int j, int t) {
  return gtsam::LabeledSymbol('F', i * 16 + j,
                              t);  // a hack here for a key with 3 numbers
}

/**
 * Link is the base class for links taking different format of parameters
 */
class Link : public std::enable_shared_from_this<Link> {
 private:
  std::string name_;

  int id_ = -1;

  // Inertial elements.
  double mass_;
  gtsam::Pose3 centerOfMass_;
  gtsam::Matrix3 inertia_;

  // SDF Elements.
  gtsam::Pose3 wTl_;    // Link frame defined in the world frame.
  gtsam::Pose3 lTcom_;  // CoM frame defined in the link frame.

  // option to fix the link, used for ground link
  bool is_fixed_;
  gtsam::Pose3 fixed_pose_;

  std::vector<JointSharedPtr> joints_;  // joints connected to the link

 public:
  /**
   * Params contains all parameters to construct a link
   */
  struct Params {
    std::string name;        // name of the link
    double mass;             // mass of the link
    gtsam::Matrix3 inertia;  // inertia of the link
    gtsam::Pose3 wTl;        // link pose expressed in world frame
    gtsam::Pose3 lTcom;      // link com expressed in link frame
  };

  Link() {}

  /**
   * Initialize Link's inertial properties with a sdf::Link instance, as
   * described in the sdformat8 documentation:
   * https://bitbucket.org/osrf/sdformat/src/7_to_gz11/include/sdf/Link.hh
   *
   * Keyword arguments:
   *    sdf_link -- sdf::Link object containing link information.
   */
  explicit Link(sdf::Link sdf_link)
      : name_(sdf_link.Name()),
        mass_(sdf_link.Inertial().MassMatrix().Mass()),
        inertia_(
            (gtsam::Matrix(3, 3) << sdf_link.Inertial().Moi()(0, 0),
             sdf_link.Inertial().Moi()(0, 1), sdf_link.Inertial().Moi()(0, 2),
             sdf_link.Inertial().Moi()(1, 0), sdf_link.Inertial().Moi()(1, 1),
             sdf_link.Inertial().Moi()(1, 2), sdf_link.Inertial().Moi()(2, 0),
             sdf_link.Inertial().Moi()(2, 1), sdf_link.Inertial().Moi()(2, 2))
                .finished()),
        wTl_(parse_ignition_pose(sdf_link.Pose())),
        lTcom_(parse_ignition_pose(sdf_link.Inertial().Pose())),
        is_fixed_(false) {}

  /** constructor using Params */
  explicit Link(const Params &params)
      : name_(params.name),
        mass_(params.mass),
        inertia_(params.inertia),
        wTl_(params.wTl),
        lTcom_(params.lTcom),
        is_fixed_(false) {}

  /** destructor */
  virtual ~Link() = default;

  // return a shared pointer of the link
  LinkSharedPtr getSharedPtr(void) { return shared_from_this(); }

  // remove the joint
  void removeJoint(JointSharedPtr joint) {
    for (auto joint_it = joints_.begin(); joint_it != joints_.end();
         joint_it++) {
      if ((*joint_it) == joint) {
        joints_.erase(joint_it);
        break;
      }
    }
  }

  // set ID for the link
  void setID(unsigned char id) {
    // if (id == 0) throw std::runtime_error("ID cannot be 0");
    id_ = id;
  }

  // return ID of the link
  int getID() const {
    if (id_ == -1)
      throw std::runtime_error(
          "Calling getID on a link whose ID has not been set");
    return id_;
  }

  // add joint to the link
  void addJoint(JointSharedPtr joint_ptr) { joints_.push_back(joint_ptr); }

  // transform from link to world frame
  const gtsam::Pose3 &wTl() const { return wTl_; }

  // transfrom from link com frame to link frame
  const gtsam::Pose3 &lTcom() const { return lTcom_; }

  // transform from link com frame to world frame
  inline const gtsam::Pose3 wTcom() const { return wTl() * lTcom(); }

  // the fixed pose of the link
  const gtsam::Pose3 &getFixedPose() const { return fixed_pose_; }

  // whether the link is fixed
  bool isFixed() const { return is_fixed_; }

  // fix the link to fixed_pose, if fixed_pose not specify, fix the link to
  // default pose
  void fix(const boost::optional<gtsam::Pose3 &> fixed_pose = boost::none) {
    is_fixed_ = true;
    fixed_pose_ = fixed_pose ? *fixed_pose : wTcom();
  }

  // unfix the link
  void unfix() { is_fixed_ = false; }

  // return all joints of the link
  const std::vector<JointSharedPtr> &getJoints(void) const { return joints_; }

  // Return link name.
  std::string name() const { return name_; }

  /// Return link mass.
  double mass() const { return mass_; }

  /// Return center of mass (gtsam::Pose3)
  const gtsam::Pose3 &centerOfMass() const { return centerOfMass_; }

  /// Return inertia.
  const gtsam::Matrix3 &inertia() const { return inertia_; }

  /// Return general mass gtsam::Matrix
  gtsam::Matrix6 inertiaMatrix() const {
    std::vector<gtsam::Matrix> gmm;
    gmm.push_back(inertia_);
    gmm.push_back(gtsam::I_3x3 * mass_);
    return gtsam::diag(gmm);
  }

  /// Return link position factors.
  gtsam::NonlinearFactorGraph qFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    if (isFixed())
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          PoseKey(getID(), t), getFixedPose(), opt.bp_cost_model));
    return graph;
  }

  /// Return link velocity factors.
  gtsam::NonlinearFactorGraph vFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    if (isFixed())
      graph.add(gtsam::PriorFactor<gtsam::Vector6>(
          TwistKey(getID(), t), gtsam::Vector6::Zero(), opt.bv_cost_model));
    return graph;
  }

  /// Return link accel factors.
  gtsam::NonlinearFactorGraph aFactors(const int &t,
                                       const OptimizerSetting &opt) const {
    gtsam::NonlinearFactorGraph graph;
    if (isFixed())
      graph.add(gtsam::PriorFactor<gtsam::Vector6>(TwistAccelKey(getID(), t),
                                                   gtsam::Vector6::Zero(),
                                                   opt.ba_cost_model));
    return graph;
  }

  // Return link dymamics factors.
  gtsam::NonlinearFactorGraph dynamicsFactors(
      const int &t, const OptimizerSetting &opt,
      const std::vector<gtsam::LabeledSymbol> &wrenches,
      const boost::optional<gtsam::Vector3> &gravity) const {
    gtsam::NonlinearFactorGraph graph;
    // Add wrench factors.
    if (wrenches.size() == 0) {
      graph.add(WrenchFactor0(TwistKey(getID(), t), TwistAccelKey(getID(), t),
                              PoseKey(getID(), t), opt.fa_cost_model,
                              inertiaMatrix(), gravity));
    } else if (wrenches.size() == 1) {
      graph.add(WrenchFactor1(TwistKey(getID(), t), TwistAccelKey(getID(), t),
                              wrenches[0], PoseKey(getID(), t),
                              opt.fa_cost_model, inertiaMatrix(), gravity));
    } else if (wrenches.size() == 2) {
      graph.add(WrenchFactor2(TwistKey(getID(), t), TwistAccelKey(getID(), t),
                              wrenches[0], wrenches[1], PoseKey(getID(), t),
                              opt.fa_cost_model, inertiaMatrix(), gravity));
    } else if (wrenches.size() == 3) {
      graph.add(WrenchFactor3(TwistKey(getID(), t), TwistAccelKey(getID(), t),
                              wrenches[0], wrenches[1], wrenches[2],
                              PoseKey(getID(), t), opt.fa_cost_model,
                              inertiaMatrix(), gravity));
    } else if (wrenches.size() == 4) {
      graph.add(WrenchFactor4(TwistKey(getID(), t), TwistAccelKey(getID(), t),
                              wrenches[0], wrenches[1], wrenches[2],
                              wrenches[3], PoseKey(getID(), t),
                              opt.fa_cost_model, inertiaMatrix(), gravity));
    } else {
      throw std::runtime_error("Wrench factor not defined");
    }

    return graph;
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_UNIVERSAL_ROBOT_LINK_H_
