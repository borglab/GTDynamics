/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsSymbol.h
 * @brief Symbols to represent keys in dynamics factor graph.
 * @Author: Yetong Zhang and Stephanie McCormick
 */

# pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

namespace gtdynamics {

class DynamicsSymbol {
 protected:
  unsigned char c1_, c2_, link_idx_, joint_idx_;
  std::uint64_t t_;

 private:
  /** Constructor.
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] link_idx  index of the link
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  DynamicsSymbol(const std::string& s, unsigned char link_idx,
                 unsigned char joint_idx, std::uint64_t t);

 public:
  /** Default constructor */
  DynamicsSymbol();

  /** Copy constructor */
  DynamicsSymbol(const DynamicsSymbol& key);

  /** Constructor for symbol related to both link and joint.
   *  See private constructor
   */
  static DynamicsSymbol LinkJointSymbol(const std::string& s,
                                        unsigned char link_idx,
                                        unsigned char joint_idx,
                                        std::uint64_t t);

  /** Constructor for symbol related to only joint (e.g. joint angle).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  static DynamicsSymbol JointSymbol(const std::string& s,
                                    unsigned char joint_idx, std::uint64_t t);

  /** Constructor for symbol related to only link (e.g. link pose).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  static DynamicsSymbol LinkSymbol(const std::string& s, unsigned char link_idx,
                                   std::uint64_t t);

  /** Constructor for symbol related to neither joint or link (e.g. time).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] t         time step
   */
  static DynamicsSymbol SimpleSymbol(const std::string& s, std::uint64_t t);

  /** Constructor that decodes an integer gtsam::Key */
  DynamicsSymbol(const gtsam::Key& key);

  /** Cast to integer */
  operator gtsam::Key() const;

  inline std::string label() const {
    std::string s = "";
    if (c1_ != 0) s += c1_;
    if (c2_ != 0) s += c2_;
    return s;
  }

  inline unsigned char linkIdx() const { return link_idx_; }

  inline unsigned char jointIdx() const { return joint_idx_; }

  /** Retrieve key index */
  inline size_t time() const { return t_; }

  // Testable Requirements
  void print(const std::string& s = "") const;

  bool equals(const DynamicsSymbol& expected, double tol = 0.0) const {
    return (*this) == expected;
  }

  /** return the integer version */
  gtsam::Key key() const { return (gtsam::Key) * this; }

  /** Create a string from the key */
  operator std::string() const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(c1_);
    ar& BOOST_SERIALIZATION_NVP(c2_);
    ar& BOOST_SERIALIZATION_NVP(link_idx_);
    ar& BOOST_SERIALIZATION_NVP(joint_idx_);
    ar& BOOST_SERIALIZATION_NVP(t_);
  }
};

/** key formatter function */
std::string _GTDKeyFormatter(gtsam::Key key);

static const gtsam::KeyFormatter GTDKeyFormatter = &_GTDKeyFormatter;

}  // namespace gtdynamics
