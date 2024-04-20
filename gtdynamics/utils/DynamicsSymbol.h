/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsSymbol.h
 * @brief Symbols to represent keys in dynamics factor graph.
 * @author Yetong Zhang and Stephanie McCormick
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include <limits>

namespace gtdynamics {

class DynamicsSymbol {
 protected:
  uint8_t c1_, c2_, link_idx_, joint_idx_;
  uint64_t t_;

 private:
  /**
   * Constructor.
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] link_idx  index of the link
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  DynamicsSymbol(const std::string& s, uint8_t link_idx, uint8_t joint_idx,
                 uint64_t t);

 public:
  /** Default constructor */
  DynamicsSymbol();

  /** Copy constructor */
  DynamicsSymbol(const DynamicsSymbol& key);

  /**
   * Constructor for symbol related to both link and joint.
   *  See private constructor
   */
  static DynamicsSymbol LinkJointSymbol(const std::string& s, uint8_t link_idx,
                                        uint8_t joint_idx, uint64_t t);

  /**
   * Constructor for symbol related to only joint (e.g. joint angle).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  static DynamicsSymbol JointSymbol(const std::string& s, uint8_t joint_idx,
                                    uint64_t t);

  /**
   * Constructor for symbol related to only link (e.g. link pose).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] joint_idx index of the joint
   * @param[in] t         time step
   */
  static DynamicsSymbol LinkSymbol(const std::string& s, uint8_t link_idx,
                                   uint64_t t);

  /**
   * Constructor for symbol related to neither joint or link (e.g. time).
   *
   * @param[in] s         1 or 2 characters to represent the variable type
   * @param[in] t         time step
   */
  static DynamicsSymbol SimpleSymbol(const std::string& s, uint64_t t);

  /**
   * Constructor that decodes an integer gtsam::Key
   */
  DynamicsSymbol(const gtsam::Key& key);

  /// Cast to a GTSAM Key.
  operator gtsam::Key() const;

  /// Return string label.
  std::string label() const;

  /// Return link id.
  inline uint8_t linkIdx() const { return link_idx_; }

  /// Return joint id.
  inline uint8_t jointIdx() const { return joint_idx_; }

  /// Retrieve key index.
  inline uint64_t time() const { return t_; }

  /// Print.
  void print(const std::string& s = "") const;

  /// Check equality.
  bool equals(const DynamicsSymbol& expected, double tol = 0.0) const {
    return (*this) == expected;
  }

  /// return the integer version
  gtsam::Key key() const { return (gtsam::Key) * this; }

  /// Create a string from the key
  operator std::string() const;

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(c1_);
    ar& BOOST_SERIALIZATION_NVP(c2_);
    ar& BOOST_SERIALIZATION_NVP(link_idx_);
    ar& BOOST_SERIALIZATION_NVP(joint_idx_);
    ar& BOOST_SERIALIZATION_NVP(t_);
  }
#endif

  /**
   * \defgroup Bitfield bit field constants
   * @{
   */
  static constexpr size_t kMax_uchar_ = std::numeric_limits<uint8_t>::max();
  // bit counts
  static constexpr size_t key_bits = sizeof(gtsam::Key) * 8;
  static constexpr size_t ch1_bits = sizeof(uint8_t) * 8;
  static constexpr size_t ch2_bits = sizeof(uint8_t) * 8;
  static constexpr size_t link_bits = sizeof(uint8_t) * 8;
  static constexpr size_t joint_bits = sizeof(uint8_t) * 8;
  static constexpr size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;
  // masks
  static constexpr gtsam::Key ch1_mask = gtsam::Key(kMax_uchar_)
                                         << (key_bits - ch1_bits);
  static constexpr gtsam::Key ch2_mask = gtsam::Key(kMax_uchar_)
                                         << (key_bits - ch1_bits - ch2_bits);
  static constexpr gtsam::Key link_mask = gtsam::Key(kMax_uchar_)
                                          << (time_bits + joint_bits);
  static constexpr gtsam::Key joint_mask = gtsam::Key(kMax_uchar_) << time_bits;
  static constexpr gtsam::Key time_mask =
      ~(ch1_mask | ch2_mask | link_mask | joint_mask);
  /**@}*/
};

/// key formatter function
std::string _GTDKeyFormatter(gtsam::Key key);

static const gtsam::KeyFormatter GTDKeyFormatter = &_GTDKeyFormatter;

bool IsQLevel(const gtsam::Key &key);

bool IsVLevel(const gtsam::Key &key);

template <typename CONTAINER>
inline int IdentifyLevel(const CONTAINER &keys) {
  int lvl = 0;
  for (const auto &key : keys) {
    if (IsQLevel(key)) {
      lvl = std::max(lvl, 0);
    } else if (IsVLevel(key)) {
      lvl = std::max(lvl, 1);
    } else {
      lvl = std::max(lvl, 2);
    }
  }
  return lvl;
}

template <typename CONTAINER>
inline void ClassifyKeysByLevel(const CONTAINER &keys, gtsam::KeySet &q_keys,
                         gtsam::KeySet &v_keys, gtsam::KeySet &ad_keys) {
  for (const gtsam::Key &key : keys) {
    if (IsQLevel(key)) {
      q_keys.insert(key);
    } else if (IsVLevel(key)) {
      v_keys.insert(key);
    } else {
      ad_keys.insert(key);
    }
  }
}

template <typename CONTAINER>
inline void ClassifyKeysByLevel(const CONTAINER &keys, gtsam::KeyVector &q_keys,
                         gtsam::KeyVector &v_keys, gtsam::KeyVector &ad_keys) {
  for (const gtsam::Key &key : keys) {
    if (IsQLevel(key)) {
      q_keys.push_back(key);
    } else if (IsVLevel(key)) {
      v_keys.push_back(key);
    } else {
      ad_keys.push_back(key);
    }
  }
}


}  // namespace gtdynamics
