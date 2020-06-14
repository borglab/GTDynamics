/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GTDSymbol.cpp
 * @brief Symbols to represent keys in dynamics factor graph.
 * @Author: Yetong Zhang and Stephanie McCormick
 */

#include <gtdynamics/utils/GTDSymbol.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

namespace gtdynamics {

/* ************************************************************************* */
GTDSymbol::GTDSymbol() : c1_(0), c2_(0), link_idx_(0), joint_idx_(0), t_(0) {}

/* ************************************************************************* */
GTDSymbol::GTDSymbol(const GTDSymbol& key)
    : c1_(key.c1_),
      c2_(key.c2_),
      link_idx_(key.link_idx_),
      joint_idx_(key.joint_idx_),
      t_(key.t_) {}

/* ************************************************************************* */
GTDSymbol::GTDSymbol(std::string s, unsigned char link_idx,
                     unsigned char joint_idx, std::uint64_t t)
    : link_idx_(link_idx), joint_idx_(joint_idx), t_(t) {
  if (s.length() > 1) {
    c1_ = s[0];
    c2_ = s[1];
  } else if (s.length() == 1) {
    c1_ = 0;
    c2_ = s[0];
  } else {
    c1_ = 0;
    c2_ = 0;
  }
}

GTDSymbol GTDSymbol::JointSymbol(std::string s, unsigned char joint_idx,
                                 std::uint64_t t) {
  return GTDSymbol(s, std::numeric_limits<unsigned char>::max(), joint_idx, t);
}

GTDSymbol GTDSymbol::LinkSymbol(std::string s, unsigned char link_idx,
                                std::uint64_t t) {
  return GTDSymbol(s, link_idx, std::numeric_limits<unsigned char>::max(), t);
}

GTDSymbol GTDSymbol::SimpleSymbol(std::string s, std::uint64_t t) {
  return GTDSymbol(s, std::numeric_limits<unsigned char>::max(),
                   std::numeric_limits<unsigned char>::max(), t);
}

/* ************************************************************************* */
GTDSymbol::GTDSymbol(gtsam::Key key) {
  const size_t key_bits = sizeof(gtsam::Key) * 8;
  const size_t ch1_bits = sizeof(unsigned char) * 8;
  const size_t ch2_bits = sizeof(unsigned char) * 8;
  const size_t link_bits = sizeof(unsigned char) * 8;
  const size_t joint_bits = sizeof(unsigned char) * 8;
  const size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;
  const gtsam::Key ch1_mask =
      gtsam::Key(std::numeric_limits<unsigned char>::max())
      << (key_bits - ch1_bits);
  const gtsam::Key ch2_mask =
      gtsam::Key(std::numeric_limits<unsigned char>::max())
      << (key_bits - ch1_bits - ch2_bits);
  const gtsam::Key link_mask =
      gtsam::Key(std::numeric_limits<unsigned char>::max())
      << (time_bits + joint_bits);
  const gtsam::Key joint_mask =
      gtsam::Key(std::numeric_limits<unsigned char>::max()) << time_bits;
  const gtsam::Key time_mask = ~(ch1_mask | ch2_mask | link_mask | joint_mask);
  c1_ = (unsigned char)((key & ch1_mask) >> (key_bits - ch1_bits));
  c2_ = (unsigned char)((key & ch2_mask) >> (key_bits - ch1_bits - ch2_bits));
  link_idx_ = (unsigned char)((key & link_mask) >> (time_bits + joint_bits));
  joint_idx_ = (unsigned char)((key & joint_mask) >> time_bits);
  t_ = key & time_mask;
}

/* ************************************************************************* */
GTDSymbol::operator gtsam::Key() const {
  const size_t key_bits = sizeof(gtsam::Key) * 8;
  const size_t ch1_bits = sizeof(unsigned char) * 8;
  const size_t ch2_bits = sizeof(unsigned char) * 8;
  const size_t link_bits = sizeof(unsigned char) * 8;
  const size_t joint_bits = sizeof(unsigned char) * 8;
  const size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;
  gtsam::Key ch1_comp = gtsam::Key(c1_) << (key_bits - ch1_bits);
  gtsam::Key ch2_comp = gtsam::Key(c2_) << (key_bits - ch1_bits - ch2_bits);
  gtsam::Key link_comp = gtsam::Key(link_idx_) << (time_bits + joint_bits);
  gtsam::Key joint_comp = gtsam::Key(joint_idx_) << time_bits;
  gtsam::Key key = ch1_comp | ch2_comp | link_comp | joint_comp | t_;
  return key;
}

/* ************************************************************************* */
void GTDSymbol::print(const std::string& s) const {
  std::cout << s << ": " << (std::string)(*this) << std::endl;
}

/* ************************************************************************* */
GTDSymbol::operator std::string() const {
  std::string s = label();
  if (link_idx_ != std::numeric_limits<unsigned char>::max()) {
    s += "[" + std::to_string((int)(link_idx_)) + "]";
  }
  if (joint_idx_ != std::numeric_limits<unsigned char>::max()) {
    s += "(" + std::to_string((int)(joint_idx_))  + ")";
  }
  s += std::to_string(t_);
  return s;
}

std::string _GTDKeyFormatter(gtsam::Key key) {
  const GTDSymbol asGTDSymbol(key);
  return (std::string) asGTDSymbol;
}

/* ************************************************************************* */

}  // namespace gtdynamics
