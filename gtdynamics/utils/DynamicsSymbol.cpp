/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsSymbol.cpp
 * @brief Symbols to represent keys in dynamics factor graph.
 * @author Yetong Zhang and Stephanie McCormick
 */

#include <gtdynamics/utils/DynamicsSymbol.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

#define kMax_uchar_ std::numeric_limits<unsigned char>::max()

using gtsam::Key;
namespace gtdynamics {

/* ************************************************************************* */
DynamicsSymbol::DynamicsSymbol()
    : c1_(0), c2_(0), link_idx_(0), joint_idx_(0), t_(0) {}

/* ************************************************************************* */
DynamicsSymbol::DynamicsSymbol(const DynamicsSymbol& key)
    : c1_(key.c1_),
      c2_(key.c2_),
      link_idx_(key.link_idx_),
      joint_idx_(key.joint_idx_),
      t_(key.t_) {}

/* ************************************************************************* */
DynamicsSymbol::DynamicsSymbol(const std::string& s, unsigned char link_idx,
                               unsigned char joint_idx, std::uint64_t t)
    : link_idx_(link_idx), joint_idx_(joint_idx), t_(t) {
  if (s.length() > 2) {
    throw std::runtime_error(
        "cannot use more than 2 characters in dynamics symbol");
  }
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

DynamicsSymbol DynamicsSymbol::LinkJointSymbol(const std::string& s,
                                               unsigned char link_idx,
                                               unsigned char joint_idx,
                                               std::uint64_t t) {
  return DynamicsSymbol(s, link_idx, joint_idx, t);
}

DynamicsSymbol DynamicsSymbol::JointSymbol(const std::string& s,
                                           unsigned char joint_idx,
                                           std::uint64_t t) {
  return DynamicsSymbol(s, kMax_uchar_, joint_idx, t);
}

DynamicsSymbol DynamicsSymbol::LinkSymbol(const std::string& s,
                                          unsigned char link_idx,
                                          std::uint64_t t) {
  return DynamicsSymbol(s, link_idx, kMax_uchar_, t);
}

DynamicsSymbol DynamicsSymbol::SimpleSymbol(const std::string& s,
                                            std::uint64_t t) {
  return DynamicsSymbol(s, kMax_uchar_, kMax_uchar_, t);
}

/* ************************************************************************* */
DynamicsSymbol::DynamicsSymbol(const Key& key) {
  const size_t key_bits = sizeof(Key) * 8;
  const size_t ch1_bits = sizeof(unsigned char) * 8;
  const size_t ch2_bits = sizeof(unsigned char) * 8;
  const size_t link_bits = sizeof(unsigned char) * 8;
  const size_t joint_bits = sizeof(unsigned char) * 8;
  const size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;
  const Key ch1_mask = Key(kMax_uchar_) << (key_bits - ch1_bits);
  const Key ch2_mask = Key(kMax_uchar_) << (key_bits - ch1_bits - ch2_bits);
  const Key link_mask = Key(kMax_uchar_) << (time_bits + joint_bits);
  const Key joint_mask = Key(kMax_uchar_) << time_bits;
  const Key time_mask = ~(ch1_mask | ch2_mask | link_mask | joint_mask);
  c1_ = (unsigned char)((key & ch1_mask) >> (key_bits - ch1_bits));
  c2_ = (unsigned char)((key & ch2_mask) >> (key_bits - ch1_bits - ch2_bits));
  link_idx_ = (unsigned char)((key & link_mask) >> (time_bits + joint_bits));
  joint_idx_ = (unsigned char)((key & joint_mask) >> time_bits);
  t_ = key & time_mask;
}

/* ************************************************************************* */
DynamicsSymbol::operator Key() const {
  const size_t key_bits = sizeof(Key) * 8;
  const size_t ch1_bits = sizeof(unsigned char) * 8;
  const size_t ch2_bits = sizeof(unsigned char) * 8;
  const size_t link_bits = sizeof(unsigned char) * 8;
  const size_t joint_bits = sizeof(unsigned char) * 8;
  const size_t time_bits =
      key_bits - ch1_bits - ch2_bits - link_bits - joint_bits;
  Key ch1_comp = Key(c1_) << (key_bits - ch1_bits);
  Key ch2_comp = Key(c2_) << (key_bits - ch1_bits - ch2_bits);
  Key link_comp = Key(link_idx_) << (time_bits + joint_bits);
  Key joint_comp = Key(joint_idx_) << time_bits;
  Key key = ch1_comp | ch2_comp | link_comp | joint_comp | t_;
  return key;
}

/* ************************************************************************* */
void DynamicsSymbol::print(const std::string& s) const {
  std::cout << s << ": " << std::string(*this) << std::endl;
}

/* ************************************************************************* */
DynamicsSymbol::operator std::string() const {
  std::string s = label();
  if (link_idx_ != kMax_uchar_) {
    s += "[" + std::to_string((int)(link_idx_)) + "]";
  }
  if (joint_idx_ != kMax_uchar_) {
    s += "(" + std::to_string((int)(joint_idx_)) + ")";
  }
  s += std::to_string(t_);
  return s;
}

std::string _GTDKeyFormatter(Key key) {
  return std::string(DynamicsSymbol(key));
}

/* ************************************************************************* */

}  // namespace gtdynamics
