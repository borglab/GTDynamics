/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GTDSymbol.h
 * @brief Symbols to represent keys in dynamics factor graph.
 * @Author: Yetong Zhang and Stephanie McCormick
 */


#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>

namespace gtdynamics {

class GTDSymbol {
protected:
  unsigned char c1_, c2_, link_idx_, joint_idx_;
  std::uint64_t t_;

public:
 /** Default constructor */
 GTDSymbol();

 /** Copy constructor */
 GTDSymbol(const GTDSymbol& key);

 /** Constructor for symbol related to both link and joint.
 *
 * @param[in] s         1 or 2 characters to represent the varaible type
 * @param[in] link_idx  index of the link
 * @param[in] joint_idx index of the joint
 * @param[in] t         time step
 */
 GTDSymbol(std::string s, unsigned char link_idx, unsigned char joint_idx,
           std::uint64_t t);

 /** Constructor for symbol related to only joint (e.g. joint angle).
 *
 * @param[in] s         1 or 2 characters to represent the varaible type
 * @param[in] joint_idx index of the joint
 * @param[in] t         time step
 */
 static GTDSymbol JointSymbol(std::string s, unsigned char joint_idx,
                              std::uint64_t t);

 /** Constructor for symbol related to only link (e.g. link pose).
 *
 * @param[in] s         1 or 2 characters to represent the varaible type
 * @param[in] joint_idx index of the joint
 * @param[in] t         time step
 */
 static GTDSymbol LinkSymbol(std::string s, unsigned char link_idx,
                             std::uint64_t t);

 /** Constructor for symbol related to neither joint or link (e.g. time).
 *
 * @param[in] s         1 or 2 characters to represent the varaible type
 * @param[in] t         time step
 */
 static GTDSymbol SimpleSymbol(std::string s, std::uint64_t t);

 /** Constructor that decodes an integer gtsam::Key */
 GTDSymbol(gtsam::Key key);

 /** Cast to integer */
 operator gtsam::Key() const;

 inline std::string label() const  {
  std::string s = "";
  if (c1_ != 0) s += c1_;
  if (c2_ != 0) s += c2_;
  return s;
 }

 inline unsigned char linkIdx() const {return link_idx_; }

 inline unsigned char jointIdx() const {return joint_idx_; }

 /** Retrieve key index */
 inline size_t time() const { return t_; }

 // Testable Requirements
 void print(const std::string& s = "") const;

 bool equals(const GTDSymbol& expected, double tol = 0.0) const {
   return (*this) == expected;
  }

  /** return the integer version */
  gtsam::Key key() const { return (gtsam::Key) *this; }

  /** Create a string from the key */
  operator std::string() const;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(c1_);
    ar & BOOST_SERIALIZATION_NVP(c2_);
    ar & BOOST_SERIALIZATION_NVP(link_idx_);
    ar & BOOST_SERIALIZATION_NVP(joint_idx_);
    ar & BOOST_SERIALIZATION_NVP(t_);
  }
};

// /// traits
// template<> struct gtsam::traits<GTDSymbol> : public Testable<GTDSymbol> {};

std::string _GTDKeyFormatter(gtsam::Key key);

static const gtsam::KeyFormatter GTDKeyFormatter =
    &_GTDKeyFormatter;

}