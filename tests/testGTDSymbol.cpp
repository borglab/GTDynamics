/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamicsGraph.cpp
 * @brief Test forward and inverse dynamics factor graph.
 * @Author: Yetong Zhang, Stephanie McCormick
 */

#include <gtdynamics/utils/GTDSymbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
TEST(GTDSymbol, KeyGTDSymbolEncoding) {
  std::string variable_type = "F";
  unsigned char link_idx = 1;
  unsigned char joint_idx = 2;
  uint64_t t = 10;
  GTDSymbol symbol(variable_type, link_idx, joint_idx, t);
  Key key = 0x004601020000000A;
  string str = "F[1](2)10";

  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal(variable_type, symbol.label()));
  EXPECT_LONGS_EQUAL(link_idx, symbol.linkIdx());
  EXPECT_LONGS_EQUAL(joint_idx, symbol.jointIdx());
  EXPECT_LONGS_EQUAL(t, symbol.time());
  EXPECT(assert_equal(str, (std::string)(symbol)));
  EXPECT(assert_equal(str, GTDKeyFormatter(symbol)));
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)GTDSymbol(key));

  GTDSymbol symbol_link = GTDSymbol::LinkSymbol("FA", 2, 10);
  Key key_link = 0x464102FF0000000A;
  string str_link = "FA[2]10";
  EXPECT_LONGS_EQUAL((long)key_link, (long)(Key)symbol_link);
  EXPECT(assert_equal("FA", symbol_link.label()));
  EXPECT_LONGS_EQUAL(2, symbol_link.linkIdx());
  EXPECT_LONGS_EQUAL(10, symbol_link.time());
  EXPECT(assert_equal(str_link, (std::string)(symbol_link)));
  EXPECT(assert_equal(str_link, GTDKeyFormatter(symbol_link)));
  EXPECT_LONGS_EQUAL((long)key_link, (long)(Key)GTDSymbol(key_link));

  GTDSymbol symbol_joint = GTDSymbol::JointSymbol("q", 1, 10);
  Key key_joint = 0x0071FF010000000A;
  string str_joint = "q(1)10";
  EXPECT_LONGS_EQUAL((long)key_joint, (long)(Key)symbol_joint);
  EXPECT(assert_equal("q", symbol_joint.label()));
  EXPECT_LONGS_EQUAL(1, symbol_joint.jointIdx());
  EXPECT_LONGS_EQUAL(10, symbol_joint.time());
  EXPECT(assert_equal(str_joint, (std::string)(symbol_joint)));
  EXPECT(assert_equal(str_joint, GTDKeyFormatter(symbol_joint)));
  EXPECT_LONGS_EQUAL((long)key_joint, (long)(Key)GTDSymbol(key_joint));

  GTDSymbol symbol_simple = GTDSymbol::SimpleSymbol("ti", 10);
  Key key_simple = 0x7469FFFF0000000A;
  string str_simple = "ti10";
  EXPECT_LONGS_EQUAL((long)key_simple, (long)(Key)symbol_simple);
  EXPECT(assert_equal("ti", symbol_simple.label()));
  EXPECT_LONGS_EQUAL(10, symbol_simple.time());
  EXPECT(assert_equal(str_simple, (std::string)(symbol_simple)));
  EXPECT(assert_equal(str_simple, GTDKeyFormatter(symbol_simple)));
  EXPECT_LONGS_EQUAL((long)key_simple, (long)(Key)GTDSymbol(key_simple));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

