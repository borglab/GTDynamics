/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamicsSymbol.cpp
 * @brief Test DynamicsSymbol.
 * @Author: Yetong Zhang, Stephanie McCormick
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
TEST(DynamicsSymbol, LinkJointSymbol) {
  std::string variable_type = "F";
  const unsigned char link_index = 1;
  const unsigned char joint_index = 2;
  const uint64_t t = 10;
  const DynamicsSymbol symbol = DynamicsSymbol::LinkJointSymbol(
      variable_type, link_index, joint_index, t);
  const Key key = 0x004601020000000A;
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal(variable_type, symbol.label()));
  EXPECT_LONGS_EQUAL(link_index, symbol.linkIdx());
  EXPECT_LONGS_EQUAL(joint_index, symbol.jointIdx());
  EXPECT_LONGS_EQUAL(t, symbol.time());
  const std::string expected_str = "F[1](2)10";
  EXPECT(assert_equal(expected_str, (std::string)(symbol)));
  EXPECT(assert_equal(expected_str, GTDKeyFormatter(symbol)));
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)DynamicsSymbol(key));
}

TEST(DynamicsSymbol, LinkSymbol) {
  const unsigned char link_index = 2;
  const uint64_t t = 10;
  const DynamicsSymbol symbol =
      DynamicsSymbol::LinkSymbol("FA", link_index, 10);
  const Key key = 0x464102FF0000000A;
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal("FA", symbol.label()));
  EXPECT_LONGS_EQUAL(link_index, symbol.linkIdx());
  EXPECT_LONGS_EQUAL(t, symbol.time());
  const std::string expected_str = "FA[2]10";
  EXPECT(assert_equal(expected_str, (std::string)(symbol)));
  EXPECT(assert_equal(expected_str, GTDKeyFormatter(symbol)));
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)DynamicsSymbol(key));
}

TEST(DynamicsSymbol, JointSymbol) {
  const unsigned char joint_index = 1;
  const uint64_t t = 10;
  const DynamicsSymbol symbol =
      DynamicsSymbol::JointSymbol("q", joint_index, 10);
  const Key key = 0x0071FF010000000A;
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal("q", symbol.label()));
  EXPECT_LONGS_EQUAL(joint_index, symbol.jointIdx());
  EXPECT_LONGS_EQUAL(t, symbol.time());
  const std::string expected_str = "q(1)10";
  EXPECT(assert_equal(expected_str, (std::string)(symbol)));
  EXPECT(assert_equal(expected_str, GTDKeyFormatter(symbol)));
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)DynamicsSymbol(key));
}

TEST(DynamicsSymbol, SimpleSymbol) {
  const DynamicsSymbol symbol = DynamicsSymbol::SimpleSymbol("ti", 10);
  const Key key = 0x7469FFFF0000000A;
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal("ti", symbol.label()));
  EXPECT_LONGS_EQUAL(10, symbol.time());
  const std::string expected_str = "ti10";
  EXPECT(assert_equal(expected_str, (std::string)(symbol)));
  EXPECT(assert_equal(expected_str, GTDKeyFormatter(symbol)));
  EXPECT_LONGS_EQUAL((long)key, (long)(Key)DynamicsSymbol(key));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
