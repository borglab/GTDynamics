/**
 * @file  testCustomWrap.cpp
 * @brief test utilities in custom wrap file
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include <gtdynamics/cablerobot/utils/CustomWrap.h>

#include <gtsam/linear/GaussianFactorGraph.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <boost/assign/list_of.hpp>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gtdynamics;
using boost::assign::list_of;

/**
 * Test eliminate sequential
 */
TEST(EliminateSequential, fullelimination) {
  GaussianFactorGraph gfg;
  gfg.add(0, Vector1(1), Vector1(0), noiseModel::Unit::Create(1));
  gfg.add(1, Vector1(1), Vector1(0), noiseModel::Unit::Create(1));
  gfg.add(2, Vector1(1),  //
          0, Vector1(1),  //
          Vector1(0), noiseModel::Unit::Create(1));

  // specify the ordering as 0 -> 1 -> 2
  Ordering ordering(list_of(0)(1)(2));
  auto actual = EliminateSequential(gfg, ordering);

  // expected Bayes net
  GaussianBayesNet expected;
  expected.push_back(GaussianConditional(0, Vector1(0), Vector1(sqrt(2)),  //
                                         2, Vector1(1) / sqrt(2)));
  expected.push_back(GaussianConditional(1, Vector1(0), Vector1(1)));
  expected.push_back(GaussianConditional(2, Vector1(0), Vector1(1 / sqrt(2))));

  // check if the result matches
  EXPECT(assert_equal(expected, *actual));
}

/**
 * Test block eliminate sequential
 */
TEST(EliminateSequential, block) {
  GaussianFactorGraph gfg;
  gfg.add(0, Vector1(1), Vector1(0), noiseModel::Unit::Create(1));
  gfg.add(1, Vector1(1), Vector1(0), noiseModel::Unit::Create(1));
  gfg.add(2, Vector1(1),  //
          0, Vector1(1),  //
          Vector1(0), noiseModel::Unit::Create(1));

  // specify the ordering as (0;1) -> 2
  BlockOrdering ordering{list_of(0)(1), list_of(2)};
  auto actual = BlockEliminateSequential(gfg, ordering);

  // expected Bayes net
  GaussianBayesNet expected;
  vector<pair<Key, Matrix>> terms{make_pair(0, Vector2(sqrt(2), 0)),
                                  make_pair(1, Vector2(0, 1)),
                                  make_pair(2, Vector2(1 / sqrt(2), 0))};
  expected.push_back(GaussianConditional(terms, 2, Vector2(0, 0)));
  expected.push_back(GaussianConditional(2, Vector1(0), Vector1(1 / sqrt(2))));

  // check if the result matches
  EXPECT(assert_equal(expected, *actual));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
