/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPhase.cpp
 * @brief Test Phase class.
 * @Author: Disha Das, Tarushree Gandhi
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/Phase.h>

using namespace std;
using gtsam::assert_equal;
using namespace gtsam;
using namespace gtdynamics;

Phase initializeExample(){
  typedef ContactPoint CP;
  const double height = -1.75;
  const auto c1 = CP{"leg_1", Point3(0, 0.19, 0), 0, height};
  const auto c2 = CP{"leg_2", Point3(0, 0.19, 0), 0, height};
  const auto c3 = CP{"leg_3", Point3(0, 0.19, 0), 0, height};
  const auto c4 = CP{"leg_4", Point3(0, 0.19, 0), 0, height};
  const int repeat = 2;
  const int time_step = 20;
  const vector<string> sequence = {"p0", "p1", "p0", "p2", "p0", "p2"};
  auto robot_phase = Phase(sequence);
  robot_phase.addContactPoints({c1, c2, c3, c4});
  robot_phase.addStance(sequence[0], {"leg_1", "leg_2","leg_3","leg_4"});
  robot_phase.addStance(sequence[1], {"leg_1",         "leg_3"        });
  robot_phase.addStance(sequence[3], {         "leg_2",        "leg_4"});
  robot_phase.setRepetitions(repeat);
  robot_phase.setPhaseStep(time_step);
  return robot_phase;
}

namespace example {
  auto robot_phase = initializeExample();
}  // namespace example

/* ************************************************************************* */
TEST(Phase, PhaseCPs) {
  vector<ContactPoints> phase_cps = example::robot_phase.getPhaseCPs();
  EXPECT(assert_equal(12, phase_cps.size()));
  EXPECT(assert_equal((string)"leg_2", (string)phase_cps[9][0].name));
}

TEST(Phase, TransitionCPs) {
  vector<ContactPoints> trans_cps = example::robot_phase.getTransitionCPs();
  EXPECT(assert_equal(11, trans_cps.size()));
  EXPECT(assert_equal(2, trans_cps[1].size()));
  EXPECT(assert_equal((string)"leg_1", (string)trans_cps[1][0].name));
  EXPECT(assert_equal(2, trans_cps[5].size()));
  EXPECT(assert_equal((string)"leg_2", (string)trans_cps[5][0].name));
}

TEST(Phase, PhaseSteps) {
  vector<int> phase_steps = example::robot_phase.getPhaseSteps();
  EXPECT(assert_equal(12, phase_steps.size()));
  int random_number = std::rand()%12;
  EXPECT(assert_equal(20, (double)phase_steps[random_number]));
}

TEST(Phase, CumulativePhaseSteps) {
  vector<int> cum_phase_steps = example::robot_phase.getCumulativePhaseSteps();
  EXPECT(assert_equal(12, cum_phase_steps.size()));
  EXPECT(assert_equal(240, (double)cum_phase_steps[11]));
}

TEST(Phase, ContactOrSwingLinks) {
  const int phase = 5;
  vector<string> phase_contact_links = example::robot_phase.getPhaseContactLinks(phase);
  vector<string> phase_swing_links = example::robot_phase.getPhaseSwingLinks(phase);
  EXPECT(assert_equal(4, phase_contact_links.size() + phase_swing_links.size()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
