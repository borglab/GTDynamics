/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTimer.cpp
 * @brief Test Timer.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/Timer.h>
#include <thread>

using namespace gtsam;

TEST(Timer, Timer) {
  Timer timer;
  timer.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  timer.stop();

  EXPECT(timer.seconds() >= 0.01);
  EXPECT(timer.milliSeconds() >= 10.0);
  EXPECT(timer.microSeconds() >= 10000.0);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
