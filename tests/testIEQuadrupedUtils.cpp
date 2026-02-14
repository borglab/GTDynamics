#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(IEQuadrupedUtils, general) {
  std::string str1 = "fr_hip";
  std::string str2 = "fl_hip";
  std::string str3 = "rr_hip";
  std::string str4 = "rl_hip";

  EXPECT(!IEVision60Robot::isLeft(str1));
  EXPECT(IEVision60Robot::isLeft(str2));
  EXPECT(!IEVision60Robot::isLeft(str3));
  EXPECT(IEVision60Robot::isLeft(str4));
  EXPECT(IEVision60Robot::isRight(str1));
  EXPECT(!IEVision60Robot::isRight(str2));
  EXPECT(IEVision60Robot::isRight(str3));
  EXPECT(!IEVision60Robot::isRight(str4));

  EXPECT(IEVision60Robot::counterpart(str1) == str2);
  EXPECT(IEVision60Robot::counterpart(str2) == str1);
  EXPECT(IEVision60Robot::counterpart(str3) == str4);
  EXPECT(IEVision60Robot::counterpart(str4) == str3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
