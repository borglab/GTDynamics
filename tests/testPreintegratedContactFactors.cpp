/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPreintegratedContactFactors.cpp
 * @brief Tests for the various preintegrated contact factors.
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <math.h>

#include <iostream>

#include "gtdynamics/factors/PreintegratedContactFactors.h"

using namespace gtdynamics;
using gtsam::assert_equal;

TEST(PreintegratedPointContactMeasurements, Constructor) {
  PreintegratedPointContactMeasurements();
  PreintegratedPointContactMeasurements(gtsam::I_3x3*0.1);
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
