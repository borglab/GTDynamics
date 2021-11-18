#include <gtdynamics/pandarobot/ikfast/ikfastwrapper.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <iostream>
#include <vector>

using namespace gtsam;
using gtsam::assert_equal;

TEST(IkFastWrapper, Forward) {
    // Create ikfastwrapper object
    panda::IkFastWrapper pandarobot = panda::IkFastWrapper();

    // Should I put this in two different tests?
    // Simple case: all joints with value 0
    vector<double> joints_simple = {0,0,0,0,0,0,0};
    Vector test_solution_simple = (Vector(12) << 1,0,0,0.088,0,-1,0,0,0,0,-1,1.033).finished(); //do I need to put the .finished()?

    vector<double> res_simple = pandarobot.forward(joints_simple);
    Vector result_simple = Vector::Map(&res_simple[0], res_simple.size());
    EXPECT(assert_equal(test_solution_simple, result_simple, 1e-5))

    // Complex case: random joint values
    vector<double> joints = {-1.9205802374693666, -0.07431401149295525, 1.9902035554092978, 
                    -2.068096770889272, 2.64181363620466, 3.505368647887169, -1.1871823345782546};
    Vector test_solution = 
        (Vector(12) << 0.0224721, -0.747238, 0.664177,  0.403041, 
                      -0.967494,  -0.18364, -0.173871, -0.00106015,
                       0.251893,  -0.63868, -0.727075,  0.480041).finished();

    vector<double> res = pandarobot.forward(joints);
    Vector result = Vector::Map(&res[0], res.size());

    // compare with solution with EXPECT
    EXPECT(assert_equal(test_solution, result, 1e-5))
}

TEST(IkFastWrapper, Inverse) {
    // Create ikfastwrapper object
    panda::IkFastWrapper pandarobot = panda::IkFastWrapper();

    vector<double> pose={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.25};
    vector<double> free_params = {0.3};
    Matrix test_solution = (
        Matrix(8,7) <<  2.84159, 0.581145, 3.14159, 2.3909, -3.14159, -0.169549, 0.3,
                        -0.3, -0.581145, -3.55271e-15, 2.3909, -3.14159, -0.169549, 0.3,
                        -0.3, 2.2104, 3.14159, 2.3909, 0, -1.45971, 0.3,
                        2.84159, -2.2104, -3.55271e-15, 2.3909, 0, -1.45971, 0.3,
                        2.84159, 0.070394, -2.53556e-14, 2.95828, -7.10543e-15, 0.253705, 0.3,
                        -0.3, -0.070394, 3.14159, 2.95828, -7.10543e-15, 0.253705, 0.3,
                        -0.3, 1.69965, -6.70103e-15, 2.95828, 3.14159, -1.88296, 0.3,
                        2.84159, -1.69965, 3.14159, 2.95828, 3.14159, -1.88296, 0.3
        ).finished();

    vector<vector<double>> res = pandarobot.inverse(pose, free_params);
    Matrix result(res.size(),res[0].size());
    for(unsigned int i=0; i<res.size(); ++i)
        for(unsigned int j=0; j<res[0].size();++j){
            result(i,j) = res[i][j];
        };

    EXPECT(assert_equal(test_solution, result, 1e-5));
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
