/**
 * @file  testUtils.cpp
 * @brief test utils functions
 * @Author: Mandy Xie and Alejandro Escontrela
 */

#include <gtsam/base/numericalDerivative.h>
#include <utils.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <string>
#include <limits.h>
#include <unistd.h>
#include <algorithm>

using namespace std;
using namespace gtsam;
using namespace manipulator;
using namespace robot;

// Test unit_twist function
TEST(utils, unit_twist) {
  Vector3 w(0, 0, 1);
  Vector3 p(1, 0, 0);
  Vector6 expected_twist = (Vector(6) << 0, 0, 1, 0, -1, 0).finished();
  auto actual_twist = unit_twist(w, p);
  EXPECT(assert_equal(expected_twist, actual_twist, 1e-6));
}

// Test calculate system transition matrix for GP
TEST(utils, calcPhi) {
  double t = 0.1;
  Matrix expected_phi =
      (Matrix(3, 3) << 1, t, 0.5 * t * t, 0, 1, t, 0, 0, 1).finished();
  auto actual_phi = calcPhi(t);
  EXPECT(assert_equal(expected_phi, actual_phi, 1e-6));
}

// Test calculate covariance matrix for GP
TEST(utils, calcQ) {
  double t = 0.1;
  Matrix Qc = I_6x6;
  Matrix expected_Q =
      (Matrix(3 * Qc.rows(), 3 * Qc.rows()) << 1.0 / 20 * pow(t, 5.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 8 * pow(t, 4.0) * Qc, 1.0 / 3 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, 1.0 / 6 * pow(t, 3.0) * Qc,
       1.0 / 2 * pow(t, 2.0) * Qc, t * Qc)
          .finished();

  auto actual_Q = calcQ(Qc, t);
  EXPECT(assert_equal(expected_Q, actual_Q, 1e-6));
}

// Load a URDF file and ensure its joints and links were parsed correctly.
TEST(utils, load_and_parse_urdf_file) {
  // Load the file and parse URDF structure.
  std::string simple_urdf_str = load_file_into_string("../../../urdfs/test/simple_urdf.urdf");
  auto simple_urdf = get_urdf(simple_urdf_str);

  // Check that physical and inertial properties were properly parsed..
  EXPECT(assert_equal(2, simple_urdf->links_.size()));
  EXPECT(assert_equal(1, simple_urdf->joints_.size()));

  EXPECT(assert_equal(100, simple_urdf->links_["l1"]->inertial->mass));
  EXPECT(assert_equal(15, simple_urdf->links_["l2"]->inertial->mass));

  EXPECT(assert_equal(3, simple_urdf->links_["l1"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l1"]->inertial->iyy));
  EXPECT(assert_equal(1, simple_urdf->links_["l1"]->inertial->izz));

  EXPECT(assert_equal(1, simple_urdf->links_["l2"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l2"]->inertial->iyy));
  EXPECT(assert_equal(3, simple_urdf->links_["l2"]->inertial->izz));
}

// Load a URDF file with a loop and ensure its joints and links were parsed correctly.
TEST(utils, load_and_parse_urdf_file_with_loop) {
  // Load the file and parse URDF structure.
  std::string simple_urdf_str = load_file_into_string("../../../urdfs/test/simple_urdf_loop.urdf");
  auto simple_urdf = get_urdf(simple_urdf_str);

  // Check that physical and inertial properties were properly parsed..
  EXPECT(assert_equal(4, simple_urdf->links_.size()));
  EXPECT(assert_equal(4, simple_urdf->joints_.size()));

  EXPECT(assert_equal(100, simple_urdf->links_["l0"]->inertial->mass));
  EXPECT(assert_equal(100, simple_urdf->links_["l1"]->inertial->mass));
  EXPECT(assert_equal(15, simple_urdf->links_["l2"]->inertial->mass));
  EXPECT(assert_equal(15, simple_urdf->links_["l3"]->inertial->mass));

  EXPECT(assert_equal(3, simple_urdf->links_["l0"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l0"]->inertial->iyy));
  EXPECT(assert_equal(1, simple_urdf->links_["l0"]->inertial->izz));

  EXPECT(assert_equal(3, simple_urdf->links_["l1"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l1"]->inertial->iyy));
  EXPECT(assert_equal(1, simple_urdf->links_["l1"]->inertial->izz));

  EXPECT(assert_equal(1, simple_urdf->links_["l2"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l2"]->inertial->iyy));
  EXPECT(assert_equal(3, simple_urdf->links_["l2"]->inertial->izz));

  EXPECT(assert_equal(1, simple_urdf->links_["l3"]->inertial->ixx));
  EXPECT(assert_equal(2, simple_urdf->links_["l3"]->inertial->iyy));
  EXPECT(assert_equal(3, simple_urdf->links_["l3"]->inertial->izz));


  EXPECT(assert_equal("j3", simple_urdf->links_["l3"]->parent_joint->name));

  std::vector<std::string> l1_child_link_names;
  for (auto it = simple_urdf->links_["l1"]->child_links.begin();
        it != simple_urdf->links_["l1"]->child_links.end(); it++)
    l1_child_link_names.push_back((*it)->name);

  std::vector<std::string> l1_child_joint_names;
  for (auto it = simple_urdf->links_["l1"]->child_joints.begin();
        it != simple_urdf->links_["l1"]->child_joints.end(); it++)
    l1_child_joint_names.push_back((*it)->name);
  
  // Check that l2 and l3 are children links of l1
  EXPECT(assert_equal(1, std::count(l1_child_link_names.begin(), l1_child_link_names.end(), "l2")));
  EXPECT(assert_equal(1, std::count(l1_child_link_names.begin(), l1_child_link_names.end(), "l3")));

  // Check that j1 and j3 are children joints of l1.
  EXPECT(assert_equal(1, std::count(l1_child_joint_names.begin(), l1_child_joint_names.end(), "j1")));
  EXPECT(assert_equal(1, std::count(l1_child_joint_names.begin(), l1_child_joint_names.end(), "j3")));
}

TEST(utils, load_and_parse_sdf_file) {
  std::string simple_sdf_path = "../../../sdfs/test/simple.sdf";
  auto simple_sdf = get_sdf(simple_sdf_path);

  std::cout << simple_sdf.Name() << std::endl;

  EXPECT(assert_equal(1, simple_sdf.LinkCount()));
  EXPECT(assert_equal(0, simple_sdf.JointCount()));
}

TEST(utils, load_and_parse_sdf_world_file) {
  std::string simple_sdf_path = "../../../sdfs/test/simple_rr.sdf";
  auto simple_sdf = get_sdf(simple_sdf_path, "simple_rr_sdf");

  std::cout << simple_sdf.Name() << std::endl;

  EXPECT(assert_equal(3, simple_sdf.LinkCount()));
  EXPECT(assert_equal(3, simple_sdf.JointCount()));

  sdf::Link l0 = *simple_sdf.LinkByName("link_0");
  sdf::Link l1 = *simple_sdf.LinkByName("link_1");

  EXPECT(assert_equal(0.05, l0.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l0.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l0.Inertial().Moi()(2, 2)));

  EXPECT(assert_equal(0.05, l1.Inertial().Moi()(0, 0)));
  EXPECT(assert_equal(0.06, l1.Inertial().Moi()(1, 1)));
  EXPECT(assert_equal(0.03, l1.Inertial().Moi()(2, 2)));


  // std::cout << l1.Inertial().Moi()(0, 0) << std::endl;

}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
