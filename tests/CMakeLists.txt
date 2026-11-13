include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# The SCATE tests (testScate*, testTrajectoryInit, testUr5*, testUr10e*) use
# the header-only SCATE code in the example folders.
include_directories(${CMAKE_SOURCE_DIR}/examples/example_scate_ur5
                    ${CMAKE_SOURCE_DIR}/examples/example_scate_ur10e)

if(GTDYNAMICS_WITH_SUITESPARSE)
  set(ROOT_TEST_EXCLUDES
      "testPenaltyOptimizer.cpp;testIERetractor.cpp;testIEQuadrupedUtilsManifold.cpp")
else()
  set(ROOT_TEST_EXCLUDES
      "testIPOptOptimizer.cpp;testPenaltyOptimizer.cpp;testIERetractor.cpp;testIEQuadrupedUtilsManifold.cpp")
endif()

gtsamAddTestsGlob(
  tests "test*.cpp"
  "${ROOT_TEST_EXCLUDES}"
  "gtdynamics")
