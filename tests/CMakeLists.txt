include_directories(${CMAKE_CURRENT_SOURCE_DIR})

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
