if(GTDYNAMICS_WITH_SUITESPARSE)
  gtsamAddTestsGlob(
    tests "test*.cpp"
    "testPenaltyOptimizer.cpp;testIERetractor.cpp;testIEQuadrupedUtilsManifold.cpp"
    "gtdynamics")
else()
  gtsamAddTestsGlob(
    tests "test*.cpp"
    "testIPOptOptimizer.cpp;testPenaltyOptimizer.cpp;testIERetractor.cpp;testIEQuadrupedUtilsManifold.cpp"
    "gtdynamics")
endif()
