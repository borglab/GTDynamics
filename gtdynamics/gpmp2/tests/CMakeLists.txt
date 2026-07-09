include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(gpmp2 "test*.cpp" "" "gtdynamics")
