include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(kinematics "test*.cpp" "" "gtdynamics")
