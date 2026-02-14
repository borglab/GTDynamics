include_directories(${CMAKE_SOURCE_DIR}/tests)

gtsamAddTestsGlob(statics "test*.cpp" "" "gtdynamics")
