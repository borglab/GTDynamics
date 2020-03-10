gtsamAddTestsGlob(scripts "*.cpp" "" "gtdynamics")

add_executable(gerry00 gerry00_ForwardDynamicsPrismatic.cpp)
target_link_libraries(gerry00 PUBLIC gtdynamics ${GTSAM_LIBS} ${SDFormat_LIBRARIES})

option(DEBUG "Turn on debug mode to print out more info?" OFF)

if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    add_definitions(-DDEBUG=0)
endif()

