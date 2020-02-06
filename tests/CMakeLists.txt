gtsamAddTestsGlob(tests "test*.cpp" "" "gtdynamics")

option(DEBUG "Turn on debug mode to print out more info?" OFF)

if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    add_definitions(-DDEBUG=0)
endif()
