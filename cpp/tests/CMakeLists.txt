gtsamAddTestsGlob(tests "test*.cpp" "" "dynamics")

option(DEBUG "Turn on debug mode to print out more info?" ON)

if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    add_definitions(-DDEBUG=0)
endif()
