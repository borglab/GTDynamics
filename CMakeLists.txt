cmake_minimum_required(VERSION 3.0)
enable_testing()
project(gtdynamics
        LANGUAGES CXX C
        VERSION 1.0.0
        DESCRIPTION "Full kinodynamics constraints for arbitrary robot configurations with factor graphs."
        HOMEPAGE_URL "https://github.com/borglab/GTDynamics")

add_compile_options(-faligned-new)

# ##############################################################################
# Find Dependencies #
# ##############################################################################

# Find GTSAM components.
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
include(GtsamMatlabWrap)

# For unit tests and scripts.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam.
find_package(Boost 1.43 REQUIRED)
find_package(
  Boost
  COMPONENTS filesystem
             system
             thread
             program_options
             serialization
             regex
             timer
  REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
  message(
    "Ignoring Boost restriction on optional lvalue assignment from rvalues")
  add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES
                  -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# For cython wrapper.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# ##############################################################################
# Dynamics library
# ##############################################################################

# Paths to SDF & URDF files.
add_definitions(-DURDF_PATH="${PROJECT_SOURCE_DIR}/urdfs")
add_definitions(-DSDF_PATH="${PROJECT_SOURCE_DIR}/sdfs")

# All subdirectories that contain source code relevant to this library.
set(SOURCE_SUBDIRS dynamics universal_robot factors utils)

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Include sdformat directories.
include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

# Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/gtdynamics/${SOURCE_SUBDIR}/*.cpp)
  list(APPEND sources ${GLOB_RESULT})
  file(GLOB GLOB_RESULT
       ${CMAKE_CURRENT_SOURCE_DIR}/gtdynamics/${SOURCE_SUBDIR}/*.h)
  list(APPEND headers ${GLOB_RESULT})
  install(FILES ${GLOB_RESULT} DESTINATION include/gtdynamics/${SOURCE_SUBDIR})
  add_subdirectory(gtdynamics/${SOURCE_SUBDIR}) # Process subdirectory.
endforeach()

add_library(gtdynamics SHARED ${sources} ${headers})

target_link_libraries(gtdynamics ${GTSAM_LIBS} ${SDFormat_LIBRARIES})
target_include_directories(gtdynamics PUBLIC ${SDFormat_INCLUDE_DIRS})

foreach(SOURCE_SUBDIR ${SOURCE_SUBDIRS})
  target_include_directories(
    gtdynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_SUBDIR})
endforeach()

# Install library and headers.
install(
  TARGETS gtdynamics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
set_target_properties(gtdynamics PROPERTIES LINKER_LANGUAGE CXX)

option(BUILD_PYTHON_WRAPPER "Build Python wrapper" ON)
message(STATUS "Build Python Wrapper: ${BUILD_PYTHON_WRAPPER}")

if(BUILD_PYTHON_WRAPPER)
    find_package(gtwrap)

    set(WRAP_PYTHON_VERSION "Default" CACHE STRING "The Python version to use for wrapping")
    gtwrap_get_python_version(${WRAP_PYTHON_VERSION})
    message(STATUS "Wrapper Python Version: ${WRAP_PYTHON_VERSION}")
    
    set(PYTHON_REQUIREMENTS_PATH "requirements.txt")
    set(PYTHON_TESTS "${PROJECT_NAME}.tests")

    set(GTDYNAMICS_PYTHON_PATH ${PROJECT_SOURCE_DIR}/python/)
    configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/setup.py.in
                   ${GTDYNAMICS_PYTHON_PATH}/setup.py)
    configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/__init__.py.in
                   ${GTDYNAMICS_PYTHON_PATH}/${PROJECT_NAME}/__init__.py)
    configure_file(${GTDYNAMICS_PYTHON_PATH}/templates/${PROJECT_NAME}.tpl
                   ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

    pybind_wrap(${PROJECT_NAME}_py # target
                ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i # interface header file
                "${PROJECT_NAME}.cpp" # the generated cpp
                "${PROJECT_NAME}" # module_name
                "${PROJECT_NAME}" # top namespace in the cpp file
                "${ignore}" # ignore classes
                ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
                ${PROJECT_NAME} # libs
                "${PROJECT_NAME}" # dependencies
                ON # use boost
             )

    set_target_properties(${PROJECT_NAME}_py PROPERTIES
             OUTPUT_NAME "${PROJECT_NAME}"
             LIBRARY_OUTPUT_DIRECTORY "${GTDYNAMICS_PYTHON_PATH}/${PROJECT_NAME}"
             DEBUG_POSTFIX "" # Otherwise you will have a wrong name
             RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
             )
endif()

# Process subdirectories.
add_subdirectory(tests)
add_subdirectory(scripts)
