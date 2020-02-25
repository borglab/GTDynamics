cmake_minimum_required(VERSION 3.0)
project(gtdynamics_demo C CXX)

#####################
# Find Dependencies #
#####################

# For parsing urdf/sdf files.
find_package(sdformat8 REQUIRED)

# GTSAM.
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

# Boost - same requirement as gtsam.
find_package(Boost 1.43 REQUIRED)
find_package(Boost COMPONENTS filesystem system thread program_options serialization regex timer REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Find GTDynamics. TODO: Change this path to the folder of your built library.
set(gtdynamics_lib_path "$ENV{GTDYNAMICS_INSTALL_PATH}/lib")
find_library(GTDYNAMICS NAMES gtdynamics PATHS ${gtdynamics_lib_path})

####################
# Build Executable #
####################

add_executable(exec test.cpp)
target_link_libraries(exec PUBLIC ${GTDYNAMICS} ${GTSAM_LIBS} ${SDFormat_LIBRARIES})

# Include headers. TODO: Change this path to the location of the installed headers.
set(gtdynamics_headers_path "$ENV{GTDYNAMICS_INSTALL_PATH}/include/")
target_include_directories(exec PUBLIC ${gtdynamics_headers_path} ${SDFormat_INCLUDE_DIRS})