cmake_minimum_required(VERSION 3.0)
project(example C CXX)

# gtsam
find_package(GTSAM REQUIRED)

# gtdynamics
find_path(gtdynamics_include gtdynamics)
find_library(gtdynamics_lib gtdynamics)
# these next 3 lines are temporary until we create a proper cmake module for gtdynamics.
set(SDFormat_VERSION 12)
find_package(sdformat${SDFormat_VERSION} REQUIRED)
list(APPEND gtdynamics_lib ${SDFormat_LIBRARIES})

# Build Executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${gtdynamics_lib} gtsam)
target_include_directories(${PROJECT_NAME} PUBLIC ${gtdynamics_include})
