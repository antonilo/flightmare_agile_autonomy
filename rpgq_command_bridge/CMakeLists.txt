cmake_minimum_required(VERSION 2.8.3)
project(rpgq_command_bridge)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

# add files
file(GLOB_RECURSE RPGQ_COMMAND_BRIDGE_SOURCE
    "*.h"
    "*.c"
    "*.hpp"
    "*.cpp"
)

# generate executable
cs_add_executable(${PROJECT_NAME} ${RPGQ_COMMAND_BRIDGE_SOURCE})

# generate library
cs_add_library(${PROJECT_NAME}_LIB ${RPGQ_COMMAND_BRIDGE_SOURCE})

# link with the necessary libraries
target_link_libraries(${PROJECT_NAME}_LIB ${catkin_LIBRARIES})

cs_install()
cs_export()
