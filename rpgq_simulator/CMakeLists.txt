cmake_minimum_required(VERSION 2.8.3)
project(rpgq_simulator)

find_package(catkin_simple REQUIRED
        roscpp
        sensor_msgs
        message_generation)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
catkin_simple(ALL_DEPS_REQUIRED)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries)

configure_file(configuration/root_directory.h.in configuration/root_directory.h)
include_directories(${CMAKE_BINARY_DIR}/configuration)

# glad
set(GLAD_DIR ${LIB_DIR}/glad)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

# add files
file(GLOB_RECURSE RPGQ_SIMULATOR_SOURCE
    "*.h"
    "*.c"
    "*.hpp"
    "*.cpp"
    "*.vert"
    "*.frag"
)

# generate library
cs_add_library(${PROJECT_NAME} ${RPGQ_SIMULATOR_SOURCE})

# link with the necessary libraries
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${GLAD_DIR}/include)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CMAKE_DL_LIBS}
    glfw
    zmq
    zmqpp
)

cs_install()
cs_export()
