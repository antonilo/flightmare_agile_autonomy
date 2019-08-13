cmake_minimum_required(VERSION 2.8.3)
project(example_multiagent)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB_RECURSE PROJECT_SOURCE
    "*.h"
    "*.hpp"
    "*.cpp"
)

# make the executable
cs_add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE}
)

# link the executable to the necessary libraries
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)

cs_install()