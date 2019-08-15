cmake_minimum_required(VERSION 2.8.3)
project(example_control)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB_RECURSE PROJECT_SOURCE
    "*.h"
    "*.hpp"
    "*.cpp"
)

# make the executable
cs_add_executable(single_agent src/single_agent.cpp src/trajectory/trajectory.cpp)
cs_add_executable(single_stereo_agent src/single_stereo_agent.cpp src/trajectory/trajectory.cpp)
cs_add_executable(multi_agent src/multi_agent.cpp src/trajectory/trajectory.cpp)
# link the executable to the necessary libraries
target_link_libraries(single_agent
        ${catkin_LIBRARIES}
        zmq
        zmqpp
)

# link the executable to the necessary libraries
target_link_libraries(multi_agent
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)

cs_install()