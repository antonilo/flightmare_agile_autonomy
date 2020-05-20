cmake_minimum_required(VERSION 2.8.3)
project(example_pcd)

find_package(catkin_simple REQUIRED COMPONENTS)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS 
)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add files
file(GLOB_RECURSE EXAMPLE_VISION_SOURCE
  "*.h"
  "*.c"
  "*.hpp"
  "*.cpp"
)

cs_add_executable(pcd_pub 
  src/pcd_pub.cpp)

#
target_link_libraries(
  pcd_pub
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)
cs_install()


