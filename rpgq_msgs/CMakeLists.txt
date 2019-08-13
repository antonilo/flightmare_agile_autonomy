cmake_minimum_required(VERSION 2.8.3)
project(rpgq_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

cs_install()
cs_export()
