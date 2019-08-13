cmake_minimum_required(VERSION 2.8.3)
project(rpgq_components)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

# add files
file(GLOB_RECURSE RPGQ_COMPONENTS_SOURCE
    "*.h"
    "*.c"
    "*.hpp"
    "*.cpp"
)

# generate library
cs_add_library(${PROJECT_NAME} ${RPGQ_COMPONENTS_SOURCE})

# link with the necessary libraries
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_install()
cs_export()
