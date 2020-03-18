cmake_minimum_required(VERSION 2.8.3)
project(rpgq_quad_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  rospy
  std_msgs
  catkin_simple REQUIRED
)

catkin_python_setup()
catkin_simple()


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_export()