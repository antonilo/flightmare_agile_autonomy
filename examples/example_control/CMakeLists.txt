cmake_minimum_required(VERSION 2.8.3)
project(example_control)

find_package(catkin_simple REQUIRED)
catkin_simple(
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB_RECURSE PROJECT_SOURCE
    "*.h"
    "*.hpp"
    "*.cpp"
)

cs_add_library(fig8_trajectory
  src/trajectory/fig8_trajectory.cpp)

# make the executable
cs_add_executable(quad 
  src/quad.cpp )
cs_add_executable(multi_quad
  src/multi_quad.cpp)
cs_add_executable(quad_with_camera
  src/quad_with_camera.cpp)
cs_add_executable(quad_with_stereo_camera 
  src/quad_with_stereo_camera.cpp )
cs_add_executable(multi_quad_with_camera 
  src/multi_quad_with_camera.cpp)

# link the executable to the necessary libraries
target_link_libraries(quad
  ${catkin_LIBRARIES}
  fig8_trajectory
  zmq
  zmqpp
)

# link the executable to the necessary libraries
target_link_libraries(multi_quad
  ${catkin_LIBRARIES}
  fig8_trajectory
  zmq
  zmqpp
)

# link the executable to the necessary libraries
target_link_libraries(quad_with_camera
  ${catkin_LIBRARIES}
  fig8_trajectory
  zmq
  zmqpp
)
# link the executable to the necessary libraries
target_link_libraries(quad_with_stereo_camera
  ${catkin_LIBRARIES}
  fig8_trajectory
  zmq
  zmqpp
)
# link the executable to the necessary libraries
target_link_libraries(multi_quad_with_camera
  ${catkin_LIBRARIES}
  fig8_trajectory
  zmq
  zmqpp
)

if (catkin_LIBRARIES MATCHES "polynomial_trajectories")
  cs_add_executable(quad_waypoints 
    src/quad_waypoints.cpp)
  # link the executable to the necessary libraries
  target_link_libraries(quad_waypoints
    ${catkin_LIBRARIES}
    zmq
    zmqpp)
endif(catkin_LIBRARIES MATCHES "polynomial_trajectories")

cs_install()