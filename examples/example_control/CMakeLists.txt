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
cs_add_executable(single_vehicle src/single_vehicle.cpp src/trajectory/fig8_trajectory.cpp)
cs_add_executable(multi_vehicle src/multi_vehicle.cpp src/trajectory/fig8_trajectory.cpp)
cs_add_executable(single_vehicle_and_rgb_camera src/single_vehicle_and_rgb_camera.cpp src/trajectory/fig8_trajectory.cpp)
cs_add_executable(single_vehicle_and_stereo_camera src/single_vehicle_and_stereo_camera.cpp src/trajectory/fig8_trajectory.cpp)
cs_add_executable(multi_vehicle_and_rgb_camera src/multi_vehicle_and_rgb_camera.cpp src/trajectory/fig8_trajectory.cpp)
cs_add_executable(single_vehicle_heading_controller src/single_vehicle_heading_controller.cpp src/heading_controller/heading_controller.cpp)


# link the executable to the necessary libraries
target_link_libraries(single_vehicle
        ${catkin_LIBRARIES}
        zmq
        zmqpp
)

# link the executable to the necessary libraries
target_link_libraries(multi_vehicle
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)

# link the executable to the necessary libraries
target_link_libraries(single_vehicle_and_rgb_camera
        ${catkin_LIBRARIES}
        zmq
        zmqpp
        )
# link the executable to the necessary libraries
target_link_libraries(single_vehicle_and_stereo_camera
        ${catkin_LIBRARIES}
        zmq
        zmqpp
        )
# link the executable to the necessary libraries
target_link_libraries(multi_vehicle_and_rgb_camera
        ${catkin_LIBRARIES}
        zmq
        zmqpp
        )
# link the executable to the necessary libraries
target_link_libraries(single_vehicle_heading_controller
        ${catkin_LIBRARIES}
        zmq
        zmqpp
        )


cs_install()