cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# ROS requirements
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# build our plugin
add_library(pendulum_plugin SHARED pendulum_plugin.cc)
target_link_libraries(pendulum_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
