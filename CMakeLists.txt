cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_ik_control)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)
find_package(kdl_codyco REQUIRED)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(SOURCE_FILES
  src/trajectory_generator.cpp
  src/ros_server.cpp
  src/main.cpp
)

add_executable(dual_manipulation_ik_control ${SOURCE_FILES})

target_link_libraries(dual_manipulation_ik_control ${catkin_LIBRARIES} ${kdl_codyco_LIBRARIES})

add_dependencies(dual_manipulation_ik_control dual_manipulation_shared_gencpp)