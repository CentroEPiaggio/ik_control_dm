cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_trajectory_planning)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(SOURCE_FILES
  src/main.cpp
)

add_executable(dual_manipulation_trajectory_planning ${SOURCE_FILES})

target_link_libraries(dual_manipulation_trajectory_planning ${catkin_LIBRARIES})