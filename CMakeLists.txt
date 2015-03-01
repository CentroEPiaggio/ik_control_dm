cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_ik_control)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS 
  rviz
  roscpp
  tf
  tf_conversions
  kdl_parser
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  dual_manipulation_shared
)

catkin_package(INCLUDE_DIRS "include" 
)
include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(${CMAKE_INSTALL_PREFIX}/include) #this line is to find lemon include files!!
link_directories(${catkin_LIBRARY_DIRS})

set(SOURCE_FILES
  src/ik_control.cpp
  src/ros_server.cpp
  src/main.cpp
)

add_executable(dual_manipulation_ik_control ${SOURCE_FILES})

target_link_libraries(dual_manipulation_ik_control ${catkin_LIBRARIES} ${kdl_codyco_LIBRARIES}
  /opt/ros/indigo/lib/libmoveit_kdl_kinematics_plugin.so
)

# add_dependencies(dual_manipulation_ik_control dual_manipulation_shared_gencpp)



add_executable(test_ik test/test_ik.cpp)
target_link_libraries(test_ik ${catkin_LIBRARIES})
# add_dependencies(test_ik dual_manipulation_shared_gencpp)

add_executable(test_bimanual test/test_bimanual.cpp)
target_link_libraries(test_bimanual ${catkin_LIBRARIES})
# add_dependencies(test_bimanual dual_manipulation_shared_gencpp)

add_executable(test_grasping test/test_grasping.cpp)
target_link_libraries(test_grasping ${catkin_LIBRARIES})
# add_dependencies(test_grasping dual_manipulation_shared_gencpp)

add_executable(test_check_ik test/test_check_ik.cpp)
target_link_libraries(test_check_ik ${catkin_LIBRARIES})
# add_dependencies(test_check_ik dual_manipulation_shared_gencpp)

add_executable(home_utility test/home_utility.cpp)
target_link_libraries(home_utility ${catkin_LIBRARIES})
add_dependencies(home_utility dual_manipulation_shared_gencpp)

add_executable(test_scene_objects test/test_scene_objects.cpp)
target_link_libraries(test_scene_objects ${catkin_LIBRARIES})
add_dependencies(test_scene_objects dual_manipulation_shared_gencpp)

add_executable(generate_grasp test/generate_grasp.cpp)
target_link_libraries(generate_grasp ${catkin_LIBRARIES})
add_dependencies(generate_grasp dual_manipulation_shared_gencpp)
