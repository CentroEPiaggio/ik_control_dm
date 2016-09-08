cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_ik_control)

ENABLE_LANGUAGE(CXX)
SET(CMAKE_BUILD_TYPE "Debug")

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

find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/ik_check_capability
  ${OMPL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}_ik_check
  ${OMPL_LIBRARIES}
)
include_directories(include ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)
# include_directories(${CMAKE_INSTALL_PREFIX}/include) #this line is to find lemon include files!!
link_directories(${catkin_LIBRARY_DIRS}
    ${OMPL_LIBRARY_DIRS}
)

set(IK_CAPABILITIES_FILES
  src/random_planning_capability.cpp
)

set(SOURCE_FILES
  src/trajectory_utils.cpp
  src/ik_check_capability.cpp
  src/scene_object_manager.cpp
  src/ik_control.cpp
  src/ros_server.cpp
  src/main.cpp
  ${IK_CAPABILITIES_FILES}
  src/group_structure_manager.cpp
  src/robot_controller_interface.cpp
)

add_executable(dual_manipulation_ik_control ${SOURCE_FILES})

target_link_libraries(dual_manipulation_ik_control ${catkin_LIBRARIES} ${kdl_codyco_LIBRARIES}
  /opt/ros/indigo/lib/libmoveit_kdl_kinematics_plugin.so
  ${PROJECT_NAME}_ik_check
  ${OMPL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_ik_check)

# add_dependencies(dual_manipulation_ik_control dual_manipulation_shared_gencpp)

add_library(${PROJECT_NAME}_ik_check SHARED src/ik_check_capability.cpp)
target_link_libraries(${PROJECT_NAME}_ik_check ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_ik_check dual_manipulation_shared_gencpp)

install(DIRECTORY include/ik_check_capability
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_ik_check
         EXPORT ${PROJECT_NAME}_ik_check
#         ARCHIVE DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT lib
#         RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT bin
         LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}" COMPONENT shlib
)

add_subdirectory(test)
