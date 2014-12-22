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



add_executable(test_ik test/test_ik.cpp)

target_link_libraries(test_ik ${catkin_LIBRARIES})

add_dependencies(test_ik dual_manipulation_shared_gencpp)