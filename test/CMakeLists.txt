add_executable(test_ik test_ik.cpp)
target_link_libraries(test_ik ${catkin_LIBRARIES})

add_executable(test_bimanual test_bimanual.cpp)
target_link_libraries(test_bimanual ${catkin_LIBRARIES})

add_executable(test_grasping test_grasping.cpp)
target_link_libraries(test_grasping ${catkin_LIBRARIES})

add_executable(test_check_ik test_check_ik.cpp)
target_link_libraries(test_check_ik ${catkin_LIBRARIES})

add_executable(home_utility home_utility.cpp)
target_link_libraries(home_utility ${catkin_LIBRARIES})

add_executable(test_scene_objects test_scene_objects.cpp)
target_link_libraries(test_scene_objects ${catkin_LIBRARIES})

add_executable(generate_grasp generate_grasp.cpp)
target_link_libraries(generate_grasp ${catkin_LIBRARIES})

add_executable(test_multithread test_multithread.cpp)
target_link_libraries(test_multithread ${catkin_LIBRARIES})

add_executable(test_ik_check_capability test_ik_check_capability.cpp)
target_link_libraries(test_ik_check_capability ${catkin_LIBRARIES} ${PROJECT_NAME}_ik_check)

add_executable(test_frame_interpolation test_frame_interpolation.cpp)
target_link_libraries(test_frame_interpolation ${catkin_LIBRARIES})

add_executable(test_multiple_plans test_multiple_plans.cpp)
target_link_libraries(test_multiple_plans ${catkin_LIBRARIES})
