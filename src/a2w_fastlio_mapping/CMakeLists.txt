cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)
find_package(GTSAM 4.1 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(keyframe_manager src/keyframe_manager.cpp)
target_include_directories(keyframe_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(keyframe_manager a2w_fastlio_common)

add_library(frontend_message_adapter src/frontend_message_adapter.cpp)
target_include_directories(frontend_message_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frontend_message_adapter
  a2w_fastlio_common nav_msgs pcl_conversions sensor_msgs)

add_library(pose_graph_optimizer src/pose_graph_optimizer.cpp)
target_include_directories(pose_graph_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_graph_optimizer gtsam)
ament_target_dependencies(pose_graph_optimizer a2w_fastlio_common)

add_library(loop_pipeline src/loop_pipeline.cpp)
target_include_directories(loop_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(loop_pipeline pose_graph_optimizer)
ament_target_dependencies(loop_pipeline a2w_fastlio_common)

add_executable(mapping_ingress_node src/mapping_ingress_node.cpp)
target_link_libraries(mapping_ingress_node keyframe_manager frontend_message_adapter)
ament_target_dependencies(mapping_ingress_node message_filters nav_msgs rclcpp sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS keyframe_manager frontend_message_adapter pose_graph_optimizer loop_pipeline
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mapping_ingress_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(a2w_fastlio_common GTSAM)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_keyframe_manager test/test_keyframe_manager.cpp)
  target_link_libraries(test_keyframe_manager keyframe_manager)

  ament_add_gtest(test_frontend_message_adapter test/test_frontend_message_adapter.cpp)
  target_link_libraries(test_frontend_message_adapter frontend_message_adapter)

  ament_add_gtest(test_pose_graph_optimizer test/test_pose_graph_optimizer.cpp)
  target_link_libraries(test_pose_graph_optimizer pose_graph_optimizer)

  ament_add_gtest(test_pose_graph_synthetic_loop test/test_pose_graph_synthetic_loop.cpp)
  target_link_libraries(test_pose_graph_synthetic_loop pose_graph_optimizer)

  ament_add_gtest(test_loop_pipeline test/test_loop_pipeline.cpp)
  target_link_libraries(test_loop_pipeline loop_pipeline)

  add_launch_test(test/test_mapping_ingress_launch.py
    TIMEOUT 30
    ENV
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
endif()

ament_package()
