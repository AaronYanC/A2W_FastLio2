cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)
find_package(a2w_fastlio_map REQUIRED)
find_package(a2w_fastlio_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM 4.1 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

add_library(map_odom_manager src/map_odom_manager.cpp)
target_include_directories(map_odom_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_odom_manager a2w_fastlio_common)

add_library(optimized_map_builder src/optimized_map_builder.cpp)
target_include_directories(optimized_map_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(optimized_map_builder a2w_fastlio_common)

add_library(global_tf_ownership src/global_tf_ownership.cpp)
target_include_directories(global_tf_ownership PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(map_bundle_service src/map_bundle_service.cpp)
target_include_directories(map_bundle_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_bundle_service a2w_fastlio_map)

add_executable(mapping_ingress_node src/mapping_ingress_node.cpp)
target_link_libraries(mapping_ingress_node keyframe_manager frontend_message_adapter)
ament_target_dependencies(mapping_ingress_node message_filters nav_msgs rclcpp sensor_msgs)

add_executable(mapping_backend_node src/mapping_backend_node.cpp)
target_link_libraries(mapping_backend_node
  frontend_message_adapter global_tf_ownership loop_pipeline map_bundle_service map_odom_manager
  optimized_map_builder pose_graph_optimizer)
ament_target_dependencies(mapping_backend_node
  a2w_fastlio_msgs geometry_msgs message_filters nav_msgs pcl_conversions rclcpp sensor_msgs tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS
  keyframe_manager frontend_message_adapter pose_graph_optimizer loop_pipeline map_odom_manager
  optimized_map_builder global_tf_ownership
  map_bundle_service
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mapping_ingress_node mapping_backend_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(a2w_fastlio_common a2w_fastlio_map a2w_fastlio_msgs GTSAM)

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

  ament_add_gtest(test_map_odom_manager test/test_map_odom_manager.cpp)
  target_link_libraries(test_map_odom_manager map_odom_manager)

  ament_add_gtest(test_optimized_map_builder test/test_optimized_map_builder.cpp)
  target_link_libraries(test_optimized_map_builder optimized_map_builder)

  ament_add_gtest(test_global_tf_ownership test/test_global_tf_ownership.cpp)
  target_link_libraries(test_global_tf_ownership global_tf_ownership)

  ament_add_gtest(test_map_bundle_service test/test_map_bundle_service.cpp)
  target_link_libraries(test_map_bundle_service map_bundle_service)

  add_launch_test(test/test_mapping_ingress_launch.py
    TIMEOUT 30
    ENV
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )

  add_launch_test(test/test_mapping_outputs_launch.py
    TIMEOUT 45
    ENV
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
endif()

ament_package()
