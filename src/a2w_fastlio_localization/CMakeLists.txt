cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)
find_package(a2w_fastlio_map REQUIRED)
find_package(a2w_fastlio_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/global_relocalizer.cpp
  src/local_map_selector.cpp
  src/map_matcher.cpp
  src/localization_manager.cpp
  src/localization_monitor.cpp
  src/relocalization_worker.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} a2w_fastlio_common a2w_fastlio_map a2w_fastlio_msgs)

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${PROJECT_NAME})
ament_target_dependencies(localization_node
  a2w_fastlio_msgs geometry_msgs message_filters nav_msgs pcl_conversions rclcpp sensor_msgs tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(a2w_fastlio_common a2w_fastlio_map a2w_fastlio_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_local_map_selector test/test_local_map_selector.cpp)
  target_link_libraries(test_local_map_selector ${PROJECT_NAME})
  ament_add_gtest(test_map_matcher test/test_map_matcher.cpp)
  target_link_libraries(test_map_matcher ${PROJECT_NAME})
  ament_add_gtest(test_localization_manager test/test_localization_manager.cpp)
  target_link_libraries(test_localization_manager ${PROJECT_NAME})
  ament_add_gtest(test_localization_monitor test/test_localization_monitor.cpp)
  target_link_libraries(test_localization_monitor ${PROJECT_NAME})
  ament_add_gtest(test_global_relocalizer test/test_global_relocalizer.cpp)
  target_link_libraries(test_global_relocalizer ${PROJECT_NAME})
  ament_add_gtest(test_relocalization_sequences test/test_relocalization_sequences.cpp)
  target_link_libraries(test_relocalization_sequences ${PROJECT_NAME})
  ament_add_gtest(test_relocalization_worker test/test_relocalization_worker.cpp)
  target_link_libraries(test_relocalization_worker ${PROJECT_NAME})

  add_executable(create_test_bundle test/create_test_bundle.cpp)
  target_link_libraries(create_test_bundle a2w_fastlio_map::a2w_fastlio_map)
  install(TARGETS create_test_bundle DESTINATION lib/${PROJECT_NAME})
  add_launch_test(test/test_localization_launch.py
    TIMEOUT 45
    ENV
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
  add_launch_test(test/test_global_relocalization_launch.py
    TIMEOUT 60
    ENV
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
      "ROS_DOMAIN_ID=149"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
endif()

ament_package()
