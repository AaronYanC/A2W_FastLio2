cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
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

install(DIRECTORY include/ DESTINATION include)
install(TARGETS keyframe_manager frontend_message_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(a2w_fastlio_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_keyframe_manager test/test_keyframe_manager.cpp)
  target_link_libraries(test_keyframe_manager keyframe_manager)

  ament_add_gtest(test_frontend_message_adapter test/test_frontend_message_adapter.cpp)
  target_link_libraries(test_frontend_message_adapter frontend_message_adapter)
endif()

ament_package()
