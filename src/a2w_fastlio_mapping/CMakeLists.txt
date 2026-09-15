cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)

add_library(keyframe_manager src/keyframe_manager.cpp)
target_include_directories(keyframe_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(keyframe_manager a2w_fastlio_common)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS keyframe_manager
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
endif()

ament_package()
