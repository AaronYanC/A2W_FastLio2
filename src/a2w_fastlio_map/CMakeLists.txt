cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(a2w_fastlio_common REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/descriptor_database.cpp
  src/keyframe_database.cpp
  src/map_manager.cpp
  src/map_io.cpp
  src/map_manifest.cpp
  src/map_bundle_reader.cpp
  src/map_bundle_writer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} a2w_fastlio_common)
target_link_libraries(${PROJECT_NAME} OpenSSL::Crypto)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${PCL_LIBRARIES})

add_executable(map_bundle_inspect src/map_bundle_inspect.cpp)
target_link_libraries(map_bundle_inspect ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS map_bundle_inspect DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(a2w_fastlio_common OpenSSL PCL yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_databases test/test_databases.cpp)
  target_link_libraries(test_databases ${PROJECT_NAME})

  ament_add_gtest(test_map_manifest test/test_map_manifest.cpp)
  target_link_libraries(test_map_manifest ${PROJECT_NAME})

  ament_add_gtest(test_map_bundle_round_trip test/test_map_bundle_round_trip.cpp)
  target_link_libraries(test_map_bundle_round_trip ${PROJECT_NAME})

  ament_add_gtest(test_map_bundle_failures test/test_map_bundle_failures.cpp)
  target_link_libraries(test_map_bundle_failures ${PROJECT_NAME})
endif()

ament_package()
