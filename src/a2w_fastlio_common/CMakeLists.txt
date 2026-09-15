cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

set(SCANCONTEXT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/scancontext_tro")

add_library(a2w_scan_context
  src/scan_context_place_recognition.cpp
  src/scan_context_index.cpp
  ${SCANCONTEXT_ROOT}/src/scan_context.cpp
)
target_include_directories(a2w_scan_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${SCANCONTEXT_ROOT}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(a2w_scan_context PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen ${PCL_LIBRARIES} a2w_scan_context)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${SCANCONTEXT_ROOT}/include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} a2w_scan_context EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types ${PROJECT_NAME})

  ament_add_gtest(test_place_recognition_types test/test_place_recognition_types.cpp)
  target_link_libraries(test_place_recognition_types ${PROJECT_NAME})

  ament_add_gtest(test_scan_context_descriptor test/test_scan_context_descriptor.cpp)
  target_link_libraries(test_scan_context_descriptor ${PROJECT_NAME})

  ament_add_gtest(test_scan_context_index test/test_scan_context_index.cpp)
  target_link_libraries(test_scan_context_index ${PROJECT_NAME})
endif()

ament_package()
