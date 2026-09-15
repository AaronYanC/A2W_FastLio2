cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters kdtree registration)
find_package(TBB REQUIRED)

set(SCANCONTEXT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/scancontext_tro")
set(QUATRO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/Quatro")
set(NANO_GICP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/nano_gicp")
set(TEASER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/TEASER-plusplus/teaser")
set(PMC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/pmc")

add_library(a2w_pmc STATIC
  ${PMC_ROOT}/pmc_heu.cpp
  ${PMC_ROOT}/pmc_maxclique.cpp
  ${PMC_ROOT}/pmcx_maxclique.cpp
  ${PMC_ROOT}/pmcx_maxclique_basic.cpp
  ${PMC_ROOT}/pmc_cores.cpp
  ${PMC_ROOT}/pmc_utils.cpp
  ${PMC_ROOT}/pmc_graph.cpp
  ${PMC_ROOT}/pmc_clique_utils.cpp
)
set_target_properties(a2w_pmc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(a2w_pmc PUBLIC ${PMC_ROOT}/include)
target_link_libraries(a2w_pmc PUBLIC OpenMP::OpenMP_CXX)

add_library(a2w_teaser_registration STATIC
  ${TEASER_ROOT}/src/registration.cc
  ${TEASER_ROOT}/src/graph.cc
)
set_source_files_properties(${TEASER_ROOT}/src/graph.cc PROPERTIES
  COMPILE_OPTIONS "-include;${CMAKE_CURRENT_SOURCE_DIR}/src/teaser_graph_compat.hpp")
set_target_properties(a2w_teaser_registration PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(a2w_teaser_registration PUBLIC ${TEASER_ROOT}/include)
target_link_libraries(a2w_teaser_registration PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX PRIVATE a2w_pmc)

add_library(a2w_quatro_core STATIC
  ${QUATRO_ROOT}/src/fpfh.cc
  ${QUATRO_ROOT}/src/matcher.cc
  ${QUATRO_ROOT}/src/quatro_module.cc
)
set_target_properties(a2w_quatro_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(a2w_quatro_core PUBLIC
  ${QUATRO_ROOT}/include
  ${TEASER_ROOT}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(a2w_quatro_core PUBLIC
  Eigen3::Eigen ${PCL_LIBRARIES} TBB::tbb a2w_teaser_registration)

add_library(a2w_nano_gicp_core STATIC
  ${NANO_GICP_ROOT}/src/nano_gicp.cc
  ${NANO_GICP_ROOT}/src/nanoflann.cc
)
set_target_properties(a2w_nano_gicp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(a2w_nano_gicp_core PUBLIC
  ${NANO_GICP_ROOT}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(a2w_nano_gicp_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

add_library(a2w_registration SHARED
  src/match_validator.cpp
  src/registration_pipeline.cpp
  src/quatro_registration.cpp
  src/nano_gicp_registration.cpp
)
target_include_directories(a2w_registration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${QUATRO_ROOT}/include ${NANO_GICP_ROOT}/include ${TEASER_ROOT}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(a2w_registration
  PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
  PRIVATE a2w_quatro_core a2w_nano_gicp_core)

add_library(a2w_scan_context
  src/local_map_builder.cpp
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
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen ${PCL_LIBRARIES} a2w_scan_context a2w_registration)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${SCANCONTEXT_ROOT}/include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} a2w_scan_context a2w_registration EXPORT export_${PROJECT_NAME}
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

  ament_add_gtest(test_registration_interfaces test/test_registration_interfaces.cpp)
  target_link_libraries(test_registration_interfaces ${PROJECT_NAME})

  ament_add_gtest(test_local_map_builder test/test_local_map_builder.cpp)
  target_link_libraries(test_local_map_builder ${PROJECT_NAME})

  ament_add_gtest(test_registration_synthetic test/test_registration_synthetic.cpp)
  target_link_libraries(test_registration_synthetic ${PROJECT_NAME})

  ament_add_gtest(test_match_validator test/test_match_validator.cpp)
  target_link_libraries(test_match_validator ${PROJECT_NAME})

  ament_add_gtest(test_registration_pipeline test/test_registration_pipeline.cpp)
  target_link_libraries(test_registration_pipeline ${PROJECT_NAME})
endif()

ament_package()
