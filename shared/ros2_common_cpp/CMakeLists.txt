cmake_minimum_required(VERSION 3.16)
project(ros2_common_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(prometheus-cpp CONFIG REQUIRED)
# imgproc: the grey erosion and dilation of the depth edge mask. Optional: the depth_mask target is built only where
# OpenCV is installed (the D435 nodes); map_guard_cpp and map3d_manager_cpp build this package without it. The test
# build (scripts/cpp-node-test.sh, CI) requires it, so the mask is always tested.
find_package(OpenCV QUIET COMPONENTS core imgproc)
if(BUILD_TESTING AND NOT OpenCV_FOUND)
  message(FATAL_ERROR "ros2_common_cpp tests need OpenCV (core, imgproc) for depth_mask")
endif()

# Plain libraries without the ROS client library: a node's pure-logic target links them without ROS.
add_library(config src/config.cpp)
add_library(${PROJECT_NAME}::config ALIAS config)
target_include_directories(config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(config PUBLIC yaml-cpp::yaml-cpp)
set_target_properties(config PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_config POSITION_INDEPENDENT_CODE ON)

add_library(metrics src/metrics.cpp src/process_collector.cpp)
add_library(${PROJECT_NAME}::metrics ALIAS metrics)
target_include_directories(metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(metrics PUBLIC prometheus-cpp::core prometheus-cpp::pull)
set_target_properties(metrics PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_metrics POSITION_INDEPENDENT_CODE ON)

set(EXPORTED_LIBRARIES config metrics)
set(EXPORTED_DEPENDENCIES yaml-cpp prometheus-cpp)
if(OpenCV_FOUND)
  add_library(depth_mask src/depth_mask.cpp)
  add_library(${PROJECT_NAME}::depth_mask ALIAS depth_mask)
  target_include_directories(depth_mask PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(depth_mask SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(depth_mask PUBLIC opencv_core opencv_imgproc)
  set_target_properties(depth_mask PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_depth_mask POSITION_INDEPENDENT_CODE ON)
  list(APPEND EXPORTED_LIBRARIES depth_mask)
  list(APPEND EXPORTED_DEPENDENCIES OpenCV)
else()
  message(STATUS "OpenCV not found: ros2_common_cpp::depth_mask is not built")
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${EXPORTED_LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns). Every in-repo C++ package sits two levels below the repo root
  # (shared/<pkg>, nodes/<pkg>), and scripts/cpp-node-test.sh keeps that layout inside its workspace.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  # Only the lint packages named as test_depend in package.xml run (clang-format, cppcheck, xmllint); no copyright.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config config)
  ament_add_gtest(test_metrics test/test_metrics.cpp)
  target_link_libraries(test_metrics metrics)
  ament_add_gtest(test_depth_mask test/test_depth_mask.cpp)
  target_link_libraries(test_depth_mask depth_mask)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${EXPORTED_DEPENDENCIES})
ament_package()
