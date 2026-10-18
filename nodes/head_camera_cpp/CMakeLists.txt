cmake_minimum_required(VERSION 3.16)
project(head_camera_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_common_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# The pure logic, without libcamera or rclcpp: config, crop geometry, encoding, zoom JSON and state machine.
# Globbed because the crop, encoding and JSON tasks each add files under src/core and test/core in parallel; a
# hand-listed source set would make every one of them edit this file. The libcamera adapter and the ROS node are
# separate explicit targets, because they link libcamera and rclcpp.
file(GLOB HEAD_CAMERA_CORE_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(head_camera_core STATIC ${HEAD_CAMERA_CORE_SOURCES})
target_include_directories(head_camera_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(head_camera_core PUBLIC
  ros2_common_cpp::config
  ros2_common_cpp::metrics
  yaml-cpp::yaml-cpp
  nlohmann_json::nlohmann_json
  opencv_core
  opencv_imgproc
  opencv_imgcodecs)
target_include_directories(head_camera_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(head_camera_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per test/core/test_<name>.cpp, named after the file stem.
  file(GLOB HEAD_CAMERA_CORE_TESTS CONFIGURE_DEPENDS test/core/test_*.cpp)
  foreach(test_source ${HEAD_CAMERA_CORE_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source} TIMEOUT 120)
    target_link_libraries(${test_name} head_camera_core)
    target_compile_definitions(${test_name} PRIVATE
      HEAD_CAMERA_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()
endif()

ament_package()
