cmake_minimum_required(VERSION 3.16)
project(head_rgbd_cpp LANGUAGES CXX)

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
# imgproc: depth downscale and colour resize. imgcodecs: the JPEG encode of the colour and the decode of the cloud.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)

# The pure logic, without ROS node code: every src/core/*.cpp (config, cloud). Globbed so adding a source file
# under src/core or test/core needs no edit to this file.
file(GLOB HEAD_RGBD_CORE_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(head_rgbd_core STATIC ${HEAD_RGBD_CORE_SOURCES})
target_include_directories(head_rgbd_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(head_rgbd_core PUBLIC
  ros2_common_cpp::config
  ros2_common_cpp::metrics
  yaml-cpp::yaml-cpp
  opencv_core
  opencv_imgproc
  opencv_imgcodecs)
# cloud.cpp: sensor_msgs messages, image_geometry's pinhole model and depth_image_proc's conversions.
ament_target_dependencies(head_rgbd_core PUBLIC sensor_msgs image_geometry depth_image_proc)
target_include_directories(head_rgbd_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(head_rgbd_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS wiring: HeadRgbdNode and the head_rgbd_node executable (single-threaded executor: the sync callback, the
# 1 s lazy timer and the config are never touched concurrently).
add_library(head_rgbd_node_lib STATIC src/head_rgbd_node.cpp)
target_link_libraries(head_rgbd_node_lib PUBLIC head_rgbd_core)
ament_target_dependencies(head_rgbd_node_lib PUBLIC
  rclcpp sensor_msgs message_filters cv_bridge)
set_target_properties(head_rgbd_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(head_rgbd_node src/main.cpp)
target_link_libraries(head_rgbd_node head_rgbd_node_lib)

install(TARGETS head_rgbd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per test/core/test_<name>.cpp, named after the file stem.
  file(GLOB HEAD_RGBD_CORE_TESTS CONFIGURE_DEPENDS test/core/test_*.cpp)
  foreach(test_source ${HEAD_RGBD_CORE_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source} TIMEOUT 120)
    target_link_libraries(${test_name} head_rgbd_core)
    target_compile_definitions(${test_name} PRIVATE HEAD_RGBD_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endforeach()

  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 300)
  target_link_libraries(test_node head_rgbd_node_lib)
endif()

ament_package()
