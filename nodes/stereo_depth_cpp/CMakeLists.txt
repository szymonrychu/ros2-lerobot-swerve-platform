cmake_minimum_required(VERSION 3.16)
project(stereo_depth_cpp LANGUAGES CXX)

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Pure logic without rclcpp (config, calibration, matcher, depth conversion, the per-pair gates and metrics), the same
# split as the Python node's config/calibration/depth/messages modules. The gtests and the bench link only this.
add_library(stereo_depth_core
  src/config.cpp
  src/calibration.cpp
  src/depth.cpp
  src/pipeline.cpp)
target_include_directories(stereo_depth_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(stereo_depth_core PUBLIC
  ros2_common_cpp::config ros2_common_cpp::metrics yaml-cpp::yaml-cpp
  opencv_core opencv_imgproc opencv_calib3d)
target_include_directories(stereo_depth_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(stereo_depth_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS node, registered as an rclcpp_components component; the stereo_depth_node executable runs it standalone
# (the deployment: its own process in the background cgroup tier).
add_library(stereo_depth_component SHARED src/stereo_depth_node.cpp)
target_link_libraries(stereo_depth_component PUBLIC
  stereo_depth_core rclcpp::rclcpp rclcpp_components::component message_filters::message_filters
  ${sensor_msgs_TARGETS} ${stereo_msgs_TARGETS} ${std_msgs_TARGETS})
rclcpp_components_register_node(stereo_depth_component
  PLUGIN "stereo_depth_cpp::StereoDepthNode"
  EXECUTABLE stereo_depth_node)

# CPU bench of the C++ depth path on PNG inputs (scripts/bench_compare.py runs it beside the Python path).
add_executable(stereo_depth_bench src/bench_main.cpp)
target_link_libraries(stereo_depth_bench stereo_depth_core opencv_imgcodecs)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS stereo_depth_core stereo_depth_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS stereo_depth_bench RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns); the package sits two levels below the repo root.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  set(FIXTURES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  foreach(name test_config test_depth test_calibration test_node_logic)
    ament_add_gtest(${name} test/${name}.cpp test/fixtures.cpp)
    target_include_directories(${name} PRIVATE test)
    target_compile_definitions(${name} PRIVATE STEREO_DEPTH_FIXTURES_DIR="${FIXTURES_DIR}")
    target_link_libraries(${name} stereo_depth_core opencv_imgcodecs)
  endforeach()
endif()

ament_package()
