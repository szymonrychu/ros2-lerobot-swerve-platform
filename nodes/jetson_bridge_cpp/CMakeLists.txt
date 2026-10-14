cmake_minimum_required(VERSION 3.16)
project(jetson_bridge_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_common_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)

# The pure logic, without the ROS client library: wire frame -> message conversion and the staleness policy.
add_library(jetson_bridge_core STATIC src/convert.cpp src/config.cpp)
target_include_directories(jetson_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(jetson_bridge_core PUBLIC
  ros2_common_cpp::depth_wire ros2_common_cpp::config
  ${sensor_msgs_TARGETS})
set_target_properties(jetson_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS node: ZeroMQ subscriber thread, publishers, metrics.
add_library(jetson_bridge_node_lib STATIC src/jetson_bridge_node.cpp)
target_include_directories(jetson_bridge_node_lib SYSTEM PUBLIC ${ZMQ_INCLUDE_DIR})
target_link_libraries(jetson_bridge_node_lib PUBLIC
  jetson_bridge_core ros2_common_cpp::metrics rclcpp::rclcpp ${ZMQ_LIBRARY})
set_target_properties(jetson_bridge_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(jetson_bridge_node src/main.cpp)
target_link_libraries(jetson_bridge_node jetson_bridge_node_lib)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS jetson_bridge_core jetson_bridge_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS jetson_bridge_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_convert test/test_convert.cpp TIMEOUT 120)
  target_link_libraries(test_convert jetson_bridge_core)
  ament_add_gtest(test_config test/test_config.cpp TIMEOUT 120)
  target_link_libraries(test_config jetson_bridge_core)
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 240)
  target_link_libraries(test_node jetson_bridge_node_lib)
endif()

ament_package()
