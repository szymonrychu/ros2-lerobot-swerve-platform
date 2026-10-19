cmake_minimum_required(VERSION 3.16)
project(map3d_manager_cpp LANGUAGES CXX)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# The pure logic, without the ROS client library: config and the command/result JSON contract.
add_library(map3d_manager_core STATIC
  src/config.cpp
  src/command_json.cpp
  src/status.cpp)
target_include_directories(map3d_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(map3d_manager_core PUBLIC
  ros2_common_cpp::config
  yaml-cpp::yaml-cpp
  nlohmann_json::nlohmann_json)
set_target_properties(map3d_manager_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS wiring: the node library and the standalone executable (single-threaded executor).
add_library(map3d_manager_component STATIC src/map3d_manager_node.cpp)
target_include_directories(map3d_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(map3d_manager_component PUBLIC
  map3d_manager_core
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${rtabmap_msgs_TARGETS})
set_target_properties(map3d_manager_component PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(map3d_manager_node src/main.cpp)
target_link_libraries(map3d_manager_node map3d_manager_component)

install(TARGETS map3d_manager_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(name config command_json status)
    ament_add_gtest(test_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_${name} map3d_manager_core)
  endforeach()
  # The ROS wiring against in-process fake RTAB-Map services.
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 120)
  target_link_libraries(test_node map3d_manager_component)
endif()

ament_package()
