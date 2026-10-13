cmake_minimum_required(VERSION 3.16)
project(map_guard_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -ffp-contract=off: no fused multiply-add, so the floating point results equal numpy's (golden fixtures).
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_common_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The pure logic, without the ROS client library: agreement, detectors, guard, timing, backup, agent, config.
add_library(map_guard_core STATIC
  src/agreement.cpp
  src/detectors.cpp
  src/guard.cpp
  src/timing.cpp
  src/backup.cpp
  src/agent.cpp
  src/config.cpp
  src/process_metrics.cpp)
target_include_directories(map_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(map_guard_core PUBLIC
  ros2_common_cpp::config
  ros2_common_cpp::metrics
  yaml-cpp::yaml-cpp
  nlohmann_json::nlohmann_json
  PkgConfig::HTTPLIB)
set_target_properties(map_guard_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS wiring: an rclcpp component plus the standalone map_guard_node executable (single-threaded executor).
add_library(map_guard_component SHARED src/map_guard_node.cpp)
target_include_directories(map_guard_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(map_guard_component
  map_guard_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${slam_toolbox_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
rclcpp_components_register_node(map_guard_component
  PLUGIN "map_guard_cpp::MapGuardNode"
  EXECUTABLE map_guard_node
  EXECUTOR SingleThreadedExecutor)

install(TARGETS map_guard_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per Python test file of nodes/map_guard/tests; each also checks the golden fixtures of
  # test/gen_fixtures.py.
  foreach(name timing agreement detectors guard backup agent config)
    ament_add_gtest(test_${name} test/test_${name}.cpp TIMEOUT 120)
    target_link_libraries(test_${name} map_guard_core)
    target_compile_definitions(test_${name} PRIVATE MAP_GUARD_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endforeach()
  # The process_* series of prometheus_client's ProcessCollector (no Python test file: they come with the library).
  ament_add_gtest(test_process_metrics test/test_process_metrics.cpp TIMEOUT 120)
  target_link_libraries(test_process_metrics map_guard_core)
  # The ROS wiring (test_node.py): service calls against a fake slam_toolbox and an end-to-end dry run.
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 120)
  target_link_libraries(test_node map_guard_component)
endif()

ament_package()
