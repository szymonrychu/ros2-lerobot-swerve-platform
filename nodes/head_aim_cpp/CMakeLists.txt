cmake_minimum_required(VERSION 3.16)
project(head_aim_cpp LANGUAGES CXX)

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
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# The pure logic, without ROS types: every src/core/*.cpp, picked up by the glob below.
file(GLOB HEAD_AIM_CORE_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(head_aim_core STATIC ${HEAD_AIM_CORE_SOURCES})
target_include_directories(head_aim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(head_aim_core PUBLIC ros2_common_cpp::config yaml-cpp::yaml-cpp)
set_target_properties(head_aim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The ROS wiring: HeadAimNode and the head_aim_node executable (single-threaded executor: the input callbacks and the
# aim timer never run concurrently).
add_library(head_aim_node_lib STATIC src/head_aim_node.cpp)
target_link_libraries(head_aim_node_lib PUBLIC head_aim_core)
ament_target_dependencies(head_aim_node_lib PUBLIC
  rclcpp nav_msgs sensor_msgs action_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
set_target_properties(head_aim_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(head_aim_node src/main.cpp)
target_link_libraries(head_aim_node head_aim_node_lib)

install(TARGETS head_aim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per test/core/test_<name>.cpp, named after the file stem.
  file(GLOB HEAD_AIM_CORE_TESTS CONFIGURE_DEPENDS test/core/test_*.cpp)
  foreach(test_source ${HEAD_AIM_CORE_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source} TIMEOUT 120)
    target_link_libraries(${test_name} head_aim_core)
    target_compile_definitions(${test_name} PRIVATE HEAD_AIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endforeach()

  # HeadAimNode in-process with a probe node and static TF.
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 300)
  target_link_libraries(test_node head_aim_node_lib)
endif()

ament_package()
