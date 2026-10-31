cmake_minimum_required(VERSION 3.16)
project(arm_drive_wait_bt_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json 3 REQUIRED)

# The pure logic, without ROS: every src/core/*.cpp, picked up by the glob below.
file(GLOB ARM_DRIVE_WAIT_CORE_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(arm_drive_wait_core STATIC ${ARM_DRIVE_WAIT_CORE_SOURCES})
target_include_directories(arm_drive_wait_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(arm_drive_wait_core PUBLIC nlohmann_json::nlohmann_json)
set_target_properties(arm_drive_wait_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The BT plugin bt_navigator loads by name (plugin_lib_names: arm_drive_wait_bt_node -> libarm_drive_wait_bt_node.so).
add_library(arm_drive_wait_bt_node SHARED src/wait_for_arm_drive_ready.cpp)
target_link_libraries(arm_drive_wait_bt_node arm_drive_wait_core)
ament_target_dependencies(arm_drive_wait_bt_node rclcpp std_msgs behaviortree_cpp)
target_compile_definitions(arm_drive_wait_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS arm_drive_wait_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per test/core/test_<name>.cpp, named after the file stem.
  file(GLOB ARM_DRIVE_WAIT_CORE_TESTS CONFIGURE_DEPENDS test/core/test_*.cpp)
  foreach(test_source ${ARM_DRIVE_WAIT_CORE_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source} TIMEOUT 120)
    target_link_libraries(${test_name} arm_drive_wait_core)
  endforeach()

  # The plugin in a BT factory (registerFromPlugin of the built library), ticked against a test publisher.
  # The repo NavigateToPose tree (nodes/nav2_bringup, copied next to this package by scripts/cpp-node-test.sh) is
  # loaded with the nav2 built-in plugins and compared with the stock nav2_bt_navigator tree.
  find_package(nav2_behavior_tree REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_bt_node test/test_bt_node.cpp TIMEOUT 300)
  ament_target_dependencies(test_bt_node rclcpp std_msgs behaviortree_cpp nav2_behavior_tree ament_index_cpp)
  add_dependencies(test_bt_node arm_drive_wait_bt_node)
  target_compile_definitions(test_bt_node PRIVATE
    ARM_DRIVE_WAIT_BT_LIBRARY="$<TARGET_FILE:arm_drive_wait_bt_node>"
    ARM_WAIT_BT_XML="${CMAKE_CURRENT_SOURCE_DIR}/../nav2_bringup/behavior_trees/navigate_to_pose_arm_wait.xml")
endif()

ament_package()
