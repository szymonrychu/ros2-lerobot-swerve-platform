cmake_minimum_required(VERSION 3.16)
# C is enabled for PCL: its VTK config looks for MPI::MPI_C, which needs a C language in the project.
project(ground_estimator_cpp LANGUAGES C CXX)

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
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED COMPONENTS common sample_consensus segmentation search kdtree)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

# The pure logic, without ROS node code: every src/core/*.cpp. Globbed, so adding a source file under src/core needs no
# edit to this file.
file(GLOB GROUND_CORE_SOURCES CONFIGURE_DEPENDS src/core/*.cpp)
add_library(ground_estimator_core STATIC ${GROUND_CORE_SOURCES})
target_include_directories(ground_estimator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ground_estimator_core PUBLIC
  ros2_common_cpp::config
  ros2_common_cpp::metrics
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  opencv_core
  opencv_imgproc
  pcl_common
  pcl_sample_consensus
  pcl_segmentation
  pcl_search
  pcl_kdtree)
target_include_directories(ground_estimator_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_definitions(ground_estimator_core PUBLIC ${PCL_DEFINITIONS})
ament_target_dependencies(ground_estimator_core PUBLIC sensor_msgs std_msgs pcl_conversions)
set_target_properties(ground_estimator_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Root .clang-format (ament style, 120 columns), two levels up as for every in-repo C++ package.
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # One gtest per test/core/test_<name>.cpp, named after the file stem.
  file(GLOB GROUND_CORE_TESTS CONFIGURE_DEPENDS test/core/test_*.cpp)
  foreach(test_source ${GROUND_CORE_TESTS})
    get_filename_component(test_name ${test_source} NAME_WE)
    ament_add_gtest(${test_name} ${test_source} TIMEOUT 120)
    target_link_libraries(${test_name} ground_estimator_core)
    target_compile_definitions(${test_name} PRIVATE
      GROUND_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      GROUND_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endforeach()
endif()

ament_package()
