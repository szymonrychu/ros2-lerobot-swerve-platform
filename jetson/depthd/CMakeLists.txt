cmake_minimum_required(VERSION 3.20)
project(depthd LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall$<SEMICOLON>-Wextra$<SEMICOLON>-Wpedantic>)

# ---- dependencies (Jetson Nano: Ubuntu 20.04, L4T R32.6.1, gcc 8.4, CUDA 10.2, TensorRT 8.0.1) ----
find_package(OpenCV 4.8 REQUIRED COMPONENTS
  core imgproc calib3d cudaarithm cudawarping cudastereo cudaimgproc cudafilters ximgproc)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

set(TEGRA_LIB_DIR /usr/lib/aarch64-linux-gnu/tegra)
set(CUDA_HOME /usr/local/cuda CACHE PATH "CUDA toolkit root")
find_path(NVBUF_INCLUDE_DIR nvbuf_utils.h HINTS /usr/src/jetson_multimedia_api/include REQUIRED)
find_library(NVBUF_UTILS_LIBRARY nvbuf_utils HINTS ${TEGRA_LIB_DIR} REQUIRED)
find_library(EGL_LIBRARY EGL REQUIRED)
find_path(CUDA_INCLUDE_DIR cuda.h HINTS ${CUDA_HOME}/include REQUIRED)
find_library(CUDA_DRIVER_LIBRARY cuda HINTS ${CUDA_HOME}/lib64/stubs ${TEGRA_LIB_DIR} REQUIRED)
find_library(CUDA_RUNTIME_LIBRARY cudart HINTS ${CUDA_HOME}/lib64 REQUIRED)
find_path(NVINFER_INCLUDE_DIR NvInfer.h REQUIRED)
find_library(NVINFER_LIBRARY nvinfer REQUIRED)

find_package(vpi 1.1 REQUIRED)  # NVIDIA VPI 1.1 (C API, CUDA backend), target "vpi"

# libSGM 3.1.0 (pinned tag), built for the Nano's sm_53 only, no samples or tests.
include(FetchContent)
set(CMAKE_CUDA_COMPILER ${CUDA_HOME}/bin/nvcc CACHE FILEPATH "CUDA compiler for libSGM and src/*.cu")
# depthd's own kernels (post-processing): nvcc 10.2 stops at C++14, sm_53, and the runtime stays shared like the
# cudart linked below.
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_ARCHITECTURES 53)
set(CMAKE_CUDA_RUNTIME_LIBRARY Shared)
enable_language(CUDA)
set(CUDA_ARCHS 53 CACHE STRING "libSGM device code architectures" FORCE)
set(ENABLE_SAMPLES OFF CACHE BOOL "" FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(ENABLE_ZED_DEMO OFF CACHE BOOL "" FORCE)
set(LIBSGM_SHARED OFF CACHE BOOL "" FORCE)
set(BUILD_OPENCV_WRAPPER OFF CACHE BOOL "" FORCE)
FetchContent_Declare(libsgm
  GIT_REPOSITORY https://github.com/fixstars/libSGM.git
  GIT_TAG 3.1.0
  GIT_SHALLOW TRUE
  PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_libsgm.cmake)
FetchContent_MakeAvailable(libsgm)

# Shared wire format (standard library + libzstd), built from source so the same gcc 8 compiles it as on the Pi.
get_filename_component(ROS2_COMMON_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/ros2_common_cpp ABSOLUTE)
add_library(depth_wire STATIC ${ROS2_COMMON_CPP_DIR}/src/depth_wire.cpp)
target_include_directories(depth_wire PUBLIC ${ROS2_COMMON_CPP_DIR}/include)
target_link_libraries(depth_wire PUBLIC PkgConfig::ZSTD)

# ---- libdepthd: every src/*.cpp except the two mains ----
file(GLOB DEPTHD_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu)
list(FILTER DEPTHD_SOURCES EXCLUDE REGEX "/(main|bench_main)\\.cpp$")
add_library(depthd_lib STATIC ${DEPTHD_SOURCES})
set_target_properties(depthd_lib PROPERTIES OUTPUT_NAME depthd)
target_include_directories(depthd_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}
  ${NVBUF_INCLUDE_DIR} ${CUDA_INCLUDE_DIR} ${NVINFER_INCLUDE_DIR})
target_link_libraries(depthd_lib PUBLIC
  ${OpenCV_LIBS} yaml-cpp nlohmann_json::nlohmann_json depth_wire
  PkgConfig::ZMQ PkgConfig::ZSTD PkgConfig::GST vpi sgm
  ${NVBUF_UTILS_LIBRARY} ${EGL_LIBRARY} ${CUDA_DRIVER_LIBRARY} ${CUDA_RUNTIME_LIBRARY} ${NVINFER_LIBRARY}
  stdc++fs pthread)

# ---- executables, only when their sources exist ----
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
  add_executable(depthd src/main.cpp)
  target_link_libraries(depthd PRIVATE depthd_lib)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/bench_main.cpp)
  add_executable(depthd-bench src/bench_main.cpp)
  target_link_libraries(depthd-bench PRIVATE depthd_lib)
endif()

# ---- tests: one gtest binary per test/test_*.cpp ----
enable_testing()
file(GLOB DEPTHD_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
foreach(test_src ${DEPTHD_TESTS})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name} PRIVATE depthd_lib GTest::gtest GTest::gtest_main)
  target_compile_definitions(${test_name} PRIVATE DEPTHD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
