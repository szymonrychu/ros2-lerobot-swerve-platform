# mcp_core_cpp: one nanobind extension module, built by scikit-build-core into mcp_server's uv venv.
# Each kernel family is a directory src/<family>/ whose .cpp files are collected here, so porting a kernel adds files
# only under its own family directory (bind.cpp holds register_<family>, called once from src/module.cpp).
cmake_minimum_required(VERSION 3.26...4.4)
project(${SKBUILD_PROJECT_NAME} VERSION ${SKBUILD_PROJECT_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The kernel families, in registration order (src/module.cpp registers the same list).
set(MCP_CORE_FAMILIES camera grasp ik_reach collision frontiers)

find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)

set(MCP_CORE_SOURCES src/module.cpp)
foreach(family IN LISTS MCP_CORE_FAMILIES)
  file(GLOB family_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/${family}/*.cpp")
  if(NOT family_sources)
    message(FATAL_ERROR "family ${family}: no sources in src/${family}/ (bind.cpp with register_${family} is required)")
  endif()
  list(APPEND MCP_CORE_SOURCES ${family_sources})
endforeach()

nanobind_add_module(mcp_core_cpp NB_STATIC ${MCP_CORE_SOURCES})
target_include_directories(mcp_core_cpp PRIVATE src)
target_compile_options(mcp_core_cpp PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(mcp_core_cpp PRIVATE
  MCP_CORE_VERSION="${SKBUILD_PROJECT_VERSION}"
  MCP_CORE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  MCP_CORE_NANOBIND_VERSION="${nanobind_VERSION}")

install(TARGETS mcp_core_cpp LIBRARY DESTINATION .)
