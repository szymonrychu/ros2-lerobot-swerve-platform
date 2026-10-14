# Run from the libSGM source dir by FetchContent (PATCH_COMMAND). libSGM 3.1.0 asks for cxx_std_17, which CMake also
# applies to CUDA, and nvcc 10.2 stops at C++14: keep C++17 for the host sources and C++14 for the .cu files.
# Idempotent: a second run finds nothing to replace.
set(_file src/CMakeLists.txt)
file(READ ${_file} _content)
string(REPLACE "target_compile_features(\${PROJECT_NAME} PRIVATE cxx_std_17)"
  "set_target_properties(\${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 CUDA_STANDARD 14)" _content "${_content}")
file(WRITE ${_file} "${_content}")
