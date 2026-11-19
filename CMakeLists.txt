cmake_minimum_required(VERSION 3.16)

project(coven LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(COVEN_BUILD_TESTS "Build the C++ tests" ON)
option(COVEN_BUILD_BENCH "Build the benchmark" ON)
option(COVEN_BUILD_PYTHON "Build the Python extension (pip sets this)" OFF)

# VAMP builds with LTO. Archives of LTO objects need the compiler's own ar and ranlib; a plain ar
# without the LTO plugin produces archives that fail to link.
if(CMAKE_CXX_COMPILER_AR AND CMAKE_CXX_COMPILER_RANLIB)
  set(CMAKE_AR "${CMAKE_CXX_COMPILER_AR}")
  set(CMAKE_RANLIB "${CMAKE_CXX_COMPILER_RANLIB}")
endif()

find_package(OpenMP REQUIRED COMPONENTS CXX)

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.1/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=117cbf2711572f113bab262933eb5187b08cfc06dce0714a1ee94f2183ddc3ec
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

# Unmodified upstream VAMP. Keep this commit in step with the vamp-planner pin in pyproject.toml.
CPMAddPackage(
  NAME vamp
  GITHUB_REPOSITORY KavrakiLab/vamp
  GIT_TAG 27cb9b66ebd8d101ce88d330f3b028398afc1cfa
  OPTIONS "VAMP_BUILD_PYTHON_BINDINGS OFF" "VAMP_INSTALL_CPP_LIBRARY OFF" "VAMP_BUILD_TESTS OFF"
)

# Everything here is compiled with VAMP's own flags, so the serial baseline is VAMP as it ships
include(${vamp_SOURCE_DIR}/cmake/CompilerSettings.cmake)

add_library(coven INTERFACE)
target_include_directories(coven INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(coven INTERFACE vamp_cpp OpenMP::OpenMP_CXX)
# Keep Eigen single-threaded, as it is in VAMP
target_compile_definitions(coven INTERFACE EIGEN_DONT_PARALLELIZE)

if(COVEN_BUILD_TESTS)
  CPMAddPackage(
    NAME Catch2
    GITHUB_REPOSITORY catchorg/Catch2
    VERSION 3.7.1
    OPTIONS "CATCH_INSTALL_DOCS OFF" "CATCH_INSTALL_EXTRAS OFF"
  )
  list(APPEND CMAKE_MODULE_PATH "${Catch2_SOURCE_DIR}/extras")
  include(Catch)
  enable_testing()

  add_executable(coven_tests tests/test_pprm.cc tests/test_service.cc)
  target_link_libraries(coven_tests PRIVATE coven Catch2::Catch2WithMain)
  # The tests run every schedule
  target_compile_definitions(coven_tests PRIVATE COVEN_SCHEDULE=runtime)
  catch_discover_tests(coven_tests)
endif()

if(COVEN_BUILD_BENCH)
  add_executable(coven_bench bench/bench.cc)
  target_link_libraries(coven_bench PRIVATE coven)
  # --schedule picks the schedule
  target_compile_definitions(coven_bench PRIVATE COVEN_SCHEDULE=runtime)

  add_executable(coven_load bench/load.cc)
  target_link_libraries(coven_load PRIVATE coven)
endif()

if(COVEN_BUILD_PYTHON)
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)
  # The nanobind commit VAMP's bindings use: extensions built from the same nanobind share their
  # types, so coven._core can take and return vamp.Environment, vamp.panda.Roadmap and the rest.
  CPMAddPackage("gh:wjakob/nanobind#9a25aed8a7edfe60ef9ad1c911e57667bc4916c4")

  nanobind_add_module(_core NB_STATIC STABLE_ABI NOMINSIZE src/python.cc)
  target_link_libraries(_core PRIVATE coven)
  install(TARGETS _core LIBRARY DESTINATION coven)
endif()
