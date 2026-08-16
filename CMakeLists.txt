# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
cmake_minimum_required(VERSION 3.16)

project(vehicle_models
        VERSION 0.1.0
        DESCRIPTION "Header-only C++17 vehicle dynamics model library"
        LANGUAGES CXX)

# Treat as a subproject when pulled in via add_subdirectory / FetchContent.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(VEHICLE_MODELS_IS_TOP_LEVEL ON)
else()
  set(VEHICLE_MODELS_IS_TOP_LEVEL OFF)
endif()

option(VEHICLE_MODELS_BUILD_TESTS    "Build unit tests"  ${VEHICLE_MODELS_IS_TOP_LEVEL})
option(VEHICLE_MODELS_BUILD_EXAMPLES "Build examples"    ${VEHICLE_MODELS_IS_TOP_LEVEL})
option(VEHICLE_MODELS_INSTALL        "Generate install rules" ${VEHICLE_MODELS_IS_TOP_LEVEL})
option(VEHICLE_MODELS_WERROR         "Treat warnings as errors in tests/examples" OFF)

# ---------------------------------------------------------------------------
# The library itself (header-only)
# ---------------------------------------------------------------------------
include(GNUInstallDirs)   # must precede the INSTALL_INTERFACE include dir

add_library(vehicle_models INTERFACE)
add_library(vehicle_models::vehicle_models ALIAS vehicle_models)

target_include_directories(vehicle_models INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(vehicle_models INTERFACE cxx_std_17)

# Warning flags applied only to our own targets, never exported to users.
add_library(vehicle_models_warnings INTERFACE)
if(MSVC)
  target_compile_options(vehicle_models_warnings INTERFACE /W4 /permissive-)
  if(VEHICLE_MODELS_WERROR)
    target_compile_options(vehicle_models_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(vehicle_models_warnings INTERFACE
                         -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  if(VEHICLE_MODELS_WERROR)
    target_compile_options(vehicle_models_warnings INTERFACE -Werror)
  endif()
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(VEHICLE_MODELS_BUILD_TESTS)
  enable_testing()
  foreach(test_name IN ITEMS test_kinematics test_dynamics test_integrator)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE vehicle_models vehicle_models_warnings)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(VEHICLE_MODELS_BUILD_EXAMPLES)
  foreach(example_name IN ITEMS step_steer handling_report)
    add_executable(${example_name} examples/${example_name}.cpp)
    target_link_libraries(${example_name} PRIVATE vehicle_models vehicle_models_warnings)
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Install / export
# ---------------------------------------------------------------------------
if(VEHICLE_MODELS_INSTALL)
  include(CMakePackageConfigHelpers)

  install(TARGETS vehicle_models EXPORT vehicle_modelsTargets)
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  install(EXPORT vehicle_modelsTargets
          FILE vehicle_modelsTargets.cmake
          NAMESPACE vehicle_models::
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vehicle_models)

  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/vehicle_modelsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/vehicle_modelsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vehicle_models)

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/vehicle_modelsConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    ARCH_INDEPENDENT)

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/vehicle_modelsConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/vehicle_modelsConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vehicle_models)
endif()
