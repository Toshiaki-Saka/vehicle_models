# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
#
# Builds vehicle_models_py._core, the pybind11 wrapper around the header-only
# C++ library. Not built by default; enable with -DVEHICLE_MODELS_BUILD_PYTHON=ON
# or install the package with `pip install .` from the repository root.

find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)

# Prefer a pybind11 already installed in the interpreter that is being targeted,
# so `pip install .` and a manual CMake build resolve to the same headers.
if(NOT pybind11_DIR)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _pybind11_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _pybind11_found)
  if(_pybind11_found EQUAL 0)
    set(pybind11_DIR "${_pybind11_dir}")
  endif()
endif()

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core MODULE src/core_module.cpp)
target_link_libraries(_core PRIVATE vehicle_models)
target_compile_features(_core PRIVATE cxx_std_17)

if(MSVC)
  # /utf-8: the headers carry UTF-8 comments and the default code page mangles
  # them (C4819). /bigobj: the tire variants instantiate every model three times.
  target_compile_options(_core PRIVATE /utf-8 /bigobj /W4)
else()
  target_compile_options(_core PRIVATE -Wall -Wextra)
endif()

# Land next to the package sources for an in-place build, so `python run_gui.py`
# works straight out of the build tree.
set_target_properties(_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py"
  LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_models_py")

# Used by scikit-build-core when building a wheel.
install(TARGETS _core DESTINATION vehicle_models_py)
