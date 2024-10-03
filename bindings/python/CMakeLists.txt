# Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_subdirectory(type_caster)
add_subdirectory(IK)


include(ConfigureFileWithCMakeIfTarget)

configure_file_with_cmakeif_target(${CMAKE_CURRENT_SOURCE_DIR}/biomechanical_analysis_framework.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/biomechanical_analysis_framework.cpp
  @ONLY)

get_property(pybind_headers GLOBAL PROPERTY pybind_headers)
get_property(pybind_sources GLOBAL PROPERTY pybind_sources)
get_property(pybind_include_dirs GLOBAL PROPERTY pybind_include_dirs)
get_property(pybind_link_libraries GLOBAL PROPERTY pybind_link_libraries)
get_property(pybind_compile_definitions GLOBAL PROPERTY pybind_compile_definitions)

pybind11_add_module(pybind11_baf MODULE
  ${CMAKE_CURRENT_BINARY_DIR}/biomechanical_analysis_framework.cpp
  ${pybind_sources}
  ${pybind_headers}
  )

target_compile_features(pybind11_baf PUBLIC cxx_std_17)
target_include_directories(pybind11_baf PUBLIC "$<BUILD_INTERFACE:${pybind_include_dirs}>")
target_compile_definitions(pybind11_baf PRIVATE ${pybind_compile_definitions})

target_link_libraries(pybind11_baf PRIVATE
  ${pybind_link_libraries})

# # The generated Python dynamic module must have the same name as the pybind11
# # module, i.e. `bindings`.
set_target_properties(pybind11_baf PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${baf_PYTHON_PACKAGE}"
    OUTPUT_NAME "bindings")

if(FRAMEWORK_TEST_PYTHON_BINDINGS)
    add_subdirectory(tests)
endif()

# Output package is:
#
# bipedal_locomotion
# |-- __init__.py (generated from main bindings CMake file)
# `-- bindings.<cpython_extension>
# `-- utils.py

install(TARGETS pybind11_baf DESTINATION ${PYTHON_INSTDIR})
