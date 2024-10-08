# Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# Detect if we are doing a standalone build of the bindings, using an external bipedal-locomotion-framework
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE ON)
  set(FRAMEWORK_COMPILE_PYTHON_BINDINGS ON)
else()
  set(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE FALSE)
endif()

if(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE)
  cmake_minimum_required(VERSION 3.16)
  project(BiomechanicalAnalysisFrameworkBindings)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 REQUIRED)
  find_package(BiomechanicalAnalysisFramework REQUIRED)
  get_filename_component(CMAKE_CURRENT_PARENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_PARENT_SOURCE_DIR}/cmake)
  include(AddBiomechanicalAnalysisPythonModule)
endif()

if(FRAMEWORK_COMPILE_PYTHON_BINDINGS)

    set(NEW_LINE "\n")

    option(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES
        "Do you want BAF to detect and use the active site-package directory? (it could be a system dir)"
        FALSE)

    if(NOT DEFINED FRAMEWORK_PYTHON_INSTALL_DIR)
      if(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
        set(FRAMEWORK_PYTHON_INSTALL_DIR ${Python3_SITELIB})
      else()
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import os;import sysconfig;relative_site_packages = sysconfig.get_path('purelib').replace(sysconfig.get_path('data'), '').lstrip(os.path.sep);print(relative_site_packages)"
                        OUTPUT_VARIABLE _PYTHON_INSTDIR)
        string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
        set(FRAMEWORK_PYTHON_INSTALL_DIR ${_PYTHON_INSTDIR_CLEAN})
      endif()
    endif()
    set(PYTHON_INSTDIR ${FRAMEWORK_PYTHON_INSTALL_DIR}/biomechanical_analysis_framework)


    # Folder of the Python package within the build tree.
    # It is used for the Python tests.
    set(BAF_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/biomechanical_analysis_framework")

    # Add the bindings directory
    add_subdirectory(python)

    # Create the __init__.py file
    file(GENERATE
         OUTPUT "${BAF_PYTHON_PACKAGE}/__init__.py"
         CONTENT "from .bindings import *${NEW_LINE}")

    # Install the __init__.py file
    install(FILES "${BAF_PYTHON_PACKAGE}/__init__.py"
            DESTINATION ${PYTHON_INSTDIR})

    # Install pip metadata files to ensure that BAF installed via CMake is listed by pip list
    # See https://packaging.python.org/specifications/recording-installed-packages/
    # and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
    option(BAF_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to off if some other tool already installs it." ON)
    mark_as_advanced(BAF_PYTHON_PIP_METADATA_INSTALL)
    set(BAF_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
    mark_as_advanced(BAF_PYTHON_PIP_METADATA_INSTALLER)
    if(BAF_PYTHON_PIP_METADATA_INSTALL)
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1${NEW_LINE}")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: biomechanical_analysis_framework${NEW_LINE}")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${BiomechanicalAnalysisFramework_VERSION}${NEW_LINE}")
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${BAF_PYTHON_PIP_METADATA_INSTALLER}${NEW_LINE}")
      install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
        DESTINATION ${FRAMEWORK_PYTHON_INSTALL_DIR}/biomechanical_analysis_framework-${BiomechanicalAnalysisFramework_VERSION}.dist-info)
    endif()

endif()
