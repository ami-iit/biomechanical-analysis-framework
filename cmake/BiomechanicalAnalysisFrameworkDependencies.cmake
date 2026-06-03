# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(BiomechanicalAnalysisFrameworkFindDependencies)


# Workaround for issue that occurs with CMake 3.26.1 and pybind11 2.4.3
# see https://github.com/ami-iit/bipedal-locomotion-framework/issues/636
# This is done here as it needs to be done before any call (even transitive)
# to find_package(pybind11)
# It can be removed once pybind11 2.4.3 is not supported anymore
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

########################## Mandatory dependencies ###############################
# Find all packages


find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(BipedalLocomotionFramework 0.19.0 REQUIRED)

# Dependencies supported either directly of via FetchContent
if(NOT DEFINED FRAMEWORK_USE_SYSTEM_ResolveRoboticsURICpp)
  find_package(ResolveRoboticsURICpp QUIET)
endif()
option(FRAMEWORK_USE_SYSTEM_ResolveRoboticsURICpp "Use find_package to find ResolveRoboticsURICpp " ${ResolveRoboticsURICpp_FOUND})
if(FRAMEWORK_USE_SYSTEM_ResolveRoboticsURICpp)
  find_package(ResolveRoboticsURICpp REQUIRED)
else()
  FetchContent_Declare(
    ResolveRoboticsURICpp
    GIT_REPOSITORY https://github.com/ami-iit/resolve-robotics-uri-cpp
    GIT_TAG        v0.0.3
  )
  FetchContent_MakeAvailable(ResolveRoboticsURICpp)
endif()

########################## Optional dependencies  ##############################

find_package(Catch2 3 QUIET)

find_package(YARP QUIET)
option(FRAMEWORK_COMPILE_YarpImplementation "Compile utilities for YARP" ${YARP_FOUND})


find_package(pybind11 2.4.3 CONFIG QUIET)
find_package(Python3 3.6 COMPONENTS Interpreter Development QUIET)


##########################      Components       ##############################
framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "Catch2_FOUND;BUILD_TESTING" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_examples
  "Compile examples?" ON
  "BUILD_EXAMPLES" OFF)
  
framework_dependent_option(FRAMEWORK_COMPILE_PYTHON_BINDINGS
  "Compile the python bindings?" ON
  "Python3_FOUND;pybind11_FOUND" OFF)
