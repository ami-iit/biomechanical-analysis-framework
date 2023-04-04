# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(BiomechanicalAnalysisFrameworkFindDependencies)

########################## Mandatory dependencies ###############################
# Find all packages

find_package(BipedalLocomotionFramework 0.12.0 REQUIRED)

find_package(Eigen3 3.2.92 REQUIRED)

########################## Optional dependencies  ##############################

find_package(Catch2 3 QUIET)

find_package(YARP QUIET)
option(FRAMEWORK_COMPILE_YarpImplementation "Compile utilities for YARP" ${YARP_FOUND})

##########################      Components       ##############################
framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "Catch2_FOUND;BUILD_TESTING" OFF)
