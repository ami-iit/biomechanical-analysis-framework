include(FetchContent)

FetchContent_Declare(
  ResolveRoboticsURICpp
  GIT_REPOSITORY https://github.com/ami-iit/resolve-robotics-uri-cpp
  GIT_TAG        v0.0.2
)
FetchContent_MakeAvailable(ResolveRoboticsURICpp)

add_biomechanical_analysis_library(
    NAME                   ID
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/ID/InverseDynamics.h
    SOURCES                src/InverseDynamics.cpp
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-estimation iDynTree::idyntree-high-level BipedalLocomotion::ParametersHandler
    PRIVATE_LINK_LIBRARIES BiomechanicalAnalysis::Logging ResolveRoboticsURICpp::ResolveRoboticsURICpp
    SUBDIRECTORIES         tests)
