
add_biomechanical_analysis_library(
    NAME                   IK
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/IK/InverseKinematics.h
    SOURCES                src/InverseKinematics.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::IK BipedalLocomotion::ParametersHandler
    SUBDIRECTORIES         tests)
