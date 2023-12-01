
add_biomechanical_analysis_library(
    NAME                   IK
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/IK/InverseKinematic.h
    SOURCES                src/InverseKinematic.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::IK)
