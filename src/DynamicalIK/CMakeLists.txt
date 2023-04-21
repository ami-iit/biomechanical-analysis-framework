
set(H_PREFIX include/BiomechanicalAnalysis/DynamicalIK)

add_biomechanical_analysis_library(
    NAME                   DynamicalIK
    SOURCES                src/InverseVelocityKinematics.cpp
                           src/StdTarget.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/InverseVelocityKinematics.h
                           ${H_PREFIX}/Target.h ${H_PREFIX}/Constraint.h
                           ${H_PREFIX}/StdTarget.h
    PUBLIC_LINK_LIBRARIES  BiomechanicalAnalysis::IK BiomechanicalAnalysis::DataSources
                           BipedalLocomotion::IK BiomechanicalAnalysis::Logging
                           BipedalLocomotion::ContinuousDynamicalSystem)
