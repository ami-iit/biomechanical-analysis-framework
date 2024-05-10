
add_biomechanical_analysis_library(
    NAME                   ID
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/ID/InverseDynamics.h
    SOURCES                src/InverseDynamics.cpp
    PRIVATE_LINK_LIBRARIES BiomechanicalAnalysis::Logging)
