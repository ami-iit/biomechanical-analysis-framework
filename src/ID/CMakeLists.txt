
add_biomechanical_analysis_library(
    NAME                   ID
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/ID/InverseDynamics.h
    SOURCES                src/InverseDynamics.cpp
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-estimation iDynTree::idyntree-high-level
    PRIVATE_LINK_LIBRARIES BiomechanicalAnalysis::Logging
    SUBDIRECTORIES         tests)
