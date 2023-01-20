
add_biomechanical_analysis_library(
    NAME                   Logging
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/Logging/Logger.h
    SOURCES                src/Logger.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::TextLogging
    SUBDIRECTORIES         YarpImplementation)
