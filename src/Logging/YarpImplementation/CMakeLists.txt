
if(FRAMEWORK_COMPILE_YarpImplementation)

  add_biomechanical_analysis_library(
    NAME                   TextLoggingYarpImplementation
    SOURCES                src/YarpLogger.cpp
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/Logging/YarpLogger.h
    PRIVATE_LINK_LIBRARIES BiomechanicalAnalysis::Logging 
                           BipedalLocomotion::TextLoggingYarpImplementation
                           YARP::YARP_os
    INSTALLATION_FOLDER    Logging)

endif()
