
include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

find_package(matioCpp REQUIRED)

add_executable(exampleIK)

target_sources(exampleIK PRIVATE exampleIK.cpp)

find_package(matioCpp REQUIRED)

target_link_libraries(exampleIK PRIVATE BiomechanicalAnalysis::IK BiomechanicalAnalysis::CommonConversions BiomechanicalAnalysis::Logging matioCpp::matioCpp BipedalLocomotion::ParametersHandlerYarpImplementation YARP::YARP_os)
