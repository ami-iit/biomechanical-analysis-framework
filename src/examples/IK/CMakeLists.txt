
find_package(matioCpp REQUIRED)

add_executable(exampleIK)

target_sources(exampleIK PRIVATE exampleIK.cpp)

find_package(matioCpp REQUIRED)

target_link_libraries(exampleIK PRIVATE BiomechanicalAnalysis::IK BiomechanicalAnalysis::CommonConversions BiomechanicalAnalysis::Logging matioCpp::matioCpp BipedalLocomotion::ParametersHandlerYarpImplementation YARP::YARP_os)
