
find_package(matioCpp REQUIRED)

add_executable(exampleIK)

target_sources(exampleIK PRIVATE exampleIK.cpp)

target_link_libraries(exampleIK PRIVATE BiomechanicalAnalysis::IK matioCpp::matioCpp BipedalLocomotion::ParametersHandlerYarpImplementation)
