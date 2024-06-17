
cmake_minimum_required(VERSION 3.5)

project(InverseKinematicsExample)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

find_package(matioCpp REQUIRED)
if(NOT BipedalLocomotionFramework_FOUND)
  find_package(BipedalLocomotionFramework 0.16.0 REQUIRED)
endif()
if(NOT FRAMEWORK_COMPILE_examples)
    find_package(BiomechanicalAnalysisFramework REQUIRED)
endif()

add_executable(exampleID)

target_sources(exampleID PRIVATE exampleID.cpp)

find_package(matioCpp REQUIRED)

target_link_libraries(exampleID PRIVATE BiomechanicalAnalysis::ID BiomechanicalAnalysis::CommonConversions BiomechanicalAnalysis::Logging matioCpp::matioCpp BipedalLocomotion::ParametersHandlerYarpImplementation)
