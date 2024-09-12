
include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

add_baf_test(
  NAME HumanIKTest
  SOURCES HumanIKTest.cpp
  LINKS BiomechanicalAnalysis::IK BipedalLocomotion::ParametersHandlerTomlImplementation)
