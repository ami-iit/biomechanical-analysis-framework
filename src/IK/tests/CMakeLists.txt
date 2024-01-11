
file(GLOB script testIK.ini)
install(FILES ${script} DESTINATION ${CMAKE_BINARY_DIR}/src/IK/tests)

add_baf_test(
  NAME HumanIKTest
  SOURCES HumanIKTest.cpp
  LINKS BiomechanicalAnalysis::IK BipedalLocomotion::ParametersHandlerYarpImplementation)
