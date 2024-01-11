
file(GLOB script testIK.ini)
install(FILES ${script} DESTINATION ${CMAKE_INSTALL_PREFIX}/HumanIKTest)

add_baf_test(
  NAME HumanIKTest
  SOURCES HumanIKTest.cpp
  LINKS BiomechanicalAnalysis::IK BipedalLocomotion::ParametersHandlerYarpImplementation)
