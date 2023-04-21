
set(H_PREFIX include/BiomechanicalAnalysis/IK)

add_biomechanical_analysis_library(
    IS_INTERFACE
    NAME                   IK
    PUBLIC_HEADERS         ${H_PREFIX}/Solver.h ${H_PREFIX}/IntegrationBasedSolver.h
                           ${H_PREFIX}/HumanStateSource.h
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen)
