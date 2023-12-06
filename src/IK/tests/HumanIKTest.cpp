// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>


TEST_CASE("InverseKinematic test")
{
    // set the number of DoFs
    int nrDoFs = 3;

    // inintialize the joint positions and velocities
    Eigen::VectorXd q(nrDoFs);
    Eigen::VectorXd qDot(nrDoFs);

    Eigen::VectorXd qInitial(nrDoFs);
    BiomechanicalAnalysis::IK::HumanIK ik;

    qInitial.setConstant(0.0);

    ik.setDoFsNumber(nrDoFs);
    ik.setDt(0.1);
    ik.setInitialJointPositions(qInitial);
    ik.setState();
    ik.advance();
    ik.getJointPositions(q);
    ik.getJointVelocities(qDot);
    std::cout << "q = " << q.transpose() << std::endl;
    std::cout << "qDot = " << qDot.transpose() << std::endl;
}
