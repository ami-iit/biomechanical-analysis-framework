// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelTestUtils.h>
#include <manif/SO3.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <ConfigFolderPath.h>

TEST_CASE("InverseKinematic test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);
    auto paramHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    auto gravityTaskParamHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();

    REQUIRE(paramHandler->setFromFile(getConfigPath() + "/configTestIK.ini"));
    REQUIRE(gravityTaskParamHandler->setFromFile(getConfigPath() + "/gravityTaskTest.ini"));
    paramHandler->setGroup("GRAVITY_TASK_1", gravityTaskParamHandler);

    // inintialize the joint positions and velocities
    Eigen::VectorXd JointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd JointVelocities(kinDyn->getNrOfDegreesOfFreedom());

    Eigen::VectorXd qInitial(kinDyn->getNrOfDegreesOfFreedom());
    BiomechanicalAnalysis::IK::HumanIK ik;

    manif::SO3d I_R_IMU;
    manif::SO3Tangentd I_omega_IMU;
    I_R_IMU.setRandom();
    I_omega_IMU.setRandom();

    qInitial.setConstant(0.0);

    REQUIRE(ik.initialize(paramHandler, kinDyn));
    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.setNodeSetPoint(3, I_R_IMU, I_omega_IMU));
    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));

    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;
}
