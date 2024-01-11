// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelTestUtils.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <ConfigFolderPath.h>

TEST_CASE("InverseKinematic test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 10;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();

    REQUIRE(paramHandler->setFromFile(getConfigPath()));

    // inintialize the joint positions and velocities
    Eigen::VectorXd JointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd JointVelocities(kinDyn->getNrOfDegreesOfFreedom());

    Eigen::VectorXd qInitial(kinDyn->getNrOfDegreesOfFreedom());
    BiomechanicalAnalysis::IK::HumanIK ik;

    qInitial.setConstant(0.0);

    ik.initialize(paramHandler, kinDyn);

    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.setInitialJointPositions(qInitial));
    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));
  
    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;
}
