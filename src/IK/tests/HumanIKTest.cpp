// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelTestUtils.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

TEST_CASE("InverseKinematic test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 10;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);

    manif::SO3d orientationDesired;
    orientationDesired.setRandom();
    manif::SO3Tangentd angVelDesired;
    angVelDesired.setRandom();

    std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler =
        std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    
    handler->setParameter("verbosity", false);

    handler->setParameter("tasks",
                                   std::vector<std::string>{"LINK1_TASK"});


    auto ikParameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    ikParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");
    handler->setGroup("IK", ikParameterHandler);

    auto SO3ParameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    SO3ParameterHandler->setParameter("frame_name", kinDyn->getFrameName(1));
    SO3ParameterHandler->setParameter("kp_angular", 1.0);
    SO3ParameterHandler->setParameter("robot_velocity_variable_name", "robotVelocity");

    handler->setGroup("LINK1_TASK", SO3ParameterHandler);

    // inintialize the joint positions and velocities
    Eigen::VectorXd JointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd JointVelocities(kinDyn->getNrOfDegreesOfFreedom());

    Eigen::VectorXd qInitial(kinDyn->getNrOfDegreesOfFreedom());
    BiomechanicalAnalysis::IK::HumanIK ik;

    qInitial.setConstant(0.0);

    ik.initialize(handler, kinDyn);

    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.setInitialJointPositions(qInitial));
    REQUIRE(ik.setLink1OrientationAndAngVel(orientationDesired, angVelDesired));
    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));
    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;
}
