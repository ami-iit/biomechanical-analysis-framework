// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelTestUtils.h>
#include <manif/SO3.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <ConfigFolderPath.h>

TEST_CASE("InverseKinematics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>();

    std::cout << "configPath = " << getConfigPath() + "/configTestIK.toml" << std::endl;

    REQUIRE(paramHandler->setFromFile(getConfigPath() + "/configTestIK.toml"));

    // inintialize the joint positions and velocities
    Eigen::VectorXd JointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd JointVelocities(kinDyn->getNrOfDegreesOfFreedom());

    Eigen::VectorXd qInitial(kinDyn->getNrOfDegreesOfFreedom());
    BiomechanicalAnalysis::IK::HumanIK ik;

    manif::SO3d I_R_IMU;
    manif::SO3Tangentd I_omega_IMU;
    I_R_IMU.setRandom();
    I_omega_IMU.setRandom();
    double ZForce = 20.0;
    Eigen::Vector3d desiredDirection;
    desiredDirection << 1.0, 2.0, 3.0;

    std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData> mapNodeData;
    mapNodeData[11].I_R_IMU.setIdentity();
    mapNodeData[6].I_R_IMU = I_R_IMU;
    mapNodeData[6].I_omega_IMU = I_omega_IMU;
    mapNodeData[7].I_R_IMU = I_R_IMU;
    mapNodeData[7].I_omega_IMU = I_omega_IMU;

    qInitial.setConstant(0.0);

    Eigen::VectorXd desiredPositionsKp(kinDyn->getNrOfDegreesOfFreedom());
    std::vector<std::string> jointsListKp;

    REQUIRE(ik.initialize(paramHandler, kinDyn));
    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.updateOrientationTask(3, I_R_IMU, I_omega_IMU));
    REQUIRE(ik.updateFloorContactTask(10, 11.0));
    REQUIRE(ik.updateGravityTask(10, I_R_IMU));
    REQUIRE(ik.updateOrientationAndGravityTasks(mapNodeData));
    REQUIRE(ik.updateJointConstraintsTask());
    REQUIRE(ik.updateJointRegularizationTask(desiredPositionsKp, jointsListKp));
    REQUIRE(ik.calibrateWorldYaw(mapNodeData));
    REQUIRE(ik.calibrateAllWithWorld(mapNodeData, "link1"));
    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));

    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;
}
