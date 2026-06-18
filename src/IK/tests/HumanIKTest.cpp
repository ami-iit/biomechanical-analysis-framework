// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelExporter.h> // Add this include
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <ConfigFolderPath.h>

using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("InverseKinematics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    iDynTree::ModelLoader mdlLoader;
    REQUIRE(mdlLoader.loadModelFromFile(getConfigPath() + "/humanSubject01_48dof.urdf"));
    kinDyn->loadRobotModel(mdlLoader.model());
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>();
    IParametersHandler::shared_ptr handler = paramHandler;
    REQUIRE(paramHandler->setFromFile(getConfigPath() + "/configTestIK.toml"));

    /* CREATE THE PARAMETERS FOR THE JOINT_REG TASK*/
    std::vector<std::string> joints_list_kp, joints_list;
    std::vector<double> joints_kp, joints_weights;

    // Iter over all the joints of the model
    for (int i = 0; i < mdlLoader.model().getNrOfJoints(); i++)
    {
        auto joint = mdlLoader.model().getJoint(i);

        // Check if the joint has only one degree of freedom
        if (joint->getNrOfDOFs() == 1)
        {
            std::string jointName = mdlLoader.model().getJointName(i);
            joints_list_kp.push_back(jointName);
            joints_kp.push_back(10.0);
            joints_list.push_back(jointName);
            joints_weights.push_back(0.1);
        }
    }

    IParametersHandler::shared_ptr setGroup = std::make_shared<TomlImplementation>();
    setGroup->setParameter("type", "JointRegularizationTask");
    setGroup->setParameter("robot_velocity_variable_name", "robot_velocity");
    setGroup->setParameter("weight", 0.01);
    setGroup->setParameter("upper_limit", 5.0);
    setGroup->setParameter("lower_limit", -5.0);
    setGroup->setParameter("joints_list_kp", joints_list_kp);
    setGroup->setParameter("joints_kp", joints_kp);
    setGroup->setParameter("joints_list", joints_list);
    setGroup->setParameter("joints_weights", joints_weights);
    REQUIRE(paramHandler->setGroup("JOINT_REG_TASK", setGroup));

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
    mapNodeData[6].I_R_IMU.setIdentity();
    mapNodeData[7].I_R_IMU = I_R_IMU;
    mapNodeData[7].I_omega_IMU = I_omega_IMU;

    qInitial.setConstant(0.0);

    REQUIRE(ik.initialize(paramHandler, kinDyn));
    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.updateOrientationTask(3, I_R_IMU, I_omega_IMU));
    REQUIRE(ik.updateFloorContactTask(11, 11.0));
    REQUIRE(ik.updateGravityTask(10, I_R_IMU));
    REQUIRE(ik.updateOrientationAndGravityTasks(mapNodeData));
    REQUIRE(ik.updateJointConstraintsTask());
    Eigen::VectorXd jointPositionSetPoint = ik.getJointPositionSetPoint();
    REQUIRE(ik.updateJointRegularizationTask(jointPositionSetPoint));

    manif::SO3d orientationSetPoint;
    Eigen::Vector3d orientationOmegaSetPoint;
    REQUIRE(ik.getOrientationTaskSetPoint(3, orientationSetPoint, orientationOmegaSetPoint));

    Eigen::Vector3d gravitySetPoint;
    REQUIRE(ik.getGravityTaskSetPoint(10, gravitySetPoint));
    REQUIRE(gravitySetPoint.norm() > 0.0);

    // Test PositionTask update
    BiomechanicalAnalysis::IK::positionData posData;
    posData.S_p_M = Eigen::Vector3d(0.3, 0.1, 1.0);
    REQUIRE(ik.updatePositionTask(20, posData));

    Eigen::Vector3d positionSetPoint;
    Eigen::Vector3d positionVelSetPoint;
    REQUIRE(ik.getPositionTaskSetPoint(20, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(Eigen::Vector3d(0.4, -0.1, 1.3)));
    REQUIRE(positionVelSetPoint.isApprox(Eigen::Vector3d::Zero()));

    // Test PoseTask update
    BiomechanicalAnalysis::IK::poseData pData;
    pData.S_H_M = manif::SE3d(Eigen::Vector3d(0.2, -0.3, 0.8), manif::SO3d::Identity().quat());
    REQUIRE(ik.updatePoseTask(21, pData));

    manif::SE3d poseSetPoint;
    manif::SE3d::Tangent poseVelSetPoint;
    REQUIRE(ik.getPoseTaskSetPoint(21, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(Eigen::Vector3d(0.6, -0.3, 0.7)));
    REQUIRE(poseVelSetPoint.coeffs().isApprox(manif::SE3d::Tangent::Zero().coeffs()));

    // Set a non-zero base position before calibration: offset in subsequent pose/position setpoints.
    Eigen::Matrix4d shiftedBasePose = Eigen::Matrix4d::Identity();
    shiftedBasePose(0, 3) = 1.0;
    shiftedBasePose(1, 3) = -0.5;
    Eigen::VectorXd currentJointPositions(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd currentJointVelocities(kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd currentBaseVelocity(6);
    Eigen::Vector3d currentGravity;
    Eigen::Matrix4d currentBasePose;
    REQUIRE(kinDyn->getRobotState(currentBasePose, currentJointPositions, currentBaseVelocity, currentJointVelocities, currentGravity));
    currentBaseVelocity.setZero();
    currentJointVelocities.setZero();
    REQUIRE(kinDyn->setRobotState(shiftedBasePose, currentJointPositions, currentBaseVelocity, currentJointVelocities, currentGravity));

    REQUIRE(ik.calibrateWorldYaw(mapNodeData));
    REQUIRE(ik.calibrateAllWithWorld(mapNodeData, "link1"));

    Eigen::Vector3d worldAnchorTranslation;
    REQUIRE(ik.getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.isApprox(Eigen::Vector3d(-1.0, 0.5, 0.0)));

    REQUIRE(ik.clearCalibrationMatrices());
    REQUIRE(ik.updatePositionTask(20, posData));
    REQUIRE(ik.getPositionTaskSetPoint(20, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(Eigen::Vector3d(-0.6, 0.4, 1.3)));
    REQUIRE(ik.updatePoseTask(21, pData));
    REQUIRE(ik.getPoseTaskSetPoint(21, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(Eigen::Vector3d(-0.4, 0.2, 0.7)));

    REQUIRE(ik.resetWorldAnchorTranslation());
    REQUIRE(ik.getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.isApprox(Eigen::Vector3d::Zero()));
    REQUIRE(ik.updatePositionTask(20, posData));
    REQUIRE(ik.getPositionTaskSetPoint(20, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(Eigen::Vector3d(0.4, -0.1, 1.3)));
    REQUIRE(ik.updatePoseTask(21, pData));
    REQUIRE(ik.getPoseTaskSetPoint(21, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(Eigen::Vector3d(0.6, -0.3, 0.7)));

    Eigen::Matrix4d recenterBasePose = Eigen::Matrix4d::Identity();
    recenterBasePose(0, 3) = 0.25;
    recenterBasePose(1, 3) = -0.15;
    REQUIRE(kinDyn->setRobotState(recenterBasePose, currentJointPositions, currentBaseVelocity, currentJointVelocities, currentGravity));
    REQUIRE(ik.recenterWorldAnchor());
    REQUIRE(ik.getWorldAnchorTranslation(worldAnchorTranslation));

    const Eigen::Vector3d expectedPositionAfterRecenter = Eigen::Vector3d(0.4, -0.1, 1.3) + worldAnchorTranslation;
    const Eigen::Vector3d expectedPoseAfterRecenter = Eigen::Vector3d(0.6, -0.3, 0.7) + worldAnchorTranslation;

    REQUIRE(ik.updatePositionTask(20, posData));
    REQUIRE(ik.getPositionTaskSetPoint(20, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(expectedPositionAfterRecenter));
    REQUIRE(positionVelSetPoint.isApprox(Eigen::Vector3d::Zero()));
    REQUIRE(ik.updatePoseTask(21, pData));
    REQUIRE(ik.getPoseTaskSetPoint(21, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(expectedPoseAfterRecenter));
    REQUIRE(poseVelSetPoint.coeffs().isApprox(manif::SE3d::Tangent::Zero().coeffs()));

    Eigen::Vector3d worldAnchorBeforeJointReset = worldAnchorTranslation;
    REQUIRE(ik.resetJointState());

    Eigen::VectorXd jointPositionsAfterReset(ik.getDoFsNumber());
    Eigen::VectorXd jointVelocitiesAfterReset(ik.getDoFsNumber());
    Eigen::Vector3d baseLinearVelocityAfterReset;
    REQUIRE(ik.getJointPositions(jointPositionsAfterReset));
    REQUIRE(ik.getJointVelocities(jointVelocitiesAfterReset));
    REQUIRE(ik.getBaseLinearVelocity(baseLinearVelocityAfterReset));
    REQUIRE(jointPositionsAfterReset.isApprox(Eigen::VectorXd::Zero(ik.getDoFsNumber())));
    REQUIRE(jointVelocitiesAfterReset.isApprox(Eigen::VectorXd::Zero(ik.getDoFsNumber())));
    REQUIRE(baseLinearVelocityAfterReset.isApprox(Eigen::Vector3d::Zero()));

    REQUIRE(ik.getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.isApprox(worldAnchorBeforeJointReset));

    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));

    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;
}
