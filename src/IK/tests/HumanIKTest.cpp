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

namespace
{

constexpr auto CORE_CONFIG_FILE = "configTestIK.toml";
constexpr auto CONST_TASKS_CONFIG_FILE = "configTestIKConstants.toml";

struct IKTestContext
{
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;
    std::shared_ptr<BipedalLocomotion::ParametersHandler::TomlImplementation> paramHandler;
    std::unique_ptr<BiomechanicalAnalysis::IK::HumanIK> ik;
    manif::SO3d I_R_IMU;
    manif::SO3Tangentd I_omega_IMU;
    std::unordered_map<int, BiomechanicalAnalysis::IK::nodeData> mapNodeData;
};

/// Create and initialize a complete IK test context with model, parameters, and IMU inputs.
IKTestContext makeContext(const std::string& configFile = CORE_CONFIG_FILE)
{
    IKTestContext ctx;
    ctx.kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    iDynTree::ModelLoader mdlLoader;
    REQUIRE(mdlLoader.loadModelFromFile(getConfigPath() + "/humanSubject01_48dof.urdf"));
    ctx.kinDyn->loadRobotModel(mdlLoader.model());

    ctx.paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>();
    REQUIRE(ctx.paramHandler->setFromFile(getConfigPath() + "/" + configFile));

    std::vector<std::string> joints_list_kp;
    std::vector<std::string> joints_list;
    std::vector<double> joints_kp;
    std::vector<double> joints_weights;

    for (int i = 0; i < mdlLoader.model().getNrOfJoints(); i++)
    {
        auto joint = mdlLoader.model().getJoint(i);
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
    REQUIRE(ctx.paramHandler->setGroup("JOINT_REG_TASK", setGroup));

    ctx.ik = std::make_unique<BiomechanicalAnalysis::IK::HumanIK>();
    ctx.I_R_IMU = manif::SO3Tangentd(Eigen::Vector3d(0.1, -0.2, 0.3)).exp();
    ctx.I_omega_IMU = manif::SO3Tangentd(Eigen::Vector3d(0.05, -0.04, 0.02));
    ctx.mapNodeData[6].I_R_IMU.setIdentity();
    ctx.mapNodeData[7].I_R_IMU = ctx.I_R_IMU;
    ctx.mapNodeData[7].I_omega_IMU = ctx.I_omega_IMU;

    REQUIRE(ctx.ik->initialize(ctx.paramHandler, ctx.kinDyn));
    REQUIRE(ctx.ik->setDt(0.1));

    return ctx;
}

/// Update the core IK tasks that must be valid before running solver-dependent checks.
void updateCoreTasks(IKTestContext& ctx)
{
    REQUIRE(ctx.ik->updateOrientationTask(3, ctx.I_R_IMU, ctx.I_omega_IMU));
    REQUIRE(ctx.ik->updateFloorContactTask(11, 11.0));
    REQUIRE(ctx.ik->updateGravityTask(10, ctx.I_R_IMU));
    BiomechanicalAnalysis::IK::positionData positionData;
    positionData.S_p_M = Eigen::Vector3d(0.3, 0.1, 1.0);
    REQUIRE(ctx.ik->updatePositionTask(20, positionData));

    BiomechanicalAnalysis::IK::poseData poseData;
    poseData.S_H_M = manif::SE3d(Eigen::Vector3d(0.2, -0.3, 0.8), manif::SO3d::Identity().quat());
    REQUIRE(ctx.ik->updatePoseTask(21, poseData));
    REQUIRE(ctx.ik->updateOrientationAndGravityTasks(ctx.mapNodeData));
    REQUIRE(ctx.ik->updateJointConstraintsTask());
    Eigen::VectorXd jointPositionSetPoint = ctx.ik->getJointPositionSetPoint();
    REQUIRE(ctx.ik->updateJointRegularizationTask(jointPositionSetPoint));
}

/// Set the robot base XY translation while preserving current joint state and gravity.
void shiftBasePose(IKTestContext& ctx, const double x, const double y)
{
    Eigen::Matrix4d shiftedBasePose = Eigen::Matrix4d::Identity();
    shiftedBasePose(0, 3) = x;
    shiftedBasePose(1, 3) = y;

    Eigen::VectorXd currentJointPositions(ctx.kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd currentJointVelocities(ctx.kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd currentBaseVelocity(6);
    Eigen::Vector3d currentGravity;
    Eigen::Matrix4d currentBasePose;
    REQUIRE(ctx.kinDyn->getRobotState(currentBasePose, currentJointPositions, currentBaseVelocity, currentJointVelocities, currentGravity));
    currentBaseVelocity.setZero();
    currentJointVelocities.setZero();
    REQUIRE(ctx.kinDyn->setRobotState(shiftedBasePose, currentJointPositions, currentBaseVelocity, currentJointVelocities, currentGravity));
}

/// Build a fixed position-task input used across tests for reproducible assertions.
BiomechanicalAnalysis::IK::positionData makePositionData()
{
    BiomechanicalAnalysis::IK::positionData posData;
    posData.S_p_M = Eigen::Vector3d(0.3, 0.1, 1.0);
    return posData;
}

/// Build a fixed pose-task input used across tests for reproducible assertions.
BiomechanicalAnalysis::IK::poseData makePoseData()
{
    BiomechanicalAnalysis::IK::poseData poseData;
    poseData.S_H_M = manif::SE3d(Eigen::Vector3d(0.2, -0.3, 0.8), manif::SO3d::Identity().quat());
    return poseData;
}

void expectConstantWorldFrameSetpoints(IKTestContext& ctx)
{
    manif::SO3d orientationSetPoint;
    Eigen::Vector3d orientationOmegaSetPoint;
    REQUIRE(ctx.ik->getOrientationTaskSetPoint(30, orientationSetPoint, orientationOmegaSetPoint));
    REQUIRE(orientationSetPoint.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    REQUIRE(orientationOmegaSetPoint.isApprox(Eigen::Vector3d::Zero(), 1e-12));

    Eigen::Vector3d gravitySetPoint;
    REQUIRE(ctx.ik->getGravityTaskSetPoint(31, gravitySetPoint));
    REQUIRE(gravitySetPoint.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

    Eigen::Vector3d positionSetPoint;
    Eigen::Vector3d positionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(32, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(Eigen::Vector3d(0.4, -0.1, 1.3), 1e-12));
    REQUIRE(positionVelSetPoint.isApprox(Eigen::Vector3d::Zero(), 1e-12));

    manif::SE3d poseSetPoint;
    manif::SE3d::Tangent poseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(33, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(Eigen::Vector3d(0.6, -0.3, 0.7), 1e-12));
    REQUIRE(poseSetPoint.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    REQUIRE(poseVelSetPoint.coeffs().isApprox(manif::SE3d::Tangent::Zero().coeffs(), 1e-12));
}
} // namespace

TEST_CASE("InverseKinematics orientation and gravity setpoint getters")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);

    manif::SO3d orientationSetPoint;
    Eigen::Vector3d orientationOmegaSetPoint;
    REQUIRE(ctx.ik->getOrientationTaskSetPoint(3, orientationSetPoint, orientationOmegaSetPoint));

    Eigen::Vector3d gravitySetPoint;
    REQUIRE(ctx.ik->getGravityTaskSetPoint(10, gravitySetPoint));
    REQUIRE(gravitySetPoint.norm() > 0.0);
}

TEST_CASE("InverseKinematics calibrateWorldYaw affects orientation and gravity setpoints")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));

    REQUIRE(ctx.ik->updateOrientationTask(3, ctx.I_R_IMU, ctx.I_omega_IMU));
    REQUIRE(ctx.ik->updateGravityTask(10, ctx.I_R_IMU));

    manif::SO3d orientationSetPointWithExplicitUpdate;
    Eigen::Vector3d orientationOmegaSetPointWithExplicitUpdate;
    REQUIRE(ctx.ik->getOrientationTaskSetPoint(3, orientationSetPointWithExplicitUpdate, orientationOmegaSetPointWithExplicitUpdate));

    Eigen::Vector3d gravitySetPointWithExplicitUpdate;
    REQUIRE(ctx.ik->getGravityTaskSetPoint(10, gravitySetPointWithExplicitUpdate));
    REQUIRE(gravitySetPointWithExplicitUpdate.norm() > 0.0);
}

TEST_CASE("InverseKinematics calibrateAllWithWorld affects orientation and gravity setpoints")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);
    shiftBasePose(ctx, 1.0, -0.5);

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));

    REQUIRE(ctx.ik->updateOrientationTask(3, ctx.I_R_IMU, ctx.I_omega_IMU));
    REQUIRE(ctx.ik->updateGravityTask(10, ctx.I_R_IMU));

    manif::SO3d orientationSetPointWithExplicitUpdate;
    Eigen::Vector3d orientationOmegaSetPointWithExplicitUpdate;
    REQUIRE(ctx.ik->getOrientationTaskSetPoint(3, orientationSetPointWithExplicitUpdate, orientationOmegaSetPointWithExplicitUpdate));

    Eigen::Vector3d gravitySetPointWithExplicitUpdate;
    REQUIRE(ctx.ik->getGravityTaskSetPoint(10, gravitySetPointWithExplicitUpdate));
    REQUIRE(gravitySetPointWithExplicitUpdate.norm() > 0.0);
}

TEST_CASE("InverseKinematics position task setpoint")
{
    auto ctx = makeContext();
    auto posData = makePositionData();

    REQUIRE(ctx.ik->updatePositionTask(20, posData));

    Eigen::Vector3d positionSetPoint;
    Eigen::Vector3d positionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, positionSetPoint, positionVelSetPoint));
    REQUIRE(positionSetPoint.isApprox(Eigen::Vector3d(0.4, -0.1, 1.3)));
    REQUIRE(positionVelSetPoint.isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("InverseKinematics pose task setpoint")
{
    auto ctx = makeContext();
    auto pData = makePoseData();

    REQUIRE(ctx.ik->updatePoseTask(21, pData));

    manif::SE3d poseSetPoint;
    manif::SE3d::Tangent poseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, poseSetPoint, poseVelSetPoint));
    REQUIRE(poseSetPoint.translation().isApprox(Eigen::Vector3d(0.6, -0.3, 0.7)));
    REQUIRE(poseVelSetPoint.coeffs().isApprox(manif::SE3d::Tangent::Zero().coeffs()));
}

TEST_CASE("InverseKinematics constant tasks stay in world frame across advance and calibration")
{
    auto ctx = makeContext(CONST_TASKS_CONFIG_FILE);
    updateCoreTasks(ctx);

    REQUIRE(ctx.ik->advance());
    expectConstantWorldFrameSetpoints(ctx);

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));
    REQUIRE(ctx.ik->recenterWorldAnchor());
    REQUIRE(ctx.ik->resetWorldAnchorTranslation());
    REQUIRE(ctx.ik->clearCalibrationMatrices());

    REQUIRE(ctx.ik->advance());
    expectConstantWorldFrameSetpoints(ctx);
}

TEST_CASE("InverseKinematics world calibration sets anchor")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);
    shiftBasePose(ctx, 1.0, -0.5);

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));

    Eigen::Vector3d worldAnchorTranslation;
    REQUIRE(ctx.ik->getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.isApprox(Eigen::Vector3d(-1.0, 0.5, 0.0)));
}

TEST_CASE("InverseKinematics calibration affects position and pose setpoints")
{
    auto ctx = makeContext();
    auto posData = makePositionData();
    auto pData = makePoseData();
    updateCoreTasks(ctx);
    shiftBasePose(ctx, 1.0, -0.5);

    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    Eigen::Vector3d baselinePositionSetPoint;
    Eigen::Vector3d baselinePositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, baselinePositionSetPoint, baselinePositionVelSetPoint));
    manif::SE3d baselinePoseSetPoint;
    manif::SE3d::Tangent baselinePoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, baselinePoseSetPoint, baselinePoseVelSetPoint));

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));

    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));

    Eigen::Vector3d refreshedPositionSetPoint;
    Eigen::Vector3d refreshedPositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, refreshedPositionSetPoint, refreshedPositionVelSetPoint));
    manif::SE3d refreshedPoseSetPoint;
    manif::SE3d::Tangent refreshedPoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, refreshedPoseSetPoint, refreshedPoseVelSetPoint));

    Eigen::Vector3d worldAnchorTranslation;
    REQUIRE(ctx.ik->getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(refreshedPositionSetPoint.isApprox(baselinePositionSetPoint + worldAnchorTranslation, 1e-8));
    REQUIRE(refreshedPositionVelSetPoint.isApprox(baselinePositionVelSetPoint, 1e-12));
    REQUIRE(refreshedPoseSetPoint.translation().isApprox(baselinePoseSetPoint.translation() + worldAnchorTranslation, 1e-8));
    REQUIRE(refreshedPoseSetPoint.quat().coeffs().isApprox(baselinePoseSetPoint.quat().coeffs(), 1e-12));
    REQUIRE(refreshedPoseVelSetPoint.coeffs().isApprox(baselinePoseVelSetPoint.coeffs(), 1e-12));

    REQUIRE(ctx.ik->clearCalibrationMatrices());
    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    Eigen::Vector3d positionSetPointAfterClear;
    Eigen::Vector3d positionVelSetPointAfterClear;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, positionSetPointAfterClear, positionVelSetPointAfterClear));
    REQUIRE(positionSetPointAfterClear.isApprox(Eigen::Vector3d(-0.6, 0.4, 1.3)));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    manif::SE3d poseSetPointAfterClear;
    manif::SE3d::Tangent poseVelSetPointAfterClear;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, poseSetPointAfterClear, poseVelSetPointAfterClear));
    REQUIRE(poseSetPointAfterClear.translation().isApprox(Eigen::Vector3d(-0.4, 0.2, 0.7)));
}

TEST_CASE("InverseKinematics floor contact default_position seeds the setpoint at initialization")
{
    auto ctx = makeContext();

    Eigen::Vector3d expectedSetPoint = iDynTree::toEigen(ctx.kinDyn->getWorldTransform("LeftFoot").getPosition());
    expectedSetPoint(2) = 0.0; // FLOOR_CONTACT_TASK_1 configures default_position = ["*", "*", "0.0"]

    Eigen::Vector3d initialSetPointPosition;
    bool initialFootInContact{true};
    REQUIRE(ctx.ik->getFloorContactTaskSetPoint(11, initialSetPointPosition, initialFootInContact));
    REQUIRE_FALSE(initialFootInContact);
    REQUIRE(initialSetPointPosition.isApprox(expectedSetPoint, 1e-8));
}

TEST_CASE("InverseKinematics floor contact activation during normal operation uses raw kinDyn, not default_position")
{
    auto ctx = makeContext();

    // default_position is only meant for initialization and post-calibration reset, not for
    // regular runtime contact activation, which should always trust the real measured position.
    Eigen::Vector3d expectedSetPoint = iDynTree::toEigen(ctx.kinDyn->getWorldTransform("LeftFoot").getPosition());

    REQUIRE(ctx.ik->updateFloorContactTask(11, 61.0));

    Eigen::Vector3d setPointPosition;
    bool footInContact{false};
    REQUIRE(ctx.ik->getFloorContactTaskSetPoint(11, setPointPosition, footInContact));
    REQUIRE(footInContact);
    REQUIRE(setPointPosition.isApprox(expectedSetPoint, 1e-8));
}

TEST_CASE("InverseKinematics floor contact setpoint is recomputed with default_position after calibration, even if active")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);

    REQUIRE(ctx.ik->updateFloorContactTask(11, 61.0));

    bool footInContactBeforeCalibration{false};
    Eigen::Vector3d setPointBeforeCalibration;
    REQUIRE(ctx.ik->getFloorContactTaskSetPoint(11, setPointBeforeCalibration, footInContactBeforeCalibration));
    REQUIRE(footInContactBeforeCalibration);

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));

    Eigen::Vector3d setPointAfterCalibration;
    bool footInContactAfterCalibration{false};
    REQUIRE(ctx.ik->getFloorContactTaskSetPoint(11, setPointAfterCalibration, footInContactAfterCalibration));
    // Regression: calibration redefines the world frame entirely, so every floor-contact setpoint
    // is recomputed from the post-reset kinDyn state merged with default_position, regardless of
    // whether the task was active beforehand (nothing pre-calibration remains meaningful).
    REQUIRE(std::abs(setPointAfterCalibration(2)) < 1e-8); // FLOOR_CONTACT_TASK_1 default_position z = 0.0
}

TEST_CASE("InverseKinematics floor contact setpoint refresh after calibration applies default_position for inactive tasks")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx); // force 11.0 < vertical_force_threshold 60.0: task never activates

    REQUIRE(ctx.ik->calibrateWorldYaw(ctx.mapNodeData));
    REQUIRE(ctx.ik->calibrateAllWithWorld(ctx.mapNodeData, "Pelvis"));

    Eigen::Vector3d setPointPosition;
    bool footInContact{true};
    REQUIRE(ctx.ik->getFloorContactTaskSetPoint(11, setPointPosition, footInContact));
    REQUIRE_FALSE(footInContact);
    REQUIRE(std::abs(setPointPosition(2)) < 1e-8);
}

TEST_CASE("InverseKinematics reset world anchor translation sets anchor to zero")
{
    auto ctx = makeContext();
    auto posData = makePositionData();
    auto pData = makePoseData();
    shiftBasePose(ctx, 1.0, -0.5);

    // Capture baseline setpoints for the same measurements.
    updateCoreTasks(ctx);
    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    Eigen::Vector3d baselinePositionSetPoint;
    Eigen::Vector3d baselinePositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, baselinePositionSetPoint, baselinePositionVelSetPoint));
    manif::SE3d baselinePoseSetPoint;
    manif::SE3d::Tangent baselinePoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, baselinePoseSetPoint, baselinePoseVelSetPoint));

    // Solve once before anchor operations.
    REQUIRE(ctx.ik->advance());

    // Create a non-zero anchor and then reset it to zero.
    REQUIRE(ctx.ik->recenterWorldAnchor());
    REQUIRE(ctx.ik->resetWorldAnchorTranslation());

    // Verify anchor is reset to zero.
    Eigen::Vector3d worldAnchorTranslation;
    REQUIRE(ctx.ik->getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.isApprox(Eigen::Vector3d::Zero()));

    // World-anchor reset does not implicitly recompute position/pose setpoints;
    // re-apply updates and verify consistency.
    updateCoreTasks(ctx);
    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    Eigen::Vector3d resetPositionSetPoint;
    Eigen::Vector3d resetPositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, resetPositionSetPoint, resetPositionVelSetPoint));
    manif::SE3d resetPoseSetPoint;
    manif::SE3d::Tangent resetPoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, resetPoseSetPoint, resetPoseVelSetPoint));

    REQUIRE(resetPositionSetPoint.isApprox(baselinePositionSetPoint, 1e-9));
    REQUIRE(resetPositionVelSetPoint.isApprox(baselinePositionVelSetPoint, 1e-9));
    REQUIRE(resetPoseSetPoint.translation().isApprox(baselinePoseSetPoint.translation(), 1e-9));
    REQUIRE(resetPoseSetPoint.quat().coeffs().isApprox(baselinePoseSetPoint.quat().coeffs(), 1e-9));
    REQUIRE(resetPoseVelSetPoint.coeffs().isApprox(baselinePoseVelSetPoint.coeffs(), 1e-9));

    // Solver still advances successfully after reset.
    Eigen::VectorXd jointPositionsAfterReset(ctx.kinDyn->getNrOfDegreesOfFreedom());
    REQUIRE(ctx.ik->advance());
    REQUIRE(ctx.ik->getJointPositions(jointPositionsAfterReset));
    REQUIRE(jointPositionsAfterReset.size() == ctx.ik->getDoFsNumber());
}

TEST_CASE("InverseKinematics recenter world anchor updates IK frame")
{
    auto ctx = makeContext();
    auto posData = makePositionData();
    auto pData = makePoseData();
    shiftBasePose(ctx, 0.25, -0.15);

    // Capture baseline setpoints for the same measurements.
    updateCoreTasks(ctx);
    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    Eigen::Vector3d baselinePositionSetPoint;
    Eigen::Vector3d baselinePositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, baselinePositionSetPoint, baselinePositionVelSetPoint));
    manif::SE3d baselinePoseSetPoint;
    manif::SE3d::Tangent baselinePoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, baselinePoseSetPoint, baselinePoseVelSetPoint));

    // Solve once before recenter.
    REQUIRE(ctx.ik->advance());
    Eigen::Vector3d basePositionBeforeRecenter;
    REQUIRE(ctx.ik->getBasePosition(basePositionBeforeRecenter));

    // Perform anchor recenter
    REQUIRE(ctx.ik->recenterWorldAnchor());
    Eigen::Vector3d worldAnchorTranslation;
    REQUIRE(ctx.ik->getWorldAnchorTranslation(worldAnchorTranslation));
    REQUIRE(worldAnchorTranslation.head<2>().isApprox(-basePositionBeforeRecenter.head<2>(), 1e-8));

    // Base is recentered in XY.
    Eigen::Vector3d basePosition;
    REQUIRE(ctx.ik->getBasePosition(basePosition));
    REQUIRE(basePosition.head<2>().isApprox(Eigen::Vector2d::Zero(), 1e-9));

    // Recenter updates internal frame/anchor state but does not implicitly recompute
    // position/pose setpoints. Re-apply updates and verify consistency.
    updateCoreTasks(ctx);
    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));
    Eigen::Vector3d recenteredPositionSetPoint;
    Eigen::Vector3d recenteredPositionVelSetPoint;
    REQUIRE(ctx.ik->getPositionTaskSetPoint(20, recenteredPositionSetPoint, recenteredPositionVelSetPoint));
    manif::SE3d recenteredPoseSetPoint;
    manif::SE3d::Tangent recenteredPoseVelSetPoint;
    REQUIRE(ctx.ik->getPoseTaskSetPoint(21, recenteredPoseSetPoint, recenteredPoseVelSetPoint));

    REQUIRE(recenteredPositionSetPoint.isApprox(baselinePositionSetPoint + worldAnchorTranslation, 1e-8));
    REQUIRE(recenteredPositionVelSetPoint.isApprox(baselinePositionVelSetPoint, 1e-9));
    REQUIRE(recenteredPoseSetPoint.translation().isApprox(baselinePoseSetPoint.translation() + worldAnchorTranslation, 1e-8));
    REQUIRE(recenteredPoseSetPoint.quat().coeffs().isApprox(baselinePoseSetPoint.quat().coeffs(), 1e-9));
    REQUIRE(recenteredPoseVelSetPoint.coeffs().isApprox(baselinePoseVelSetPoint.coeffs(), 1e-9));

    // Solver still advances successfully after recenter.
    Eigen::VectorXd jointPositionsAfterRecenter(ctx.kinDyn->getNrOfDegreesOfFreedom());
    REQUIRE(ctx.ik->advance());
    REQUIRE(ctx.ik->getJointPositions(jointPositionsAfterRecenter));
    REQUIRE(jointPositionsAfterRecenter.size() == ctx.ik->getDoFsNumber());
}

TEST_CASE("InverseKinematics reset joint state zeroes velocities")
{
    auto ctx = makeContext();
    updateCoreTasks(ctx);

    REQUIRE(ctx.ik->resetJointState());

    Eigen::VectorXd jointPositionsAfterReset(ctx.ik->getDoFsNumber());
    Eigen::VectorXd jointVelocitiesAfterReset(ctx.ik->getDoFsNumber());
    Eigen::Vector3d baseLinearVelocityAfterReset;
    REQUIRE(ctx.ik->getJointPositions(jointPositionsAfterReset));
    REQUIRE(ctx.ik->getJointVelocities(jointVelocitiesAfterReset));
    REQUIRE(ctx.ik->getBaseLinearVelocity(baseLinearVelocityAfterReset));
    REQUIRE(jointPositionsAfterReset.isApprox(Eigen::VectorXd::Zero(ctx.ik->getDoFsNumber())));
    REQUIRE(jointVelocitiesAfterReset.isApprox(Eigen::VectorXd::Zero(ctx.ik->getDoFsNumber())));
    REQUIRE(baseLinearVelocityAfterReset.isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("InverseKinematics advance solver step")
{
    auto ctx = makeContext();
    auto posData = makePositionData();
    auto pData = makePoseData();
    updateCoreTasks(ctx);

    REQUIRE(ctx.ik->updatePositionTask(20, posData));
    REQUIRE(ctx.ik->updatePoseTask(21, pData));

    Eigen::VectorXd jointPositions(ctx.kinDyn->getNrOfDegreesOfFreedom());
    Eigen::VectorXd jointVelocities(ctx.kinDyn->getNrOfDegreesOfFreedom());
    REQUIRE(ctx.ik->advance());
    REQUIRE(ctx.ik->getJointPositions(jointPositions));
    REQUIRE(ctx.ik->getJointVelocities(jointVelocities));
    REQUIRE(jointPositions.size() == ctx.ik->getDoFsNumber());
}
