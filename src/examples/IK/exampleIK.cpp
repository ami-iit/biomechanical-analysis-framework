/**
 * @file ExampleIK.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#include <iostream>
#include <thread>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/Visualizer.h>
#include <matioCpp/matioCpp.h>
#include <yarp/os/ResourceFinder.h>

#include <BiomechanicalAnalysis/Conversions/CommonConversions.h>
#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

using namespace BiomechanicalAnalysis::Conversions;

std::atomic<bool> tPoseFlag{false};
std::atomic<bool> stopThread{false};

bool getNodeData(matioCpp::Struct& ifeel_struct,
                 int nodeNum,
                 size_t index_i,
                 iDynTree::Rotation& I_R_IMU,
                 iDynTree::AngVelocity& I_omeg_IMU)
{
    matioCpp::Struct vLink_node
        = ifeel_struct("iFeelSuit_vLink_Node_" + std::to_string(nodeNum)).asStruct();
    if (!vLink_node.isValid())
    {
        BiomechanicalAnalysis::log()->error("Cannot read the iFeelSuit_vLink_Node_{} "
                                            "struct",
                                            std::to_string(nodeNum));
        return false;
    }
    matioCpp::MultiDimensionalArray<double> data1
        = vLink_node("data").asMultiDimensionalArray<double>();
    if (!data1.isValid())
    {
        BiomechanicalAnalysis::log()->error("Cannot read the data field of the node number {}",
                                            std::to_string(nodeNum));
        return false;
    }

    size_t index0 = 0;
    size_t index_w = 4;
    size_t index_x = 5;
    size_t index_y = 6;
    size_t index_z = 7;

    size_t angVel_x = 11;
    size_t angVel_y = 12;
    size_t angVel_z = 13;

    iDynTree::Vector4 quat;

    quat[1] = data1({index_x, index0, index_i});
    quat[2] = data1({index_y, index0, index_i});
    quat[3] = data1({index_z, index0, index_i});
    quat[0] = data1({index_w, index0, index_i});

    I_omeg_IMU[0] = data1({angVel_x, index0, index_i});
    I_omeg_IMU[1] = data1({angVel_y, index0, index_i});
    I_omeg_IMU[2] = data1({angVel_z, index0, index_i});

    I_R_IMU = iDynTree::Rotation::RotationFromQuaternion(quat);

    return true;
}

const std::vector<std::string> getJointsList()
{
    std::vector<std::string> nodesName;
    nodesName.push_back("jT9T8_rotx");
    nodesName.push_back("jT9T8_rotz");
    nodesName.push_back("jRightShoulder_rotx");
    nodesName.push_back("jRightShoulder_roty");
    nodesName.push_back("jRightShoulder_rotz");
    nodesName.push_back("jRightElbow_roty");
    nodesName.push_back("jRightElbow_rotz");
    nodesName.push_back("jLeftShoulder_rotx");
    nodesName.push_back("jLeftShoulder_roty");
    nodesName.push_back("jLeftShoulder_rotz");
    nodesName.push_back("jLeftElbow_roty");
    nodesName.push_back("jLeftElbow_rotz");
    nodesName.push_back("jLeftHip_rotx");
    nodesName.push_back("jLeftHip_roty");
    nodesName.push_back("jLeftHip_rotz");
    nodesName.push_back("jLeftKnee_roty");
    nodesName.push_back("jLeftKnee_rotz");
    nodesName.push_back("jLeftAnkle_rotx");
    nodesName.push_back("jLeftAnkle_roty");
    nodesName.push_back("jLeftAnkle_rotz");
    nodesName.push_back("jLeftBallFoot_roty");
    nodesName.push_back("jRightHip_rotx");
    nodesName.push_back("jRightHip_roty");
    nodesName.push_back("jRightHip_rotz");
    nodesName.push_back("jRightKnee_roty");
    nodesName.push_back("jRightKnee_rotz");
    nodesName.push_back("jRightAnkle_rotx");
    nodesName.push_back("jRightAnkle_roty");
    nodesName.push_back("jRightAnkle_rotz");
    nodesName.push_back("jRightBallFoot_roty");
    nodesName.push_back("jL5S1_roty");

    return nodesName;
}

void setTPoseThread()
{
    BiomechanicalAnalysis::log()->info("Enter 'calib' to run the T-pose calibration");
    std::string input;
    while (!stopThread)
    {
        std::cin >> input;
        if (input == "calib")
        {
            tPoseFlag = true;
        } else
        {
            BiomechanicalAnalysis::log()->warn("{} is an invalid input; please enter 'calib' if "
                                               "you want to run the T-pose calibration",
                                               input);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main()
{

    yarp::os::ResourceFinder rf;
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    iDynTree::ModelLoader mdlLoader;
    std::string urdfPath = rf.findFileByName("humanSubject01_48dof.urdf");
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList());
    kinDyn->loadRobotModel(mdlLoader.model());
    iDynTree::Visualizer viz;
    viz.addModel(mdlLoader.model(), "model");
    iDynTree::IModelVisualization& modelViz = viz.modelViz("model");
    viz.draw();
    iDynTree::Visualizer viz2;
    viz2.addModel(mdlLoader.model(), "model2");
    iDynTree::IModelVisualization& modelViz2 = viz2.modelViz("model2");
    viz2.draw();

    iDynTree::Rotation I_R_IMU;
    iDynTree::AngVelocity I_omega_IMU;
    Eigen::VectorXd initialJointPositions;
    Eigen::VectorXd initialJointVelocities;
    Eigen::VectorXd jointPositions;
    Eigen::VectorXd jointVelocities;
    Eigen::Vector3d gravity;
    Eigen::Matrix4d basePose;
    Eigen::Matrix<double, 6, 1> baseVelocity;
    Eigen::Matrix3d baseOrientation;
    Eigen::Vector3d basePosition;
    iDynTree::VectorDynSize jointPos, jointPos2;
    iDynTree::Transform w_H_b = iDynTree::Transform::Identity();
    iDynTree::Transform w_H_b2 = iDynTree::Transform::Identity();
    jointPos.resize(kinDyn->getNrOfDegreesOfFreedom());
    jointPos2.resize(kinDyn->getNrOfDegreesOfFreedom());
    jointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom());
    jointPositions.resize(kinDyn->getNrOfDegreesOfFreedom());
    initialJointPositions.resize(kinDyn->getNrOfDegreesOfFreedom());
    initialJointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom());
    initialJointPositions.setZero();
    gravity << 0.0, 0.0, -9.81;
    basePose.setIdentity();
    baseVelocity.setZero();

    auto paramHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();

    if (!paramHandler->setFromFile("/path/to/exampleIK.ini"))
    {
        BiomechanicalAnalysis::log()->error("Cannot configure the parameter handler");
        return 1;
    }

    matioCpp::File file("/path/to/ifeel_data.mat");

    matioCpp::Struct ifeel_data = file.read("ifeel_data").asStruct();
    matioCpp::Struct node12 = ifeel_data("iFeelSuit_vLink_Node_12").asStruct();

    matioCpp::File file2("/path/to/human_data.mat");
    matioCpp::Struct human_data = file2.read("human_data").asStruct();
    matioCpp::Struct human_state = human_data("human_state").asStruct();
    matioCpp::Struct joint_positions = human_state("joint_positions").asStruct();
    matioCpp::MultiDimensionalArray<double> jointPos_data
        = joint_positions("data").asMultiDimensionalArray<double>();

    for (size_t ii = 0; ii < 31; ii++)
    {
        initialJointPositions[ii] = jointPos_data({ii, 0, 0});
    }

    // get the dimension of the data
    matioCpp::Vector<double> data = node12("timestamps").asVector<double>();
    size_t dataLength = data.size();

    // list of nodes
    std::vector<int> nodesNumber = {3, 6, 7, 8, 5, 4, 11, 12, 9, 10};

    BiomechanicalAnalysis::IK::HumanIK ik;

    if (!ik.initialize(paramHandler, kinDyn))
    {
        BiomechanicalAnalysis::log()->error("Cannot initialize the inverse kinematics solver");
        return 1;
    }

    kinDyn->setRobotState(basePose,
                          initialJointPositions,
                          baseVelocity,
                          initialJointVelocities,
                          gravity);
    ik.setDt(0.01);

    std::thread tPoseThread = std::thread(setTPoseThread);

    for (size_t ii = 0; ii < dataLength; ii++)
    {
        // perform the T-pose calibration if the user inputs 't'
        if (tPoseFlag)
        {
            for (auto& node : nodesNumber)
            {
                getNodeData(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);
                ik.TPoseCalibrationNode(node,
                                        manif::SO3d(fromiDynTreeToEigenQuatConversion(I_R_IMU)));
            }
            BiomechanicalAnalysis::log()->info("T-pose calibration done");
            tPoseFlag = false;
        }

        // implement cycle over the nodes
        for (auto& node : nodesNumber)
        {
            // cycle over nodes to get orientation and angular velocity
            getNodeData(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);

            // manif object is built from Eigen::Quaterniond
            manif::SO3d I_R_IMU_manif = manif::SO3d(fromiDynTreeToEigenQuatConversion(I_R_IMU));
            manif::SO3Tangentd I_omega_IMU_manif
                = manif::SO3Tangentd(iDynTree::toEigen(I_omega_IMU));

            if (!ik.setNodeSetPoint(node, I_R_IMU_manif, I_omega_IMU_manif))
            {
                BiomechanicalAnalysis::log()->error("[error] Cannot set the node number {} set "
                                                    "point",
                                                    node);
                return 1;
            }
        }
        if (!ik.advance())
        {
            BiomechanicalAnalysis::log()->error("Cannot advance the inverse kinematics solver");
            return 1;
        }
        ik.getJointPositions(jointPositions);
        ik.getJointVelocities(jointVelocities);
        ik.getBaseOrientation(baseOrientation);
        ik.getBasePosition(basePosition);
        for (size_t jj = 0; jj < 31; jj++)
        {
            jointPos2[jj] = jointPos_data({jj, 0, ii});
        }
        iDynTree::Rotation w_R_b;
        iDynTree::Position w_p_b;
        iDynTree::toEigen(w_R_b) = baseOrientation;
        iDynTree::toEigen(w_p_b) = basePosition;

        iDynTree::toEigen(jointPos) = jointPositions;

        modelViz.setPositions(w_H_b, jointPos);
        viz.draw();
        modelViz2.setPositions(w_H_b2, jointPos2);
        viz2.draw();
    }

    stopThread = true;

    BiomechanicalAnalysis::log()->info("\nPlease enter a character to terminate the program");

    if (tPoseThread.joinable())
    {
        tPoseThread.join();
    }

    BiomechanicalAnalysis::log()->info("done");

    return 0;
}
