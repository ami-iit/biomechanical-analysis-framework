#include <iostream>
#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <matioCpp/matioCpp.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/EigenHelpers.h>
#include <yarp/os/ResourceFinder.h>

bool getNodeData(matioCpp::Struct &ifeel_struct, int nodeNum, size_t index_i, iDynTree::Rotation &I_R_IMU, iDynTree::AngVelocity &I_omeg_IMU)
{
    matioCpp::Struct vLink_node = ifeel_struct("iFeelSuit_vLink_Node_" + std::to_string(nodeNum)).asStruct();
    if (!vLink_node.isValid())
    {
        std::cerr << "[error] Cannot read the iFeelSuit_vLink_Node_" + std::to_string(nodeNum) + " struct" << std::endl;
        return false;
    }
    matioCpp::MultiDimensionalArray<double> data1 = vLink_node("data").asMultiDimensionalArray<double>();
    if (!data1.isValid())
    {
        std::cerr << "[error] Cannot read the data field of the iFeelSuit_vLink_Node_" + std::to_string(nodeNum) + " struct" << std::endl;
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

    quat[0] = data1({index_x, index0, index_i});
    quat[1] = data1({index_y, index0, index_i});
    quat[2] = data1({index_z, index0, index_i});
    quat[3] = data1({index_w, index0, index_i});

    I_omeg_IMU[0] = data1({angVel_x, index0, index_i});
    I_omeg_IMU[1] = data1({angVel_y, index0, index_i});
    I_omeg_IMU[2] = data1({angVel_z, index0, index_i});

    I_R_IMU = iDynTree::Rotation::RotationFromQuaternion(quat);

    return true;
}

std::vector<std::string> getJointsList()
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

int main() {

    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();

    if (!paramHandler->setFromFile("/home/dgorbani/software/ergoCub/biomechanical-analysis-framework/src/examples/IK/exampleIK.ini"))
    {
        std::cerr << "[error] Cannot configure the parameter handler" << std::endl;
        return 1;
    }

    matioCpp::File file("/home/dgorbani/matlab.mat");

    matioCpp::Struct ifeel_data = file.read("ifeel_data").asStruct();
    matioCpp::Struct node12 = ifeel_data("iFeelSuit_vLink_Node_12").asStruct();

    // get the dimension of the data
    matioCpp::Vector<double> data = node12("timestamps").asVector<double>();
    size_t dataLength = data.size();
    std::cout << "data length: " << dataLength << std::endl;

    // list of nodes
    std::vector<int> nodesNumber = {3, 6, 7, 8, 4, 5, 11, 12, 9, 10};

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    iDynTree::ModelLoader mdlLoader;
    yarp::os::ResourceFinder rf;

    std::string urdfPath = rf.findFileByName("humanSubject01_48dof.urdf");
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList());
    kinDyn->loadRobotModel(mdlLoader.model());

    BiomechanicalAnalysis::IK::HumanIK ik;

    if (!ik.initialize(paramHandler, kinDyn))
    {
        std::cerr << "[error] Cannot initialize the inverse kinematics solver" << std::endl;
        return 1;
    }

    iDynTree::Rotation I_R_IMU;
    iDynTree::AngVelocity I_omega_IMU;
    size_t maxIndex = 3;

    for (size_t ii = 0; ii < maxIndex; ii++)
    {
        // implement cycle
        for(auto& node : nodesNumber)
        {
            // cycle over nodes to get orientation and angular velocity
            getNodeData(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);
            manif::SO3d I_R_IMU_manif(I_R_IMU.asQuaternion()(1), I_R_IMU.asQuaternion()(2), I_R_IMU.asQuaternion()(3), I_R_IMU.asQuaternion()(0));
            manif::SO3Tangentd I_omega_IMU_manif(iDynTree::toEigen(I_omega_IMU));
            if (!ik.setNodeSetPoint(node, I_R_IMU_manif, I_omega_IMU_manif))
            {
                std::cerr << "[error] Cannot set the node number " << node << " set point" << std::endl;
                return 1;
            }
        }
        if (!ik.advance())
        {
            std::cerr << "[error] Cannot advance the inverse kinematics solver" << std::endl;
            // return 1;
        }
    }

    return 0;
}
