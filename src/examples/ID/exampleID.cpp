/**
 * @file ExampleID.cpp
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#include <iostream> // defines I/O standard C++ classes (out,in,err, etc.)

#include <iDynTree/EigenHelpers.h> //support for linear algebra
#include <iDynTree/ModelLoader.h> // loads URDF models
#include <iDynTree/ModelTestUtils.h> // utility for models testing
#include <iDynTree/Visualizer.h> // functionality for models visualization
#include <matioCpp/matioCpp.h> // reading/writing of Matlab files
#include <yarp/os/ResourceFinder.h> //finding resources

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h> //handle logging
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h> //handle parameters for Yarp

#include <ConfigFolderPath.h>

const std::vector<std::string> getJointsList()
{
    // Create a vector to store joint names
    std::vector<std::string> nodesName;

    // Add joint names to the vector
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

int main()
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    yarp::os::ResourceFinder rf;

    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    paramHandler->setFromFile(getConfigPath() + "/configExampleID.ini");

    iDynTree::ModelLoader mdlLoader; // Create a ModelLoader object
    std::string urdfPath = rf.findFileByName("humanSubject03_48dof.urdf"); // Find the URDF file
                                                                           // path
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList()); // Load the URDF model and create
                                                                   // a reduced model
    kinDyn->loadRobotModel(mdlLoader.model()); // Load the model into the KinDynComputations object
    kinDyn->setFloatingBase("Pelvis"); // Set the floating base of the model

    BiomechanicalAnalysis::ID::HumanID id; // Create an object of the HumanID class
    if (!id.initialize(paramHandler, kinDyn)) // Initialize the HumanID object
    {
        BiomechanicalAnalysis::log()->error("Error in initializing the HumanID object");
        return -1;
    }

    std::string matFilePath;
    if (!paramHandler->getParameter("matFilePath", matFilePath))
    {
        BiomechanicalAnalysis::log()->error("Error in reading the mat file path");
        return -1;
    }
    std::string ifeelFilePath;

    if (!paramHandler->getParameter("ifeelFilePath", ifeelFilePath))
    {
        BiomechanicalAnalysis::log()->error("Error in reading the ifeel file path");
        return -1;
    }

    matioCpp::File ifeelFile(ifeelFilePath);
    matioCpp::Struct ifeel_data = ifeelFile.read("ifeel_data").asStruct();
    matioCpp::Struct ft6dNode1 = ifeel_data("iFeelSuit_ft6D_Node_1").asStruct();
    matioCpp::Struct ft6dNode2 = ifeel_data("iFeelSuit_ft6D_Node_2").asStruct();
    matioCpp::MultiDimensionalArray<double> ft6dNode1Data = ft6dNode1("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> ft6dNode2Data = ft6dNode2("data").asMultiDimensionalArray<double>();
    matioCpp::File humanFile(matFilePath);
    matioCpp::Struct human_data = humanFile.read("human_data").asStruct();
    matioCpp::Struct human_state = human_data("human_state").asStruct();
    matioCpp::Struct human_wrench = human_data("human_wrench").asStruct();
    matioCpp::Struct joints_state = human_data("joints_state").asStruct();
    matioCpp::Struct wrenches = human_wrench("wrenches").asStruct();
    matioCpp::Struct joint_positions = joints_state("positions").asStruct();
    matioCpp::Struct joint_velocities = joints_state("velocities").asStruct();
    matioCpp::Struct base_velocity = human_state("base_velocity").asStruct();
    matioCpp::Struct base_position = human_state("base_position").asStruct();
    matioCpp::Struct base_orientation = human_state("base_orientation").asStruct();
    matioCpp::MultiDimensionalArray<double> jointPosData = joint_positions("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> wrenchesData = wrenches("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> jointVelData = joint_velocities("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> baseVelData = base_velocity("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> basePosData = base_position("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> baseOriData = base_orientation("data").asMultiDimensionalArray<double>();
    Eigen::VectorXd jointPos;
    Eigen::VectorXd jointVel;
    Eigen::Matrix4d basePose;
    Eigen::Matrix<double, 6, 1> baseVelocity;
    iDynTree::Vector4 orientation;
    Eigen::Vector3d gravity;
    std::unordered_map<std::string, iDynTree::Wrench> wrenchesMap;
    gravity << 0, 0, -9.81;
    jointPos.resize(31);
    jointVel.resize(31);
    matioCpp::File file = matioCpp::File::Create("test.mat");
    size_t len = 1550;
    std::unordered_map<std::string, std::vector<double>> jointTorquesMap;
    for (auto joint : id.getJointsList())
    {
        jointTorquesMap[joint] = std::vector<double>(len);
    }
    std::vector<double> RHx(len);
    std::vector<double> RHy(len);
    std::vector<double> RHz(len);
    std::vector<double> LHx(len);
    std::vector<double> LHy(len);
    std::vector<double> LHz(len);
    std::vector<double> RFx(len);
    std::vector<double> RFy(len);
    std::vector<double> RFz(len);
    std::vector<double> LFx(len);
    std::vector<double> LFy(len);
    std::vector<double> LFz(len);
    matioCpp::Vector<double> RightHandXMatVec("RHx");
    matioCpp::Vector<double> RightHandYMatVec("RHy");
    matioCpp::Vector<double> RightHandZMatVec("RHz");
    matioCpp::Vector<double> LeftHandXMatVec("LHx");
    matioCpp::Vector<double> LeftHandYMatVec("LHy");
    matioCpp::Vector<double> LeftHandZMatVec("LHz");
    matioCpp::Vector<double> RightFootXMatVec("RFx");
    matioCpp::Vector<double> RightFootYMatVec("RFy");
    matioCpp::Vector<double> RightFootZMatVec("RFz");
    matioCpp::Vector<double> LeftFootXMatVec("LFx");
    matioCpp::Vector<double> LeftFootYMatVec("LFy");
    matioCpp::Vector<double> LeftFootZMatVec("LFz");
    std::vector<matioCpp::Vector<double>> jointTorquesMatVec;
    jointTorquesMatVec.resize(id.getJointsList().size());
    for (int i = 0; i < id.getJointsList().size(); i++)
    {
        jointTorquesMatVec[i] = matioCpp::Vector<double>(id.getJointsList()[i]);
    }

    for (size_t i = 0; i < len; i++)
    {
        for (size_t j = 0; j < 31; j++)
        {
            jointPos(j) = jointPosData({j, 0, i});
            jointVel(j) = jointVelData({j, 0, i});
        }
        for (size_t j = 0; j < 3; j++)
        {
            basePose(j, 3) = basePosData({j, 0, i});
        }
        for (size_t j = 0; j < 6; j++)
        {
            baseVelocity(j) = baseVelData({j, 0, i});
        }
        for (size_t j = 0; j < 4; j++)
        {
            orientation[j] = baseOriData({j, 0, i});
        }
        iDynTree::Wrench leftFootWrench, rightFootWrench;
        for (size_t j = 0; j < 6; j++)
        {
            rightFootWrench(j) = ft6dNode2Data({j + 1, 0, i + 492});
            leftFootWrench(j) = ft6dNode1Data({j + 1, 0, i + 492});
        }
        wrenchesMap["RightFoot"] = rightFootWrench;
        wrenchesMap["LeftFoot"] = leftFootWrench;
        iDynTree::Rotation rot = iDynTree::Rotation::RotationFromQuaternion(orientation);
        basePose.topLeftCorner(3, 3) = iDynTree::toEigen(rot);
        kinDyn->setRobotState(basePose, jointPos, baseVelocity, jointVel, gravity);
        if (!id.updateExtWrenchesMeasurements(wrenchesMap))
        {
            BiomechanicalAnalysis::log()->error("Error in updating external wrenches");
            return -1;
        }
        if (!id.solve())
        {
            BiomechanicalAnalysis::log()->error("Error in solving the inverse dynamics");
            return -1;
        }
        std::vector<iDynTree::Wrench> extWrenches = id.getEstimatedExtWrenches();
        RHx[i] = extWrenches[3](0);
        RHy[i] = extWrenches[3](1);
        RHz[i] = extWrenches[3](2);
        LHx[i] = extWrenches[2](0);
        LHy[i] = extWrenches[2](1);
        LHz[i] = extWrenches[2](2);
        RFx[i] = extWrenches[0](0);
        RFy[i] = extWrenches[0](1);
        RFz[i] = extWrenches[0](2);
        LFx[i] = extWrenches[1](0);
        LFy[i] = extWrenches[1](1);
        LFz[i] = extWrenches[1](2);
        iDynTree::VectorDynSize jointTorques = id.getJointTorques();
        for (size_t j = 0; j < jointTorques.size(); j++)
        {
            jointTorquesMap[id.getJointsList()[j]][i] = jointTorques(j);
        }
    }
    RightHandXMatVec = RHx;
    RightHandYMatVec = RHy;
    RightHandZMatVec = RHz;
    LeftHandXMatVec = LHx;
    LeftHandYMatVec = LHy;
    LeftHandZMatVec = LHz;
    RightFootXMatVec = RFx;
    RightFootYMatVec = RFy;
    RightFootZMatVec = RFz;
    LeftFootXMatVec = LFx;
    LeftFootYMatVec = LFy;
    LeftFootZMatVec = LFz;
    // save the estimated joint torques and external wrenches in a mat file
    for (int i = 0; i < id.getJointsList().size(); i++)
    {
        jointTorquesMatVec[i] = jointTorquesMap[id.getJointsList()[i]];
        file.write(jointTorquesMatVec[i]);
    }
    file.write(RightHandXMatVec);
    file.write(RightHandYMatVec);
    file.write(RightHandZMatVec);
    file.write(LeftHandXMatVec);
    file.write(LeftHandYMatVec);
    file.write(LeftHandZMatVec);
    file.write(RightFootXMatVec);
    file.write(RightFootYMatVec);
    file.write(RightFootZMatVec);
    file.write(LeftFootXMatVec);
    file.write(LeftFootYMatVec);
    file.write(LeftFootZMatVec);

    return 0;
}
