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

    auto paramHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    paramHandler->setFromFile(getConfigPath() + "/configExampleID.ini");

    iDynTree::ModelLoader mdlLoader; // Create a ModelLoader object
    std::string urdfPath = rf.findFileByName("humanSubject03_48dof.urdf"); // Find the URDF file
                                                                           // path
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList()); // Load the URDF model and create
                                                                   // a reduced model
    kinDyn->loadRobotModel(mdlLoader.model()); // Load the model into the KinDynComputations object
    kinDyn->setFloatingBase("Pelvis"); // Set the floating base of the model

    BiomechanicalAnalysis::ID::HumanID id; // Create an object of the HumanID class
    id.initialize(paramHandler, kinDyn); // Initialize the HumanID object

    std::string matFilePath;
    if (!paramHandler->getParameter("matFilePath", matFilePath))
    {
        BiomechanicalAnalysis::log()->error("Error in reading the mat file path");
        return -1;
    }

    matioCpp::File file2(matFilePath);
    matioCpp::Struct human_data = file2.read("human_data").asStruct();
    matioCpp::Struct human_state = human_data("human_state").asStruct();
    matioCpp::Struct human_wrench = human_data("human_wrench").asStruct();
    matioCpp::Struct wrenches = human_wrench("wrenches").asStruct();
    matioCpp::Struct joint_positions = human_state("joint_positions").asStruct();
    matioCpp::Struct joint_velocities = human_state("joint_velocities").asStruct();
    matioCpp::Struct base_velocity = human_state("base_velocity").asStruct();
    matioCpp::Struct base_position = human_state("base_position").asStruct();
    matioCpp::Struct base_orientation = human_state("base_orientation").asStruct();
    matioCpp::MultiDimensionalArray<double> jointPosData
        = joint_positions("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> wrenchesData
        = wrenches("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> jointVelData
        = joint_velocities("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> baseVelData
        = base_velocity("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> basePosData
        = base_position("data").asMultiDimensionalArray<double>();
    matioCpp::MultiDimensionalArray<double> baseOriData
        = base_orientation("data").asMultiDimensionalArray<double>();
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

    for (size_t i = 0; i < 2; i++)
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
            rightFootWrench(j) = wrenchesData({j, 0, i});
            leftFootWrench(j) = wrenchesData({j + 6, 0, i});
        }
        wrenchesMap["RightFoot"] = rightFootWrench;
        wrenchesMap["LeftFoot"] = leftFootWrench;
        iDynTree::Rotation rot = iDynTree::Rotation::RotationFromQuaternion(orientation);
        basePose.topLeftCorner(3, 3) = iDynTree::toEigen(rot);
        kinDyn->setRobotState(basePose, jointPos, baseVelocity, jointVel, gravity);
        if (!id.updateExtWrenchesMeasurements(wrenchesMap))
        {
            std::cerr << "Error in updating external wrenches" << std::endl;
            return -1;
        }
        if (!id.solve())
        {
            std::cerr << "Error in solving the inverse dynamics" << std::endl;
            return -1;
        }
        for (int i = 0; i < id.getEstimatedExtWrenches().size(); i++)
        {
            std::cout << "Estimated external wrenches: "
                      << id.getEstimatedExtWrenches()[i].toString() << std::endl;
        }
        iDynTree::VectorDynSize jointTorques = id.getJointTorques();
        std::cout << "Joint torques: " << jointTorques.toString() << std::endl;
    }

    return 0;
}
