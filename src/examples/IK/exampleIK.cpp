/**
 * @file ExampleIK.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#include <iostream> // defines I/O standard C++ classes (out,in,err, etc.)
#include <thread> //handling threads

#include <iDynTree/EigenHelpers.h> //support for linear algebra
#include <iDynTree/ModelLoader.h> // loads URDF models
#include <iDynTree/ModelTestUtils.h> // utility for models testing
#include <iDynTree/Visualizer.h> // functionality for models visualization
#include <matioCpp/matioCpp.h> // reading/writing of Matlab files
#include <yarp/os/ResourceFinder.h> //finding resources

#include <BiomechanicalAnalysis/Conversions/CommonConversions.h> //conversion of rotation from iDyntree to Eigen
#include <BiomechanicalAnalysis/IK/InverseKinematics.h> // declaration of classes for IK inherited from iDyntree
#include <BiomechanicalAnalysis/Logging/Logger.h> //handle logging
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h> //handle parameters for Yarp

#include <ConfigFolderPath.h>

using namespace BiomechanicalAnalysis::Conversions;

std::atomic<bool> tPoseFlag{false}; // Flag indicating if T-pose has been requested
std::atomic<bool> stopThread{false}; // Flag used to terminate a thread

/**
 * Retrieves the orientation and angular velocity of a specified node starting from info from matlab file.
 * 
 * @param ifeel_struct Reference to the structure containing node data coming from Matlab file, and stored using matioCpp lib
 * @param nodeNum Number of the node to retrieve data for
 * @param index_i Index of the data point to retrieve
 * @param I_R_IMU Reference to store the orientation of the node
 * @param I_omeg_IMU Reference to store the angular velocity of the node
 * @return True if the data retrieval is successful, false otherwise
 */
bool getNodeOrientation(matioCpp::Struct& ifeel_struct,
                        int nodeNum,
                        size_t index_i,
                        iDynTree::Rotation& I_R_IMU,
                        iDynTree::AngVelocity& I_omeg_IMU)
{
    // Retrieve the node data structure from matlab file
    matioCpp::Struct vLink_node = ifeel_struct("iFeelSuit_vLink_Node_" + std::to_string(nodeNum)).asStruct(); 
    if (!vLink_node.isValid())
    {
        // Log an error if the node data structure is not valid
        BiomechanicalAnalysis::log()->error("Cannot read the iFeelSuit_vLink_Node_{} struct", std::to_string(nodeNum));
        return false;
    }

    // Retrieve the array containing node data (contained in vLink struct) and store in data1
    matioCpp::MultiDimensionalArray<double> data1 = vLink_node("data").asMultiDimensionalArray<double>();
    if (!data1.isValid())
    {
        // Log an error if the node data array is not valid
        BiomechanicalAnalysis::log()->error("Cannot read the data field of the node number {}", std::to_string(nodeNum));
        return false;
    }

    // Define indices for accessing data fields

    // index for element 0
    size_t index0 = 0;

    // indices for quaternion in the array
    size_t index_w = 4;
    size_t index_x = 5;
    size_t index_y = 6;
    size_t index_z = 7;

    // indices for angular velocity in the array
    size_t angVel_x = 11;
    size_t angVel_y = 12;
    size_t angVel_z = 13;

    // Define a 4-elements array to store quaternion components
    iDynTree::Vector4 quat;

    // Retrieve quaternion components from the data array
    quat[1] = data1({index_x, index0, index_i});
    quat[2] = data1({index_y, index0, index_i});
    quat[3] = data1({index_z, index0, index_i});
    quat[0] = data1({index_w, index0, index_i});

    // Retrieve angular velocity components from the data array
    I_omeg_IMU[0] = data1({angVel_x, index0, index_i});
    I_omeg_IMU[1] = data1({angVel_y, index0, index_i});
    I_omeg_IMU[2] = data1({angVel_z, index0, index_i});

    // Calculate the orientation from quaternion components
    I_R_IMU = iDynTree::Rotation::RotationFromQuaternion(quat);

    return true;
}


/**
 * Retrieves the vertical force acting on a specified node starting from info from matlab file.
 * 
 * @param ifeel_struct Reference to the structure containing node data coming from Matlab file, and stored using matioCpp lib
 * @param nodeNum Number of the node to retrieve data for
 * @param index_i Index of the data point to retrieve
 * @param force Reference to store the vertical force acting on the node
 * @return True if the data retrieval is successful, false otherwise
 */
bool getNodeVerticalForce(matioCpp::Struct& ifeel_struct, int nodeNum, size_t index_i, double& force)
{
    // Retrieve the node data structure
    // Store the ifeel_struct (input coming from matlab) in vLink_node
    matioCpp::Struct vLink_node = ifeel_struct("iFeelSuit_ft6D_Node_" + std::to_string(nodeNum)).asStruct(); 
    if (!vLink_node.isValid())
    {
        // Log an error if the node data structure is not valid
        BiomechanicalAnalysis::log()->error("Cannot read the iFeelSuit_vLink_Node_{} struct", std::to_string(nodeNum));
        return false;
    }

    // Retrieve the multi-dimensional array containing node data
    matioCpp::MultiDimensionalArray<double> data1 = vLink_node("data").asMultiDimensionalArray<double>();
    if (!data1.isValid())
    {
        // Log an error if the node data array is not valid
        BiomechanicalAnalysis::log()->error("Cannot read the data field of the node number {}", std::to_string(nodeNum));
        return false;
    }

    // Define index for accessing vertical force data field
    size_t index0 = 0;
    size_t index_z = 3;

    // Retrieve the vertical force component from the data array
    force = data1({index_z, index0, index_i});

    return true;
}

/**
 * Creates a list of joint names.
 * 
 * @return A vector of strings containing the names of the joints.
 */
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

/**
 * Thread function to set the T-pose flag based on user input.
 */
void setTPoseThread()
{
    // Log a message indicating how to trigger T-pose calibration
    BiomechanicalAnalysis::log()->info("Enter 'calib' to run the T-pose calibration");
    
    // Variable to store user input
    std::string input;

    // Continuously loop until stopThread flag is set
    while (!stopThread)
    {
        // Read user input from the console
        std::cin >> input;

        // Check if the input is "calib"
        if (input == "calib")
        {
            // Set the T-pose flag to true
            tPoseFlag = true;
        } 
        else
        {
            // Log a warning message for invalid input
            BiomechanicalAnalysis::log()->warn("{} is an invalid input; please enter 'calib' if you want to run the T-pose calibration", input);
        }

        // Wait for a short duration before checking for input again
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main()
{
    // Initialize ResourceFinder
    yarp::os::ResourceFinder rf;

    
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>(); // Create a KinDynComputations object
    iDynTree::ModelLoader mdlLoader; // Create a ModelLoader object
    std::string urdfPath = rf.findFileByName("humanSubject03_48dof.urdf"); // Find the URDF file path
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList()); // Load the URDF model and create a reduced model
    kinDyn->loadRobotModel(mdlLoader.model()); // Load the model into the KinDynComputations object
    kinDyn->setFloatingBase("Pelvis"); // Set the floating base of the model

    /**
    Initialize the visualizer
     */

    iDynTree::Visualizer viz; // Create a Visualizer object
    iDynTree::VisualizerOptions options; // Create options for the visualizer
    options.winWidth = 800; // Set window width
    options.winHeight = 600; // Set window height
    viz.init(options); // Initialize the visualizer with the options
    viz.setColorPalette("meshcat"); // Set the color palette for visualization

    // Set camera position and enable mouse control
    iDynTree::Position cameraPosition; // Define camera position
    cameraPosition.zero(); // Set camera position to zero
    cameraPosition[0] = 2.0; // Set camera x position
    cameraPosition[1] = 0.0; // Set camera y position
    cameraPosition[2] = 0.5; // Set camera z position
    viz.camera().setPosition(cameraPosition); // Set camera position
    viz.camera().animator()->enableMouseControl(true); // Enable mouse control for the camera

    // Add model to visualizer and set color
    viz.addModel(mdlLoader.model(), "model"); // Add the model to the visualizer
    iDynTree::IModelVisualization& modelViz = viz.modelViz("model"); // Get model visualization
    viz.addModel(mdlLoader.model(), "model2"); // Add another model to the visualizer
    iDynTree::IModelVisualization& modelViz2 = viz.modelViz("model2"); // Get second model visualization
    iDynTree::ColorViz modelColor(1.0, 0.0, 0.0, 1.0); // Create color (red)
    modelViz2.setModelColor(modelColor); // Set model color

    viz.draw(); // Draw the visualizer


    /**
    Initialize variables for joint positions, velocities, base pose, etc.
     */

    iDynTree::Rotation I_R_IMU; // Rotation of the IMU in the inertial frame
    iDynTree::AngVelocity I_omega_IMU; // Angular velocity of the IMU
    Eigen::VectorXd initialJointPositions; // Initial positions of the joints
    Eigen::VectorXd initialJointVelocities; // Initial velocities of the joints
    Eigen::VectorXd jointPositions; // Current positions of the joints
    Eigen::VectorXd jointVelocities; // Current velocities of the joints
    Eigen::Vector3d gravity; // Gravity vector
    Eigen::Matrix4d basePose; // Pose of the base
    Eigen::Matrix<double, 6, 1> baseVelocity; // Linear and angular velocity of the base
    Eigen::Matrix3d baseOrientation; // Orientation of the base
    Eigen::Vector3d basePosition; // Position of the base
    iDynTree::VectorDynSize jointPos, jointPos2; // Dynamic-size vectors for joint positions
    iDynTree::Transform w_H_b = iDynTree::Transform::Identity(); // Transformation from the world frame to the base frame
    iDynTree::Transform w_H_b2 = iDynTree::Transform::Identity(); // Another transformation from the world frame to the base frame
    iDynTree::Position basePositionOld; // Previous position of the base
    iDynTree::Position cameraDeltaPosition; // Delta position of the camera
    jointPos.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize the joint position vector based on the number of degrees of freedom
    jointPos2.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize another joint position vector based on the number of degrees of freedom
    jointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize the joint velocity vector based on the number of degrees of freedom
    jointPositions.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize the joint position vector based on the number of degrees of freedom
    initialJointPositions.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize the initial joint position vector based on the number of degrees of freedom
    initialJointVelocities.resize(kinDyn->getNrOfDegreesOfFreedom()); // Resize the initial joint velocity vector based on the number of degrees of freedom
    initialJointPositions.setZero(); // Set initial joint positions to zero
    gravity << 0.0, 0.0, -9.81; // Assign values to the gravity vector
    basePose.setIdentity(); // Set the base pose to the identity matrix
    baseVelocity.setZero(); // Set base velocity to zero


    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(); // Create a parameter handler

    // Set parameters from a configuration file
    if (!paramHandler->setFromFile(getConfigPath() + "/exampleIK.ini")) // Check if setting parameters from file is successful
    {
        BiomechanicalAnalysis::log()->error("Cannot configure the parameter handler"); // Log error if configuration fails
        return 1; // Return error code
    }

    // Load data from Matlab files
    matioCpp::File file("/path/to/matlab1.mat");
    matioCpp::Struct ifeel_data = file.read("ifeel_data").asStruct();
    matioCpp::Struct node12 = ifeel_data("iFeelSuit_vLink_Node_12").asStruct();
    matioCpp::File file2("/path/to/human_data.mat");    matioCpp::Struct human_data = file2.read("human_data").asStruct(); // Read "human_data" structure from the MATLAB file and convert it to a structure
    matioCpp::Struct human_data = file2.read("human_data").asStruct(); // Read "human_data" structure from the MATLAB file and convert it to a structure    
    matioCpp::Struct human_state = human_data("human_state").asStruct(); // Access structure "human_state" within "human_data" structure and convert it to a structure
    matioCpp::Struct joint_positions = human_state("joint_positions").asStruct(); // Access structure "joint_positions" within "human_state" structure and convert it to a structure
    matioCpp::MultiDimensionalArray<double> jointPos_data = joint_positions("data").asMultiDimensionalArray<double>(); // Access multidimensional array "data" within "joint_positions" structure and convert it to a multidimensional array of type double
    matioCpp::Struct base_position = human_state("base_position").asStruct(); // Access structure "base_position" within "human_state" structure and convert it to a structure
    matioCpp::MultiDimensionalArray<double> basePos_data = base_position("data").asMultiDimensionalArray<double>(); // Access multidimensional array "data" within "base_position" structure and convert it to a multidimensional array of type double
    matioCpp::Struct base_orientation = human_state("base_orientation").asStruct(); // Access structure "base_orientation" within "human_state" structure and convert it to a structure
    matioCpp::MultiDimensionalArray<double> baseOrientation_data = base_orientation("data").asMultiDimensionalArray<double>(); // Access multidimensional array "data" within "base_orientation" structure and convert it to a multidimensional array of type double


    // Set initial base position from data
    iDynTree::Position w_p_b2_init;
    w_p_b2_init[0] = basePos_data({0, 0, 0});
    w_p_b2_init[1] = basePos_data({1, 0, 0});
    w_p_b2_init[2] = 0;

    // Set initial joint positions from data
    for (size_t ii = 0; ii < 31; ii++)
    {
        initialJointPositions[ii] = jointPos_data({ii, 0, 0});
    }

    // Get the dimension of the data
    matioCpp::Vector<double> data = node12("timestamps").asVector<double>();
    size_t dataLength = data.size();

    // Define lists of nodes
    std::vector<int> orientationNodes = {3, 6, 7, 8, 5, 4, 11, 12, 9, 10};
    std::vector<int> floorContactNodes = {1, 2};

    // Initialize the HUMANIK-class object ik: initialize Inverse Kinematics (IK) solver with related tasks, assigning the kinDyn object
    BiomechanicalAnalysis::IK::HumanIK ik;
    if (!ik.initialize(paramHandler, kinDyn)) // Check if IK initialization is successful
    {
        BiomechanicalAnalysis::log()->error("Cannot initialize the inverse kinematics solver"); // Log error if initialization fails
        return 1; // Return error code
    }

    // Set robot state for IK
    kinDyn->setRobotState(basePose, initialJointPositions, baseVelocity, initialJointVelocities, gravity); 
    ik.setDt(0.01); // Set time step for IK

    // Start T-pose calibration thread
    std::thread tPoseThread = std::thread(setTPoseThread);


    // Continuosly read from the sensor measurements (span time instants) and compute the IK

    // Cycle over the time instants
    for (size_t ii = 0; ii < dataLength; ii++)
    {
        // Perform the T-pose calibration if the user inputs 'calib'
        if (tPoseFlag)
        {
            for (auto& node : orientationNodes)
            {
                // Retrieve orientation and angular velocity for each node
                getNodeOrientation(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);

                // Perform T-Pose calibration for each node
                ik.TPoseCalibrationNode(node, manif::SO3d(fromiDynTreeToEigenQuatConversion(I_R_IMU)));
            }
            BiomechanicalAnalysis::log()->info("T-pose calibration done");
            tPoseFlag = false;
        }

        // Update orientation task
         
        for (auto& node : orientationNodes)// Cycle over the IMU nodes
        {
            // Retrieve orientation and angular velocity for each node
            getNodeOrientation(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);

            // Convert orientation and angular velocity to manif objects
            manif::SO3d I_R_IMU_manif = manif::SO3d(fromiDynTreeToEigenQuatConversion(I_R_IMU));
            manif::SO3Tangentd I_omega_IMU_manif = manif::SO3Tangentd(iDynTree::toEigen(I_omega_IMU));

            // IK solver: Update orientation task for the current node
            if (!ik.updateOrientationTask(node, I_R_IMU_manif, I_omega_IMU_manif))
            {
                BiomechanicalAnalysis::log()->error("[error] Cannot set the node number {} set point", node);
                return 1;
            }
        }

        // Update floor contact tasks and gravity tasks

        for (auto& node : floorContactNodes) // Cycle over the shoes nodes
        {
            // vertical force for each node
            double force;

            // Retrieve orientation and angular velocity for each node
            getNodeOrientation(ifeel_data, node, ii, I_R_IMU, I_omega_IMU);

            // Retrieve vertical force for each node
            getNodeVerticalForce(ifeel_data, node, ii, force);

            // IK solver: Update floor contact task for the current node
            ik.updateFloorContactTask(node, force);

            // Convert orientation to a manif object 
            manif::SO3d I_R_IMU_manif = manif::SO3d(fromiDynTreeToEigenQuatConversion(I_R_IMU));
            
            // IK solver: update gravity task
            ik.updateGravityTask(node, I_R_IMU_manif);
        }

        // IK Solver: Update joint constraints and regularization tasks
        ik.updateJointConstraintsTask();
        ik.updateJointRegularizationTask();

        // Advance the inverse kinematics solver: compute solution
        if (!ik.advance())
        {
            BiomechanicalAnalysis::log()->error("Cannot advance the inverse kinematics solver");
            return 1;
        }

        // Retrieve resulting joint positions, joint velocities, base orientation, and base position (input vector updating)
        ik.getJointPositions(jointPositions);
        ik.getJointVelocities(jointVelocities);
        ik.getBaseOrientation(baseOrientation);
        ik.getBasePosition(basePosition);


        // Update joint positions for visualization
        for (size_t jj = 0; jj < 31; jj++)
        {
            jointPos2[jj] = jointPos_data({jj, 0, ii});
        }

        // Update base position and orientation
        iDynTree::Position w_p_b2;
        w_p_b2[0] = basePos_data({0, 0, ii});
        w_p_b2[1] = basePos_data({1, 0, ii});
        w_p_b2[2] = basePos_data({2, 0, ii});

        // Convert the quaternion of the base in rotation matrix
        iDynTree::Rotation w_R_b2;
        iDynTree::Vector4 quat2;

        quat2[0] = baseOrientation_data({0, 0, ii});
        quat2[1] = baseOrientation_data({1, 0, ii});
        quat2[2] = baseOrientation_data({2, 0, ii});
        quat2[3] = baseOrientation_data({3, 0, ii});
        w_R_b2 = iDynTree::Rotation::RotationFromQuaternion(quat2);

        iDynTree::Rotation w_R_b;
        iDynTree::Position w_p_b;
        iDynTree::toEigen(w_R_b) = baseOrientation;
        iDynTree::toEigen(w_p_b) = basePosition;

        // Set joint positions and visualize models
        iDynTree::toEigen(jointPos) = jointPositions;
        w_H_b.setRotation(w_R_b);
        w_H_b2.setRotation(w_R_b2);
        w_H_b.setPosition(w_p_b);
        w_H_b2.setPosition(w_p_b2 - w_p_b2_init);

        // Update camera position and visualize
        cameraDeltaPosition = viz.camera().getPosition() - basePositionOld;
        viz.camera().setPosition(w_p_b + cameraDeltaPosition);
        viz.camera().setTarget(w_p_b);
        basePositionOld = w_p_b;

        modelViz.setPositions(w_H_b, jointPos);
        modelViz2.setPositions(w_H_b2, jointPos2);

        viz.draw();
    }

    // Terminate the T-pose calibration thread
    stopThread = true;

    // Inform the user to enter a character to terminate the program
    BiomechanicalAnalysis::log()->info("\nPlease enter a character to terminate the program");

    // Wait for the T-pose calibration thread to join
    if (tPoseThread.joinable())
    {
        tPoseThread.join();
    }

    // Log the completion of the program
    BiomechanicalAnalysis::log()->info("done");

    // Return 0 to indicate successful program execution
    return 0;
}
