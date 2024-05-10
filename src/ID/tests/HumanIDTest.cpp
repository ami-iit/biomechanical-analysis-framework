#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>
#include <yarp/os/ResourceFinder.h>

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

TEST_CASE("Inverse Dynamics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;
    yarp::os::ResourceFinder rf;

    iDynTree::ModelLoader mdlLoader; // Create a ModelLoader object
    std::string urdfPath = rf.findFileByName("humanSubject03_48dof.urdf"); // Find the URDF file
                                                                           // path
    mdlLoader.loadReducedModelFromFile(urdfPath, getJointsList()); // Load the URDF model and create
                                                                   // a reduced model
    kinDyn->loadRobotModel(mdlLoader.model()); // Load the model into the KinDynComputations object
    kinDyn->setFloatingBase("Pelvis"); // Set the floating base of the model
    BiomechanicalAnalysis::ID::HumanID id;
    REQUIRE(id.initialize(kinDyn));
}
