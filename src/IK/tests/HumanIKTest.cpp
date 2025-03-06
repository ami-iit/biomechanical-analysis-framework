// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/IK/InverseKinematics.h>
#include <iDynTree/ModelTestUtils.h>
#include <manif/SO3.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <ConfigFolderPath.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


TEST_CASE("InverseKinematics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);   
    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>();
    
    std::cout << "configPath = " << getConfigPath() + "/configTestIK.toml" << std::endl;



    /* CREATE THE PARAMETERS FOR THE JOINT_REG TASK*/
    std::vector<std::string> joints_list_kp, joints_list;
    std::vector<double> joints_kp, joints_weights;

    // Iter over all the joints of the model
    for (int i = 0; i < model.getNrOfJoints(); i++) {
        auto joint = model.getJoint(i);

        // Check if the joint has only one degree of freedom
        if (joint->getNrOfDOFs() == 1) {
            std::string jointName = model.getJointName(i);
            joints_list_kp.push_back(jointName);
            joints_kp.push_back(10.0);
            joints_list.push_back(jointName);
            joints_weights.push_back(0.1);
        }
    }

    std::string filename = getConfigPath() + "/configTestIK.toml";
    std::ifstream inputFile(filename);
    std::string tempFilename = getConfigPath() + "/configTestIKTEST.toml";
    std::ofstream outputFile(tempFilename);
    std::string line;
    bool inTargetSection = false;
    bool foundType = false;
    bool foundVelocity = false;
    bool foundWeight = false;

    while (std::getline(inputFile, line)) {
        outputFile << line << "\n";
        
        if (line == "[JOINT_REG_TASK]") {
            inTargetSection = true;
        } else if (inTargetSection) {
            if (line.find("type") != std::string::npos) {
                foundType = true;
            } else if (line.find("robot_velocity_variable_name") != std::string::npos) {
                foundVelocity = true;
            } else if (line.find("weight") != std::string::npos) {
                foundWeight = true;
            }
            
            if (foundType && foundVelocity && foundWeight) {
                outputFile << "joints_list = [";
                for (size_t i = 0; i < joints_list.size(); ++i) {
                    outputFile << "\"" << joints_list[i] << "\"";
                    if (i != joints_list.size() - 1) {
                        outputFile << ", ";
                    }
                }
                outputFile << "]\n"; // End of joints_list line
                
                outputFile << "joints_weights = [";
                for (size_t i = 0; i < joints_weights.size(); ++i) {
                    outputFile << joints_weights[i];
                    if (i != joints_weights.size() - 1) {
                        outputFile << ", ";
                    }
                }
                outputFile << "]\n"; // End of joints_weights line
                
                outputFile << "joints_list_kp = [";
                for (size_t i = 0; i < joints_list_kp.size(); ++i) {
                    outputFile << "\"" << joints_list_kp[i] << "\"";
                    if (i != joints_list_kp.size() - 1) {
                        outputFile << ", ";
                    }
                }
                outputFile << "]\n"; // End of joints_list_kp line
                
                outputFile << "joints_kp = [";
                for (size_t i = 0; i < joints_kp.size(); ++i) {
                    outputFile << std::fixed << std::setprecision(1) << joints_kp[i];
                    if (i != joints_kp.size() - 1) {
                        outputFile << ", ";
                    }
                }
                outputFile << "]\n"; // End of joints_kp line
    
                inTargetSection = false;
            }
        }
    }
    
    inputFile.close();
    outputFile.close();
    /* END CREATE THE PARAMETERS FOR THE JOINT_REG TASK*/



    // set the parameters from the config file
    REQUIRE(paramHandler->setFromFile(tempFilename));

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


    REQUIRE(ik.initialize(paramHandler, kinDyn));
    REQUIRE(ik.setDt(0.1));
    REQUIRE(ik.updateOrientationTask(3, I_R_IMU, I_omega_IMU));
    REQUIRE(ik.updateFloorContactTask(10, 11.0));
    REQUIRE(ik.updateGravityTask(10, I_R_IMU));
    REQUIRE(ik.updateOrientationAndGravityTasks(mapNodeData));
    REQUIRE(ik.updateJointConstraintsTask());
    Eigen::VectorXd jointPositionSetPoint = ik.getJointPositionSetPoint();
    REQUIRE(ik.updateJointRegularizationTask(jointPositionSetPoint));
    REQUIRE(ik.calibrateWorldYaw(mapNodeData));
    REQUIRE(ik.calibrateAllWithWorld(mapNodeData, "link1"));
    REQUIRE(ik.advance());
    REQUIRE(ik.getJointPositions(JointPositions));
    REQUIRE(ik.getJointVelocities(JointVelocities));

    std::cout << "JointPositions = " << JointPositions.transpose() << std::endl;
    std::cout << "JointVelocities = " << JointVelocities.transpose() << std::endl;

    // Remove the temporary file after the test
    std::remove(tempFilename.c_str());
}


