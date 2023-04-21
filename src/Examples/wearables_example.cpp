
#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <BiomechanicalAnalysis/DataSources/Pose.h>
#include <Wearable/IWear/IWear.h>
#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/Force6DSensor.h>
#include <BiomechanicalAnalysis/WearablesImplementation/DataSources/VirtualLinkSensor.h>
#include <Eigen/Dense>

#include <BiomechanicalAnalysis/DynamicalIK/InverseVelocityKinematics.h>
#include <BiomechanicalAnalysis/DynamicalIK/StdTarget.h>

#include <yarp/dev/PolyDriver.h>

#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <thread>

namespace BAF = BiomechanicalAnalysis;
namespace BAF_DS = BAF::DataSources;
namespace BAF_DIK = BAF::DynamicalIK;


int main()
{
    constexpr int numberOfJoints{6};
    {
    BipedalLocomotion::log()->debug("Qui0");
    }

    iDynTree::ModelLoader loader;
    loader.loadModelFromFile("/home/riccardo/Code/robotology-superbuild/build/install/share/human-gazebo/humanSubject07_48dof.urdf");
    //const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
    const iDynTree::Model model = loader.model();
    std::shared_ptr<iDynTree::Model> modelPtr =  std::make_shared<iDynTree::Model>(model);
    iDynTree::Visualizer visualizer;
    iDynTree::VisualizerOptions options;
    //options.winHeight = 1000;
    //options.winWidth = 1000;
    visualizer.init(options);
    
    visualizer.setColorPalette("meshcat");

    //visualizer.camera().setPosition(iDynTree::Position::Zero());
    //visualizer.camera().setTarget(fixedCameraTarget);

    visualizer.camera().animator()->enableMouseControl(true);

    visualizer.addModel(model, "model");

    BiomechanicalAnalysis::DynamicalIK::InverseVelocityKinematics iksolver;

    // initialize solver
    iksolver.setModel(modelPtr);
    iksolver.setBaseFrame(model.getLinkName(model.getDefaultBaseLink()));

    const Eigen::Quaterniond constQuaternion = Eigen::Quaterniond::Identity();
    std::shared_ptr<BAF_DS::Orientation<double>> orientationSource = std::make_shared<BiomechanicalAnalysis::DataSources::ConstOrientation<double>>(constQuaternion);


    const std::string portName = "/XSensSuit/WearableData/data:o";//"/iFeelSuit/WearableData/data:o";
    yarp::os::Bottle bot;
    auto& portList = bot.addList();
    portList.addString(portName);

    yarp::os::Property prop;
    prop.put("device", "iwear_remapper");
    prop.put("wearableDataPorts", bot.get(0));

    yarp::dev::PolyDriver myDev(prop);
    if(!myDev.isValid())
    {
        BiomechanicalAnalysis::log()->error("Unable to open device");
    }


    wearable::IWear* iWearSource;
    if(!myDev.view(iWearSource))
    {
        BiomechanicalAnalysis::log()->error("Unable to view device as IWear interface");
    }

    std::unordered_map<std::string, std::string> sensorToFrame;
    // sensorToFrame.insert({"XsensSuit::vLink::RightLowerLeg", "RightLowerLeg"});
    // sensorToFrame.insert({"XsensSuit::vLink::LeftLowerLeg", "LeftLowerLeg"});
    // sensorToFrame.insert({"XsensSuit::vLink::LeftUpperLeg", "LeftUpperLeg"});
    // sensorToFrame.insert({"XsensSuit::vLink::RightUpperLeg", "RightUpperLeg"});
    sensorToFrame.insert({"XsensSuit::vLink::Pelvis", "Pelvis"});


    std::unordered_map<std::string, std::shared_ptr<BAF_DS::Wearables::VirtualLinkSensor>> nameToSensorSource;

    std::vector<std::shared_ptr<BAF_DS::Wearables::VirtualLinkSensor>> vSensorDataSources;
    std::vector<std::shared_ptr<BAF_DIK::StdOrientationTarget>> orientationTargets;
    std::vector<std::shared_ptr<BAF_DIK::StdPositionTarget>> positionTargets;


    while(iWearSource->getStatus()!=wearable::WearStatus::Ok)
    {
        BiomechanicalAnalysis::log()->info("Waiting for iwear to be ready");
    }

    for(auto& vLink : iWearSource->getVirtualLinkKinSensors())
    {
        BiomechanicalAnalysis::log()->info("Found sensor {}", vLink->getSensorName());
        if(sensorToFrame.find(vLink->getSensorName()) == sensorToFrame.end()){
            continue;
        }
        BiomechanicalAnalysis::log()->info("Adding tasks for sensor {}", vLink->getSensorName());

        auto vLinkSensorDataSource = std::make_shared<BAF_DS::Wearables::VirtualLinkSensor>(vLink);
        nameToSensorSource.insert({vLink->getSensorName(), vLinkSensorDataSource});

        auto orientationTarget = std::make_shared<BAF_DIK::StdOrientationTarget>(sensorToFrame[vLink->getSensorName()]+"_SO3");
        orientationTarget->setFrame(sensorToFrame[vLink->getSensorName()]);
        orientationTarget->setOrientationSource(std::dynamic_pointer_cast<BAF_DS::Orientation<double>>(vLinkSensorDataSource));
        orientationTarget->setAngularVelocitySource(std::dynamic_pointer_cast<BAF_DS::AngularVelocity<double>>(vLinkSensorDataSource));
        orientationTarget->setFeedbackGain(20);


        vSensorDataSources.push_back(vLinkSensorDataSource);
        orientationTargets.push_back(orientationTarget);

        iksolver.addTarget(orientationTarget);

    }

    auto& vLinkSensorDataSource = nameToSensorSource["XsensSuit::vLink::Pelvis"];
    auto positionTarget = std::make_shared<BAF_DIK::StdPositionTarget>("Pelvis_R3");
    positionTarget->setFrame("Pelvis");
    positionTarget->setPositionSource(std::dynamic_pointer_cast<BAF_DS::Position<double>>(vLinkSensorDataSource));
    positionTarget->setLinearVelocitySource(std::dynamic_pointer_cast<BAF_DS::LinearVelocity<double>>(vLinkSensorDataSource));
    positionTarget->setFeedbackGain(-0.01);

    positionTargets.push_back(positionTarget);

    iksolver.addTarget(positionTarget);

    iksolver.setStep(0.01);
    

    if(!iksolver.initialize()){
        BiomechanicalAnalysis::log()->error("Unable to initialize the solver");
        return -1;
    }

    iDynTree::Transform baseTransform;
    iDynTree::VectorDynSize jointPos;
    jointPos.resize(iksolver.getNumberOfDofs());
    
    long it = 0;
    iDynTree::Transform startPos = iDynTree::Transform::Identity();
    while(visualizer.run())
    {
        // update data sources and targets
        for(auto& vLinkSensor : vSensorDataSources){
            if(!vLinkSensor->update()){
                BiomechanicalAnalysis::log()->error("Unable to update sensor");
            }
        }

        for(auto& posTarget : positionTargets){ 
            if(!posTarget->update()){
                BiomechanicalAnalysis::log()->error("Unable to update position target {}", posTarget->getName());
            }
        }

        for(auto& orientationTarget : orientationTargets){ 
            if(!orientationTarget->update()){
                BiomechanicalAnalysis::log()->error("Unable to update orientation target {}", orientationTarget->getName());
            }
        }

        if(!iksolver.update()){
            BiomechanicalAnalysis::log()->error("Unable to update the solver");
            return -1;
        }

        iDynTree::fromEigen(baseTransform, iksolver.getBasePose());

        if(it==200)
        {
            startPos.setPosition(baseTransform.getPosition());
            it=0;
        }

        iDynTree::toEigen(jointPos) = iksolver.getJointPositions(); 
        baseTransform.setPosition(baseTransform.getPosition()-startPos.getPosition());
        visualizer.modelViz("model").setPositions(baseTransform, jointPos);
        //visualizer.camera().setTarget(baseTransform.getPosition());
        iDynTree::Position cameraOffset;
        cameraOffset.zero();
        cameraOffset(0) = 10;
        //visualizer.camera().setPosition(baseTransform.getPosition()-cameraOffset);
        
        visualizer.draw();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        BiomechanicalAnalysis::log()->info("Loop");
        it++;
    }

    return 0;
}
