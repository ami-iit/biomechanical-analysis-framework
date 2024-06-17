#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>
#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <ConfigFolderPath.h>

TEST_CASE("Inverse Dynamics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;
    yarp::os::ResourceFinder rf;

    auto paramHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    paramHandler->setFromFile(getConfigPath() + "/configTestID.ini");

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);
    std::unordered_map<std::string, iDynTree::Wrench> wrenches;
    wrenches["link0"] = iDynTree::Wrench();
    wrenches["link1"] = iDynTree::Wrench();
    BiomechanicalAnalysis::ID::HumanID id;
    REQUIRE(id.initialize(paramHandler, kinDyn));
    REQUIRE(id.updateExtWrenchesMeasurements(wrenches));
    REQUIRE(id.solve());
}
