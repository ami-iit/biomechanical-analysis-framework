#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelTestUtils.h>
#include <yarp/os/ResourceFinder.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <ConfigFolderPath.h>

TEST_CASE("Inverse Dynamics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::TomlImplementation>();
    paramHandler->setFromFile(getConfigPath() + "/configTestID.toml");

    auto extWrenchGroup = paramHandler->getGroup("JOINT_TORQUES");
    if (!extWrenchGroup.lock())
    {
        std::cerr << "Error while reading the JOINT_TORQUES group" << std::endl;
    }
    double var;
    if (!extWrenchGroup.lock()->getParameter("cov_dyn_variables", var))
    {
        std::cerr << "Error while reading the cov_dyn_variables parameter" << std::endl;
    }
    std::cout << "var = " << var << std::endl;

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
