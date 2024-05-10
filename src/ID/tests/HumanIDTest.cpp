#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <iDynTree/ModelTestUtils.h>

TEST_CASE("Inverse Dynamics test")
{
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();

    // set the number of DoFs
    int nrDoFs = 20;

    const iDynTree::Model model = iDynTree::getRandomModel(nrDoFs);
    kinDyn->loadRobotModel(model);
    BiomechanicalAnalysis::ID::HumanID id;
    REQUIRE(id.initialize(kinDyn));
}
