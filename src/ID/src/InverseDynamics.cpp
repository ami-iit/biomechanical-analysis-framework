#include <BiomechanicalAnalysis/ID/InverseDynamics.h>
#include <BiomechanicalAnalysis/Logging/Logger.h>

using namespace BiomechanicalAnalysis::ID;

bool HumanID::initialize(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{

    constexpr auto logPrefix = "[HumanID::intizialize]";

    // Check the validity of the kinDyn object
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        BiomechanicalAnalysis::log()->error("{} Invalid kinDyn object.", logPrefix);
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}
