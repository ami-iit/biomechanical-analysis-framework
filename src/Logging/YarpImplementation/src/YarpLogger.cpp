#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <BiomechanicalAnalysis/Logging/YarpLogger.h>

#include <BipedalLocomotion/TextLogging/YarpLogger.h>

void BiomechanicalAnalysis::Logging::useYarpLogger()
{
    static std::shared_ptr<BipedalLocomotion::TextLogging::LoggerFactory> _bafYarpLogFactory = 
        std::dynamic_pointer_cast<BipedalLocomotion::TextLogging::LoggerFactory>(std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>("baf"));
    BiomechanicalAnalysis::Logging::setLoggerFactory(_bafYarpLogFactory);
}
