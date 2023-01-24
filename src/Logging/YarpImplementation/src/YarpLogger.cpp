#include <BiomechanicalAnalysis/Logging/Logger.h>
#include <BiomechanicalAnalysis/Logging/YarpLogger.h>

#include <BipedalLocomotion/TextLogging/YarpLogger.h>

using namespace BipedalLocomotion::TextLogging;

void BiomechanicalAnalysis::Logging::useYarpLogger()
{
    static auto _bafYarpLoggerFactoryBasePtr = std::make_shared<YarpLoggerFactory>("baf");
    static std::shared_ptr<LoggerFactory> _bafYarpLogFactory = std::dynamic_pointer_cast<LoggerFactory>(_bafYarpLoggerFactoryBasePtr);
    BiomechanicalAnalysis::Logging::setLoggerFactory(_bafYarpLogFactory);
}
