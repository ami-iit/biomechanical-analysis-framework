#include <BiomechanicalAnalysis/Logging/Logger.h>

#include <BipedalLocomotion/TextLogging/DefaultLogger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

static std::shared_ptr<BipedalLocomotion::TextLogging::LoggerFactory> _defaultFactory = 
        std::make_shared<BipedalLocomotion::TextLogging::DefaultLoggerFactory>("baf");

using namespace BiomechanicalAnalysis;

void Logging::setLoggerFactory(const std::shared_ptr<LoggerFactory>& loggerFactory)
{
    _defaultFactory = loggerFactory;
}

Logging::Logger* const log()
{
    static bool loggerCreated = []{BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(_defaultFactory); return;};
    return BipedalLocomotion::log();
}

void Logging::setVerbosity(const Verbosity verbosity)
{
    BipedalLocomotion::TextLogging::setVerbosity(verbosity);
}

