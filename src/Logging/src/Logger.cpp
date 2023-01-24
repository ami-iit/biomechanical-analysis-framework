#include <BiomechanicalAnalysis/Logging/Logger.h>

#include <BipedalLocomotion/TextLogging/DefaultLogger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>

static std::shared_ptr<BipedalLocomotion::TextLogging::LoggerFactory> _defaultFactory = 
        std::make_shared<BipedalLocomotion::TextLogging::DefaultLoggerFactory>("baf");

static std::shared_ptr<BipedalLocomotion::TextLogging::LoggerFactory> _bafFactory = _defaultFactory;

using namespace BiomechanicalAnalysis;

void Logging::setLoggerFactory(const std::shared_ptr<LoggerFactory>& loggerFactory)
{
    _bafFactory = loggerFactory;
}

Logging::Logger* const log()
{
    static bool _loggerCreated = []{BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(_bafFactory); return true;};
    return BipedalLocomotion::log();
}

void Logging::setVerbosity(const Verbosity verbosity)
{
    BipedalLocomotion::TextLogging::setVerbosity(verbosity);
}

