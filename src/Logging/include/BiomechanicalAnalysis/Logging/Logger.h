/**
 * @file Logger.h
 * @authors Riccardo Grieco <riccardo.grieco@hotmail.com>
 */

#ifndef BIOMECHANICAL_ANALYSIS_LOGGING_LOGGER_H
#define BIOMECHANICAL_ANALYSIS_LOGGING_LOGGER_H

#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BiomechanicalAnalysis
{
namespace Logging
{

using Logger = BipedalLocomotion::TextLogging::Logger;
using Verbosity = BipedalLocomotion::TextLogging::Verbosity;
using LoggerFactory = BipedalLocomotion::TextLogging::LoggerFactory;

/**
 * Set the log provider.
 * Calling this method after log() will have no effect.
 *
 * @param loggerFactory
 */
void setLoggerFactory(const std::shared_ptr<LoggerFactory>& loggerFactory);

/**
 * Set the verbosity of the logger.
 *
 * @param verbosity the verbosity of the logger
 */
void setVerbosity(const Verbosity verbosity);

} // namespace Logging
} // namespace BiomechanicalAnalysis

namespace BiomechanicalAnalysis
{
/**
 * Get an the instance of the log
 */
std::shared_ptr<Logging::Logger> const log();

} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_LOGGING_LOGGER_H
