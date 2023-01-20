#ifndef BIOMECHANICAL_ANALYSIS_YARP_LOGGER_H
#define BIOMECHANICAL_ANALYSIS_YARP_LOGGER_H

namespace BiomechanicalAnalysis
{
namespace Logging
{

/**
 * Use YARP logger instead of the default one.
 * The method works only if the log has not yet been used
 */
void useYarpLogger();

}
}

#endif // BIOMECHANICAL_ANALYSIS_YARP_LOGGER_H