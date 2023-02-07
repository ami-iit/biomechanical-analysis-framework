// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BiomechanicalAnalysis/Logging/Logger.h>

/**
 * This test simply checks that the log runs fine.
 */
TEST_CASE("Default logger test")
{
    //TODO check that the output message is the expected one?
    BiomechanicalAnalysis::log()->info("Test message: {}", "test");
}
