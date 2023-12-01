/**
 * @file InverseKinematic.h
 * @authors Davide Gorbani <davide.gorbani@iit.it>
 */

#ifndef BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
#define BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H

#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace BiomechanicalAnalysis
{

namespace IK
{

class HumanIK
{
private:
    /* data */
public:
    HumanIK(){};
    ~HumanIK(){};
    bool setState();

};

} // namespace IK
} // namespace BiomechanicalAnalysis



#endif // BIOMECHANICAL_ANALYSIS_INVERSE_KINEMATIC_H
