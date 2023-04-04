
#ifndef BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H
#define BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H

#include <Eigen/Dense>

#include <BiomechanicalAnalysis/DataSources/Position.h>
#include <BiomechanicalAnalysis/DataSources/Orientation.h>

namespace BiomechanicalAnalysis
{
namespace DataSources
{

template<typename Scalar>
class Pose : public virtual Position<Scalar>, virtual Orientation<Scalar>
{
public:
    //TODO
    //inline virtual bool getPose(Eigen::Ref<Eigen::Matrix<Scalar,4,4>> pose) const
    //{
    //    // if(!getPosition(pose.block<3,1>(0,3)) || !getOrientation(pose.block<3,3>(0,0)))
    //    // {
    //    //     return false;
    //    // }
    //    
    //    auto a = pose.block<1,4>(3,0);// = {0, 0, 0, 1};
    //    
    //    return true;
    //}

};


} // namespace DataSources
} // namespace BiomechanicalAnalysis

#endif // BIOMECHANICAL_ANALYSIS_DATA_SOURCES_POSE_H
