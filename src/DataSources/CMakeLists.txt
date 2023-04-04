
set(H_PREFIX include/BiomechanicalAnalysis/DataSources)

add_biomechanical_analysis_library(
    NAME                   DataSources 
    IS_INTERFACE
    PUBLIC_HEADERS         ${H_PREFIX}/Position.h ${H_PREFIX}/Orientation.h ${H_PREFIX}/Pose.h
                           ${H_PREFIX}/LinearVelocity.h ${H_PREFIX}/AngularVelocity.h ${H_PREFIX}/Velocity6D.h
                           ${H_PREFIX}/LinearAcceleration.h ${H_PREFIX}/AngularAcceleration.h ${H_PREFIX}/Acceleration6D.h
                           ${H_PREFIX}/Force6D.h
                           ${H_PREFIX}/TransformedDataSource.h
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen)
