
add_biomechanical_analysis_library(
    NAME                   CommonConversions
    IS_INTERFACE
    PUBLIC_HEADERS         include/BiomechanicalAnalysis/Conversions/CommonConversions.h
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
    INSTALLATION_FOLDER    Conversions
)
