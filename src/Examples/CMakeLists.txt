
if (${FRAMEWORK_COMPILE_Wearables_Implementation}) #TODO
    
    add_executable(wearables_example wearables_example.cpp)
    target_link_libraries(wearables_example PUBLIC 
                      BiomechanicalAnalysis::DataSources BiomechanicalAnalysis::Logging
                      BiomechanicalAnalysis::DynamicalIK
                      BiomechanicalAnalysis::WearablesImplementationDataSources
                      YARP::YARP_dev YARP::YARP_os
                      iDynTree::idyntree-visualization)

else()

    message(WARNING "Example wearables_example cannot be compiled without FRAMEWORK_COMPILE_Wearables_Implementation")
    
endif()
