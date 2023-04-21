
add_subdirectory(Logging)
add_subdirectory(DataSources)
add_subdirectory(IK)
add_subdirectory(DynamicalIK)
if (${FRAMEWORK_COMPILE_Wearables_Implementation}) #TODO
    add_subdirectory(WearablesImplementation)
endif()
