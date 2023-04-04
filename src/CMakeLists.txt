
add_subdirectory(Logging)
add_subdirectory(DataSources)
add_subdirectory(IK)
if (${FRAMEWORK_COMPILE_Wearables_Implementation}) #TODO
    add_subdirectory(WearablesImplementation)
endif()
