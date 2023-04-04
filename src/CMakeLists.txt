
add_subdirectory(Logging)
add_subdirectory(DataSources)
if (${FRAMEWORK_COMPILE_Wearables_Implementation}) #TODO
    add_subdirectory(WearablesImplementation)
endif()
