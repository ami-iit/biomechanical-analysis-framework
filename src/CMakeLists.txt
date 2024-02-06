
add_subdirectory(IK)
add_subdirectory(Logging)
add_subdirectory(Conversions)

if(FRAMEWORK_COMPILE_examples)
    add_subdirectory(examples)
endif()
