
function(add_baf_test)

    if(FRAMEWORK_COMPILE_tests)

      set(options )
      set(oneValueArgs NAME)
      set(multiValueArgs SOURCES LINKS)

      set(prefix "baf")

      cmake_parse_arguments(${prefix}
          "${options}"
          "${oneValueArgs}"
          "${multiValueArgs}"
          ${ARGN})

      set(name ${${prefix}_NAME})
      set(unit_test_files ${${prefix}_SOURCES})

      set(targetname ${name}UnitTests)
      add_executable(${targetname}
          "${unit_test_files}")

      target_link_libraries(${targetname} PRIVATE Catch2::Catch2WithMain ${${prefix}_LINKS})
      target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
      target_compile_features(${targetname} PUBLIC cxx_std_17)
      target_compile_definitions(${targetname} PRIVATE -D_USE_MATH_DEFINES)

      add_test(NAME ${targetname} COMMAND ${targetname})

    endif()

endfunction()
