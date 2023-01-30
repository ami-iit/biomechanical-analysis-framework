
include(CMakeDependentOption)

macro(FRAMEWORK_DEPENDENT_OPTION _option _doc _default _deps _force)

  if(DEFINED ${_option})
    get_property(_option_strings_set CACHE ${_option} PROPERTY STRINGS SET)
    if(_option_strings_set)
      # If the user thinks he is smarter than the machine, he deserves an error
      get_property(_option_strings CACHE ${_option} PROPERTY STRINGS)
      list(GET _option_strings 0 _option_strings_first)
      string(REGEX REPLACE ".+\"(.+)\".+" "\\1" _option_strings_first "${_option_strings_first}")
      list(LENGTH _option_strings _option_strings_length)
      math(EXPR _option_strings_last_index "${_option_strings_length} - 1")
      list(GET _option_strings ${_option_strings_last_index} _option_strings_last)
      if("${${_option}}" STREQUAL "${_option_strings_last}")
        message(SEND_ERROR "That was a trick, you cannot outsmart me! I will never let you win! ${_option} stays OFF until I say so! \"${_option_strings_first}\" is needed to enable ${_option}. Now stop bothering me, and install your dependencies, if you really want to enable this option.")
      endif()
      unset(${_option} CACHE)
    endif()
  endif()

  cmake_dependent_option(${_option} "${_doc}" ${_default} "${_deps}" ${_force})

  unset(_missing_deps)
  foreach(_dep ${_deps})
    string(REGEX REPLACE " +" ";" _depx "${_dep}")
    if(NOT (${_depx}))
      list(APPEND _missing_deps "${_dep}")
    endif()
  endforeach()

  if(DEFINED _missing_deps)
    set(${_option}_disable_reason " (dependencies unsatisfied: \"${_missing_deps}\")")
    # Set a value that can be visualized on ccmake and on cmake-gui, but
    # still evaluates to false
    set(${_option} "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND" CACHE STRING "${_option_doc}" FORCE)
    string(REPLACE ";" "\;" _missing_deps "${_missing_deps}")
    set_property(CACHE ${_option}
                PROPERTY STRINGS "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND"
                                 "OFF - You can try as much as you want, but '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Are you crazy or what? '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Didn't I already tell you that '${_missing_deps}' is needed to enable ${_option}? - ${_option}-NOTFOUND"
                                 "OFF - Stop it! - ${_option}-NOTFOUND"
                                 "OFF - This is insane! Leave me alone! - ${_option}-NOTFOUND"
                                 "ON - All right, you win. The option is enabled. Are you happy now? You just broke the build.")
    # Set non-cache variable that will override the value in current scope
    # For parent scopes, the "-NOTFOUND ensures that the variable still
    # evaluates to false
    set(${_option} ${_force})
  endif()

endmacro()
