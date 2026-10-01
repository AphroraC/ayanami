# ayanami-tests.cmake -- test build helpers.
# Included from the top-level CMakeLists.txt; used only by tests/CMakeLists.txt.
#
# Two helpers replace the per-test add_executable/target_link_libraries/add_test
# boilerplate:
#
# ayanami_add_test_plugin(name [DEPS ...] [DEFINES ...] src...)
#   One dlopen fixture .so under tests/modules (PREFIX "lib"); always linked
#   against ayanami.
#
# ayanami_add_test(name SOURCES src... [DEPS tgt...] [NEEDS tgt...]
#                  [DEFINES d...] [ARGS a...] [INCLUDES d...] [TIMEOUT s]
#                  [NO_MAIN])
#   One gtest executable + ctest registration. SOURCES/INCLUDES resolve
#   relative to tests/. libayanami is always linked implicitly (every test
#   consumes it); DEPS are linked on top; NEEDS are gate-only (module .so that
#   the test only receives via $<TARGET_FILE:...> ARGS).
#
#   Module gate: every non-:: DEPS/NEEDS entry must name an existing target
#   ("ayanami" always does). A module disabled via AYANAMI_BUILD_<NAME>=OFF is
#   not a target, so tests that need it skip themselves at configure time --
#   this is what keeps the per-module switches usable without breaking the
#   test suite. ARGS may reference test-plugin targets freely (plugins are
#   always built under BUILD_TESTING).

function(ayanami_add_test_plugin name)
  cmake_parse_arguments(P "" "" "DEPS;DEFINES" ${ARGN})
  add_library(${name} SHARED ${P_UNPARSED_ARGUMENTS})
  target_link_libraries(${name} PRIVATE ayanami ${P_DEPS})
  if(P_DEFINES)
    target_compile_definitions(${name} PRIVATE ${P_DEFINES})
  endif()
  set_target_properties(${name} PROPERTIES PREFIX "lib")
endfunction()

function(ayanami_add_test name)
  cmake_parse_arguments(T "NO_MAIN" "TIMEOUT"
    "SOURCES;DEPS;NEEDS;DEFINES;ARGS;INCLUDES" ${ARGN})

  foreach(dep ${T_DEPS} ${T_NEEDS})
    if(NOT dep STREQUAL "ayanami" AND NOT TARGET ${dep})
      message(STATUS "ayanami: test ${name} skipped "
        "(module target '${dep}' is not built)")
      return()
    endif()
  endforeach()

  add_executable(${name} ${T_SOURCES})
  set(_gtest_libs GTest::gtest)
  if(NOT T_NO_MAIN)
    list(APPEND _gtest_libs GTest::gtest_main)
  endif()
  target_link_libraries(${name} PRIVATE ayanami ${T_DEPS} ${_gtest_libs})
  if(T_INCLUDES)
    target_include_directories(${name} PRIVATE ${T_INCLUDES})
  endif()
  if(T_DEFINES)
    target_compile_definitions(${name} PRIVATE ${T_DEFINES})
  endif()
  add_test(NAME ${name} COMMAND ${name} ${T_ARGS})
  if(T_TIMEOUT)
    set_tests_properties(${name} PROPERTIES TIMEOUT ${T_TIMEOUT})
  endif()
endfunction()
