# ayanami-modules.cmake -- production module (.so plugin) build helper.
# Included from the top-level CMakeLists.txt before the module loop.
#
# ayanami_add_module(name src...) builds one production module as a SHARED
# library linked against libayanami. It must be called from the module's own
# CMakeLists (modules/<name>/CMakeLists.txt): the sources are resolved
# relative to the module directory, and module-private extra dependencies are
# declared in that same file right after the call.
#
# Output naming is load-bearing: the .so lands as libayanami_<name>.so, and
# config/modules/*.json (so_name) + navigation.behavior_tree_extension dlopen
# exactly that name.

function(ayanami_add_module name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE ayanami)
  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/modules
    ${PROJECT_SOURCE_DIR}/modules/${name})
  # Output beside libayanami.so (build root): openSo()'s dladdr fallback
  # resolves bare sonames against the CORE library's directory, so the
  # build-tree modules must deploy next to it, exactly like the install
  # tree deploys them beside it in lib/.
  set_target_properties(${name} PROPERTIES
    PREFIX "libayanami_"
    OUTPUT_NAME ${name}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  install(TARGETS ${name} LIBRARY DESTINATION lib)
endfunction()
