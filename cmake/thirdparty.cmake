# thirdparty.cmake -- vendored third-party interface libraries.
# Included from the top-level CMakeLists.txt.
#
# Provides:
#   BT::behaviortree_cpp         (real subdirectory build of vendored BT.CPP v4)
#   nlohmann_json::nlohmann_json (header-only interface -> thirdparty/nlohmann)
#
# Header-only vendored trees are installed alongside the ayanami headers so
# downstream consumers (ayanami_ros) resolve <nlohmann/...> from the install
# space instead of reaching back into the ayanami source tree.

# ---- BehaviorTree.CPP v4 (PRIVATE dependency, statically embedded) ----
# ayanami's BT is not a public artifact: building it STATIC and linking it
# PRIVATELY into libayanami keeps libbehaviortree_cpp out of the install
# space entirely, where a second BehaviorTree.CPP build (same soname) would
# collide. Build-tree consumers (modules/, tests/) still compile BT headers
# through the BUILD_INTERFACE include path ayanami propagates, and their
# external BT symbol references resolve against libayanami.so, which exports
# the embedded archive's symbols (weak/vague-linkage definitions still merge
# across DSOs as usual -- do NOT add -Bsymbolic; see top-level CMakeLists).
# BTCPP_SQLITE_LOGGING=OFF: nothing in ayanami uses the BT file loggers, and
# it keeps sqlite3 out of libayanami's dependency surface.
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_INSTALL OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(USE_VENDORED_TINYXML2 ON CACHE BOOL "" FORCE)
set(_btcpp_old_build_testing ${BUILD_TESTING})
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/BehaviorTree.CPP)
set(BUILD_TESTING ${_btcpp_old_build_testing} CACHE BOOL "" FORCE)
unset(_btcpp_old_build_testing)


# ---- nlohmann_json (header-only) ----
# Vendored tree keeps `#include <nlohmann/...>` prefixes internally, so the
# include root must be thirdparty/ (not thirdparty/nlohmann). With the wrong
# dir, <nlohmann/json.hpp> silently falls back to the system copy (3.7.3,
# incompatible with std::any under this toolchain).
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:include>)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
install(TARGETS nlohmann_json EXPORT ayanamiTargets)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/nlohmann/
  DESTINATION include/nlohmann)
