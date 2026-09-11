# thirdparty.cmake -- vendored third-party interface libraries.
# Included from the top-level CMakeLists.txt.
#
# Provides:
#   BT::behaviortree_cpp        (real subdirectory build of vendored BT.CPP v4)
#   fast_tf::fast_tf            (header-only interface -> thirdparty/fast_tf)
#   nlohmann_json::nlohmann_json (header-only interface -> thirdparty/nlohmann)
#   ankerl::unordered_dense     (header-only interface -> thirdparty/ankerl)

# ---- BehaviorTree.CPP v4 ----
# P2 / spec §4.2: -Wl,-Bsymbolic makes libbehaviortree_cpp.so bind its internal
# references to the vendored tinyxml2 to its own copy, avoiding symbol clashes
# with the system libtinyxml2 (which crashed registerBehaviorTreeFromFile()).
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(USE_VENDORED_TINYXML2 ON CACHE BOOL "" FORCE)
set(_btcpp_old_build_testing ${BUILD_TESTING})
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/BehaviorTree.CPP)
set(BUILD_TESTING ${_btcpp_old_build_testing} CACHE BOOL "" FORCE)
unset(_btcpp_old_build_testing)
target_link_options(behaviortree_cpp PRIVATE "-Wl,-Bsymbolic")


# ---- nlohmann_json (header-only) ----
# Vendored tree keeps `#include <nlohmann/...>` prefixes internally, so the
# include root must be thirdparty/ (not thirdparty/nlohmann). With the wrong
# dir, <nlohmann/json.hpp> silently falls back to the system copy (3.7.3,
# incompatible with std::any under this toolchain).
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:include>)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
install(TARGETS nlohmann_json EXPORT ayanamiTargets)

# ---- ankerl::unordered_dense (header-only) ----
add_library(unordered_dense INTERFACE)
target_include_directories(unordered_dense INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/thirdparty/ankerl>
  $<INSTALL_INTERFACE:include>)
add_library(ankerl::unordered_dense ALIAS unordered_dense)
install(TARGETS unordered_dense EXPORT ayanamiTargets)
