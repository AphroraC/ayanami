# thirdparty.cmake -- vendored third-party interface libraries.
# Included from the top-level CMakeLists.txt.
#
# Provides:
#   nlohmann_json::nlohmann_json (header-only interface -> thirdparty/nlohmann)
#
# Header-only vendored trees are installed alongside the ayanami headers so
# downstream consumers (ayanami_ros) resolve <nlohmann/...> from the install
# space instead of reaching back into the ayanami source tree.

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
