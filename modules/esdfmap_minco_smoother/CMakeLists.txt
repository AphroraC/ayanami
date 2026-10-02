# modules/esdfmap_minco_smoother -- MINCO minimum-snap smoother plugin
# (esdfmap family). The vendored minco/trajectory/lbfgs headers are
# header-only and need no extra linkage.
ayanami_add_module(esdfmap_minco_smoother
  esdfmap_minco_smoother.cpp
  module.cpp)

target_include_directories(esdfmap_minco_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
