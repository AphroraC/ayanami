# modules/rosemap_minco_smoother -- MINCO minimum-snap smoother plugin
# (rosemap family). The vendored minco/trajectory/lbfgs headers are
# header-only and need no extra linkage.
ayanami_add_module(rosemap_minco_smoother
  rosemap_minco_smoother.cpp
  module.cpp)

target_include_directories(rosemap_minco_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
