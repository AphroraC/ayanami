# modules/rose_navigation_fsm -- the FSM-era navigation host plugin
# (costmap machinery + derived ESDF field + FsmDriver).
ayanami_add_module(rose_navigation_fsm
  rose_navigation_fsm.cpp
  module.cpp)

target_include_directories(rose_navigation_fsm PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
