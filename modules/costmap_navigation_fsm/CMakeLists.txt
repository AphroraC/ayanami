# modules/costmap_navigation_fsm -- the FSM-era costmap navigation host
# plugin (full costmap stack + FsmDriver).
ayanami_add_module(costmap_navigation_fsm
  costmap_navigation_fsm.cpp
  module.cpp)

target_include_directories(costmap_navigation_fsm PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
