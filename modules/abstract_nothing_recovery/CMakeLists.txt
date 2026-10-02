# modules/abstract_nothing_recovery -- no-op recovery placeholder plugin
# (test phase: the FSM recovery orchestration is exercised with no action).
ayanami_add_module(abstract_nothing_recovery
  abstract_nothing_recovery.cpp
  module.cpp)
