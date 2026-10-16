# modules/costmap_cleanup_recovery -- map clearing recovery plugin
# (executes ClearCostmapCommand through the sole-writer port).
ayanami_add_module(costmap_cleanup_recovery
  costmap_cleanup_recovery.cpp
  module.cpp)
