# modules/costmap_cleanup_recovery -- map clearing recovery plugin
# (submits a typed ClearCostmapCommand through the map command port).
ayanami_add_module(costmap_cleanup_recovery
  costmap_cleanup_recovery.cpp
  module.cpp)
