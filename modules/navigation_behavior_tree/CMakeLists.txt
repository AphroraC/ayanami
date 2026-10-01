# modules/navigation_behavior_tree -- BehaviorTreeExtension extension module.
# The default navigation tree (XML) and all custom navigation BT node
# registrations live in this .so; the core BehaviorTreePilot is genericized:
# it only loads/holds BehaviorTreeExtension(s).
ayanami_add_module(navigation_behavior_tree
  navigation_behavior_tree.cpp
  module.cpp)
