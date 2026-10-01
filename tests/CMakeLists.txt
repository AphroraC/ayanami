# tests/CMakeLists.txt -- all test declarations.
# Boilerplate lives in cmake/ayanami-tests.cmake (ayanami_add_test /
# ayanami_add_test_plugin); per-module switches work because every test that
# needs a module target gates on it and skips itself when the module is off.
#
# Test and target names match the legacy top-level declarations one-to-one.

# ---- dlopen fixture plugins (tests/modules) ----
ayanami_add_test_plugin(dummy_planner_module
  modules/dummy_planner_module.cpp)
ayanami_add_test_plugin(dummy_extension_module
  modules/dummy_extension_module.cpp)
ayanami_add_test_plugin(robot_state_dso_extension
  modules/robot_state_dso_extension.cpp)
ayanami_add_test_plugin(echo_extension
  modules/echo_extension.cpp)

ayanami_add_test_plugin(test_planner_module
  modules/test_planner_module.cpp)
ayanami_add_test_plugin(test_failing_planner_module
  modules/test_failing_planner_module.cpp)
# Interface-version guard: a genuine AbstractPlanner reporting a stale version.
ayanami_add_test_plugin(test_stale_api_planner_module
  modules/test_stale_api_planner_module.cpp)
ayanami_add_test_plugin(test_smoother_module
  modules/test_smoother_module.cpp)
ayanami_add_test_plugin(test_controller_module
  modules/test_controller_module.cpp)
ayanami_add_test_plugin(test_looping_controller_module
  modules/test_looping_controller_module.cpp)
ayanami_add_test_plugin(test_recovery_module
  modules/test_recovery_module.cpp)
# Exception injection: a recovery whose start() throws.
ayanami_add_test_plugin(test_throwing_recovery_module
  modules/test_throwing_recovery_module.cpp)

# Each .so has a single public create_extension_module factory; typing is done
# by ModuleManager dynamic_cast (costmap family / plain abstract family).
foreach(_m test_costmap_planner test_costmap_smoother test_costmap_controller
           test_costmap_recovery
           test_plain_planner test_plain_smoother test_plain_controller
           test_plain_recovery)
  ayanami_add_test_plugin(${_m}_module modules/${_m}_module.cpp
    DEPS costmap_server::costmap_server)
endforeach()

ayanami_add_test_plugin(navigation_test_controller
  modules/navigation_test_controller.cpp
  DEPS costmap_server::costmap_server)

foreach(_role PLANNER SMOOTHER CONTROLLER)
  ayanami_add_test_plugin(goal_update_${_role}
    modules/goal_update_follow_modules.cpp
    DEFINES AYANAMI_GOAL_UPDATE_ROLE_${_role})
endforeach()

# ---- shared paths ----
set(_bt_xml ${PROJECT_SOURCE_DIR}/config/BT/costmap_navigation.xml)
set(_config_dir ${PROJECT_SOURCE_DIR}/config)

# ---- core utility / messages / state ----
ayanami_add_test(config_test SOURCES config_test.cpp)
ayanami_add_test(logging_test SOURCES logging_test.cpp)
ayanami_add_test(callback_slot_test SOURCES callback_slot_test.cpp)
ayanami_add_test(messages_test SOURCES messages_test.cpp)
ayanami_add_test(robot_state_controller_test
  SOURCES robot_state_controller_test.cpp)
# TF-query performance benchmark (10 edges @ 500 Hz; gates: success > 98%,
# mean/p99 latency < 3 ms).
ayanami_add_test(robot_state_controller_bench
  SOURCES robot_state_controller_bench.cpp)
# Cross-DSO singleton: instance() is defined in a libayanami .cpp, so
# dlopen'ed plugins and the main binary share one instance.
ayanami_add_test(robot_state_controller_cross_dso_test
  SOURCES robot_state_controller_cross_dso_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:robot_state_dso_extension>)
ayanami_add_test(load_module_test
  SOURCES load_module_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:dummy_planner_module>
       $<TARGET_FILE:dummy_extension_module>)
# Cross-.so echo plugin: added to a NavigationCallbacks slot after dlopen.
ayanami_add_test(callbacks_test
  SOURCES callbacks_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:echo_extension>)

# ---- abstract components / server ----
ayanami_add_test(core_interfaces_test SOURCES core_interfaces_test.cpp)
ayanami_add_test(execution_test SOURCES execution_test.cpp)
ayanami_add_test(action_test SOURCES action_test.cpp)
ayanami_add_test(module_manager_test
  SOURCES module_manager_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:dummy_planner_module>
       $<TARGET_FILE:dummy_extension_module>
       $<TARGET_FILE:test_stale_api_planner_module>)
# Derived destructors callable through a base pointer (virtual ~ regression).
ayanami_add_test(module_teardown_no_ub_test
  SOURCES module_teardown_no_ub_test.cpp)
ayanami_add_test(recovery_exception_test
  SOURCES recovery_exception_test.cpp
  TIMEOUT 30
  DEFINES AYANAMI_TEST_THROWING_RECOVERY_SO="$<TARGET_FILE:test_throwing_recovery_module>"
          AYANAMI_TEST_PLANNER_SO="$<TARGET_FILE:test_planner_module>")
ayanami_add_test(navigation_server_test
  SOURCES navigation_server_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:test_planner_module>
       $<TARGET_FILE:test_failing_planner_module>
       $<TARGET_FILE:test_smoother_module>
       $<TARGET_FILE:test_controller_module>
       $<TARGET_FILE:test_recovery_module>)

# ---- behavior tree ----
ayanami_add_test(behavior_tree_control_test
  SOURCES behavior_tree_control_test.cpp)
ayanami_add_test(behavior_tree_action_test
  SOURCES behavior_tree_action_test.cpp
  NEEDS navigation_behavior_tree
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}"
          AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
ayanami_add_test(pilot_test
  SOURCES pilot_test.cpp
  NEEDS navigation_behavior_tree
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}"
          AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
ayanami_add_test(recovery_exhaustion_test
  SOURCES recovery_exhaustion_test.cpp
  NEEDS navigation_behavior_tree)
ayanami_add_test(session_state_dispatch_test
  SOURCES session_state_dispatch_test.cpp
  NEEDS navigation_behavior_tree
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}"
          AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
  ARGS $<TARGET_FILE:test_planner_module>
       $<TARGET_FILE:test_smoother_module>
       $<TARGET_FILE:test_controller_module>
       $<TARGET_FILE:test_recovery_module>)
ayanami_add_test(pipeline_sequence_scenarios_test
  SOURCES pipeline_sequence_scenarios_test.cpp
  NEEDS navigation_behavior_tree)
# Goal update during navigation: replan + re-smooth + in-place controller
# update (rviz preemption regression, production default tree).
ayanami_add_test(goal_update_follow_test
  SOURCES goal_update_follow_test.cpp
  TIMEOUT 60
  NEEDS navigation_behavior_tree
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}"
          AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
          AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO="$<TARGET_FILE:goal_update_PLANNER>"
          AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO="$<TARGET_FILE:goal_update_SMOOTHER>"
          AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO="$<TARGET_FILE:goal_update_CONTROLLER>")
ayanami_add_test(production_tree_recovery_test
  SOURCES production_tree_recovery_test.cpp
  NEEDS navigation_behavior_tree
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}")

# ---- packages/* ----
ayanami_add_test(costmap_2d_test
  SOURCES costmap_2d_test.cpp
  DEPS costmap_2d::costmap_2d)
ayanami_add_test(costmap_layers_test
  SOURCES costmap_layers_test.cpp
  DEPS costmap_2d_layers::costmap_2d_layers)
ayanami_add_test(costmap_server_test
  SOURCES costmap_server_test.cpp
  DEPS costmap_server::costmap_server)

# ---- costmap core / components ----
ayanami_add_test(costmap_context_test
  SOURCES costmap_context_test.cpp
  DEPS costmap_server::costmap_server)
ayanami_add_test(map_family_abstraction_test
  SOURCES map_family_abstraction_test.cpp)
ayanami_add_test(costmap_components_test
  SOURCES costmap_components_test.cpp
  DEPS costmap_server::costmap_server)
# Controller per-cycle short lock: no obstacle-feed starvation.
ayanami_add_test(concurrent_controller_obstacle_test
  SOURCES concurrent_controller_obstacle_test.cpp
  DEPS costmap_server::costmap_server
  TIMEOUT 10)
ayanami_add_test(recovery_vs_obstacle_test
  SOURCES recovery_vs_obstacle_test.cpp
  DEPS costmap_cleanup_recovery
       costmap_2d_layers::costmap_2d_layers
       costmap_server::costmap_server
  TIMEOUT 10)
ayanami_add_test(snapshot_stability_test
  SOURCES snapshot_stability_test.cpp
  DEPS costmap_server::costmap_server
  TIMEOUT 10)
ayanami_add_test(planner_writer_concurrency_test
  SOURCES planner_writer_concurrency_test.cpp
  DEPS costmap_server::costmap_server
  TIMEOUT 10)
ayanami_add_test(minor_carry_wirings_test
  SOURCES minor_carry_wirings_test.cpp
  DEPS costmap_astar_planner
       costmap_pid_controller
       costmap_2d_layers::costmap_2d_layers
       costmap_server::costmap_server)

# ---- production modules ----
ayanami_add_test(costmap_navigation_server_test
  SOURCES costmap_navigation_server_test.cpp
  NO_MAIN
  DEPS costmap_astar_planner costmap_server::costmap_server
  DEFINES AYANAMI_TEST_CONFIG_DIR="${_config_dir}"
  ARGS $<TARGET_FILE:test_costmap_planner_module>
       $<TARGET_FILE:test_costmap_smoother_module>
       $<TARGET_FILE:test_costmap_controller_module>
       $<TARGET_FILE:test_costmap_recovery_module>
       $<TARGET_FILE:test_plain_planner_module>
       $<TARGET_FILE:test_plain_smoother_module>
       $<TARGET_FILE:test_plain_controller_module>
       $<TARGET_FILE:test_plain_recovery_module>)
ayanami_add_test(costmap_astar_planner_test
  SOURCES costmap_astar_planner_test.cpp
  DEPS costmap_astar_planner costmap_2d_layers::costmap_2d_layers
  ARGS $<TARGET_FILE:costmap_astar_planner>)
ayanami_add_test(costmap_layer_plugin_test
  SOURCES costmap_layer_plugin_test.cpp
  NO_MAIN
  DEPS costmap_2d_layers::costmap_2d_layers costmap_server::costmap_server
  NEEDS costmap_static_layer costmap_obstacle_layer costmap_inflation_layer
  ARGS $<TARGET_FILE:costmap_static_layer>
       $<TARGET_FILE:costmap_obstacle_layer>
       $<TARGET_FILE:costmap_inflation_layer>)
ayanami_add_test(costmap_minsnap_smoother_test
  SOURCES costmap_minsnap_smoother_test.cpp
  DEPS costmap_minsnap_smoother costmap_2d_layers::costmap_2d_layers
  INCLUDES ${PROJECT_SOURCE_DIR}/modules/costmap_minsnap_smoother
  TIMEOUT 30
  ARGS $<TARGET_FILE:costmap_minsnap_smoother>)
ayanami_add_test(costmap_pid_controller_test
  SOURCES costmap_pid_controller_test.cpp
  DEPS costmap_pid_controller simple_goal_checker
  ARGS $<TARGET_FILE:costmap_pid_controller>)
ayanami_add_test(costmap_mppi_controller_test
  SOURCES costmap_mppi_controller_test.cpp
  DEPS costmap_mppi_controller costmap_2d_layers::costmap_2d_layers
  INCLUDES ${PROJECT_SOURCE_DIR}/modules/costmap_mppi_controller
  TIMEOUT 60
  ARGS $<TARGET_FILE:costmap_mppi_controller>)
ayanami_add_test(costmap_recovery_modules_test
  SOURCES costmap_recovery_modules_test.cpp
  DEPS costmap_spin_recovery
       costmap_back_up_recovery
       costmap_wait_recovery
       costmap_cleanup_recovery
       costmap_server::costmap_server
  DEFINES AYANAMI_MODULES_SRC="${PROJECT_SOURCE_DIR}/modules"
  ARGS $<TARGET_FILE:costmap_spin_recovery>
       $<TARGET_FILE:costmap_back_up_recovery>
       $<TARGET_FILE:costmap_wait_recovery>
       $<TARGET_FILE:costmap_cleanup_recovery>
       "${PROJECT_SOURCE_DIR}/modules")
ayanami_add_test(costmap_navigation_test
  SOURCES costmap_navigation_test.cpp
  DEPS costmap_navigation costmap_server::costmap_server
  NEEDS navigation_behavior_tree
        costmap_astar_planner
        costmap_minsnap_smoother
        costmap_pid_controller
        costmap_spin_recovery
        simple_goal_checker
        simple_progress_checker
  DEFINES AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
  ARGS $<TARGET_FILE:costmap_navigation>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:costmap_pid_controller>
       $<TARGET_FILE:navigation_test_controller>
       $<TARGET_FILE:costmap_spin_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>)
# Headless end-to-end: dlopen host + simulated robot pose/pointcloud +
# cmd_vel side-channel assertions + no crash on destruction.
ayanami_add_test(headless_e2e_test
  SOURCES headless_e2e_test.cpp
  DEPS costmap_navigation costmap_server::costmap_server
  NEEDS navigation_behavior_tree
        costmap_astar_planner
        costmap_minsnap_smoother
        costmap_spin_recovery
        simple_goal_checker
        simple_progress_checker
  DEFINES AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
  ARGS $<TARGET_FILE:costmap_navigation>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:navigation_test_controller>
       $<TARGET_FILE:costmap_spin_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       ${PROJECT_SOURCE_DIR}/tests/assets)
ayanami_add_test(checker_test
  SOURCES checker_test.cpp
  DEPS simple_goal_checker simple_progress_checker
  ARGS $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       $<TARGET_FILE:test_planner_module>
       $<TARGET_FILE:test_smoother_module>
       $<TARGET_FILE:test_looping_controller_module>
       $<TARGET_FILE:test_recovery_module>)
ayanami_add_test(config_consistency_test
  SOURCES config_consistency_test.cpp
  DEPS costmap_navigation costmap_server::costmap_server
  NEEDS navigation_behavior_tree
        costmap_astar_planner
        costmap_minsnap_smoother
        costmap_pid_controller
        costmap_spin_recovery
        costmap_back_up_recovery
        costmap_wait_recovery
        costmap_cleanup_recovery
  DEFINES AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
  ARGS "${_config_dir}"
       $<TARGET_FILE:costmap_navigation>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:costmap_pid_controller>
       $<TARGET_FILE:costmap_spin_recovery>
       $<TARGET_FILE:costmap_back_up_recovery>
       $<TARGET_FILE:costmap_wait_recovery>
       $<TARGET_FILE:costmap_cleanup_recovery>)
ayanami_add_test(extension_module_factory_test
  SOURCES extension_module_factory_test.cpp
  NEEDS navigation_behavior_tree
        costmap_navigation
        costmap_astar_planner
        costmap_minsnap_smoother
        costmap_pid_controller
        costmap_spin_recovery
        costmap_back_up_recovery
        costmap_wait_recovery
        costmap_cleanup_recovery
        simple_goal_checker
        simple_progress_checker
  DEFINES AYANAMI_TEST_BT_XML="${_bt_xml}"
  ARGS $<TARGET_FILE:costmap_navigation>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:costmap_pid_controller>
       $<TARGET_FILE:costmap_spin_recovery>
       $<TARGET_FILE:costmap_back_up_recovery>
       $<TARGET_FILE:costmap_wait_recovery>
       $<TARGET_FILE:costmap_cleanup_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       $<TARGET_FILE:navigation_behavior_tree>)

# ---- pure-protocol tests (no module fixtures) ----
ayanami_add_test(action_protocol_test SOURCES action_protocol_test.cpp)
ayanami_add_test(cancel_responsiveness_test
  SOURCES cancel_responsiveness_test.cpp)
ayanami_add_test(continuous_goal_test SOURCES continuous_goal_test.cpp)
ayanami_add_test(controller_session_test
  SOURCES controller_session_test.cpp)
ayanami_add_test(latest_wins_test SOURCES latest_wins_test.cpp)
ayanami_add_test(local_window_rolling_test
  SOURCES local_window_rolling_test.cpp)

# ---- rosemap family ----
ayanami_add_test(rosemap_context_test
  SOURCES rosemap_context_test.cpp
  DEPS costmap_2d::costmap_2d)
ayanami_add_test(rosemap_astar_planner_test
  SOURCES rosemap_astar_planner_test.cpp
  DEPS rosemap_astar_planner)
ayanami_add_test(rosemap_minco_smoother_test
  SOURCES rosemap_minco_smoother_test.cpp
  DEPS rosemap_minco_smoother)
ayanami_add_test(rosemap_mpc_controller_test
  SOURCES rosemap_mpc_controller_test.cpp
  DEPS rosemap_mpc_controller)
ayanami_add_test(rosemap_recovery_modules_test
  SOURCES rosemap_recovery_modules_test.cpp
  DEPS rosemap_escape_recovery rosemap_wait_recovery)
# rosemap headless E2E: the full production rosemap stack (dlopen host +
# real planner/smoother/MPC/recoveries/BT) against a closed-loop virtual
# chassis; navigation must REACH the goal.
ayanami_add_test(rosemap_e2e_test
  SOURCES rosemap_e2e_test.cpp
  DEPS rosemap_navigation costmap_server::costmap_server
  NEEDS navigation_behavior_tree rosemap_astar_planner rosemap_minco_smoother
        rosemap_mpc_controller rosemap_escape_recovery rosemap_wait_recovery
        simple_goal_checker simple_progress_checker
  ARGS $<TARGET_FILE:rosemap_navigation>
       $<TARGET_FILE:rosemap_astar_planner>
       $<TARGET_FILE:rosemap_minco_smoother>
       $<TARGET_FILE:rosemap_mpc_controller>
       $<TARGET_FILE:rosemap_escape_recovery>
       $<TARGET_FILE:rosemap_wait_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       $<TARGET_FILE:navigation_behavior_tree>)
