cmake_minimum_required(VERSION 3.16)
project(ayanami VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

# libstdc++'s shared_ptr atomic free functions only take the lock-free path when cmpxchg16b is enabled
# (-mcx16); otherwise they fall back to a per-bucket spinlock implementation. All TUs compiling
# CallbackSlot/CostmapWrapper headers (lib, module plugins, tests, downstream ayanami_ros) must use the same policy,
# otherwise mixing lock-free readers and lock-pool writers on the same shared_ptr object in one process is undefined behavior.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" AYANAMI_HAS_MCX16)
if(AYANAMI_HAS_MCX16)
  add_compile_options(-mcx16)
else()
  message(WARNING "ayanami: -mcx16 不受支持; shared_ptr 原子操作退回 libstdc++ 锁池实现")
endif()

option(BUILD_SHARED "Build libayanami as shared library" ON)
option(BUILD_TESTING "Build tests" ON)
option(BUILD_WITH_BT "Build behavior tree component" ON)

if(BUILD_SHARED)
  set(AYANAMI_BUILD_SHARED_BOOL SHARED)
else()
  set(AYANAMI_BUILD_SHARED_BOOL STATIC)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)  # or vendored

# M2 Controller Ruling #M2-1: catkin as the build system (no dependency on the ROS communication layer);
# only catkin_package() is declared so the catkin workspace recognizes ayanami as a catkin package
# and generates devel/share/ayanami/cmake/ayanamiConfig.cmake so downstream ayanami_ros
# can resolve find_package(ayanami COMPONENTS) (catkin component resolution path).
# The M1 pure-CMake install export (lib/cmake/ayanami/) coexists with this catkin_package
# without breaking external (non-catkin) find_package(ayanami CONFIG).
find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ayanami
    CFG_EXTRAS ""   # no catkin auto macros attached
  )
endif()

# Third-party interface libraries (introduced via the new cmake/thirdparty.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/thirdparty.cmake)  # defines BT::behaviortree_cpp / fast_tf::fast_tf / nlohmann_json / ankerl

# Task 14: BT-dependent sources (excluded when BUILD_WITH_BT=OFF to avoid compile failures from missing BT headers)
set(AYANAMI_BT_SOURCES "")
if(BUILD_WITH_BT)
  list(APPEND AYANAMI_BT_SOURCES src/behavior_tree/pilot.cpp)
endif()

# Task 15: packages/* pure C++ sub-libraries
add_subdirectory(packages/costmap_2d)
add_subdirectory(packages/costmap_layers)
add_subdirectory(packages/costmap_server)

# ayanami_add_module(name src...) macro: builds a SHARED module .so linked against libayanami
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL}
  src/utility/config.cpp
  src/utility/logging.cpp
  src/utility/load_module.cpp
  src/utility/extension_module.cpp
  src/interfaces/callbacks.cpp
  src/interfaces/transform_store.cpp
  src/abstract/components/execution_base.cpp
  src/abstract/components/plugin_manager.cpp
  src/components/navigation_server_base.cpp
  src/components/robot_state_controller.cpp
  src/costmap/components/costmap_navigation_server.cpp
  ${AYANAMI_BT_SOURCES}
  thirdparty/fast_tf/fast_tf.cpp)
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/modules>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
if(AYANAMI_HAS_MCX16)
  target_compile_options(ayanami PUBLIC -mcx16)
endif()
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
# Task 20: CostmapNavigationServer public headers include costmap_server/costmap_wrapper.hpp
target_link_libraries(ayanami PUBLIC costmap_server::costmap_server)
# src/utility/load_module.cpp uses dlfcn (dlopen/dlsym)
target_link_libraries(ayanami PRIVATE ${CMAKE_DL_LIBS})
# src/utility/config.cpp (via config_impl.hpp) depends on the vendored nlohmann;
# the public header config.hpp does not include nlohmann (spec §2.1), so PRIVATE is enough.
target_link_libraries(ayanami PRIVATE nlohmann_json::nlohmann_json)
if(BUILD_WITH_BT)
  target_link_libraries(ayanami PUBLIC BT::behaviortree_cpp fast_tf::fast_tf)
endif()
install(TARGETS ayanami EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY config config_sentry DESTINATION share/ayanami
  PATTERN "*.yaml" EXCLUDE PATTERN "*.yml" EXCLUDE)

# Module macro
function(ayanami_add_module name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE ayanami)
  target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/modules/${name})
  set_target_properties(${name} PROPERTIES PREFIX "libayanami_" OUTPUT_NAME ${name})
  install(TARGETS ${name} LIBRARY DESTINATION lib)
  list(APPEND AYANAMI_INSTALL_TARGETS ${name})
  set(AYANAMI_INSTALL_TARGETS ${AYANAMI_INSTALL_TARGETS} PARENT_SCOPE)
endfunction()

# Task 21: modules/costmap_smac_planner (A* global path planner plugin)
ayanami_add_module(costmap_smac_planner
  modules/costmap_smac_planner/costmap_smac_planner.cpp
  modules/costmap_smac_planner/module.cpp)

# Task 22: modules/costmap_savitzky_smoother (7-point SG path smoother plugin)
ayanami_add_module(costmap_savitzky_smoother
  modules/costmap_savitzky_smoother/costmap_savitzky_smoother.cpp
  modules/costmap_savitzky_smoother/module.cpp)

# Task 23: modules/costmap_pid_controller (3-axis PID local controller plugin)
ayanami_add_module(costmap_pid_controller
  modules/costmap_pid_controller/costmap_pid_controller.cpp
  modules/costmap_pid_controller/module.cpp)

# modules/costmap_dwb_controller (DWB local controller, ported from nav2_dwb_controller;
# zero ROS: pluginlib -> static registry / tf2 -> TransformStore / rclcpp -> Config)
ayanami_add_module(costmap_dwb_controller
  modules/costmap_dwb_controller/costmap_dwb_controller.cpp
  modules/costmap_dwb_controller/module.cpp
  modules/costmap_dwb_controller/exceptions.cpp
  modules/costmap_dwb_controller/trajectory_utils.cpp
  modules/costmap_dwb_controller/path_ops.cpp
  modules/costmap_dwb_controller/registration.cpp
  modules/costmap_dwb_controller/costmap_queue/costmap_queue.cpp
  modules/costmap_dwb_controller/costmap_queue/limited_costmap_queue.cpp
  modules/costmap_dwb_controller/plugins/kinematic_parameters.cpp
  modules/costmap_dwb_controller/plugins/xy_theta_iterator.cpp
  modules/costmap_dwb_controller/plugins/standard_traj_generator.cpp
  modules/costmap_dwb_controller/plugins/limited_accel_generator.cpp
  modules/costmap_dwb_controller/critics/alignment_util.cpp
  modules/costmap_dwb_controller/critics/map_grid.cpp
  modules/costmap_dwb_controller/critics/base_obstacle.cpp
  modules/costmap_dwb_controller/critics/obstacle_footprint.cpp
  modules/costmap_dwb_controller/critics/goal_dist.cpp
  modules/costmap_dwb_controller/critics/path_dist.cpp
  modules/costmap_dwb_controller/critics/goal_align.cpp
  modules/costmap_dwb_controller/critics/path_align.cpp
  modules/costmap_dwb_controller/critics/oscillation.cpp
  modules/costmap_dwb_controller/critics/prefer_forward.cpp
  modules/costmap_dwb_controller/critics/rotate_to_goal.cpp
  modules/costmap_dwb_controller/critics/twirling.cpp)

# Task 24: modules/costmap_*_recovery (4 recovery plugins, named per spec §6.6)
ayanami_add_module(costmap_spin_recovery
  modules/costmap_spin_recovery/costmap_spin_recovery.cpp
  modules/costmap_spin_recovery/module.cpp)
ayanami_add_module(costmap_back_up_recovery
  modules/costmap_back_up_recovery/costmap_back_up_recovery.cpp
  modules/costmap_back_up_recovery/module.cpp)
ayanami_add_module(costmap_wait_recovery
  modules/costmap_wait_recovery/costmap_wait_recovery.cpp
  modules/costmap_wait_recovery/module.cpp)
ayanami_add_module(costmap_clear_costmap_recovery
  modules/costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.cpp
  modules/costmap_clear_costmap_recovery/module.cpp)

# Task 25: modules/costmap_navigation (AbstractNavigation host plugin)
ayanami_add_module(costmap_navigation
  modules/costmap_navigation/costmap_navigation.cpp
  modules/costmap_navigation/module.cpp)

# Task 26+: modules/simple_goal_checker / simple_progress_checker
# (goal check + progress check plugins, abstract family, no map dependency)
ayanami_add_module(simple_goal_checker
  modules/simple_goal_checker/simple_goal_checker.cpp
  modules/simple_goal_checker/module.cpp)
ayanami_add_module(simple_progress_checker
  modules/simple_progress_checker/simple_progress_checker.cpp
  modules/simple_progress_checker/module.cpp)

# Requirement 3: modules/navigation_behavior_tree (BehaviorTreeExtension extension).
# The default navigation tree (XML) and all custom navigation BT node registrations live in this .so; the core
# BehaviorTreePilot is genericized: it only loads/holds BehaviorTreeExtension(s).
ayanami_add_module(navigation_behavior_tree
  modules/navigation_behavior_tree/navigation_behavior_tree.cpp
  modules/navigation_behavior_tree/module.cpp)

# Package export
install(EXPORT ayanamiTargets NAMESPACE ayanami:: FILE ayanamiTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/ayanami-config.cmake.in
  ${CMAKE_BINARY_DIR}/ayanami-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
write_basic_package_version_file(ayanami-configVersion.cmake COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_BINARY_DIR}/ayanami-config.cmake ${CMAKE_BINARY_DIR}/ayanami-configVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)

# ---- Tests ----
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest)
  if(GTest_FOUND)
    add_executable(config_test tests/config_test.cpp)
    target_link_libraries(config_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ConfigTest COMMAND config_test)

    add_executable(logging_test tests/logging_test.cpp)
    target_link_libraries(logging_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME LoggingTest COMMAND logging_test)

    add_executable(callback_slot_test tests/callback_slot_test.cpp)
    target_link_libraries(callback_slot_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME CallbackSlotTest COMMAND callback_slot_test)

    # load_module / ExtensionModule test plugins (.so loaded via dlopen)
    add_library(dummy_planner_module SHARED tests/plugins/dummy_planner_module.cpp)
    target_link_libraries(dummy_planner_module PRIVATE ayanami)
    set_target_properties(dummy_planner_module PROPERTIES PREFIX "lib")
    add_library(dummy_extension_module SHARED tests/plugins/dummy_extension_module.cpp)
    target_link_libraries(dummy_extension_module PRIVATE ayanami)
    set_target_properties(dummy_extension_module PROPERTIES PREFIX "lib")

    add_executable(messages_test tests/messages_test.cpp)
    target_link_libraries(messages_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME MessagesTest COMMAND messages_test)

    add_executable(transform_store_test tests/transform_store_test.cpp)
    target_link_libraries(transform_store_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME TransformStoreTest COMMAND transform_store_test)

    add_executable(robot_state_controller_test tests/robot_state_controller_test.cpp)
    target_link_libraries(robot_state_controller_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME RobotStateControllerTest COMMAND robot_state_controller_test)

    # RobotStateController cross-DSO singleton: instance() is defined in a libayanami .cpp,
    # so dlopen'ed plugins (linked against libayanami) and the main binary resolve to the same instance and share state.
    add_library(robot_state_dso_extension SHARED
      tests/plugins/robot_state_dso_extension.cpp)
    target_link_libraries(robot_state_dso_extension PRIVATE ayanami)
    set_target_properties(robot_state_dso_extension PROPERTIES PREFIX "lib")

    add_executable(robot_state_controller_cross_dso_test
      tests/robot_state_controller_cross_dso_test.cpp)
    target_link_libraries(robot_state_controller_cross_dso_test PRIVATE
      ayanami GTest::gtest)
    add_test(NAME RobotStateControllerCrossDSO COMMAND
      robot_state_controller_cross_dso_test
      $<TARGET_FILE:robot_state_dso_extension>)

    add_executable(load_module_test tests/load_module_test.cpp)
    target_link_libraries(load_module_test PRIVATE ayanami GTest::gtest)
    add_test(NAME LoadModuleTest COMMAND load_module_test
      $<TARGET_FILE:dummy_planner_module> $<TARGET_FILE:dummy_extension_module>)

    # NavigationCallbacks cross-.so echo test plugin (added to the slot after dlopen, linked against libayanami)
    add_library(echo_extension SHARED tests/plugins/echo_extension.cpp)
    target_link_libraries(echo_extension PRIVATE ayanami)
    set_target_properties(echo_extension PROPERTIES PREFIX "lib")

    add_executable(callbacks_test tests/callbacks_test.cpp)
    target_link_libraries(callbacks_test PRIVATE ayanami GTest::gtest)
    add_test(NAME CallbacksTest COMMAND callbacks_test
      $<TARGET_FILE:echo_extension>)

    add_executable(core_interfaces_test tests/core_interfaces_test.cpp)
    target_link_libraries(core_interfaces_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME CoreInterfacesTest COMMAND core_interfaces_test)

    add_executable(execution_test tests/execution_test.cpp)
    target_link_libraries(execution_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ExecutionTest COMMAND execution_test)

    add_executable(action_test tests/action_test.cpp)
    target_link_libraries(action_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ActionTest COMMAND action_test)

    add_executable(plugin_manager_test tests/plugin_manager_test.cpp)
    target_link_libraries(plugin_manager_test PRIVATE ayanami GTest::gtest)
    add_test(NAME PluginManagerTest COMMAND plugin_manager_test
      $<TARGET_FILE:dummy_planner_module> $<TARGET_FILE:dummy_extension_module>)

    # Task 0: BLOCK #2 derived destructors callable through a base pointer (UB from missing virtual ~ in abstract base)
    add_executable(plugin_teardown_no_ub_test tests/plugin_teardown_no_ub_test.cpp)
    target_link_libraries(plugin_teardown_no_ub_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME PluginTeardownNoUB COMMAND plugin_teardown_no_ub_test)

    # Task 11: NavigationServerBase (the 4 plugin-family factories live in test plugin .so files; each .so has a single
    # public create_extension_module factory; typing is done by PluginManager dynamic_cast).
    add_library(test_planner_plugin SHARED tests/plugins/test_planner_plugin.cpp)
    target_link_libraries(test_planner_plugin PRIVATE ayanami)
    set_target_properties(test_planner_plugin PROPERTIES PREFIX "lib")
    add_library(test_failing_planner_plugin SHARED tests/plugins/test_failing_planner_plugin.cpp)
    target_link_libraries(test_failing_planner_plugin PRIVATE ayanami)
    set_target_properties(test_failing_planner_plugin PROPERTIES PREFIX "lib")
    add_library(test_smoother_plugin SHARED tests/plugins/test_smoother_plugin.cpp)
    target_link_libraries(test_smoother_plugin PRIVATE ayanami)
    set_target_properties(test_smoother_plugin PROPERTIES PREFIX "lib")
    add_library(test_controller_plugin SHARED tests/plugins/test_controller_plugin.cpp)
    target_link_libraries(test_controller_plugin PRIVATE ayanami)
    set_target_properties(test_controller_plugin PROPERTIES PREFIX "lib")
    add_library(test_looping_controller_plugin SHARED tests/plugins/test_looping_controller_plugin.cpp)
    target_link_libraries(test_looping_controller_plugin PRIVATE ayanami)
    set_target_properties(test_looping_controller_plugin PROPERTIES PREFIX "lib")
    add_library(test_recovery_plugin SHARED tests/plugins/test_recovery_plugin.cpp)
    target_link_libraries(test_recovery_plugin PRIVATE ayanami)
    set_target_properties(test_recovery_plugin PROPERTIES PREFIX "lib")
    # Exception injection: a recovery whose start() throws (regression closing the exception path).
    add_library(test_throwing_recovery_plugin SHARED
      tests/plugins/test_throwing_recovery_plugin.cpp)
    target_link_libraries(test_throwing_recovery_plugin PRIVATE ayanami)
    set_target_properties(test_throwing_recovery_plugin PROPERTIES PREFIX "lib")

    add_executable(recovery_exception_test tests/recovery_exception_test.cpp)
    target_link_libraries(recovery_exception_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    target_compile_definitions(recovery_exception_test PRIVATE
      AYANAMI_TEST_THROWING_RECOVERY_SO="$<TARGET_FILE:test_throwing_recovery_plugin>"
      AYANAMI_TEST_PLANNER_SO="$<TARGET_FILE:test_planner_plugin>")
    add_test(NAME RecoveryException COMMAND recovery_exception_test)
    set_tests_properties(RecoveryException PROPERTIES TIMEOUT 30)

    add_executable(navigation_server_test tests/navigation_server_test.cpp)
    target_link_libraries(navigation_server_test PRIVATE ayanami GTest::gtest)
    add_test(NAME NavigationServerTest COMMAND navigation_server_test
      $<TARGET_FILE:test_planner_plugin>
      $<TARGET_FILE:test_failing_planner_plugin>
      $<TARGET_FILE:test_smoother_plugin>
      $<TARGET_FILE:test_controller_plugin>
      $<TARGET_FILE:test_recovery_plugin>)

    # Task 12: BT control nodes + condition nodes + blackboard protocol
    add_executable(behavior_tree_control_test tests/behavior_tree_control_test.cpp)
    target_link_libraries(behavior_tree_control_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME BehaviorTreeControlTest COMMAND behavior_tree_control_test)

    # Task 13: BT action nodes (wired directly to TaskHandle) + registration + extension default XML
    add_executable(behavior_tree_action_test tests/behavior_tree_action_test.cpp)
    target_link_libraries(behavior_tree_action_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    target_compile_definitions(behavior_tree_action_test PRIVATE
      AYANAMI_TEST_BT_XML="${CMAKE_SOURCE_DIR}/tests/assets/navigation.xml"
      AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
    add_test(NAME BehaviorTreeActionTest COMMAND behavior_tree_action_test)

    # Task 14: BT Pilot (resident tree + pending_goal blackboard + zero velocity on preemption)
    if(BUILD_WITH_BT)
      add_executable(pilot_test tests/pilot_test.cpp)
      target_link_libraries(pilot_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
      target_compile_definitions(pilot_test PRIVATE
        AYANAMI_TEST_BT_XML="${CMAKE_SOURCE_DIR}/tests/assets/navigation.xml"
        AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
      add_test(NAME PilotTest COMMAND pilot_test)
    endif()

    # Task 15: packages/costmap_2d (pure C++ header-only sub-library)
    add_executable(costmap_2d_test tests/costmap_2d_test.cpp)
    target_link_libraries(costmap_2d_test PRIVATE costmap_2d::costmap_2d GTest::gtest GTest::gtest_main)
    add_test(NAME Costmap2DTest COMMAND costmap_2d_test)

    # Task 16: packages/costmap_layers (Static/Inflation/SimpleObstacle layer)
    add_executable(costmap_layers_test tests/costmap_layers_test.cpp)
    target_link_libraries(costmap_layers_test PRIVATE
      costmap_2d_layers::costmap_2d_layers GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapLayersTest COMMAND costmap_layers_test)

    # Task 17: packages/costmap_server (CostmapWrapper lifecycle)
    add_executable(costmap_server_test tests/costmap_server_test.cpp)
    target_link_libraries(costmap_server_test PRIVATE
      costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapServerTest COMMAND costmap_server_test)

    # Task 18: costmap/core + CostmapContext (AbstractMapContext override)
    add_executable(costmap_context_test tests/costmap_context_test.cpp)
    target_link_libraries(costmap_context_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapContextTest COMMAND costmap_context_test)

    # Map Family abstraction: non-costmap map families are assembled through the same virtual hooks (zero costmap dependency in the core)
    add_executable(map_family_abstraction_test tests/map_family_abstraction_test.cpp)
    target_link_libraries(map_family_abstraction_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME MapFamilyAbstractionTest COMMAND map_family_abstraction_test)

    # Task 19: costmap/components (Costmap*Execution + Costmap*Action)
    add_executable(costmap_components_test tests/costmap_components_test.cpp)
    target_link_libraries(costmap_components_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapComponentsTest COMMAND costmap_components_test)

    # Task 1 (M2 Phase A BLOCK #1): controller held the lock for the whole cycle -> per-cycle short lock,
    # preventing obstacle feed starvation (regression: concurrent controller + obstacle callback starvation).
    add_executable(concurrent_controller_obstacle_test
      tests/concurrent_controller_obstacle_test.cpp)
    target_link_libraries(concurrent_controller_obstacle_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME ConcurrentControllerObstacle COMMAND concurrent_controller_obstacle_test)
    set_tests_properties(ConcurrentControllerObstacle PROPERTIES TIMEOUT 10)

    # Task 2 (M2 Phase A BLOCK #3): recovery locks global+local (scoped_lock) to prevent
    # a concurrent race between clear_costmap writing the local master grid and handleObstaclePointCloud.
    add_executable(recovery_vs_obstacle_test tests/recovery_vs_obstacle_test.cpp)
    target_link_libraries(recovery_vs_obstacle_test PRIVATE
      ayanami costmap_clear_costmap_recovery
      costmap_2d_layers::costmap_2d_layers costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    add_test(NAME RecoveryVsObstacle COMMAND recovery_vs_obstacle_test)
    set_tests_properties(RecoveryVsObstacle PROPERTIES TIMEOUT 10)

    # Redesign: smoother holds a stable snapshot vs concurrent obstacle writers (immutable epoch).
    add_executable(snapshot_stability_test tests/snapshot_stability_test.cpp)
    target_link_libraries(snapshot_stability_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME SnapshotStability COMMAND snapshot_stability_test)
    set_tests_properties(SnapshotStability PROPERTIES TIMEOUT 10)

    # Redesign: a long-running planner does not block obstacle writers (snapshot replaces holding the lock for the whole plan).
    add_executable(planner_writer_concurrency_test
      tests/planner_writer_concurrency_test.cpp)
    target_link_libraries(planner_writer_concurrency_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME PlannerWriterConcurrency COMMAND planner_writer_concurrency_test)
    set_tests_properties(PlannerWriterConcurrency PROPERTIES TIMEOUT 10)

    # Task 3 (M2 Phase A wrap-up): minor carry wiring, 5 items combined
    # (smac start fallback / single source for robot_frame / guard against double configureCoreMaps /
    #  pid coarse signature / cmake find_dependency).
    add_executable(minor_carry_wirings_test tests/minor_carry_wirings_test.cpp)
    target_link_libraries(minor_carry_wirings_test PRIVATE
      ayanami costmap_smac_planner costmap_pid_controller
      costmap_2d_layers::costmap_2d_layers costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    add_test(NAME MinorCarryWiring COMMAND minor_carry_wirings_test)

    # Task 20: costmap/components (CostmapNavigationServer virtual map injection).
    # Each test plugin .so has a single public create_extension_module factory.
    foreach(plugin test_costmap_planner test_costmap_smoother
                 test_costmap_controller test_costmap_recovery
                 test_plain_planner test_plain_smoother
                 test_plain_controller test_plain_recovery)
      add_library(${plugin}_plugin SHARED tests/plugins/${plugin}_plugin.cpp)
      target_link_libraries(${plugin}_plugin PRIVATE
        ayanami costmap_server::costmap_server)
      set_target_properties(${plugin}_plugin PROPERTIES PREFIX "lib")
    endforeach()

    add_executable(costmap_navigation_server_test
      tests/costmap_navigation_server_test.cpp)
    target_link_libraries(costmap_navigation_server_test PRIVATE
      ayanami costmap_smac_planner costmap_server::costmap_server GTest::gtest)
    target_compile_definitions(costmap_navigation_server_test PRIVATE
      AYANAMI_TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    add_test(NAME CostmapNavigationServerTest COMMAND costmap_navigation_server_test
      $<TARGET_FILE:test_costmap_planner_plugin>
      $<TARGET_FILE:test_costmap_smoother_plugin>
      $<TARGET_FILE:test_costmap_controller_plugin>
      $<TARGET_FILE:test_costmap_recovery_plugin>
      $<TARGET_FILE:test_plain_planner_plugin>
      $<TARGET_FILE:test_plain_smoother_plugin>
      $<TARGET_FILE:test_plain_controller_plugin>
      $<TARGET_FILE:test_plain_recovery_plugin>)

    # modules/costmap_dwb_controller (DWB, ported from nav2_dwb_controller)
    add_executable(costmap_dwb_controller_test
      tests/costmap_dwb_controller_test.cpp)
    target_link_libraries(costmap_dwb_controller_test PRIVATE
      ayanami costmap_dwb_controller costmap_server::costmap_server
      costmap_2d::costmap_2d GTest::gtest GTest::gtest_main)
    target_include_directories(costmap_dwb_controller_test PRIVATE
      ${CMAKE_SOURCE_DIR}/modules/costmap_dwb_controller)
    add_test(NAME CostmapDwBControllerTest COMMAND costmap_dwb_controller_test
      ${CMAKE_SOURCE_DIR}/config/modules/costmap_dwb_controller.json)

# Task 21: modules/costmap_smac_planner (A* global path planner plugin)
    add_executable(costmap_smac_planner_test tests/costmap_smac_planner_test.cpp)
    target_link_libraries(costmap_smac_planner_test PRIVATE
      ayanami costmap_smac_planner costmap_2d_layers::costmap_2d_layers
      GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapSmacPlannerTest COMMAND costmap_smac_planner_test
      $<TARGET_FILE:costmap_smac_planner>)

# Task 22: modules/costmap_savitzky_smoother (7-point SG path smoother plugin)
    add_executable(costmap_savitzky_smoother_test
      tests/costmap_savitzky_smoother_test.cpp)
    target_link_libraries(costmap_savitzky_smoother_test PRIVATE
      ayanami costmap_savitzky_smoother costmap_2d_layers::costmap_2d_layers
      GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapSavitzkySmootherTest COMMAND
      costmap_savitzky_smoother_test $<TARGET_FILE:costmap_savitzky_smoother>)

# Task 23: modules/costmap_pid_controller (3-axis PID local controller plugin)
    add_executable(costmap_pid_controller_test
      tests/costmap_pid_controller_test.cpp)
    target_link_libraries(costmap_pid_controller_test PRIVATE
      ayanami costmap_pid_controller simple_goal_checker GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapPidControllerTest COMMAND costmap_pid_controller_test
      $<TARGET_FILE:costmap_pid_controller>)

    # Task 24: modules/costmap_*_recovery (4 recovery plugins + dlopen factories)
    add_executable(costmap_recovery_modules_test
      tests/costmap_recovery_modules_test.cpp)
    target_link_libraries(costmap_recovery_modules_test PRIVATE
      ayanami costmap_spin_recovery costmap_back_up_recovery
      costmap_wait_recovery costmap_clear_costmap_recovery
      costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    target_compile_definitions(costmap_recovery_modules_test PRIVATE
      AYANAMI_MODULES_SRC="${CMAKE_SOURCE_DIR}/modules")
    add_test(NAME CostmapRecoveryModulesTest COMMAND costmap_recovery_modules_test
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:costmap_back_up_recovery>
      $<TARGET_FILE:costmap_wait_recovery>
      $<TARGET_FILE:costmap_clear_costmap_recovery>
      "${CMAKE_SOURCE_DIR}/modules")

    # Task 25: modules/costmap_navigation (AbstractNavigation host + BT preemption zero velocity)
    add_library(navigation_test_controller SHARED
      tests/plugins/navigation_test_controller.cpp)
    target_link_libraries(navigation_test_controller PRIVATE
      ayanami costmap_server::costmap_server)
    set_target_properties(navigation_test_controller PROPERTIES PREFIX "lib")

    add_executable(costmap_navigation_test tests/costmap_navigation_test.cpp)
    target_link_libraries(costmap_navigation_test PRIVATE
      ayanami costmap_navigation costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    target_compile_definitions(costmap_navigation_test PRIVATE
      AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
    add_test(NAME CostmapNavigationTest COMMAND costmap_navigation_test
      $<TARGET_FILE:costmap_navigation>
      $<TARGET_FILE:costmap_smac_planner>
      $<TARGET_FILE:costmap_savitzky_smoother>
      $<TARGET_FILE:costmap_pid_controller>
      $<TARGET_FILE:navigation_test_controller>
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:simple_goal_checker>
      $<TARGET_FILE:simple_progress_checker>)

    # Task 28 (M1 terminus): headless end-to-end E2E (dlopen host + simulated robot pose/pointcloud
    # + cmd_vel side-channel assertions + no crash on destruction).
    if(BUILD_WITH_BT)
      add_executable(headless_e2e_test tests/headless_e2e_test.cpp)
      target_link_libraries(headless_e2e_test PRIVATE
        ayanami costmap_navigation costmap_server::costmap_server
        GTest::gtest GTest::gtest_main)
      target_compile_definitions(headless_e2e_test PRIVATE
        AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
      add_test(NAME HeadlessE2E COMMAND headless_e2e_test
        $<TARGET_FILE:costmap_navigation>
        $<TARGET_FILE:costmap_smac_planner>
        $<TARGET_FILE:costmap_savitzky_smoother>
        $<TARGET_FILE:navigation_test_controller>
        $<TARGET_FILE:costmap_spin_recovery>
        $<TARGET_FILE:simple_goal_checker>
        $<TARGET_FILE:simple_progress_checker>
        "${CMAKE_SOURCE_DIR}/tests/assets")
    endif()

    # Task 26 wrap-up: checker unit tests (AbstractGoalChecker/AbstractProgressChecker abstractions
    # + SimpleGoalChecker/SimpleProgressChecker modules + PluginManager loading +
    # NavigationServerBase createActions injecting ControllerExecution).
    add_executable(checker_test tests/checker_test.cpp)
    target_link_libraries(checker_test PRIVATE
      ayanami simple_goal_checker simple_progress_checker
      GTest::gtest GTest::gtest_main)
    add_test(NAME CheckerTest COMMAND checker_test
      $<TARGET_FILE:simple_goal_checker>
      $<TARGET_FILE:simple_progress_checker>
      $<TARGET_FILE:test_planner_plugin>
      $<TARGET_FILE:test_smoother_plugin>
      $<TARGET_FILE:test_looping_controller_plugin>
      $<TARGET_FILE:test_recovery_plugin>)

    # Task 26: full config/*.json set + id<->name consistency check (spec §6.6/§9).
    # Requires dlopen'ing the host module + real algorithm module .so files for reverse-resolution assertions.
    add_executable(config_consistency_test tests/config_consistency_test.cpp)
    target_link_libraries(config_consistency_test PRIVATE
      ayanami costmap_navigation costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    target_compile_definitions(config_consistency_test PRIVATE
      AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
    add_test(NAME ConfigConsistencyTest COMMAND config_consistency_test
      "${CMAKE_SOURCE_DIR}/config"
      $<TARGET_FILE:costmap_navigation>
      $<TARGET_FILE:costmap_smac_planner>
      $<TARGET_FILE:costmap_savitzky_smoother>
      $<TARGET_FILE:costmap_pid_controller>
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:costmap_back_up_recovery>
      $<TARGET_FILE:costmap_wait_recovery>
      $<TARGET_FILE:costmap_clear_costmap_recovery>)

    # Task 27: replicates the 6 legacy gtest contracts (assertions rewritten equivalently against the new interfaces)
    add_executable(action_protocol_test tests/action_protocol_test.cpp)
    target_link_libraries(action_protocol_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ActionProtocolTest COMMAND action_protocol_test)

    add_executable(cancel_responsiveness_test tests/cancel_responsiveness_test.cpp)
    target_link_libraries(cancel_responsiveness_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME CancelResponsivenessTest COMMAND cancel_responsiveness_test)

    add_executable(continuous_goal_test tests/continuous_goal_test.cpp)
    target_link_libraries(continuous_goal_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ContinuousGoalTest COMMAND continuous_goal_test)

    add_executable(controller_session_test tests/controller_session_test.cpp)
    target_link_libraries(controller_session_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ControllerSessionTest COMMAND controller_session_test)

    add_executable(latest_wins_test tests/latest_wins_test.cpp)
    target_link_libraries(latest_wins_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME LatestWinsTest COMMAND latest_wins_test)

    if(BUILD_WITH_BT)
      add_executable(recovery_exhaustion_test tests/recovery_exhaustion_test.cpp)
      target_link_libraries(recovery_exhaustion_test PRIVATE
        ayanami GTest::gtest GTest::gtest_main)
      add_test(NAME RecoveryExhaustionTest COMMAND recovery_exhaustion_test)
    endif()

    # Requirement 2/3: all production modules loaded through the common factory (typed cast) + default tree extension
    # loaded/registered/executed independently.
    add_executable(extension_module_factory_test
      tests/extension_module_factory_test.cpp)
    target_link_libraries(extension_module_factory_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ExtensionModuleFactoryTest COMMAND extension_module_factory_test
      $<TARGET_FILE:costmap_navigation>
      $<TARGET_FILE:costmap_dwb_controller>
      $<TARGET_FILE:costmap_smac_planner>
      $<TARGET_FILE:costmap_savitzky_smoother>
      $<TARGET_FILE:costmap_pid_controller>
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:costmap_back_up_recovery>
      $<TARGET_FILE:costmap_wait_recovery>
      $<TARGET_FILE:costmap_clear_costmap_recovery>
      $<TARGET_FILE:simple_goal_checker>
      $<TARGET_FILE:simple_progress_checker>
      $<TARGET_FILE:navigation_behavior_tree>)

    # This phase: single source for session/version + failure context + plugin id dispatch +
    # controller in-place plan update (focus tests).
    add_executable(session_state_dispatch_test tests/session_state_dispatch_test.cpp)
    target_link_libraries(session_state_dispatch_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    target_compile_definitions(session_state_dispatch_test PRIVATE
      AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>")
    add_test(NAME SessionStateDispatchTest COMMAND session_state_dispatch_test
      $<TARGET_FILE:test_planner_plugin>
      $<TARGET_FILE:test_smoother_plugin>
      $<TARGET_FILE:test_controller_plugin>
      $<TARGET_FILE:test_recovery_plugin>)

    # Dedicated tests for PipelineSequence / RecoveryNode / RateController reset key scenarios
    # (nav2 semantics: planner+controller in parallel, replanning does not interrupt tracking, recovery retries).
    add_executable(pipeline_sequence_scenarios_test
      tests/pipeline_sequence_scenarios_test.cpp)
    target_link_libraries(pipeline_sequence_scenarios_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME PipelineSequenceScenarios COMMAND pipeline_sequence_scenarios_test)

    # goal_update_follow_test: goal update during navigation -> planner replans + smoother
    # re-smooths + controller follows the new path in place (rviz preemption scenario regression, production default tree).
    foreach(role PLANNER SMOOTHER CONTROLLER)
      add_library(goal_update_${role} SHARED
        tests/plugins/goal_update_follow_plugins.cpp)
      target_link_libraries(goal_update_${role} PRIVATE ayanami)
      target_compile_definitions(goal_update_${role} PRIVATE
        AYANAMI_GOAL_UPDATE_ROLE_${role})
      set_target_properties(goal_update_${role} PROPERTIES PREFIX "lib")
    endforeach()

    add_executable(goal_update_follow_test tests/goal_update_follow_test.cpp)
    target_link_libraries(goal_update_follow_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    target_compile_definitions(goal_update_follow_test PRIVATE
      AYANAMI_TEST_BT_EXT_SO="$<TARGET_FILE:navigation_behavior_tree>"
      AYANAMI_TEST_GOAL_UPDATE_PLANNER_SO="$<TARGET_FILE:goal_update_PLANNER>"
      AYANAMI_TEST_GOAL_UPDATE_SMOOTHER_SO="$<TARGET_FILE:goal_update_SMOOTHER>"
      AYANAMI_TEST_GOAL_UPDATE_CONTROLLER_SO="$<TARGET_FILE:goal_update_CONTROLLER>")
    add_test(NAME GoalUpdateFollow COMMAND goal_update_follow_test)
    set_tests_properties(GoalUpdateFollow PROPERTIES TIMEOUT 60)

    # production_tree_recovery_test: production default tree + MockRuntime recovery re-entry integration
    # test (regression for the rviz recovery storm bug: FollowPath must be re-enterable + bounded retry budget).
    add_executable(production_tree_recovery_test
      tests/production_tree_recovery_test.cpp)
    target_link_libraries(production_tree_recovery_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME ProductionTreeRecovery COMMAND production_tree_recovery_test)

    # local_window_rolling_test: local rolling window follows the robot (regression for the stall bug).
    add_executable(local_window_rolling_test
      tests/local_window_rolling_test.cpp)
    target_link_libraries(local_window_rolling_test PRIVATE
      ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME LocalWindowRolling COMMAND local_window_rolling_test)
  else()
    message(FATAL_ERROR "BUILD_TESTING=ON 需要系统 GTest (find_package(GTest) 失败)")
  endif()
endif()
