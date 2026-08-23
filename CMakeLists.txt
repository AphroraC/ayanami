cmake_minimum_required(VERSION 3.16)
project(ayanami VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

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
find_package(spdlog REQUIRED)  # 或 vendored

# M2 Controller Ruling #M2-1: catkin 作构建系统 (不依赖 ROS 通信层)
# 仅引入 catkin_package() 让 catkin workspace 识别 ayanami 为 catkin pkg
# 并生成 devel/share/ayanami/cmake/ayanamiConfig.cmake 供下游 ayanami_ros
# find_package(ayanami COMPONENTS) 解析 (catkin component 解析路径)。
# M1 纯 CMake install export (lib/cmake/ayanami/) 与本 catkin_package 并存，
# 不破坏外部 (非 catkin 环境) 的 find_package(ayanami CONFIG)。
find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ayanami
    CFG_EXTRAS ""   # 不附加 catkin 自动 macros
  )
endif()

# 第三方接口库 (新增 cmake/thirdparty.cmake 引入)
include(${CMAKE_SOURCE_DIR}/cmake/thirdparty.cmake)  # 定义 BT::behaviortree_cpp / fast_tf::fast_tf / nlohmann_json / ankerl

# Task 14: BT 依赖源 (BUILD_WITH_BT=OFF 时剔除, 避免缺 BT 头编译失败)
set(AYANAMI_BT_SOURCES "")
if(BUILD_WITH_BT)
  list(APPEND AYANAMI_BT_SOURCES src/behavior_tree/pilot.cpp)
endif()

# Task 15: packages/* 纯 C++ 子库
add_subdirectory(packages/costmap_2d)
add_subdirectory(packages/costmap_layers)
add_subdirectory(packages/costmap_server)

# ayanami_add_module(name src...) 宏: 生成 SHARED 模块 .so, 链接 libayanami
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL}
  src/utility/config.cpp
  src/utility/logging.cpp
  src/utility/load_module.cpp
  src/utility/extension_module.cpp
  src/interfaces/callbacks.cpp
  include/ayanami/abstract/components/impl/execution_base.cpp
  include/ayanami/abstract/components/impl/plugin_manager.cpp
  src/components/navigation_server_base.cpp
  src/components/impl/navigation_server_base_impl.cpp
  src/costmap/components/costmap_navigation_server.cpp
  ${AYANAMI_BT_SOURCES}
  thirdparty/fast_tf/fast_tf.cpp)
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/modules>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
# Task 20: CostmapNavigationServer 公开头 include costmap_server/costmap_wrapper.hpp
target_link_libraries(ayanami PUBLIC costmap_server::costmap_server)
# src/utility/load_module.cpp uses dlfcn (dlopen/dlsym)
target_link_libraries(ayanami PRIVATE ${CMAKE_DL_LIBS})
# src/utility/config.cpp (via config_impl.hpp) 依赖 vendored nlohmann；
# 公开头 config.hpp 不 include nlohmann (spec §2.1)，故 PRIVATE 即可。
target_link_libraries(ayanami PRIVATE nlohmann_json::nlohmann_json)
if(BUILD_WITH_BT)
  target_link_libraries(ayanami PUBLIC BT::behaviortree_cpp fast_tf::fast_tf)
endif()
install(TARGETS ayanami EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY config assets DESTINATION share/ayanami
  PATTERN "*.yaml" EXCLUDE PATTERN "*.yml" EXCLUDE)

# 模块宏
function(ayanami_add_module name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE ayanami)
  target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/modules)
  set_target_properties(${name} PROPERTIES PREFIX "libayanami_" OUTPUT_NAME ${name})
  install(TARGETS ${name} LIBRARY DESTINATION lib)
  list(APPEND AYANAMI_INSTALL_TARGETS ${name})
  set(AYANAMI_INSTALL_TARGETS ${AYANAMI_INSTALL_TARGETS} PARENT_SCOPE)
endfunction()

# Task 21: modules/costmap_smac_planner (A* 全局路径规划插件)
ayanami_add_module(costmap_smac_planner
  modules/costmap_smac_planner/costmap_smac_planner.cpp
  modules/costmap_smac_planner/module.cpp)

# Task 22: modules/costmap_savitzky_smoother (7 点 SG 路径平滑插件)
ayanami_add_module(costmap_savitzky_smoother
  modules/costmap_savitzky_smoother/costmap_savitzky_smoother.cpp
  modules/costmap_savitzky_smoother/module.cpp)

# Task 23: modules/costmap_pid_controller (3 轴 PID 局部控制器插件)
ayanami_add_module(costmap_pid_controller
  modules/costmap_pid_controller/costmap_pid_controller.cpp
  modules/costmap_pid_controller/module.cpp)

# Task 24: modules/costmap_*_recovery (4 个恢复插件, spec §6.6 命名)
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

# Task 25: modules/costmap_navigation (AbstractNavigation 宿主插件)
ayanami_add_module(costmap_navigation
  modules/costmap_navigation/costmap_navigation.cpp
  modules/costmap_navigation/module.cpp)

# Task 26+: modules/simple_goal_checker / simple_progress_checker
# (目标检查 + 进展检查插件, abstract 族, 无地图依赖)
ayanami_add_module(simple_goal_checker
  modules/simple_goal_checker/simple_goal_checker.cpp
  modules/simple_goal_checker/module.cpp)
ayanami_add_module(simple_progress_checker
  modules/simple_progress_checker/simple_progress_checker.cpp
  modules/simple_progress_checker/module.cpp)

# 包导出
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

    # load_module / ExtensionModule 测试插件 (被 dlopen 的 .so)
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

    add_executable(load_module_test tests/load_module_test.cpp)
    target_link_libraries(load_module_test PRIVATE ayanami GTest::gtest)
    add_test(NAME LoadModuleTest COMMAND load_module_test
      $<TARGET_FILE:dummy_planner_module> $<TARGET_FILE:dummy_extension_module>)

    # NavigationCallbacks 跨 .so echo 测试插件 (dlopen 后 add 到槽, 链接 libayanami)
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
      $<TARGET_FILE:dummy_planner_module>)

    # Task 0: BLOCK #2 派生析构经 base ptr 可调 (abstract base virtual ~ 缺失 UB)
    add_executable(plugin_teardown_no_ub_test tests/plugin_teardown_no_ub_test.cpp)
    target_link_libraries(plugin_teardown_no_ub_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME PluginTeardownNoUB COMMAND plugin_teardown_no_ub_test)

    # Task 11: NavigationServerBase (4 族插件工厂在 navigation_plugins .so)
    add_library(navigation_plugins SHARED tests/plugins/navigation_plugins.cpp)
    target_link_libraries(navigation_plugins PRIVATE ayanami)
    set_target_properties(navigation_plugins PROPERTIES PREFIX "lib")

    add_executable(navigation_server_test tests/navigation_server_test.cpp)
    target_link_libraries(navigation_server_test PRIVATE ayanami GTest::gtest)
    add_test(NAME NavigationServerTest COMMAND navigation_server_test
      $<TARGET_FILE:navigation_plugins>)

    # Task 12: BT 控制节点 + 条件节点 + 黑板协议
    add_executable(behavior_tree_control_test tests/behavior_tree_control_test.cpp)
    target_link_libraries(behavior_tree_control_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    add_test(NAME BehaviorTreeControlTest COMMAND behavior_tree_control_test)

    # Task 13: BT action 节点 (TaskHandle 直连) + 注册 + 默认 XML
    add_executable(behavior_tree_action_test tests/behavior_tree_action_test.cpp)
    target_link_libraries(behavior_tree_action_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
    target_compile_definitions(behavior_tree_action_test PRIVATE
      AYANAMI_TEST_BT_XML="${CMAKE_SOURCE_DIR}/config/BT/navigation.xml")
    add_test(NAME BehaviorTreeActionTest COMMAND behavior_tree_action_test)

    # Task 14: BT Pilot (常驻树 + pending_goal 黑板 + 抢占零速度)
    if(BUILD_WITH_BT)
      add_executable(pilot_test tests/pilot_test.cpp)
      target_link_libraries(pilot_test PRIVATE ayanami GTest::gtest GTest::gtest_main)
      target_compile_definitions(pilot_test PRIVATE
        AYANAMI_TEST_BT_XML="${CMAKE_SOURCE_DIR}/config/BT/navigation.xml")
      add_test(NAME PilotTest COMMAND pilot_test)
    endif()

    # Task 15: packages/costmap_2d (纯 C++ header-only 子库)
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

    # Task 19: costmap/components (Costmap*Execution + Costmap*Action)
    add_executable(costmap_components_test tests/costmap_components_test.cpp)
    target_link_libraries(costmap_components_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapComponentsTest COMMAND costmap_components_test)

    # Task 1 (M2 阶段 A BLOCK #1): controller 整循环持锁 → per-cycle short lock,
    # 防 obstacle feed 饿死 (并发 controller + 障碍回调饿死回归)。
    add_executable(concurrent_controller_obstacle_test
      tests/concurrent_controller_obstacle_test.cpp)
    target_link_libraries(concurrent_controller_obstacle_test PRIVATE
      ayanami costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME ConcurrentControllerObstacle COMMAND concurrent_controller_obstacle_test)
    set_tests_properties(ConcurrentControllerObstacle PROPERTIES TIMEOUT 10)

    # Task 2 (M2 阶段 A BLOCK #3): recovery 锁 global+local (scoped_lock) 防
    # clear_costmap 写 local master grid 与 handleObstaclePointCloud 并发竞态。
    add_executable(recovery_vs_obstacle_test tests/recovery_vs_obstacle_test.cpp)
    target_link_libraries(recovery_vs_obstacle_test PRIVATE
      ayanami costmap_clear_costmap_recovery
      costmap_2d_layers::costmap_2d_layers costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    add_test(NAME RecoveryVsObstacle COMMAND recovery_vs_obstacle_test)
    set_tests_properties(RecoveryVsObstacle PROPERTIES TIMEOUT 10)

    # Task 3 (M2 阶段 A 收尾): minor carry wiring 综合 5 项
    # (smac start fallback / robot_frame 单一来源 / 二次 configureCoreMaps 守护 /
    #  pid 粗签名 / cmake find_dependency)。
    add_executable(minor_carry_wirings_test tests/minor_carry_wirings_test.cpp)
    target_link_libraries(minor_carry_wirings_test PRIVATE
      ayanami costmap_smac_planner costmap_pid_controller
      costmap_2d_layers::costmap_2d_layers costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    add_test(NAME MinorCarryWiring COMMAND minor_carry_wirings_test)

    # Task 20: costmap/components (CostmapNavigationServer virtual 注入地图)
    add_library(costmap_plugins SHARED tests/plugins/costmap_plugins.cpp)
    target_link_libraries(costmap_plugins PRIVATE
      ayanami costmap_server::costmap_server)
    set_target_properties(costmap_plugins PROPERTIES PREFIX "lib")

    add_executable(costmap_navigation_server_test
      tests/costmap_navigation_server_test.cpp)
    target_link_libraries(costmap_navigation_server_test PRIVATE
      ayanami costmap_smac_planner costmap_server::costmap_server GTest::gtest)
    target_compile_definitions(costmap_navigation_server_test PRIVATE
      AYANAMI_TEST_CONFIG_DIR="${CMAKE_SOURCE_DIR}/config")
    add_test(NAME CostmapNavigationServerTest COMMAND costmap_navigation_server_test
      $<TARGET_FILE:costmap_plugins>)

    # Task 21: modules/costmap_smac_planner (A* 全局路径规划插件)
    add_executable(costmap_smac_planner_test tests/costmap_smac_planner_test.cpp)
    target_link_libraries(costmap_smac_planner_test PRIVATE
      ayanami costmap_smac_planner costmap_2d_layers::costmap_2d_layers
      GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapSmacPlannerTest COMMAND costmap_smac_planner_test
      $<TARGET_FILE:costmap_smac_planner>)

    # Task 22: modules/costmap_savitzky_smoother (7 点 SG 路径平滑插件)
    add_executable(costmap_savitzky_smoother_test
      tests/costmap_savitzky_smoother_test.cpp)
    target_link_libraries(costmap_savitzky_smoother_test PRIVATE
      ayanami costmap_savitzky_smoother costmap_2d_layers::costmap_2d_layers
      GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapSavitzkySmootherTest COMMAND
      costmap_savitzky_smoother_test $<TARGET_FILE:costmap_savitzky_smoother>)

    # Task 23: modules/costmap_pid_controller (3 轴 PID 局部控制器插件)
    add_executable(costmap_pid_controller_test
      tests/costmap_pid_controller_test.cpp)
    target_link_libraries(costmap_pid_controller_test PRIVATE
      ayanami costmap_pid_controller simple_goal_checker GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapPidControllerTest COMMAND costmap_pid_controller_test
      $<TARGET_FILE:costmap_pid_controller>)

    # Task 24: modules/costmap_*_recovery (4 个恢复插件 + dlopen 工厂)
    add_executable(costmap_recovery_modules_test
      tests/costmap_recovery_modules_test.cpp)
    target_link_libraries(costmap_recovery_modules_test PRIVATE
      ayanami costmap_spin_recovery costmap_back_up_recovery
      costmap_wait_recovery costmap_clear_costmap_recovery
      costmap_server::costmap_server GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapRecoveryModulesTest COMMAND costmap_recovery_modules_test
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:costmap_back_up_recovery>
      $<TARGET_FILE:costmap_wait_recovery>
      $<TARGET_FILE:costmap_clear_costmap_recovery>)

    # Task 25: modules/costmap_navigation (AbstractNavigation 宿主 + BT 抢占零速度)
    add_library(navigation_test_controller SHARED
      tests/plugins/navigation_test_controller.cpp)
    target_link_libraries(navigation_test_controller PRIVATE
      ayanami costmap_server::costmap_server)
    set_target_properties(navigation_test_controller PROPERTIES PREFIX "lib")

    add_executable(costmap_navigation_test tests/costmap_navigation_test.cpp)
    target_link_libraries(costmap_navigation_test PRIVATE
      ayanami costmap_navigation costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
    add_test(NAME CostmapNavigationTest COMMAND costmap_navigation_test
      $<TARGET_FILE:costmap_navigation>
      $<TARGET_FILE:costmap_smac_planner>
      $<TARGET_FILE:costmap_savitzky_smoother>
      $<TARGET_FILE:costmap_pid_controller>
      $<TARGET_FILE:navigation_test_controller>
      $<TARGET_FILE:costmap_spin_recovery>
      $<TARGET_FILE:simple_goal_checker>
      $<TARGET_FILE:simple_progress_checker>)

    # Task 28 (M1 终止): headless E2E 全链路 (dlopen 宿主 + 模拟 robot pose/pointcloud
    # + cmd_vel 旁路断言 + 析构不崩)。
    if(BUILD_WITH_BT)
      add_executable(headless_e2e_test tests/headless_e2e_test.cpp)
      target_link_libraries(headless_e2e_test PRIVATE
        ayanami costmap_navigation costmap_server::costmap_server
        GTest::gtest GTest::gtest_main)
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

    # Task 26 收尾: checker 单测 (AbstractGoalChecker/AbstractProgressChecker 抽象
    # + SimpleGoalChecker/SimpleProgressChecker 模块 + PluginManager 装载 +
    # NavigationServerBase createActions 注入 ControllerExecution)。
    add_executable(checker_test tests/checker_test.cpp)
    target_link_libraries(checker_test PRIVATE
      ayanami simple_goal_checker simple_progress_checker
      GTest::gtest GTest::gtest_main)
    add_test(NAME CheckerTest COMMAND checker_test
      $<TARGET_FILE:simple_goal_checker>
      $<TARGET_FILE:simple_progress_checker>
      $<TARGET_FILE:navigation_plugins>)

    # Task 26: config/*.json 全套 + id↔name 一致性检查 (spec §6.6/§9)。
    # 需要 host 模块 dlopen + 真实算法模块 .so 做反向解析断言。
    add_executable(config_consistency_test tests/config_consistency_test.cpp)
    target_link_libraries(config_consistency_test PRIVATE
      ayanami costmap_navigation costmap_server::costmap_server
      GTest::gtest GTest::gtest_main)
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

    # Task 27: 复刻 legacy 6 个 gtest 契约 (新接口等价改写断言)
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
  else()
    message(FATAL_ERROR "BUILD_TESTING=ON 需要系统 GTest (find_package(GTest) 失败)")
  endif()
endif()
