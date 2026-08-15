cmake_minimum_required(VERSION 3.16)
project(ayanami VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
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
find_package(spdlog REQUIRED)  # 或 vendored

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
  src/utility/load_module.cpp
  src/utility/extension_module.cpp
  src/interfaces/callbacks.cpp
  include/ayanami/components/impl/execution_base.cpp
  include/ayanami/components/impl/plugin_manager.cpp
  src/components/navigation_server_base.cpp
  src/components/impl/navigation_server_base_impl.cpp
  ${AYANAMI_BT_SOURCES}
  thirdparty/fast_tf/fast_tf.cpp)
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
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

# 模块宏
function(ayanami_add_module name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE ayanami)
  set_target_properties(${name} PROPERTIES PREFIX "libayanami_" OUTPUT_NAME ${name})
  install(TARGETS ${name} LIBRARY DESTINATION lib)
  list(APPEND AYANAMI_INSTALL_TARGETS ${name})
  set(AYANAMI_INSTALL_TARGETS ${AYANAMI_INSTALL_TARGETS} PARENT_SCOPE)
endfunction()

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
  else()
    message(FATAL_ERROR "BUILD_TESTING=ON 需要系统 GTest (find_package(GTest) 失败)")
  endif()
endif()
