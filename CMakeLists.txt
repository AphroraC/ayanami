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

# ayanami_add_module(name src...) 宏: 生成 SHARED 模块 .so, 链接 libayanami
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL}
  src/utility/config.cpp
  src/utility/load_module.cpp
  src/utility/extension_module.cpp
  src/interfaces/callbacks.cpp
  include/ayanami/components/impl/execution_base.cpp
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
  else()
    message(FATAL_ERROR "BUILD_TESTING=ON 需要系统 GTest (find_package(GTest) 失败)")
  endif()
endif()
