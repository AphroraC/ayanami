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
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL} src/dummy.cpp)  # 占位; 后续 task 替换为真实源
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
if(BUILD_WITH_BT)
  target_link_libraries(ayanami PUBLIC BT::behaviortree_cpp fast_tf::fast_tf nlohmann_json::nlohmann_json)
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
