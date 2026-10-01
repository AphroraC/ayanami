cmake_minimum_required(VERSION 3.16)
project(ayanami VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
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
  message(WARNING "ayanami: -mcx16 not supported; shared_ptr atomics fall back to the libstdc++ lock-pool implementation")
endif()

option(BUILD_SHARED "Build libayanami as shared library" ON)
option(BUILD_TESTING "Build tests" ON)

if(BUILD_SHARED)
  set(AYANAMI_BUILD_SHARED_BOOL SHARED)
else()
  set(AYANAMI_BUILD_SHARED_BOOL STATIC)
endif()

include(GNUInstallDirs)  # defines CMAKE_INSTALL_LIBDIR (used by install(EXPORT) below)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)  
# PointCloud2 message type (header + pcl::PointCloud<PointXYZI>)
find_package(PCL REQUIRED COMPONENTS common)

# Build model (mirrors glim): ayanami is a PURE C++ CMake library. catkin is
# NOT the build system -- package.xml declares <build_type>cmake</build_type>,
# and this block exists only so a ROS1 catkin workspace recognizes the package
# when ROS_VERSION=1 is in the environment. Outside a ROS environment the
# whole block is skipped and ayanami configures as a plain CMake project.
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ayanami
    )
  endif()
endif()

# Third-party interface libraries (introduced via the new cmake/thirdparty.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/thirdparty.cmake)  # defines BT::behaviortree_cpp / nlohmann_json / ankerl

# Task 15: packages/* pure C++ sub-libraries
add_subdirectory(packages/costmap_2d)
add_subdirectory(packages/costmap_layers)
add_subdirectory(packages/costmap_server)

# ayanami_add_module(name src...) macro: builds a SHARED module .so linked against libayanami
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL}
  src/utility/config.cpp
  src/utility/logging.cpp
  src/utility/bootstrap.cpp
  src/utility/load_module.cpp
  src/utility/extension_module.cpp
  src/interfaces/callbacks.cpp
  src/abstract/components/execution_base.cpp
  src/abstract/components/module_manager.cpp
  src/behavior_tree/pilot.cpp
  src/components/navigation_server_base.cpp
  src/components/robot_state_controller.cpp
  src/costmap/components/costmap_navigation_server.cpp
  src/rosemap/components/rosemap_navigation_server.cpp)
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/modules>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
if(AYANAMI_HAS_MCX16)
  target_compile_options(ayanami PUBLIC -mcx16)
endif()
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
# messages/sensor_msgs.hpp (PointCloud2) uses PCL point types
target_link_libraries(ayanami PUBLIC ${PCL_LIBRARIES})
target_include_directories(ayanami SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
# Task 20: CostmapNavigationServer public headers include costmap_server/costmap_wrapper.hpp
target_link_libraries(ayanami PUBLIC costmap_server::costmap_server)
# src/utility/load_module.cpp uses dlfcn (dlopen/dlsym)
target_link_libraries(ayanami PRIVATE ${CMAKE_DL_LIBS})
# src/utility/config.cpp (via config_impl.hpp) depends on the vendored nlohmann;
# the public header config.hpp does not include nlohmann (spec §2.1), so PRIVATE is enough.
target_link_libraries(ayanami PRIVATE nlohmann_json::nlohmann_json)
# BehaviorTree.CPP v4 (vendored, always built): the behavior-tree pilot is a
# core component. BT is a PRIVATE dependency statically embedded into
# libayanami (see thirdparty.cmake) -- never installed or exported, so it
# cannot collide with another BehaviorTree.CPP in the install space.
# --whole-archive embeds EVERY BT object (a plain archive link pulls members
# on demand, which would leave BT classes unreferenced by ayanami itself --
# loggers etc. -- unresolved for build-tree consumers). $<BUILD_INTERFACE>
# keeps the archive out of the exported target even for BUILD_SHARED=OFF (a
# static libayanami consumer must link BT itself).
target_link_libraries(ayanami PRIVATE
  $<BUILD_INTERFACE:-Wl,--whole-archive>
  $<BUILD_INTERFACE:BT::behaviortree_cpp>
  $<BUILD_INTERFACE:-Wl,--no-whole-archive>)
# BT headers still reach build-tree consumers (modules/, tests/) that include
# pilot.hpp / behavior_tree/*.hpp; nothing BT-related is installed.
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/BehaviorTree.CPP/include>)
# NOTE: deliberately NO -Wl,-Bsymbolic here. It would bind ayanami's internal
# references to its own weak/vague-linkage definitions (BT inline-function
# statics, typeinfo), defeating the cross-DSO ODR merging that dlopen'd
# modules rely on (measured: recovery-role blackboard entries vanished).
install(TARGETS ayanami EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
# NOTE: config/ is deliberately NOT installed. The runtime reads the
# SOURCE-TREE config (launch files pin an absolute config_path; the devel
# snapshot this install used to produce only refreshed on rebuild, which
# made parameter edits silently stale until the next build).
# If an install-space deployment is ever needed, restore the install rule
# AND point the launch config_path at the install share dir.

# ---- Production modules ----
# Each module builds from its own modules/<name>/CMakeLists.txt via the
# ayanami_add_module helper. Per-module cached switches AYANAMI_BUILD_<NAME>
# are generated from this list (all ON by default; AYANAMI_BUILD_MODULES=OFF
# flips the default for every module). Tests declare module dependencies and
# skip themselves when a module is disabled (see cmake/ayanami-tests.cmake).
include(${PROJECT_SOURCE_DIR}/cmake/ayanami-modules.cmake)

set(AYANAMI_MODULES
  costmap_astar_planner            # planner
  costmap_minsnap_smoother         # smoother (minimum-snap)
  costmap_pid_controller           # controller
  costmap_mppi_controller          # controller (MPPI, 4D swerve space)
  costmap_spin_recovery            # recovery
  costmap_back_up_recovery
  costmap_wait_recovery
  costmap_cleanup_recovery
  costmap_static_layer             # costmap layer plugins
  costmap_obstacle_layer
  costmap_inflation_layer
  costmap_navigation               # host module (AbstractNavigation)
  rosemap_navigation               # host module (rosemap family)
  rosemap_astar_planner            # planner (rosemap family)
  rosemap_minco_smoother           # smoother (rosemap family)
  rosemap_mpc_controller           # controller (rosemap family, needs OsqpEigen)
  rosemap_escape_recovery          # recovery (rosemap family)
  rosemap_wait_recovery            # recovery (rosemap family)
  simple_goal_checker              # checkers (abstract family)
  simple_progress_checker
  navigation_behavior_tree         # BT extension (family-neutral)
)

option(AYANAMI_BUILD_MODULES "Build the production modules" ON)
# Map-family master switches: coarse gates over their family's modules. OFF
# wins over AYANAMI_BUILD_MODULES and any per-module switch (a disabled
# family never builds). Family-neutral modules (the checkers and the BT
# extension) answer only to AYANAMI_BUILD_MODULES / their per-module switch.
option(BUILD_COSTMAP_MODULES "Build the costmap map-family modules" ON)
option(BUILD_ROSEMAP_MODULES "Build the rosemap map-family modules" ON)
foreach(_module ${AYANAMI_MODULES})
  string(TOUPPER "${_module}" _module_uc)
  option(AYANAMI_BUILD_${_module_uc} "Build module ${_module}"
    ${AYANAMI_BUILD_MODULES})
endforeach()

foreach(_module ${AYANAMI_MODULES})
  string(TOUPPER "${_module}" _module_uc)
  if(NOT AYANAMI_BUILD_${_module_uc})
    message(STATUS "ayanami: module ${_module} disabled (AYANAMI_BUILD_${_module_uc}=OFF)")
    continue()
  endif()
  if(_module MATCHES "^costmap_" AND NOT BUILD_COSTMAP_MODULES)
    message(STATUS "ayanami: module ${_module} disabled (BUILD_COSTMAP_MODULES=OFF)")
    continue()
  endif()
  if(_module MATCHES "^rosemap_" AND NOT BUILD_ROSEMAP_MODULES)
    message(STATUS "ayanami: module ${_module} disabled (BUILD_ROSEMAP_MODULES=OFF)")
    continue()
  endif()
  add_subdirectory(modules/${_module} modules/${_module})
endforeach()

# Package export
install(EXPORT ayanamiTargets NAMESPACE ayanami:: FILE ayanamiTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/ayanami-config.cmake.in
  ${CMAKE_BINARY_DIR}/ayanami-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
write_basic_package_version_file(ayanami-configVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_BINARY_DIR}/ayanami-config.cmake ${CMAKE_BINARY_DIR}/ayanami-configVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)

# ---- Tests ----
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  include(${PROJECT_SOURCE_DIR}/cmake/ayanami-tests.cmake)
  add_subdirectory(tests)
endif()
