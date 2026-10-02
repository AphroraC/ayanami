# packages/neupan -- vendored libneupan (GPL-3.0, see LICENSE).
#
# Pure C++ port of the NeuPAN planner (DUNE encoder + NRMP QP + PAN
# alternating loop) by zhangkaiyuan007, ported from NeuPAN by Ruihua Han.
# Upstream: https://github.com/hanruihua/NeuPAN (algorithm) and the neupan_cpp
# port tree this was copied from (test/neupan/neupan_cpp/libneupan).
# Local modifications are listed in README.md. No ayanami / ROS dependency:
# yaml-cpp is only used by the optional fromYaml() entry; the ayanami module
# builds its NeuPANPlanner::Config programmatically.
#
# Vendored layout mirrors upstream (include/src/tests/tools/models) so future
# upstream diffs stay mechanical; the build glue is ayanami's packages/ style
# (STATIC lib + alias, GTest targets registered here).
# Eigen comes from the top-level find_package(Eigen3) (system 3.3.7 here;
# the reshaped() call sites were patched to 3.3-compatible Maps, see README)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(neupan STATIC
  src/mlp.cpp
  src/robot.cpp
  src/dune.cpp
  src/nrmp.cpp
  src/initial_path.cpp
  src/pan.cpp
  src/neupan_planner.cpp
  src/tensor_io.cpp
  src/log.cpp
  src/safety_supervisor.cpp)
add_library(neupan::neupan ALIAS neupan)

target_include_directories(neupan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# OSQP and yaml-cpp only appear in .cpp files, so they stay out of the
# interface (upstream rule, kept).
target_link_libraries(neupan PUBLIC Eigen3::Eigen
                             PRIVATE OsqpEigen::OsqpEigen yaml-cpp)

set_target_properties(neupan PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS neupan EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY models/ DESTINATION share/ayanami/neupan/models)

# ---- upstream GTest suite (reference data lives in tests/data) ----
# enable_testing() here: this subdirectory is added BEFORE the top-level
# BUILD_TESTING block runs, and add_test is ignored in a directory whose
# ancestors never enabled testing.
enable_testing()
find_package(GTest REQUIRED)
foreach(t test_config test_dune test_initial_path test_nrmp test_replay
          test_robot test_safety_supervisor test_tensor_io test_omni)
  add_executable(neupan_${t} tests/${t}.cpp)
  target_link_libraries(neupan_${t} PRIVATE neupan GTest::gtest GTest::gtest_main)
  target_include_directories(neupan_${t} PRIVATE tests)
  target_compile_definitions(neupan_${t} PRIVATE
    NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
    NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
  add_test(NAME neupan_${t} COMMAND neupan_${t})
endforeach()

# ---- upstream tools: single-frame benchmark (latency gate data) ----
add_executable(neupan_bench tools/bench.cpp)
target_link_libraries(neupan_bench PRIVATE neupan)
target_include_directories(neupan_bench PRIVATE tests)
target_compile_definitions(neupan_bench PRIVATE
  NEUPAN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  NEUPAN_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
