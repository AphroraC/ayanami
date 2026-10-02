# packages/esdfmap -- pure C++ ESDF sub-library (namespace esdfmap).
# Derives a signed distance field from an obstacle mask (navi-style fused
# semantics: live local marks OR projected global static geometry) and
# publishes immutable snapshots. No ayanami / ROS / Eigen dependency: the
# writer consumes injected readers (mask / global mask / 2D transform).
add_library(esdfmap STATIC
  src/esdf_writer.cpp)
add_library(esdfmap::esdfmap ALIAS esdfmap)

target_include_directories(esdfmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(esdfmap PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS esdfmap EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
