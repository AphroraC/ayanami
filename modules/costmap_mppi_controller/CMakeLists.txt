# modules/costmap_mppi_controller -- MPPI local controller in the 4D swerve
# control space (pure Eigen, no external dependencies).
ayanami_add_module(costmap_mppi_controller
  costmap_mppi_controller.cpp
  mppi_core.cpp
  mppi_critics.cpp
  swerve_kinematics.cpp
  module.cpp)
