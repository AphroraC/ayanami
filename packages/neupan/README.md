# packages/neupan

Vendored copy of **libneupan** — the pure C++ (Eigen + OsqpEigen) port of the
NeuPAN planner (DUNE obstacle encoder + NRMP QP + PAN alternating loop).
Upstream sources:

- Algorithm reference: https://github.com/hanruihua/NeuPAN (Ruihua Han)
- C++ port tree copied from: `test/neupan/neupan_cpp/libneupan` in this
  workspace (author zhangkaiyuan007)

License: **GPL-3.0** (see LICENSE — ayanami itself is GPLv3, so the vendoring
is license-compatible).

## Local modifications (kept minimal, recorded here)

1. `CMakeLists.txt` rewritten for the ayanami packages/ build (STATIC lib +
   `neupan::neupan` alias, install via ayanamiTargets, upstream GTest suite
   registered as `neupan_*`); upstream `package.xml` dropped so catkin does
   not pick the directory up as a package.
2. Eigen 3.3 compatibility: the four `.reshaped()` call sites (mlp.cpp x3,
   dune.cpp x1) rewritten as `Eigen::Map` flattening (the workspace Eigen is
   3.3.7; `.reshaped()` is 3.4 API). Verified by the Python reference tests.
3. OsqpEigen 0.8 compatibility: `nrmp.cpp` solve()'s update path swapped from
   the osqp-1.x C call (`osqp_update_data_mat`) to the 0.8 wrappers
   (`updateLinearConstraintsMatrix` + `updateGradient` + `updateBounds`;
   P is constant and A's sparsity is fixed, so this is exact). Verified by
   test_nrmp / test_replay.
4. `Kinematics::Omni` implemented (robot.hpp/cpp + planner wiring):
   `omniRectangle` factory, u = (v, phi), B-matrix theta row zero (heading
   is the caller's business -- the ayanami module runs a small outer loop),
   C0 tracks position only (theta weight suppressed in the planner
   constructor), forward() converts (v, phi) -> (vx, vy). Tests:
   tests/test_omni.cpp. NOTE: the phi lever scales with the NOMINAL v
   (linearization property, upstream Python identical) -- at v = 0 lateral
   authority vanishes until v rebuilds.
5. `src/nrmp.cpp`: the dynamics sparsity pattern gained the u_phi column on
   the x/y rows (explicit zero for diff) so BOTH kinematics share one fixed
   pattern -- without it omni cannot command lateral motion (verified by a
   constraint-matrix coefficient trace).

Everything else is byte-identical to the upstream copy; do not edit upstream
files in place — append new entries here instead.

## Models

`models/*.bin` are NPTF-exported DUNE weights (rectangle footprint geometry
is baked into the weights; `tools/export_dune_weights.py` regenerates them
from a Python `.pth` offline).

## Tests / tools

- `neupan_*` GTest targets: upstream suite incl. reference-data replay
  against the Python implementation.
- `neupan_bench`: single-frame latency benchmark (stage-1 acceptance data).
