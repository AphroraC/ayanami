# Task 3 Report: minor carry wiring 综合 (ayanami M2 Plan)

## Status
DONE_WITH_CONCERNS

## Commits
- `fix: minor carry wiring (robot pose fallback / robot_frame 单一来源 / find_dependency / plan_signature 粗签名 / configureCoreMaps 二次守护)` (见 Commits 行, git log 查)

## Test summary
- `minor_carry_wirings_test` (MinorCarryWiring, 4 个 TEST 合并 1 个 ctest target): **PASS**
  - SmacPlannerFallsBackToRobotPose / RobotFrameSingleSourceFromCostmapNavigationJson /
    ConfigureCoreMapsTwiceWarns / PidSignatureCoarseSameGoalNoReset
- 全量 `ctest --test-dir build_m1 --output-on-failure`: **37/37 PASS** (M1 36 + T3 1)。
  - TDD: 先写 test 全红 (4 FAILED), 实现后全绿。
- 静态红线 grep 通过:
  - `config/costmap.json` 无 `robot_frame` 字段; `config/costmap_navigation.json` 保留 `robot_frame: "base_footprint"`。
  - src 内无 `cfg.param<std::string>("costmap","robot_frame",...)` 残留。
  - `cmake/ayanami-config.cmake.in` 含 `find_dependency(Eigen3/spdlog/Threads)`。

## Files (7)
- `CMakeLists.txt` — 新增 minor_carry_wirings_test target + MinorCarryWiring ctest
- `cmake/ayanami-config.cmake.in` — 3.4 find_dependency (T0 minor carry)
- `config/costmap.json` — 3.2 移除 robot_frame 字段 (其余字段保持)
- `modules/costmap_pid_controller/costmap_pid_controller.cpp` — 3.5 computePlanSignature 粗签名 (goal×1000 + size; 移除 FNV-1a 逐 pose hash)
- `modules/costmap_smac_planner/costmap_smac_planner.cpp` — 3.1 start 留空 fallback getRobotPose
- `src/costmap/components/costmap_navigation_server.cpp` — 3.2 robot_frame 单一来源 + robot_state_provider_.setFrames + ayanami_ros 一致性校验; 3.3 二次 configureCoreMaps 守护
- `tests/minor_carry_wirings_test.cpp` — 4 TEST (brief Step 1 verbatim)

## Concerns (DONE_WITH_CONCERNS)

1. **robot_frame 读取实现偏离 task instruction 字面**: 3.2 指令写
   `GlobalConfig::instance()->param<std::string>("navigation","robot_frame","base_link")`,
   但 `GlobalConfig::instance()` 的文档只含根 `config.json` 的 `global` 段 (mapping),
   **不含** `navigation` 段内容 (sub-config 不 merge)。按字面实现会恒回退 `base_link`,
   使 `RobotFrameSingleSourceFromCostmapNavigationJson` (期望 `base_footprint`) 无法通过。
   改为 codebase 既有模式 `Config nav_cfg(GlobalConfig::get_config_path("config_navigation"))`
   + `nav_cfg.param("navigation","robot_frame","base_link")` (与 loadModules/pilot 一致),
   语义仍为 "robot_frame 单一来源 = costmap_navigation.json"。若 T10 打算让 config.json
   内联 navigation 段, 后续可切回字面形式。

2. **brief Files 列表含 `controller_execution.hpp (RobotStateProvider fallback)`, 但 Step 3
   的 5 项实现与 task instruction 均未包含**, 未改该文件 (无对应 carry 需求)。

3. **二次 configureCoreMaps 为 void, 无 return false**: brief 伪代码 "return false" 不适用
   (函数签名 void)。守护为 `spdlog::warn + return`, 测试断言同 ptr 不替换 + warn 文本
   ("called twice") 被捕获。

4. **pid 粗签名测试与 brief Step 1 "多加中间点" 措辞略不同**: 粗签名含 size 项,
   "多加中间点"会改 size → 签名变化 → 重置, 与 no-reset 断言矛盾。改为同 size 同 goal
   仅中间点平移 0.005 (真实 re-smooth 场景), 验证不重置 (ramp 连续 c3≈3*max_acc*dt)。
   旧细签名下该场景会重置 (test red), 证明测试有效。

5. **`ayanami_ros` 一致性校验在 Release (NDEBUG) 下 assert 为 no-op**: 构造期 assert 仅
   Debug 生效; mismatch 时 Release 只 spdlog::error。核心 scope 无 ayanami_ros 段 → 跳过,
   符合 brief "config_ros 未载则跳过"。

## Self-review
- 未修改 spec/plan/ledger; 未 spawn 子 subagent; 未引入 ROS。
- M1 公开接口未破坏: makePlan/setPlan/configureCoreMaps 签名不变; 现有 36 测试全绿。
- costmap.json 其余字段 (sizes/layers/inflation/footprint 等) 保持不变。
- 未 git add -A / 未 push; 仅 add 涉及 7 文件。
