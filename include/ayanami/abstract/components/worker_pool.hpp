#pragma once

#include <cstddef>
#include <functional>
#include <memory>

namespace ayanami {
namespace components {

/**
 * @brief 进程级常驻线程池 (取代 "每次请求临时 std::thread" 的执行宿主)。
 *
 * ActionBase 槽驱动任务 (runImpl 一次完整执行) 与 ExecutionBase 状态机任务
 * (preRun→run→postRun) 都作为 std::function<void()> 提交到此池, 由固定 N 个
 * worker 经 FIFO 队列 + condition_variable 消费。同槽串行由 ActionBase 提交侧
 * 完成 (提交前等待前序任务结束, 等价 join), 不同槽由池内多 worker 自然并行。
 *
 * 跨 DSO 单例纪律 (同 TransformStore, spec Ruling): instance() 与类实现全部
 * 定义在 src/abstract/components/worker_pool.cpp (非 inline), 保证 libayanami
 * 及其插件 .so 解析到同一实例。
 *
 * 生命周期: 池懒创建于首次 submit; 析构 (进程退出/库卸载) 优雅排空 —— 置停
 * 标志唤醒并 join 全部 worker, 队列遗留任务在析构线程尽力执行。停池之后 submit
 * 直接丢弃 (组件应在自身析构中先等齐其任务, 正常路径不会触发)。
 */
class WorkerPool {
public:
  static WorkerPool& instance();

  WorkerPool(const WorkerPool&) = delete;
  WorkerPool& operator=(const WorkerPool&) = delete;
  WorkerPool(WorkerPool&&) = delete;
  WorkerPool& operator=(WorkerPool&&) = delete;

  /// @brief 提交任务 (fire-and-forget, 内部吸收任务异常)。
  void submit(std::function<void()> task);

  /// @brief 当前队列中未取走的任务数 (观测/测试用)。
  std::size_t queuedCount() const;

  /// @brief 停池 (排空并停 worker); 幂等。
  void shutdown();

private:
  WorkerPool();
  ~WorkerPool();

  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace components
} // namespace ayanami