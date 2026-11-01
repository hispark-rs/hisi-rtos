# Embassy 时间契约

此开发分支在现有 `hisi-rtos` 内整合时间驱动，不新建平行组件。
`embassy` 保留调度器共享定时器模式；`embassy-only,unstable,chip-*` 是新增实验模式，
不编译线程调度器、上下文切换和栈存储，不安装 RF 运行时。两个模式须分别构建。
发布的 alpha.25 尚不包含新模式；开发测试固定远端提交，不视为发布验收。

| 契约 | `embassy` | `embassy-only` |
|---|---|---|
| 时间来源 | 注入的 `monotonic_ms` | 平台显式交出的 HAL TCXO |
| ABI tick | 1 µs | 1 µs |
| 时间分辨率 | **1 ms**；乘 1000 不增加精度 | 配置为 1 µs，物理频率未由此证明 |
| 期限 | 向上舍入毫秒，与线程、时间片、预算和锁期限取最早 | 绝对微秒期限，32 位闹钟分段 |
| 所有者 | 已有 `SchedulerPort` | 全部 TIMER 和 TCXO 永久交接 |
| 初始化 | 原 start/port 契约 | `unsafe embassy::init(timer, clock)` |
| 失败 | 保留原启动合同 | 返回资源；时钟检查可能留下快照请求 |

两种模式复用 `embassy/queue.rs`，只在 `embassy/mod.rs` 注册一次全局驱动。
它们编译互斥，运行中不会创建两套队列或两个硬件定时器所有者。HAL 不反向依赖 RTOS。
`embassy-only` 禁止 scheduler start API，Cargo feature 误合并会使线程消费者编译失败；
不是运行时切换模式。现有 mandatory Cargo 依赖仍参与解析，不代表纯模式启动了堆或 RF。

## 队列与竞态

16 个不同 Waker 槽位；同任务保留最早期限。满队列时取出最晚任务并在锁外唤醒，
使其重新检查 Future；唤醒不表示 Timer 已到期。`u64::MAX` 也是合法期限。
取消 Future 不撤回已注册唤醒，最多保留有界 stale Waker。

沿用原 WS53 的有界实现而非再保留 queue-utils：后者的过期和满队列路径会在队列
借用内执行回调，不满足此次锁外 clone/drop/wake 及重入契约。两种模式统一使用本实现。
共享模式在调用过期 Waker **之前**领取重编程 generation；回调造成新计划时旧写入必须重试。
纯模式通过 busy/dirty 处理编程期间插入和 IRQ；计数快照未完成时保留单调缓存并短期重试。
计数倒退或完整回绕报告 epoch 故障，不假装已支持跨重置时间延续。

`tests` 覆盖回绕、长时间、重入、队列满、空/重复通知、编程窗口、取消和初始化失败。
模型测试不证明真实 MMIO 延迟、多核交接、物理频率或低功耗恢复。改变唤醒位置的共享
模式和全部新适配必须重新实板验收，不能继承历史 A3 或 HAL 的成功记录。

## 平台接入与迁移

纯模式将 `hisi_hal::embassy` 替换为 `hisi_rtos::embassy`，依赖启用
`embassy-only,unstable` 与唯一 `chip-ws63/chip-ws53/chip-bs21`。
应用先建立独占资源、稳定时钟、跨核静止及 IRQ 路由，再交出静态 TIMER/TCXO；成功后无
`deinit`。全部 TIMER 通道禁止另作 delay 或调度器使用，TCXO 不能另给 DMA 等待使用。
详细可审计的 unsafe 前提以 `embassy::init` API 文档为准。

普通 HAL `async` 不需要 Embassy/RTOS，继续使用执行器无关的 Future、Waker 和取消状态机。
WS63/BS2X 普通外设 API 保持兼容，原 HAL Embassy 是实验接口；共享 `embassy` 的原有
启动 API 保持不变。WS53/BS2X 的线程调度端口尚未实现，不能因纯模式能运行就宣称支持。
未来如拆出 `hisi-rtos-embassy`，必须先抽取 core 契约、由门面组合，避免反向依赖循环。

## 独立验证

`scripts/check-time-profiles.sh` 在本仓运行主机测试、文档测试、三芯片 Clippy、非法 feature
检查和公共依赖边界检查。开发依赖覆盖显式复制 `.ci/development-dependencies.toml` 到独立检出的
`.cargo/config.toml`，使 trybuild 子进程继承相同远端图；默认使用 registry 和
本仓锁文件。父仓只负责组合场景的 QEMU 与硬件验收。

未发布阻塞：HAL alpha.10、支持 `chip-ws53` 的 runtime、RF driver alpha.20 以及本 profile。
`ws53-pac 0.1.0-alpha.1` 已发布。这些阻塞不能由本机 patch 通过来关闭；本轮不发布本 crate。
