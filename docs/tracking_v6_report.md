# 6 m/s 多气动模型跟踪测试报告

> 历史窗口报告，指标保持不变。后续已实现完整尾段与末点重复，见
> [6/8/10/12 m/s + 末端保持报告](tracking_speed_report.md)。

## 结论

**本轮完成性与科学数据验收通过：五种 plant 气动模型均完成同一 17.48 s 可执行窗口，所有求解状态为 0，离线回放均通过独立验证。** 位置 RMSE 为 0.01859–0.18528 m；最差位置瞬时误差为 0.21697 m。这里没有预先冻结的精度容差，因此不把这些数值写成某项未定义精度规格的“达标”。

**这不是严格限速验收，也不是飞行验收。** `v_max=6.0` 是规划软约束设置，参考峰值为 **6.0020065575 m/s**，实际采样峰值最高为 **6.0332740698 m/s**。完成的 17.48 s 是扣除 NMPC 预测窗口后的实验全窗口，不是整个 18.4845550315 s 参考轨迹。

下一步先在洁净提交上重复该基线，并冻结精度/限速验收规格；随后迁移旧优化代价坐标语义，补足 guard/过渡与模型误差诊断。不能从一次轨迹的结果推广到全工况或硬件。

## 1. 数据与可阅读报告在哪里

项目根记作绝对路径 `AP_PNC_DIR`。本报告的科学权威数据都在本机，未上传私有参数或原始日志。

| 内容 | 项目根锚定位置 |
|---|---|
| 本报告 | `$AP_PNC_DIR/docs/tracking_v6_report.md` |
| 测试规范 | `$AP_PNC_DIR/docs/tracking_evaluation.md` |
| 五模型任务索引 | `$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu/index.json` |
| 原始指标汇总与源文件 SHA256 | `$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu/summary.json` |
| 本次重新核验回执 | `$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu/report-verification.json` |
| 原始科学图，26 张 | `$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu/figures/` |
| 手机版 HTML 与选定图表 | `$AP_PNC_DIR/.artifacts/html-reports/tracking-v6-20261005/index.html` |

HTML 发布状态另存本地产物。只有上传、HTTPS 和浏览器验证均成功时，才能把其链接称为可访问报告；不能以本地生成成功代替公网交付。

## 2. 测试条件与窗口

- 执行日期：2026-10-05；原生 Linux-aarch64 Docker，GNU 11.4.0。
- 编排：Rust/dora；数值引擎：当前 C++ golden。不是全 Rust 算法验收。
- 主坐标：世界 ENU，机体 FLU，body +Z collective；public reference 直接生成 FLU。
- NMPC：15 状态/4 输入/7 参数的一阶 collective 与三轴角速度模型，solver ABI 2。
- Plant：同一四电机 `matlab_vtol` profile，RK4；保留机翼气动力，无舵面。
- 物理步长 1 ms；控制周期 20 ms；无随机过程，零风。
- NMPC horizon 1.0 s，10 个 0.1 s 区间，含 stage 0–10；每 tick 11 行 reference。
- 五模型只改变 plant 的气动模型：`none / lyu / phi / ma / advanced`。其余 simulation 设置、profile、生成 OCP 和规划配置相同；没有逐模型重调权重。
- 五个 `reference.yaml` 的 SHA256 完全相同。

**时间范围必须分开：**

| 时间对象 | 范围/数量 | 含义 |
|---|---:|---|
| 保存的多项式参考 | 18.484555031513143 s | 覆盖预测用的尾段 |
| 理论可执行窗口 | 17.484555031513143 s | 参考长度减去 1.0 s horizon |
| 实际实验结束 | 17.48 s | 20 ms 整周期向下取整 |
| 控制调用时间 | 0–17.46 s | 874 次调用/提交 |
| 科学指标采样 | 875 个状态 | 含 17.48 s 最终状态 |
| 最后 terminal reference | 18.46 s | 未越过保存的参考范围 |

因此尾段约 1.0046 s 没有作为 plant 的当前状态执行，但仍供 NMPC 预测使用。本轮不是末端停稳/落地/全轨迹尾段验收。

## 3. 五模型同窗口结果

指标来自原始 `metrics.json`。本次从 `steps.csv` 和保存的多项式重算了 875 个状态的指标，与原值差异小于 `1e-12`。

| Plant 气动模型 | 完成时间 s | 位置 RMSE m | 速度 RMSE m/s | 位置最大误差 m | 实际采样速度峰值 m/s |
|---|---:|---:|---:|---:|---:|
| none | 17.48 | 0.019775987 | 0.025029896 | 0.063869588 | 6.033274070 |
| lyu | 17.48 | 0.058769583 | 0.029596178 | 0.083835627 | 6.010737304 |
| phi | 17.48 | 0.185284284 | 0.059018177 | 0.216971746 | 6.007318214 |
| ma | 17.48 | 0.018588729 | 0.020206681 | 0.058015245 | 6.025513220 |
| advanced | 17.48 | 0.028620447 | 0.037416265 | 0.103005938 | 6.010144119 |

每个模型均为 874 committed steps、875 metric samples；实际 solver status 全为 0，status2/status4 均为 0。每个模型均有独立 RRD decoder 验证回执；本次重新核对了回执中的源文件和 recording 哈希。

**结果含义：** `ma` 在本轨迹中位置/速度 RMSE 最小；`phi` 的位置误差最大，曲线有持续偏差。该排序只反映同一控制器、同一参考下的闭环表现，不证明哪个气动模型最接近真实飞机，也不确认偏差的物理根因。

位置误差定义为 `||p_actual - p_ref||₂`；位置 RMSE 为其平方均值的平方根。速度 RMSE 同理。实际速度峰值包含最终状态，但仍是 20 ms 采样峰值，不能当连续时间上界。

## 4. “6 m/s”的准确含义

- 规划设置：`problem_formulation.cost.v_max = 6.0`。
- 保存参考峰值：`6.0020065574951325 m/s`；由每段速度模长平方的端点和实驻点求最大值，不是从标签或稀疏曲线估计。
- 参考越界：约 `0.00200656 m/s`，相对 6 m/s 约 `0.03344%`。
- 最高实际采样速度出现在 `none`：约比 6 m/s 高 `0.0332741 m/s`，约 `0.55457%`。

**结论：可称“v_max 设置为 6 的软约束跟踪试验”，不可称“参考与飞机全程严格不超过 6 m/s”。** 若需要硬限速，必须另定容差、验证连续时间峰值并设计约束，不能改报告标签解决。

## 5. 实际 NMPC reference 审计

每模型 `nmpc_reference.csv` 有 **9614 行**，即 874 tick × 11 stage。它记录求解器实际收到的 `yref`、planner reference、x0、参数及 status，而非事后由好看的曲线重建。

本次重算确认，所有 stage 的 planner→solver **p/v/yb 最大绝对差均为 0**。stage0 相邻 yb 点最小 dot 为 `0.9926471588741538`，负 dot 事件为 0。本轨迹没有观察到 yb 正负号翻转，但不等于所有低速/反向/guard 工况均已通过。

| Objective 通道 | 实际目标 | 权重与解释 |
|---|---|---|
| p_W / v_W | planner 原值 | future 与 terminal 有权重 |
| body Y 在 W 的方向 | planner 原值，未取负 | future `[52.5,52.5,126]` |
| state omega_B | 0 | future/terminal 权重为 0，不是 active planned-rate tracking |
| collective command | hover 9.81 N/kg | 非 terminal 输入代价，权重 0.03 |
| rate command | 三轴 0 | 非 terminal 输入代价，各轴 0.03 |
| stage0 所有通道 | 如实记录 | **W0 全为 0**，不能称 active stage0 tracking objective |

Future 的 p 权重为 `[15,15,10]`，v 为 `[1,1,1]`；terminal 取前 12 通道。Planner 计算出的物理 FLU omega/thrust 并没有被当作当前 solver rate/collective command reference。实际命令是优化的 `u0`，不是未来 knot 的预测状态。

Future horizon 图按 **reference time** 画点；不能把未来 reference 与当前 plant 状态当作同一时刻比较。各模型 body-Y 实际方向误差最大约 5.26°；这只是方向通道误差，不是完整姿态误差。

## 6. 耗时证据与实时边界

下表是原始 `solve_time_ms` 的 wall-time 分位数，单位 ms。其真实计时范围为 **整个 `controller.compute()`**，包括 reference 采样及 reference CSV 写入/flush。它不是 acados 内部纯求解时间，也不包含完整 plant/RK4 周期。

| 模型 | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| none | 0.473 | 0.584 | 0.716 | 2.587 |
| lyu | 0.463 | 0.494 | 0.590 | 6.187 |
| phi | 0.530 | 0.575 | 0.679 | 0.946 |
| ma | 0.462 | 0.518 | 0.630 | 0.788 |
| advanced | 0.460 | 0.493 | 0.610 | 1.157 |

**这仅说明本机离线运行中测得的控制器耗时，不证明完整控制周期满足硬实时 20 ms deadline。** CPU 负载、虚拟机调度、启动瞬态和 I/O 均可能影响 wall time。仿真时间与 wall time 分开。

## 7. 图表清单与解读

原始图目录为 `$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu/figures/`。每模型均有以下五张，共 25 张，另有 `comparison.png`。

| 文件后缀 | 内容 | 应如何阅读 |
|---|---|---|
| `*-tracking.png` | p/v、实际 omega/命令、yb | world ENU 与 body FLU 不能混读；零 solver rate reference 不等于 planned omega |
| `*-path.png` | 3D ENU reference/actual | 路径重合不能替代误差曲线；近零 z 的自动缩放可能夸大视差 |
| `*-diagnostics.png` | 位置/yb 误差、collective、气动力矩 | actuator Fz/m 与含 aero 的 IMU z 不是同一量 |
| `*-nmpc-reference.png` | stage0 实际 yref 与 planner | 说明输入一致性；标题注明 W0=0 |
| `*-horizons.png` | tick100/300/600 的 future knots | 横轴是 reference time，terminal 单独有未来时间 |
| `comparison.png` | 五模型误差与实际速度 | matched-window 对照；虚线 6 是软设置而非硬上界 |

HTML 另有手机可读的误差、速度及 Lyu body-Y 图；这些图只读取本次已保存数据，不重新运行 planner 或 plant。五模型完整 tracking 图也包含在选定报告 bundle 中。

## 8. 来源、复核与复现

本轮执行时 manifest 记录 `a2face08797c786b49847249b58f254954dcb363 + dirty=true`。相关源码后来归档到 `ded009840ea5e94a36ef07392192e8e4fd482d39`。**本报告没有声称五次历史运行来自 ded0098 的洁净检出重跑。** Job 保留了源码状态、镜像身份、可执行文件/solver SHA、Cargo.lock 和本地上游 provenance。

共同参考 SHA256：

```text
e3b7ce105b4aa1f4d872e8f4e2236ac483b235cc054fa7633be7b99a3371c709
```

Solver bundle SHA256：

```text
5254e096bb0ceac5647748468f6210ec49f6e42e4e12bcb5101f731374fa103f
```

生成 OCP SHA256：

```text
86b81cdec9082e647b0f1bea7fbd630fce694a0f36680746f93179088509bfa4
```

科学 run 的绝对路径从 `index.json` 读取，不猜测另一个目录。每个 run 内含 `steps.csv / nmpc_reference.csv / reference.yaml / planning.yaml / nmpc.yaml / simple_sim.yaml / manifest.json / metrics.json / replay.rrd / replay.json / verified.json`。私有 profile 与上游内容仅本地保存，不属于公开报告 bundle。

仅重画现有科学数据，不重跑仿真：

```bash
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" plot \
  --output "$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-flu"
```

独立回放/坏数据验收使用 `index.json` 中的绝对 `JOB`：

```bash
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" test "$JOB"
```

新一轮运行必须使用新的输出目录，避免覆盖历史 index：

```bash
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" run \
  --output "$AP_PNC_DIR/.artifacts/benchmark/tracking-v6-repeat-clean"
```

该命令依赖已准备的 native stack 镜像、当前 ABI solver 与获授权的本地 profile，不是无需依赖即可执行的安装命令。原生 AMD64、MATLAB/Octave golden、硬件/飞行均未由此报告验收。

## 9. 未覆盖项与下一步

1. **冻结可复现基线与验收阈值。** 在洁净提交上重跑；明确定义位置/速度精度、硬限速或软限速容差、末端行为。
2. **完成 frame 与 reference 全工况审查。** MINCO/gcopter 旧混-frame cost 尚未迁移；补 hover、reverse、guard 边界、过渡与电机动态脉冲。单轨迹的 yb 连续不能代替这些验收。
3. **解释 phi 持续误差与 actuator 行为。** 分离 aero/actuator 分项力矩、PID/antiwindup、饱和和 QP 残差；不能以 RMSE 排序确认根因或模型真实性。
4. **扩展科学窗口。** 明确最后预测尾段的执行策略，再测末端、长时间、多轨迹及 wind contract；当前 estimator/predictor 的风语义尚未统一。
5. **另做实时与硬件门槛。** 实测全周期延迟与负载/队列，随后按安全流程逐层验收，不能由离线 RRD 推出可飞。

早期旧参考下的 status4 失败保留为历史诊断，不与本次完整窗口的 RMSE 混排。更高速度失败也不改写成“6 m/s 测试失败”。本轮通过不代表其历史根因已被独立解释。
