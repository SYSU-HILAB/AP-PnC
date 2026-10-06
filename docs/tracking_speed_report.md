# 6/8/10/12 m/s：完整尾段与末点保持测试

## 结论与建议

**完成性门槛：6 m/s 五模型全部完成；8 m/s 四模型完成、Lyu 失败；10 与 12 m/s 五模型均失败。** 当前证据支持保留 6 m/s 作为共同仿真基线，不支持把 8 m/s 当作全模型通过，更不支持把 10/12 m/s 当作可用控制范围。没有由此进行硬件或飞行验收。

末端处理已改成用户指定的**原样重复最后一个 reference 点**，不再外推多项式，也不再提前扣除 1 s NMPC horizon。所有自动运行包括完整轨迹和 **2 s 末端保持**，按 20 ms 周期向上取整。新 6 m/s 全部完成 20.50 s；8 m/s 成功模型完成 17.08 s。

优先诊断 8 m/s Lyu 的角速度跟踪与执行器响应，再采集实际 NLP/QP 残差和约束违例。失败原因目前确认到 `NMPC solve failed, status=4`，不把该错误码当作物理根因。

## 1. 实现与日志语义

- ROS `ReferenceBuffer` 本来就会把超出轨迹的 horizon point 设置为末点；离线路径现在与它一致。
- Planner-owned `ReferenceTrajectory::sample(t)` 对有限、非负的 t 使用 `min(t,T)`，p/v/a/yb/omega/rotation/thrust/fallback 全部重复末点。负数、NaN 和空轨迹仍拒绝。
- 重复的是最后一个点，而不是另造一个姿态或把 yb 取负。当前轨迹末点 v/a 约为零；由末端 jerk 算出的 planned omega 不必为零，不能冒充 active solver rate reference。现有 solver state/command rate reference 仍为零，hover command reference 为 9.81 N/kg。
- `reference_time_ns` 保留真正的预测时间 `control_time + stage*dt`；新 `sample_time_ns` 记录实际 clamp 后的源时间。不能把两者混成同一个时钟。
- 保存的 reference 显式声明 `terminal_policy: hold_last_point`。独立 Rust decoder 仅在该声明下允许末点扩展；旧数据仍按原 coverage 规则核验。
- `duration_s=0` 自动执行整个轨迹加 `terminal_hold_s`；本轮设为 2.0 s。显式短 duration 仍按完整周期向下取整。

## 2. 匹配条件与范围

原生 Linux ARM64 Docker；同一 C++ 数值 executable、四电机 plant profile、solver bundle/生成 OCP、controller 权重/时序，零风、无随机过程。每个速度组的初态相同；不同速度的初始角速度由各自 reference 的 flatness 派生，不声称它们相同。物理步长 1 ms，控制周期 20 ms，NMPC horizon 为 1 s。仅改变规划 `v_max` 和 plant aero model，不进行逐模型重调。

每个速度组的五模型保存了完全相同的 reference；不同速度之间重新规划，轨迹时长和多项式不同，不能当作同一时间函数比较。

| v_max 设置 m/s | 保存参考峰值 m/s | 参考时长 s | 目标结束时间 s | 完成/总数 |
|---:|---:|---:|---:|---:|
| 6 | 6.002007 | 18.484555 | 20.50 | 5/5 |
| 8 | 8.000440 | 15.06539 | 17.08 | 4/5 |
| 10 | 10.000316 | 13.26661 | 15.28 | 0/5 |
| 12 | 12.000213 | 12.376852 | 14.38 | 0/5 |

规划设置仍为软约束；参考和实际采样峰值都不构成硬限速保证。

## 3. 只对完整成功窗口报告性能

| 设置 m/s | 模型 | 位置 RMSE m | 速度 RMSE m/s | 最大位置误差 m | 实际采样速度峰值 m/s |
|---:|---|---:|---:|---:|---:|
| 6 | none | 0.018866 | 0.024338 | 0.063870 | 6.033274 |
| 6 | lyu | 0.055676 | 0.031542 | 0.083836 | 6.010737 |
| 6 | phi | 0.171150 | 0.054943 | 0.216972 | 6.007318 |
| 6 | ma | 0.017839 | 0.020179 | 0.058015 | 6.025513 |
| 6 | advanced | 0.026600 | 0.035272 | 0.103006 | 6.010144 |
| 8 | none | 0.047556 | 0.060365 | 0.117555 | 8.016491 |
| 8 | phi | 0.347318 | 0.157921 | 0.505485 | 8.118575 |
| 8 | ma | 0.038066 | 0.041216 | 0.081682 | 8.003462 |
| 8 | advanced | 0.070936 | 0.100505 | 0.234032 | 7.989040 |

这 9 个运行的 solver status 均为 0，status2 为 0，均通过独立 RRD decoder 验证。实际峰值计入最终状态，但仍只是采样峰值。

新 6 m/s RMSE 与旧 17.48 s 窗口不能直接称为“性能改善”：新指标包含减速尾段和容易跟踪的 2 s 保持期。8 m/s phi 的误差明显较大，但不能据此判定气动模型真假或确认偏差根因。没有预先冻结的误差容差，完成性不代替精度规格。

## 4. 失败窗口单独列示，不混排 RMSE

所有失败保留原始 manifest/CSV/reference/日志，均无成功 replay/verified 标记。下表是最后 committed time；失败 solve 的参考和状态也保留在 horizon CSV。

| 设置 m/s | 模型 | 失败/停止时间 s | committed ticks | solver status |
|---:|---|---:|---:|---:|
| 8 | lyu | 4.02 | 201 | 4 |
| 10 | none | 14.92 | 746 | 4 |
| 10 | lyu | 2.98 | 149 | 4 |
| 10 | phi | 9.76 | 488 | 4 |
| 10 | ma | 5.08 | 254 | 4 |
| 10 | advanced | 6.18 | 309 | 4 |
| 12 | none | 4.56 | 228 | 4 |
| 12 | lyu | 4.80 | 240 | 4 |
| 12 | phi | 4.44 | 222 | 4 |
| 12 | ma | 4.24 | 212 | 4 |
| 12 | advanced | 4.96 | 248 | 4 |

**重要：求解状态 0 不等于闭环安全。** 10 m/s none 从约 2.92 s 起位置误差已超过 1 m，远早于参考末端 13.2666 s；后续失控，采样速度达到约 63.79 m/s，失败段最大位置误差约 463 m。不能把它在 14.92 s 才报错解释为“接近全程成功”。1 m 在这里是诊断标记，不是预先制定的验收阈值。

8 m/s Lyu 在任何末点 clamp 生效前失败。因此现有证据不支持把这些失败统一归因于末端保持逻辑。10 m/s none 在早期无 aero 的条件下也已发散；不能把整个高速度问题只归咎于 Lyu 气动力。

### 已确认与未确认

- 8 m/s Lyu 实际 body pitch rate（包含最后 committed state）最大约 **18.413 rad/s**，该轴 command 最大绝对值约 **2.5 rad/s**。这说明实际响应与参考命令差异很大，不证明单一物理根因。
- 10 m/s none 某些输出 rate command 略超生成 OCP 名义边界：x 约 1.523、z 约 1.572 rad/s。不能声称全部 command 严格满足 bounds；须连同 residual/constraint violation 审计。
- 已确认停止错误码与控制/plant 失配现象；尚未确认是哪项 PID/饱和/gyro/耦合/预测模型/优化残差触发失效。
- 本轮没有重调响应 tau、PID、weights 或输入限制来掩盖失败。

## 5. 末点保持验收

9 个成功运行都跨过真实轨迹末端并保持至少 2 s。每个成功运行有 **1375 行**越界 horizon reference；这些行的 p/v/a/omega/yb/thrust/fallback 原样重复同一末点，逐字段值完全一致。实际源时间 clamp 到 T，预测时间仍正常递增；不是暂停仿真时钟。

- 6 m/s：最终位置误差 0.000345–0.000486 m，最终速度不超过 0.001018 m/s。
- 8 m/s 成功模型：最终位置误差 0.001185–0.001338 m，最终速度不超过 0.002588 m/s。
- 末点 v/a 的绝对值均小于 `1e-9`。
- 失败运行的部分持有数据不被包装成悬停成功。

重复了 8 m/s Lyu、10 m/s none、12 m/s Lyu：失败时刻/tick 完全复现，科学 CSV 除 wall-time 外逐字段相同，整个实际 NMPC reference CSV 字节相同。不是偶发一次的结果。

## 6. 产物与复核

全部在绝对项目根下：

```text
.artifacts/benchmark/tracking-speed-hold/
  build.log, ffi.log, cli-tests.log, acceptance.log
  audit.json, repeat-audit.json, matched-config-audit.json
  legacy-decoder-check.log
  v6/, v8/, v10/, v12/
    index.json, summary.json, figures/, <model>.log
```

每组 `index.json` 指向实际 JOB 和科学 run；每模型都有 tracking/path/diagnostics/实际 NMPC reference/horizon 图，另有 comparison 图。**失败图仅为 partial diagnostics**，不属于完整性能或成功回放。没有上传私有模型源码、参数或全量日志。

检查结果：C++ 4/4 CTest；Rust 5 个 bin test 与 6 个 FFI test，fmt/clippy；70 CLI tests；project lint/Ruff/shell/diff。坏数据验收覆盖错误的 clamped sample time；旧 6 m/s 数据仍通过新 decoder 的严格检查。

复现新组时使用新输出目录，防止覆盖：

```bash
uv run --project "$AP_PNC_DIR" python \
  "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/evaluate.py" run \
  --v-max 8 --hold-seconds 2 \
  --output "$AP_PNC_DIR/.artifacts/benchmark/NEW_V8_HOLD"
```

报告和结果来自 `ded0098 + dirty` 工作树的 native build，没有伪称洁净检出或 native AMD64 验收。Solver ABI/layout 未变，沿用已验收 bundle；数值 executable 已实际重建/重链。原始失败状态未被改为 finished。

## 7. 下一步

1. 以可重复的 8 m/s Lyu 为最小失效案例，加入真实 QP/NLP residual、constraint violation、iter/status 和一阶预测/实际响应对照。
2. 分离执行器与气动力矩、PID integral/derivative/antiwindup、饱和、gyro/惯量耦合；用 axis pulse 与已知输入验证，不能只放大 weights。
3. 独立审查命令边界与有效性门槛，避免只有 status0 就发布失控命令；任何保护改变必须另做科学与 transaction 验收。
4. 完成旧 MINCO cost frame 迁移和 guard/reverse/transition，再继续提高速度。当前不把 10/12 m/s 用作安全工作范围。
