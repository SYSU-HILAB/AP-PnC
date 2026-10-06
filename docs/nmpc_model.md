# NMPC：command 积分模型与变化率代价（ABI3）

## 本轮改动

参考只包含 **位置、速度、机体 Y 轴方向**。四轴 command 为
`c = [collective specific thrust, body rate x/y/z]`，优化输入为 `u = dc/dt`。

- `y = [p(3), v(3), yb(3), u(4)]`，`yref = [p_ref, v_ref, yb_ref, 0(4)]`。
- 终端只跟踪 `[p, v, yb]`，不包含 command 或 command 变化率目标。
- 阶段代价为 `0.5 * (e_p' Qp e_p + e_v' Qv e_v + e_yb' Qyb e_yb + u' R u)`。
- `dc/dt = u`：删除一阶 thrust/rate 滞后、时间常数和 lagged-thrust 估计。
- 不惩罚 command 偏离 hover 或零角速度。`initial_specific_thrust=9.81` 只用于初始化。
- 第 0 段也惩罚 `u0`。绝对 command 边界施加在积分后的状态，包含终端节点；
  不将原来的 N/kg、rad/s 边界错误地用作 N/kg/s、rad/s² 的变化率边界。

这里的正则项是**每秒变化率**。在固定控制周期下，单周期变化量为 `Δc = dt_control * u0`；
若以后改为直接对 `Δc` 加权，需要明确调整权重尺度，不能混用单位。

## 状态与执行

世界 ENU，机体 FLU，Hamilton 四元数 wxyz：

```
x = [p_W(3), v_W(3), body_rate_command(3), q_WB(4), specific_thrust_command, k_aero]
u = [d(specific_thrust_command)/dt, d(body_rate_command_xyz)/dt]
p = [cx_alpha_slope]
```

这是**理想内环跟踪**的预测模型：姿态运动学使用 rate command，平动使用 thrust command。
不是刚体力矩模型，也没有在预测中辨识或模拟实际电机/PID 的响应。
`cx * |v|²` 是保留的 **lumped body-X** 气动比力项（IMU 标定，含电机 body-X 分量）；
因为 `+X_B = +Z_L`，这个单轴就是机翼法向（升力）方向。其既有攻角导数近似/
系数演化仍需独立坐标与运动学核验；本轮已修正 dα/dt 缺失的 `−ω×v` 项。

每次求解使用测得的 p/v/q 与上一条 command。初始 command 可以通过 reset 显式设置。
求解接受且数值有限后：

```
c_next = c_previous + u0 / ctrl_frq
```

默认 50 Hz 即 **20 ms**，不发布导数 `u0` 本身，也不使用 **100 ms** 预测节点的 `x1`。
失败求解不推进 command memory。实际 plant 仍保留原有 rate loop 或四电机/PID/
混控/气动力/力矩/RK4；本轮没有改这些参数。

## YAML 与命名

时域、代价权重、command 边界及运行参数的配置来源是 `<AP_PNC_DIR>/core/bringup/config/nmpc.yaml`
（部署可使用根目录 `bringup/config/nmpc.yaml`）：

- 时域、节点间隔、控制频率、初始化 command 和气动系数演化参数；
- `objective.{position,velocity,body_y,command_derivative}`；
- `command_bounds.{lower,upper}`，顺序是 `[specific thrust, rate x, rate y, rate z]`。

`ap_pnc/nmpc_gen/config.py` 已删除；没有 `MpcConfig`、零权重角速度目标、
`drag_est`、`qDynamicVel`、`Metrics` 等未使用配置。生成器明确拒绝旧时间常数/
无效或未知调参键。YAML 权重、边界和时域编译进 solver，修改它们后仍必须重新生成并静态重链。
原有气动数值 guard、估计器 clip 与求解算法设置仍在实现中保留，不宣称本轮已迁移全部数值常数。

`model.py` 使用统一 snake_case 与显式坐标语义：
`rotation_matrix_from_quaternion` 返回 `R_WB`；`quaternion_kinematics_matrix`
返回 4×4 Hamilton 运动学矩阵，而不是把它含糊地称为 3×3 skew matrix。
状态、command 和 command derivative 分开命名。生成 C 符号前缀保留 `tailsitter_flu`。

## 验收边界

- ABI **3**：`NX/NU/NP = 15/4/1`；阶段/终端输出 `NY/NYN = 13/9`。
  runtime ABI 与生成 header 静态检查拒绝旧 ABI2 archive；ARM64 solver 已实际再生、重链。
- 符号模型/OCP 检查包括 hover、command Jacobian、1000 步积分、100 个四元数 Rdot case、
  无绝对 command 代价、第 0 段正则和终端 command 边界。
- 独立离线 decoder 支持旧 ABI2 文件，不重解释其 yb offset；新文件逐行校验
  `u0_derivative`、`published_command`、20 ms 积分以及 command memory 连续性。
- 真实 acados 短程回归仅 **none / lyu / ma**，2.0 kg 的 ideal-rate-loop plant，
  每模型 1 s / 50 ticks，并完成独立回放验证。这不证明四电机完整轨迹、高速跟踪或飞行有效。
- 之前的 6/8/10/12 m/s 报告是 **ABI2 历史证据**，不能作为 ABI3 的完整窗口验收。
  四电机 2.0 kg 的 6/8/10/12 m/s 测试与攻角/侧滑角/三轴气动力图现已完成：
  **11 组报 status 4，ma/6 跑满但位置 RMSE 18.98 m、最大误差 69.36 m**。
  详见 [新版速度报告](tracking_integrator_speed_report.md)。不测试 phi/advanced。

本地证据：`<AP_PNC_DIR>/.artifacts/benchmark/nmpc-command-integrator/`。
不发布 OSS，不包含实际飞行或硬件验收。
