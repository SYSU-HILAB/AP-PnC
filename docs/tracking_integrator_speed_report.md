# 新版 NMPC 速度测试：四电机、2.0 kg、ABI3

## 结论：这些速度档目前都不能作为可用控制范围

已完成用户要求的 **6 / 8 / 10 / 12 m/s** 测试，每档只运行 **none / lyu / ma**。
**11 组中途 solver status 4；唯一跑满的 6 m/s ma 也严重发散。**
它的位置 RMSE 为 **18.9776 m**，最大误差 **69.3574 m**。这不是控制通过。

这次是完整参考轨迹加末端 2 秒保持，不是 1 秒理想内环回归。
短程回归不能证明不同速度或四电机闭环有效。

下一步应先固定 2.0 kg，检查 none 模型的 command/实际角速度差、PID/混控与饱和。
因为 none 的气动力和气动力矩确实为零，但它也失败，不能把所有失效归因于气动模型。
后续独立核验已确认两处实现错误：NMPC 的攻角导数与当前攻角定义不一致；
ma 在更新当前攻角前就计算样条基函数。同一速度只改传入旧 alpha，实际静态库返回力相差 2.143128 N。
这些错误是否解释全部失效、安装坐标是否符合真实机翼、理想内环预测是否失配，仍需分别验证。
本报告曲线是原实现的真实运行输出，不是正确气动模型的性能验收。模型尚未修复或重跑。

## 条件与匹配性

- 新版 ABI3：command 积分预测，pos/vel/yb 跟踪，四轴 command 变化率正则。
- 使用原四电机 plant，无舵面；机翼气动力仍保留。
- 将 plant/planner/controller 的质量同步设为 **2.0 kg**。
- 私有原始 profile 和 provenance 不修改。惯量、电机标定、PID、权重、边界与步长均保持不变。
- 零风、无随机扰动、50 Hz 控制、同一实际 executable 和生成 OCP。
- 同一速度档的三模型具有完全相同的已保存参考；不同速度档重新规划。
- `v_max` 是规划器软约束设置，不代表实际速度上限。
- 末点原样重复，执行整个参考后保持 2 秒，并向上对齐 20 ms 周期。
- **不测试 phi / advanced**。未执行飞行、硬件或生产控制，也未发布 OSS。

审计验证：除质量外私有 profile 参数不变；12 组配置仅改变 aero model / v_max。
实际 command 积分、参考 yb、记录的气动速度坐标、攻角/侧滑角几何一致性均经过独立复核。

## 完整窗口结果

数字为最后成功提交的物理状态时刻，不是最后一个 before-state 的时间。
除 ma/6 外，均在该时刻的下一次求解报 status 4；失败 period 不提交。

| v_max 设置 | 目标结束 | none | lyu | ma |
|---:|---:|---:|---:|---:|
| 6 m/s | 20.50 s | 14.56 s 失败 | 4.94 s 失败 | 跑满，但严重发散 |
| 8 m/s | 17.08 s | 10.50 s 失败 | 16.52 s 失败 | 12.60 s 失败 |
| 10 m/s | 15.28 s | 6.46 s 失败 | 11.24 s 失败 | 8.28 s 失败 |
| 12 m/s | 14.38 s | 6.18 s 失败 | 8.96 s 失败 | 5.34 s 失败 |

失败日志和实际求解参考全部保留，不为失败组生成成功回放标记。
失败组 partial RMSE 不作为完整窗口性能排名。

### 跑满的 ma/6 也不能算跟踪有效

| 指标 | 值 |
|---|---:|
| ticks / samples | 1025 / 1026 |
| 位置 RMSE | 18.977593 m |
| 最大位置误差 | 69.357376 m |
| 速度 RMSE | 10.335156 m/s |
| 实际最大速度（包含最终状态） | 21.861083 m/s |
| solver status | 全 0 |

独立 RRD 验证成功只证明记录可解码且与科学数据一致。
**status 0、跑满、回放通过，都不能代替闭环跟踪验收。**

## 攻角、侧滑角、三轴气动力图

每张 `*-aerodynamic-history.png` 都包含：攻角 α、侧滑角 β、FLU 的 Fx/Fy/Fz，以及实际空速。

- α/β 直接来自实际 plant 气动模型返回值，不从 planner 或 NMPC 预测重建。
- 气动模块使用 wing FRD：`v_wing = (v_body_Z, -v_body_Y, v_body_X)`。
- α 通常为 `atan2(v_wing_z, v_wing_x)`；β 通常为 `asin(v_wing_y / |v_air|)`。
- 低速 guard 由各模型自己处理。灰区标出空速低于 0.5 m/s，低速角度不可过度解读。
- α 保留原始 wrap，±180° 跳变不能直接当成物理力跳变或代码错误。
- 力为 **N、body FLU、纯气动力**；不包含重力或转子推力，也不是 F/m。
- 曲线含最后 committed state，失败图明确标为 partial。
- 附带记录气动力矩，可在各速度档 `*-diagnostics.png` 查看。

本地所有图：

```
<AP_PNC_DIR>/.artifacts/benchmark/nmpc-integrator-speed-m2/
  v6/figures/{none,lyu,ma}-aerodynamic-history.png
  v8/figures/{none,lyu,ma}-aerodynamic-history.png
  v10/figures/{none,lyu,ma}-aerodynamic-history.png
  v12/figures/{none,lyu,ma}-aerodynamic-history.png
```

每档另有 tracking / diagnostics / actual-reference / horizon 图。
图只读取既有记录，不重跑 planner 或 plant。

## 可重复性与证据

复测 **6/lyu、6/ma、8/none**：失败/结束时刻和 ticks 一致。
除 `solve_time_ms` 外，科学 CSV 完全相同；实际求解参考、metrics、manifest 字节相同。

每档 `index.json` 包含准确 job/run 路径，不猜目录。证据根目录：

`<AP_PNC_DIR>/.artifacts/benchmark/nmpc-integrator-speed-m2/`

主要文件：`audit.json`、`repeat-audit.json`、各档 `summary.json` / `aerodynamic-summary.json`，
以及 build / CLI / 原始运行日志。源码仍为 working tree；未提交或推送。

历史 ABI2 报告保留原始结果，但控制模型与质量条件不同，不能把此次变化简单归因为单一算法或气动因素。
