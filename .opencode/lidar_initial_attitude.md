# 添加 LIDAR_INITIAL_ATTITUDE 配置参数

## 意图澄清

用户希望添加一个名为 `LIDAR_INITIAL_ATTITUDE` 的配置参数，用于校正激光雷达的初始姿态偏移。

**原始需求**：
- 添加 `LIDAR_INITIAL_ATTITUDE` 配置参数，类似于 `extrinsic_R` 的读取方式。
- 修改 `mid360.yaml` 中的相应字段。
- 仅在使用 `/livox/imu` 时生效。
- 将最终输出话题（如 Odometry）进行旋转：`R_LIDAR_INITIAL_ATTITUDE * Odometry`。
- 世界坐标系假设激光雷达初始姿态是 `[1,0,0,0]`（单位四元数），但实际可能存在偏移。

**澄清后的操作性意图**：
1. 在 `mapping` 配置部分添加 `lidar_initial_attitude` 参数（3x3 旋转矩阵）。
2. 在 `laserMapping.cpp` 中读取该参数。
3. 在发布 `/Odometry`、`/Path` 和 TF 变换时，应用旋转矩阵：将位置和姿态都进行旋转。
4. 条件判断：当 `imu_topic` 为 `/livox/imu` 时，应用此旋转；否则不应用（或可配置）。

## 可行性评估

**可行**。原因：
- 代码结构清晰，`extrinsic_R` 的读取和使用模式可以复用。
- 输出发布点（`publish_odometry`、`publish_path`）明确，易于修改。
- 旋转矩阵操作在 Eigen 中简单直接。

## 任务定义

### 父任务：添加 LIDAR_INITIAL_ATTITUDE 配置并应用到输出话题

**目标**：使 FAST-LIO2 能够通过配置文件指定激光雷达的初始姿态偏移，并在发布 Odometry、Path 和 TF 时自动应用该偏移。

**需要的交付物**：
1. 配置文件更新：在 `mid360.yaml`（及其他相关 YAML 文件）中添加 `lidar_initial_attitude` 参数。
2. 代码更新：在 `laserMapping.cpp` 中读取参数，并在输出发布时应用旋转。
3. 功能验证：确保旋转正确应用，且仅在指定条件下生效。

### 子任务

#### 子任务 1：更新配置文件

**交付物**：在 `mid360.yaml`（及其他 Livox 相关 YAML 文件）的 `mapping` 部分添加 `lidar_initial_attitude` 参数。

**依赖**：无。

**完成标志**：配置文件包含新参数，默认值为单位矩阵。

#### 子任务 2：在代码中读取参数

**交付物**：在 `laserMapping.cpp` 中添加参数声明、读取和解析逻辑。

**依赖**：子任务 1。

**完成标志**：代码能够从配置文件读取旋转矩阵，并存储为 `M3D` 类型。

#### 子任务 3：在输出发布时应用旋转

**交付物**：修改 `set_posestamp` 函数（或相关发布函数），在发布前应用旋转矩阵。

**依赖**：子任务 2。

**完成标志**：Odometry、Path 和 TF 输出的姿态和位置都经过旋转。

#### 子任务 4：添加条件判断（可选）

**交付物**：根据 `imu_topic` 是否为 `/livox/imu` 决定是否应用旋转。

**依赖**：子任务 2。

**完成标志**：仅当使用 `/livox/imu` 时应用旋转。

## 上下文

- **代码库**：FAST-LIO2 ROS2 版本。
- **关键文件**：
  - `config/mid360.yaml`：配置文件。
  - `src/laserMapping.cpp`：主逻辑文件。
  - `include/common_lib.h`：通用定义。
- **现有模式**：`extrinsic_R` 的读取和使用方式（第 835、871、896 行）。

## 约束

- 不修改核心算法（如 EKF 更新）。
- 仅修改输出发布部分。
- 保持向后兼容：默认值为单位矩阵，不影响现有行为。

## 规则

- 旋转矩阵应为 3x3，行优先存储（与 `extrinsic_R` 一致）。
- 旋转应用于世界坐标系下的位置和姿态。

## 验证

- [ ] 配置文件包含 `lidar_initial_attitude` 参数，默认值为单位矩阵。
- [ ] 代码能够读取该参数并打印日志。
- [ ] 当设置为非单位矩阵时，Odometry 输出的姿态和位置发生相应变化。
- [ ] 当 `imu_topic` 不是 `/livox/imu` 时，旋转不应用（如果实现条件判断）。

## 子任务详细说明

### 子任务 1：更新配置文件

**具体步骤**：
1. 在 `config/mid360.yaml` 的 `mapping` 部分添加：
   ```yaml
   lidar_initial_attitude: [1., 0., 0.,
                            0., 1., 0.,
                            0., 0., 1.]
   ```
2. 在其他 Livox 相关 YAML 文件（`avia.yaml`、`horizon.yaml`）中添加相同参数。

**交付物**：更新后的 YAML 文件。

### 子任务 2：在代码中读取参数

**具体步骤**：
1. 在 `laserMapping.cpp` 中添加变量声明：
   ```cpp
   vector<double> lidar_initial_attitude(9, 0.0);
   M3D lidar_initial_attitude_mat;
   ```
2. 在参数声明部分添加：
   ```cpp
   this->declare_parameter<vector<double>>("mapping.lidar_initial_attitude", vector<double>());
   ```
3. 在参数读取部分添加：
   ```cpp
   this->get_parameter_or<vector<double>>("mapping.lidar_initial_attitude", lidar_initial_attitude, vector<double>());
   ```
4. 解析为矩阵：
   ```cpp
   if (!lidar_initial_attitude.empty()) {
       lidar_initial_attitude_mat << MAT_FROM_ARRAY(lidar_initial_attitude);
   } else {
       lidar_initial_attitude_mat.setIdentity();
   }
   ```
5. 打印日志：
   ```cpp
   RCLCPP_INFO(this->get_logger(), "lidar_initial_attitude: %f %f %f %f %f %f %f %f %f",
               lidar_initial_attitude[0], lidar_initial_attitude[1], lidar_initial_attitude[2],
               lidar_initial_attitude[3], lidar_initial_attitude[4], lidar_initial_attitude[5],
               lidar_initial_attitude[6], lidar_initial_attitude[7], lidar_initial_attitude[8]);
   ```

**交付物**：更新后的 `laserMapping.cpp`。

### 子任务 3：在输出发布时应用旋转

**具体步骤**：
1. 修改 `set_posestamp` 函数，添加旋转矩阵参数：
   ```cpp
   template<typename T>
   void set_posestamp(T & out, const M3D &rot_mat)
   {
       V3D rotated_pos = rot_mat * state_point.pos;
       out.pose.position.x = rotated_pos(0);
       out.pose.position.y = rotated_pos(1);
       out.pose.position.z = rotated_pos(2);
       
       Eigen::Quaterniond rotated_quat(rot_mat * state_point.rot.toRotationMatrix());
       out.pose.orientation.x = rotated_quat.x();
       out.pose.orientation.y = rotated_quat.y();
       out.pose.orientation.z = rotated_quat.z();
       out.pose.orientation.w = rotated_quat.w();
   }
   ```
2. 在 `publish_odometry` 和 `publish_path` 中调用时传入 `lidar_initial_attitude_mat`。
3. 在 TF 广播时也应用旋转。

**交付物**：更新后的 `set_posestamp` 和发布函数。

### 子任务 4：添加条件判断（可选）

**具体步骤**：
1. 添加一个布尔变量 `apply_lidar_initial_attitude`，默认为 `false`。
2. 在读取 `imu_topic` 后判断：
   ```cpp
   bool apply_lidar_initial_attitude = (imu_topic == "/livox/imu");
   ```
3. 在应用旋转前检查该标志。

**交付物**：条件判断逻辑。

## 总结

**可行性**：可行。

**计划**：
1. 更新配置文件，添加 `lidar_initial_attitude` 参数。
2. 在代码中读取该参数。
3. 修改输出发布函数，应用旋转矩阵。
4. （可选）添加条件判断，仅在使用 `/livox/imu` 时应用。

**下一步**：执行子任务，按顺序完成配置更新、代码修改和测试。