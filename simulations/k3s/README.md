# K3s Simulation Orchestration for AP-PnC

基于 K3s `ConfigMap` 实现动态、声明式选择并启动 `simple-sim`（轻量级 LiDAR + 气动仿真）或 `gazebo`（stock PX4 v1.17.0 + Gazebo Harmonic SITL，mavros2 通用接口栈）。

---

## 架构设计

```
                    ┌─────────────────────────┐
                    │   ConfigMap: sim-config │
                    │   SIM_TYPE: simple-sim  │ (或 "gazebo" / "stop")
                    └────────────┬────────────┘
                                 │ Watch / Reconcile
                                 ▼
                    ┌─────────────────────────┐
                    │ Deployment:             │
                    │ sim-dispatcher          │
                    └──────┬────────────┬─────┘
          SIM_TYPE=simple-sim           SIM_TYPE=gazebo
          Scale: 1 / 0                  Scale: 0 / 1
                  │                              │
                  ▼                              ▼
      ┌──────────────────────┐       ┌──────────────────────┐
      │  ap-pnc-simple-sim   │       │    ap-pnc-gazebo     │
      │  (simple-sim:latest) │       │  (px4 + ap-pnc)      │
      └──────────────────────┘       └──────────────────────┘
```

1. **`sim-config` ConfigMap**：
   - 核心字段 `SIM_TYPE`：支持 `"simple-sim"`、`"gazebo"`、`"stop"`。
   - 内置包含 `simple_sim.yaml`，直接支持无人机质量、惯量、风阻、气动力学参数热配置与注入。
2. **`sim-dispatcher` 控制器**：
   - 轻量级监听 Pod，检测到 `SIM_TYPE` 发生变更时，自动调用 Kubernetes API 动态缩放对应 Deployment 的 `replicas`，实现零资源浪费、无缝热切换。
3. **`ap-pnc-simple-sim`**：
   - 运行 C++ 气动力学与激光雷达模拟器，开箱即用。
4. **`ap-pnc-gazebo`**（三容器 Pod，hostNetwork）：
   - `px4-simulator`：stock PX4 v1.17.0 + Gazebo Harmonic（`gz_x500`）。
   - `nav_infra`：基础服务网关容器（mavros2 + nmpc_bridge + flight_mux，
     sim profile；实机 profile 再加 livox + fast_lio + ekf_quat，镜像不变，
     `NAV_PROFILE` ConfigMap 键切换）。
   - `ap-pnc`：NMPC / 规划 / px4ctrl / flight_mux 控制栈。

---

## 快速使用

### 1. 部署到 K3s 集群

使用 `ap-pnc` CLI：
```bash
uv run ap-pnc k3s apply
```
或直接通过 `kubectl`：
```bash
kubectl apply -f simulations/k3s/all-in-one.yaml
```

### 2. 动态切换仿真器

通过 CLI 一键切换：
```bash
# 切换至 simple_sim
uv run ap-pnc k3s switch simple-sim

# 切换至 gazebo
uv run ap-pnc k3s switch gazebo

# 停止所有仿真实例
uv run ap-pnc k3s switch stop
```

或直接修改 ConfigMap：
```bash
kubectl patch cm sim-config -n sim -p '{"data":{"SIM_TYPE":"gazebo"}}'
```

### 3. 查看运行状态

```bash
uv run ap-pnc k3s status
```
输出示例：
```
Current ConfigMap SIM_TYPE: simple-sim

AP-PnC Workloads in namespace:
NAME                                 READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/ap-pnc-gazebo        0/0     0            0           1m
deployment.apps/ap-pnc-simple-sim    1/1     1            1           1m
deployment.apps/sim-dispatcher       1/1     1            1           1m

NAME                                      READY   STATUS    RESTARTS   AGE
pod/ap-pnc-simple-sim-748bc55959-4vxgm    1/1     Running   0          10s
pod/sim-dispatcher-65c5d4b574-x8r7k       1/1     Running   0          1m
```

### 4. 查看日志

```bash
# 实时跟踪当前活跃仿真的日志
uv run ap-pnc k3s logs
```

### 5. 清理资源

```bash
uv run ap-pnc k3s delete
```

## Bringup config mounting (core stack)

Core ROS2 nodes read their configuration directly from sectioned yaml files
(`bringup/config/planning.yaml`, `bringup/config/c_nmpc.yaml`) — no ROS
parameters. When the core stack is deployed into k3s, mount these via the
same `sim-config` ConfigMap pattern:

```yaml
volumeMounts:
  - mountPath: /workspace/bringup/config/planning.yaml
    name: bringup-config
    subPath: planning.yaml
    readOnly: true
volumes:
  - name: bringup-config
    configMap:
      name: sim-config
```

Nodes read the conventional cwd-relative path `bringup/config/*.yaml`,
which matches the `/workspace` workdir — no environment variables needed.
ConfigMap updates take effect on rollout restart.
