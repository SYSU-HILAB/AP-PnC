# FAST-LIO ROS2 Topic Processing Chains

## Input Topics

```
Topic: /livox/lidar (type: livox_ros_driver2/msg/CustomMsg)
  └─> livox_pcl_cbk() [laserMapping.cpp:311]
       └─> p_pre->process(msg, ptr) [preprocess.cpp:47]
            └─> avia_handler(msg) [preprocess.cpp:95]
                 └─> pl_surf (PointCloudXYZI, in LiDAR frame)
                      └─> lidar_buffer.push_back(ptr) [line 342]
                           └─> time_buffer.push_back(last_timestamp_lidar) [line 343]

Topic: /livox/imu (type: sensor_msgs/msg/Imu)
  └─> imu_cbk() [laserMapping.cpp:350]
       └─> Apply time offset correction [line 357]
       └> imu_buffer.push_back(msg) [line 376]
```

---

## Processing Pipeline (timer_callback, 100Hz)

```
Step 1: sync_packages(Measures) [laserMapping.cpp:960]
  ├─> Pop lidar_buffer.front() → Measures.lidar
 ├─> Compute lidar_end_time from point timestamps
 └─> Pop imu_buffer until imu_time ≥ lidar_end_time → Measures.imu
  Output: MeasureGroup Measures (sync'ed LiDAR + IMU)

Step 2: p_imu->Process(Measures, kf, feats_undistort) [line 979]
  ├─> IMU preintegration (between lidar start and end)
 ├─> Motion distortion removal
 └─> Output: feats_undistort (motion-compensated points in LiDAR frame)

Step 3: lasermap_fov_segment() [line 992]
 └─> Update LocalMap_Points bounding box based on lidar position
  (map management, no point modification)

Step 4: Downsample feats_undistort → feats_down_body [lines 995-996]
 └─> VoxelGrid filter (leaf_size = filter_size_surf_min, default 0.5m)
  Output: feats_down_body (LiDAR frame, ~2000-5000 points)

Step 5: Transform to world frame [lines 1006-1011, also in map_incremental:447]
  For each point in feats_down_body:
   pointBodyToWorld() [laserMapping.cpp:182]:
    p_world = state_point.rot *
              (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
              state_point.pos
  Output: feats_down_world (world frame, "camera_init")
```

---

## Output Topics

### 1. `/cloud_registered` (sensor_msgs/msg/PointCloud2)
**Publisher**: `pubLaserCloudFull_` [line 930]
**Publish function**: `publish_frame_world()` [line 489]
**Trigger**: `if (scan_pub_en)` [line 1073]
**Frequency**: 10 Hz (timer period 100ms, published every scan)

**Data source** [line 493]:
```cpp
PointCloudXYZI::Ptr laserCloudFullRes(
    dense_pub_en ? feats_undistort : feats_down_body);
```
- If `dense_pub_en=true`: all motion-compensated points (feats_undistort)
- If `dense_pub_en=false`: downsampled points only (feats_down_body)

**Transformation** [lines 500-502]:
```cpp
for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
}
```
Same transform as before: `p_world = state.rot * (offset_R_L_I * p_body + offset_T_L_I) + state.pos`

**Frame ID**: `"camera_init"` (world frame)
**Timestamp**: `lidar_end_time`

**Purpose**: Full-resolution registered scan for visualization (RViz)

---

### 2. `/cloud_registered_body` (sensor_msgs/msg/PointCloud2)
**Publisher**: `pubLaserCloudFull_body_` [line 931]
**Publish function**: `publish_frame_body()` [line 546]
**Trigger**: `if (scan_pub_en && scan_body_pub_en)` [line 1074]
**Frequency**: 10 Hz

**Data source**: `feats_undistort` (motion-compensated, LiDAR frame)

**Transformation** [lines 553-554]:
```cpp
RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
```
Which does [line 218]:
```cpp
V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);
```
Only applies **extrinsic transform** (LiDAR → IMU body frame), NOT world rotation/translation.

**Frame ID**: `"body"` (IMU body frame)
**Timestamp**: `lidar_end_time`

**Purpose**: Debug - see points in IMU body frame (before EKF state transform)

---

### 3. `/cloud_effected` (sensor_msgs/msg/PointCloud2)
**Publisher**: `pubLaserCloudEffect_` [line 932]
**Publish function**: `publish_effect_world()` [line 565]
**Trigger**: `if (effect_pub_en)` [line 1075]
**Frequency**: 10 Hz (when enabled)

**Data source**: `laserCloudOri` [line 569-573]
- `laserCloudOri` is filled in `h_share_model()` [line 740]
- Contains only **effective feature points** that passed plane quality check
- These are the points used in EKF update (typically 100-1000 per scan)

**Transformation**: Same world transform [line 571-572]

**Frame ID**: `"camera_init"`
**Timestamp**: `lidar_end_time`

**Purpose**: Debug - visualize which points contribute to pose estimation

---

### 4. `/Laser_map` (sensor_msgs/msg/PointCloud2)
**Publisher**: `pubLaserCloudMap_` [line 933]
**Publish function**: `publish_map()` [line 581]
**Trigger**: `if (map_pub_en)` [line 1076] + also called by timer [line 1112]
**Frequency**: 1 Hz (map_publish_callback timer)

**Data source**: `pcl_wait_pub` [line 593]
- `pcl_wait_pub` accumulates points from `map_incremental()` [line 482]
- `map_incremental()` adds `feats_down_world` points to map every scan
- Map grows over time (sliding window, not full history)

**Transformation**: Points already in world frame when added to map.

**Frame ID**: `"camera_init"`
**Timestamp**: `lidar_end_time` (of latest added points)

**Purpose**: Visualize global map (downsampled accumulation)

---

### 5. `/Odometry` (nav_msgs/msg/Odometry)
**Publisher**: `pubOdomAftMapped_` [line 934]
**Publish function**: `publish_odometry()` [line 628]
**Trigger**: Called every scan [line 1064]
**Frequency**: 10 Hz

**Data source**: `state_point` (EKF state after update) + covariance from KF

**Content** [lines 633-645]:
- `pose.position` = `state_point.pos` (world position of IMU)
- `pose.orientation` = `state_point.rot` (world rotation of IMU as quaternion)
- `pose.covariance` = reordered P matrix (6×6 position+orientation covariance)

**Frame IDs**:
- `header.frame_id = "camera_init"` (world)
- `child_frame_id = "body"` (IMU frame)

**Additional**: Also broadcasts TF transform [lines 647-658]

**Purpose**: Real-time pose estimate for navigation/stacking

---

### 6. `/path` (nav_msgs/msg/Path)
**Publisher**: `pubPath_` [line 935]
**Publish function**: `publish_path()` [line 661]
**Trigger**: `if (path_en)` [line 1072]
**Frequency**: 10 Hz (but only publishes every 10th scan [line 670-673])

**Data source**: `path.poses` (global vector accumulating all poses)

**Content** [lines 663-674]:
- `msg_body_pose` = current `state_point.pos` + `state_point.rot`
- Appended to `path` vector every 10 frames
- `path.header.frame_id = "camera_init"`

**Purpose**: Full trajectory visualization (sparse due to downsampling)

---

## TF Broadcasting

**TransformBroadcaster** [line 936] publishes:
```
camera_init → body
```

Every odometry publish [lines 647-658]:
```cpp
trans.header.frame_id = "camera_init";
trans.child_frame_id = "body";
trans.transform.translation = {pos.x, pos.y, pos.z};
trans.transform.rotation = {quat.x, quat.y, quat.z, quat.w};
```

**Timestamp**: `lidar_end_time`

---

## Topic Summary Table

| Topic | Type | Frame | Data Source | Key Transform | Purpose |
|-------|------|-------|-------------|---------------|---------|
| `/livox/lidar` | CustomMsg | `livox` | Sensor raw | - | Input |
| `/livox/imu` | Imu | `livox` | Sensor raw | - | Input |
| `/cloud_registered` | PointCloud2 | `camera_init` | feats_undistort/downsampled | LiDAR→IMU→World | Visualization |
| `/cloud_registered_body` | PointCloud2 | `body` | feats_undistort | LiDAR→IMU only | Debug |
| `/cloud_effected` | PointCloud2 | `camera_init` | laserCloudOri (effective points) | LiDAR→IMU→World | Debug |
| `/Laser_map` | PointCloud2 | `camera_init` | pcl_wait_pub (accumulated) | Already world | Map |
| `/Odometry` | Odometry | `camera_init`→`body` | state_point | - | Navigation |
| `/path` | Path | `camera_init` | path vector | - | Trajectory |

---

## Configuration Flags (YAML)

```yaml
publish:
  path_en: true           # Enable /path
  scan_publish_en: true   # Enable /cloud_registered
  dense_publish_en: true  # Full resolution vs downsampled
  scan_bodyframe_pub_en: true  # Enable /cloud_registered_body

mapping:
  extrinsic_est_en: true  # If true, extrinsics optimized online
  extrinsic_T: [0.04165, 0.02326, -0.0284]
  extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
```

---

## Critical Dependencies

**All world-frame topics depend on**:
1. `state_point.rot` (EKF orientation estimate)
2. `state_point.pos` (EKF position estimate)
3. `state_point.offset_R_L_I` (extrinsic rotation)
4. `state_point.offset_T_L_I` (extrinsic translation)

If any of these are wrong (especially extrinsics), points project incorrectly → **no effective points** → odometry diverges.

**All body-frame topics depend on**:
1. Only extrinsics (`offset_R_L_I`, `offset_T_L_I`)
2. Independent of EKF state (good for debugging extrinsic errors)

---

## Timing & Synchronization

**Timer**: 100 Hz main loop [line 939-940]
- Calls `timer_callback()` which processes one lidar scan if synced

**Map publish timer**: 1 Hz [line 942-943]
- Separate timer for `/Laser_map`

**Time synchronization**:
- `time_sync_en=false`: Uses hardware timestamps with `time_offset_lidar_to_imu`
- `time_sync_en=true`: Auto-detects offset [lines 333-338]
- IMU messages get timestamp adjusted [line 357]

---

## Debug Publishing (if enabled)

In `laserMapping.cpp:1034-1040`, if `if(1)`:
```cpp
ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
featsFromMap->clear();
featsFromMap->points = ikdtree.PCL_Storage;
```
This dumps entire map KD-tree contents for debugging.

---

## Evidence for Extrinsic Error Diagnosis

**Check**: If `/cloud_registered` shows points in wrong location but `/cloud_registered_body` looks correct (LiDAR shape intact), then:
- Extrinsics `offset_R_L_I` / `offset_T_L_I` are **correct** (body frame is right)
- EKF state `state_point.rot` / `state_point.pos` are **wrong** (world transform broken)

**If both are wrong**: Extrinsics themselves are incorrect (points already distorted in body frame).

**Quick test**:
```bash
# 1. Enable body frame publishing
ros2 param set /laser_mapping publish.scan_bodyframe_pub_en true

# 2. Visualize both:
rviz2
# Add PointCloud2:
#   Topic: /cloud_registered        (world frame)
#   Topic: /cloud_registered_body   (body frame)
```

Compare shapes:
- Correct: Both show similar point cloud shape, just different pose
- Wrong extrinsics: Body frame cloud is distorted/offset from expected LiDAR shape
