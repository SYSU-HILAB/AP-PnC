# `time_offset_lidar_to_imu`: Meaning & Function

## Definition

**`time_offset_lidar_to_imu`**: A **scalar time offset** (in seconds) that aligns IMU timestamps with LiDAR timestamps.

- **Positive value**: IMU timestamps are **LATER** than LiDAR by this amount
- **Negative value**: IMU timestamps are **EARLIER** than LiDAR by this amount
- **Zero**: IMU and LiDAR timestamps are perfectly synchronized

**Location in config**:
```yaml
common:
  time_offset_lidar_to_imu: 0.0  # seconds
```

---

## Why This Offset Exists

### Physical Reality:
LiDAR and IMU are **separate sensors** with:
1. Independent clocks (crystal oscillators)
2. Different signal chain delays
3. Different onboard timestamp generation

**Result**: Even if both sensors trigger simultaneously, their **reported timestamps may differ by a constant offset**.

### Example:
```
True event time:      T = 1000.000 seconds
LiDAR reports:        t_lidar = 1000.001  (1ms ahead)
IMU reports:          t_imu   =  999.998  (2ms behind)

Offset: t_imu - t_lidar = -0.003 seconds
Need to add +0.003 to IMU timestamps to align with LiDAR
```

---

## Where It's Used in Code

### Location 1: IMU callback (`imu_cbk`) - **PRIMARY USE**

**File**: `src/laserMapping.cpp:350-379`

```cpp
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // LINE 357: Apply manual time offset
    msg->header.stamp = get_ros_time(
        get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu
    );

    // LINE 358-362: OR apply auto-detected offset (if time_sync_en=true)
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
            rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);
    // ... push to imu_buffer
}
```

**Function**:
1. Takes incoming IMU message with original timestamp `t_imu_original`
2. Subtracts `time_diff_lidar_to_imu`: `t_imu_corrected = t_imu_original - time_offset`
3. Stores corrected IMU in `imu_buffer`

**Note**: The subtraction means:
- If `time_offset_lidar_to_imu = +0.01` (IMU is 10ms ahead), subtract 10ms → makes it earlier
- If `time_offset_lidar_to_imu = -0.01` (IMU is 10ms behind), subtract (-10ms) → adds 10ms → makes it later

This aligns IMU timestamps to the **same time base as LiDAR**.

---

## How Sync Works After Offset Applied

### Step 1: Buffering
```
imu_buffer: [imu_msg(t1), imu_msg(t2), ...]   ← timestamps already shifted
lidar_buffer: [lidar_msg(T1), lidar_msg(T2), ...]
time_buffer:  [T1_beg, T2_beg, ...]
```

### Step 2: `sync_packages(MeasureGroup &meas)` - lines 383-435

```cpp
bool sync_packages(MeasureGroup &meas)
{
    // 1. Pop one LiDAR scan
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();   // e.g., 1000.5
    lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;  // e.g., 1000.6

    // 2. Find IMU data covering [lidar_beg_time, lidar_end_time]
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    }

    // 3. Return true if we have IMU data covering the scan duration
}
```

**Critical requirement**: `imu_buffer` timestamps must overlap with `[lidar_beg_time, lidar_end_time]`.

If `time_offset_lidar_to_imu` is wrong:
- IMU timestamps shifted too much → no overlap
- `sync_packages` returns `false` (wait for more data)
- Eventually: **"Too few input point cloud!"** or no processing

---

## Two Synchronization Modes

### Mode A: Manual Offset (`time_sync_en = false`, default)

```
imu_cbk line 357 applies offset:
  t_imu_corrected = t_imu_raw - time_offset_lidar_to_imu

Config YAML:
  common:
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0  # ← YOU SET THIS MANUALLY

How to get correct value:
  1. Use calibration tool (e.g., livox_camera_lidar_calib)
  2. Or manually tune until sync_packages succeeds consistently
```

### Mode B: Auto-detection (`time_sync_en = true`)

```
Triggered in livox_pcl_cbk() lines 333-338:

if (time_sync_en && !timediff_set_flg &&
    abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
    !imu_buffer.empty())
{
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
}

Logic:
  - Waits until IMU and LiDAR timestamps differ by > 1 second
  - Assumes IMU is delayed by 0.1s relative to LiDAR arrival
  - Computes: timediff = t_lidar + 0.1 - t_imu
  - Stores in global variable timediff_lidar_wrt_imu

Then in imu_cbk lines 358-362:
  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
  {
      msg->header.stamp = rclcpp::Time(
          timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp)
      );
  }

This OVERWRITES the manual offset adjustment!
```

**Priority**:
```
if (time_sync_en && |auto_offset| > 0.1):
    use auto_offset  ← takes precedence
else:
    use manual time_offset_lidar_to_imu
```

---

## Practical Example

### Scenario: Livox Avia + IMU (Horizon)

**Hardware setup**:
- LiDAR and IMU connected to same computer
- LiDAR driver timestamps at driver level (slightly delayed)
- IMU timestamps at sensor output (accurate)

**Observed**:
- LiDAR timestamp: `1000.005` (actual event was at 1000.000)
- IMU timestamp: `1000.002` (actual event was at 1000.000)

**Offsets**:
- LiDAR is **+5ms ahead** (reports later than actual)
- IMU is **+2ms ahead** (reports later than actual)
- Relative: IMU appears **3ms earlier** than LiDAR

**Correct offset**:
```
Want: t_imu_corrected to align with t_lidar timeline
t_imu_corrected = t_imu_raw - time_offset

We need: t_imu_corrected ≈ t_lidar
If t_imu_raw = 1000.002, t_lidar = 1000.005
Then: 1000.002 - offset = 1000.005
       offset = -0.003

So: time_offset_lidar_to_imu = -0.003 (IMU 3ms behind LiDAR)
```

**What happens if wrong**:

Case 1: `time_offset_lidar_to_imu = 0.0`
```
t_imu_corrected = 1000.002 - 0.0 = 1000.002
t_lidar_beg = 1000.005
IMU buffer has data at 1000.002, 1000.004, 1000.006...
sync_packages:
  Needs IMU covering [1000.005, 1000.006]
  First IMU ≤ 1000.005 is at 1000.004 ✓
  But last IMU needed is at 1000.006 ✓
  Actually might still work if IMU rate high enough...
  BUT: IMU integration interval is wrong!
  IMU propagates from 1000.002 to 1000.006 but thinks LiDAR is at 1000.005
  → Motion distortion removal uses WRONG delta t
  → Points severely distorted
```

Case 2: `time_offset_lidar_to_imu = +0.01` (IMU 10ms ahead)
```
t_imu_corrected = 1000.002 - 0.01 = 999.992
t_lidar_beg = 1000.005
imu_buffer timestamps: 999.992, 999.996, 1000.000, ...
sync_packages:
  Looking for IMU ≥ 1000.005
  Last IMU in buffer might be 1000.000
  No IMU data after lidar_beg_time! → returns false
  Result: "Too few input point cloud!" or no processing
```

---

## Interaction with `time_sync_en`

### Behavior Matrix:

| `time_sync_en` | `time_offset_lidar_to_imu` | What happens |
|----------------|----------------------------|--------------|
| `false` (default) | `0.0` | No correction applied |
| `false` | `±X` | Subtract X from IMU timestamps |
| `true` | any | Auto-detects after first scan, OVERRIDES manual value |

**Important**: Even with `time_sync_en: true`, the YAML value is still read (line 848) but only used if auto-detection fails or offset < 0.1.

---

## Calibration Methods

### Method 1: Use LI-Init (recommended)
```
Run LI-Init tool first → outputs calibrated offset
Set in YAML:
  common:
    time_offset_lidar_to_imu: 0.023  # example from LI-Init
```

### Method 2: Manual Tuning
```
Set time_sync_en: false
Start with 0.0
Watch console output:
  - "IMU and LiDAR not Synced" → offset too large
  - "Too few input point cloud!" → no IMU data for scan
  - Motion distortion visible in point cloud

Adjust incrementally ±0.001 until smooth.
```

### Method 3: Auto (`time_sync_en: true`)
```
Set:
  common:
    time_sync_en: true
    time_offset_lidar_to_imu: 0.0  # ignored after auto-detection

Algorithm auto-detects after first few scans.
Works if initial offset < 1 second.
```

---

## Symptoms of Wrong `time_offset_lidar_to_imu`

1. **"Too few input point cloud!"** (laserMapping.cpp:397)
   - `meas.lidar->points.size() <= 1`
   - Caused by: IMU buffer empty → sync_packages never succeeds

2. **Point cloud distortion** (smearing, streaks)
   - Motion distortion removal uses wrong time bounds
   - IMU integration interval doesn't match LiDAR scan duration

3. **"No Effective Points!"**
   - Distorted points project to wrong locations
   - Fail to match map → no inliers

4. **Inconsistent IMU readings** in `/cloud_registered_body`

5. **Diverging EKF** (poses jump, map corrupts)

---

## Debug Evidence

### Check 1: Print timestamps in callback

Add to `imu_cbk`:
```cpp
printf("IMU raw: %.6f, corrected: %.6f, offset: %.6f\n",
       get_time_sec(msg_in->header.stamp),
       get_time_sec(msg->header.stamp),
       time_diff_lidar_to_imu);
```

### Check 2: Print sync status

Add to `sync_packages`:
```cpp
printf("Lidar: [%.6f, %.6f], IMU first: %.6f, last: %.6f\n",
       meas.lidar_beg_time, lidar_end_time,
       get_time_sec(meas.imu.front()->header.stamp),
       get_time_sec(meas.imu.back()->header.stamp));
```

### Check 3: Time difference message

Already at line 330:
```cpp
if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0)
{
    printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
           last_timestamp_imu, last_timestamp_lidar);
}
```

If this prints → offset is large (>10s) or sensors not publishing.

---

## Summary: What It Does

```
FUNCTION: Aligns IMU timestamps to LiDAR timeline

MECHANISM:
  IMU_corrected_timestamp = IMU_raw_timestamp - time_offset_lidar_to_imu

PURPOSE:
  Ensure that in sync_packages():
    IMU data interval [t_imu_first, t_imu_last] overlaps with
    LiDAR scan interval   [t_lidar_beg, t_lidar_end]

RESULT IF CORRECT:
  - IMU and LiDAR data properly paired
  - Motion distortion accurately removed
  - Points project correctly → effective points found

RESULT IF WRONG:
  - No IMU data for scans → "Too few input point cloud!"
  - Or wrong motion compensation → distorted points
  - → effct_feat_num = 0 → "No Effective Points!"
```

---

## Quick Test

```bash
# 1. Check current offset
ros2 param get /laser_mapping common.time_offset_lidar_to_imu

# 2. Try zero first
ros2 param set /laser_mapping common.time_offset_lidar_to_imu 0.0

# 3. Enable auto-sync
ros2 param set /laser_mapping common.time_sync_en true

# 4. Watch for auto-detection message:
#   "Self sync IMU and LiDAR, time diff is X.XXXXXX"

# 5. If auto works, YAML can stay at 0.0 (auto overrides after detection)
```

---

## Key Code References

| Line | File | Purpose |
|------|------|---------|
| 357 | laserMapping.cpp | Apply manual offset: `t_imu - time_diff` |
| 358-362 | laserMapping.cpp | Apply auto offset (overrides manual) |
| 812, 848 | laserMapping.cpp | Declare and read parameter |
| 333-338 | laserMapping.cpp | Auto-detection trigger |
| 220-221 | IMU_Processing.hpp | Uses corrected IMU timestamps for undistortion |
| 393, 410 | laserMapping.cpp | `lidar_beg_time` and `lidar_end_time` from LiDAR |

---

## Bottom Line

**`time_offset_lidar_to_imu` is a timestamp correction for IMU messages** so they align with LiDAR's timeline. It's critical for:

1. **Data association**: Pairing the right IMU data with each LiDAR scan
2. **Motion compensation**: Accurate integration over correct time interval
3. **Extrinsic calibration validity**: Even perfect extrinsics fail if timing is wrong

If wrong → points get wrong motion compensation → transformed incorrectly → no effective points.

**Fix**: Set `time_sync_en: true` for auto-detection, or calibrate using LI-Init tool.
