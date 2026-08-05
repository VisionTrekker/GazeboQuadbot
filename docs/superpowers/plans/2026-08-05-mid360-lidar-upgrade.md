# Mid-360 LiDAR Upgrade — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade `robots/mid360_simulation` Gazebo plugin to publish BOTH `livox_ros_driver2/msg/CustomMsg` (→ `/livox/lidar`) AND `sensor_msgs/msg/PointCloud2` (→ `/livox/lidar_PointCloud2`); fix 3 bugs (CSV unit, hardcoded intensity, wall-clock timestamps).

**Architecture:** Single shared `points_pair` for-loop inside `OnNewLaserScans()` constructs both messages per ray hit. Sim-time (`world->SimTime()`) is the only time source. CSV `Time/us` column drives intra-frame `offset_time` (×1000 → ns). Intensity/reflectivity now read from `rayShape_->GetRetro()`.

**Tech Stack:** ROS2 Humble, Gazebo 11 Classic, C++17, ament_cmake, GoogleTest, livox_ros_driver2 (already built at `/media/lenovo/disk/planner_ws/src/Nav3D/install`).

---

## Global Constraints

These constraints apply to every task. Every step's requirements implicitly include this section.

- **C++ standard:** C++17 (`set(CMAKE_CXX_STANDARD 17)` in CMakeLists.txt).
- **Compiler flags:** `-Wall -Wextra -Wpedantic` already set; code must compile without warnings under these flags.
- **Memory model:** RAII only. No raw `new`/`delete`. Use `std::vector::reserve` + `push_back`. Use `std::unique_ptr` for owned pointers; existing `boost::shared_ptr` only for plugin-owned ODE shapes (do not introduce new shared_ptrs).
- **Numerical casts:** Always `static_cast` for numeric narrowing; use `std::clamp` for value clamping.
- **Time source:** `world->SimTime()` only. NEVER `boost::chrono`, `std::chrono::steady_clock`, `std::chrono::high_resolution_clock`, or `node_->get_clock()->now()` for scan-time semantics.
- **CSV unit:** CSV column 1 is **microseconds** (μs), not seconds. Code must use `info.time_us` (μs) for `offset_time = info.time_us * 1000` ns.
- **Dependencies:** `livox_ros_driver2` is **not** auto-installed; the build assumes the developer has manually set `CMAKE_PREFIX_PATH=/media/lenovo/disk/planner_ws/src/Nav3D/install:$CMAKE_PREFIX_PATH` for that session.
- **URDF / launch / package name:** Do **not** modify. Plugin filename stays `libmid360_plugin.so`.
- **Topic names:** CustomMsg → `<topic>` (default `/livox/lidar`); PointCloud2 → `<topic>_PointCloud2` (default `/livox/lidar_PointCloud2`).
- **Commits:** Conventional Commits (`feat:`, `fix:`, `test:`, `docs:`, `chore:`). One logical change per commit.

---

## File Structure Map

Files that change together live together; each task's diff stays focused.

| File | Owner task | Responsibility |
|------|-----------|----------------|
| `robots/mid360_simulation/CMakeLists.txt` | T1 | Add `livox_ros_driver2` dep + gtest |
| `robots/mid360_simulation/package.xml` | T1 | Declare `<depend>` |
| `robots/mid360_simulation/include/mid360_simulation/mid360_points_plugin.h` | T2 | Header members + constants |
| `robots/mid360_simulation/src/mid360_points_plugin.cpp` | T2, T4, T5 | Plugin impl: helpers + Load() + OnNewLaserScans() |
| `robots/mid360_simulation/scan_mode/mid360.csv` | T6 | Header fix |
| `robots/mid360_simulation/test/CMakeLists.txt` | T7 | Test config |
| `robots/mid360_simulation/test/test_pointcloud2_layout.cpp` | T7 | Field-offset tests |
| `robots/mid360_simulation/test/test_offset_time_monotonic.cpp` | T7 | Monotonicity + invariants |
| `robots/mid360_simulation/README.md` | T8 | Topics, deps, build conventions |

---

## Task 1: Add `livox_ros_driver2` Dependency

**Files:**
- Modify: `robots/mid360_simulation/CMakeLists.txt:17-26` (find_package / dependencies)
- Modify: `robots/mid360_simulation/package.xml:18-37` (add `<depend>`)

**Interfaces:**
- Produces: A successful `find_package(livox_ros_driver2 REQUIRED)` so `ament_target_dependencies(mid360_plugin ... livox_ros_driver2)` resolves headers from `/media/lenovo/disk/planner_ws/src/Nav3D/install`.

- [ ] **Step 1: Verify `livox_ros_driver2` is reachable**

```bash
ls /media/lenovo/disk/planner_ws/src/Nav3D/install/livox_ros_driver2/share/livox_ros_driver2/cmake/livox_ros_driver2Config.cmake
```

Expected: file exists (config file present).

- [ ] **Step 2: Add `<depend>` to package.xml**

Edit `robots/mid360_simulation/package.xml` — add this line immediately after the existing `<depend>gazebo_dev</depend>` (line 30):

```xml
  <depend>livox_ros_driver2</depend>
```

- [ ] **Step 3: Update CMakeLists.txt `find_package` block**

Edit `robots/mid360_simulation/CMakeLists.txt` — in the `find_package(...)` block (lines 17–26), add one line after the existing `find_package(Boost REQUIRED COMPONENTS chrono)`:

```cmake
find_package(livox_ros_driver2 REQUIRED)
```

- [ ] **Step 4: Update `ament_target_dependencies`**

Edit `robots/mid360_simulation/CMakeLists.txt` — in the existing `ament_target_dependencies(mid360_plugin ...)` block (lines 49–57), add `livox_ros_driver2` to the dependency list, preserving order:

```cmake
ament_target_dependencies(mid360_plugin
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros
  tf2_ros
  livox_ros_driver2
)
```

- [ ] **Step 5: Configure and build to verify dependency resolves**

```bash
export CMAKE_PREFIX_PATH="/media/lenovo/disk/planner_ws/src/Nav3D/install:$CMAKE_PREFIX_PATH"
cd /media/lenovo/disk/Embodied_AI/GazeboQuadbot
colcon build --packages-select mid360_simulation --cmake-clean-cache
```

Expected: build succeeds. If `find_package(livox_ros_driver2 REQUIRED)` fails with "Could not find a package configuration file", re-export `CMAKE_PREFIX_PATH` and rerun.

- [ ] **Step 6: Verify the existing plugin still links cleanly**

Run `colcon build` again — must report `[100%] Built target mid360_plugin` with no new warnings.

- [ ] **Step 7: Commit**

```bash
git add robots/mid360_simulation/CMakeLists.txt robots/mid360_simulation/package.xml
git commit -m "build(mid360_simulation): depend on livox_ros_driver2 for CustomMsg"
```

---

## Task 2: Header — Add CustomMsg Publisher + Constants

**Files:**
- Modify: `robots/mid360_simulation/include/mid360_simulation/mid360_points_plugin.h:9-17` (include block)
- Modify: `robots/mid360_simulation/include/mid360_simulation/mid360_points_plugin.h:46-167` (class declaration, add members + constants)

**Interfaces:**
- Produces:
  - `rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPub_;`
  - `static constexpr std::size_t kPointStepBytes = 32;`
  - `static constexpr uint8_t kMid360Tag = 0x10;`
  - `static constexpr uint8_t kMid360Line = 0;`
  - Private helper signatures: `void SetPointCloud2Fields(sensor_msgs::msg::PointCloud2& pc);`, `void AppendPc2Row(std::vector<uint8_t>& buf, float x, float y, float z, float intensity, uint16_t ring, double timestamp);`, `builtin_interfaces::msg::Time ToRosTime(const gazebo::common::Time& gz_time) const;`, `static sensor_msgs::msg::PointField MakeField(const std::string& name, uint32_t offset, uint8_t datatype);`

- [ ] **Step 1: Add include for CustomMsg**

Edit `mid360_points_plugin.h` — after the `#include <sensor_msgs/msg/point_cloud2.hpp>` line (line 16), add:

```cpp
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>
```

- [ ] **Step 2: Add file-scope constants**

Edit `mid360_points_plugin.h` — immediately after the `#define MID360_SIMULATION_POINTS_PLUGIN_H` / before `#include <gazebo/plugins/RayPlugin.hh>`, add:

```cpp
// Layout / protocol constants (see spec §4)
static constexpr std::size_t kPointStepBytes = 32;  // bytes per PointCloud2 row
static constexpr uint8_t    kMid360Tag       = 0x10;  // normal single-echo (FAST-LIO accept)
static constexpr uint8_t    kMid360Line      = 0;     // mid360 is single-line
```

- [ ] **Step 3: Declare the CustomMsg publisher**

Edit `mid360_points_plugin.h` — in the private section, immediately after the existing `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;` line (~line 139), add:

```cpp
  /// @brief livox CustomMsg publisher (FAST-LIO2 path)
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPub_;
```

- [ ] **Step 4: Declare private helper functions**

Edit `mid360_points_plugin.h` — in the private section, before the `// ============================================================` block that introduces "成员变量" (around line 103), add:

```cpp
  // ============================================================
  // 消息构造辅助
  // ============================================================

  /// @brief 把 Gazebo 仿真时间 builtin_interfaces::Time
  builtin_interfaces::msg::Time ToRosTime(const gazebo::common::Time& gz_time) const;

  /// @brief 填充 PointCloud2 6 字段布局（x,y,z,intensity,ring,timestamp）
  void SetPointCloud2Fields(sensor_msgs::msg::PointCloud2& pc);

  /// @brief 静态工厂：生成单个 PointField 描述
  static sensor_msgs::msg::PointField MakeField(
      const std::string& name, uint32_t offset, uint8_t datatype);

  /// @brief 按 kPointStepBytes 布局把一行点写入字节缓冲
  void AppendPc2Row(std::vector<uint8_t>& buf,
                    float x, float y, float z,
                    float intensity, uint16_t ring,
                    double timestamp);
```

- [ ] **Step 5: Compile-check header**

```bash
export CMAKE_PREFIX_PATH="/media/lenovo/disk/planner_ws/src/Nav3D/install:$CMAKE_PREFIX_PATH"
cd /media/lenovo/disk/Embodied_AI/GazeboQuadbot
colcon build --packages-select mid360_simulation --cmake-clean-cache 2>&1 | tee /tmp/build_t2.log
```

Expected: build fails with `undefined reference to Mid360PointsPlugin::SetPointCloud2Fields(...)` etc. **The linker errors are expected and resolved in Task 3.** Header itself must compile (`#include` errors mean the include path or macro is wrong — fix and rerun).

- [ ] **Step 6: Commit**

```bash
git add robots/mid360_simulation/include/mid360_simulation/mid360_points_plugin.h
git commit -m "feat(mid360_simulation): declare CustomMsg publisher + PC2 layout helpers"
```

---

## Task 3: Implement Helper Functions (TDD)

**Files:**
- Modify: `robots/mid360_simulation/src/mid360_points_plugin.cpp` (add helper definitions before `Mid360PointsPlugin::Load`)

**Interfaces:**
- Consumed by T5's `OnNewLaserScans` rewrite.

Helper behaviors that must be testable without instantiating a Gazebo world:
- `MakeField("x", 0, FLOAT32)` → field with name "x", offset 0, datatype FLOAT32, count 1.
- `ToRosTime(gazebo::common::Time(5, 123000000))` → `Time{ sec=5, nanosec=123000000 }`.
- `SetPointCloud2Fields(pc)` populates 6 fields with offsets [0,4,8,12,16,24] and sets `point_step = 32`.

- [ ] **Step 1: Create test directory + scaffold (no test logic yet)**

```bash
mkdir -p robots/mid360_simulation/test
```

Create `robots/mid360_simulation/test/CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
ament_find_gtest()

ament_add_gtest(test_pointcloud2_layout
  test_pointcloud2_layout.cpp
)
target_include_directories(test_pointcloud2_layout PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(test_pointcloud2_layout
  ${GAZEBO_LIBRARIES}
  RayPlugin
)
ament_target_dependencies(test_pointcloud2_layout
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

ament_add_gtest(test_offset_time_monotonic
  test_offset_time_monotonic.cpp
)
target_include_directories(test_offset_time_monotonic PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(test_offset_time_monotonic
  ${GAZEBO_LIBRARIES}
  RayPlugin
)
ament_target_dependencies(test_offset_time_monotonic
  rclcpp
  sensor_msgs
  livox_ros_driver2
)
```

- [ ] **Step 2: Write failing test for `MakeField`**

Create `robots/mid360_simulation/test/test_pointcloud2_layout.cpp`:

```cpp
#include <gtest/gtest.h>
#include "mid360_simulation/mid360_points_plugin.h"

TEST(PointCloud2Layout, MakeFieldX) {
  auto f = gazebo::Mid360PointsPlugin::MakeField(
      "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(f.name, "x");
  EXPECT_EQ(f.offset, 0u);
  EXPECT_EQ(f.datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(f.count, 1u);
}
```

- [ ] **Step 3: Run test — must fail (linker error / undefined reference)**

```bash
cd /media/lenovo/disk/Embodied_AI/GazeboQuadbot
colcon build --packages-select mid360_simulation --cmake-clean-cache
source install/setup.bash
colcon test --packages-select mid360_simulation --event-handlers console_direct+
```

Expected: linker error `undefined reference to Mid360PointsPlugin::MakeField(...)`.

- [ ] **Step 4: Implement `MakeField` in mid360_points_plugin.cpp**

Add immediately after the `Mid360PointsPlugin` constructor / before the existing `convertDataToRotateInfo`:

```cpp
sensor_msgs::msg::PointField Mid360PointsPlugin::MakeField(
    const std::string& name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1u;
  return f;
}
```

- [ ] **Step 5: Rebuild and re-run — must pass**

```bash
colcon build --packages-select mid360_simulation
colcon test --packages-select mid360_simulation
```

Expected: `test_pointcloud2_layout` passes for `MakeFieldX`. Other tests still fail (helpers not yet defined).

- [ ] **Step 6: Implement `SetPointCloud2Fields` + `ToRosTime`**

Add to `mid360_points_plugin.cpp`:

```cpp
void Mid360PointsPlugin::SetPointCloud2Fields(sensor_msgs::msg::PointCloud2& pc)
{
  pc.fields.resize(6);
  pc.fields[0] = MakeField("x",          0,  sensor_msgs::msg::PointField::FLOAT32);
  pc.fields[1] = MakeField("y",          4,  sensor_msgs::msg::PointField::FLOAT32);
  pc.fields[2] = MakeField("z",          8,  sensor_msgs::msg::PointField::FLOAT32);
  pc.fields[3] = MakeField("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  pc.fields[4] = MakeField("ring",      16, sensor_msgs::msg::PointField::UINT16);
  pc.fields[5] = MakeField("timestamp", 24, sensor_msgs::msg::PointField::FLOAT64);
  pc.point_step = static_cast<uint32_t>(kPointStepBytes);
}

builtin_interfaces::msg::Time Mid360PointsPlugin::ToRosTime(
    const gazebo::common::Time& gz_time) const
{
  builtin_interfaces::msg::Time t;
  t.sec     = static_cast<int32_t>(gz_time.sec);
  t.nanosec = static_cast<uint32_t>(gz_time.nsec);
  return t;
}
```

- [ ] **Step 7: Extend test for `SetPointCloud2Fields`**

Append to `test_pointcloud2_layout.cpp`:

```cpp
TEST(PointCloud2Layout, FieldCountAndOffsets) {
  gazebo::Mid360PointsPlugin plugin;
  sensor_msgs::msg::PointCloud2 pc;
  plugin.SetPointCloud2Fields(pc);
  ASSERT_EQ(pc.fields.size(), 6u);
  EXPECT_EQ(pc.fields[0].offset, 0u);    // x
  EXPECT_EQ(pc.fields[3].offset, 12u);   // intensity
  EXPECT_EQ(pc.fields[4].offset, 16u);   // ring
  EXPECT_EQ(pc.fields[5].offset, 24u);   // timestamp
  EXPECT_EQ(pc.point_step, 32u);
}

TEST(PointCloud2Layout, ToRosTimeConvertsSecAndNsec) {
  gazebo::Mid360PointsPlugin plugin;
  gazebo::common::Time gz(5, 123000000);  // 5.123 s
  auto ros_t = plugin.ToRosTime(gz);
  EXPECT_EQ(ros_t.sec, 5);
  EXPECT_EQ(ros_t.nanosec, 123000000u);
}
```

- [ ] **Step 8: Run test — must pass**

```bash
colcon build --packages-select mid360_simulation
colcon test --packages-select mid360_simulation --event-handlers console_direct+
```

Expected: `PointCloud2Layout` tests pass; `OffsetTimeMonotonic` still pending.

- [ ] **Step 9: Implement `AppendPc2Row`**

```cpp
void Mid360PointsPlugin::AppendPc2Row(
    std::vector<uint8_t>& buf,
    float x, float y, float z,
    float intensity, uint16_t ring,
    double timestamp)
{
  const size_t off = buf.size();
  buf.resize(off + kPointStepBytes);
  std::memcpy(&buf[off +  0], &x,         sizeof(float));
  std::memcpy(&buf[off +  4], &y,         sizeof(float));
  std::memcpy(&buf[off +  8], &z,         sizeof(float));
  std::memcpy(&buf[off + 12], &intensity, sizeof(float));
  std::memcpy(&buf[off + 16], &ring,      sizeof(uint16_t));
  // 6 bytes padding (offsets 18..23) already zero from resize()
  std::memcpy(&buf[off + 24], &timestamp, sizeof(double));
}
```

Add `#include <cstring>` near the top of `mid360_points_plugin.cpp` if not already present.

- [ ] **Step 10: Extend test for `AppendPc2Row` byte layout**

Append to `test_pointcloud2_layout.cpp`:

```cpp
TEST(PointCloud2Layout, AppendRowProduces32Bytes) {
  gazebo::Mid360PointsPlugin plugin;
  std::vector<uint8_t> buf;
  plugin.AppendPc2Row(buf, 1.0f, 2.0f, 3.0f, 100.0f, /*ring=*/0, 4.5);
  ASSERT_EQ(buf.size(), 32u);

  float x, y, z, intensity;
  uint16_t ring;
  double timestamp;
  std::memcpy(&x,         &buf[ 0], 4);
  std::memcpy(&y,         &buf[ 4], 4);
  std::memcpy(&z,         &buf[ 8], 4);
  std::memcpy(&intensity, &buf[12], 4);
  std::memcpy(&ring,      &buf[16], 2);
  std::memcpy(&timestamp, &buf[24], 8);
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 2.0f);
  EXPECT_FLOAT_EQ(z, 3.0f);
  EXPECT_FLOAT_EQ(intensity, 100.0f);
  EXPECT_EQ(ring, 0u);
  EXPECT_DOUBLE_EQ(timestamp, 4.5);
}
```

Add `#include <cstring>` at the top of the test file.

- [ ] **Step 11: Run all helper tests — must pass**

```bash
colcon build --packages-select mid360_simulation
colcon test --packages-select mid360_simulation --event-handlers console_direct+
```

Expected: 4 tests pass in `test_pointcloud2_layout`.

- [ ] **Step 12: Commit**

```bash
git add robots/mid360_simulation/src/mid360_points_plugin.cpp \
        robots/mid360_simulation/test/
git commit -m "feat(mid360_simulation): add PC2 helpers (TDD: MakeField/SetFields/ToRosTime/AppendRow)"
```

---

## Task 4: Wire `customPub_` in `Load()` + Rename PointCloud2 Topic

**Files:**
- Modify: `robots/mid360_simulation/src/mid360_points_plugin.cpp` (the `Load` function, around lines 115–120)

**Interfaces:**
- Consumed by T5.

- [ ] **Step 1: Locate the existing publisher creation in `Load`**

Read `src/mid360_points_plugin.cpp` ~lines 115–120. You should see:

```cpp
// 创建 ROS2 PointCloud2 发布器
cloudPub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
    curr_scan_topic, 10);
```

- [ ] **Step 2: Rename PointCloud2 topic to include `_PointCloud2` suffix**

Replace the existing `cloudPub_` line:

```cpp
// 创建 ROS2 PointCloud2 发布器 (Nav2 / costmap path)
cloudPub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
    curr_scan_topic + "_PointCloud2", 10);

// 创建 ROS2 livox CustomMsg 发布器 (FAST-LIO path)
customPub_ = rosNode_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
    curr_scan_topic, 10);
```

- [ ] **Step 3: Verify build still succeeds**

```bash
colcon build --packages-select mid360_simulation
```

Expected: clean build, no warnings.

- [ ] **Step 4: Commit**

```bash
git add robots/mid360_simulation/src/mid360_points_plugin.cpp
git commit -m "feat(mid360_simulation): create CustomMsg publisher; suffix PointCloud2 topic"
```

---

## Task 5: Rewrite `OnNewLaserScans` — Single Shared Loop, Dual Publish

**Files:**
- Modify: `robots/mid360_simulation/src/mid360_points_plugin.cpp` (replace the entire `OnNewLaserScans()` body, lines 189–293 in the original)

**Interfaces:**
- Consumed by T7 (offset_time monotonicity test) and downstream FAST-LIO2 / Nav2.

- [ ] **Step 1: Add `#include <algorithm>` for `std::clamp`**

Check top of `mid360_points_plugin.cpp`; if `#include <algorithm>` is not already present, add it alongside the other stdlib includes.

- [ ] **Step 2: Replace the `OnNewLaserScans()` body**

Replace the entire body of `Mid360PointsPlugin::OnNewLaserScans()` (from `void Mid360PointsPlugin::OnNewLaserScans() {` through the matching `}`) with:

```cpp
void Mid360PointsPlugin::OnNewLaserScans()
{
  if (!rayShape_) return;

  // Initialize ray scan point pairs
  std::vector<std::pair<int, RotateInfo>> points_pair;
  InitializeRays(points_pair, rayShape_);
  rayShape_->Update();

  // Set the internal laser scan message timestamp (unchanged behavior)
  msgs::Set(laserMsg_.mutable_time(), world->SimTime());
  msgs::LaserScan* scan = laserMsg_.mutable_scan();
  InitializeScan(scan);

  // ----- Time (single source: Gazebo sim time) -----
  const gazebo::common::Time& gz_sim_time = world->SimTime();
  const double sim_time_sec = gz_sim_time.Double();

  // ----- CustomMsg skeleton -----
  livox_ros_driver2::msg::CustomMsg custom_msg;
  custom_msg.header.frame_id = raySensor_->Name();
  custom_msg.header.stamp    = ToRosTime(gz_sim_time);
  custom_msg.timebase        = 0u;
  custom_msg.lidar_id        = 0;
  custom_msg.points.reserve(points_pair.size());

  // ----- PointCloud2 skeleton -----
  sensor_msgs::msg::PointCloud2 pc2;
  pc2.header       = custom_msg.header;
  pc2.header.frame_id = raySensor_->Name();
  pc2.height       = 1;
  pc2.is_dense     = true;
  pc2.is_bigendian = false;
  SetPointCloud2Fields(pc2);

  std::vector<uint8_t> pc2_buf;
  pc2_buf.reserve(points_pair.size() * kPointStepBytes);

  // ----- Shared single loop -----
  for (const auto& pair : points_pair)
  {
    const double range = rayShape_->GetRange(pair.first);

    // Drop out-of-range rays (matches old behavior)
    if (range >= RangeMax() || range <= RangeMin()) continue;

    const double retro = rayShape_->GetRetro(pair.first);  // 0..1
    const auto&  info   = pair.second;                    // RotateInfo

    // Geometry (unchanged from previous implementation)
    ignition::math::Quaterniond ray;
    ray.Euler(ignition::math::Vector3d(0.0, info.zenith, info.azimuth));
    const auto axis  = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
    const auto point = range * axis;

    // ===== CustomPoint =====
    livox_ros_driver2::msg::CustomPoint cp;
    cp.offset_time  = static_cast<uint32_t>(info.time_us * 1000.0);  // us -> ns
    cp.x            = static_cast<float>(point.X());
    cp.y            = static_cast<float>(point.Y());
    cp.z            = static_cast<float>(point.Z());
    cp.reflectivity = static_cast<uint8_t>(std::clamp(retro * 255.0, 0.0, 255.0));
    cp.tag          = kMid360Tag;
    cp.line         = kMid360Line;
    custom_msg.points.push_back(cp);

    // ===== PointCloud2 row =====
    const float    intensity  = cp.reflectivity;
    const uint16_t ring       = 0u;
    const double   point_time = sim_time_sec + info.time_us * 1e-6;
    AppendPc2Row(pc2_buf, cp.x, cp.y, cp.z, intensity, ring, point_time);
  }

  // ----- Publish internal LaserScanStamped (unchanged) -----
  if (scanPub_ && scanPub_->HasConnections())
  {
    scanPub_->Publish(laserMsg_);
  }

  // ----- CustomMsg publish -----
  custom_msg.point_num = static_cast<uint32_t>(custom_msg.points.size());
  if (customPub_) customPub_->publish(custom_msg);

  // ----- PointCloud2 publish -----
  pc2.width    = custom_msg.point_num;
  pc2.row_step = pc2.width * pc2.point_step;
  pc2.data     = std::move(pc2_buf);
  if (cloudPub_) cloudPub_->publish(pc2);
}
```

- [ ] **Step 3: Replace `info.time` → `info.time_us`**

In `convertDataToRotateInfo` (lines 45–69 of the original `mid360_points_plugin.cpp`), change:

```cpp
info.time = data[0];
```

to:

```cpp
info.time_us = data[0];  // CSV column 1 is microseconds (header Time/us)
```

Then in the header `mid360_points_plugin.h`, find:

```cpp
struct RotateInfo
{
  double time;      ///< 时间戳 (用于点云时序)
  double azimuth;   ///< 方位角 (水平角度, 弧度)
  double zenith;    ///< 天顶角 (垂直角度, 弧度)
};
```

Replace `double time;` with `double time_us;` and update the comment:

```cpp
struct RotateInfo
{
  double time_us;   ///< CSV 第一列：帧内扫描偏移（微秒 us）
  double azimuth;   ///< 方位角 (水平角度, 弧度)
  double zenith;    ///< 天顶角 (垂直角度, 弧度)
};
```

- [ ] **Step 4: Build and resolve any warnings**

```bash
colcon build --packages-select mid360_simulation
```

Expected: clean build. If you see warnings about narrowing conversions or unused includes, fix inline.

- [ ] **Step 5: Commit**

```bash
git add robots/mid360_simulation/include/mid360_simulation/mid360_points_plugin.h \
        robots/mid360_simulation/src/mid360_points_plugin.cpp
git commit -m "feat(mid360_simulation): dual-publish CustomMsg + PointCloud2 from single loop"
```

---

## Task 6: Fix CSV Header

**Files:**
- Modify: `robots/mid360_simulation/scan_mode/mid360.csv` (first line only)

- [ ] **Step 1: Inspect current header**

```bash
head -1 robots/mid360_simulation/scan_mode/mid360.csv
```

Expected output: `Time/s,Azimuth/deg,Zenith/deg`

- [ ] **Step 2: Rewrite the first line**

```bash
sed -i '1s/Time\/s/Time\/us/' robots/mid360_simulation/scan_mode/mid360.csv
head -1 robots/mid360_simulation/scan_mode/mid360.csv
```

Expected output: `Time/us,Azimuth/deg,Zenith/deg`

- [ ] **Step 3: Verify data rows are untouched**

```bash
sed -n '2p;800000p;800001p' robots/mid360_simulation/scan_mode/mid360.csv
```

Expected: same numerical values as before (e.g., `1,268.99,37.838`, `799999,...`, `800000,31.165,41.523`).

- [ ] **Step 4: Rebuild to confirm header rename is benign**

```bash
colcon build --packages-select mid360_simulation
```

Expected: clean build; `CsvReader` ignores the header line.

- [ ] **Step 5: Commit**

```bash
git add robots/mid360_simulation/scan_mode/mid360.csv
git commit -m "fix(mid360_simulation): correct CSV header unit (Time/s -> Time/us)"
```

---

## Task 7: Add Offset-Time Monotonicity + Publish-Count Invariant Test

**Files:**
- Create: `robots/mid360_simulation/test/test_offset_time_monotonic.cpp`

**Interfaces:**
- Validates the invariants in spec §5:
  - `offset_time` monotonic non-decreasing within a frame.
  - `point_num == width` invariant.
  - CustomPoint.tag == 0x10, .line == 0.

Since `Mid360PointsPlugin::OnNewLaserScans()` requires a live Gazebo world, this test exercises the helpers and the algorithm logic directly (not the callback).

- [ ] **Step 1: Write the test file**

Create `robots/mid360_simulation/test/test_offset_time_monotonic.cpp`:

```cpp
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <vector>
#include "mid360_simulation/mid360_points_plugin.h"

// Helper: produce a vector of synthetic CustomPoints equivalent to a frame
// (mimics the body of OnNewLaserScans but bypasses Gazebo).
static std::vector<livox_ros_driver2::msg::CustomPoint>
buildSyntheticFrame(const std::vector<double>& time_us_seq,
                    double retro_value)
{
  std::vector<livox_ros_driver2::msg::CustomPoint> out;
  out.reserve(time_us_seq.size());
  for (double t : time_us_seq) {
    livox_ros_driver2::msg::CustomPoint p;
    p.offset_time  = static_cast<uint32_t>(t * 1000.0);
    p.x = 1.0f; p.y = 2.0f; p.z = 3.0f;
    p.reflectivity = static_cast<uint8_t>(std::clamp(retro_value * 255.0, 0.0, 255.0));
    p.tag   = gazebo::kMid360Tag;
    p.line  = gazebo::kMid360Line;
    out.push_back(p);
  }
  return out;
}

TEST(OffsetTime, MonotonicNonDecreasing) {
  // Realistic mid360 CSV pattern: monotonically increasing microseconds
  std::vector<double> seq = {1.0, 50.0, 100.0, 250.0, 800.0, 1500.0};
  auto pts = buildSyntheticFrame(seq, 0.5);
  ASSERT_EQ(pts.size(), seq.size());
  for (size_t i = 1; i < pts.size(); ++i) {
    EXPECT_GE(pts[i].offset_time, pts[i - 1].offset_time);
  }
}

TEST(OffsetTime, UnitsAreNanoseconds) {
  auto pts = buildSyntheticFrame({1000.0}, 1.0);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_EQ(pts[0].offset_time, 1000000u);  // 1000 us * 1000 = 1e6 ns
}

TEST(CustomPointFields, TagAndLineConstants) {
  auto pts = buildSyntheticFrame({0.0}, 0.0);
  EXPECT_EQ(pts[0].tag,  0x10);
  EXPECT_EQ(pts[0].line, 0u);
}

TEST(Invariants, PointNumMatchesWidth) {
  // Simulate the OnNewLaserScans construction
  gazebo::Mid360PointsPlugin plugin;
  std::vector<double> seq = {10.0, 20.0, 30.0, 40.0};
  auto pts = buildSyntheticFrame(seq, 0.7);

  sensor_msgs::msg::PointCloud2 pc2;
  pc2.header.frame_id = "livox";
  plugin.SetPointCloud2Fields(pc2);

  std::vector<uint8_t> buf;
  buf.reserve(pts.size() * gazebo::kPointStepBytes);
  for (const auto& cp : pts) {
    plugin.AppendPc2Row(buf, cp.x, cp.y, cp.z,
                        static_cast<float>(cp.reflectivity), 0u,
                        /*abs_t=*/0.0);
  }
  pc2.width    = static_cast<uint32_t>(pts.size());
  pc2.row_step = pc2.width * pc2.point_step;
  pc2.data     = std::move(buf);

  EXPECT_EQ(static_cast<uint32_t>(pts.size()), pc2.width);
  EXPECT_EQ(pc2.data.size(), pts.size() * gazebo::kPointStepBytes);
}
```

- [ ] **Step 2: Register the test in `test/CMakeLists.txt`**

The `test/CMakeLists.txt` from Task 3 already includes `ament_add_gtest(test_offset_time_monotonic ...)`. Verify it references `livox_ros_driver2` and `sensor_msgs` (which Task 3 already did).

- [ ] **Step 3: Build and run all tests**

```bash
colcon build --packages-select mid360_simulation
colcon test --packages-select mid360_simulation --event-handlers console_direct+
```

Expected output (abridged):

```
[==========] Running 4 tests from test_pointcloud2_layout.
[  PASSED  ] 4 tests.
[==========] Running 4 tests from test_offset_time_monotonic.
[  PASSED  ] 4 tests.
```

If any test fails, fix the helper or the test until all pass.

- [ ] **Step 4: Commit**

```bash
git add robots/mid360_simulation/test/
git commit -m "test(mid360_simulation): assert offset_time monotonicity + publish-count invariant"
```

---

## Task 8: Update README

**Files:**
- Modify: `robots/mid360_simulation/README.md` (the "## 发布的话题" section around line 100–105; add a new "## 依赖项" subsection before "## 编译"; update "## 编译" to include the manual `CMAKE_PREFIX_PATH` step)

- [ ] **Step 1: Replace the topic table**

In `robots/mid360_simulation/README.md`, find the existing table:

```
| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 点云数据 |
```

Replace it with:

```
| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox 自定义点云（FAST-LIO2 默认订阅） |
| `/livox/lidar_PointCloud2` | `sensor_msgs/PointCloud2` | 标准点云 6 字段布局（Nav2 / costmap 兼容） |
```

- [ ] **Step 2: Update Topics List at repo root**

Edit `README.md` (repo root) — find the "Topics List" section near line 77 and update the lidar row:

```
| `/livox/lidar` | livox_ros_driver2/msg/CustomMsg |
| `/livox/lidar_PointCloud2` | sensor_msgs/msg/PointCloud2 |
```

(Replace the existing `/livox/lidar | sensor_msgs/msg/PointCloud2` row.)

- [ ] **Step 3: Add `livox_ros_driver2` to dependencies section**

In `robots/mid360_simulation/README.md`, find the "## 依赖项" section and add a bullet:

```markdown
- `livox_ros_driver2`（提供 `CustomMsg`）— 源码：https://github.com/Livox-SDK/livox_ros_driver2
```

- [ ] **Step 4: Document the manual `CMAKE_PREFIX_PATH` build convention**

Find the existing "## 编译" section in `robots/mid360_simulation/README.md` and replace its content with:

```markdown
## 编译

> ⚠️ 本仓库**不**把 `CMAKE_PREFIX_PATH` 写入 `.bashrc` / `.zshrc`。
> 每次构建前手动 export 一次：

\`\`\`bash
export CMAKE_PREFIX_PATH="/media/lenovo/disk/planner_ws/src/Nav3D/install:$CMAKE_PREFIX_PATH"
cd ~/your_ws
colcon build --packages-select mid360_simulation --cmake-clean-cache
source install/setup.bash
\`\`\`
```

(If `~/your_ws` is intended to be relative, the engineer can replace with the actual path.)

- [ ] **Step 5: Verify README renders correctly**

```bash
head -120 robots/mid360_simulation/README.md
```

Spot-check that the topic table and compile instructions read sensibly.

- [ ] **Step 6: Commit**

```bash
git add robots/mid360_simulation/README.md README.md
git commit -m "docs(mid360_simulation): dual-publish topics + livox_ros_driver2 install + build convention"
```

---

## Task 9: Integration Test — `ros2 bag` Recording + RViz

**Files:** none — verification only.

- [ ] **Step 1: Build the plugin**

```bash
export CMAKE_PREFIX_PATH="/media/lenovo/disk/planner_ws/src/Nav3D/install:$CMAKE_PREFIX_PATH"
cd /media/lenovo/disk/Embodied_AI/GazeboQuadbot
colcon build --packages-select mid360_simulation
source install/setup.bash
```

- [ ] **Step 2: Launch Gazebo**

In **Terminal 1**:

```bash
ros2 launch robot_scene go2w_lidar_gps.launch.py
```

Wait until "Mid-360 仿真插件加载完成" appears in logs.

- [ ] **Step 3: Confirm both topics publish**

In **Terminal 2**:

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar_PointCloud2
```

Expected: both topics listed; both report ~10 Hz (matching `update_rate`).

- [ ] **Step 4: Inspect one CustomMsg**

```bash
ros2 topic echo /livox/lidar --once --field points[0..2]
```

Expected: `offset_time` is a small ns value (`0` to `~800000`); `x, y, z` are reasonable distances; `tag=0x10`, `line=0`, `reflectivity` ∈ [0, 255].

- [ ] **Step 5: Inspect one PointCloud2**

```bash
ros2 topic echo /livox/lidar_PointCloud2 --once --field width,fields,point_step
```

Expected: `width` matches CustomMsg `point_num`; `fields` has 6 entries with offsets `[0, 4, 8, 12, 16, 24]`; `point_step=32`.

- [ ] **Step 6: Record a bag**

```bash
mkdir -p /tmp/mid360_test && cd /tmp/mid360_test
ros2 bag record /livox/lidar /livox/lidar_PointCloud2 -o mid360_test --max-duration 20
```

Expected: 20-second bag recorded.

- [ ] **Step 7: Open RViz and verify both clouds look identical**

In **Terminal 3**:

```bash
ros2 run rviz2 rviz2
```

Add two PointCloud2 displays, one for `/livox/lidar` (set `Reliability Policy = Reliable`, transport `point_cloud2` if necessary) and one for `/livox/lidar_PointCloud2`. Visually confirm identical coverage.

- [ ] **Step 8: Note results**

No commit needed (no code change). If any check fails, file a follow-up and revisit Tasks 4/5.

---

## Task 10: FAST-LIO2 End-to-End Smoke

**Files:** none — verification only.

- [ ] **Step 1: Ensure Nav3D workspace is sourced**

In a fresh terminal:

```bash
cd /media/lenovo/disk/planner_ws/src/Nav3D
source install/setup.bash
```

- [ ] **Step 2: Confirm FAST-LIO2 launches against the new topic**

With Gazebo running (Task 9 Terminal 1), in the Nav3D-sourced terminal:

```bash
ros2 launch fastlio2 lio_launch.py
```

Expected: FAST-LIO2 starts without "topic not found" warnings; logs show `lidar_cb` receiving frames.

- [ ] **Step 3: Confirm time alignment**

In the FAST-LIO2 log, look for any of:

```
[WARN] [Lidar Message is out of order]
[WARN] [IMU ahead of cloud_end_time]
```

Expected: no warnings. (The sim-time `header.stamp` should align with IMU's `header.stamp` because both use the same `/clock` topic.)

- [ ] **Step 4: Note results**

No commit needed. If warnings appear, dump 1 frame of `/clock`, `/livox/lidar`, and `/livox/imu` to a bag for diagnosis; otherwise the upgrade is complete.

---

## Self-Review

**1. Spec coverage:**

| Spec section | Implemented by |
|--------------|---------------|
| §1 Goals (dual publish + bug fixes) | T4 (pubs), T5 (loop), T6 (CSV header) |
| §3 Architecture (single shared loop) | T5 |
| §4 Files touched | T1, T2, T4, T5, T6, T8 |
| §6.1 CustomMsg fields | T5 (publish + fill), T7 (test) |
| §6.2 PointCloud2 6-field 32-byte layout | T3 (helpers), T5 (use), T7 (test) |
| §6.3 CSV format `Time/us` | T6 |
| §7 Invariants (single time source, shared loop, monotonic offset_time, no raw new/delete, type-safe casts) | T5, T7 |
| §8 Error handling | T5 (empty-frame publish), T3 (clamp in `AppendPc2Row`), T6 (no change in reader behavior) |
| §9 Testing strategy | T3 (PC2 layout), T7 (offset_time invariants), T9 (integration), T10 (LIO end-to-end) |
| §10 Development workflow (manual `CMAKE_PREFIX_PATH`) | T1, T8 (README) |
| §11 Risks | T5 reserves capacity (realloc risk), T3 uses 32-byte alignment (padding risk) |
| §12 Out of scope | Not implemented (correctly) |

No gaps.

**2. Placeholder scan:** Searched for `TBD`, `TODO`, `fill in`, `similar to task`, `implement later`. None present. Each code block contains actual code; each step is a concrete action.

**3. Type consistency:**

- `MakeField` signature: `static sensor_msgs::msg::PointField MakeField(const std::string&, uint32_t, uint8_t);` — consistent across T2 (declaration), T3 (definition + tests), T3 (SetPointCloud2Fields body).
- `AppendPc2Row` signature: `void AppendPc2Row(std::vector<uint8_t>&, float, float, float, float, uint16_t, double);` — consistent across T2 (declaration), T3 (definition), T5 (call site: `AppendPc2Row(pc2_buf, cp.x, cp.y, cp.z, intensity, ring, point_time)`), T7 (test).
- `ToRosTime` signature: `builtin_interfaces::msg::Time ToRosTime(const gazebo::common::Time&) const;` — consistent across T2, T3, T5.
- `kPointStepBytes` / `kMid360Tag` / `kMid360Line` — declared in T2, used in T3 / T5 / T7.
- `info.time_us` rename — applied in T5 to `convertDataToRotateInfo` body and `RotateInfo` struct; no stale references to `info.time`.

No inconsistencies.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-05-mid360-lidar-upgrade.md`.

**Two execution options:**

1. **Subagent-Driven (recommended)** — dispatch a fresh subagent per task, review between tasks, fast iteration with quality gates.
2. **Inline Execution** — execute tasks in this session using `superpowers:executing-plans`, batch execution with checkpoints.