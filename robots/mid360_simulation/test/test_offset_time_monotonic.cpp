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
