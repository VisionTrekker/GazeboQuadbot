#include <gtest/gtest.h>
#include <cstring>
#include "mid360_simulation/mid360_points_plugin.h"

TEST(PointCloud2Layout, MakeFieldX) {
  auto f = gazebo::Mid360PointsPlugin::MakeField(
      "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(f.name, "x");
  EXPECT_EQ(f.offset, 0u);
  EXPECT_EQ(f.datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(f.count, 1u);
}

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
