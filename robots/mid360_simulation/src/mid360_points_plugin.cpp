/**
 * @file mid360_points_plugin.cpp
 * @brief Mid-360 激光雷达点云仿真插件实现
 *
 * 实现基于真实扫描模式的 Livox Mid-360 激光雷达仿真
 * 发布标准 ROS2 PointCloud2 消息
 */

#include <algorithm>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>

#include "mid360_simulation/mid360_points_plugin.h"
#include "mid360_simulation/csv_reader.hpp"
#include "mid360_simulation/mid360_ode_multiray_shape.h"

namespace gazebo
{

// 注册 Gazebo 传感器插件
GZ_REGISTER_SENSOR_PLUGIN(Mid360PointsPlugin)

//==============================================================================
// 构造函数和析构函数
//==============================================================================

Mid360PointsPlugin::Mid360PointsPlugin() {}

Mid360PointsPlugin::~Mid360PointsPlugin() {}

//==============================================================================
// 消息构造辅助方法 (TDD helpers)
//==============================================================================

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

//==============================================================================
// 辅助函数: 将 CSV 数据转换为旋转信息
//==============================================================================

/**
 * @brief 将 CSV 数据转换为扫描旋转信息
 * @param datas CSV 原始数据 [time, azimuth_deg, zenith_deg]
 * @param infos 输出的旋转信息列表
 */
static void convertDataToRotateInfo(
    const std::vector<std::vector<double>>& datas,
    std::vector<RotateInfo>& infos)
{
    infos.reserve(datas.size());
    constexpr double deg_to_rad = M_PI / 180.0;

    for (const auto& data : datas)
    {
        if (data.size() == 3)
        {
            RotateInfo info;
            info.time_us = data[0];  // CSV column 1 is microseconds (header Time/us)
            info.azimuth = data[1] * deg_to_rad;
            // 转换为标准右手坐标系角度
            info.zenith = data[2] * deg_to_rad - M_PI_2;
            infos.push_back(info);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Mid360Plugin"),
                         "CSV 数据格式错误: 期望 3 列，实际 %zu 列", data.size());
        }
    }
}

//==============================================================================
// 插件加载
//==============================================================================

void Mid360PointsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf)
{
    // 获取 ROS2 节点
    rosNode_ = gazebo_ros::Node::Get(sdf);

    // 读取扫描模式 CSV 文件
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "加载扫描模式文件: %s", file_name.c_str());

    if (!mid360_simulation::CsvReader::ReadCsvFile(file_name, datas))
    {
        RCLCPP_ERROR(rclcpp::get_logger("Mid360Plugin"),
                     "无法读取 CSV 文件: %s", file_name.c_str());
        return;
    }

    // 保存 SDF 配置
    sdfPtr_ = sdf;
    auto rayElem = sdfPtr_->GetElement("ray");
    auto rangeElem = rayElem->GetElement("range");

    // 初始化传感器
    raySensor_ = _parent;
    auto curr_scan_topic = sdf->Get<std::string>("topic");
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "ROS 话题: %s", curr_scan_topic.c_str());

    // 获取坐标系名称
    childName_ = raySensor_->Name();
    parentName_ = raySensor_->ParentName();
    size_t delimiter_pos = parentName_.find("::");
    parentName_ = parentName_.substr(delimiter_pos + 2);

    // 初始化 Gazebo 传输节点
    gazeboNode_ = transport::NodePtr(new transport::Node());
    gazeboNode_->Init(raySensor_->WorldName());

    // 创建 ROS2 PointCloud2 发布器 (Nav2 / costmap path)
    cloudPub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
        curr_scan_topic + "_PointCloud2", 10);

    // 创建 ROS2 livox CustomMsg 发布器 (FAST-LIO path)
    customPub_ = rosNode_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        curr_scan_topic, 10);

    // 创建 Gazebo 内部扫描消息发布器
    scanPub_ = gazeboNode_->Advertise<msgs::LaserScanStamped>(
        curr_scan_topic + "laserscan", 50);

    // 转换扫描模式数据
    scanInfos_.clear();
    convertDataToRotateInfo(datas, scanInfos_);
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "扫描点数: %zu", scanInfos_.size());
    maxPointSize_ = scanInfos_.size();

    // 加载基类
    RayPlugin::Load(_parent, sdfPtr_);
    laserMsg_.mutable_scan()->set_frame(_parent->ParentName());

    // 获取父实体
    parentEntity_ = this->world->EntityByName(_parent->ParentName());

    // 创建激光碰撞体
    auto physics = world->Physics();
    laserCollision_ = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision_->SetName("mid360_ray_collision");
    laserCollision_->SetRelativePose(_parent->Pose());
    laserCollision_->SetInitialRelativePose(_parent->Pose());

    // 创建多射线形状
    rayShape_.reset(new physics::Mid360OdeMultiRayShape(laserCollision_));
    laserCollision_->SetShape(rayShape_);

    // 读取采样参数
    samplesStep_ = sdfPtr_->Get<int>("samples");
    downSample_ = sdfPtr_->Get<int>("downsample");
    if (downSample_ < 1) downSample_ = 1;

    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "采样数: %ld, 降采样: %ld", samplesStep_, downSample_);

    // 初始化射线形状
    rayShape_->RayShapes().reserve(samplesStep_ / downSample_);
    rayShape_->Load(sdfPtr_);
    rayShape_->Init();

    // 读取测距范围
    minDist_ = rangeElem->Get<double>("min");
    maxDist_ = rangeElem->Get<double>("max");

    // 创建初始射线
    auto offset = laserCollision_->RelativePose();
    ignition::math::Vector3d start_point, end_point;

    for (int j = 0; j < samplesStep_; j += downSample_)
    {
        int index = j % maxPointSize_;
        auto& rotate_info = scanInfos_[index];

        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

        start_point = minDist_ * axis + offset.Pos();
        end_point = maxDist_ * axis + offset.Pos();
        rayShape_->AddRay(start_point, end_point);
    }

    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"), "Mid-360 仿真插件加载完成");
}

//==============================================================================
// 新扫描数据回调
//==============================================================================

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

//==============================================================================
// 初始化射线
//==============================================================================

void Mid360PointsPlugin::InitializeRays(
    std::vector<std::pair<int, RotateInfo>>& points_pair,
    boost::shared_ptr<physics::Mid360OdeMultiRayShape>& ray_shape)
{
    auto& rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision_->RelativePose();

    int64_t end_index = currStartIndex_ + samplesStep_;
    size_t ray_index = 0;
    auto ray_size = rays.size();
    points_pair.reserve(rays.size());

    for (int64_t k = currStartIndex_; k < end_index; k += downSample_)
    {
        auto index = k % maxPointSize_;
        auto& rotate_info = scanInfos_[index];

        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

        start_point = minDist_ * axis + offset.Pos();
        end_point = maxDist_ * axis + offset.Pos();

        if (ray_index < ray_size)
        {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }

    currStartIndex_ += samplesStep_;
}

//==============================================================================
// 初始化扫描消息
//==============================================================================

void Mid360PointsPlugin::InitializeScan(msgs::LaserScan*& scan)
{
    msgs::Set(scan->mutable_world_pose(),
              raySensor_->Pose() + parentEntity_->WorldPose());

    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j)
    {
        for (unsigned int i = 0; i < rangeCount; ++i)
        {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

//==============================================================================
// 角度和范围获取方法
//==============================================================================

ignition::math::Angle Mid360PointsPlugin::AngleMin() const
{
    return rayShape_ ? rayShape_->MinAngle() : ignition::math::Angle(-1);
}

ignition::math::Angle Mid360PointsPlugin::AngleMax() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->MaxAngle().Radian())
                     : ignition::math::Angle(-1);
}

double Mid360PointsPlugin::AngleResolution() const
{
    return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1);
}

double Mid360PointsPlugin::RangeMin() const
{
    return minDist_;
}

double Mid360PointsPlugin::RangeMax() const
{
    return maxDist_;
}

double Mid360PointsPlugin::RangeResolution() const
{
    return rayShape_ ? rayShape_->GetResRange() : -1;
}

int Mid360PointsPlugin::RayCount() const
{
    return rayShape_ ? rayShape_->GetSampleCount() : -1;
}

int Mid360PointsPlugin::RangeCount() const
{
    return rayShape_ ? rayShape_->GetSampleCount() * rayShape_->GetScanResolution() : -1;
}

int Mid360PointsPlugin::VerticalRayCount() const
{
    return rayShape_ ? rayShape_->GetVerticalSampleCount() : -1;
}

int Mid360PointsPlugin::VerticalRangeCount() const
{
    return rayShape_ ? rayShape_->GetVerticalSampleCount() * rayShape_->GetVerticalScanResolution() : -1;
}

ignition::math::Angle Mid360PointsPlugin::VerticalAngleMin() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->VerticalMinAngle().Radian())
                     : ignition::math::Angle(-1);
}

ignition::math::Angle Mid360PointsPlugin::VerticalAngleMax() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->VerticalMaxAngle().Radian())
                     : ignition::math::Angle(-1);
}

double Mid360PointsPlugin::VerticalAngleResolution() const
{
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}

} // namespace gazebo
