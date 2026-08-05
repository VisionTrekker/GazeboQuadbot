/**
 * @file mid360_points_plugin.h
 * @brief Mid-360 激光雷达 Gazebo 仿真插件
 *
 * 实现 Livox Mid-360 激光雷达在 Gazebo 中的仿真
 * 基于真实扫描模式生成点云数据，发布 PointCloud2 消息
 */

#ifndef MID360_SIMULATION_POINTS_PLUGIN_H
#define MID360_SIMULATION_POINTS_PLUGIN_H

#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "mid360_simulation/mid360_ode_multiray_shape.h"

namespace gazebo
{

// Layout / protocol constants (see spec §4)
static constexpr std::size_t kPointStepBytes = 32;  // bytes per PointCloud2 row
static constexpr uint8_t     kMid360Tag       = 0x10;  // normal single-echo (FAST-LIO accept)
static constexpr uint8_t     kMid360Line      = 0;     // mid360 is single-line

/**
 * @struct RotateInfo
 * @brief 扫描点旋转信息
 *
 * 存储从 CSV 文件读取的每个扫描点的角度信息
 */
struct RotateInfo
{
    double time;      ///< 时间戳 (用于点云时序)
    double azimuth;   ///< 方位角 (水平角度, 弧度)
    double zenith;    ///< 天顶角 (垂直角度, 弧度)
};

/**
 * @class Mid360PointsPlugin
 * @brief Mid-360 点云仿真插件
 *
 * 继承自 Gazebo RayPlugin，实现激光雷达仿真功能:
 * - 加载扫描模式 CSV 文件
 * - 创建射线传感器
 * - 生成并发布 PointCloud2 消息
 */
class Mid360PointsPlugin : public RayPlugin
{
public:
    Mid360PointsPlugin();
    virtual ~Mid360PointsPlugin();

    /**
     * @brief 插件加载回调
     * @param _parent 父传感器指针
     * @param _sdf SDF 元素指针
     */
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
    /**
     * @brief 新扫描数据回调
     *
     * 每次传感器更新时调用，生成并发布点云
     */
    virtual void OnNewLaserScans();

private:
    // ============================================================
    // 角度和范围获取方法
    // ============================================================

    ignition::math::Angle AngleMin() const;
    ignition::math::Angle AngleMax() const;
    double AngleResolution() const;
    double RangeMin() const;
    double RangeMax() const;
    double RangeResolution() const;
    int RayCount() const;
    int RangeCount() const;
    int VerticalRayCount() const;
    int VerticalRangeCount() const;
    ignition::math::Angle VerticalAngleMin() const;
    ignition::math::Angle VerticalAngleMax() const;
    double VerticalAngleResolution() const;

    // ============================================================
    // 内部辅助方法
    // ============================================================

    /**
     * @brief 初始化射线扫描点对
     * @param points_pair 输出的射线索引和旋转信息对
     * @param ray_shape 射线形状指针
     */
    void InitializeRays(std::vector<std::pair<int, RotateInfo>>& points_pair,
                        boost::shared_ptr<physics::Mid360OdeMultiRayShape>& ray_shape);

    /**
     * @brief 初始化激光扫描消息
     * @param scan 激光扫描消息指针
     */
    void InitializeScan(msgs::LaserScan*& scan);

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

    // ============================================================
    // 成员变量
    // ============================================================

    /// @brief 多射线形状指针
    boost::shared_ptr<physics::Mid360OdeMultiRayShape> rayShape_;

    /// @brief 激光碰撞体指针
    physics::CollisionPtr laserCollision_;

    /// @brief 父实体指针
    physics::EntityPtr parentEntity_;

    /// @brief Gazebo 扫描消息发布器
    transport::PublisherPtr scanPub_;

    /// @brief SDF 元素指针
    sdf::ElementPtr sdfPtr_;

    /// @brief Gazebo 传输节点
    transport::NodePtr gazeboNode_;

    /// @brief 激光扫描消息
    msgs::LaserScanStamped laserMsg_;

    /// @brief 射线传感器指针
    sensors::SensorPtr raySensor_;

    /// @brief 扫描模式信息列表
    std::vector<RotateInfo> scanInfos_;

    /// @brief ROS2 节点指针
    gazebo_ros::Node::SharedPtr rosNode_;

    /// @brief PointCloud2 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

    /// @brief livox CustomMsg publisher (FAST-LIO2 path)
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPub_;

    /// @brief 父坐标系名称
    std::string parentName_;

    /// @brief 子坐标系名称
    std::string childName_;

    /// @brief 每帧采样步数
    int64_t samplesStep_ = 0;

    /// @brief 当前起始索引
    int64_t currStartIndex_ = 0;

    /// @brief 最大点数
    int64_t maxPointSize_ = 1000;

    /// @brief 降采样因子
    int64_t downSample_ = 1;

    /// @brief 最大测距距离
    double maxDist_ = 400.0;

    /// @brief 最小测距距离
    double minDist_ = 0.1;
};

} // namespace gazebo

#endif // MID360_SIMULATION_POINTS_PLUGIN_H
