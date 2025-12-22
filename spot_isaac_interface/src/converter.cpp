#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <memory>
#include <algorithm>

using std::placeholders::_1;

// -----------------------
// Custom Point Type
// -----------------------
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, time, time)
)

// -----------------------
// Node Definition
// -----------------------
class RsConverterNode : public rclcpp::Node
{
public:
  RsConverterNode()
  : Node("rs_converter")
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/ouster/point_cloud");
    n_scan_      = declare_parameter<int>("n_scan", 128);   // Ouster default
    frame_id_    = declare_parameter<std::string>("frame_id", "OS2");

    sub_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RsConverterNode::lidarCallback, this, _1));

    pub_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "Ouster RTX converter started (n_scan=%d)", n_scan_);
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<PointXYZIRT>::Ptr pc_out(
      new pcl::PointCloud<PointXYZIRT>());

    pcl::fromROSMsg(*msg, *pc_in);
    pc_out->points.reserve(pc_in->points.size());

    // Typical Ouster vertical FOV (Isaac Sim)
    constexpr float VERT_MIN = -0.392699f; // -22.5 deg
    constexpr float VERT_MAX =  0.392699f; // +22.5 deg

    const float inv_range = 1.0f / static_cast<float>(pc_in->points.size());

    for (size_t i = 0; i < pc_in->points.size(); ++i)
    {
      const auto &p = pc_in->points[i];
      PointXYZIRT pt;

      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;

      // -----------------------
      // Ouster-style ring computation
      // -----------------------
      float vert_angle =
        std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y));

      float ratio = (vert_angle - VERT_MIN) / (VERT_MAX - VERT_MIN);
      int ring = static_cast<int>(ratio * n_scan_);

      ring = std::clamp(ring, 0, n_scan_ - 1);
      pt.ring = static_cast<uint16_t>(ring);

      // -----------------------
      // Normalized time
      // -----------------------
      pt.time = static_cast<float>(i) * inv_range;

      pc_out->points.push_back(pt);
    }

    pc_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*pc_out, out_msg);
    out_msg.header = msg->header;
    out_msg.header.frame_id = frame_id_;

    pub_pc_->publish(out_msg);
  }

  std::string lidar_topic_;
  std::string frame_id_;
  int n_scan_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
};

// -----------------------
// Main
// -----------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RsConverterNode>());
  rclcpp::shutdown();
  return 0;
}
