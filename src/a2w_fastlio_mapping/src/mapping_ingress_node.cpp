#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "a2w_fastlio_mapping/frontend_message_adapter.hpp"
#include "a2w_fastlio_mapping/keyframe_manager.hpp"

namespace a2w_fastlio_mapping
{

namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

}  // namespace

class MappingIngressNode : public rclcpp::Node
{
public:
  MappingIngressNode()
  : Node("mapping_ingress_node")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    const auto body_cloud_topic =
      declare_parameter<std::string>("body_cloud_topic", "/cloud_registered_body");
    const auto keyframe_odom_topic =
      declare_parameter<std::string>("keyframe_odom_topic", "/mapping/keyframe_odom");
    const auto keyframe_cloud_topic =
      declare_parameter<std::string>("keyframe_cloud_topic", "/mapping/keyframe_cloud");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "body");

    const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 20);
    if (sync_queue_size <= 0) {
      throw std::invalid_argument("sync_queue_size must be greater than zero");
    }

    KeyframeConfig keyframe_config;
    keyframe_config.translation_threshold_m =
      declare_parameter<double>("translation_threshold_m", 1.0);
    keyframe_config.rotation_threshold_rad =
      declare_parameter<double>("rotation_threshold_deg", 10.0) * kDegreesToRadians;
    keyframe_config.max_interval_s = declare_parameter<double>("max_interval_s", 2.0);
    keyframe_manager_ = std::make_unique<KeyframeManager>(keyframe_config);

    const auto qos = rclcpp::QoS{rclcpp::KeepLast{20}}.reliable();
    keyframe_odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(keyframe_odom_topic, qos);
    keyframe_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(keyframe_cloud_topic, qos);

    odometry_subscriber_.subscribe(this, odom_topic, rmw_qos_profile_default);
    body_cloud_subscriber_.subscribe(this, body_cloud_topic, rmw_qos_profile_default);
    synchronizer_ = std::make_shared<Synchronizer>(
      odometry_subscriber_, body_cloud_subscriber_,
      static_cast<std::uint32_t>(sync_queue_size));
    synchronizer_->registerCallback(
      std::bind(&MappingIngressNode::synchronizedCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Stage 1 ingress: %s + %s -> %s + %s",
      odom_topic.c_str(), body_cloud_topic.c_str(),
      keyframe_odom_topic.c_str(), keyframe_cloud_topic.c_str());
  }

private:
  using Synchronizer = message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  void synchronizedCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & body_cloud)
  {
    const auto adapted = makeFrontendFrame(
      *odometry, *body_cloud, odom_frame_, tracking_frame_);
    if (!adapted.success) {
      ++adapter_rejection_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected synchronized FAST-LIO pair (%zu total): %s",
        adapter_rejection_count_, adapted.error.c_str());
      return;
    }

    const auto decision = keyframe_manager_->consider(adapted.frame);
    if (!decision.keyframe) {
      ++policy_rejection_count_;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Keyframe policy rejected %zu synchronized frames",
        policy_rejection_count_);
      return;
    }

    keyframe_odom_publisher_->publish(*odometry);
    keyframe_cloud_publisher_->publish(*body_cloud);
    RCLCPP_INFO(
      get_logger(), "Accepted keyframe %zu at %ld ns",
      keyframe_manager_->size() - 1,
      static_cast<long>(adapted.frame.stamp_ns));
  }

  std::string odom_frame_;
  std::string tracking_frame_;
  std::unique_ptr<KeyframeManager> keyframe_manager_;
  std::size_t adapter_rejection_count_{0};
  std::size_t policy_rejection_count_{0};

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> body_cloud_subscriber_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr keyframe_odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_publisher_;
};

}  // namespace a2w_fastlio_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<a2w_fastlio_mapping::MappingIngressNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mapping_ingress_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
