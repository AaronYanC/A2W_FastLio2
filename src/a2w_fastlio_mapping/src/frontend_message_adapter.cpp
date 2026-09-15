#include "a2w_fastlio_mapping/frontend_message_adapter.hpp"

#include <cmath>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>

namespace a2w_fastlio_mapping
{

namespace
{

std::int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

FrontendMessageResult makeFrontendFrame(
  const nav_msgs::msg::Odometry & odometry,
  const sensor_msgs::msg::PointCloud2 & body_cloud,
  const std::string_view expected_odom_frame,
  const std::string_view expected_body_frame)
{
  FrontendMessageResult result;
  const auto odometry_stamp_ns = stampToNanoseconds(odometry.header.stamp);
  if (odometry_stamp_ns != stampToNanoseconds(body_cloud.header.stamp)) {
    result.error = "odometry and body cloud timestamps differ";
    return result;
  }
  if (!expected_odom_frame.empty() && odometry.header.frame_id != expected_odom_frame) {
    result.error = "unexpected odometry parent frame";
    return result;
  }
  if (!expected_body_frame.empty() && odometry.child_frame_id != expected_body_frame) {
    result.error = "unexpected odometry child frame";
    return result;
  }
  if (!expected_body_frame.empty() && body_cloud.header.frame_id != expected_body_frame) {
    result.error = "unexpected body cloud frame";
    return result;
  }

  const auto & position = odometry.pose.pose.position;
  const auto & orientation = odometry.pose.pose.orientation;
  const bool pose_is_finite =
    std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
    std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
    std::isfinite(orientation.z) && std::isfinite(orientation.w);
  const double quaternion_squared_norm =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!pose_is_finite || quaternion_squared_norm <= 1.0e-12) {
    result.error = "odometry pose is invalid";
    return result;
  }

  result.frame.stamp_ns = odometry_stamp_ns;
  result.frame.odom_pose.translation = Eigen::Vector3d{
    odometry.pose.pose.position.x,
    odometry.pose.pose.position.y,
    odometry.pose.pose.position.z};
  result.frame.odom_pose.rotation = Eigen::Quaterniond{
    odometry.pose.pose.orientation.w,
    odometry.pose.pose.orientation.x,
    odometry.pose.pose.orientation.y,
    odometry.pose.pose.orientation.z};
  result.frame.odom_pose.rotation.normalize();

  auto converted_cloud = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  pcl::fromROSMsg(body_cloud, *converted_cloud);
  if (converted_cloud->empty()) {
    result.error = "body cloud is empty";
    return result;
  }
  result.frame.body_cloud = converted_cloud;
  result.success = true;
  return result;
}

}  // namespace a2w_fastlio_mapping
