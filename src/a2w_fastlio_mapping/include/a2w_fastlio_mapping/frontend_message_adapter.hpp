#pragma once

#include <string>
#include <string_view>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_mapping
{

struct FrontendMessageResult
{
  bool success{false};
  std::string error{};
  a2w_fastlio_common::FrontendFrame frame{};
};

FrontendMessageResult makeFrontendFrame(
  const nav_msgs::msg::Odometry & odometry,
  const sensor_msgs::msg::PointCloud2 & body_cloud,
  std::string_view expected_odom_frame,
  std::string_view expected_body_frame);

}  // namespace a2w_fastlio_mapping
