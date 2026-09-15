#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "a2w_fastlio_common/point_types.hpp"

namespace a2w_fastlio_common
{

struct Pose3d
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

struct FrontendFrame
{
  std::int64_t stamp_ns{0};
  Pose3d odom_pose{};
  CloudConstPtr body_cloud{new Cloud{}};
};

struct KeyFrame
{
  std::uint64_t id{0};
  std::int64_t stamp_ns{0};
  Pose3d odom_pose{};
  Pose3d optimized_pose{};
  CloudPtr body_cloud{new Cloud{}};
  std::vector<float> scan_context_descriptor{};
};

}  // namespace a2w_fastlio_common
