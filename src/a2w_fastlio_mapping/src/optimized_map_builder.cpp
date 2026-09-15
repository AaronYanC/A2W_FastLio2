#include "a2w_fastlio_mapping/optimized_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_mapping
{

OptimizedMapBuilder::OptimizedMapBuilder(OptimizedMapConfig config) : config_{config}
{
  if (!std::isfinite(config_.full_voxel_leaf_m) || config_.full_voxel_leaf_m < 0.0 ||
    !std::isfinite(config_.preview_voxel_leaf_m) || config_.preview_voxel_leaf_m <= 0.0 ||
    config_.preview_max_points == 0U)
  {
    throw std::invalid_argument{"invalid optimized-map configuration"};
  }
}

a2w_fastlio_common::CloudPtr OptimizedMapBuilder::buildTransformed(
  const a2w_fastlio_common::KeyFrameProvider & provider,
  const OptimizedPoseSnapshot & poses) const
{
  if (poses.poses.empty()) {
    throw std::invalid_argument{"optimized pose snapshot is empty"};
  }
  auto output = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  for (std::size_t index = 0U; index < poses.poses.size(); ++index) {
    const auto & optimized = poses.poses[index];
    if (optimized.id != index || !a2w_fastlio_common::isFinitePose(optimized.pose)) {
      throw std::invalid_argument{"optimized poses must be finite and contiguous from zero"};
    }
    const auto keyframe = provider.get(optimized.id);
    if (!keyframe || !keyframe->body_cloud || keyframe->body_cloud->empty()) {
      throw std::invalid_argument{"optimized map keyframe is missing or empty"};
    }
    const auto rotation = optimized.pose.rotation.normalized();
    for (const auto & point : keyframe->body_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::invalid_argument{"optimized map keyframe contains a non-finite point"};
      }
      const Eigen::Vector3d xyz =
        rotation * point.getVector3fMap().cast<double>() + optimized.pose.translation;
      auto transformed = point;
      transformed.x = static_cast<float>(xyz.x());
      transformed.y = static_cast<float>(xyz.y());
      transformed.z = static_cast<float>(xyz.z());
      output->push_back(transformed);
    }
  }
  return output;
}

a2w_fastlio_common::CloudPtr OptimizedMapBuilder::voxelFilter(
  const a2w_fastlio_common::CloudPtr & input, const double leaf_m)
{
  if (leaf_m == 0.0) {
    return a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{*input}};
  }
  pcl::VoxelGrid<a2w_fastlio_common::PointT> filter;
  const auto leaf = static_cast<float>(leaf_m);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(input);
  auto output = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  filter.filter(*output);
  return output;
}

void OptimizedMapBuilder::sortAndCap(
  const a2w_fastlio_common::CloudPtr & cloud, const std::size_t max_points)
{
  std::sort(cloud->points.begin(), cloud->points.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.x != rhs.x) {return lhs.x < rhs.x;}
      if (lhs.y != rhs.y) {return lhs.y < rhs.y;}
      if (lhs.z != rhs.z) {return lhs.z < rhs.z;}
      return lhs.intensity < rhs.intensity;
    });
  if (cloud->size() > max_points) {
    cloud->resize(max_points);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
}

a2w_fastlio_common::CloudPtr OptimizedMapBuilder::buildFull(
  const a2w_fastlio_common::KeyFrameProvider & provider,
  const OptimizedPoseSnapshot & poses) const
{
  auto output = voxelFilter(buildTransformed(provider, poses), config_.full_voxel_leaf_m);
  sortAndCap(output, std::numeric_limits<std::size_t>::max());
  return output;
}

a2w_fastlio_common::CloudPtr OptimizedMapBuilder::buildPreview(
  const a2w_fastlio_common::KeyFrameProvider & provider,
  const OptimizedPoseSnapshot & poses) const
{
  auto output = voxelFilter(buildTransformed(provider, poses), config_.preview_voxel_leaf_m);
  sortAndCap(output, config_.preview_max_points);
  return output;
}

}  // namespace a2w_fastlio_mapping
