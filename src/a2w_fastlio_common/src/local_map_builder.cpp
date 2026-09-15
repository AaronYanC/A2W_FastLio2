#include "a2w_fastlio_common/local_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_common
{
namespace
{

bool finitePoint(const PointT & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::uint64_t subtractClamped(std::uint64_t value, std::size_t amount)
{
  const auto delta = static_cast<std::uint64_t>(amount);
  return value < delta ? 0U : value - delta;
}

std::uint64_t addClamped(std::uint64_t value, std::size_t amount)
{
  const auto delta = static_cast<std::uint64_t>(amount);
  return value > std::numeric_limits<std::uint64_t>::max() - delta ?
    std::numeric_limits<std::uint64_t>::max() : value + delta;
}

}  // namespace

LocalMapBuilder::LocalMapBuilder(LocalMapConfig config)
: config_(config)
{
  if (!std::isfinite(config_.voxel_leaf_m) || config_.voxel_leaf_m < 0.0 ||
    config_.max_points == 0U)
  {
    throw std::invalid_argument("invalid local map configuration");
  }
}

LocalMapResult LocalMapBuilder::build(
  std::uint64_t center_id, std::size_t before, std::size_t after,
  const KeyFrameProvider & provider) const
{
  const auto provider_bounds = provider.bounds();
  if (!provider_bounds) {
    throw std::runtime_error("cannot build a local map from an empty keyframe provider");
  }
  if (center_id < provider_bounds->first || center_id > provider_bounds->second) {
    throw std::runtime_error("local map center keyframe is outside provider bounds");
  }

  const auto first = std::max(provider_bounds->first, subtractClamped(center_id, before));
  const auto last = std::min(provider_bounds->second, addClamped(center_id, after));
  LocalMapResult result;

  for (std::uint64_t id = first;; ++id) {
    const auto frame = provider.get(id);
    if (!frame) {
      throw std::runtime_error("local map neighborhood contains a missing keyframe");
    }
    if (!frame->body_cloud || frame->body_cloud->empty()) {
      throw std::runtime_error("local map keyframe cloud is empty");
    }
    if (!isFinitePose(frame->optimized_pose)) {
      throw std::runtime_error("local map keyframe pose is invalid");
    }

    const auto rotation = frame->optimized_pose.rotation.normalized();
    for (const auto & input : frame->body_cloud->points) {
      if (!finitePoint(input)) {
        throw std::runtime_error("local map keyframe cloud contains a non-finite point");
      }
      const Eigen::Vector3d transformed =
        rotation * input.getVector3fMap().cast<double>() + frame->optimized_pose.translation;
      auto output = input;
      output.x = static_cast<float>(transformed.x());
      output.y = static_cast<float>(transformed.y());
      output.z = static_cast<float>(transformed.z());
      result.cloud->push_back(output);
    }
    result.included_ids.push_back(id);
    if (id == last) {
      break;
    }
  }

  if (config_.voxel_leaf_m > 0.0) {
    pcl::VoxelGrid<PointT> filter;
    filter.setLeafSize(
      static_cast<float>(config_.voxel_leaf_m),
      static_cast<float>(config_.voxel_leaf_m),
      static_cast<float>(config_.voxel_leaf_m));
    filter.setInputCloud(result.cloud);
    auto filtered = CloudPtr{new Cloud{}};
    filter.filter(*filtered);
    result.cloud = filtered;
  }

  std::sort(result.cloud->points.begin(), result.cloud->points.end(), [](const PointT & lhs,
      const PointT & rhs) {
      if (lhs.x != rhs.x) {return lhs.x < rhs.x;}
      if (lhs.y != rhs.y) {return lhs.y < rhs.y;}
      if (lhs.z != rhs.z) {return lhs.z < rhs.z;}
      return lhs.intensity < rhs.intensity;
    });
  if (result.cloud->size() > config_.max_points) {
    result.cloud->resize(config_.max_points);
  }
  result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
  result.cloud->height = 1U;
  result.cloud->is_dense = true;
  return result;
}

}  // namespace a2w_fastlio_common
