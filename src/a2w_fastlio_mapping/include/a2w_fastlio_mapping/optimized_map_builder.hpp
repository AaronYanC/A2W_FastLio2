#pragma once

#include <cstddef>

#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{

struct OptimizedMapConfig
{
  double full_voxel_leaf_m{0.0};
  double preview_voxel_leaf_m{0.5};
  std::size_t preview_max_points{100000U};
};

class OptimizedMapBuilder
{
public:
  explicit OptimizedMapBuilder(OptimizedMapConfig config);

  a2w_fastlio_common::CloudPtr buildFull(
    const a2w_fastlio_common::KeyFrameProvider & provider,
    const OptimizedPoseSnapshot & poses) const;
  a2w_fastlio_common::CloudPtr buildPreview(
    const a2w_fastlio_common::KeyFrameProvider & provider,
    const OptimizedPoseSnapshot & poses) const;

private:
  a2w_fastlio_common::CloudPtr buildTransformed(
    const a2w_fastlio_common::KeyFrameProvider & provider,
    const OptimizedPoseSnapshot & poses) const;
  static a2w_fastlio_common::CloudPtr voxelFilter(
    const a2w_fastlio_common::CloudPtr & input, double leaf_m);
  static void sortAndCap(
    const a2w_fastlio_common::CloudPtr & cloud, std::size_t max_points);

  OptimizedMapConfig config_;
};

}  // namespace a2w_fastlio_mapping
