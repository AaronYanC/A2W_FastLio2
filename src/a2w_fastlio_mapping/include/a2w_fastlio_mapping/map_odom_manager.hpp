#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_mapping
{

a2w_fastlio_common::Pose3d composePose(
  const a2w_fastlio_common::Pose3d & lhs,
  const a2w_fastlio_common::Pose3d & rhs);
a2w_fastlio_common::Pose3d inversePose(const a2w_fastlio_common::Pose3d & pose);

struct MapOdomCorrection
{
  std::uint64_t keyframe_id{0U};
  std::int64_t stamp_ns{0};
  a2w_fastlio_common::Pose3d map_camera_init{};

  std::int64_t ageNs(std::int64_t now_ns) const;
};

class MapOdomManager
{
public:
  MapOdomCorrection update(
    std::uint64_t keyframe_id, std::int64_t stamp_ns,
    const a2w_fastlio_common::Pose3d & map_body,
    const a2w_fastlio_common::Pose3d & camera_init_body);
  std::optional<MapOdomCorrection> latest() const;

private:
  mutable std::mutex mutex_;
  std::optional<MapOdomCorrection> latest_{};
};

}  // namespace a2w_fastlio_mapping
