#include "a2w_fastlio_mapping/map_odom_manager.hpp"

#include <stdexcept>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_mapping
{

a2w_fastlio_common::Pose3d composePose(
  const a2w_fastlio_common::Pose3d & lhs,
  const a2w_fastlio_common::Pose3d & rhs)
{
  if (!a2w_fastlio_common::isFinitePose(lhs) || !a2w_fastlio_common::isFinitePose(rhs)) {
    throw std::invalid_argument{"cannot compose a non-finite pose"};
  }
  a2w_fastlio_common::Pose3d result;
  result.rotation = (lhs.rotation.normalized() * rhs.rotation.normalized()).normalized();
  result.translation = lhs.rotation.normalized() * rhs.translation + lhs.translation;
  return result;
}

a2w_fastlio_common::Pose3d inversePose(const a2w_fastlio_common::Pose3d & pose)
{
  if (!a2w_fastlio_common::isFinitePose(pose)) {
    throw std::invalid_argument{"cannot invert a non-finite pose"};
  }
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation.normalized().inverse();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

std::int64_t MapOdomCorrection::ageNs(const std::int64_t now_ns) const
{
  if (now_ns < stamp_ns) {
    throw std::invalid_argument{"age reference precedes correction timestamp"};
  }
  return now_ns - stamp_ns;
}

MapOdomCorrection MapOdomManager::update(
  const std::uint64_t keyframe_id, const std::int64_t stamp_ns,
  const a2w_fastlio_common::Pose3d & map_body,
  const a2w_fastlio_common::Pose3d & camera_init_body)
{
  if (stamp_ns < 0 || !a2w_fastlio_common::isFinitePose(map_body) ||
    !a2w_fastlio_common::isFinitePose(camera_init_body))
  {
    throw std::invalid_argument{"invalid map-to-odom correction input"};
  }

  std::lock_guard<std::mutex> lock{mutex_};
  if (latest_ && stamp_ns <= latest_->stamp_ns) {
    throw std::invalid_argument{"correction timestamp must increase"};
  }
  if (latest_ && keyframe_id <= latest_->keyframe_id) {
    throw std::invalid_argument{"correction keyframe ID must increase"};
  }

  MapOdomCorrection next;
  next.keyframe_id = keyframe_id;
  next.stamp_ns = stamp_ns;
  next.map_camera_init = composePose(map_body, inversePose(camera_init_body));
  latest_ = next;
  return next;
}

std::optional<MapOdomCorrection> MapOdomManager::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return latest_;
}

}  // namespace a2w_fastlio_mapping
