#include "a2w_fastlio_map/keyframe_database.hpp"

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_map
{

a2w_fastlio_common::KeyFrame KeyFrameDatabase::clone(
  const a2w_fastlio_common::KeyFrame & keyframe)
{
  auto result = keyframe;
  result.body_cloud = a2w_fastlio_common::CloudPtr{
    new a2w_fastlio_common::Cloud{*keyframe.body_cloud}};
  return result;
}

bool KeyFrameDatabase::add(const a2w_fastlio_common::KeyFrame & keyframe)
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (keyframe.id != keyframes_.size() || keyframe.stamp_ns < 0 || !keyframe.body_cloud ||
    keyframe.body_cloud->empty() || !a2w_fastlio_common::isFinitePose(keyframe.odom_pose) ||
    !a2w_fastlio_common::isFinitePose(keyframe.optimized_pose))
  {
    return false;
  }
  keyframes_.push_back(clone(keyframe));
  return true;
}

std::optional<a2w_fastlio_common::KeyFrame> KeyFrameDatabase::get(
  const std::uint64_t id) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return id < keyframes_.size() ?
         std::optional<a2w_fastlio_common::KeyFrame>{clone(keyframes_[id])} : std::nullopt;
}

std::optional<std::pair<std::uint64_t, std::uint64_t>> KeyFrameDatabase::bounds() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return keyframes_.empty() ? std::nullopt :
         std::optional<std::pair<std::uint64_t, std::uint64_t>>{
    std::pair<std::uint64_t, std::uint64_t>{0U, keyframes_.size() - 1U}};
}

std::vector<a2w_fastlio_common::KeyFrame> KeyFrameDatabase::ordered() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  std::vector<a2w_fastlio_common::KeyFrame> result;
  result.reserve(keyframes_.size());
  for (const auto & keyframe : keyframes_) {
    result.push_back(clone(keyframe));
  }
  return result;
}

std::size_t KeyFrameDatabase::size() const noexcept
{
  std::lock_guard<std::mutex> lock{mutex_};
  return keyframes_.size();
}

}  // namespace a2w_fastlio_map
