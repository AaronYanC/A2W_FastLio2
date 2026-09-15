#include "a2w_fastlio_map/map_manager.hpp"

#include <stdexcept>

namespace a2w_fastlio_map
{

MapManager::MapManager(const MapAccess access) : access_{access} {}

void MapManager::requireWritable() const
{
  if (access_ == MapAccess::kReadOnly) {
    throw std::logic_error{"map manager is read-only"};
  }
}

bool MapManager::append(
  const a2w_fastlio_common::KeyFrame & keyframe,
  const a2w_fastlio_common::ScanDescriptor & descriptor)
{
  requireWritable();
  std::lock_guard<std::mutex> lock{mutex_};
  if (keyframe.id != keyframes_.size() || keyframe.id != descriptors_.size()) {
    return false;
  }
  const auto existing_descriptors = descriptors_.records();
  if (!existing_descriptors.empty() &&
    (descriptor.rings() != existing_descriptors.front().descriptor.rings() ||
    descriptor.sectors() != existing_descriptors.front().descriptor.sectors()))
  {
    return false;
  }
  if (!keyframes_.add(keyframe)) {
    return false;
  }
  descriptors_.add(keyframe.id, descriptor);
  return true;
}

void MapManager::setGlobalMap(const a2w_fastlio_common::CloudConstPtr & global_map)
{
  requireWritable();
  if (!global_map || global_map->empty()) {
    throw std::invalid_argument{"global map must not be empty"};
  }
  std::lock_guard<std::mutex> lock{mutex_};
  global_map_ = a2w_fastlio_common::CloudPtr{
    new a2w_fastlio_common::Cloud{*global_map}};
}

MapSnapshot MapManager::snapshot() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  MapSnapshot result;
  result.keyframes = keyframes_.ordered();
  result.descriptors = descriptors_.records();
  if (result.keyframes.size() != result.descriptors.size()) {
    throw std::runtime_error{"map database counts are inconsistent"};
  }
  result.global_map = a2w_fastlio_common::CloudPtr{
    new a2w_fastlio_common::Cloud{*global_map_}};
  return result;
}

MapAccess MapManager::access() const noexcept
{
  return access_;
}

}  // namespace a2w_fastlio_map
