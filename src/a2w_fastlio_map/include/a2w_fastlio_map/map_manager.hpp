#pragma once

#include <mutex>
#include <vector>

#include "a2w_fastlio_map/descriptor_database.hpp"
#include "a2w_fastlio_map/keyframe_database.hpp"

namespace a2w_fastlio_map
{

enum class MapAccess
{
  kReadWrite,
  kReadOnly,
};

struct MapSnapshot
{
  std::vector<a2w_fastlio_common::KeyFrame> keyframes;
  std::vector<DescriptorRecord> descriptors;
  a2w_fastlio_common::CloudPtr global_map{new a2w_fastlio_common::Cloud{}};
};

class MapManager
{
public:
  explicit MapManager(MapAccess access = MapAccess::kReadWrite);

  bool append(
    const a2w_fastlio_common::KeyFrame & keyframe,
    const a2w_fastlio_common::ScanDescriptor & descriptor);
  void setGlobalMap(const a2w_fastlio_common::CloudConstPtr & global_map);
  MapSnapshot snapshot() const;
  MapAccess access() const noexcept;

private:
  void requireWritable() const;

  MapAccess access_;
  mutable std::mutex mutex_;
  KeyFrameDatabase keyframes_;
  DescriptorDatabase descriptors_;
  a2w_fastlio_common::CloudPtr global_map_{new a2w_fastlio_common::Cloud{}};
};

}  // namespace a2w_fastlio_map
