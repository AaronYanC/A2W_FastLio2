#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "a2w_fastlio_map/map_manager.hpp"

namespace a2w_fastlio_localization
{

struct LocalMapSelectorConfig
{
  double radius_m{30.0};
  std::size_t max_neighbors{20U};
  std::size_t minimum_neighbors{3U};
};

struct SelectionResult
{
  bool success{false};
  std::string reason;
  std::uint64_t nearest_keyframe_id{0U};
  a2w_fastlio_common::Pose3d initial_map_body{};
  std::vector<std::uint64_t> keyframe_ids;
};

class LocalMapSelector
{
public:
  LocalMapSelector(a2w_fastlio_map::MapSnapshot snapshot, LocalMapSelectorConfig config);
  SelectionResult select(const a2w_fastlio_common::Pose3d & map_body) const;

private:
  a2w_fastlio_map::MapSnapshot snapshot_;
  LocalMapSelectorConfig config_;
};

}  // namespace a2w_fastlio_localization
