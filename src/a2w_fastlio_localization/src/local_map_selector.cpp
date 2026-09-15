#include "a2w_fastlio_localization/local_map_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{

LocalMapSelector::LocalMapSelector(
  a2w_fastlio_map::MapSnapshot snapshot, LocalMapSelectorConfig config)
: snapshot_{std::move(snapshot)}, config_{config}
{
  if (!std::isfinite(config_.radius_m) || config_.radius_m <= 0.0 ||
    config_.max_neighbors == 0U || config_.minimum_neighbors == 0U ||
    config_.minimum_neighbors > config_.max_neighbors || snapshot_.keyframes.empty())
  {
    throw std::invalid_argument{"invalid local-map selector configuration or snapshot"};
  }
}

SelectionResult LocalMapSelector::select(const a2w_fastlio_common::Pose3d & map_body) const
{
  SelectionResult result;
  if (!a2w_fastlio_common::isFinitePose(map_body)) {
    result.reason = "global_pose_not_finite";
    return result;
  }
  std::vector<std::tuple<double, std::uint64_t, const a2w_fastlio_common::KeyFrame *>> nearby;
  for (const auto & frame : snapshot_.keyframes) {
    const auto distance = (frame.optimized_pose.translation - map_body.translation).norm();
    if (distance <= config_.radius_m) {
      nearby.emplace_back(distance, frame.id, &frame);
    }
  }
  std::sort(nearby.begin(), nearby.end(), [](const auto & lhs, const auto & rhs) {
      return std::tie(std::get<0>(lhs), std::get<1>(lhs)) <
             std::tie(std::get<0>(rhs), std::get<1>(rhs));
    });
  if (nearby.size() < config_.minimum_neighbors) {
    result.reason = "no_nearby_keyframes";
    return result;
  }
  nearby.resize(std::min(nearby.size(), config_.max_neighbors));
  result.nearest_keyframe_id = std::get<1>(nearby.front());
  result.initial_map_body = map_body;
  result.keyframe_ids.reserve(nearby.size());
  for (const auto & candidate : nearby) {
    result.keyframe_ids.push_back(std::get<1>(candidate));
  }
  result.success = true;
  result.reason = "ok";
  return result;
}

}  // namespace a2w_fastlio_localization
