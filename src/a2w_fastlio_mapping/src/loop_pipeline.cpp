#include "a2w_fastlio_mapping/loop_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_mapping
{
namespace
{

using a2w_fastlio_common::Pose3d;

Pose3d compose(const Pose3d & lhs, const Pose3d & rhs)
{
  Pose3d result;
  result.rotation = (lhs.rotation * rhs.rotation).normalized();
  result.translation = lhs.rotation * rhs.translation + lhs.translation;
  return result;
}

Pose3d between(const Pose3d & from, const Pose3d & to)
{
  Pose3d result;
  result.rotation = (from.rotation.inverse() * to.rotation).normalized();
  result.translation = from.rotation.inverse() * (to.translation - from.translation);
  return result;
}

Pose3d yawHint(const double yaw_rad)
{
  Pose3d result;
  result.rotation = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
  return result;
}

}  // namespace

class LoopPipeline::StoredKeyFrameProvider final :
  public a2w_fastlio_common::KeyFrameProvider
{
public:
  explicit StoredKeyFrameProvider(
    const std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> & keyframes)
  : keyframes_{keyframes} {}

  std::optional<a2w_fastlio_common::KeyFrame> get(std::uint64_t id) const override
  {
    const auto iterator = keyframes_.find(id);
    return iterator == keyframes_.end() ? std::nullopt :
           std::optional<a2w_fastlio_common::KeyFrame>{iterator->second};
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    if (keyframes_.empty()) {
      return std::nullopt;
    }
    return std::pair<std::uint64_t, std::uint64_t>{
      keyframes_.begin()->first, keyframes_.rbegin()->first};
  }

private:
  const std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> & keyframes_;
};

LoopPipeline::LoopPipeline(
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition,
  std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
  std::shared_ptr<PoseGraphOptimizer> graph,
  a2w_fastlio_common::LocalMapBuilder local_map_builder,
  LoopPipelineConfig config)
: place_recognition_{std::move(place_recognition)}, descriptor_index_{std::move(descriptor_index)},
  registration_{std::move(registration)}, graph_{std::move(graph)},
  local_map_builder_{std::move(local_map_builder)}, config_{config}
{
  if (!place_recognition_ || !descriptor_index_ || !registration_ || !graph_ ||
    config_.top_k == 0U || config_.minimum_keyframes_between_accepted_loops == 0U ||
    config_.queue_capacity == 0U || !std::isfinite(config_.maximum_descriptor_distance) ||
    config_.maximum_descriptor_distance <= 0.0)
  {
    throw std::invalid_argument{"invalid loop-pipeline dependencies or configuration"};
  }
}

std::vector<LoopPipelineEvent> LoopPipeline::process(
  const a2w_fastlio_common::KeyFrame & current)
{
  std::vector<LoopPipelineEvent> events;
  if (current.id != keyframes_.size()) {
    const auto graph_state = graph_->addKeyFrame(current.id, current.odom_pose);
    events.push_back({
      current.id, 0U, 0U, false, graph_state.reason, {}, graph_state.factor_count});
    return events;
  }

  a2w_fastlio_common::ScanDescriptor descriptor{1U, 1U, {0.0F}};
  try {
    descriptor = place_recognition_->describe(current.body_cloud);
  } catch (const std::exception & error) {
    events.push_back({
      current.id, 0U, 0U, false, std::string{"descriptor_failed:"} + error.what(), {},
      keyframes_.size()});
    return events;
  }

  const auto graph_add = graph_->addKeyFrame(current.id, current.odom_pose);
  if (!graph_add.accepted) {
    events.push_back({
      current.id, 0U, 0U, false, graph_add.reason, {}, graph_add.factor_count});
    return events;
  }
  keyframes_.emplace(current.id, current);

  const bool cooldown = last_accepted_loop_id_ &&
    current.id < *last_accepted_loop_id_ + config_.minimum_keyframes_between_accepted_loops;
  std::vector<a2w_fastlio_common::LoopCandidate> candidates;
  if (!cooldown && current.id > 0U) {
    a2w_fastlio_common::CandidateFilter filter;
    filter.max_inclusive_id = current.id - 1U;
    filter.exclude_recent = config_.exclude_recent;
    candidates = descriptor_index_->queryTopK(descriptor, config_.top_k, filter);
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [this](const auto & candidate) {
          return candidate.distance > config_.maximum_descriptor_distance;
        }),
      candidates.end());
  }

  if (cooldown) {
    events.push_back({current.id, 0U, 0U, false, "loop_policy_cooldown"});
  } else if (candidates.empty()) {
    events.push_back({current.id, 0U, 0U, false, "no_loop_candidates"});
  } else {
    const StoredKeyFrameProvider provider{keyframes_};
    const auto source = local_map_builder_.build(
      current.id, config_.local_map_before, config_.local_map_after, provider);
    a2w_fastlio_common::ValidationContext validation_context;
    validation_context.best_descriptor_distance = candidates.front().distance;
    if (candidates.size() > 1U) {
      validation_context.second_descriptor_distance = candidates[1].distance;
    }

    for (const auto & candidate : candidates) {
      LoopPipelineEvent event;
      event.current_id = current.id;
      event.candidate_id = candidate.keyframe_id;
      event.candidate_rank = candidate.rank;
      try {
        const auto target = local_map_builder_.build(
          candidate.keyframe_id, config_.local_map_before, config_.local_map_after, provider);
        const auto registration = registration_->run(
          source.cloud, target.cloud, yawHint(candidate.yaw_hint_rad), validation_context);
        event.registration = registration.fine;
        event.reason = registration.validation.reason;
        if (registration.success) {
          const auto corrected_current = compose(registration.final_transform, current.optimized_pose);
          LoopConstraint constraint;
          constraint.from_id = current.id;
          constraint.to_id = candidate.keyframe_id;
          constraint.relative_pose = between(
            corrected_current, keyframes_.at(candidate.keyframe_id).optimized_pose);
          constraint.accepted = true;
          const auto loop_add = graph_->addLoopConstraint(constraint);
          event.accepted = loop_add.accepted;
          event.reason = loop_add.accepted ? "loop_accepted" : loop_add.reason;
          events.push_back(event);
          if (loop_add.accepted) {
            last_accepted_loop_id_ = current.id;
            break;
          }
          continue;
        }
      } catch (const std::exception & error) {
        event.reason = std::string{"candidate_processing_failed:"} + error.what();
      }
      events.push_back(event);
    }
  }

  descriptor_index_->add(current.id, descriptor);
  const auto update = graph_->update();
  for (auto & event : events) {
    event.graph_factor_count = update.factor_count;
  }
  if (update.accepted) {
    for (const auto & optimized : graph_->optimizedPoses().poses) {
      keyframes_.at(optimized.id).optimized_pose = optimized.pose;
    }
  } else if (!events.empty() && events.back().reason.empty()) {
    events.back().reason = update.reason;
  }
  return events;
}

bool LoopPipeline::enqueue(a2w_fastlio_common::KeyFrame keyframe)
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  if (queue_.size() >= config_.queue_capacity) {
    return false;
  }
  queue_.push_back(std::move(keyframe));
  return true;
}

std::optional<std::vector<LoopPipelineEvent>> LoopPipeline::processNext()
{
  a2w_fastlio_common::KeyFrame keyframe;
  {
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty()) {
      return std::nullopt;
    }
    keyframe = std::move(queue_.front());
    queue_.pop_front();
  }
  return process(keyframe);
}

std::size_t LoopPipeline::pending() const noexcept
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  return queue_.size();
}

}  // namespace a2w_fastlio_mapping
